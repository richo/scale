#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::main;
use hal::clock::CpuClock;


use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_time::Timer;

use trouble_host::prelude::*;

use hx711;

use embedded_hal::delay::DelayNs;

// use esp_wifi::{ble::controller::BleConnector, initialize, EspWifiInitFor};
use esp_radio::ble::controller::{BleConnector};

use hal::gpio::{Input, InputConfig, Io, Level, Output, OutputConfig, Pull};
use hal::time;

use hal::rng::{Trng, TrngSource};

use hal::delay::Delay;
use hal::rtc_cntl::Rtc;
use hal::timer::timg::TimerGroup;
use core::sync::atomic::{AtomicBool, Ordering};
use static_cell::StaticCell;



use scale::{Scale, Buffer};

static SHOULD_TARE: AtomicBool = AtomicBool::new(false);
static DISABLE_DRIVERS: AtomicBool = AtomicBool::new(false);
static ENABLE_DRIVERS: AtomicBool = AtomicBool::new(false);

const UPDATE_INTERVAL: u64 = 200;
const TARE_DEBOUNCE: u64 = 500;

// Calibrated with the drip tray in situ
// This isn't currently true but should be again soon
const LEFT_FACTOR: f32 = (901813.0) / 176.9;
const RIGHT_FACTOR: f32 = (950360.0) / 176.9;

esp_bootloader_esp_idf::esp_app_desc!();


/// Max number of connections
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

// GATT Server definition
#[gatt_server]
struct Server {
    scale_service: ScaleService
}

const DECENT_SCALE: BluetoothUuid16 = BluetoothUuid16::new(0xFFF0);

/// Battery service
#[gatt_service(uuid = DECENT_SCALE)]
struct ScaleService {
    #[characteristic(uuid = "000036F5-0000-1000-8000-00805F9B34FB", write)]
    command: u8,
    #[characteristic(uuid = "0000FFF4-0000-1000-8000-00805F9B34FB", read, notify)]
    weight: i16,
}


#[esp_rtos::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger_from_env();

    let now = || time::Instant::now().duration_since_epoch().as_millis();

    let config = hal::Config::default().with_cpu_clock(CpuClock::max());
    let mut peripherals = hal::init(config);

    esp_alloc::heap_allocator!(#[hal::ram(reclaimed)] size: 98768);

    let rtc = Rtc::new(peripherals.LPWR);

    log::info!("Logger is setup");
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");

    let mut io = Io::new(peripherals.IO_MUX);
    let mut delay = Delay::new();

    let floating_config = InputConfig::default().with_pull(Pull::None);
    let output_config = OutputConfig::default();

    let dout = Input::new(peripherals.GPIO16, floating_config);
    let pd_sck = Output::new(peripherals.GPIO4, Level::Low, output_config);
    let mut hx = hx711::Hx711::new(delay, dout, pd_sck).unwrap();
    let mut left = Scale::new(&mut hx);

    let dout = Input::new(peripherals.GPIO18, floating_config);
    let pd_sck = Output::new(peripherals.GPIO5, Level::Low, output_config);
    let mut hx = hx711::Hx711::new(delay, dout, pd_sck).unwrap();
    let mut right = Scale::new(&mut hx);

    let pullup_config = InputConfig::default().with_pull(Pull::Up);
    let tare = Input::new(peripherals.GPIO21, pullup_config);

            let _ = left.enable();
            let _ = right.enable();
            let _ = left.tare();
            let _ = right.tare();


    let connector = BleConnector::new(&radio_init, peripherals.BT, Default::default()).expect("Couldn't build connector");
    let controller: ExternalController<_, 1> = ExternalController::new(connector);
    ble_scale_run(controller).await;
}

pub async fn ble_scale_run<C>(controller: C)
where
    C: Controller,
{
    // TODO(richo) Figure this out
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    log::info!("Our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    log::info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

	let _ = join(ble_task(runner), async {
		loop {
			match advertise("Decent Scale 2", &mut peripheral, &server).await {
				Ok(conn) => {
					// set up tasks when the connection is established to a central, so they don't
					// run when no one is connected.
					let a = gatt_events_task(&server, &conn);
					let b = custom_task(&server, &conn, &stack);
					// run until any task ends (usually because the connection has been closed),
					// then return to advertising state.
					select(a, b).await;
				}
				Err(e) => {
					panic!("[adv] error: {:?}", e);
				}
			}
		}
	})
	.await;
}



async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let weight = server.scale_service.weight;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Read(event) => {
                        log::info!("[gatt] Read Event to something");
                        if event.handle() == weight.handle {
                            let value = server.get(&weight);
                            log::info!("[gatt] Read Event to Level Characteristic: {:?}", value);
                        }
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == weight.handle {
                            log::info!(
                                "[gatt] Write Event to Level Characteristic: {:?}",
                                event.data()
                            );
                        }
                    }
                    _ => {}
                };
                // This step is also performed at drop(), but writing it explicitly is necessary
                // in order to ensure reply is sent.
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => log::warn!("[gatt] error sending response: {:?}", e),
                };
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    log::info!("[gatt] disconnected: {:?}", reason);
    Ok(())
}



fn parse_data(data: &[u8]) {
	log::info!("RECEIVED: data {:x?}", data);
	match data {
		// CMD TARE
		[0x03, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0C] => {
			SHOULD_TARE.store(true, Ordering::Relaxed);
			log::info!("TARE: Triggering tare");
		},

		// TIMER ZERO
		[0x03, 0x0b, 0x02, 0x00, 0x00, 0x00, 0x0a] => {
			SHOULD_TARE.store(true, Ordering::Relaxed);
			log::info!("TIMERZERO: Triggering tare because of timer");
		}
		// TIMER STOP
		[0x03, 0x0b, 0x02, 0x00, 0x00, 0x00, 0x08] => {
			log::info!("TIMERSTOP: Got a timer stop");
		}

		//TIMERSTART
		[0x03, 0x0B, 0x03, 0x00, 0x00, 0x00, 0x0B] => {
			log::info!("TIMERSTART:");
		}

		// LED ON
		[0x03, 0x0A, 0x01, 0x01, 0x00, 0x00, 0x09] => {
			ENABLE_DRIVERS.store(true, Ordering::Relaxed);
			SHOULD_TARE.store(true, Ordering::Relaxed);
			log::info!("LEDON: enabling hx711's and taring.");
		}
		// LED OFF
		[0x03, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x09] => {
			DISABLE_DRIVERS.store(true, Ordering::Relaxed);
			log::info!("LEDOFF: disabling hx711's.");
		}
		_ => {},
	}
}
    // let threshold = 500.0;

    // loop {
    //     match srv.do_work() {
    //         Ok(res) => {
    //             // log::info!("do_work: {:?}", res);
    //             if let WorkResult::GotDisconnected = res {
    //                 log::info!("We're diconnected");
    //             }
    //         }
    //         Err(err) => {
    //             log::info!("do_work: {:?}", err);
    //         }
    //     }

    //     let mut notification = None;
    //     if ENABLE_DRIVERS.load(Ordering::Relaxed) {
    //         log::info!("enabling hx711's");
    //         let _ = left.enable();
    //         let _ = right.enable();
    //         ENABLE_DRIVERS.store(false, Ordering::Relaxed);
    //         // Give them a sec to wake up before we tare
    //         delay.delay_ns(100_000u32);
    //     }

    //     if DISABLE_DRIVERS.load(Ordering::Relaxed) {
    //         log::info!("disabling hx711's");
    //         let _ = left.disable();
    //         let _ = right.disable();
    //         DISABLE_DRIVERS.store(false, Ordering::Relaxed);
    //     }

    //     if SHOULD_TARE.load(Ordering::Relaxed) {
    //         log::info!("Taring");
    //         left.tare();
    //         right.tare();
    //         // Fill the values buffer back up with zeros;
    //         values.zero();
    //         SHOULD_TARE.store(false, Ordering::Relaxed);
    //     }

    //     let now = rtc.current_time_us() * 1000;
    //     if last + UPDATE_INTERVAL < now {
    //         last = now;
    //         let l = left.corrected_value();
    //         let r = right.corrected_value();
    //         let w = (l as f32 / LEFT_FACTOR) + (r as f32 / RIGHT_FACTOR);

    //         let av = values.average();
    //         if av == 0.0 || w < (1.0 + av) * (1.0 + av) * threshold {
    //             values.push(w);
    //         }

    //         let av = values.corrected_average();


    //         // int repr of grams *10
    //         let i = if av < 1.0 {
    //             0
    //         } else {
    //             (av * 10.0) as i16
    //         };

    //         #[cfg(feature = "log_weights")]
    //         {
    //             let tare = SHOULD_TARE.load(Ordering::Relaxed);
    //             let enable = ENABLE_DRIVERS.load(Ordering::Relaxed);
    //             let disable = DISABLE_DRIVERS.load(Ordering::Relaxed);
    //             log::info!("t:{tare} e:{enable} d:{disable}: {l} + {r} = {av} -> {i}");
    //         }

    // TODO(richo) notification stuff
    // let mut cccd = [0u8; 1];
    // if let Some(1) = srv.get_characteristic_value(
    //     weight_notify_enable_handle,
    //     0,
    //     &mut cccd,
    // ) {
        // // if notifications enabled
        // if cccd[0] == 1 {
        //     // log::info!("{i} {:x?}", &payload[..]);
        //     notification = Some(NotificationData::new(
        //             weight_handle,
        //             &payload[..],
        //     ));
        // }

fn pack_weight(weight: i16) -> [u8; 7] {
        let mut payload = [0x03, 0xCE, 0x00, 0x00, 0x00, 0x00, 0x00];
        payload[2..4].copy_from_slice(&weight.to_be_bytes());

        // Calculate the xor thingy
        let mut xor = 0x00;
        for b in &payload[..] {
            xor ^= b;
        }
        payload[6] = xor;

        payload

}


async fn advertise<'values, 'server, C: Controller>(
	name: &'values str,
	peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
	server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
	let mut advertiser_data = [0; 31];
	let len = AdStructure::encode_slice(
		&[
		AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
		AdStructure::ServiceUuids16(&[[0x09, 0x18]]),
		AdStructure::CompleteLocalName(name.as_bytes()),
		],
		&mut advertiser_data[..],
	)?;
	let advertiser = peripheral
		.advertise(
			&Default::default(),
			Advertisement::ConnectableScannableUndirected {
				adv_data: &advertiser_data[..len],
				scan_data: &[],
			},
		)
		.await?;
	log::info!("[adv] advertising");
	let conn = advertiser.accept().await?.with_attribute_server(server)?;
	log::info!("[adv] connection established");
	Ok(conn)
}

async fn custom_task<C: Controller, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let mut tick: i16 = 0;
    let weight = server.scale_service.weight;
    loop {
        tick = tick.wrapping_add(1);
        log::info!("[custom_task] notifying connection of tick {}", tick);
        // TODO(richo) this is where we shove in the weight I think
        if weight.notify(conn, &tick).await.is_err() {
            log::info!("[custom_task] error notifying connection");
            break;
        };
        // read RSSI (Received Signal Strength Indicator) of the connection.
        if let Ok(rssi) = conn.raw().rssi(stack).await {
            log::info!("[custom_task] RSSI: {:?}", rssi);
        } else {
            log::info!("[custom_task] error getting RSSI");
            break;
        };
        Timer::after_secs(2).await;
    }
}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            panic!("[ble_task] error: {:?}", e);
        }
    }
}
