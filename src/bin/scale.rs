#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::main;
use hal::clock::CpuClock;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    attribute_server::{AttributeServer, NotificationData, WorkResult},
    gatt, Addr, Ble, HciConnector,
};

use hx711;

use embedded_hal::delay::DelayNs;

// use esp_wifi::{ble::controller::BleConnector, initialize, EspWifiInitFor};
use esp_radio::ble::controller::{BleConnector};

use hal::gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull};
use hal::time::{self, Duration};

use hal::rng::{Trng, TrngSource};

use hal::delay::Delay;
use hal::rtc_cntl::Rtc;
use hal::timer::timg::TimerGroup;
use core::sync::atomic::{AtomicBool, Ordering};


use scale::{Scale, Buffer};

static SHOULD_TARE: AtomicBool = AtomicBool::new(false);
static DISABLE_DRIVERS: AtomicBool = AtomicBool::new(false);
static ENABLE_DRIVERS: AtomicBool = AtomicBool::new(false);

const UPDATE_INTERVAL: u64 = 200;
const TARE_DEBOUNCE: u64 = 500;

// Calibrated with the drip tray in situ
const FACTOR: f32 = (621670.25) / 99.9;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let now = || time::Instant::now().duration_since_epoch().as_millis();

    esp_println::logger::init_logger_from_env();

    let config = hal::Config::default().with_cpu_clock(CpuClock::max());
    let mut peripherals = hal::init(config);

    esp_alloc::heap_allocator!(#[hal::ram(reclaimed)] size: 98768);

    let rtc = Rtc::new(peripherals.LPWR);

    let _trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1.reborrow());
    let mut trng = Trng::try_new().unwrap();

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
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

    // let right_scale = 273577.0 / 807.5 / 2.0;
    // let left_scale = 319663.0 / 807.5 / 2.0;

    // Calibrated with nothing on the load cell
    // let factor = (273577.0 + 319663.0) / 807.5;


    // loop {
    //     let l = left.corrected_value(left_tare);
    //     let r = right.corrected_value(right_tare);
    //     let w = (l as f32 / left_scale) + (r as f32 / right_scale);
    //     weight.store((((l + r) as f32 / factor) * 10.0 ) as i32, Ordering::Relaxed);
    //     log::info!("val: {w}\t{}\t{}", l, r);

    // }

    let mut last = rtc.current_time_us() * 1000;
    let connector = BleConnector::new(&radio_init, peripherals.BT, Default::default()).expect("Couldn't build connector");
    let hci = HciConnector::new(connector, now);


    let mut ble = Ble::new(&hci);

    println!("{:?}", ble.init());
    println!("{:?}", ble.cmd_set_le_advertising_parameters());
    println!(
        "{:?}",
        ble.cmd_set_le_advertising_data(
            create_advertising_data(&[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                AdStructure::CompleteLocalName("Decent Scale 2"),
            ]).unwrap()
        )
    );
    println!("{:?}", ble.cmd_set_le_advertise_enable(true));

    println!("started advertising");
    let mut wf = |offset: usize, data: &[u8]| {
        println!("RECEIVED: Offset {}, data {:x?}", offset, data);
        match data {
            // CMD TARE
            [0x03, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0C] => {
                SHOULD_TARE.store(true, Ordering::Relaxed);
                println!("TARE: Triggering tare");
            },

            // TIMER ZERO
            [0x03, 0x0b, 0x02, 0x00, 0x00, 0x00, 0x0a] => {
                SHOULD_TARE.store(true, Ordering::Relaxed);
                println!("TIMERZERO: Triggering tare because of timer");
            }
            // TIMER STOP
            [0x03, 0x0b, 0x02, 0x00, 0x00, 0x00, 0x08] => {
                println!("TIMERSTOP: Got a timer stop");
            }

            //TIMERSTART
            [0x03, 0x0B, 0x03, 0x00, 0x00, 0x00, 0x0B] => {
                println!("TIMERSTART:");
            }

            // LED ON
            [0x03, 0x0A, 0x01, 0x01, 0x00, 0x00, 0x09] => {
                ENABLE_DRIVERS.store(true, Ordering::Relaxed);
                SHOULD_TARE.store(true, Ordering::Relaxed);
                println!("LEDON: enabling hx711's and taring.");
            }
            // LED OFF
            [0x03, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x09] => {
                DISABLE_DRIVERS.store(true, Ordering::Relaxed);
                println!("LEDOFF: disabling hx711's.");
            }
            _ => {},
        }
    };
    let mut rf3 = |_offset: usize, _data: &mut [u8]| {
        0
    };

    gatt!([service {
        uuid: "FFF0",
        characteristics: [
            characteristic {
                uuid: "000036F5-0000-1000-8000-00805F9B34FB",
                write: wf,
            },
            characteristic {
                name: "weight",
                uuid: "0000FFF4-0000-1000-8000-00805F9B34FB",
                notify: true,
                read: rf3,
            },
        ],
    },]);

    let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut trng);

    let mut values: Buffer<3> = Buffer::new();

    let mut last_tare_press = rtc.current_time_us() * 1000;

    // We'll ignore spikes greater than this magnitude
    let threshold = 500.0;

    loop {
        match srv.do_work() {
            Ok(res) => {
                if let WorkResult::GotDisconnected = res {
                    println!("We're diconnected");
                }
            }
            Err(err) => {
                println!("{:?}", err);
            }
        }

        let mut notification = None;
        if ENABLE_DRIVERS.load(Ordering::Relaxed) {
            println!("enabling hx711's");
            let _ = left.enable();
            let _ = right.enable();
            ENABLE_DRIVERS.store(false, Ordering::Relaxed);
            // Give them a sec to wake up before we tare
            delay.delay_ns(100_000u32);
        }

        if DISABLE_DRIVERS.load(Ordering::Relaxed) {
            println!("disabling hx711's");
            let _ = left.disable();
            let _ = right.disable();
            DISABLE_DRIVERS.store(false, Ordering::Relaxed);
        }

        if SHOULD_TARE.load(Ordering::Relaxed) {
            println!("Taring");
            left.tare();
            right.tare();
            // Fill the values buffer back up with zeros;
            values.zero();
            SHOULD_TARE.store(false, Ordering::Relaxed);
        }

        let now = rtc.current_time_us() * 1000;
        if tare.is_low() && last_tare_press + TARE_DEBOUNCE < now {
            println!("Taring because of buttan");
            left.tare();
            right.tare();
            last_tare_press = now;

            values.zero()
        }

        let now = rtc.current_time_us() * 1000;
        if last + UPDATE_INTERVAL < now {
            last = now;
            let l = left.corrected_value();
            let r = right.corrected_value();
            // TODO(richo) figure out how to do this at 10hz
            // let w = (l as f32 / left_scale) + (r as f32 / right_scale);
            let w = (l + r) as f32 / FACTOR;

            let av = values.average();
            if av == 0.0 || w < (1.0 + av) * (1.0 + av) * threshold {
                values.push(w);
            }

            let av = values.corrected_average();


            // int repr of grams *10
            let i = if av < 1.0 {
                0
            } else {
                (av * 10.0) as i16
            };

            #[cfg(feature = "log_weights")]
            {
                let tare = SHOULD_TARE.load(Ordering::Relaxed);
                let enable = ENABLE_DRIVERS.load(Ordering::Relaxed);
                let disable = DISABLE_DRIVERS.load(Ordering::Relaxed);
                println!("t:{tare} e:{enable} d:{disable}: {l} + {r} = {av} -> {i}");
            }

            let mut cccd = [0u8; 1];
            if let Some(1) = srv.get_characteristic_value(
                weight_notify_enable_handle,
                0,
                &mut cccd,
            ) {
                let mut payload = [0x03, 0xCE, 0x00, 0x00, 0x00, 0x00, 0x00];
                payload[2..4].copy_from_slice(&i.to_be_bytes());

                // Calculate the xor thingy
                let mut xor = 0x00;
                for b in &payload[..] {
                    xor ^= b;
                }
                payload[6] = xor;

                // if notifications enabled
                if cccd[0] == 1 {
                    // println!("{i} {:x?}", &payload[..]);
                    notification = Some(NotificationData::new(
                            weight_handle,
                            &payload[..],
                    ));
                }
            }


            match srv.do_work_with_notification(notification) {
                Ok(res) => {
                    if let WorkResult::GotDisconnected = res {
                        println!("Disconnected");

                    }
                }
                Err(err) => {
                    println!("{:?}", err);
                }
            }
        }
    }
}
