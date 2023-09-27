#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay, IO};
use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    attribute_server::{AttributeServer, NotificationData, WorkResult},
    Ble, HciConnector,
};
use bleps_macros::gatt;

use hx711;
use nb::block;

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};

use esp_wifi::{ble::controller::BleConnector, initialize, EspWifiInitFor};

use hal::{timer::TimerGroup, Rng};
use core::sync::atomic::{AtomicI32, AtomicBool, Ordering};
use core::iter::Sum;
// use core::thread;
use uuid::Uuid;

static weight: AtomicI32 = AtomicI32::new(0);
static should_tare: AtomicBool = AtomicBool::new(false);


trait Scale {
    fn tare(&mut self) -> i32 {
        const N: i32 = 8;
        let mut val = 0;
        for _ in 0..N {
            val += self.value();
        }
        val / N
    }

    fn value(&mut self) -> i32;

    fn corrected_value(&mut self, tare: i32) -> i32 {
        self.value() - tare
    }
}

impl<D, IN, OUT> Scale for hx711::Hx711<D, IN, OUT>
where
    D: DelayUs<u32>,
    IN: InputPin,
    IN::Error: core::fmt::Debug,
    OUT: OutputPin,
    OUT::Error: core::fmt::Debug,
{

    fn value(&mut self) -> i32 {
        let mut val = -1;
        while val == -1 {
            val = block!(self.retrieve()).unwrap();
        }
        val
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");
    println!("Hello world!");
    let timer = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    )
    .timer0;
    let init = initialize(
        EspWifiInitFor::Ble,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let (_, mut bluetooth, ..) = peripherals.RADIO.split();


    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut left = {
        let dout = io.pins.gpio16.into_floating_input();
        let pd_sck = io.pins.gpio4.into_push_pull_output();
        hx711::Hx711::new(delay, dout, pd_sck).unwrap()
    };

    let mut right = {
        let dout = io.pins.gpio18.into_floating_input();
        let pd_sck = io.pins.gpio5.into_push_pull_output();
        hx711::Hx711::new(delay, dout, pd_sck).unwrap()
    };

    let right_scale = 273577.0 / 807.5 / 2.0;
    let left_scale = 319663.0 / 807.5 / 2.0;

    let factor = (273577.0 + 319663.0) / 807.5;

    // loop {
    //     let l = left.corrected_value(left_tare);
    //     let r = right.corrected_value(right_tare);
    //     let w = (l as f32 / left_scale) + (r as f32 / right_scale);
    //     weight.store((((l + r) as f32 / factor) * 10.0 ) as i32, Ordering::Relaxed);
    //     log::info!("val: {w}\t{}\t{}", l, r);

    // }
    loop {
        let connector = BleConnector::new(&init, &mut bluetooth);
        let hci = HciConnector::new(connector, esp_wifi::current_millis);
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
        let mut rf = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            17
        };
        let mut wf = |offset: usize, data: &[u8]| {
            println!("RECEIVED: Offset {}, data {:?}", offset, data);
            if data == &[0x03, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0C] || true {
                println!("Triggering tare");
                should_tare.store(true, Ordering::Relaxed);
            }
        };

        let mut wf2 = |offset: usize, data: &[u8]| {
            println!("RECEIVED: {} {:?}", offset, data);
        };

        let mut rf3 = |_offset: usize, data: &mut [u8]| {
            data[..5].copy_from_slice(&b"Hola!"[..]);
            5
        };

        let mut wf3 = |offset: usize, data: &[u8]| {
            println!("RECEIVED: Offset {}, data {:?}", offset, data);
            if data == &[0x03, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0C] || true {
                println!("Triggering tare");
                should_tare.store(true, Ordering::Relaxed);
            }
        };

        gatt!([service {
            uuid: "FFF0",
            characteristics: [
                characteristic {
                    uuid: "000036F5-0000-1000-8000-00805F9B34FB",
                    read: rf,
                    write: wf,
                },
                characteristic {
                    uuid: "957312e0-2354-11eb-9f10-fbc30a62cf38",
                    write: wf2,
                },
                characteristic {
                    name: "weight",
                    uuid: "0000FFF4-0000-1000-8000-00805F9B34FB",
                    notify: true,
                    read: rf3,
                    write: wf3,
                },
            ],
        },]);

        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes);

        let mut left_tare = left.tare();
        let mut right_tare = right.tare();

        let mut values = [0.0, 0.0, 0.0, 0.0];

        // We'll ignore spikes greater than this magnitude
        let threshold = 100.0;

        loop {
            let mut notification = None;

            if should_tare.load(Ordering::Relaxed) {
                left_tare = left.tare();
                right_tare = right.tare();
                should_tare.store(false, Ordering::Relaxed);
            }

            let l = left.corrected_value(left_tare);
            let r = right.corrected_value(right_tare);
            // TODO(richo) figure out how to do this at 10hz
            // let w = (l as f32 / left_scale) + (r as f32 / right_scale);
            let w = (l + r) as f32 / factor;

            let av = values.iter().sum::<f32>() / 4.0;
            if av == 0.0 || w < (1.0 + av) * (1.0 + av) * threshold {
                values.rotate_right(1);
                values[0] = w;
            }

            let av: f32 = values.iter().sum::<f32>() / 4.0;

            // int repr of grams *10
            let i = (av * 10.0) as i16;

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
                        break;
                    }
                }
                Err(err) => {
                    println!("{:?}", err);
                }
            }
        }
    }
}
