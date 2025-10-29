#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay, IO};
use hx711;


use esp_wifi::{initialize, EspWifiInitFor};

use hal::{timer::TimerGroup, Rng, Rtc};
use scale::{Scale, Buffer};

const UPDATE_INTERVAL: u64 = 200;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let rtc = Rtc::new(peripherals.RTC_CNTL);

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");
    let timer = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    )
    .timer0;
    let _init = initialize(
        EspWifiInitFor::Ble,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();


    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let dout = io.pins.gpio16.into_floating_input();
    let pd_sck = io.pins.gpio4.into_push_pull_output();
    let mut hx = hx711::Hx711::new(delay, dout, pd_sck).unwrap();
    let mut left = Scale::new(&mut hx);

    let dout = io.pins.gpio18.into_floating_input();
    let pd_sck = io.pins.gpio5.into_push_pull_output();
    let mut hx = hx711::Hx711::new(delay, dout, pd_sck).unwrap();
    let mut right = Scale::new(&mut hx);

    let mut values: Buffer<4> = Buffer::new();

    let mut last = rtc.get_time_ms();
    loop {
        if last + UPDATE_INTERVAL < rtc.get_time_ms() {
            delay.delay_ms(50u32);
        }

        let l = left.corrected_value();
        let r = right.corrected_value();

        values.push((l + r) as f32);
        println!("{l} + {r} => {}", values.average());
        last = rtc.get_time_ms();
    }
}
