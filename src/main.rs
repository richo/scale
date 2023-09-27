#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay, IO};

use hx711;
use nb::block;

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};

use esp_wifi::{initialize, EspWifiInitFor};

use hal::{timer::TimerGroup, Rng};

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
    let _init = initialize(
        EspWifiInitFor::Ble,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut left = {
        let dout = io.pins.gpio16.into_floating_input();
        let pd_sck = io.pins.gpio4.into_push_pull_output();
        hx711::Hx711::new(delay, dout, pd_sck).unwrap()
    };
    let left_tare = left.tare();

    let mut right = {
        let dout = io.pins.gpio16.into_floating_input();
        let pd_sck = io.pins.gpio4.into_push_pull_output();
        hx711::Hx711::new(delay, dout, pd_sck).unwrap()
    }
    let right_tare = left.tare();

    loop {
        let val = left.corrected_value(left_tare);
        log::info!("val: {}\t", val);

    }
}
