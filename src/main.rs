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

struct Scale<D, IN, OUT>
where
    D: DelayUs<u32>,
    IN: InputPin,
    IN::Error: core::fmt::Debug,
    OUT: OutputPin,
    OUT::Error: core::fmt::Debug,
{
    hx: hx711::Hx711<D, IN, OUT>,
    tare: i32,
}

impl<D, IN, OUT> Scale<D, IN, OUT>
where
    D: DelayUs<u32>,
    IN: InputPin,
    IN::Error: core::fmt::Debug,
    OUT: OutputPin,
    OUT::Error: core::fmt::Debug,
{
    fn new(hx: hx711::Hx711<D, IN, OUT>) -> Self {
        Self {
            hx,
            tare: 0,
        }
    }

    fn value(&mut self) -> i32 {
        block!(self.hx.retrieve()).unwrap() - self.tare
    }

    fn tare(&mut self) {
        let tare;
        const N: i32 = 8;
        let mut val = 0;
        for _ in 0..N {
            val += block!(self.hx.retrieve()).unwrap(); // or unwrap, see features below
        }
        tare = val / N;
        log::info!("Tared at {}", self.tare);
        self.tare = tare;
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

    let dout = io.pins.gpio16.into_floating_input();
    let pd_sck = io.pins.gpio4.into_push_pull_output();
    let mut left = Scale::new(hx711::Hx711::new(delay, dout, pd_sck).unwrap());
    left.tare();

    let dout = io.pins.gpio16.into_floating_input();
    let pd_sck = io.pins.gpio4.into_push_pull_output();
    let mut right = Scale::new(hx711::Hx711::new(delay, dout, pd_sck).unwrap());
    right.tare();

    loop {
        let val = left.value();
        log::info!("val: {}\t", val);

    }
}
