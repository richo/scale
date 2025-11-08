#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::clock::CpuClock;
use hx711;
use hal::main;


use hal::rtc_cntl::Rtc;
use hal::time;
use hal::timer::timg::TimerGroup;
use hal::delay::Delay;
use embedded_hal::delay::DelayNs;
use hal::gpio::{Input, InputConfig, Io, Level, Output, OutputConfig, Pull};
use scale::{Scale, Buffer};

const UPDATE_INTERVAL: u64 = 200;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let now = || time::Instant::now().duration_since_epoch().as_millis();
    esp_println::logger::init_logger_from_env();
    let config = hal::Config::default().with_cpu_clock(CpuClock::max());
    let mut peripherals = hal::init(config);

    let rtc = Rtc::new(peripherals.LPWR);
    let output_config = OutputConfig::default();

    log::info!("Logger is setup");
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let mut io = Io::new(peripherals.IO_MUX);
    let mut delay = Delay::new();

    let mut led = Output::new(peripherals.GPIO2, Level::Low, output_config);
    led.set_high();
    log::info!("LED on?");

    let floating_config = InputConfig::default();

    let dout = Input::new(peripherals.GPIO16, floating_config);
    let pd_sck = Output::new(peripherals.GPIO4, Level::Low, output_config);
    let mut hx = hx711::Hx711::new(delay, dout, pd_sck).unwrap();
    let mut left = Scale::new(&mut hx);

    let dout = Input::new(peripherals.GPIO18, floating_config);
    let pd_sck = Output::new(peripherals.GPIO5, Level::Low, output_config);
    let mut hx = hx711::Hx711::new(delay, dout, pd_sck).unwrap();
    let mut right = Scale::new(&mut hx);

    let mut values: Buffer<4> = Buffer::new();

    let mut last = now();
    loop {
        if last + UPDATE_INTERVAL < now() {
            delay.delay_ns(50u32);
        }

        let l = left.corrected_value();
        let r = right.corrected_value();

        values.push((l + r) as f32);
        println!("{l} + {r} => {}", values.average());
        last = now();
    }
}
