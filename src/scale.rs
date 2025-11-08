use nb::block;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};

pub trait ScaleExt {
    fn tare_value(&mut self) -> i32 {
        const N: i32 = 8;
        let mut val = 0;
        for _ in 0..N {
            val += self.value();
        }
        val / N
    }

    fn value(&mut self) -> i32;

    fn enable(&mut self);
    fn disable(&mut self);
}

impl<D, IN, OUT> ScaleExt for hx711::Hx711<D, IN, OUT>
where
    D: DelayNs,
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

    fn enable(&mut self) {
        let _ = hx711::Hx711::enable(self);
    }

    fn disable(&mut self) {
        let _ = hx711::Hx711::disable(self);
    }
}

pub struct Scale<'a> {
    sensor: &'a mut dyn ScaleExt,
    offset: i32,
}

impl<'a> Scale<'a> {
    pub fn new(sensor: &'a mut dyn ScaleExt) -> Self {
        let offset = sensor.tare_value();
        Self {
            sensor, offset
        }
    }

    pub fn tare(&mut self) {
        self.offset = self.sensor.tare_value()
    }

    pub fn corrected_value(&mut self) -> i32 {
        self.sensor.value() - self.offset
    }

    pub fn disable(&mut self) {
        self.sensor.disable();
    }

    pub fn enable(&mut self) {
        self.sensor.enable();
    }
}

