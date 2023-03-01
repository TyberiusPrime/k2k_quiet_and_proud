use stm32f1xx_hal::gpio::{gpioa::*, gpiob::*, Input, OpenDrain, Output, PullUp};
//use stm32f1xx_hal::prelude::*;
use crate::StringSender;
use cortex_m;
use embedded_hal::digital::v2::{InputPin, OutputPin};
#[allow(unused_imports)]
use embedded_hal::digital::v2_compat;
use no_std_compat::prelude::v1::*;
use smallbitvec::SmallBitVec;

type SinksA = Vec<PAx<Input<PullUp>>>;
type SinksB = Vec<PBx<Input<PullUp>>>;

pub struct Matrix {
    sinks_pa: SinksA,
    sinks_pb: SinksB,
    sources_pa: Vec<PAx<Output<OpenDrain>>>,
    sources_pb: Vec<PBx<Output<OpenDrain>>>,
    pub output: SmallBitVec,
    singles_pb: SinksB,
}

impl Matrix {
    pub fn new(
        sinks_pa: SinksA,
        sinks_pb: SinksB,
        sources_pa: Vec<PAx<Output<OpenDrain>>>,
        sources_pb: Vec<PBx<Output<OpenDrain>>>,
        singles_pb: SinksB,
    ) -> Matrix {
        let sink_count = sinks_pa.len() + sinks_pb.len();
        let source_count = sources_pa.len() + sources_pb.len();
        let single_count = singles_pb.len();
        let output = SmallBitVec::with_capacity(sink_count * source_count + single_count);
        Matrix {
            sinks_pa,
            sinks_pb,
            sources_pa,
            sources_pb,
            output,
            singles_pb,
        }
    }

    pub fn len(&self) -> usize {
        return self.output.capacity();
    }

    pub fn read_matrix(&mut self) {
        self.output.clear();
        for source in self.sources_pa.iter_mut() {
            source.set_high().ok();
        }
        for source in self.sources_pb.iter_mut() {
            source.set_high().ok();
        }

        for source in self.sources_pa.iter_mut() {
            source.set_low().ok();
            Self::read_row(&mut self.output, &self.sinks_pa, &self.sinks_pb);
            source.set_high().ok();
        }
        for source in self.sources_pb.iter_mut() {
            source.set_low().ok();
            Self::read_row(&mut self.output, &self.sinks_pa, &self.sinks_pb);
            source.set_high().ok();
        }

        for single in self.singles_pb.iter_mut() {
            self.output.push(single.is_low().unwrap_or(false));
        }
    }

    fn read_row(output: &mut SmallBitVec, sinks_pa: &SinksA, sinks_pb: &SinksB) {
        cortex_m::asm::delay(4800);
        for sink in sinks_pa.iter() {
            output.push(sink.is_low().unwrap_or(false));
        }
        for sink in sinks_pb.iter() {
            output.push(sink.is_low().unwrap_or(false));
        }
    }

    pub fn debug_serial(states: &SmallBitVec, tx: &mut impl StringSender) {
        //let mut counter = 0;
        for (ii, value) in states.iter().enumerate() {
            if value {
                let o = format!("{}", ii);
                tx.writeln(&o);
                //       counter += 1;
            }
        }
        //tx.writeln(&format!("Count: {}\r\n", counter));
    }
}
