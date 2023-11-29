use embedded_hal::digital::v2::OutputPin;
use pio::Instruction;
use rp2040_hal::{
    pac::PIO1,
    pio::{InstalledProgram, UninitStateMachine, PIO, SM0},
};

use crate::{BE, BL, BR, DN1, DN2, DN3, DN4, DN5, DNE};

pub const MAX_VALUE: u16 = 1024;

pub struct IrSensors<const FIRST_PIN: u8> {
    program: Option<InstalledProgram<PIO1>>,
    sm: Option<UninitStateMachine<(PIO1, SM0)>>,
    dne: DNE,
    be: BE,
}

impl<const FIRST_PIN: u8> IrSensors<FIRST_PIN> {
    pub fn new(
        pins: (BL, BR, DN1, DN2, DN3, DN4, DN5, DNE, BE),
        mut pio: PIO<PIO1>,
        sm: UninitStateMachine<(PIO1, SM0)>,
    ) -> Self {
        let ir_program = pio_proc::pio_file!("src/qtr_sensor_counter.pio");

        let installed = pio.install(&ir_program.program).unwrap();

        assert!(pins.1.id().num == FIRST_PIN);

        Self {
            program: Some(installed),
            sm: Some(sm),
            dne: pins.7,
            be: pins.8,
        }
    }

    fn read(&mut self, delay: &mut cortex_m::delay::Delay) -> [u16; 7] {
        let installed = self.program.take().unwrap();
        let sm = self.sm.take().unwrap();
        let (mut sm, mut rx, tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
            .in_shift_direction(hal::pio::ShiftDirection::Left)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .push_threshold(16 + 7)
            .autopush(true)
            .in_pin_base(FIRST_PIN)
            .out_pins(FIRST_PIN, 7)
            .clock_divisor_fixed_point(15, 160) // 125/(15+160/256) = 8 MHz
            .build(sm);
        sm.set_pins((FIRST_PIN..FIRST_PIN + 7).map(|id| (id, hal::pio::PinState::High)));
        sm.set_pindirs((FIRST_PIN..FIRST_PIN + 7).map(|id| (id, hal::pio::PinDir::Output)));
        delay.delay_us(32);

        sm.clear_fifos();

        sm.exec_instruction(Instruction {
            operands: pio::InstructionOperands::MOV {
                destination: pio::MovDestination::OSR,
                op: pio::MovOperation::Invert,
                source: pio::MovSource::NULL,
            },
            delay: 0,
            side_set: None,
        });

        sm.exec_instruction(Instruction {
            operands: pio::InstructionOperands::OUT {
                destination: pio::OutDestination::Y,
                bit_count: 10,
            },
            delay: 0,
            side_set: None,
        });

        sm.exec_instruction(Instruction {
            operands: pio::InstructionOperands::MOV {
                destination: pio::MovDestination::OSR,
                op: pio::MovOperation::None,
                source: pio::MovSource::NULL,
            },
            delay: 0,
            side_set: None,
        });

        let sm = sm.start();

        let mut last_state = 0xFF;
        let mut output: [u16; 7] = [MAX_VALUE; 7];

        loop {
            if let Some(data) = rx.read() {
                if data == 0xFFFFFFF {
                    break;
                }
                let time_left: u16 = (data & 0xFFFF) as u16;
                let state = data >> 16 & 0b1111111;
                let new_zeros = last_state & !state;

                for (i, output) in output.iter_mut().enumerate() {
                    if new_zeros & (1 << i) != 0 {
                        *output = MAX_VALUE - time_left;
                    }
                }
                last_state = state;
            }
        }
        let sm = sm.stop();
        let (sm, installed) = sm.uninit(rx, tx);
        self.sm = Some(sm);
        self.program = Some(installed);
        output
    }

    pub fn measure_downward_facing(&mut self, delay: &mut cortex_m::delay::Delay) -> [u16; 5] {
        self.dne.set_high().unwrap();
        let mut result = self.read(delay);
        self.dne.set_low().unwrap();
        result.reverse();
        [result[0], result[1], result[2], result[3], result[4]]
    }

    pub fn measure_bumpers(&mut self, delay: &mut cortex_m::delay::Delay) -> [u16; 2] {
        self.be.set_high().unwrap();
        let result = self.read(delay);
        self.be.set_low().unwrap();
        [result[0], result[1]]
    }

    pub fn measure_all(&mut self, delay: &mut cortex_m::delay::Delay) -> [u16; 7] {
        self.dne.set_high().unwrap();
        self.be.set_high().unwrap();
        let result = self.read(delay);
        self.dne.set_low().unwrap();
        self.be.set_low().unwrap();
        result
    }
}
