use pio::Instruction;
use rp2040_hal::{
    pac::PIO1,
    pio::{Rx, StateMachine, Stopped, Tx, SM0},
};

pub struct IrSensors {
    pub(crate) sm: StateMachine<(PIO1, SM0), Stopped>,
    pub(crate) rx: Rx<(PIO1, SM0)>,
    pub(crate) tx: Tx<(PIO1, SM0)>,
}

impl IrSensors {
    const TIMEOUT: u16 = 1024;

    pub fn read(mut self, delay: &mut cortex_m::delay::Delay) -> (Self, [u16; 5]) {
        let (sm, installed) = self.sm.uninit(self.rx, self.tx);
        let (sm, mut rx, tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
            .in_shift_direction(hal::pio::ShiftDirection::Left)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .push_threshold(16 + 5)
            .autopush(true)
            .in_pin_base(18)
            .out_pins(18, 5)
            .clock_divisor_fixed_point(15, 160) // 125/(15+160/256) = 8 MHz
            .build(sm);
        self.sm = sm;
        self.sm
            .set_pins((18..18 + 5).map(|id| (id, hal::pio::PinState::High)));
        self.sm
            .set_pindirs((18..18 + 5).map(|id| (id, hal::pio::PinDir::Output)));
        delay.delay_us(32);

        self.sm.clear_fifos();

        self.sm.exec_instruction(Instruction {
            operands: pio::InstructionOperands::MOV {
                destination: pio::MovDestination::OSR,
                op: pio::MovOperation::Invert,
                source: pio::MovSource::NULL,
            },
            delay: 0,
            side_set: None,
        });

        self.sm.exec_instruction(Instruction {
            operands: pio::InstructionOperands::OUT {
                destination: pio::OutDestination::Y,
                bit_count: 10,
            },
            delay: 0,
            side_set: None,
        });

        self.sm.exec_instruction(Instruction {
            operands: pio::InstructionOperands::MOV {
                destination: pio::MovDestination::OSR,
                op: pio::MovOperation::None,
                source: pio::MovSource::NULL,
            },
            delay: 0,
            side_set: None,
        });

        let sm = self.sm.start();

        let mut last_state = 0xFF;
        let mut output: [u16; 5] = [Self::TIMEOUT; 5];

        loop {
            if let Some(data) = rx.read() {
                if data == 0x1FFFFF {
                    break;
                }
                let time_left: u16 = (data & 0xFFFF) as u16;
                let state = data >> 16 & 0b11111;
                let new_zeros = last_state & !state;

                for i in 0..output.len() {
                    if new_zeros & (1 << i) != 0 {
                        output[i] = Self::TIMEOUT - time_left;
                    }
                }
                last_state = state;
            }
        }
        let sm = sm.stop();

        (Self { sm, rx, tx }, output)
    }
}
