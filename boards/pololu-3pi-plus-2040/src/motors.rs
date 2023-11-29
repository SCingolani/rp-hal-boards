use embedded_hal::{digital::v2::OutputPin, PwmPin};

use crate::{MotorLeftDir, MotorRightDir};

pub const MAX_SPEED: u16 = 60000;

pub struct Motors {
    pub(crate) pwm: hal::pwm::Slice<hal::pwm::Pwm7, hal::pwm::FreeRunning>,
    pub(crate) left_dir: MotorLeftDir,
    pub(crate) right_dir: MotorRightDir,
}

impl Motors {
    pub fn set_speeds(&mut self, left: i32, right: i32) {
        let left_duty = left.abs().clamp(0, MAX_SPEED as i32 - 1) as u16;
        let right_duty = right.abs().clamp(0, MAX_SPEED as i32 - 1) as u16;
        self.pwm.channel_b.set_duty(left_duty);
        self.pwm.channel_a.set_duty(right_duty);
        if left.is_negative() {
            self.left_dir.set_low().unwrap();
        } else {
            self.left_dir.set_high().unwrap();
        }
        if right.is_negative() {
            self.right_dir.set_high().unwrap();
        } else {
            self.right_dir.set_low().unwrap();
        }
    }
}
