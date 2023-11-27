#![no_std]

//! A Hardware Abstraction Layer for Pololu's 3Pi+ 2040 robot.
//!
//! This crate serves as a HAL (Hardware Abstraction Layer) for Pololu's 3Pi+ 2040 robot. Since the
//! it is based on the RP2040 chip, it re-exports the [rp2040_hal] crate which contains the tooling
//! to work with the rp2040 chip.
//!
//! # Examples:
//!
//! The following example turns on the onboard LED. Note that most of the logic works through the [rp2040_hal] crate.
//! ```ignore
//! #![no_main]
//! use rp_pico::entry;
//! use panic_halt as _;
//! use embedded_hal::digital::v2::OutputPin;
//! use rp_pico::hal::pac;
//! use rp_pico::hal;

//! #[entry]
//! fn does_not_have_to_be_main() -> ! {
//!   let mut pac = pac::Peripherals::take().unwrap();
//!   let sio = hal::Sio::new(pac.SIO);
//!   let pins = rp_pico::Pins::new(
//!        pac.IO_BANK0,
//!        pac.PADS_BANK0,
//!        sio.gpio_bank0,
//!        &mut pac.RESETS,
//!   );
//!   let mut led_pin = pins.led.into_push_pull_output();
//!   led_pin.set_high().unwrap();
//!   loop {
//!   }
//! }
//! ```

pub mod visual_output;

pub extern crate rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;

use embedded_hal::blocking::delay::DelayMs;
use fugit::HertzU32;
/// The `entry` macro declares the starting function to the linker.
/// This is similar to the `main` function in console applications.
///
/// It is based on the [cortex_m_rt](https://docs.rs/cortex-m-rt/latest/cortex_m_rt/attr.entry.html) crate.
///
/// # Examples
/// ```ignore
/// #![no_std]
/// #![no_main]
/// use rp_pico::entry;
/// #[entry]
/// fn you_can_use_a_custom_main_name_here() -> ! {
///   loop {}
/// }
/// ```

#[cfg(feature = "rt")]
pub use hal::entry;
use hal::{
    pac::I2C0,
    pio::{PIOExt, Rx, UninitStateMachine},
    I2C,
};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub use hal::pac;
use lis3mdl::Lis3mdl;
use lsm6dso::Lsm6dso;
use visual_output::VisualOutput;

hal::bsp_pins!(
    /// GPIO 0 is connected to Button C and to the Display data/command select (D/C̅)
    Gpio0 {
        name: gpio0,
        aliases: {
            /// Input Function alias for ButtonC.
            FunctionSioInput, PullUp: ButtonC,
            /// Display data/command select (D/C̅) alias.
            FunctionSioOutput, PullNone: DisplayDC
        }
    },

    /// GPIO 1 is connected to the Display R̅E̅S̅E̅T̅
    Gpio1 {
        name: gpio1,
        aliases: {
            /// Digital output for display reset
            FunctionSioOutput, PullNone: DisplayReset
        }
    },

    /// GPIO 2 is connected to Display SCK
    Gpio2 {
        name: gpio2,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio2].
            FunctionSpi, PullNone: DisplaySCK,
            FunctionSioOutput, PullNone: DisplaySckNoDrive
        }
    },

    /// GPIO 3 is connected to the Display MOSI and the RGB LED data
    Gpio3 {
        name: gpio3,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio3].
            FunctionSpi, PullNone: DisplayMOSI,
            /// SPI Function alias for pin [crate::Pins::gpio3].
            FunctionSpi, PullNone: RgbLedData
        }
    },

    /// GPIO 4 is connected to the inertial sensors through I2C0.
    Gpio4 {
        name: gpio4,
        aliases: {
            /// I2C Function alias for pin [crate::Pins::gpio4].
            FunctionI2C, PullUp: InertialSda
        }
    },

    /// GPIO 5 is connected to the inertial sensors through I2C0.
    Gpio5 {
        name: gpio5,
        aliases: {
            /// I2C Function alias for pin [crate::Pins::gpio5].
            FunctionI2C, PullUp: InertialScl
        }
    },

    /// GPIO 6 is used for the clock of the RGB LEDs
    Gpio6 {
        name: gpio6,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio6].
            FunctionSpi, PullNone: RgbLedSck,
            FunctionSioOutput, PullNone: RgbLedSckNoDrive
        }
    },

    /// GPIO 7 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`    | [crate::Gp7Spi0Tx]          |
    /// | `UART1 RTS`  | [crate::Gp7Uart1Rts]        |
    /// | `I2C1 SCL`   | [crate::Gp7I2C1Scl]         |
    /// | `PWM3 B`     | [crate::Gp7Pwm3B]           |
    /// | `PIO0`       | [crate::Gp7Pio0]            |
    /// | `PIO1`       | [crate::Gp7Pio1]            |
    Gpio7 {
        name: gpio7,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio7].
            FunctionUart, PullNone: Gp7Uart1Rts,
            /// SPI Function alias for pin [crate::Pins::gpio7].
            FunctionSpi, PullNone: Gp7Spi0Tx,
            /// I2C Function alias for pin [crate::Pins::gpio7].
            FunctionI2C, PullUp: Gp7I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::gpio7].
            FunctionPwm, PullNone: Gp7Pwm3B,
            /// PIO0 Function alias for pin [crate::Pins::gpio7].
            FunctionPio0, PullNone: Gp7Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio7].
            FunctionPio1, PullNone: Gp7Pio1
        }
    },

    /// GPIO 8 is connected to the right encoder
    Gpio8 {
        name: gpio8,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio8].
            FunctionPio0, PullNone: EncoderRightA
        }
    },

    /// GPIO 9 is connected to the right encoder
    Gpio9 {
        name: gpio9,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio9].
            FunctionPio0, PullNone: EncoderRightB
        }
    },

    /// GPIO 10 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 SCK`   | [crate::Gp10Spi1Sck]        |
    /// | `UART1 CTS`  | [crate::Gp10Uart1Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp10I2C1Sda]        |
    /// | `PWM5 A`     | [crate::Gp10Pwm5A]          |
    /// | `PIO0`       | [crate::Gp10Pio0]           |
    /// | `PIO1`       | [crate::Gp10Pio1]           |
    Gpio10 {
        name: gpio10,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio10].
            FunctionUart, PullNone: Gp10Uart1Cts,
            /// SPI Function alias for pin [crate::Pins::gpio10].
            FunctionSpi, PullNone: Gp10Spi1Sck,
            /// I2C Function alias for pin [crate::Pins::gpio10].
            FunctionI2C, PullUp: Gp10I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio10].
            FunctionPwm, PullNone: Gp10Pwm5A,
            /// PIO0 Function alias for pin [crate::Pins::gpio10].
            FunctionPio0, PullNone: Gp10Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio10].
            FunctionPio1, PullNone: Gp10Pio1
        }
    },

    /// GPIO 11 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 TX`    | [crate::Gp11Spi1Tx]         |
    /// | `UART1 RTS`  | [crate::Gp11Uart1Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp11I2C1Scl]        |
    /// | `PWM5 B`     | [crate::Gp11Pwm5B]          |
    /// | `PIO0`       | [crate::Gp11Pio0]           |
    /// | `PIO1`       | [crate::Gp11Pio1]           |
    Gpio11 {
        name: gpio11,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio11].
            FunctionUart, PullNone: Gp11Uart1Rts,
            /// SPI Function alias for pin [crate::Pins::gpio11].
            FunctionSpi, PullNone: Gp11Spi1Tx,
            /// I2C Function alias for pin [crate::Pins::gpio11].
            FunctionI2C, PullUp: Gp11I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::gpio11].
            FunctionPwm, PullNone: Gp11Pwm5B,
            /// PIO0 Function alias for pin [crate::Pins::gpio11].
            FunctionPio0, PullNone: Gp11Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio11].
            FunctionPio1, PullNone: Gp11Pio1
        }
    },

    /// GPIO 12 is connected to the left encoder
    Gpio12 {
        name: gpio12,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio12].
            FunctionPio0, PullNone: EncoderLeftA
        }
    },

    /// GPIO 13 is connected to the left encoder
    Gpio13 {
        name: gpio13,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio13].
            FunctionPio0, PullNone: EncoderLeftB
        }
    },

    /// GPIO 14 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 SCK`   | [crate::Gp14Spi1Sck]        |
    /// | `UART0 CTS`  | [crate::Gp14Uart0Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp14I2C1Sda]        |
    /// | `PWM7 A`     | [crate::Gp14Pwm7A]          |
    /// | `PIO0`       | [crate::Gp14Pio0]           |
    /// | `PIO1`       | [crate::Gp14Pio1]           |
    Gpio14 {
        name: gpio14,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio14].
            FunctionUart, PullNone: Gp14Uart0Cts,
            /// SPI Function alias for pin [crate::Pins::gpio14].
            FunctionSpi, PullNone: Gp14Spi1Sck,
            /// I2C Function alias for pin [crate::Pins::gpio14].
            FunctionI2C, PullUp: Gp14I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio14].
            FunctionPwm, PullNone: Gp14Pwm7A,
            /// PIO0 Function alias for pin [crate::Pins::gpio14].
            FunctionPio0, PullNone: Gp14Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio14].
            FunctionPio1, PullNone: Gp14Pio1
        }
    },

    /// GPIO 15 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 TX`    | [crate::Gp15Spi1Tx]         |
    /// | `UART0 RTS`  | [crate::Gp15Uart0Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp15I2C1Scl]        |
    /// | `PWM7 B`     | [crate::Gp15Pwm7B]          |
    /// | `PIO0`       | [crate::Gp15Pio0]           |
    /// | `PIO1`       | [crate::Gp15Pio1]           |
    Gpio15 {
        name: gpio15,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio15].
            FunctionUart, PullNone: Gp15Uart0Rts,
            /// SPI Function alias for pin [crate::Pins::gpio15].
            FunctionSpi, PullNone: Gp15Spi1Tx,
            /// I2C Function alias for pin [crate::Pins::gpio15].
            FunctionI2C, PullUp: Gp15I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::gpio15].
            FunctionPwm, PullNone: Gp15Pwm7B,
            /// PIO0 Function alias for pin [crate::Pins::gpio15].
            FunctionPio0, PullNone: Gp15Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio15].
            FunctionPio1, PullNone: Gp15Pio1
        }
    },

    /// GPIO 16 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp16Spi0Rx]         |
    /// | `UART0 TX`   | [crate::Gp16Uart0Tx]        |
    /// | `I2C0 SDA`   | [crate::Gp16I2C0Sda]        |
    /// | `PWM0 A`     | [crate::Gp16Pwm0A]          |
    /// | `PIO0`       | [crate::Gp16Pio0]           |
    /// | `PIO1`       | [crate::Gp16Pio1]           |
    Gpio16 {
        name: gpio16,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio16].
            FunctionUart, PullNone: Gp16Uart0Tx,
            /// SPI Function alias for pin [crate::Pins::gpio16].
            FunctionSpi, PullNone: Gp16Spi0Rx,
            /// I2C Function alias for pin [crate::Pins::gpio16].
            FunctionI2C, PullUp: Gp16I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::gpio16].
            FunctionPwm, PullNone: Gp16Pwm0A,
            /// PIO0 Function alias for pin [crate::Pins::gpio16].
            FunctionPio0, PullNone: Gp16Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio16].
            FunctionPio1, PullNone: Gp16Pio1
        }
    },

    /// GPIO 17 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 CSn`   | [crate::Gp17Spi0Csn]        |
    /// | `UART0 RX`   | [crate::Gp17Uart0Rx]        |
    /// | `I2C0 SCL`   | [crate::Gp17I2C0Scl]        |
    /// | `PWM0 B`     | [crate::Gp17Pwm0B]          |
    /// | `PIO0`       | [crate::Gp17Pio0]           |
    /// | `PIO1`       | [crate::Gp17Pio1]           |
    Gpio17 {
        name: gpio17,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio17].
            FunctionUart, PullNone: Gp17Uart0Rx,
            /// SPI Function alias for pin [crate::Pins::gpio17].
            FunctionSpi, PullNone: Gp17Spi0Csn,
            /// I2C Function alias for pin [crate::Pins::gpio17].
            FunctionI2C, PullUp: Gp17I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio17].
            FunctionPwm, PullNone: Gp17Pwm0B,
            /// PIO0 Function alias for pin [crate::Pins::gpio17].
            FunctionPio0, PullNone: Gp17Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio17].
            FunctionPio1, PullNone: Gp17Pio1
        }
    },

    /// GPIO 18 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 SCK`   | [crate::Gp18Spi0Sck]        |
    /// | `UART0 CTS`  | [crate::Gp18Uart0Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp18I2C1Sda]        |
    /// | `PWM1 A`     | [crate::Gp18Pwm1A]          |
    /// | `PIO0`       | [crate::Gp18Pio0]           |
    /// | `PIO1`       | [crate::Gp18Pio1]           |
    Gpio18 {
        name: gpio18,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio18].
            FunctionUart, PullNone: Gp18Uart0Cts,
            /// SPI Function alias for pin [crate::Pins::gpio18].
            FunctionSpi, PullNone: Gp18Spi0Sck,
            /// I2C Function alias for pin [crate::Pins::gpio18].
            FunctionI2C, PullUp: Gp18I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio18].
            FunctionPwm, PullNone: Gp18Pwm1A,
            /// PIO0 Function alias for pin [crate::Pins::gpio18].
            FunctionPio0, PullNone: Gp18Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio18].
            FunctionPio1, PullNone: Gp18Pio1
        }
    },

    /// GPIO 19 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`    | [crate::Gp19Spi0Tx]         |
    /// | `UART0 RTS`  | [crate::Gp19Uart0Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp19I2C1Scl]        |
    /// | `PWM1 B`     | [crate::Gp19Pwm1B]          |
    /// | `PIO0`       | [crate::Gp19Pio0]           |
    /// | `PIO1`       | [crate::Gp19Pio1]           |
    Gpio19 {
        name: gpio19,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio19].
            FunctionUart, PullNone: Gp19Uart0Rts,
            /// SPI Function alias for pin [crate::Pins::gpio19].
            FunctionSpi, PullNone: Gp19Spi0Tx,
            /// I2C Function alias for pin [crate::Pins::gpio19].
            FunctionI2C, PullUp: Gp19I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::gpio19].
            FunctionPwm, PullNone: Gp19Pwm1B,
            /// PIO0 Function alias for pin [crate::Pins::gpio19].
            FunctionPio0, PullNone: Gp19Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio19].
            FunctionPio1, PullNone: Gp19Pio1
        }
    },

    /// GPIO 20 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp20Spi0Rx]         |
    /// | `UART1 TX`   | [crate::Gp20Uart1Tx]        |
    /// | `I2C0 SDA`   | [crate::Gp20I2C0Sda]        |
    /// | `PWM2 A`     | [crate::Gp20Pwm2A]          |
    /// | `PIO0`       | [crate::Gp20Pio0]           |
    /// | `PIO1`       | [crate::Gp20Pio1]           |
    Gpio20 {
        name: gpio20,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio20].
            FunctionUart, PullNone: Gp20Uart1Tx,
            /// SPI Function alias for pin [crate::Pins::gpio20].
            FunctionSpi, PullNone: Gp20Spi0Rx,
            /// I2C Function alias for pin [crate::Pins::gpio20].
            FunctionI2C, PullUp: Gp20I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::gpio20].
            FunctionPwm, PullNone: Gp20Pwm2A,
            /// PIO0 Function alias for pin [crate::Pins::gpio20].
            FunctionPio0, PullNone: Gp20Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio20].
            FunctionPio1, PullNone: Gp20Pio1
        }
    },

    /// GPIO 21 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 CSn`   | [crate::Gp21Spi0Csn]        |
    /// | `UART1 RX`   | [crate::Gp21Uart1Rx]        |
    /// | `I2C0 SCL`   | [crate::Gp21I2C0Scl]        |
    /// | `PWM2 B`     | [crate::Gp21Pwm2B]          |
    /// | `PIO0`       | [crate::Gp21Pio0]           |
    /// | `PIO1`       | [crate::Gp21Pio1]           |
    Gpio21 {
        name: gpio21,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio21].
            FunctionUart, PullNone: Gp21Uart1Rx,
            /// SPI Function alias for pin [crate::Pins::gpio21].
            FunctionSpi, PullNone: Gp21Spi0Csn,
            /// I2C Function alias for pin [crate::Pins::gpio21].
            FunctionI2C, PullUp: Gp21I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio21].
            FunctionPwm, PullNone: Gp21Pwm2B,
            /// PIO0 Function alias for pin [crate::Pins::gpio21].
            FunctionPio0, PullNone: Gp21Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio21].
            FunctionPio1, PullNone: Gp21Pio1
        }
    },

    /// GPIO 22 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 SCK`   | [crate::Gp22Spi0Sck]        |
    /// | `UART1 CTS`  | [crate::Gp22Uart1Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp22I2C1Sda]        |
    /// | `PWM3 A`     | [crate::Gp22Pwm3A]          |
    /// | `PIO0`       | [crate::Gp22Pio0]           |
    /// | `PIO1`       | [crate::Gp22Pio1]           |
    Gpio22 {
        name: gpio22,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio22].
            FunctionUart, PullNone: Gp22Uart1Cts,
            /// SPI Function alias for pin [crate::Pins::gpio22].
            FunctionSpi, PullNone: Gp22Spi0Sck,
            /// I2C Function alias for pin [crate::Pins::gpio22].
            FunctionI2C, PullUp: Gp22I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio22].
            FunctionPwm, PullNone: Gp22Pwm3A,
            /// PIO0 Function alias for pin [crate::Pins::gpio22].
            FunctionPio0, PullNone: Gp22Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio22].
            FunctionPio1, PullNone: Gp22Pio1
        }
    },

    /// GPIO 23 is connected to b_power_save of the Raspberry Pi Pico board.
    Gpio23 {
        name: b_power_save,
    },

    /// GPIO 24 is connected to vbus_detect of the Raspberry Pi Pico board.
    Gpio24 {
        name: vbus_detect,
    },

    /// GPIO 25 is connected to led of the Raspberry Pi Pico board.
    Gpio25 {
        name: led,
    },

    /// GPIO 26 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 SCK`   | [crate::Gp26Spi1Sck]        |
    /// | `UART1 CTS`  | [crate::Gp26Uart1Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp26I2C1Sda]        |
    /// | `PWM5 A`     | [crate::Gp26Pwm5A]          |
    /// | `PIO0`       | [crate::Gp26Pio0]           |
    /// | `PIO1`       | [crate::Gp26Pio1]           |
    Gpio26 {
        name: gpio26,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio26].
            FunctionUart, PullNone: Gp26Uart1Cts,
            /// SPI Function alias for pin [crate::Pins::gpio26].
            FunctionSpi, PullNone: Gp26Spi1Sck,
            /// I2C Function alias for pin [crate::Pins::gpio26].
            FunctionI2C, PullUp: Gp26I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio26].
            FunctionPwm, PullNone: Gp26Pwm5A,
            /// PIO0 Function alias for pin [crate::Pins::gpio26].
            FunctionPio0, PullNone: Gp26Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio26].
            FunctionPio1, PullNone: Gp26Pio1
        }
    },

    /// GPIO 27 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 TX`    | [crate::Gp27Spi1Tx]         |
    /// | `UART1 RTS`  | [crate::Gp27Uart1Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp27I2C1Scl]        |
    /// | `PWM5 B`     | [crate::Gp27Pwm5B]          |
    /// | `PIO0`       | [crate::Gp27Pio0]           |
    /// | `PIO1`       | [crate::Gp27Pio1]           |
    Gpio27 {
        name: gpio27,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio27].
            FunctionUart, PullNone: Gp27Uart1Rts,
            /// SPI Function alias for pin [crate::Pins::gpio27].
            FunctionSpi, PullNone: Gp27Spi1Tx,
            /// I2C Function alias for pin [crate::Pins::gpio27].
            FunctionI2C, PullUp: Gp27I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::gpio27].
            FunctionPwm, PullNone: Gp27Pwm5B,
            /// PIO0 Function alias for pin [crate::Pins::gpio27].
            FunctionPio0, PullNone: Gp27Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio27].
            FunctionPio1, PullNone: Gp27Pio1
        }
    },

    /// GPIO 28 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 RX`    | [crate::Gp28Spi1Rx]         |
    /// | `UART0 TX`   | [crate::Gp28Uart0Tx]        |
    /// | `I2C0 SDA`   | [crate::Gp28I2C0Sda]        |
    /// | `PWM6 A`     | [crate::Gp28Pwm6A]          |
    /// | `PIO0`       | [crate::Gp28Pio0]           |
    /// | `PIO1`       | [crate::Gp28Pio1]           |
    Gpio28 {
        name: gpio28,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio28].
            FunctionUart, PullNone: Gp28Uart0Tx,
            /// SPI Function alias for pin [crate::Pins::gpio28].
            FunctionSpi, PullNone: Gp28Spi1Rx,
            /// I2C Function alias for pin [crate::Pins::gpio28].
            FunctionI2C, PullUp: Gp28I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::gpio28].
            FunctionPwm, PullNone: Gp28Pwm6A,
            /// PIO0 Function alias for pin [crate::Pins::gpio28].
            FunctionPio0, PullNone: Gp28Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio28].
            FunctionPio1, PullNone: Gp28Pio1
        }
    },

    /// GPIO 29 is connected to voltage_monitor of the Raspberry Pi Pico board.
    Gpio29 {
        name: voltage_monitor,
    },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

pub struct ThreePiPlus2040<'a> {
    pub visual_output: VisualOutput,
    pub magnetometer: Lis3mdl<I2CProxy<'a>>,
    pub accelerometer: Lsm6dso<I2CProxy<'a>>,
    pub i2c0bus: &'static shared_bus::BusManagerCortexM<ConfiguredI2C0>,
    pub encoder_right: Rx<(pac::PIO0, hal::pio::SM0)>,
    pub encoder_left: Rx<(pac::PIO0, hal::pio::SM1)>,
}

type ConfiguredI2C0 = I2C<
    I2C0,
    (
        hal::gpio::Pin<hal::gpio::bank0::Gpio4, hal::gpio::FunctionI2c, hal::gpio::PullUp>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::FunctionI2c, hal::gpio::PullUp>,
    ),
>;
type I2CProxy<'a> = shared_bus::I2cProxy<'a, shared_bus::CortexMMutex<ConfiguredI2C0>>;

impl ThreePiPlus2040<'_> {
    pub fn new<Delay, F, SystemF>(
        io: pac::IO_BANK0,
        pads: pac::PADS_BANK0,
        sio: hal::sio::SioGpioBank0,
        spi0: pac::SPI0,
        i2c0: pac::I2C0,
        pio0: pac::PIO0,
        freq: F,
        system_clock: SystemF,
        resets: &mut pac::RESETS,
        delay: &mut Delay,
    ) -> Self
    where
        Delay: DelayMs<u8>,
        F: Into<HertzU32>,
        SystemF: Into<HertzU32>,
    {
        let internal_pins = Pins::new(io, pads, sio, resets);
        let visual_output = VisualOutput::new(
            spi0,
            internal_pins.gpio3.reconfigure(),
            internal_pins.gpio2.reconfigure(),
            internal_pins.gpio0.reconfigure(),
            internal_pins.gpio1.reconfigure(),
            internal_pins.gpio6.reconfigure(),
            resets,
            delay,
        );

        let i2c = I2C::i2c0(
            i2c0,
            internal_pins.gpio4.reconfigure(),
            internal_pins.gpio5.reconfigure(),
            freq,
            resets,
            system_clock,
        );
        let i2c0_bus_manager: &'static _ = shared_bus::new_cortexm!(ConfiguredI2C0 = i2c).unwrap();

        let lis3mdl =
            Lis3mdl::new(i2c0_bus_manager.acquire_i2c(), lis3mdl::Address::Addr1E).unwrap();
        let lsm6dso = Lsm6dso::new(i2c0_bus_manager.acquire_i2c(), 107).unwrap();

        // Set PIO for encoders
        let encoder_program = pio_proc::pio_file!("src/quadrature_encoder.pio");

        // Initialize and start PIO
        let (mut pio, sm0, sm1, _, _) = pio0.split(resets);
        let installed = pio.install(&encoder_program.program).unwrap();

        let (encoder_right_a, _encoder_right_b): (EncoderRightA, EncoderRightB) = (
            internal_pins.gpio8.reconfigure(),
            internal_pins.gpio9.reconfigure(),
        );
        let encoder_right =
            init_encoder_pio(encoder_right_a.id().num, sm0, unsafe { installed.share() });

        let (encoder_left_a, _encoder_left_b): (EncoderLeftA, EncoderLeftB) = (
            internal_pins.gpio12.reconfigure(),
            internal_pins.gpio13.reconfigure(),
        );
        let encoder_left = init_encoder_pio(encoder_left_a.id().num, sm1, installed);

        Self {
            visual_output,
            magnetometer: lis3mdl,
            accelerometer: lsm6dso,
            i2c0bus: i2c0_bus_manager,
            encoder_right,
            encoder_left,
        }
    }

    pub fn is_button_c_pressed(&mut self) -> bool {
        self.visual_output.read_gpio0()
    }
}

fn init_encoder_pio<T: hal::pio::StateMachineIndex>(
    pin_number: u8,
    sm: UninitStateMachine<(pac::PIO0, T)>,
    installed: hal::pio::InstalledProgram<pac::PIO0>,
) -> Rx<(pac::PIO0, T)> {
    let (mut sm, rx, _) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .in_shift_direction(hal::pio::ShiftDirection::Left)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .jmp_pin(pin_number)
        .in_pin_base(pin_number)
        .clock_divisor_fixed_point(16, 0)
        .autopull(false)
        .autopush(false)
        .build(sm);

    sm.set_pindirs([
        (pin_number, hal::pio::PinDir::Input),
        (pin_number + 1, hal::pio::PinDir::Input),
    ]);

    sm.start();
    rx
}
