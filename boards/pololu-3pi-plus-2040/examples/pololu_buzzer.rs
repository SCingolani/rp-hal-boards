//! # Pico Blinky Example
//!
//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use embedded_hal::PwmPin;
use fugit::MicrosDurationU32;
use fugit::RateExtU32;

// Shorten the name of our board support package
use pololu_3pi_plus_2040 as pololu;

// The macro for our start-up function
use pololu::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use pololu::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pololu::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use pololu::hal;

// We will use some timers to have cyclic tasks on our main loop
use embedded_hal::timer::CountDown;

// Abstraction to display graphics and text
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::Text,
    Drawable,
};

// Formatted strings:
use core::fmt::Write;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        pololu::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds). It is needed for the display driver since it has to wait for the display to initialize.
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Init PWMs
    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // The main instance of our board support devices.
    let mut mouse = pololu::ThreePiPlus2040::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        pac.SPI0,
        pac.I2C0,
        pac.PIO0,
        pac.PIO1,
        pwm_slices.pwm7,
        400_u32.kHz(),
        clocks.system_clock.freq(),
        &mut pac.RESETS,
        &mut delay,
    );

    // Create a new character style
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    // We can have as many `CountDown`s as we want with one timer.
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut display_refresh_countdown = timer.count_down();
    let mut buzzer_countdown = timer.count_down();

    // Two ways of creating duration constants:
    display_refresh_countdown.start(MicrosDurationU32::millis(1));
    buzzer_countdown.start(MicrosDurationU32::millis(100));

    let display = mouse.visual_output.as_display(&mut pac.RESETS);
    let mut encoder_measurement_right_last = 0;
    let mut encoder_measurement_left_last = 0;

    // Configure PWM3
    let mut pwm = pwm_slices.pwm3;
    pwm.set_ph_correct();
    pwm.enable();

    pwm.channel_b.output_to(mouse.buzzer);

    let mut freq: u32 = 440;
    loop {
        if buzzer_countdown.wait().is_ok() {
            pwm.channel_b.set_duty(0);
        }
        if display_refresh_countdown.wait().is_ok() {
            let mut encoder_measurement_right = 0;
            // empty the FIFO
            while let Some(encoder_value) = mouse.encoder_right.read() {
                encoder_measurement_right = encoder_value as i32;
            }
            let mut encoder_measurement_left = 0;
            // empty the FIFO
            while let Some(encoder_value) = mouse.encoder_left.read() {
                encoder_measurement_left = encoder_value as i32;
            }

            if encoder_measurement_right_last != encoder_measurement_right
                || encoder_measurement_left_last != encoder_measurement_left
            {
                encoder_measurement_right_last = encoder_measurement_right;
                encoder_measurement_left_last = encoder_measurement_left;
                // Clear the internal buffer (does nothing to the display yet)
                display.clear();
                // Create a text at position (20, 30) and draw it using the previously defined style
                let mut text = heapless::String::<64>::new();
                write!(
                    text,
                    "Encoder right: {}\nEncoder left:  {}",
                    encoder_measurement_right, encoder_measurement_left,
                )
                .unwrap();
                Text::new(&text, Point::new(0, 32), style)
                    .draw(display)
                    .unwrap();

                display.flush().unwrap();
                freq += 10;
                if freq > 6000 {
                    freq = 440;
                }
                pololu::buzzer::set_freq(&mut pwm, freq);
                pwm.channel_b.set_duty(pwm.channel_b.get_max_duty() / 2);
            }
        }
    }
}
