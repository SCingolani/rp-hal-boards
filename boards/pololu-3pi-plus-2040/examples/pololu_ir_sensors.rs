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

use embedded_graphics::geometry::Size;
use embedded_graphics::primitives::Primitive;
use embedded_graphics::primitives::PrimitiveStyleBuilder;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::primitives::RoundedRectangle;
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
use embedded_graphics::{pixelcolor::BinaryColor, prelude::Point, Drawable};

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

    // We can have as many `CountDown`s as we want with one timer.
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut display_refresh_countdown = timer.count_down();

    // Two ways of creating duration constants:
    display_refresh_countdown.start(MicrosDurationU32::millis(20));

    let display = mouse.visual_output.as_display(&mut pac.RESETS);
    let mut ir_sensors = mouse.ir_sensors;

    let gfx_stroke = PrimitiveStyleBuilder::new()
        .stroke_width(2)
        .stroke_color(BinaryColor::On)
        .fill_color(BinaryColor::Off)
        .build();
    let gfx_fill = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::On)
        .build();

    const SPACING: u32 = 20;
    const PADDING: u32 = 3;
    let left_margin: u32 = (display.get_dimensions().0 as u32 - SPACING * 5) / 2;
    let height = (display.get_dimensions().1 - 10) as u32;
    loop {
        let mut text = heapless::String::<64>::new();
        if display_refresh_countdown.wait().is_ok() {
            let before = timer.get_counter();
            let output = ir_sensors.measure_downward_facing(&mut delay);
            let after = timer.get_counter();
            write!(text, "dt: {}\n", after - before).unwrap();
            // Clear the internal buffer (does nothing to the display yet)
            display.clear();

            for i in 0..5_u32 {
                RoundedRectangle::with_equal_corners(
                    Rectangle::new(
                        Point::new((left_margin + SPACING * i + PADDING) as i32, 5),
                        Size::new(SPACING - PADDING * 2, height),
                    ),
                    Size::new(3, 3),
                )
                .into_styled(gfx_stroke)
                .draw(display)
                .unwrap();
                Rectangle::new(
                    Point::new((left_margin + SPACING * i + PADDING) as i32, 5),
                    Size::new(
                        SPACING - PADDING * 2,
                        height * (output[i as usize] as u32)
                            / (pololu::ir_sensors::MAX_VALUE as u32),
                    ),
                )
                .into_styled(gfx_fill)
                .draw(display)
                .unwrap();
            }

            // Text::new(&text, Point::new(0, 10), style)
            //     .draw(display)
            //     .unwrap();

            display.flush().unwrap();
        }
    }
}
