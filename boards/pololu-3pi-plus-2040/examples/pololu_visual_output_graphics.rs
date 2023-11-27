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

use embedded_graphics::geometry::OriginDimensions;
use embedded_graphics::image::Image;
use embedded_graphics::image::ImageRawLE;
use embedded_graphics::transform::Transform;
use fugit::ExtU32;
use fugit::MicrosDurationU32;
use fugit::RateExtU32;
use hal::timer::Instant;

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

// Abstraction to use RGB leds
use smart_leds::brightness;
use smart_leds::hsv::Hsv;
use smart_leds::{gamma, hsv::hsv2rgb, SmartLedsWrite, RGB8};

// Abstraction to display graphics and text
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::Point,
    Drawable,
};

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

    // The main instance of our board support devices.
    let mut mouse = pololu::ThreePiPlus2040::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        pac.SPI0,
        pac.I2C0,
        pac.PIO0,
        400_u32.kHz(),
        clocks.system_clock.freq(),
        &mut pac.RESETS,
        &mut delay,
    );

    // We can have as many `CountDown`s as we want with one timer.
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut display_refresh_countdown = timer.count_down();
    let mut led_refresh_countdown = timer.count_down();

    // Two ways of creating duration constants:
    display_refresh_countdown.start(MicrosDurationU32::millis(20));
    led_refresh_countdown.start(1000u32.micros());

    // Some state to do a nice animation with the LEDs
    let mut led_state = LedState::default();

    let raw_im: ImageRawLE<BinaryColor> = ImageRawLE::new(include_bytes!("./rust.raw"), 64);

    let im = Image::new(&raw_im, Point::new(0, 0));

    let mut position: i32 = 0;
    let mut speed: i32 = 1;

    loop {
        let now = timer.get_counter();

        if display_refresh_countdown.wait().is_ok() {
            // Get a reference to the Display driver; it is mutually exclusive with the RGB LED!
            // Switching from Display to RGB and vice versa incurs a small cost
            let display = mouse.visual_output.as_display(&mut pac.RESETS);

            // Clear the internal buffer (does nothing to the display yet)
            display.clear();

            im.translate(Point { x: position, y: 0 })
                .draw(display)
                .unwrap();

            position += speed;
            if position + raw_im.size().width as i32 > display.get_dimensions().0 as i32 {
                position = display.get_dimensions().0 as i32 - raw_im.size().width as i32;
                speed = -speed;
            } else if position < 0 {
                position = 0;
                speed = -speed;
            }

            display.flush().unwrap();
        }

        if led_refresh_countdown.wait().is_ok() {
            let color: RGB8 = hsv2rgb(Hsv {
                hue: led_state.hue,
                sat: 255,
                val: 255,
            });

            let colors = led_state.brightness.map(|brightness| {
                let brightness = (brightness / 256).clamp(256 / 3, 255);
                RGB8 {
                    r: (color.r as u16 * (brightness + 1) / 256) as u8,
                    g: (color.g as u16 * (brightness + 1) / 256) as u8,
                    b: (color.b as u16 * (brightness + 1) / 256) as u8,
                }
            });

            // Get a reference to the RGB Led driver; it is mutually exclusive with the  Display!
            // Switching from Display to RGB and vice versa incurs a small cost
            let rgbleds = mouse.visual_output.as_rgbled(&mut pac.RESETS);

            // write out the colors, with a scaled down brightness (the LEDs are very bright!) and gamma corrected
            rgbleds
                .write(gamma(brightness(colors.iter().cloned(), 255 / 4 * 3)))
                .unwrap();

            led_state.update(now);
        }
    }
}

/// Little animation for the LEDs
#[derive(Default)]
struct LedState {
    /// Hue of the next color
    hue: u8,
    brightness: [u16; 6],
    counter: u8,
}

impl LedState {
    fn update(&mut self, now: Instant) {
        const ONE_SECOND_IN_MICROS: u64 = 1_000_000;
        // change the color
        self.counter = self.counter.wrapping_add(1);
        if self.counter == 0 {
            self.hue = self.hue.wrapping_add(1);
        }
        // change highlighted LED every 1/6 of a second.
        let hightlight_led_index = (now.duration_since_epoch().to_micros() % ONE_SECOND_IN_MICROS
            / (ONE_SECOND_IN_MICROS / 6)) as usize;
        for (i, b) in self.brightness.iter_mut().enumerate() {
            if i == hightlight_led_index {
                *b = u16::MAX
            } else {
                *b = b.saturating_sub(u16::MAX / (100 * 166 / 20))
            }
        }
    }
}
