//! OLED Display and RGB LEDs output Because they share the same SPI bus, but with different
//! configurations, a single instance is given which can be converted into either the display
//! interface or the RGB LED interface.

use core::{convert::Infallible, mem::MaybeUninit};

use apa102_spi::Apa102;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::{
    pac::{self, RESETS},
    Spi,
};
use sh1106::mode::displaymode::DisplayModeTrait;
use sh1106::NoOutputPin;

use crate::{
    ButtonC, DisplayDC, DisplayMOSI, DisplayReset, DisplaySCK, DisplaySckNoDrive, RgbLedData,
    RgbLedSck, RgbLedSckNoDrive,
};

type DisplaySpi = Spi<hal::spi::Enabled, pac::SPI0, (DisplayMOSI, DisplaySCK), 8>;
type RgbLedSpi = Spi<hal::spi::Enabled, pac::SPI0, (RgbLedData, RgbLedSck), 8>;

const SPI_FREQ: fugit::HertzU32 = fugit::HertzU32::MHz(125);

const DISPLAY_SPI_BAUDRATE: fugit::HertzU32 = fugit::HertzU32::MHz(41);
const DISPLAY_SPI_FRAME_FORMAT: hal::spi::FrameFormat =
    hal::spi::FrameFormat::MotorolaSpi(embedded_hal::spi::MODE_0);

const RGBLED_SPI_BAUDRATE: fugit::HertzU32 = fugit::HertzU32::MHz(41);
const RGBLED_SPI_FRAME_FORMAT: hal::spi::FrameFormat =
    hal::spi::FrameFormat::MotorolaSpi(embedded_hal::spi::MODE_0);

pub(crate) struct DisplayPins {
    pub sck: DisplaySckNoDrive,
    pub dc: DisplayDC,
}

pub struct VisualOutput {
    inner: EitherDisplayOrRgbLed,
    display_pins: Option<DisplayPins>,
    rgbled_sck: Option<RgbLedSckNoDrive>,
    display_reset: DisplayReset,
}

type Display = sh1106::mode::GraphicsMode<
    sh1106::interface::SpiInterface<DisplaySpi, DisplayDC, sh1106::NoOutputPin<Infallible>>,
>;

enum EitherDisplayOrRgbLed {
    Display(Display),
    RgbLed(Apa102<RgbLedSpi>),
}

impl VisualOutput {
    pub(crate) fn new<Delay: DelayMs<u8>>(
        spi: pac::SPI0,
        data_pin: DisplayMOSI,
        display_sck: DisplaySCK,
        display_dc: DisplayDC,
        mut display_reset: DisplayReset,
        mut rgbled_sck: RgbLedSckNoDrive,
        resets: &mut RESETS,
        delay: &mut Delay,
    ) -> Self {
        let spi = Spi::new(spi, (data_pin, display_sck.reconfigure())).init(
            resets,
            SPI_FREQ,
            DISPLAY_SPI_BAUDRATE,
            DISPLAY_SPI_FRAME_FORMAT,
        );
        let mut display: Display = sh1106::Builder::new()
            .connect_spi(spi, display_dc, NoOutputPin::new())
            .into();
        display.reset(&mut display_reset, delay).unwrap();
        display.init().unwrap();
        let display = EitherDisplayOrRgbLed::Display(display);
        rgbled_sck.set_low().unwrap();
        Self {
            inner: display,
            display_pins: None,
            rgbled_sck: Some(rgbled_sck),
            display_reset,
        }
    }

    /// Get the display driver. Might internally reconfigure pins and Spi if last visual output used was the RGB LED.
    pub fn as_display(&mut self, resets: &mut RESETS) -> &mut Display {
        if !self.inner.is_display() {
            // leave an uninit inside inner temporarily so that we can take ownership in prev
            let inner: &mut MaybeUninit<EitherDisplayOrRgbLed> =
                unsafe { core::mem::transmute(&mut self.inner) };
            let prev = core::mem::replace(inner, MaybeUninit::uninit());
            // now that we have ownership, we update it (we know it is init because we took it from self.inner)
            let (next, rgbled_sck) = unsafe { prev.assume_init() }
                .into_display(self.display_pins.take().unwrap(), resets);
            // finally, replace back the new value in our inner
            let _ = core::mem::replace(inner, MaybeUninit::new(next));
            // and put back the pins
            self.rgbled_sck = Some(rgbled_sck);
        }
        let EitherDisplayOrRgbLed::Display(ref mut display) = self.inner else {
            unreachable!()
        };
        display
    }

    pub fn into_display(self, resets: &mut RESETS) -> Self {
        if !self.inner.is_display() {
            let display_pins = self.display_pins.unwrap();
            let (new_inner, rgbpins) = self.inner.into_display(display_pins, resets);
            Self {
                inner: new_inner,
                display_pins: None,
                rgbled_sck: Some(rgbpins),
                display_reset: self.display_reset,
            }
        } else {
            self
        }
    }

    /// Get the rgbled driver. Might internally reconfigure pins and Spi if last visual output used was the Display.
    pub fn as_rgbled(&mut self, resets: &mut RESETS) -> &mut Apa102<RgbLedSpi> {
        if !self.inner.is_rgbled() {
            // leave an uninit inside inner temporarily so that we can take ownership in prev
            let inner: &mut MaybeUninit<EitherDisplayOrRgbLed> =
                unsafe { core::mem::transmute(&mut self.inner) };
            let prev = core::mem::replace(inner, MaybeUninit::uninit());
            // now that we have ownership, we update it (we know it is init because we took it from self.inner)
            let (next, display_pins) = unsafe { prev.assume_init() }
                .into_rgbled(self.rgbled_sck.take().unwrap().reconfigure(), resets);
            // finally, replace back the new value in our inner
            let _ = core::mem::replace(inner, MaybeUninit::new(next));
            // and put back the pins
            self.display_pins = Some(display_pins);
        }
        let EitherDisplayOrRgbLed::RgbLed(ref mut rgbled) = self.inner else {
            unreachable!()
        };
        rgbled
    }

    pub(crate) fn read_gpio0(&mut self) -> bool {
        if self.inner.is_display() {
            let inner: &mut MaybeUninit<EitherDisplayOrRgbLed> =
                unsafe { core::mem::transmute(&mut self.inner) };
            let prev = core::mem::replace(inner, MaybeUninit::uninit());
            let EitherDisplayOrRgbLed::Display(display) = (unsafe { prev.assume_init() }) else {
                unreachable!()
            };
            // temporarily release the display interface so we can own the gpio0 pin
            let (spi, display_dc, _) = display.release().release_interface();

            let button_c: ButtonC = display_dc.reconfigure();
            let result = button_c.is_high().unwrap();

            // return the display
            let display = EitherDisplayOrRgbLed::Display(
                sh1106::Builder::new()
                    .connect_spi(spi, button_c.reconfigure(), NoOutputPin::new())
                    .into(),
            );
            let _ = core::mem::replace(inner, MaybeUninit::new(display));

            result
        } else {
            let DisplayPins { sck, dc } = self.display_pins.take().unwrap();
            let button_c: ButtonC = dc.reconfigure();
            let result = button_c.is_high().unwrap();

            let dc = button_c.reconfigure();
            self.display_pins = Some(DisplayPins { sck, dc });

            result
        }
    }
}

impl EitherDisplayOrRgbLed {
    fn is_display(&self) -> bool {
        match self {
            EitherDisplayOrRgbLed::Display(_) => true,
            EitherDisplayOrRgbLed::RgbLed(_) => false,
        }
    }

    fn is_rgbled(&self) -> bool {
        !self.is_display()
    }

    fn into_display(
        self,
        display_pins: DisplayPins,
        resets: &mut RESETS,
    ) -> (Self, RgbLedSckNoDrive) {
        if let EitherDisplayOrRgbLed::RgbLed(rgbled) = self {
            let DisplayPins { sck, dc } = display_pins;
            let spi = rgbled.free();
            let (spi, spi_pins) = spi.disable().free();
            let spi = Spi::new(spi, (spi_pins.0, sck.reconfigure())).init(
                resets,
                SPI_FREQ,
                DISPLAY_SPI_BAUDRATE,
                DISPLAY_SPI_FRAME_FORMAT,
            );
            let mut sck = spi_pins.1.reconfigure();
            sck.set_low().unwrap();
            (
                Self::Display(
                    sh1106::Builder::new()
                        .connect_spi(spi, dc, NoOutputPin::new())
                        .into(),
                ),
                sck,
            )
        } else {
            panic!("Tried to make a EitherDisplayOrRgbLed into a display when it already was");
        }
    }

    fn into_rgbled(self, rgbled_sck: RgbLedSck, resets: &mut RESETS) -> (Self, DisplayPins) {
        if let EitherDisplayOrRgbLed::Display(display) = self {
            let (spi, dc, _) = display.release().release_interface();
            let (spi, spi_pins) = spi.disable().free();
            let spi = Spi::new(spi, (spi_pins.0, rgbled_sck)).init(
                resets,
                SPI_FREQ,
                RGBLED_SPI_BAUDRATE,
                RGBLED_SPI_FRAME_FORMAT,
            );
            let mut sck = spi_pins.1.reconfigure();
            sck.set_low().unwrap();
            (
                Self::RgbLed(Apa102::new_with_options(
                    spi,
                    1,
                    false,
                    apa102_spi::PixelOrder::BGR,
                )),
                DisplayPins { sck, dc },
            )
        } else {
            panic!("Tried to make a EitherDisplayOrRgbLed into a display when it already was");
        }
    }
}

// impl Display {
//     fn release(self) -> (DisplaySpi, DisplayDC, DisplayReset) {
//         let (spi, dc, _) = self.0.release().release_interface();
//         (spi, dc, self.1)
//     }

//     pub fn reset<Delay: DelayMs<u8>>(&mut self, delay: &mut Delay) {
//         self.0.reset(&mut self.1, delay).unwrap()
//     }
// }
