#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::Io, i2c::I2C, peripherals::Peripherals, prelude::*,
    system::SystemControl,
};
use hd44780_driver::{Cursor, CursorBlink, Display, DisplayMode, HD44780};

const LCD_ADDR: u8 = 0x27;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    esp_println::logger::init_logger_from_env();

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio8,
        io.pins.gpio9,
        400.kHz(),
        &clocks,
    );

    let mut lcd = HD44780::new_i2c(i2c, LCD_ADDR, &mut delay).unwrap();
    let _ = lcd.reset(&mut delay);
    let _ = lcd.clear(&mut delay);
    let _ = lcd.set_display_mode(
        DisplayMode {
            display: Display::On,
            cursor_visibility: Cursor::Visible,
            cursor_blink: CursorBlink::On,
        },
        &mut delay,
    );
    let _ = lcd.write_str("Hello, ESP32C3!", &mut delay);

    loop {
        log::info!("Hello world!");
        delay.delay(500.millis());
    }
}
