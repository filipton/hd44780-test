#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{AnyOutput, Io},
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use hd44780_driver::{Cursor, CursorBlink, Display, DisplayMode, HD44780};

const BACKLIGHT_SHIFT: u8 = 0b1000_0000;
const RS_SHIFT: u8 = 0b0100_0000;
const E_SHIFT: u8 = 0b0010_0000;
const D4_SHIFT: u8 = 0b0000_1000;
const D5_SHIFT: u8 = 0b0000_0100;
const D6_SHIFT: u8 = 0b0000_0010;
const D7_SHIFT: u8 = 0b0000_0001;
const LCD_ADDR: u8 = 0x27;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut data_pin = AnyOutput::new(io.pins.gpio10, esp_hal::gpio::Level::Low);
    let mut clk_pin = AnyOutput::new(io.pins.gpio21, esp_hal::gpio::Level::Low);
    let mut latch_pin = AnyOutput::new(io.pins.gpio1, esp_hal::gpio::Level::Low);

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

    latch_pin.set_low();
    shift_out(&mut clk_pin, &mut data_pin, 0);
    shift_out(&mut clk_pin, &mut data_pin, 255);
    latch_pin.set_high();

    delay.delay_millis(1000);

    latch_pin.set_low();
    shift_out(&mut clk_pin, &mut data_pin, BACKLIGHT_SHIFT);
    shift_out(&mut clk_pin, &mut data_pin, 255);
    latch_pin.set_high();

    delay.delay_millis(1000);

    latch_pin.set_low();
    shift_out(&mut clk_pin, &mut data_pin, 0);
    shift_out(&mut clk_pin, &mut data_pin, 255);
    latch_pin.set_high();

    lcd_byte(0x33, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    lcd_byte(0x32, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    lcd_byte(0x28, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    lcd_byte(0x0C, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    lcd_byte(0x06, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    lcd_byte(0x01, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);

    delay.delay_millis(1000);

    lcd_byte(0x01, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    lcd_byte(0x80, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    lcd_byte(b'X', true, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    lcd_byte(0xC0, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    lcd_byte(b'D', true, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);

    loop {
        log::info!("Hello world!");
        delay.delay(500.millis());
    }
}

fn shift_out(clk_pin: &mut AnyOutput, data_pin: &mut AnyOutput, mut val: u8) {
    for _ in 0..8 {
        let level = esp_hal::gpio::Level::from(val & 1 > 0);
        data_pin.set_level(level);
        val >>= 1;

        clk_pin.set_high();
        clk_pin.set_low();
    }
}

const E_PULSE: u32 = 500; // microseconds
const E_DELAY: u32 = 500; // microseconds

/// `byte`: byte to be sent to LCD
/// `mode`: true for char, false for cmd
fn lcd_byte(
    byte: u8,
    mode: bool,
    clk_pin: &mut AnyOutput,
    data_pin: &mut AnyOutput,
    latch_pin: &mut AnyOutput,
    delay: &Delay,
) {
    // high nibble
    let mut tmp = match mode {
        true => RS_SHIFT,
        false => 0,
    };

    // TODO: make this better XD
    if byte & 0x10 != 0 {
        tmp += D4_SHIFT;
    }
    if byte & 0x20 != 0 {
        tmp += D5_SHIFT;
    }
    if byte & 0x40 != 0 {
        tmp += D6_SHIFT;
    }
    if byte & 0x80 != 0 {
        tmp += D7_SHIFT;
    }
    tmp += E_SHIFT;
    tmp += BACKLIGHT_SHIFT;

    delay.delay_micros(E_DELAY);
    latch_pin.set_low();
    shift_out(clk_pin, data_pin, tmp);
    shift_out(clk_pin, data_pin, 255); // FIRST SHIFTER (FOR BUTTONS)
    latch_pin.set_high();
    delay.delay_micros(E_PULSE);
    latch_pin.set_low();
    shift_out(clk_pin, data_pin, BACKLIGHT_SHIFT);
    shift_out(clk_pin, data_pin, 255); // FIRST SHIFTER (FOR BUTTONS)
    latch_pin.set_high();
    delay.delay_micros(E_DELAY);

    // low nibble
    let mut tmp = match mode {
        true => RS_SHIFT,
        false => 0,
    };

    // TODO: make this better XD
    if byte & 0x01 != 0 {
        tmp += D4_SHIFT;
    }
    if byte & 0x02 != 0 {
        tmp += D5_SHIFT;
    }
    if byte & 0x04 != 0 {
        tmp += D6_SHIFT;
    }
    if byte & 0x08 != 0 {
        tmp += D7_SHIFT;
    }
    tmp += E_SHIFT;
    tmp += BACKLIGHT_SHIFT;

    delay.delay_micros(E_DELAY);
    latch_pin.set_low();
    shift_out(clk_pin, data_pin, tmp);
    shift_out(clk_pin, data_pin, 255); // FIRST SHIFTER (FOR BUTTONS)
    latch_pin.set_high();
    delay.delay_micros(E_PULSE);
    latch_pin.set_low();
    shift_out(clk_pin, data_pin, BACKLIGHT_SHIFT);
    shift_out(clk_pin, data_pin, 255); // FIRST SHIFTER (FOR BUTTONS)
    latch_pin.set_high();
    delay.delay_micros(E_DELAY);

    latch_pin.set_low();
}
