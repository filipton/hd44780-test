#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{AnyOutput, Io},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

const BACKLIGHT_SHIFT: u8 = 0b1000_0000;
const RS_SHIFT: u8 = 0b0100_0000;
const E_SHIFT: u8 = 0b0010_0000;
/*
const D4_SHIFT: u8 = 0b0000_0001;
const D5_SHIFT: u8 = 0b0000_0010;
const D6_SHIFT: u8 = 0b0000_0100;
const D7_SHIFT: u8 = 0b0000_1000;
*/

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut data_pin = AnyOutput::new(io.pins.gpio10, esp_hal::gpio::Level::Low);
    let mut clk_pin = AnyOutput::new(io.pins.gpio21, esp_hal::gpio::Level::Low);
    let mut latch_pin = AnyOutput::new(io.pins.gpio1, esp_hal::gpio::Level::Low);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    esp_println::logger::init_logger_from_env();

    latch_pin.set_low();
    shift_out(&mut clk_pin, &mut data_pin, 0);
    shift_out(&mut clk_pin, &mut data_pin, 255);
    latch_pin.set_high();

    lcd_byte(0x33, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay,);
    lcd_byte(0x32, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay,);
    lcd_byte(0x28, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay,);
    lcd_byte(0x0C, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay,);
    lcd_byte(0x06, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay,);
    lcd_byte(0x01, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay,);
    delay.delay_millis(1000);

    lcd_byte(0x01, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay,);
    lcd_byte(0x80, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay,);

    let string = "Lorem Ipsum";
    for c in string.bytes() {
        lcd_byte(c, true, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    }
    /*
    // MOVE CURSOR RIGHT
    lcd_byte(0x14, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay,);
    */

    // MOVE CURSOR TO (x, y)
    let move_byte = (get_lcd_position((13, 0), (16, 2)) & 0b0111_1111) | 0b1000_0000;
    log::info!("move_byte: 0x{move_byte:02X}");
    lcd_byte(move_byte, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);

    let string = "WOW";
    for c in string.bytes() {
        lcd_byte(c, true, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    }

    lcd_byte(0xC0, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    let string = "Test 1234567890";
    for c in string.bytes() {
        lcd_byte(c, true, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    }

    delay.delay_millis(5000);
    // clear second line (to the end without first 5 chars)
    let move_byte = (get_lcd_position((5, 1), (16, 2)) & 0b0111_1111) | 0b1000_0000;
    lcd_byte(move_byte, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);

    for _ in 0..11 {
        lcd_byte(b' ', true, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
    }

    let start = esp_hal::time::current_time().duration_since_epoch();
    loop {
        delay.delay(66.millis());

        let elapsed = esp_hal::time::current_time().duration_since_epoch() - start;
        let move_byte = (get_lcd_position((5, 1), (16, 2)) & 0b0111_1111) | 0b1000_0000;
        lcd_byte(move_byte, false, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);

        for digit in num_to_digits(elapsed.to_millis() as u128) {
            if digit == 0xFF {
                break;
            }

            lcd_byte(digit + 0x30, true, &mut clk_pin, &mut data_pin, &mut latch_pin, &delay);
        }
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
        true => RS_SHIFT | ((byte >> 4) & 0b00001111),
        false => (byte >> 4) & 0b00001111,
    };

    tmp |= E_SHIFT;
    tmp |= BACKLIGHT_SHIFT;

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
        true => RS_SHIFT | (byte & 0b00001111),
        false => byte & 0b00001111,
    };

    tmp |= E_SHIFT;
    tmp |= BACKLIGHT_SHIFT;

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

// https://github.com/JohnDoneth/hd44780-driver/blob/3381df150b3eb7d65c81195c4730e3a007af2e2a/src/lib.rs#L114C1-L130C2
pub fn get_lcd_position(position: (u8, u8), size: (u8, u8)) -> u8 {
    if (position.0 >= size.0) || (position.1 >= size.1) {
        panic!(
            "Coordinates out of bounds: ({};{}) not fitting in a {}x{} display",
            position.0, position.1, size.0, size.1
        );
    }

    let mut addr = position.0 & 0x3f;
    if (position.1 & 1) == 1 {
        addr += 0x40;
    }
    if (position.1 & 2) == 2 {
        addr += size.0;
    }

    addr
}

fn num_to_digits(mut num: u128) -> [u8; 40] {
    let mut tmp = [0xFF; 40];
    let mut pos = 0;
    while num > 0 {
        let digit = (num % 10) as u8;
        tmp[pos] = digit;

        pos += 1;
        num /= 10;
    }

    // reverse
    for i in 0..(pos / 2) {
        let end_i = pos - i - 1;
        tmp.swap(i, end_i);
    }

    tmp
}
