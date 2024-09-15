#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{AnyOutput, Io},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use hd44780_driver::{display_size::DisplaySize, DisplayMode, HD44780};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let data_pin = AnyOutput::new(io.pins.gpio10, esp_hal::gpio::Level::Low);
    let clk_pin = AnyOutput::new(io.pins.gpio21, esp_hal::gpio::Level::Low);
    let latch_pin = AnyOutput::new(io.pins.gpio1, esp_hal::gpio::Level::Low);

    let mut adv_shift_reg =
        adv_shift_registers::AdvancedShiftRegister::<8, _>::new(data_pin, clk_pin, latch_pin, 0);
    adv_shift_reg.update_shifters();

    // backlight transistor
    let mut bl_pin = adv_shift_reg.get_pin_mut(1, 0);
    _ = bl_pin.set_high();

    let reg_sel_pin = adv_shift_reg.get_pin_mut(1, 1);
    let e_pin = adv_shift_reg.get_pin_mut(1, 2);
    let d4_pin = adv_shift_reg.get_pin_mut(1, 7);
    let d5_pin = adv_shift_reg.get_pin_mut(1, 6);
    let d6_pin = adv_shift_reg.get_pin_mut(1, 5);
    let d7_pin = adv_shift_reg.get_pin_mut(1, 4);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    esp_println::logger::init_logger_from_env();

    let mut lcd = HD44780::new_4bit(
        reg_sel_pin,
        e_pin,
        d4_pin,
        d5_pin,
        d6_pin,
        d7_pin,
        &mut delay,
    )
    .unwrap();

    _ = lcd.reset(&mut delay);
    _ = lcd.set_display_mode(
        DisplayMode {
            display: hd44780_driver::Display::On,
            cursor_visibility: hd44780_driver::Cursor::Invisible,
            cursor_blink: hd44780_driver::CursorBlink::Off,
        },
        &mut delay,
    );
    _ = lcd.set_display_size(DisplaySize::new(16, 2));
    _ = lcd.clear(&mut delay);

    _ = lcd.write_str("Lorem Ipsum", &mut delay);

    _ = lcd.set_cursor_xy((13, 0), &mut delay);
    _ = lcd.write_str("WOW", &mut delay);

    _ = lcd.set_cursor_xy((0, 1), &mut delay);
    _ = lcd.write_str("Test 1234567890", &mut delay);

    delay.delay_millis(5000);
    _ = lcd.set_cursor_xy((5, 1), &mut delay);
    _ = lcd.write_bytes(&[b' '; 11], &mut delay);

    let start = esp_hal::time::current_time().duration_since_epoch();
    loop {
        delay.delay(66.millis());

        let elapsed = esp_hal::time::current_time().duration_since_epoch() - start;
        _ = lcd.set_cursor_xy((5, 1), &mut delay);

        for digit in num_to_digits(elapsed.to_millis() as u128) {
            if digit == 0xFF {
                break;
            }

            _ = lcd.write_char((digit + 0x30) as char, &mut delay);
        }
    }
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
