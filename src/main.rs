#![no_std]
#![no_main]

// pick a panicking behavior
// extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger
extern crate stm32f4xx_hal as hal;

use crate::hal::{
    prelude::*,
    rcc::{Rcc, Clocks},
    i2c::I2c,
    stm32,
    timer::Timer,
    timer::Event,
    delay::Delay,
    interrupt,
};
use cortex_m::interrupt::{Mutex, free};
use cortex_m_rt::{entry};
use cortex_m_semihosting::hprintln;
use embedded_graphics::{fonts::Font6x12, prelude::*};
use ssd1306::{prelude::*, Builder as SSD1306Builder};
use core::cell::{Cell, RefCell};
use core::ops::DerefMut;
use core::fmt;
use arrayvec::ArrayString;

static ELAPSED_MS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0u32));
static TIMER_TIM2: Mutex<RefCell<Option<Timer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (stm32::Peripherals::take(), cortex_m::Peripherals::take()) {
        let rcc = dp.RCC.constrain();
        let clocks = setup_clocks(rcc);
        let gpiob = dp.GPIOB.split();
        let i2c = I2c::i2c1(
            dp.I2C1,
            (
                gpiob.pb8.into_alternate_af4(),
                gpiob.pb9.into_alternate_af4(),
            ),
            400.khz(),
            clocks,
        );
        let gpioc = dp.GPIOC.split();
        let _board_btn = gpioc.pc13.into_pull_up_input();

        let mut disp: GraphicsMode<_> = SSD1306Builder::new().connect_i2c(i2c).into();
        disp.init().unwrap();
        disp.flush().unwrap();

        let mut timer = Timer::tim2(dp.TIM2, 1.khz(), clocks);
        timer.listen(Event::TimeOut);
        free(|cs| *TIMER_TIM2.borrow(cs).borrow_mut() = Some(timer));

        let mut delay = Delay::new(cp.SYST, clocks);

        stm32::NVIC::unpend(stm32f4xx_hal::interrupt::TIM2);
        unsafe { stm32::NVIC::unmask(stm32f4xx_hal::interrupt::TIM2); };

        loop {
            let elapsed = free(|cs| ELAPSED_MS.borrow(cs).get());

            let mut format_buf = ArrayString::<[u8; 10]>::new();
            format_elapsed(&mut format_buf, elapsed);

            let text = Font6x12::render_str(format_buf.as_str())
                .with_stroke(Some(1u8.into()))
                .translate(Coord::new(0, 0));
            disp.clear();
            disp.draw(text.into_iter());
            disp.flush().unwrap();

            delay.delay_ms(100u32);
        }
    }

    loop {}
}

#[interrupt]
fn TIM2() {
    free(|cs| {
        stm32::NVIC::unpend(stm32f4xx_hal::interrupt::TIM2);
        if let Some(ref mut tim2) = TIMER_TIM2.borrow(cs).borrow_mut().deref_mut() {
            tim2.clear_interrupt(Event::TimeOut);
        }

        let cell = ELAPSED_MS.borrow(cs);
        let val = cell.get();
        cell.replace(val + 1);

        // unsafe { (*stm32f405::TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit()) };
        // unsafe { (*stm32::TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit()); };
    });
}

fn setup_clocks(rcc: Rcc) -> Clocks {
    return rcc
        .cfgr
        .hclk(48.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .pclk2(24.mhz())
        .freeze();
}

fn format_elapsed(buf: &mut ArrayString<[u8; 10]>, elapsed: u32) {
    let minutes = elapsed_to_m(elapsed);
    let seconds = elapsed_to_s(elapsed);
    let millis = elapsed_to_ms(elapsed);
    fmt::write(buf, format_args!("{}:{:02}.{:03}", minutes, seconds, millis)).unwrap();
}

fn elapsed_to_ms(elapsed: u32) -> u32 {
    return elapsed % 1000
}

fn elapsed_to_s(elapsed: u32) -> u32 {
    return (elapsed - elapsed_to_ms(elapsed)) % 60000 / 1000
}

fn elapsed_to_m(elapsed: u32) -> u32 {
    return elapsed / 60000
}
