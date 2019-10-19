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
    gpio::{ExtiPin, Edge, PullUp, Input, gpioc::PC13},
    rcc::{Rcc, Clocks},
    i2c::I2c,
    stm32,
    timer::{Timer, Event},
    delay::Delay,
    interrupt,
};
use cortex_m::interrupt::{Mutex, free, CriticalSection};
use cortex_m_rt::{entry};
use embedded_graphics::{fonts::{Font6x12, Font12x16}, prelude::*};
use ssd1306::{prelude::*, Builder as SSD1306Builder};
use core::cell::{Cell, RefCell};
use core::ops::{DerefMut};
use core::fmt;
use arrayvec::ArrayString;

static ELAPSED_MS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0u32));
static TIMER_TIM2: Mutex<RefCell<Option<Timer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
static STATE: Mutex<Cell<StopwatchState>> = Mutex::new(Cell::new(StopwatchState::Ready));
static EXTI: Mutex<RefCell<Option<stm32::EXTI>>> = Mutex::new(RefCell::new(None));
static BUTTON: Mutex<RefCell<Option<PC13<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));

#[derive(Clone, Copy)]
enum StopwatchState {
    Ready,
    Running,
    Stopped
}

#[entry]
fn main() -> ! {
    if let (Some(mut dp), Some(cp)) = (stm32::Peripherals::take(), cortex_m::Peripherals::take()) {
        dp.RCC.apb2enr.write(|w| w.syscfgen().enabled());

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
        let mut board_btn = gpioc.pc13.into_pull_up_input();
        board_btn.make_interrupt_source(&mut dp.SYSCFG);
        board_btn.enable_interrupt(&mut dp.EXTI);
        board_btn.trigger_on_edge(&mut dp.EXTI, Edge::FALLING);

        let mut disp: GraphicsMode<_> = SSD1306Builder::new().connect_i2c(i2c).into();
        disp.init().unwrap();
        disp.flush().unwrap();

        let mut timer = Timer::tim2(dp.TIM2, 1.khz(), clocks);
        timer.listen(Event::TimeOut);

        let exti = dp.EXTI;
        free(|cs| {
            TIMER_TIM2.borrow(cs).replace(Some(timer));
            EXTI.borrow(cs).replace(Some(exti));
            BUTTON.borrow(cs).replace(Some(board_btn));
        });

        stm32::NVIC::unpend(hal::interrupt::TIM2);
        stm32::NVIC::unpend(hal::interrupt::EXTI15_10);
        unsafe {
            stm32::NVIC::unmask(hal::interrupt::EXTI15_10);
        };

        let mut delay = Delay::new(cp.SYST, clocks);

        loop {
            let elapsed = free(|cs| ELAPSED_MS.borrow(cs).get());

            let mut format_buf = ArrayString::<[u8; 10]>::new();
            format_elapsed(&mut format_buf, elapsed);

            disp.clear();

            let state = free(|cs| STATE.borrow(cs).get());
            let state_msg = match state {
                StopwatchState::Ready => "Ready",
                StopwatchState::Running => "",
                StopwatchState::Stopped => "Stopped"
            };
            let state_text = Font6x12::render_str(state_msg)
                .with_stroke(Some(1u8.into()))
                .translate(Coord::new(0, 0));
            disp.draw(state_text.into_iter());

            let elapsed_text = Font12x16::render_str(format_buf.as_str())
                .with_stroke(Some(1u8.into()))
                .translate(Coord::new(0, 14));
            disp.draw(elapsed_text.into_iter());

            disp.flush().unwrap();

            delay.delay_ms(100u32);
        }
    }

    loop {}
}

#[interrupt]
fn TIM2() {
    free(|cs| {
        if let Some(ref mut tim2) = TIMER_TIM2.borrow(cs).borrow_mut().deref_mut() {
            tim2.clear_interrupt(Event::TimeOut);
        }

        let cell = ELAPSED_MS.borrow(cs);
        let val = cell.get();
        cell.replace(val + 1);
    });
}

#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        let mut btn_ref = BUTTON.borrow(cs).borrow_mut();
        let mut exti_ref = EXTI.borrow(cs).borrow_mut();
        if let (Some(ref mut btn), Some(ref mut exti)) = (btn_ref.deref_mut(), exti_ref.deref_mut()) {
            // We cheat and don't bother checking _which_ exact interrupt line fired - there's only
            // ever going to be one in this example.
            btn.clear_interrupt_pending_bit(exti);

            let state = STATE.borrow(cs).get();
            // Run the state machine in an ISR - probably not something you want to do in most
            // cases but this one only starts and stops TIM2 interrupts
            match state {
                StopwatchState::Ready => {
                    stopwatch_start(cs);
                    STATE.borrow(cs).replace(StopwatchState::Running);
                },
                StopwatchState::Running => {
                    stopwatch_stop(cs);
                    STATE.borrow(cs).replace(StopwatchState::Stopped);
                },
                StopwatchState::Stopped => {},
            }
        }
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

fn stopwatch_start<'cs>(cs: &'cs CriticalSection) {
    ELAPSED_MS.borrow(cs).replace(0);
    unsafe { stm32::NVIC::unmask(hal::interrupt::TIM2); }
}

fn stopwatch_stop<'cs>(_cs: &'cs CriticalSection) {
    stm32::NVIC::mask(hal::interrupt::TIM2);
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
