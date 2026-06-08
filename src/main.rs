#![no_std]
#![no_main]

use core::sync::atomic::{AtomicI16, AtomicU32, AtomicU8, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use panic_halt as _;
use rm_robot::{
    app::robot::{RobotController, RobotSensors},
    config::REMOTE_SWB_CHANNEL_INDEX,
    estimation::attitude::Attitude,
};
use stm32f4xx_hal::{
    gpio::Speed,
    pac::{self, interrupt},
    prelude::*,
};

mod platform;

static TICK_MS: AtomicU32 = AtomicU32::new(0);

#[no_mangle]
pub static ROBOT_ARMED: AtomicU8 = AtomicU8::new(0);
#[no_mangle]
pub static CHASSIS_ONLINE: AtomicU8 = AtomicU8::new(0);
#[no_mangle]
pub static GIMBAL_ONLINE: AtomicU8 = AtomicU8::new(0);
#[no_mangle]
pub static CHASSIS_WHEEL_MODE: AtomicU8 = AtomicU8::new(0);
#[no_mangle]
pub static SWB_CHANNEL_RAW: AtomicI16 = AtomicI16::new(0);
#[no_mangle]
pub static CONTROL_LOOP_COUNT: AtomicU32 = AtomicU32::new(0);

pub fn now_ms() -> u32 {
    TICK_MS.load(Ordering::Relaxed)
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let _clocks = rcc
        .cfgr
        .use_hse(12u32.MHz())
        .sysclk(168u32.MHz())
        .pclk1(42u32.MHz())
        .pclk2(84u32.MHz())
        .freeze();

    let gpioh = dp.GPIOH.split();
    let mut led_r = gpioh.ph12.into_push_pull_output();
    let mut led_g = gpioh.ph11.into_push_pull_output();
    let mut led_b = gpioh.ph10.into_push_pull_output();
    led_r.set_high();
    led_g.set_high();
    led_b.set_high();

    let gpiod = dp.GPIOD.split();
    let _can1_rx = gpiod.pd0.into_alternate::<9>().speed(Speed::VeryHigh);
    let _can1_tx = gpiod.pd1.into_alternate::<9>().speed(Speed::VeryHigh);
    let gpiob = dp.GPIOB.split();
    let _can2_rx = gpiob.pb5.into_alternate::<9>().speed(Speed::VeryHigh);
    let _can2_tx = gpiob.pb6.into_alternate::<9>().speed(Speed::VeryHigh);
    let gpioc = dp.GPIOC.split();
    let _sbus_rx = gpioc
        .pc11
        .into_alternate::<7>()
        .speed(Speed::VeryHigh)
        .internal_pull_up(true);

    init_usart3();
    platform::can::init();
    platform::sbus::init();

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::CAN1_RX0);
        pac::NVIC::unmask(pac::Interrupt::CAN2_RX0);
        pac::NVIC::unmask(pac::Interrupt::USART3);
    }

    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(168_000 - 1);
    cp.SYST.clear_current();
    cp.SYST.enable_interrupt();
    cp.SYST.enable_counter();

    let mut robot = RobotController::new();
    let mut last_tick = now_ms();
    let mut last_green_toggle = last_tick;
    let mut green_on = false;

    loop {
        let now = now_ms();
        if now == last_tick {
            cortex_m::asm::wfi();
            continue;
        }
        last_tick = now;

        let can = platform::can::snapshot();
        let sensors = RobotSensors {
            remote: platform::sbus::snapshot(),
            chassis: can.chassis,
            yaw_6623: can.yaw_6623,
            pitch_6020: can.pitch_6020,
            attitude: Attitude::default(),
        };
        let output = robot.update(&sensors, now);
        platform::can::send_chassis(output.chassis.current);
        platform::can::send_gimbal(output.gimbal.yaw_current, output.gimbal.pitch_current);

        ROBOT_ARMED.store(output.armed as u8, Ordering::Relaxed);
        CHASSIS_ONLINE.store(output.chassis.online as u8, Ordering::Relaxed);
        GIMBAL_ONLINE.store(output.gimbal.online as u8, Ordering::Relaxed);
        CHASSIS_WHEEL_MODE.store(output.chassis.wheel_mode as u8, Ordering::Relaxed);
        SWB_CHANNEL_RAW.store(
            sensors.remote.channels[REMOTE_SWB_CHANNEL_INDEX],
            Ordering::Relaxed,
        );
        CONTROL_LOOP_COUNT.fetch_add(1, Ordering::Relaxed);

        if now.wrapping_sub(last_green_toggle) >= 500 {
            last_green_toggle = now;
            green_on = !green_on;
            if green_on {
                led_g.set_low();
            } else {
                led_g.set_high();
            }
        }

        // 红灯表示任一运动模块离线，蓝灯表示整车已解锁。
        if now > 1000 && (!output.chassis.online || !output.gimbal.online) {
            led_r.set_low();
        } else {
            led_r.set_high();
        }
        if output.armed {
            led_b.set_low();
        } else {
            led_b.set_high();
        }
    }
}

fn init_usart3() {
    let usart = unsafe { &*pac::USART3::ptr() };
    let rcc = unsafe { &*pac::RCC::ptr() };
    rcc.apb1enr().modify(|_, w| w.usart3en().set_bit());
    usart.brr().write(|w| unsafe { w.bits(420) });
    usart.cr1().write(|w| {
        w.m()
            .bit(true)
            .pce()
            .set_bit()
            .ps()
            .clear_bit()
            .re()
            .set_bit()
            .te()
            .clear_bit()
    });
    usart.cr2().write(|w| unsafe { w.stop().bits(0b10) });
    usart.cr3().write(|w| w.eie().set_bit());
    usart
        .cr1()
        .modify(|_, w| w.ue().set_bit().rxneie().set_bit().peie().set_bit());
}

#[interrupt]
fn CAN1_RX0() {
    platform::can::irq(1);
}

#[interrupt]
fn CAN2_RX0() {
    platform::can::irq(2);
}

#[interrupt]
fn USART3() {
    platform::sbus::irq();
}

#[exception]
fn SysTick() {
    TICK_MS.fetch_add(1, Ordering::Relaxed);
}
