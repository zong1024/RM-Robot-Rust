//! 双 CAN 总线驱动和电机反馈仓库。

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use cortex_m::interrupt::{free, Mutex};
use rm_robot::{
    config::{
        CAN_CHASSIS_COMMAND_ID, CAN_GIMBAL_COMMAND_ID, CHASSIS_FEEDBACK_IDS,
        PITCH_6020_FEEDBACK_ID, YAW_6623_FEEDBACK_ID,
    },
    domain::{
        can_protocol::{decode_6623, decode_standard_motor, encode_current_group},
        motor::MotorFeedback,
    },
};
use stm32f4xx_hal::pac;

const CAN_BTR_1M_AT_42M: u32 = 0x0029_0002;

#[derive(Clone, Copy, Default)]
pub struct CanSnapshot {
    pub chassis: [MotorFeedback; 4],
    pub yaw_6623: MotorFeedback,
    pub pitch_6020: MotorFeedback,
}

struct CanState {
    snapshot: CanSnapshot,
}

impl CanState {
    const fn new() -> Self {
        const EMPTY: MotorFeedback = MotorFeedback {
            encoder: 0,
            speed_rpm: 0,
            measured_current: 0,
            commanded_current: 0,
            temperature: 0,
            received_at_ms: 0,
            frame_count: 0,
        };
        Self {
            snapshot: CanSnapshot {
                chassis: [EMPTY; 4],
                yaw_6623: EMPTY,
                pitch_6020: EMPTY,
            },
        }
    }
}

static STATE: Mutex<RefCell<CanState>> = Mutex::new(RefCell::new(CanState::new()));

#[no_mangle]
pub static CAN1_RX_COUNT: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static CAN2_RX_COUNT: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static CAN_TX_OK_COUNT: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static CAN_TX_BUSY_COUNT: AtomicU32 = AtomicU32::new(0);

pub fn init() {
    let can1 = unsafe { &*pac::CAN1::ptr() };
    let can2 = unsafe { &*pac::CAN2::ptr() };
    let rcc = unsafe { &*pac::RCC::ptr() };

    rcc.apb1enr()
        .modify(|_, w| w.can1en().set_bit().can2en().set_bit());
    init_one(can1);
    init_one(can2);

    // CAN1 与 CAN2 共用过滤器，14 号过滤器起归 CAN2。
    can1.fmr().write(|w| unsafe { w.bits((14 << 8) | 1) });
    configure_accept_all_filter(can1, 0);
    configure_accept_all_filter(can1, 14);
    can1.fmr().modify(|r, w| unsafe { w.bits(r.bits() & !1) });

    can1.ier().modify(|_, w| w.fmpie0().set_bit());
    can2.ier().modify(|_, w| w.fmpie0().set_bit());
    can1.mcr()
        .modify(|_, w| w.inrq().clear_bit().sleep().clear_bit());
    can2.mcr()
        .modify(|_, w| w.inrq().clear_bit().sleep().clear_bit());
    while can1.msr().read().inak().bit_is_set() {}
    while can2.msr().read().inak().bit_is_set() {}
}

pub fn irq(bus: u8) {
    let can = if bus == 1 {
        unsafe { &*pac::CAN1::ptr() }
    } else {
        unsafe { &*pac::CAN2::ptr() }
    };

    while can.rf0r().read().fmp().bits() > 0 {
        let rx = can.rx(0);
        let rir = rx.rir().read().bits();
        let low = rx.rdlr().read().bits();
        let high = rx.rdhr().read().bits();
        let id = (rir >> 21) as u16;
        let data = [
            low as u8,
            (low >> 8) as u8,
            (low >> 16) as u8,
            (low >> 24) as u8,
            high as u8,
            (high >> 8) as u8,
            (high >> 16) as u8,
            (high >> 24) as u8,
        ];
        receive(bus, id, data, crate::now_ms());
        can.rf0r().modify(|_, w| w.rfom().set_bit());
    }
}

pub fn snapshot() -> CanSnapshot {
    free(|cs| STATE.borrow(cs).borrow().snapshot)
}

pub fn send_chassis(currents: [i16; 4]) {
    send_group(1, CAN_CHASSIS_COMMAND_ID, currents);
}

pub fn send_gimbal(yaw_current: i16, pitch_current: i16) {
    send_group(2, CAN_GIMBAL_COMMAND_ID, [yaw_current, pitch_current, 0, 0]);
}

fn receive(bus: u8, id: u16, data: [u8; 8], now_ms: u32) {
    if bus == 1 {
        CAN1_RX_COUNT.fetch_add(1, Ordering::Relaxed);
    } else {
        CAN2_RX_COUNT.fetch_add(1, Ordering::Relaxed);
    }

    free(|cs| {
        let mut state = STATE.borrow(cs).borrow_mut();
        if bus == 1 {
            if let Some(index) = CHASSIS_FEEDBACK_IDS.iter().position(|value| *value == id) {
                update_standard_motor(&mut state.snapshot.chassis[index], data, now_ms);
            }
        } else if id == YAW_6623_FEEDBACK_ID {
            update_6623(&mut state.snapshot.yaw_6623, data, now_ms);
        } else if id == PITCH_6020_FEEDBACK_ID {
            update_standard_motor(&mut state.snapshot.pitch_6020, data, now_ms);
        }
    });
}

fn update_standard_motor(motor: &mut MotorFeedback, data: [u8; 8], now_ms: u32) {
    *motor = decode_standard_motor(*motor, data, now_ms);
}

fn update_6623(motor: &mut MotorFeedback, data: [u8; 8], now_ms: u32) {
    *motor = decode_6623(*motor, data, now_ms);
}

fn init_one(can: &pac::can1::RegisterBlock) {
    can.mcr()
        .modify(|_, w| w.inrq().set_bit().sleep().clear_bit());
    while can.msr().read().inak().bit_is_clear() {}
    can.mcr().modify(|_, w| {
        w.abom()
            .set_bit()
            .nart()
            .set_bit()
            .rflm()
            .clear_bit()
            .txfp()
            .clear_bit()
    });
    can.btr().write(|w| unsafe { w.bits(CAN_BTR_1M_AT_42M) });
}

fn configure_accept_all_filter(can1: &pac::can1::RegisterBlock, bank: usize) {
    let bit = 1u32 << bank;
    can1.fa1r()
        .modify(|r, w| unsafe { w.bits(r.bits() & !bit) });
    can1.fm1r()
        .modify(|r, w| unsafe { w.bits(r.bits() & !bit) });
    can1.fs1r().modify(|r, w| unsafe { w.bits(r.bits() | bit) });
    can1.ffa1r()
        .modify(|r, w| unsafe { w.bits(r.bits() & !bit) });
    can1.fb(bank).fr1().write(|w| unsafe { w.bits(0) });
    can1.fb(bank).fr2().write(|w| unsafe { w.bits(0) });
    can1.fa1r().modify(|r, w| unsafe { w.bits(r.bits() | bit) });
}

fn send_group(bus: u8, id: u16, currents: [i16; 4]) {
    let can = if bus == 1 {
        unsafe { &*pac::CAN1::ptr() }
    } else {
        unsafe { &*pac::CAN2::ptr() }
    };
    let data = encode_current_group(currents);

    let tsr = can.tsr().read();
    let mailbox = if tsr.tme0().bit_is_set() {
        0
    } else if tsr.tme1().bit_is_set() {
        1
    } else if tsr.tme2().bit_is_set() {
        2
    } else {
        CAN_TX_BUSY_COUNT.fetch_add(1, Ordering::Relaxed);
        return;
    };
    let tx = can.tx(mailbox);
    tx.tdtr().write(|w| unsafe { w.bits(8) });
    tx.tdlr()
        .write(|w| unsafe { w.bits(u32::from_le_bytes(data[0..4].try_into().unwrap())) });
    tx.tdhr()
        .write(|w| unsafe { w.bits(u32::from_le_bytes(data[4..8].try_into().unwrap())) });
    tx.tir()
        .write(|w| unsafe { w.bits(((id as u32) << 21) | 1) });
    CAN_TX_OK_COUNT.fetch_add(1, Ordering::Relaxed);
}
