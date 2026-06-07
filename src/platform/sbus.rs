//! USART3 S.BUS 中断适配层。

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use cortex_m::interrupt::{free, Mutex};
use rm_robot::domain::remote::{RemoteData, SbusDecoder};
use stm32f4xx_hal::pac;

struct State {
    decoder: SbusDecoder,
    remote: RemoteData,
}

impl State {
    const fn new() -> Self {
        Self {
            decoder: SbusDecoder::new(),
            remote: RemoteData::new(),
        }
    }
}

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State::new()));

#[no_mangle]
pub static SBUS_RAW_BYTE_COUNT: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static SBUS_FRAME_COUNT: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static SBUS_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);

pub fn init() {
    let usart = unsafe { &*pac::USART3::ptr() };
    let _ = usart.sr().read().bits();
    let _ = usart.dr().read().bits();
    usart
        .cr1()
        .modify(|_, w| w.rxneie().set_bit().peie().set_bit());
    usart.cr3().modify(|_, w| w.eie().set_bit());
}

pub fn irq() {
    let usart = unsafe { &*pac::USART3::ptr() };
    let status = usart.sr().read();
    if !status.rxne().bit_is_set()
        && !status.pe().bit_is_set()
        && !status.fe().bit_is_set()
        && !status.nf().bit_is_set()
        && !status.ore().bit_is_set()
    {
        return;
    }

    let byte = usart.dr().read().dr().bits() as u8;
    let has_error = status.pe().bit_is_set()
        || status.fe().bit_is_set()
        || status.nf().bit_is_set()
        || status.ore().bit_is_set();

    free(|cs| {
        let mut state = STATE.borrow(cs).borrow_mut();
        if has_error {
            state.decoder.note_uart_error();
        } else if let Some(remote) = state.decoder.push(byte, crate::now_ms()) {
            state.remote = remote;
            SBUS_FRAME_COUNT.store(remote.frame_count, Ordering::Relaxed);
        }
        SBUS_RAW_BYTE_COUNT.store(state.decoder.raw_byte_count(), Ordering::Relaxed);
        SBUS_ERROR_COUNT.store(state.decoder.error_count(), Ordering::Relaxed);
    });
}

pub fn snapshot() -> RemoteData {
    free(|cs| STATE.borrow(cs).borrow().remote)
}
