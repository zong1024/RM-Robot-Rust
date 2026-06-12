//! 独立看门狗。只有完整控制周期结束后才允许喂狗。

use stm32f4xx_hal::pac;

pub struct Watchdog;

impl Watchdog {
    /// LSI 约 32 kHz，64 分频、重装载 250，超时约 500 ms。
    pub fn start() -> Self {
        let iwdg = unsafe { &*pac::IWDG::ptr() };
        iwdg.kr().write(|w| unsafe { w.bits(0x5555) });
        iwdg.pr().write(|w| unsafe { w.bits(0b100) });
        iwdg.rlr().write(|w| unsafe { w.bits(250) });
        while iwdg.sr().read().bits() != 0 {}
        iwdg.kr().write(|w| unsafe { w.bits(0xcccc) });
        iwdg.kr().write(|w| unsafe { w.bits(0xaaaa) });
        Self
    }

    #[inline]
    pub fn feed(&self) {
        let iwdg = unsafe { &*pac::IWDG::ptr() };
        iwdg.kr().write(|w| unsafe { w.bits(0xaaaa) });
    }
}
