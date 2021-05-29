/**
 * @file gpio.rs
 *
 * This file contains the peripheral GPIO register functions
 *
 * Copyright (c) 2021 admaunaloa admaunaloa@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
use super::base;
use super::bit;
use core::mem;

const NR_PINS: usize = 16;
const PINS_MASK: usize = 0x001F;

pub struct InitIn {
    mode: Mode,
    pupd: Pupd,
    af: Af,
}

impl InitIn {
    pub fn new(mode: Mode, pupd: Pupd, af: Af) -> InitIn {
        InitIn {
            mode: mode,
            pupd: pupd,
            af: af,
        }
    }
}

pub struct InitOut {
    mode: Mode,
    otype: Otype,
    ospeed: Ospeed,
    af: Af,
}

impl InitOut {
    pub fn new(mode: Mode, otype: Otype, ospeed: Ospeed, af: Af) -> InitOut {
        InitOut {
            mode: mode,
            otype: otype,
            ospeed: ospeed,
            af: af,
        }
    }
}

// One bit read write
#[repr(C, packed)]
pub struct Out(bit::Rw1);

impl Out {
    pub fn get(&self) -> bool {
        self.0.get()
    }
    pub fn set(&self, value: bool) {
        self.0.set(value)
    }
    pub fn init(&self, param: &InitOut) {
        let reg = self.0.bit_address();
        let pin = base::bitband_to_offset(reg) & PINS_MASK;
        let gpio = (reg & base::GPIO_BB_BIT_TO_BASE_FILTER) as *mut Gpio;

        unsafe {
            (*gpio).moder[pin].set(param.mode.value());
            (*gpio).otyper[pin].set(param.otype.value());
            (*gpio).ospeedr[pin].set(param.ospeed.value());
            (*gpio).pupdr[pin].set(Pupd::None.value());
            (*gpio).afr[pin].set(param.af.value());
        }
    }
}

// One bit read only
#[repr(C, packed)]
pub struct In(bit::R1);

impl In {
    pub fn get(&self) -> bool {
        self.0.get()
    }
    pub fn init(&self, param: &InitIn) {
        let reg = self.0.bit_address();
        let pin = base::bitband_to_offset(reg) & PINS_MASK;
        let gpio = (reg & base::GPIO_BB_BIT_TO_BASE_FILTER) as *mut Gpio;

        unsafe {
            (*gpio).moder[pin].set(param.mode.value());
            (*gpio).otyper[pin].set(Otype::PushPull.value());
            (*gpio).ospeedr[pin].set(Ospeed::Low.value());
            (*gpio).pupdr[pin].set(param.pupd.value());
            (*gpio).afr[pin].set(param.af.value());
        }
    }
}

pub struct Bit(usize);

impl Bit {
    pub fn input(&self, gpio: *mut Gpio) -> &'static In {
        unsafe { &(*gpio).idr[self.0] }
    }
    pub fn output(&self, gpio: *mut Gpio) -> &'static Out {
        unsafe { &(*gpio).odr[self.0] }
    }
}

pub const BIT0: Bit = Bit(0);
pub const BIT1: Bit = Bit(1);
pub const BIT2: Bit = Bit(2);
pub const BIT3: Bit = Bit(3);
pub const BIT4: Bit = Bit(4);
pub const BIT5: Bit = Bit(5);
pub const BIT6: Bit = Bit(6);
pub const BIT7: Bit = Bit(7);
pub const BIT8: Bit = Bit(8);
pub const BIT9: Bit = Bit(9);
pub const BIT10: Bit = Bit(10);
pub const BIT11: Bit = Bit(11);
pub const BIT12: Bit = Bit(12);
pub const BIT13: Bit = Bit(13);
pub const BIT14: Bit = Bit(14);
pub const BIT15: Bit = Bit(15);

pub enum Mode {
    Input,     // Input (reset state)
    Output,    // General purpose output mode
    Alternate, // Alternate function mode
    Analog,    // Analog mode
}

impl Mode {
    fn value(&self) -> u16 {
        match *self {
            Mode::Input => 0b00,
            Mode::Output => 0b01,
            Mode::Alternate => 0b10,
            Mode::Analog => 0b11,
        }
    }
}

pub enum Otype {
    PushPull,  // Output push-pull (reset state)
    OpenDrain, // Output open-drain
}

impl Otype {
    fn value(&self) -> bool {
        match *self {
            Otype::PushPull => false,
            Otype::OpenDrain => true,
        }
    }
}

pub enum Ospeed {
    Low,    // Low speed
    Medium, // Medium speed
    Fast,   // Fast speed
    High,   // High speed
}

impl Ospeed {
    fn value(&self) -> u16 {
        match *self {
            Ospeed::Low => 0b00,
            Ospeed::Medium => 0b01,
            Ospeed::Fast => 0b10,
            Ospeed::High => 0b11,
        }
    }
}

pub enum Pupd {
    None,     // No pull-up, pull-down
    PullUp,   // Pull-up
    PullDown, // Pull-down
}

impl Pupd {
    fn value(&self) -> u16 {
        match *self {
            Pupd::None => 0b00,
            Pupd::PullUp => 0b01,
            Pupd::PullDown => 0b10,
        }
    }
}

pub enum Af {
    None,     // AF0
    Rtc50Hz,  // AF0
    Mco,      // AF0
    Tamper,   // AF0
    Swj,      // AF0
    Trace,    // AF0
    Tim1,     // AF1
    Tim2,     // AF1
    Tim3,     // AF2
    Tim4,     // AF2
    Tim5,     // AF2
    Tim8,     // AF3
    Tim9,     // AF3
    Tim10,    // AF3
    Tim11,    // AF3
    I2c1,     // AF4
    I2c2,     // AF4
    I2c3,     // AF4
    Spi1,     // AF5
    Spi2,     // AF5
    I2S3ext5, // AF5
    Spi3,     // AF6
    I2S2ext,  // AF6
    Usart1,   // AF7
    Usart2,   // AF7
    Usart3,   // AF7
    I2S3ext7, // AF7
    Uart4,    // AF8
    Uart5,    // AF8
    Usart6,   // AF8
    Can1,     // AF9
    Can2,     // AF9
    Tim12,    // AF9
    Tim13,    // AF9
    Tim14,    // AF9
    OtgFs,    // AF10
    OtgHs,    // AF10
    Eth,      // AF11
    Fsmc,     // AF12
    OtgHsFs,  // AF12
    Sdio,     // AF12
    Dcmi,     // AF13
    Af14,     // AF14
    EventOut, // AF15
}

impl Af {
    fn value(&self) -> u16 {
        match *self {
            Af::None => 0,
            Af::Rtc50Hz => 0,
            Af::Mco => 0,
            Af::Tamper => 0,
            Af::Swj => 0,
            Af::Trace => 0,
            Af::Tim1 => 1,
            Af::Tim2 => 1,
            Af::Tim3 => 2,
            Af::Tim4 => 2,
            Af::Tim5 => 2,
            Af::Tim8 => 3,
            Af::Tim9 => 3,
            Af::Tim10 => 3,
            Af::Tim11 => 3,
            Af::I2c1 => 4,
            Af::I2c2 => 4,
            Af::I2c3 => 4,
            Af::Spi1 => 5,
            Af::Spi2 => 5,
            Af::I2S3ext5 => 5,
            Af::Spi3 => 6,
            Af::I2S2ext => 6,
            Af::Usart1 => 7,
            Af::Usart2 => 7,
            Af::Usart3 => 7,
            Af::I2S3ext7 => 7,
            Af::Uart4 => 8,
            Af::Uart5 => 8,
            Af::Usart6 => 8,
            Af::Can1 => 9,
            Af::Can2 => 9,
            Af::Tim12 => 9,
            Af::Tim13 => 9,
            Af::Tim14 => 9,
            Af::OtgFs => 10,
            Af::OtgHs => 10,
            Af::Eth => 11,
            Af::Fsmc => 12,
            Af::OtgHsFs => 12,
            Af::Sdio => 12,
            Af::Dcmi => 13,
            Af::Af14 => 14,
            Af::EventOut => 15,
        }
    }
}

#[repr(C, packed)]
pub struct Gpio {
    // MODER port mode register
    moder: [bit::Rw2; NR_PINS],

    // OTYPER port output type register
    otyper: [bit::Rw1; NR_PINS],
    otyper_reserved: [bit::R1; base::PERIPH_REG_BITS - NR_PINS],

    // OSPEEDR port output speed register
    ospeedr: [bit::Rw2; NR_PINS],

    // PUPDR port pull-up/pull-down register
    pupdr: [bit::Rw2; NR_PINS],

    // IDR port input data register
    idr: [In; NR_PINS],
    idr_reserved: [bit::R1; base::PERIPH_REG_BITS - NR_PINS],

    // ODR port output data register
    odr: [Out; NR_PINS],
    odr_reserved: [bit::R1; base::PERIPH_REG_BITS - NR_PINS],

    // BSRR port bit set/reset register
    bs: [bit::Rw1; NR_PINS],
    br: [bit::Rw1; NR_PINS],

    // LCK port configuration lock register
    lck: [bit::Rw1; NR_PINS],
    lckk: bit::Rw1,
    lck_reserved: [bit::Rw1; base::PERIPH_REG_BITS - NR_PINS - 1],

    // AFRL AFRH alternate function low/high register
    afr: [bit::Rw4; NR_PINS],
}

pub const A: *mut Gpio = base::GPIO_A_BASE.bit_band_base() as *mut Gpio;
pub const B: *mut Gpio = base::GPIO_B_BASE.bit_band_base() as *mut Gpio;
pub const C: *mut Gpio = base::GPIO_C_BASE.bit_band_base() as *mut Gpio;
pub const D: *mut Gpio = base::GPIO_D_BASE.bit_band_base() as *mut Gpio;
pub const E: *mut Gpio = base::GPIO_E_BASE.bit_band_base() as *mut Gpio;
pub const F: *mut Gpio = base::GPIO_F_BASE.bit_band_base() as *mut Gpio;
pub const G: *mut Gpio = base::GPIO_G_BASE.bit_band_base() as *mut Gpio;
pub const H: *mut Gpio = base::GPIO_H_BASE.bit_band_base() as *mut Gpio;
pub const I: *mut Gpio = base::GPIO_I_BASE.bit_band_base() as *mut Gpio;

// Temporary test method
//#[cfg(test)]
//mod tests {
//    #[test]
pub fn unit_test() {
    assert_eq!(0x28 * base::PERIPH_REG_BITS, mem::size_of::<Gpio>());
}
//}
