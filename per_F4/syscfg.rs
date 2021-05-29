/**
 * @file syscfg.rs
 *
 * This file contains the peripheral syscfg register functions
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

#[repr(C, packed)]
pub struct Syscfg {
    // SYSCFG memory remap register (SYSCFG_MEMRMP)
    memmode: bit::Rw2,
    memrmpbit2: [bit::N1; 30],

    // SYSCFG peripheral mode configuration register (SYSCFG_PMC)
    pmcbit0: [bit::N1; 23],
    miirmiisel: bit::Rw1,
    pmcbit24: [bit::N1; 8],

    // SYSCFG external interrupt configuration register 1 (SYSCFG_EXTICR1)
    exti0: bit::Rw4,
    exti1: bit::Rw4,
    exti2: bit::Rw4,
    exti3: bit::Rw4,
    exti1bit16: [bit::N1; 16],

    // SYSCFG external interrupt configuration register 2 (SYSCFG_EXTICR2)
    exti4: bit::Rw4,
    exti5: bit::Rw4,
    exti6: bit::Rw4,
    exti7: bit::Rw4,
    exti2bit16: [bit::N1; 16],

    // SYSCFG external interrupt configuration register 3 (SYSCFG_EXTICR3)
    exti8: bit::Rw4,
    exti9: bit::Rw4,
    exti10: bit::Rw4,
    exti11: bit::Rw4,
    exti3bit16: [bit::N1; 16],

    // SYSCFG external interrupt configuration register 4 (SYSCFG_EXTICR4)
    exti12: bit::Rw4,
    exti13: bit::Rw4,
    exti14: bit::Rw4,
    exti15: bit::Rw4,
    exti4bit16: [bit::N1; 16],

    // SYSCFG Reserved
    reserved_0x18: bit::N32,
    reserved_0x1c: bit::N32,

    // Compensation cell control register (SYSCFG_CMPCR)
    cmppd: bit::Rw1, // Compensation cell power-down
    cmpcrbit1: [bit::N1; 7],
    ready: bit::R1, // Compensation cell ready flag
    cmpcrbit9: [bit::N1; 23],
}

pub fn base() -> &'static Syscfg {
    unsafe { &(*(base::SYSCFG_BASE.bit_band_base() as *mut Syscfg)) }
}

// Temporary test method
// #[cfg(test)]
// mod tests {
// #[test]
pub fn unit_test() {
    assert_eq!(0x24 * base::PERIPH_REG_BITS, mem::size_of::<Syscfg>());
}
//}
