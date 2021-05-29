/**
 * @file base.rs
 *
 * This file contains the peripheral BITBAND register functions
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

const PERIPH_BASE: usize = 0x40000000;
const PERIPH_BB_BASE: usize = 0x42000000;
const PERIPH_BB_REG_SHIFT: u32 = 5;
const PERIPH_BB_REG_MASK: usize = (0x0001 << PERIPH_BB_REG_SHIFT) - 1;
const PERIPH_BB_BIT_SHIFT: u32 = 2;
const PERIPH_BB_BIT_MASK: usize = (0x0001 << PERIPH_BB_BIT_SHIFT) - 1;
const PERIPH_BB_BIT_FILTER: usize = !PERIPH_BB_BIT_MASK;
const PERIPH_TO_BIT_BAND_BASE: usize = PERIPH_BB_BASE - (PERIPH_BASE << PERIPH_BB_REG_SHIFT);
const PERIPH_TO_REG: usize = PERIPH_BASE - (PERIPH_BB_BASE >> PERIPH_BB_REG_SHIFT);
const PERIPH_GPIO_BASE_SIZE: usize = 0x0400;
pub const GPIO_BB_BIT_TO_BASE_FILTER: usize = !((PERIPH_GPIO_BASE_SIZE << PERIPH_BB_BIT_SHIFT) - 1);

pub const PERIPH_REG_BITS: usize = 32; // 32 bits in an u32 register

pub struct PeriphBase(usize); // Peripheral base address

impl PeriphBase {
    pub const fn bit_band_base(&self) -> usize {
        (self.0 << PERIPH_BB_REG_SHIFT) + PERIPH_TO_BIT_BAND_BASE
    }
}

const APB1PERIPH_BASE: usize = PERIPH_BASE;
const APB2PERIPH_BASE: usize = PERIPH_BASE + 0x00010000;
const AHB1PERIPH_BASE: usize = PERIPH_BASE + 0x00020000;
const AHB2PERIPH_BASE: usize = PERIPH_BASE + 0x10000000;

pub const GPIO_A_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x0000);
pub const GPIO_B_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x0400);
pub const GPIO_C_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x0800);
pub const GPIO_D_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x0C00);
pub const GPIO_E_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x1000);
pub const GPIO_F_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x1400);
pub const GPIO_G_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x1800);
pub const GPIO_H_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x1C00);
pub const GPIO_I_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x2000);
pub const CRC_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x3000);
pub const RCC_BASE: PeriphBase = PeriphBase(AHB1PERIPH_BASE + 0x3800);

pub const TIM1_BASE: PeriphBase = PeriphBase(APB2PERIPH_BASE + 0x0000);
pub const TIM8_BASE: PeriphBase = PeriphBase(APB2PERIPH_BASE + 0x0400);
pub const SYSCFG_BASE: PeriphBase = PeriphBase(APB2PERIPH_BASE + 0x3800);

pub const fn bitband_to_register(bb: usize) -> *mut u32 {
    (((bb >> PERIPH_BB_REG_SHIFT) + PERIPH_TO_REG) & PERIPH_BB_BIT_FILTER) as *mut u32
}

pub const fn bitband_to_offset(bb: usize) -> usize {
    (bb >> PERIPH_BB_BIT_SHIFT) & PERIPH_BB_REG_MASK
}
