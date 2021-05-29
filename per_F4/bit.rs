/**
 * @file bit.rs
 *
 * This file contains the peripheral bitband functions
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
use core::cell::UnsafeCell;
use core::ptr::{read_volatile, write_volatile};

const BITMASK_0: u16 = 0b00000001;
const SET_RETRIES: u16 = 3; // The probability of an interrupted write should be minimal. Therefore three retries should be more than enough

// Get register as 32 bit
unsafe fn get_reg_32(cell: &(UnsafeCell<u32>)) -> u32 {
    let address = cell.get() as usize;
    let value = read_volatile(base::bitband_to_register(address));
    (value >> base::bitband_to_offset(address))
}

// Get register as 16 bit
unsafe fn get_reg_16(cell: &(UnsafeCell<u32>)) -> u16 {
    let address = cell.get() as usize;
    let value = read_volatile(base::bitband_to_register(address) as *mut u16);
    (value >> base::bitband_to_offset(address))
}

// Set register as 32 bit
unsafe fn set_reg_32(cell: &(UnsafeCell<u32>), value: u32) {
    let address = cell.get() as usize;
    write_volatile(base::bitband_to_register(address), value)
}

// Set register as 16 bit
unsafe fn set_reg_16(cell: &(UnsafeCell<u32>), value: u16) {
    let address = cell.get() as usize;
    write_volatile(base::bitband_to_register(address) as *mut u16, value)
}

macro_rules! read_write {
    ($name:ident, $size:expr) => {
        #[repr(C, packed)]
        pub struct $name([UnsafeCell<u32>; $size]);
        impl $name {
            pub fn get(&self) -> u16 {
                unsafe { get_reg_16(&self.0[0]) & ((1 << $size) - 1) }
            }
            pub fn bit_address(&self) -> usize {
                unsafe { self.0[0].get() as usize }
            }
            fn set_do(&self, value: u16) {
                for x in 0..$size {
                    unsafe {
                        write_volatile(self.0[x].get(), ((value >> x) & BITMASK_0) as u32);
                    }
                }
            }
            pub fn set(&self, value: u16) {
                self.set_do(value);
            }
            pub fn set_block(&self, value: u16) {
                cortex_m::interrupt::free(|_| self.set_do(value))
            }
        }
    };
}

macro_rules! read {
    ($name:ident, $size:expr) => {
        #[repr(C, packed)]
        pub struct $name([UnsafeCell<u32>; $size]);
        impl $name {
            pub fn get(&self) -> u32 {
                unsafe { get_reg_32(&self.0[0]) & ((1 << $size) - 1) }
            }
        }
    };
}

// One bit reserved. No access possible
#[repr(C, packed)]
pub struct N1(u32);

// 32 bit Reserved. No access possible
#[repr(C, packed)]
pub struct N32([u32; base::PERIPH_REG_BITS]);

// One bit read write
#[repr(C, packed)]
pub struct Rw1(UnsafeCell<u32>);

impl Rw1 {
    #[inline]
    pub fn get(&self) -> bool {
        unsafe { read_volatile(self.0.get()) != 0 }
    }
    #[inline]
    pub fn set(&self, value: bool) {
        unsafe { write_volatile(self.0.get(), value as u32) }
    }
    pub fn bit_address(&self) -> usize {
        unsafe { self.0.get() as usize }
    }
    pub fn new() -> Rw1 {
        Rw1(UnsafeCell::new(0))
    }
}

// One bit read only
#[repr(C, packed)]
pub struct R1(UnsafeCell<u32>);

impl R1 {
    #[inline]
    pub fn get(&self) -> bool {
        unsafe { read_volatile(self.0.get()) != 0 }
    }
    pub fn bit_address(&self) -> usize {
        unsafe { self.0.get() as usize }
    }
}

// One bit write only
#[repr(C, packed)]
pub struct W1(UnsafeCell<u32>);

impl W1 {
    #[inline]
    pub fn set(&self, value: bool) {
        unsafe { write_volatile(self.0.get(), value as u32) }
    }
}

// One bit read and clear only
#[repr(C, packed)]
pub struct Rc1(UnsafeCell<u32>);

impl Rc1 {
    #[inline]
    pub fn get(&self) -> bool {
        unsafe { read_volatile(self.0.get()) != 0 }
    }
    #[inline]
    pub fn clear(&self) {
        unsafe { write_volatile(self.0.get(), 0) }
    }
}

read!(R2, 2);
read!(R8, 8);

read_write!(Rw2, 2);
read_write!(Rw3, 3);
read_write!(Rw4, 4);
read_write!(Rw5, 5);
read_write!(Rw6, 6);
read_write!(Rw8, 8);
read_write!(Rw9, 9);
read_write!(Rw13, 13);
read_write!(Rw15, 15);

// Rw16 with register block read write
#[repr(C, packed)]
pub struct Rw16Reg([UnsafeCell<u32>; 16]);

impl Rw16Reg {
    pub fn get(&self) -> u16 {
        unsafe { get_reg_16(&self.0[0]) }
    }
    pub fn set(&self, value: u16) {
        unsafe { set_reg_16(&self.0[0], value) }
    }
}

// Rw32 with register block read write
#[repr(C, packed)]
pub struct Rw32Reg([UnsafeCell<u32>; 32]);

impl Rw32Reg {
    pub fn get(&self) -> u32 {
        unsafe { get_reg_32(&self.0[0]) }
    }
    pub fn set(&self, value: u32) {
        unsafe { set_reg_32(&self.0[0], value) }
    }
}

