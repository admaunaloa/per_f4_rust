/**
 * @file bsp.rs
 *
 * This file contains the BSP functions
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
 
use super::bit;
use super::gpio;
use super::rcc;
use super::syscfg;
use super::tim_ad;

const FCLCK2: u32 = 42000000;
const TIMX_CLOCK: u32 = FCLCK2 * 2;

const TIM_PWM_FREQ: u32 = 36000; // [Hz] Puse Width Modulation (PWM) frequency
const TIM_PWM_DEAD_TIME: u32 = 250; // [nano seconds] Dead time between high and low FET

const TIM_PWM_FREQ_COUNT: u16 = (TIMX_CLOCK / TIM_PWM_FREQ) as u16;
const TIM_PWM_DEAD_TIME_COUNT: u16 = (((TIMX_CLOCK / 1000000) * TIM_PWM_DEAD_TIME) / 1000) as u16;

fn motor1_ul() -> &'static gpio::Out {
    gpio::BIT5.output(gpio::A) // A5
}

fn motor1_bkin() -> &'static gpio::In {
    gpio::BIT6.input(gpio::A) // A6
}

fn motor1_vl() -> &'static gpio::Out {
    gpio::BIT0.output(gpio::B) // B0
}

fn motor1_wl() -> &'static gpio::Out {
    gpio::BIT1.output(gpio::B) // B1
}

pub fn in_1() -> &'static gpio::In {
    gpio::BIT8.input(gpio::B) // B8
}

fn motor1_uh() -> &'static gpio::Out {
    gpio::BIT6.output(gpio::C) // C6
}

fn motor1_vh() -> &'static gpio::Out {
    gpio::BIT7.output(gpio::C) // C7
}

fn motor1_wh() -> &'static gpio::Out {
    gpio::BIT8.output(gpio::C) // C8
}

pub fn out_1() -> &'static gpio::Out {
    gpio::BIT3.output(gpio::D) // D3
}

pub fn out_0() -> &'static gpio::Out {
    gpio::BIT4.output(gpio::D) // D4
}

pub fn in_2() -> &'static gpio::In {
    gpio::BIT5.input(gpio::D) // D5
}

pub fn in_3() -> &'static gpio::In {
    gpio::BIT6.input(gpio::D) // D6
}

pub fn in_0() -> &'static gpio::In {
    gpio::BIT7.input(gpio::D) // D7
}

fn enable_clocks() {
    rcc::base().gpioaen.set(true);
    rcc::base().gpioben.set(true);
    rcc::base().gpiocen.set(true);
    rcc::base().gpioden.set(true);
    rcc::base().gpioeen.set(true);

    rcc::base().tim1en.set(true);
    rcc::base().tim8en.set(true);
}

fn init_inputs() {
    let conf = gpio::InitIn::new(gpio::Mode::Input, gpio::Pupd::PullDown, gpio::Af::None);
    in_0().init(&conf);
    in_1().init(&conf);
    in_2().init(&conf);
    in_3().init(&conf);
}

fn init_outputs() {
    let conf = gpio::InitOut::new(
        gpio::Mode::Output,
        gpio::Otype::PushPull,
        gpio::Ospeed::Low,
        gpio::Af::None,
    );
    out_0().init(&conf);
    out_1().init(&conf);
}

fn init_tim8() {
    tim_ad::tim8().set_auto_reload(TIM_PWM_FREQ_COUNT);
    tim_ad::tim8().set_center_aligned_mode(tim_ad::Cms::CenterAligned3);
    tim_ad::tim8().set_master_mode(tim_ad::Mms::Oc4Ref);
    tim_ad::tim8().enable_break(true);
    tim_ad::tim8().set_dead_time(TIM_PWM_DEAD_TIME_COUNT);
    tim_ad::tim8().off_state(true, true);
    tim_ad::tim8().set_capture_compare_output(false, true, tim_ad::CcmOutputMode::PwmMode1, false);
    tim_ad::tim8().set_lock(tim_ad::BdtrLock::Level1);
    tim_ad::tim8().generate_update();

    let conf_out = gpio::InitOut::new(
        gpio::Mode::Alternate,
        gpio::Otype::PushPull,
        gpio::Ospeed::Medium,
        gpio::Af::Tim8,
    );
    motor1_ul().init(&conf_out);
    motor1_vl().init(&conf_out);
    motor1_wl().init(&conf_out);
    motor1_uh().init(&conf_out);
    motor1_vh().init(&conf_out);
    motor1_wh().init(&conf_out);

    let conf_in = gpio::InitIn::new(gpio::Mode::Alternate, gpio::Pupd::PullUp, gpio::Af::Tim8);
    motor1_bkin().init(&conf_in);
}

pub fn init() {
    // Temporary do test execution here
    bit::unit_test();
    gpio::unit_test();
    rcc::unit_test();
    syscfg::unit_test();
    tim_ad::unit_test();

    enable_clocks();

    init_inputs();
    init_outputs();
    init_tim8();
}
