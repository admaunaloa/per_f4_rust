/**
 * @file tim_ad.rs
 *
 * This file contains the peripheral TIMER Advanced register functions
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

// Advanced-control timers (TIM1 & TIM8)

// CMS Center-aligned mode selection
pub enum Cms {
    EdgeAligned,    // The counter counts up or down depending on the direction bit
    CenterAligned1, // The counter counts up and down alternatively. Output compare interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set only when the counter is counting down.
    CenterAligned2, // The counter counts up and down alternatively. Output compare interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set only when the counter is counting up.
    CenterAligned3, // The counter counts up and down alternatively. Output compare interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set both when the counter is counting up or down.
}

impl Cms {
    fn value(&self) -> u16 {
        match *self {
            Cms::EdgeAligned => 0b00,
            Cms::CenterAligned1 => 0b01,
            Cms::CenterAligned2 => 0b10,
            Cms::CenterAligned3 => 0b11,
        }
    }
}

// MMS Master mode selection
pub enum Mms {
    Reset,        // The UG bit from the TIMx_EGR register
    Enable,       // The Counter Enable signal CNT_EN
    Update,       // The update event
    ComparePulse, // The trigger output send a positive pulse when the CC1IF flag is to be set
    Oc1Ref,       // Compare - OC1REF signal
    Oc2Ref,       // Compare - OC2REF signal
    Oc3Ref,       // Compare - OC3REF signal
    Oc4Ref,       // Compare - OC4REF signal
}

impl Mms {
    fn value(&self) -> u16 {
        match *self {
            Mms::Reset => 0b000,
            Mms::Enable => 0b001,
            Mms::Update => 0b010,
            Mms::ComparePulse => 0b011,
            Mms::Oc1Ref => 0b100,
            Mms::Oc2Ref => 0b101,
            Mms::Oc3Ref => 0b110,
            Mms::Oc4Ref => 0b111,
        }
    }
}

// CCM Selection
pub enum CcmSelection {
    Output,   // CCx channel is configured as output
    InputTIx, // CCx channel is configured as input, IC1 is mapped on TI1
    InputTIy, // CCx channel is configured as input, IC1 is mapped on TI2
    InputTRC, // CCx channel is configured as input, IC1 is mapped on TRC
}

impl CcmSelection {
    fn value(&self) -> u16 {
        match *self {
            CcmSelection::Output => 0b00,
            CcmSelection::InputTIx => 0b01,
            CcmSelection::InputTIy => 0b10,
            CcmSelection::InputTRC => 0b11,
        }
    }
}

// CCM Output Mode
pub enum CcmOutputMode {
    Frozen,        // generate a timing base
    MatchActive,   // Set channel x to active level on match
    MatchInactive, // Set channel x to inactive level on match
    Toggle,        // OCxREF toggles
    ForceInactive, // Force inactive level
    ForceActive,   // Force active level
    PwmMode1,      // PWM mode 1 Up counting
    PwmMode2,      // PWM mode 2 Down counting
}

impl CcmOutputMode {
    fn value(&self) -> u16 {
        match *self {
            CcmOutputMode::Frozen => 0b000,
            CcmOutputMode::MatchActive => 0b001,
            CcmOutputMode::MatchInactive => 0b010,
            CcmOutputMode::Toggle => 0b011,
            CcmOutputMode::ForceInactive => 0b100,
            CcmOutputMode::ForceActive => 0b101,
            CcmOutputMode::PwmMode1 => 0b110,
            CcmOutputMode::PwmMode2 => 0b111,
        }
    }
}

#[repr(C, packed)]
struct Ccm {
    // capture/compare mode register 1 (TIMx_CCMR1)
    ccs: bit::Rw2,  // Capture/Compare 1 selection
    ocfe: bit::Rw1, // Output Compare 1 fast enable
    ocpe: bit::Rw1, // Output Compare 1 preload enable
    ocm: bit::Rw3,  // Output Compare 1 mode
    occe: bit::Rw1, // Output Compare 1 clear enable
}

impl Ccm {
    pub fn set_output(
        &self,
        fast_enable: bool,
        pre_load_enable: bool,
        mode: &CcmOutputMode,
        clear_enable: bool,
    ) {
        // CMMR
        self.ccs.set(CcmSelection::Output.value());
        self.ocfe.set(fast_enable);
        self.ocpe.set(pre_load_enable);
        self.ocm.set(mode.value());
        self.occe.set(clear_enable);
    }
}

// Winding options
pub enum Phase {
    Vp,  // 0 V 
    Un,  // 1 U
    Wp,  // 2 W
    Vn,  // 3 V
    Up,  // 4 U
    Wn,  // 5 W
    Off, // None, all off
}

impl Phase {
    fn value(&self) -> u16 {
        match *self {
            Phase::Vp => 0b0101_0101_0000_0101,
            Phase::Un => 0b0101_0101_0101_0000,
            Phase::Wp => 0b0101_0000_0101_0101,
            Phase::Vn => 0b0101_0101_0000_0101,
            Phase::Up => 0b0101_0101_0101_0000,
            Phase::Wn => 0b0101_0000_0101_0101,
            Phase::Off => 0b0101_0000_0000_0000,
        }
    }
}

// Lock
pub enum BdtrLock {
    Off,    // No bit is write protected
    Level1, // DTG bits OISx and OISxN bits and BKE/BKP/AOE bits
    Level2, // LOCK Level 1 + CCxP/CCxNP bits OSSR and OSSI bits
    Level3, // LOCK Level 2 + OCxM and OCxPE bits
}

impl BdtrLock {
    fn value(&self) -> u16 {
        match *self {
            BdtrLock::Off => 0b00,
            BdtrLock::Level1 => 0b01,
            BdtrLock::Level2 => 0b10,
            BdtrLock::Level3 => 0b11,
        }
    }
}

#[repr(C, packed)]
pub struct TimAd {
    // control register 1 (TIMx_CR1)
    cen: bit::Rw1,           // Counter enable
    udis: bit::Rw1,          // Update disable
    urs: bit::Rw1,           // Update request source
    opm: bit::Rw1,           // One pulse mode
    dir: bit::Rw1,           // Direction
    cms: bit::Rw2,           // Center-aligned mode selection
    arpe: bit::Rw1,          // Auto-reload preload enable
    ckd: bit::Rw2,           // Clock division
    cr1bit10: [bit::N1; 22], // Reserved, must be kept at reset value.

    // control register 2 (TIMx_CR2)
    ccpc: bit::Rw1,          // Counter enable
    cr2bit1: bit::N1,        // Reserved, must be kept at reset value
    ccus: bit::Rw1,          // Capture/compare control update selection
    ccds: bit::Rw1,          // Capture/compare DMA selection
    mms: bit::Rw3,           // Master mode selection
    ti1s: bit::Rw1,          // TI1 selection
    ois1: bit::Rw1,          // Output Idle state 1 (OC1 output)
    ois1n: bit::Rw1,         // Output Idle state 1 (OC1N output)
    ois2: bit::Rw1,          // Output Idle state 2 (OC2 output)
    ois2n: bit::Rw1,         // Output Idle state 2 (OC2N output)
    ois3: bit::Rw1,          // Output Idle state 3 (OC3 output)
    ois3n: bit::Rw1,         // Output Idle state 3 (OC3N output)
    ois4: bit::Rw1,          // Output Idle state 4 (OC4 output)
    cr2bit15: [bit::N1; 17], // Reserved, must be kept at reset value.

    // slave mode control register (TIMx_SMCR)
    sms: bit::Rw3,            // Slave mode selection
    smcrbit1: bit::N1,        // Reserved, must be kept at reset value
    ts: bit::Rw3,             // Trigger selection
    msm: bit::Rw1,            // Master/slave mode
    etf: bit::Rw4,            // External trigger filter
    etps: bit::Rw2,           // External trigger prescaler
    ece: bit::Rw1,            // External clock enable
    etp: bit::Rw1,            // External trigger polarity
    smcrbit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // DMA/interrupt enable register (TIMx_DIER)
    uie: bit::Rw1,            // Update interrupt enable
    cc1ie: bit::Rw1,          // Capture/Compare 1 interrupt enable
    cc2ie: bit::Rw1,          // Capture/Compare 2 interrupt enable
    cc3ie: bit::Rw1,          // Capture/Compare 3 interrupt enable
    cc4ie: bit::Rw1,          // Capture/Compare 4 interrupt enable
    comie: bit::Rw1,          // COM interrupt enable
    tie: bit::Rw1,            // Trigger interrupt enable
    bie: bit::Rw1,            // Break interrupt enable
    ude: bit::Rw1,            // Update DMA request enable
    cc1de: bit::Rw1,          // Capture/Compare 1 DMA request enable
    cc2de: bit::Rw1,          // Capture/Compare 2 DMA request enable
    cc3de: bit::Rw1,          // Capture/Compare 3 DMA request enable
    cc4de: bit::Rw1,          // Capture/Compare 4 DMA request enable
    comde: bit::Rw1,          // COM DMA request enable
    tde: bit::Rw1,            // Trigger DMA request enable
    dierbit15: [bit::N1; 17], // Reserved, must be kept at reset value.

    // status register (TIMx_SR)
    uif: bit::Rc1,          // Update interrupt flag
    cc1if: bit::Rc1,        // Capture/Compare 1 interrupt flag
    cc2if: bit::Rc1,        // Capture/Compare 2 interrupt flag
    cc3if: bit::Rc1,        // Capture/Compare 3 interrupt flag
    cc4if: bit::Rc1,        // Capture/Compare 4 interrupt flag
    comif: bit::Rc1,        // COM interrupt flag
    tif: bit::Rc1,          // Trigger interrupt flag
    bif: bit::Rc1,          // Break interrupt flag
    srbit8: bit::N1,        // Reserved, must be kept at reset value
    cc1of: bit::Rc1,        // Capture/Compare 1 overcapture flag
    cc2of: bit::Rc1,        // Capture/Compare 2 overcapture flag
    cc3of: bit::Rc1,        // Capture/Compare 3 overcapture flag
    cc4of: bit::Rc1,        // Capture/Compare 4 overcapture flag
    srbit13: [bit::N1; 19], // Reserved, must be kept at reset value.

    // event generation register (TIMx_EGR)
    ug: bit::W1,            // Update generation
    cc1g: bit::W1,          // Capture/Compare 1 generation
    cc2g: bit::W1,          // Capture/Compare 2 generation
    cc3g: bit::W1,          // Capture/Compare 3 generation
    cc4g: bit::W1,          // Capture/Compare 4 generation
    comg: bit::W1,          // Capture/Compare control update generation
    tg: bit::W1,            // Trigger generation
    bg: bit::W1,            // Break generation
    egrbit8: [bit::N1; 24], // Reserved, must be kept at reset value.

    // capture/compare mode register 1 (TIMx_CCMR1)
    ccm1: Ccm,                 // Capture/Compare 1
    ccm2: Ccm,                 // Capture/Compare 2
    ccmr1bit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // capture/compare mode register 2 (TIMx_CCMR2)
    ccm3: Ccm,                 // Capture/Compare 3
    ccm4: Ccm,                 // Capture/Compare 4
    ccmr2bit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // capture/compare enable register (TIMx_CCER)
    cce: bit::Rw16Reg,       //
    ccebit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // counter (TIMx_CNT)
    cnt: bit::Rw16Reg,       //
    cntbit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // prescaler (TIMx_PSC)
    psc: bit::Rw16Reg,       //
    pscbit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // auto-reload register (TIMx_ARR)
    arr: bit::Rw16Reg,       //
    arrbit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // 	repetition counter register (TIMx_RCR)
    rcr: bit::Rw8,          //
    rcrbit8: [bit::N1; 24], // Reserved, must be kept at reset value.

    // capture/compare register 1 (TIMx_CCR1)
    ccr1: bit::Rw16Reg,       //
    ccr1bit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // capture/compare register 2 (TIMx_CCR2)
    ccr2: bit::Rw16Reg,       //
    ccr2bit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // capture/compare register 3 (TIMx_CCR3)
    ccr3: bit::Rw16Reg,       //
    ccr3bit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // capture/compare register 4 (TIMx_CCR4)
    ccr4: bit::Rw16Reg,       //
    ccr4bit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // break and dead-time register (TIMx_BDTR)
    dtg: bit::Rw8,            // Dead-time generator setup
    lock: bit::Rw2,           // Lock configuration
    ossi: bit::Rw1,           // Off-state selection for Idle mode
    ossr: bit::Rw1,           // Off-state selection for Run mode
    bke: bit::Rw1,            // Break enable
    bkp: bit::Rw1,            // Break polarity
    aoe: bit::Rw1,            // Automatic output enable
    moe: bit::Rw1,            // Main output enable
    bdtrbit16: [bit::N1; 16], // Reserved, must be kept at reset value.

    // DMA control register (TIMx_DCR)
    dba: bit::Rw5,           // DMA base address
    dcrbit5: [bit::N1; 3],   // Reserved, must be kept at reset value.
    dbl: bit::Rw5,           // DMA burst length
    dcrbit13: [bit::N1; 19], // Reserved, must be kept at reset value.

    // DMA address for full transfer (TIMx_DMAR)
    dmab: bit::Rw16Reg,       //
    dmabbit16: [bit::N1; 16], // Reserved, must be kept at reset value.
}

impl TimAd {
    pub fn set_main_output(&self, enable: bool) {
        // Main output
        self.moe.set(enable);
    }

    pub fn generate_update(&self) {
        // Update generation
        self.ug.set(true);
    }

    pub fn set_auto_reload(&self, reload: u16) {
        self.arr.set(reload);
    }

    pub fn set_center_aligned_mode(&self, mode: Cms) {
        // CMS Center-aligned mode selection
        self.cms.set(mode.value());
    }

    pub fn set_master_mode(&self, mode: Mms) {
        // MMS Master mode selection
        self.mms.set(mode.value());
    }

    pub fn set_dead_time(&self, count: u16) {
        // dead time generation
        self.dtg.set(count);
    }

    pub fn enable_break(&self, enable: bool) {
        // break input enable
        self.bke.set(enable);
    }

    pub fn get_break(&self) -> bool {
        // break input get
        self.bif.get()
    }

    pub fn clear_break(&self) {
        // break input clear
        self.bif.clear();
    }

    pub fn off_state(&self, idle: bool, run: bool) {
        // Off-state selection
        self.ossi.set(idle);
        self.ossr.set(run);
    }

    pub fn set_capture_compare_output(
        &self,
        fast_enable: bool,
        pre_load_enable: bool,
        mode: CcmOutputMode,
        clear_enable: bool,
    ) {
        // CMMR
        self.ccm1
            .set_output(fast_enable, pre_load_enable, &mode, clear_enable);
        self.ccm2
            .set_output(fast_enable, pre_load_enable, &mode, clear_enable);
        self.ccm3
            .set_output(fast_enable, pre_load_enable, &mode, clear_enable);
        self.ccm4
            .set_output(fast_enable, pre_load_enable, &mode, clear_enable);
    }

    pub fn set_lock(&self, lock: BdtrLock) {
        // Lock configuration
        self.lock.set(lock.value());
    }


pub fn tim1() -> &'static TimAd {
    unsafe { &(*(base::TIM1_BASE.bit_band_base() as *mut TimAd)) }
}

pub fn tim8() -> &'static TimAd {
    unsafe { &(*(base::TIM8_BASE.bit_band_base() as *mut TimAd)) }
}

// Temporary test method
// #[cfg(test)]
// mod tests {
// #[test]
pub fn unit_test() {
    assert_eq!(0x50 * base::PERIPH_REG_BITS, mem::size_of::<TimAd>());
    //    assert_eq!(0x50 * base::PERIPH_REG_BITS, mem::size_of::<TimAd>());
}
//}
