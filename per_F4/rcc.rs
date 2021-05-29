/**
 * @file rcc.rs
 *
 * This file contains the peripheral RCC register functions
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
// Enable bit that requires an extra read after setting
#[repr(C, packed)]
pub struct En(bit::Rw1);

impl En {
    pub fn get(&self) -> bool {
        self.0.get()
    }
    pub fn set(&self, value: bool) {
        self.0.set(value);
        self.0.get();
    }
    // pub fn new() -> En {
    // En(bit::Rw1::new())
    // }
}

#[repr(C, packed)]
pub struct Rcc {
    // Control register (RCC_CR)
    pub hsion: bit::Rw1,    // Internal high-speed clock enable
    pub hsirdy: bit::R1,    // Internal high-speed clock ready flag
    cr_bit2: bit::N1,       // Reserved, must be kept at reset value
    pub hsitrim: bit::Rw5,  // Internal high-speed clock trimming
    pub hsical: bit::R8,    // Internal high-speed clock calibration
    pub hseon: bit::Rw1,    // HSE clock enable
    pub hserdy: bit::R1,    // HSE clock ready flag
    pub hsebyp: bit::Rw1,   // HSE clock bypass
    pub csson: bit::Rw1,    // Clock security system enable
    cr_bit20: [bit::N1; 4], // Reserved, must be kept at reset value
    pub pllon: bit::Rw1,    // Main PLL (PLL) enable
    pub pllrdy: bit::R1,    // Main PLL (PLL) clock ready flag
    pub plli2son: bit::Rw1, // PLLI2S enable
    pub plli2srdy: bit::R1, // PLLI2S clock ready flag
    cr_bit28: [bit::N1; 4], // Reserved, must be kept at reset value

    // PLL configuration register (RCC_PLLCFGR)
    pub pllm: bit::Rw6, // Division factor for the main PLL (PLL) and audio PLL (PLLI2S) input clock
    pub plln: bit::Rw9, // Main PLL (PLL) multiplication factor for VCO
    pub pllcfgr_bit15: bit::N1, // Reserved, must be kept at reset value
    pub pllp: bit::Rw2, // Main PLL (PLL) division factor for main system clock
    pllcfgr_bit18: [bit::N1; 4], // Reserved, must be kept at reset value
    pub pllscr: bit::Rw1, // Main PLL(PLL) and audio PLL (PLLI2S) entry clock source
    pllcfgr_bit23: bit::N1, // Reserved, must be kept at reset value
    pub pllq: bit::Rw4, // Main PLL (PLL) division factor for USB OTG FS, SDIO and random number generator clocks
    pllcfgr_bit28: [bit::N1; 4], // Reserved, must be kept at reset value

    // clock configuration register (RCC_CFGR)
    pub sw: bit::Rw2,       // System clock switch
    pub sws: bit::R2,       // System clock switch status
    pub hpre: bit::Rw4,     // AHB prescaler
    cir_bit8: [bit::N1; 2], // Reserved, must be kept at reset value
    pub ppre1: bit::Rw3,    // APB Low speed prescaler (APB1)
    pub ppre2: bit::Rw3,    // APB high-speed prescaler (APB2)
    pub rtcpre: bit::Rw5,   // HSE division factor for RTC clock
    pub mco1: bit::Rw2,     // Microcontroller clock output 1
    pub i2sscr: bit::Rw1,   // I2S clock selection
    pub mco1pre: bit::Rw3,  // MCO1 prescaler
    pub mco2pre: bit::Rw3,  // MCO2 prescaler
    pub mco2: bit::Rw2,     // Microcontroller clock output 2

    // clock interrupt register (RCC_CIR)
    pub lsirdyf: bit::R1,      // LSI ready interrupt flag
    pub lserdyf: bit::R1,      // LSE ready interrupt flag
    pub hsirdyf: bit::R1,      // HSI ready interrupt flag
    pub hserdyf: bit::R1,      // HSE ready interrupt flag
    pub pllrdyf: bit::R1,      // Main PLL (PLL) ready interrupt flag
    pub plli2srdyf: bit::R1,   // PLLI2S ready interrupt flag
    cir_bit6: bit::N1,         // Reserved, must be kept at reset value
    pub cssf: bit::R1,         // Clock security system interrupt flag
    pub lsirdyie: bit::Rw1,    // LSI ready interrupt enable
    pub lserdyie: bit::Rw1,    // LSE ready interrupt enable
    pub hsirdyie: bit::Rw1,    // HSI ready interrupt enable
    pub hserdyie: bit::Rw1,    // HSE ready interrupt enable
    pub pllrdyie: bit::Rw1,    // Main PLL (PLL) ready interrupt enable
    pub plli2srdyie: bit::Rw1, // PLLI2S ready interrupt enable
    cir_bit14: [bit::N1; 2],   // Reserved, must be kept at reset value.
    pub lsirdyc: bit::W1,      // LSI ready interrupt clear
    pub lserdyc: bit::W1,      // LSE ready interrupt clear
    pub hsirdyc: bit::W1,      // HSI ready interrupt clear
    pub hserdyc: bit::W1,      // HSE ready interrupt clear
    pub pllrdyc: bit::W1,      // Main PLL(PLL) ready interrupt clear
    pub plli2srdyc: bit::W1,   // PLLI2S ready interrupt clear
    cir_bit22: bit::N1,        // Reserved, must be kept at reset value.
    pub cssc: bit::W1,         // Clock security system interrupt clear
    cir_bit24: [bit::N1; 8],   // Reserved, must be kept at reset value.

    // RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
    pub gpioarst: bit::Rw1,       // IO port A reset
    pub gpiobrst: bit::Rw1,       // IO port B reset
    pub gpiocrst: bit::Rw1,       // IO port C reset
    pub gpiodrst: bit::Rw1,       // IO port D reset
    pub gpioerst: bit::Rw1,       // IO port E reset
    pub gpiofrst: bit::Rw1,       // IO port F reset
    pub gpiogrst: bit::Rw1,       // IO port G reset
    pub gpiohrst: bit::Rw1,       // IO port H reset
    pub gpioirst: bit::Rw1,       // IO port I reset
    ahb1rstr_bit9: [bit::N1; 3],  // Reserved, must be kept at reset value
    pub crcrst: bit::Rw1,         // CRC reset
    ahb1rstr_bit13: [bit::N1; 8], // Reserved, must be kept at reset value
    pub dma1rst: bit::Rw1,        // DMA1 reset
    pub dma2rst: bit::Rw1,        // DMA2 reset
    ahb1rstr_bit23: [bit::N1; 2], // Reserved, must be kept at reset value
    pub ethmacrst: bit::Rw1,      // Ethernet MAC reset
    ahb1rstr_bit26: [bit::N1; 3], // Reserved, must be kept at reset value
    pub otghsrst: bit::Rw1,       // USB OTG HS module reset
    ahb1rstr_bit30: [bit::N1; 2], // Reserved, must be kept at reset value

    // RCC AHB2 peripheral reset register (RCC_AHB2RSTR)
    pub dcmirst: bit::Rw1,           // Camera interface reset
    pub ahb2rstr_bit1: [bit::N1; 3], // Reserved, must be kept at reset value
    pub cryprst: bit::Rw1,           // Cryptographic module reset
    pub hashrst: bit::Rw1,           // Hash module reset
    pub rngrst: bit::Rw1,            // Random number generator module reset
    pub otgfsrst: bit::Rw1,          // USB OTG FS module reset
    ahb2rstr_bit8: [bit::N1; 24],    // Reserved, must be kept at reset value

    // RCC AHB3 peripheral reset register (RCC_AHB3RSTR)
    pub fsmcrst: bit::Rw1, // Flexible static memory controller module reset
    ahb3rstr_bit1: [bit::N1; 31], // Reserved, must be kept at reset value

    reserved_0x1c: bit::N32,

    // RCC APB1 peripheral reset register (RCC_APB1RSTR)
    pub tim2rst: bit::Rw1,        // TIM2 reset
    pub tim3rst: bit::Rw1,        // TIM3 reset
    pub tim4rst: bit::Rw1,        // TIM4 reset
    pub tim5rst: bit::Rw1,        // TIM5 reset
    pub tim6rst: bit::Rw1,        // TIM6 reset
    pub tim7rst: bit::Rw1,        // TIM7 reset
    pub tim12rst: bit::Rw1,       // TIM12 reset
    pub tim13rst: bit::Rw1,       // TIM13 reset
    pub tim14rst: bit::Rw1,       // TIM14 reset
    apb1rstr_bit9: [bit::N1; 2],  // Reserved, must be kept at reset value
    pub wwdgrst: bit::Rw1,        // Window watchdog reset
    apb1rstr_bit12: [bit::N1; 2], // Reserved, must be kept at reset value
    pub spi2rst: bit::Rw1,        // SPI2 reset
    pub spi3rst: bit::Rw1,        // SPI3 reset
    apb1rstr_bit_bit16: bit::N1,  // Reserved, must be kept at reset value
    pub usart2rst: bit::Rw1,      // USART2 reset
    pub usart3rst: bit::Rw1,      // USART3 reset
    pub uart4rst: bit::Rw1,       // UART4 reset
    pub uart5rst: bit::Rw1,       // UART5 reset
    pub i2c1rst: bit::Rw1,        // I2C1 reset
    pub i2c2rst: bit::Rw1,        // I2C2 reset
    pub i2c3rst: bit::Rw1,        // I2C3 reset
    apb1rstr_bit24: bit::N1,      // Reserved, must be kept at reset value
    pub can1rst: bit::Rw1,        // CAN1 reset
    pub can2rst: bit::Rw1,        // CAN2 reset
    apb1rstr_bit27: bit::N1,      // Reserved, must be kept at reset value
    pub pwrrst: bit::Rw1,         // Power interface reset
    pub dacrst: bit::Rw1,         // DAC reset
    apb1rstr_bit30: [bit::N1; 2], // Reserved, must be kept at reset value

    // RCC APB2 peripheral reset register (RCC_APB2RSTR)
    pub tim1rst: bit::Rw1,        // TIM1 reset
    pub tim8rst: bit::Rw1,        // TIM8 reset
    apb2rst_bit2: [bit::N1; 2],   // Reserved, must be kept at reset value
    pub usart1rst: bit::Rw1,      // USART1 reset
    pub usart6rst: bit::Rw1,      // USART6 reset
    apb2rst_bit6: [bit::N1; 2],   // Reserved, must be kept at reset value
    pub adcrst: bit::Rw1,         // ADC1 reset
    apb2rst_bit9: [bit::N1; 2],   // Reserved, must be kept at reset value
    pub sdiorst: bit::Rw1,        // SDIO reset
    pub spi1rst: bit::Rw1,        // SPI1 reset
    apb2rst_bit13: bit::N1,       // Reserved, must be kept at reset value
    pub syscfgrst: bit::Rw1,      // SYSCFGEN reset
    apb2rst_bit15: bit::N1,       // Reserved, must be kept at reset value
    pub tim9rst: bit::Rw1,        // TIM9 reset
    pub tim10rst: bit::Rw1,       // TIM10 reset
    pub tim11rst: bit::Rw1,       // TIM11 reset
    apb2rst_bit19: [bit::N1; 13], // Reserved, must be kept at reset value

    reserved_0x28: bit::N32,
    reserved_0x2c: bit::N32,

    // RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
    pub gpioaen: En,             // IO port A clock enable
    pub gpioben: En,             // IO port B clock enable
    pub gpiocen: En,             // IO port C clock enable
    pub gpioden: En,             // IO port D clock enable
    pub gpioeen: En,             // IO port E clock enable
    pub gpiofen: En,             // IO port F clock enable
    pub gpiogen: En,             // IO port G clock enable
    pub gpiohen: En,             // IO port H clock enable
    pub gpioien: En,             // IO port I clock enable
    ahb1enr_bit9: [bit::N1; 3],  // Reserved, must be kept at reset value
    pub crcen: En,               // CRC clock enable
    ahb1enr_bit13: [bit::N1; 5], // Reserved, must be kept at reset value
    pub bkpsramen: En,           // Backup SRAM interface clock enable
    ahb1enr_bit19: bit::N1,      // Reserved, must be kept at reset value.
    pub ccmdataramen: En,        // CCM data RAM clock enable
    pub dma1en: En,              // DMA1 clock enable
    pub dma2en: En,              // DMA2 clock enable
    ahb1enr_bit23: [bit::N1; 2], // Reserved, must be kept at reset value
    pub ethmacen: En,            // Ethernet MAC clock enable
    pub ethmactxen: En,          // Ethernet Transmission clock enable
    pub ethmacrxen: En,          // Ethernet Reception clock enable
    pub ethmacptpen: En,         // Ethernet PTP clock enable
    pub otghsen: En,             // USB OTG HS clock enable
    pub otghsulpien: En,         // USB OTG HSULPI clock enable
    ahb1enr_bit31: bit::N1,      // Reserved, must be kept at reset value

    // RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)
    pub dcmien: En,              // Camera interface enable
    ahb2enr_bit1: [bit::N1; 3],  // Reserved, must be kept at reset value.
    pub crypen: En,              // Cryptographic modules clock enable
    pub hashen: En,              // Hash modules clock enable
    pub rngen: En,               // Random number generator clock enable
    pub otgfsen: En,             // USB OTG FS clock enable
    ahb2enr_bit8: [bit::N1; 24], // Reserved, must be kept at reset value

    // RCC AHB3 peripheral clock enable register (RCC_AHB3ENR)
    pub fsmcen: En, // Flexible static memory controller module clock enable
    ahb3enr_bit1: [bit::N1; 31], // Reserved, must be kept at reset value

    reserved_0x3c: bit::N32,

    // RCC APB1 peripheral clock enable register (RCC_APB1ENR)
    pub tim2en: En,              // TIM2 clock enable
    pub tim3en: En,              // TIM3 clock enable
    pub tim4en: En,              // TIM4 clock enable
    pub tim5en: En,              // TIM5 clock enable
    pub tim6en: En,              // TIM6 clock enable
    pub tim7en: En,              // TIM7 clock enable
    pub tim12en: En,             // TIM12 clock enable
    pub tim13en: En,             // TIM13 clock enable
    pub tim14en: En,             // TIM14 clock enable
    apb1enr_bit9: [bit::N1; 2],  // Reserved, must be kept at reset value
    pub wwdgen: En,              // Window watchdog clock enable
    apb1enr_bit12: [bit::N1; 2], // Reserved, must be kept at reset value
    pub spi2en: En,              // SPI2 clock enable
    pub spi3en: En,              // SPI3 clock enable
    apb1enr_bit16: bit::N1,      // Reserved, must be kept at reset value
    pub usart2en: En,            // USART2 clock enable
    pub usart3en: En,            // USART3 clock enable
    pub uart4en: En,             // UART4 clock enable
    pub uart5en: En,             // UART5 clock enable
    pub i2c1en: En,              // I2C1 clock enable
    pub i2c2en: En,              // I2C2 clock enable
    pub i2c3en: En,              // I2C3 clock enable
    apb1enr_bit24: bit::N1,      // Reserved, must be kept at reset value
    pub can1en: En,              // CAN1 clock enable
    pub can2en: En,              // CAN2 clock enable
    apb1enr_bit27: bit::N1,      // Reserved, must be kept at reset value
    pub pwren: En,               // Power interface clock enable
    pub dacen: En,               // DAC interface clock enable
    apb1enr_bit30: [bit::N1; 2], // Reserved, must be kept at reset value

    // RCC APB2 peripheral clock enable register (RCC_APB2ENR)
    pub tim1en: En,               // TIM1 clock enable
    pub tim8en: En,               // TIM8 clock enable
    apb2enr_bit2: [bit::N1; 2],   // Reserved, must be kept at reset value
    pub usart1en: En,             // USART1 clock enable
    pub usart6en: En,             // USART6 clock enable
    apb2enr_bit6: [bit::N1; 2],   // Reserved, must be kept at reset value
    pub adc1en: En,               // ADC1 clock enable
    pub adc2en: En,               // ADC2 clock enable
    pub adc3en: En,               // ADC3 clock enable
    pub sdioen: En,               // SDIO clock enable
    pub spi1en: En,               // SPI1 clock enable
    apb2enr_bit13: bit::N1,       // Reserved, must be kept at reset value
    pub syscfgen: En,             // SYSCFGEN clock enable
    apb2enr_bit15: bit::N1,       // Reserved, must be kept at reset value
    pub tim9en: En,               // TIM9 clock enable
    pub tim10en: En,              // TIM10 clock enable
    pub tim11en: En,              // TIM11 clock enable
    apb2enr_bit19: [bit::N1; 13], // Reserved, must be kept at reset value

    reserved_0x48: bit::N32,
    reserved_0x4c: bit::N32,

    // RCC AHB1 peripheral clock enable in low power mode register (RCC_AHB1LPENR)
    pub gpioalpen: En,             // IO port A clock enable during Sleep mode
    pub gpioblpen: En,             // IO port B clock enable during Sleep mode
    pub gpioclpen: En,             // IO port C clock enable during Sleep mode
    pub gpiodlpen: En,             // IO port D clock enable during Sleep mode
    pub gpioelpen: En,             // IO port E clock enable during Sleep mode
    pub gpioflpen: En,             // IO port F clock enable during Sleep mode
    pub gpioglpen: En,             // IO port G clock enable during Sleep mode
    pub gpiohlpen: En,             // IO port H clock enable during Sleep mode
    pub gpioilpen: En,             // IO port I clock enable during Sleep mode
    ahb1lpenr_bit9: [bit::N1; 3],  // Reserved, must be kept at reset value.
    pub crclpen: En,               // CRC clock enable during Sleep mode
    ahb1lpenr_bit13: [bit::N1; 2], // Reserved, must be kept at reset value.
    pub flitflpen: En,             // Flash interface clock enable during Sleep mode
    pub sram1lpen: En,             // SRAM 1interface clock enable during Sleep mode
    pub sram2lpen: En,             // SRAM 2 interface clock enable during Sleep mode
    pub bkpsramlpen: En,           // Backup SRAM interface clock enable during Sleep mode
    ahb1lpenr_bit19: [bit::N1; 2], // Reserved, must be kept at reset value
    pub dma1lpen: En,              // DMA1 clock enable during Sleep mode
    pub dma2lpen: En,              // DMA2 clock enable during Sleep mode
    ahb1lpenr_bit23: [bit::N1; 2], // Reserved, must be kept at reset value
    pub ethmaclpen: En,            // Ethernet MAC clock enable during Sleep mode
    pub ethmactxlpen: En,          // Ethernet Transmission clock enable during Sleep mode
    pub ethmacrxlpen: En,          // Ethernet Reception clock enable during Sleep mode
    pub ethmacptplpen: En,         // Ethernet PTP clock enable during Sleep mode
    pub otghslpen: En,             // USB OTG HS clock enable during Sleep mode
    pub otghsulpilpen: En,         // USB OTG HSULPI clock enable during Sleep mode
    ahb1lpenr_bit31: bit::N1,      // Reserved, must be kept at reset value

    // RCC AHB2 peripheral clock enable in low power mode register (RCC_AHB2LPENR)
    pub dcmilpen: En,              // Camera interface clock enable during Sleep mode
    ahb2lpenr_bit1: [bit::N1; 3],  // Reserved, must be kept at reset value.
    pub cryplpen: En,              // Cryptographic modules clock enable during Sleep mode
    pub hashlpen: En,              // Hash modules clock enable during Sleep mode
    pub rnglpen: En,               // Random number generator clock enable during Sleep mode
    pub otgfslpen: En,             // USB OTG FS clock enable during Sleep mode
    ahb2lpenr_bit8: [bit::N1; 24], // Reserved, must be kept at reset value.

    // RCC AHB3 peripheral clock enable in low power mode register (RCC_AHB3LPENR)
    pub fsmclpen: En, // Flexible static memory controller module clock enable during Sleep mode
    ahb3lpenr_bit1: [bit::N1; 31], // Reserved, must be kept at reset value.

    reserved_0x5c: bit::N32,

    // RCC APB1 peripheral clock enable in low power mode register (RCC_APB1LPENR)
    pub tim2lpen: En,              // TIM2 clock enable during Sleep mode
    pub tim3lpen: En,              // TIM3 clock enable during Sleep mode
    pub tim4lpen: En,              // TIM4 clock enable during Sleep mode
    pub tim5lpen: En,              // TIM5 clock enable during Sleep mode
    pub tim6lpen: En,              // TIM6 clock enable during Sleep mode
    pub tim7lpen: En,              // TIM7 clock enable during Sleep mode
    pub tim12lpen: En,             // TIM12 clock enable during Sleep mode
    pub tim13lpen: En,             // TIM13 clock enable during Sleep mode
    pub tim14lpen: En,             // TIM14 clock enable during Sleep mode
    apb1lpenr_bit9: [bit::N1; 2],  // Reserved, must be kept at reset value
    pub wwdglpen: En,              // Window watchdog clock enable during Sleep mode
    apb1lpenr_bit12: [bit::N1; 2], // Reserved, must be kept at reset value
    pub spi2lpen: En,              // SPI2 clock enable during Sleep mode
    pub spi3lpen: En,              // SPI3 clock enable during Sleep mode
    apb1lpenr_bit16: bit::N1,      // Reserved, must be kept at reset value
    pub usart2lpen: En,            // USART2 clock enable during Sleep mode
    pub usart3lpen: En,            // USART3 clock enable during Sleep mode
    pub uart4lpen: En,             // UART4 clock enable during Sleep mode
    pub uart5lpen: En,             // UART5 clock enable during Sleep mode
    pub i2c1lpen: En,              // I2C1 clock enable during Sleep mode
    pub i2c2lpen: En,              // I2C2 clock enable during Sleep mode
    pub i2c3lpen: En,              // I2C3 clock enable during Sleep mode
    apb1lpenr_bit24: bit::N1,      // Reserved, must be kept at reset value
    pub can1lpen: En,              // CAN1 clock enable during Sleep mode
    pub can2lpen: En,              // CAN2 clock enable during Sleep mode
    apb1lpenr_bit27: bit::N1,      // Reserved, must be kept at reset value
    pub pwrlpen: En,               // Power interface clock enable during Sleep mode
    pub daclpen: En,               // DAC interface clock enable during Sleep mode
    apb1lpenr_bit: [bit::N1; 2],   // Reserved, must be kept at reset value

    // RCC APB2 peripheral clock enabled in low power mode register (RCC_APB2LPENR)
    pub tim1lpen: En,               // TIM1 clock enable during Sleep mode
    pub tim8lpen: En,               // TIM8 clock enable during Sleep mode
    apb2lpenr_bit2: [bit::N1; 2],   // Reserved, must be kept at reset value
    pub usart1lpen: En,             // USART1 clock enable during Sleep mode
    pub usart6lpen: En,             // USART6 clock enable during Sleep mode
    apb2lpenr_bit6: [bit::N1; 2],   // Reserved, must be kept at reset value
    pub adc1lpen: En,               // ADC1 clock enable during Sleep mode
    pub adc2lpen: En,               // ADC2 clock enable during Sleep mode
    pub adc3lpen: En,               // ADC3 clock enable during Sleep mode
    pub sdiolpen: En,               // SDIO clock enable during Sleep mode
    pub spi1lpen: En,               // SPI1 clock enable during Sleep mode
    apb2lpenr_bit13: bit::N1,       // Reserved, must be kept at reset value
    pub syscfglpen: En,             // SYSCFGEN clock enable during Sleep mode
    apb2lpenr_bit15: bit::N1,       // Reserved, must be kept at reset value
    pub tim9lpen: En,               // TIM9 clock enable during Sleep mode
    pub tim10lpen: En,              // TIM10 clock enable during Sleep mode
    pub tim11lpen: En,              // TIM11 clock enable during Sleep mode
    apb2lpenr_bit19: [bit::N1; 13], // Reserved, must be kept at reset value

    reserved_0x68: bit::N32,
    reserved_0x6c: bit::N32,

    // Backup domain control register (RCC_BDCR)
    pub lseon: bit::Rw1,       // Independent watchdog reset flag
    pub lserdy: bit::R1,       // Window watchdog reset flag
    pub lsebyp: bit::Rw1,      // Low-power reset flag
    bdcr_bit3: [bit::N1; 5],   // Reserved, must be kept at reset value
    pub rtsel: bit::Rw2,       // RTC clock source selection
    bdcr_bit10: [bit::N1; 5],  // Reserved, must be kept at reset value
    pub rtcen: bit::Rw1,       // RTC clock enable
    pub bdrst: bit::Rw1,       // Backup domain software reset
    bdcr_bit17: [bit::N1; 15], // Reserved, must be kept at reset value

    // clock control & status register (RCC_CSR)
    pub lsion: bit::Rw1,     // Internal low-speed oscillator enable
    pub lsirdy: bit::R1,     // Internal low-speed oscillator ready
    csr_bit2: [bit::N1; 22], // Reserved, must be kept at reset value
    pub rmvf: bit::Rw1,      // Remove reset flag
    pub borrstf: bit::R1,    // BOR reset flag
    pub pinrstf: bit::R1,    // PIN reset flag
    pub porrstf: bit::R1,    // POR/PDR reset flag
    pub sftrstf: bit::R1,    // Software reset flag
    pub iwdgrstf: bit::R1,   // Independent watchdog reset flag
    pub wwdgrstf: bit::R1,   // Window watchdog reset flag
    pub lpwrrstf: bit::R1,   // Low-power reset flag

    reserved_0x78: bit::N32,
    reserved_0x7c: bit::N32,

    // spread spectrum clock generation register (RCC_SSCGR)
    pub modper: bit::Rw13,     // Modulation period
    pub incstep: bit::Rw15,    // Incrementation step
    sscgr_bit28: [bit::N1; 2], // Reserved, must be kept at reset value
    pub spreadsel: bit::Rw1,   // Spread Select
    pub sscgen: bit::Rw1,      // Spread spectrum modulation enable

    // PLLI2S configuration register (RCC_PLLI2SCFGR)
    pub plli2scfgr_bit0: [bit::N1; 6], // Reserved, must be kept at reset value
    pub plli2sn: bit::Rw9,             // PLLI2S multiplication factor for VCO
    pub plli2scfgr_bit15: [bit::N1; 13], // Reserved, must be kept at reset value
    pub plli2sr: bit::Rw3,             // PLLI2S division factor for I2S clocks
    pub plli2scfgr_bit31: bit::N1,     // Reserved, must be kept at reset value
}

pub fn base() -> &'static Rcc {
    unsafe { &(*(base::RCC_BASE.bit_band_base() as *mut Rcc)) }
}

// Temporary test method
// #[cfg(test)]
// mod tests {
// #[test]
pub fn unit_test() {
    assert_eq!(0x88 * base::PERIPH_REG_BITS, mem::size_of::<Rcc>());

    // // let en: En;
    // // let mut value = false;

    // // en.set(value);
    // // assert_eq!(value, en.get());

    // // value = true;
    // // en.set(value);
    // // assert_eq!(value, en.get());
}
//}
