/* Auto-generated from SVD: cmsis-svd-stm32/stm32f4/STM32F407.svd */
#ifndef STM32F407_H
#define STM32F407_H

#include <stdint.h>
#include "cortex_m4.h"
typedef struct {
    volatile uint32_t CR          ; // 0x00 control register
    volatile uint32_t SR          ; // 0x04 status register
    volatile uint32_t DR          ; // 0x08 data register
} RNG_TypeDef;

#define RNG   ((RNG_TypeDef *) 0x50060800UL)

typedef struct {
    volatile uint32_t CR          ; // 0x00 control register 1
    volatile uint32_t SR          ; // 0x04 status register
    volatile uint32_t RIS         ; // 0x08 raw interrupt status register
    volatile uint32_t IER         ; // 0x0C interrupt enable register
    volatile uint32_t MIS         ; // 0x10 masked interrupt status register
    volatile uint32_t ICR         ; // 0x14 interrupt clear register
    volatile uint32_t ESCR        ; // 0x18 embedded synchronization code register
    volatile uint32_t ESUR        ; // 0x1C embedded synchronization unmask register
    volatile uint32_t CWSTRT      ; // 0x20 crop window start
    volatile uint32_t CWSIZE      ; // 0x24 crop window size
    volatile uint32_t DR          ; // 0x28 data register
} DCMI_TypeDef;

#define DCMI   ((DCMI_TypeDef *) 0x50050000UL)

typedef struct {
    volatile uint32_t BCR1        ; // 0x00 SRAM/NOR-Flash chip-select control register 1
    volatile uint32_t BTR1        ; // 0x04 SRAM/NOR-Flash chip-select timing register 1
    volatile uint32_t BCR2        ; // 0x08 SRAM/NOR-Flash chip-select control register 2
    volatile uint32_t BTR2        ; // 0x0C SRAM/NOR-Flash chip-select timing register 2
    volatile uint32_t BCR3        ; // 0x10 SRAM/NOR-Flash chip-select control register 3
    volatile uint32_t BTR3        ; // 0x14 SRAM/NOR-Flash chip-select timing register 3
    volatile uint32_t BCR4        ; // 0x18 SRAM/NOR-Flash chip-select control register 4
    volatile uint32_t BTR4        ; // 0x1C SRAM/NOR-Flash chip-select timing register 4
    volatile uint32_t RESERVED_0[16]; // 0x20 - 0x5F
    volatile uint32_t PCR2        ; // 0x60 PC Card/NAND Flash control register 2
    volatile uint32_t SR2         ; // 0x64 FIFO status and interrupt register 2
    volatile uint32_t PMEM2       ; // 0x68 Common memory space timing register 2
    volatile uint32_t PATT2       ; // 0x6C Attribute memory space timing register 2
    volatile uint32_t RESERVED_1[1]; // 0x70 - 0x73
    volatile uint32_t ECCR2       ; // 0x74 ECC result register 2
    volatile uint32_t RESERVED_2[2]; // 0x78 - 0x7F
    volatile uint32_t PCR3        ; // 0x80 PC Card/NAND Flash control register 3
    volatile uint32_t SR3         ; // 0x84 FIFO status and interrupt register 3
    volatile uint32_t PMEM3       ; // 0x88 Common memory space timing register 3
    volatile uint32_t PATT3       ; // 0x8C Attribute memory space timing register 3
    volatile uint32_t RESERVED_3[1]; // 0x90 - 0x93
    volatile uint32_t ECCR3       ; // 0x94 ECC result register 3
    volatile uint32_t RESERVED_4[2]; // 0x98 - 0x9F
    volatile uint32_t PCR4        ; // 0xA0 PC Card/NAND Flash control register 4
    volatile uint32_t SR4         ; // 0xA4 FIFO status and interrupt register 4
    volatile uint32_t PMEM4       ; // 0xA8 Common memory space timing register 4
    volatile uint32_t PATT4       ; // 0xAC Attribute memory space timing register 4
    volatile uint32_t PIO4        ; // 0xB0 I/O space timing register 4
    volatile uint32_t RESERVED_5[20]; // 0xB4 - 0x103
    volatile uint32_t BWTR1       ; // 0x104 SRAM/NOR-Flash write timing registers 1
    volatile uint32_t BWTR3       ; // 0x104 SRAM/NOR-Flash write timing registers 3
    volatile uint32_t RESERVED_6[1]; // 0x108 - 0x10B
    volatile uint32_t BWTR2       ; // 0x10C SRAM/NOR-Flash write timing registers 2
    volatile uint32_t BWTR4       ; // 0x10C SRAM/NOR-Flash write timing registers 4
    volatile uint32_t RESERVED_7[12]; // 0x110 - 0x13F
    volatile uint32_t SDCR1       ; // 0x140 SDRAM Control Register 1
    volatile uint32_t SDCR2       ; // 0x144 SDRAM Control Register 2
    volatile uint32_t SDTR1       ; // 0x148 SDRAM Timing register 1
    volatile uint32_t SDTR2       ; // 0x14C SDRAM Timing register 2
    volatile uint32_t SDCMR       ; // 0x150 SDRAM Command Mode register
    volatile uint32_t SDRTR       ; // 0x154 SDRAM Refresh Timer register
    volatile uint32_t SDSR        ; // 0x158 SDRAM Status register
} FSMC_TypeDef;

#define FSMC   ((FSMC_TypeDef *) 0xA0000000UL)

typedef struct {
    volatile uint32_t DBGMCU_IDCODE; // 0x00 IDCODE
    volatile uint32_t DBGMCU_CR   ; // 0x04 Control Register
    volatile uint32_t DBGMCU_APB1_FZ; // 0x08 Debug MCU APB1 Freeze registe
    volatile uint32_t DBGMCU_APB2_FZ; // 0x0C Debug MCU APB2 Freeze registe
} DBG_TypeDef;

#define DBG   ((DBG_TypeDef *) 0xE0042000UL)

typedef struct {
    volatile uint32_t LISR        ; // 0x00 low interrupt status register
    volatile uint32_t HISR        ; // 0x04 high interrupt status register
    volatile uint32_t LIFCR       ; // 0x08 low interrupt flag clear register
    volatile uint32_t HIFCR       ; // 0x0C high interrupt flag clear register
    volatile uint32_t S0CR        ; // 0x10 stream configuration register
    volatile uint32_t S0NDTR      ; // 0x14 stream number of data register
    volatile uint32_t S0PAR       ; // 0x18 stream peripheral address register
    volatile uint32_t S0M0AR      ; // 0x1C stream memory 0 address register
    volatile uint32_t S0M1AR      ; // 0x20 stream memory 1 address register
    volatile uint32_t S0FCR       ; // 0x24 stream FIFO control register
    volatile uint32_t S1CR        ; // 0x28 stream configuration register
    volatile uint32_t S1NDTR      ; // 0x2C stream number of data register
    volatile uint32_t S1PAR       ; // 0x30 stream peripheral address register
    volatile uint32_t S1M0AR      ; // 0x34 stream memory 0 address register
    volatile uint32_t S1M1AR      ; // 0x38 stream memory 1 address register
    volatile uint32_t S1FCR       ; // 0x3C stream FIFO control register
    volatile uint32_t S2CR        ; // 0x40 stream configuration register
    volatile uint32_t S2NDTR      ; // 0x44 stream number of data register
    volatile uint32_t S2PAR       ; // 0x48 stream peripheral address register
    volatile uint32_t S2M0AR      ; // 0x4C stream memory 0 address register
    volatile uint32_t S2M1AR      ; // 0x50 stream memory 1 address register
    volatile uint32_t S2FCR       ; // 0x54 stream FIFO control register
    volatile uint32_t S3CR        ; // 0x58 stream configuration register
    volatile uint32_t S3NDTR      ; // 0x5C stream number of data register
    volatile uint32_t S3PAR       ; // 0x60 stream peripheral address register
    volatile uint32_t S3M0AR      ; // 0x64 stream memory 0 address register
    volatile uint32_t S3M1AR      ; // 0x68 stream memory 1 address register
    volatile uint32_t S3FCR       ; // 0x6C stream FIFO control register
    volatile uint32_t S4CR        ; // 0x70 stream configuration register
    volatile uint32_t S4NDTR      ; // 0x74 stream number of data register
    volatile uint32_t S4PAR       ; // 0x78 stream peripheral address register
    volatile uint32_t S4M0AR      ; // 0x7C stream memory 0 address register
    volatile uint32_t S4M1AR      ; // 0x80 stream memory 1 address register
    volatile uint32_t S4FCR       ; // 0x84 stream FIFO control register
    volatile uint32_t S5CR        ; // 0x88 stream configuration register
    volatile uint32_t S5NDTR      ; // 0x8C stream number of data register
    volatile uint32_t S5PAR       ; // 0x90 stream peripheral address register
    volatile uint32_t S5M0AR      ; // 0x94 stream memory 0 address register
    volatile uint32_t S5M1AR      ; // 0x98 stream memory 1 address register
    volatile uint32_t S5FCR       ; // 0x9C stream FIFO control register
    volatile uint32_t S6CR        ; // 0xA0 stream configuration register
    volatile uint32_t S6NDTR      ; // 0xA4 stream number of data register
    volatile uint32_t S6PAR       ; // 0xA8 stream peripheral address register
    volatile uint32_t S6M0AR      ; // 0xAC stream memory 0 address register
    volatile uint32_t S6M1AR      ; // 0xB0 stream memory 1 address register
    volatile uint32_t S6FCR       ; // 0xB4 stream FIFO control register
    volatile uint32_t S7CR        ; // 0xB8 stream configuration register
    volatile uint32_t S7NDTR      ; // 0xBC stream number of data register
    volatile uint32_t S7PAR       ; // 0xC0 stream peripheral address register
    volatile uint32_t S7M0AR      ; // 0xC4 stream memory 0 address register
    volatile uint32_t S7M1AR      ; // 0xC8 stream memory 1 address register
    volatile uint32_t S7FCR       ; // 0xCC stream FIFO control register
} DMA2_TypeDef;

#define DMA2   ((DMA2_TypeDef *) 0x40026400UL)

typedef struct {
    volatile uint32_t CR          ; // 0x00 clock control register
    volatile uint32_t PLLCFGR     ; // 0x04 PLL configuration register
    volatile uint32_t CFGR        ; // 0x08 clock configuration register
    volatile uint32_t CIR         ; // 0x0C clock interrupt register
    volatile uint32_t AHB1RSTR    ; // 0x10 AHB1 peripheral reset register
    volatile uint32_t AHB2RSTR    ; // 0x14 AHB2 peripheral reset register
    volatile uint32_t AHB3RSTR    ; // 0x18 AHB3 peripheral reset register
    volatile uint32_t RESERVED_0[1]; // 0x1C - 0x1F
    volatile uint32_t APB1RSTR    ; // 0x20 APB1 peripheral reset register
    volatile uint32_t APB2RSTR    ; // 0x24 APB2 peripheral reset register
    volatile uint32_t RESERVED_1[2]; // 0x28 - 0x2F
    volatile uint32_t AHB1ENR     ; // 0x30 AHB1 peripheral clock register
    volatile uint32_t AHB2ENR     ; // 0x34 AHB2 peripheral clock enable register
    volatile uint32_t AHB3ENR     ; // 0x38 AHB3 peripheral clock enable register
    volatile uint32_t RESERVED_2[1]; // 0x3C - 0x3F
    volatile uint32_t APB1ENR     ; // 0x40 APB1 peripheral clock enable register
    volatile uint32_t APB2ENR     ; // 0x44 APB2 peripheral clock enable register
    volatile uint32_t RESERVED_3[2]; // 0x48 - 0x4F
    volatile uint32_t AHB1LPENR   ; // 0x50 AHB1 peripheral clock enable in low power mode register
    volatile uint32_t AHB2LPENR   ; // 0x54 AHB2 peripheral clock enable in low power mode register
    volatile uint32_t AHB3LPENR   ; // 0x58 AHB3 peripheral clock enable in low power mode register
    volatile uint32_t RESERVED_4[1]; // 0x5C - 0x5F
    volatile uint32_t APB1LPENR   ; // 0x60 APB1 peripheral clock enable in low power mode register
    volatile uint32_t APB2LPENR   ; // 0x64 APB2 peripheral clock enabled in low power mode register
    volatile uint32_t RESERVED_5[2]; // 0x68 - 0x6F
    volatile uint32_t BDCR        ; // 0x70 Backup domain control register
    volatile uint32_t CSR         ; // 0x74 clock control & status register
    volatile uint32_t RESERVED_6[2]; // 0x78 - 0x7F
    volatile uint32_t SSCGR       ; // 0x80 spread spectrum clock generation register
    volatile uint32_t PLLI2SCFGR  ; // 0x84 PLLI2S configuration register
} RCC_TypeDef;

#define RCC   ((RCC_TypeDef *) 0x40023800UL)

typedef struct {
    volatile uint32_t MODER       ; // 0x00 GPIO port mode register
    volatile uint32_t OTYPER      ; // 0x04 GPIO port output type register
    volatile uint32_t OSPEEDR     ; // 0x08 GPIO port output speed register
    volatile uint32_t PUPDR       ; // 0x0C GPIO port pull-up/pull-down register
    volatile uint32_t IDR         ; // 0x10 GPIO port input data register
    volatile uint32_t ODR         ; // 0x14 GPIO port output data register
    volatile uint32_t BSRR        ; // 0x18 GPIO port bit set/reset register
    volatile uint32_t LCKR        ; // 0x1C GPIO port configuration lock register
    volatile uint32_t AFRL        ; // 0x20 GPIO alternate function low register
    volatile uint32_t AFRH        ; // 0x24 GPIO alternate function high register
} GPIO_TypeDef;

#define GPIOA   ((GPIO_TypeDef *) 0x40020000UL)
#define GPIOB   ((GPIO_TypeDef *) 0x40020400UL)
#define GPIOC   ((GPIO_TypeDef *) 0x40020800UL)
#define GPIOD   ((GPIO_TypeDef *) 0x40020C00UL)
#define GPIOE   ((GPIO_TypeDef *) 0x40021000UL)
#define GPIOF   ((GPIO_TypeDef *) 0x40021400UL)
#define GPIOG   ((GPIO_TypeDef *) 0x40021800UL)
#define GPIOH   ((GPIO_TypeDef *) 0x40021C00UL)
#define GPIOI   ((GPIO_TypeDef *) 0x40022000UL)
#define GPIOJ   ((GPIO_TypeDef *) 0x40022400UL)
#define GPIOK   ((GPIO_TypeDef *) 0x40022800UL)

extern GPIO_TypeDef * const GpioPorts[];

typedef struct {
    volatile uint32_t MEMRM       ; // 0x00 memory remap register
    volatile uint32_t PMC         ; // 0x04 peripheral mode configuration register
    volatile uint32_t EXTICR1     ; // 0x08 external interrupt configuration register 1
    volatile uint32_t EXTICR2     ; // 0x0C external interrupt configuration register 2
    volatile uint32_t EXTICR3     ; // 0x10 external interrupt configuration register 3
    volatile uint32_t EXTICR4     ; // 0x14 external interrupt configuration register 4
    volatile uint32_t RESERVED_0[2]; // 0x18 - 0x1F
    volatile uint32_t CMPCR       ; // 0x20 Compensation cell control register
} SYSCFG_TypeDef;

#define SYSCFG   ((SYSCFG_TypeDef *) 0x40013800UL)

typedef struct {
    volatile uint32_t CR1         ; // 0x00 control register 1
    volatile uint32_t CR2         ; // 0x04 control register 2
    volatile uint32_t SR          ; // 0x08 status register
    volatile uint32_t DR          ; // 0x0C data register
    volatile uint32_t CRCPR       ; // 0x10 CRC polynomial register
    volatile uint32_t RXCRCR      ; // 0x14 RX CRC register
    volatile uint32_t TXCRCR      ; // 0x18 TX CRC register
    volatile uint32_t I2SCFGR     ; // 0x1C I2S configuration register
    volatile uint32_t I2SPR       ; // 0x20 I2S prescaler register
} SPI1_TypeDef;

#define SPI1   ((SPI1_TypeDef *) 0x40013000UL)

typedef struct {
    volatile uint32_t POWER       ; // 0x00 power control register
    volatile uint32_t CLKCR       ; // 0x04 SDI clock control register
    volatile uint32_t ARG         ; // 0x08 argument register
    volatile uint32_t CMD         ; // 0x0C command register
    volatile uint32_t RESPCMD     ; // 0x10 command response register
    volatile uint32_t RESP1       ; // 0x14 response 1..4 register
    volatile uint32_t RESP2       ; // 0x18 response 1..4 register
    volatile uint32_t RESP3       ; // 0x1C response 1..4 register
    volatile uint32_t RESP4       ; // 0x20 response 1..4 register
    volatile uint32_t DTIMER      ; // 0x24 data timer register
    volatile uint32_t DLEN        ; // 0x28 data length register
    volatile uint32_t DCTRL       ; // 0x2C data control register
    volatile uint32_t DCOUNT      ; // 0x30 data counter register
    volatile uint32_t STA         ; // 0x34 status register
    volatile uint32_t ICR         ; // 0x38 interrupt clear register
    volatile uint32_t MASK        ; // 0x3C mask register
    volatile uint32_t RESERVED_0[2]; // 0x40 - 0x47
    volatile uint32_t FIFOCNT     ; // 0x48 FIFO counter register
    volatile uint32_t RESERVED_1[13]; // 0x4C - 0x7F
    volatile uint32_t FIFO        ; // 0x80 data FIFO register
} SDIO_TypeDef;

#define SDIO   ((SDIO_TypeDef *) 0x40012C00UL)

typedef struct {
    volatile uint32_t SR          ; // 0x00 status register
    volatile uint32_t CR1         ; // 0x04 control register 1
    volatile uint32_t CR2         ; // 0x08 control register 2
    volatile uint32_t SMPR1       ; // 0x0C sample time register 1
    volatile uint32_t SMPR2       ; // 0x10 sample time register 2
    volatile uint32_t JOFR1       ; // 0x14 injected channel data offset register
    volatile uint32_t JOFR2       ; // 0x18 injected channel data offset register
    volatile uint32_t JOFR3       ; // 0x1C injected channel data offset register
    volatile uint32_t JOFR4       ; // 0x20 injected channel data offset register
    volatile uint32_t HTR         ; // 0x24 watchdog higher threshold register
    volatile uint32_t LTR         ; // 0x28 watchdog lower threshold register
    volatile uint32_t SQR1        ; // 0x2C regular sequence register 1
    volatile uint32_t SQR2        ; // 0x30 regular sequence register 2
    volatile uint32_t SQR3        ; // 0x34 regular sequence register 3
    volatile uint32_t JSQR        ; // 0x38 injected sequence register
    volatile uint32_t JDR1        ; // 0x3C injected data register
    volatile uint32_t JDR2        ; // 0x40 injected data register
    volatile uint32_t JDR3        ; // 0x44 injected data register
    volatile uint32_t JDR4        ; // 0x48 injected data register
    volatile uint32_t DR          ; // 0x4C regular data register
} ADC1_TypeDef;

#define ADC1   ((ADC1_TypeDef *) 0x40012000UL)

typedef struct {
    volatile uint32_t SR          ; // 0x00 Status register
    volatile uint32_t DR          ; // 0x04 Data register
    volatile uint32_t BRR         ; // 0x08 Baud rate register
    volatile uint32_t CR1         ; // 0x0C Control register 1
    volatile uint32_t CR2         ; // 0x10 Control register 2
    volatile uint32_t CR3         ; // 0x14 Control register 3
    volatile uint32_t GTPR        ; // 0x18 Guard time and prescaler register
} USART6_TypeDef;

#define USART6   ((USART6_TypeDef *) 0x40011400UL)

typedef struct {
    volatile uint32_t CR          ; // 0x00 control register
    volatile uint32_t SWTRIGR     ; // 0x04 software trigger register
    volatile uint32_t DHR12R1     ; // 0x08 channel1 12-bit right-aligned data holding register
    volatile uint32_t DHR12L1     ; // 0x0C channel1 12-bit left aligned data holding register
    volatile uint32_t DHR8R1      ; // 0x10 channel1 8-bit right aligned data holding register
    volatile uint32_t DHR12R2     ; // 0x14 channel2 12-bit right aligned data holding register
    volatile uint32_t DHR12L2     ; // 0x18 channel2 12-bit left aligned data holding register
    volatile uint32_t DHR8R2      ; // 0x1C channel2 8-bit right-aligned data holding register
    volatile uint32_t DHR12RD     ; // 0x20 Dual DAC 12-bit right-aligned data holding register
    volatile uint32_t DHR12LD     ; // 0x24 DUAL DAC 12-bit left aligned data holding register
    volatile uint32_t DHR8RD      ; // 0x28 DUAL DAC 8-bit right aligned data holding register
    volatile uint32_t DOR1        ; // 0x2C channel1 data output register
    volatile uint32_t DOR2        ; // 0x30 channel2 data output register
    volatile uint32_t SR          ; // 0x34 status register
} DAC_TypeDef;

#define DAC   ((DAC_TypeDef *) 0x40007400UL)

typedef struct {
    volatile uint32_t CR          ; // 0x00 power control register
    volatile uint32_t CSR         ; // 0x04 power control/status register
} PWR_TypeDef;

#define PWR   ((PWR_TypeDef *) 0x40007000UL)

typedef struct {
    volatile uint32_t CR1         ; // 0x00 Control register 1
    volatile uint32_t CR2         ; // 0x04 Control register 2
    volatile uint32_t OAR1        ; // 0x08 Own address register 1
    volatile uint32_t OAR2        ; // 0x0C Own address register 2
    volatile uint32_t DR          ; // 0x10 Data register
    volatile uint32_t SR1         ; // 0x14 Status register 1
    volatile uint32_t SR2         ; // 0x18 Status register 2
    volatile uint32_t CCR         ; // 0x1C Clock control register
    volatile uint32_t TRISE       ; // 0x20 TRISE register
} I2C3_TypeDef;

#define I2C3   ((I2C3_TypeDef *) 0x40005C00UL)

typedef struct {
    volatile uint32_t KR          ; // 0x00 Key register
    volatile uint32_t PR          ; // 0x04 Prescaler register
    volatile uint32_t RLR         ; // 0x08 Reload register
    volatile uint32_t SR          ; // 0x0C Status register
} IWDG_TypeDef;

#define IWDG   ((IWDG_TypeDef *) 0x40003000UL)

typedef struct {
    volatile uint32_t CR          ; // 0x00 Control register
    volatile uint32_t CFR         ; // 0x04 Configuration register
    volatile uint32_t SR          ; // 0x08 Status register
} WWDG_TypeDef;

#define WWDG   ((WWDG_TypeDef *) 0x40002C00UL)

typedef struct {
    volatile uint32_t TR          ; // 0x00 time register
    volatile uint32_t DR          ; // 0x04 date register
    volatile uint32_t CR          ; // 0x08 control register
    volatile uint32_t ISR         ; // 0x0C initialization and status register
    volatile uint32_t PRER        ; // 0x10 prescaler register
    volatile uint32_t WUTR        ; // 0x14 wakeup timer register
    volatile uint32_t CALIBR      ; // 0x18 calibration register
    volatile uint32_t ALRMAR      ; // 0x1C alarm A register
    volatile uint32_t ALRMBR      ; // 0x20 alarm B register
    volatile uint32_t WPR         ; // 0x24 write protection register
    volatile uint32_t SSR         ; // 0x28 sub second register
    volatile uint32_t SHIFTR      ; // 0x2C shift control register
    volatile uint32_t TSTR        ; // 0x30 time stamp time register
    volatile uint32_t TSDR        ; // 0x34 time stamp date register
    volatile uint32_t TSSSR       ; // 0x38 timestamp sub second register
    volatile uint32_t CALR        ; // 0x3C calibration register
    volatile uint32_t TAFCR       ; // 0x40 tamper and alternate function configuration register
    volatile uint32_t ALRMASSR    ; // 0x44 alarm A sub second register
    volatile uint32_t ALRMBSSR    ; // 0x48 alarm B sub second register
    volatile uint32_t RESERVED_0[1]; // 0x4C - 0x4F
    volatile uint32_t BKP0R       ; // 0x50 backup register
    volatile uint32_t BKP1R       ; // 0x54 backup register
    volatile uint32_t BKP2R       ; // 0x58 backup register
    volatile uint32_t BKP3R       ; // 0x5C backup register
    volatile uint32_t BKP4R       ; // 0x60 backup register
    volatile uint32_t BKP5R       ; // 0x64 backup register
    volatile uint32_t BKP6R       ; // 0x68 backup register
    volatile uint32_t BKP7R       ; // 0x6C backup register
    volatile uint32_t BKP8R       ; // 0x70 backup register
    volatile uint32_t BKP9R       ; // 0x74 backup register
    volatile uint32_t BKP10R      ; // 0x78 backup register
    volatile uint32_t BKP11R      ; // 0x7C backup register
    volatile uint32_t BKP12R      ; // 0x80 backup register
    volatile uint32_t BKP13R      ; // 0x84 backup register
    volatile uint32_t BKP14R      ; // 0x88 backup register
    volatile uint32_t BKP15R      ; // 0x8C backup register
    volatile uint32_t BKP16R      ; // 0x90 backup register
    volatile uint32_t BKP17R      ; // 0x94 backup register
    volatile uint32_t BKP18R      ; // 0x98 backup register
    volatile uint32_t BKP19R      ; // 0x9C backup register
} RTC_TypeDef;

#define RTC   ((RTC_TypeDef *) 0x40002800UL)

typedef struct {
    volatile uint32_t SR          ; // 0x00 Status register
    volatile uint32_t DR          ; // 0x04 Data register
    volatile uint32_t BRR         ; // 0x08 Baud rate register
    volatile uint32_t CR1         ; // 0x0C Control register 1
    volatile uint32_t CR2         ; // 0x10 Control register 2
    volatile uint32_t CR3         ; // 0x14 Control register 3
} UART4_TypeDef;

#define UART4   ((UART4_TypeDef *) 0x40004C00UL)

typedef struct {
    volatile uint32_t CSR         ; // 0x00 ADC Common status register
    volatile uint32_t CCR         ; // 0x04 ADC common control register
    volatile uint32_t CDR         ; // 0x08 ADC common regular data register for dual and triple modes
} C_ADC_TypeDef;

#define C_ADC   ((C_ADC_TypeDef *) 0x40012300UL)

typedef struct {
    volatile uint32_t CR1         ; // 0x00 control register 1
    volatile uint32_t CR2         ; // 0x04 control register 2
    volatile uint32_t SMCR        ; // 0x08 slave mode control register
    volatile uint32_t DIER        ; // 0x0C DMA/Interrupt enable register
    volatile uint32_t SR          ; // 0x10 status register
    volatile uint32_t EGR         ; // 0x14 event generation register
    volatile uint32_t CCMR1_Output; // 0x18 capture/compare mode register 1 (output mode)
    volatile uint32_t CCMR1_Input ; // 0x18 capture/compare mode register 1 (input mode)
    volatile uint32_t CCMR2_Output; // 0x1C capture/compare mode register 2 (output mode)
    volatile uint32_t CCMR2_Input ; // 0x1C capture/compare mode register 2 (input mode)
    volatile uint32_t CCER        ; // 0x20 capture/compare enable register
    volatile uint32_t CNT         ; // 0x24 counter
    volatile uint32_t PSC         ; // 0x28 prescaler
    volatile uint32_t ARR         ; // 0x2C auto-reload register
    volatile uint32_t RCR         ; // 0x30 repetition counter register
    volatile uint32_t CCR1        ; // 0x34 capture/compare register 1
    volatile uint32_t CCR2        ; // 0x38 capture/compare register 2
    volatile uint32_t CCR3        ; // 0x3C capture/compare register 3
    volatile uint32_t CCR4        ; // 0x40 capture/compare register 4
    volatile uint32_t BDTR        ; // 0x44 break and dead-time register
    volatile uint32_t DCR         ; // 0x48 DMA control register
    volatile uint32_t DMAR        ; // 0x4C DMA address for full transfer
} TIM1_TypeDef;

#define TIM1   ((TIM1_TypeDef *) 0x40010000UL)

typedef struct {
    volatile uint32_t CR1         ; // 0x00 control register 1
    volatile uint32_t CR2         ; // 0x04 control register 2
    volatile uint32_t SMCR        ; // 0x08 slave mode control register
    volatile uint32_t DIER        ; // 0x0C DMA/Interrupt enable register
    volatile uint32_t SR          ; // 0x10 status register
    volatile uint32_t EGR         ; // 0x14 event generation register
    volatile uint32_t CCMR1_Output; // 0x18 capture/compare mode register 1 (output mode)
    volatile uint32_t CCMR1_Input ; // 0x18 capture/compare mode register 1 (input mode)
    volatile uint32_t CCMR2_Output; // 0x1C capture/compare mode register 2 (output mode)
    volatile uint32_t CCMR2_Input ; // 0x1C capture/compare mode register 2 (input mode)
    volatile uint32_t CCER        ; // 0x20 capture/compare enable register
    volatile uint32_t CNT         ; // 0x24 counter
    volatile uint32_t PSC         ; // 0x28 prescaler
    volatile uint32_t ARR         ; // 0x2C auto-reload register
    volatile uint32_t RESERVED_0[1]; // 0x30 - 0x33
    volatile uint32_t CCR1        ; // 0x34 capture/compare register 1
    volatile uint32_t CCR2        ; // 0x38 capture/compare register 2
    volatile uint32_t CCR3        ; // 0x3C capture/compare register 3
    volatile uint32_t CCR4        ; // 0x40 capture/compare register 4
    volatile uint32_t RESERVED_1[1]; // 0x44 - 0x47
    volatile uint32_t DCR         ; // 0x48 DMA control register
    volatile uint32_t DMAR        ; // 0x4C DMA address for full transfer
    volatile uint32_t OR          ; // 0x50 TIM5 option register
} TIM2_TypeDef;

#define TIM2   ((TIM2_TypeDef *) 0x40000000UL)

typedef struct {
    volatile uint32_t CR1         ; // 0x00 control register 1
    volatile uint32_t CR2         ; // 0x04 control register 2
    volatile uint32_t SMCR        ; // 0x08 slave mode control register
    volatile uint32_t DIER        ; // 0x0C DMA/Interrupt enable register
    volatile uint32_t SR          ; // 0x10 status register
    volatile uint32_t EGR         ; // 0x14 event generation register
    volatile uint32_t CCMR1_Output; // 0x18 capture/compare mode register 1 (output mode)
    volatile uint32_t CCMR1_Input ; // 0x18 capture/compare mode register 1 (input mode)
    volatile uint32_t CCMR2_Output; // 0x1C capture/compare mode register 2 (output mode)
    volatile uint32_t CCMR2_Input ; // 0x1C capture/compare mode register 2 (input mode)
    volatile uint32_t CCER        ; // 0x20 capture/compare enable register
    volatile uint32_t CNT         ; // 0x24 counter
    volatile uint32_t PSC         ; // 0x28 prescaler
    volatile uint32_t ARR         ; // 0x2C auto-reload register
    volatile uint32_t RESERVED_0[1]; // 0x30 - 0x33
    volatile uint32_t CCR1        ; // 0x34 capture/compare register 1
    volatile uint32_t CCR2        ; // 0x38 capture/compare register 2
    volatile uint32_t CCR3        ; // 0x3C capture/compare register 3
    volatile uint32_t CCR4        ; // 0x40 capture/compare register 4
    volatile uint32_t RESERVED_1[1]; // 0x44 - 0x47
    volatile uint32_t DCR         ; // 0x48 DMA control register
    volatile uint32_t DMAR        ; // 0x4C DMA address for full transfer
} TIM3_TypeDef;

#define TIM3   ((TIM3_TypeDef *) 0x40000400UL)

typedef struct {
    volatile uint32_t CR1         ; // 0x00 control register 1
    volatile uint32_t CR2         ; // 0x04 control register 2
    volatile uint32_t SMCR        ; // 0x08 slave mode control register
    volatile uint32_t DIER        ; // 0x0C DMA/Interrupt enable register
    volatile uint32_t SR          ; // 0x10 status register
    volatile uint32_t EGR         ; // 0x14 event generation register
    volatile uint32_t CCMR1_Output; // 0x18 capture/compare mode register 1 (output mode)
    volatile uint32_t CCMR1_Input ; // 0x18 capture/compare mode register 1 (input mode)
    volatile uint32_t CCMR2_Output; // 0x1C capture/compare mode register 2 (output mode)
    volatile uint32_t CCMR2_Input ; // 0x1C capture/compare mode register 2 (input mode)
    volatile uint32_t CCER        ; // 0x20 capture/compare enable register
    volatile uint32_t CNT         ; // 0x24 counter
    volatile uint32_t PSC         ; // 0x28 prescaler
    volatile uint32_t ARR         ; // 0x2C auto-reload register
    volatile uint32_t RESERVED_0[1]; // 0x30 - 0x33
    volatile uint32_t CCR1        ; // 0x34 capture/compare register 1
    volatile uint32_t CCR2        ; // 0x38 capture/compare register 2
    volatile uint32_t CCR3        ; // 0x3C capture/compare register 3
    volatile uint32_t CCR4        ; // 0x40 capture/compare register 4
    volatile uint32_t RESERVED_1[1]; // 0x44 - 0x47
    volatile uint32_t DCR         ; // 0x48 DMA control register
    volatile uint32_t DMAR        ; // 0x4C DMA address for full transfer
    volatile uint32_t OR          ; // 0x50 TIM5 option register
} TIM5_TypeDef;

#define TIM5   ((TIM5_TypeDef *) 0x40000C00UL)

typedef struct {
    volatile uint32_t CR1         ; // 0x00 control register 1
    volatile uint32_t CR2         ; // 0x04 control register 2
    volatile uint32_t SMCR        ; // 0x08 slave mode control register
    volatile uint32_t DIER        ; // 0x0C DMA/Interrupt enable register
    volatile uint32_t SR          ; // 0x10 status register
    volatile uint32_t EGR         ; // 0x14 event generation register
    volatile uint32_t CCMR1_Output; // 0x18 capture/compare mode register 1 (output mode)
    volatile uint32_t CCMR1_Input ; // 0x18 capture/compare mode register 1 (input mode)
    volatile uint32_t RESERVED_0[1]; // 0x1C - 0x1F
    volatile uint32_t CCER        ; // 0x20 capture/compare enable register
    volatile uint32_t CNT         ; // 0x24 counter
    volatile uint32_t PSC         ; // 0x28 prescaler
    volatile uint32_t ARR         ; // 0x2C auto-reload register
    volatile uint32_t RESERVED_1[1]; // 0x30 - 0x33
    volatile uint32_t CCR1        ; // 0x34 capture/compare register 1
    volatile uint32_t CCR2        ; // 0x38 capture/compare register 2
} TIM9_TypeDef;

#define TIM9   ((TIM9_TypeDef *) 0x40014000UL)

typedef struct {
    volatile uint32_t CR1         ; // 0x00 control register 1
    volatile uint32_t RESERVED_0[2]; // 0x04 - 0x0B
    volatile uint32_t DIER        ; // 0x0C DMA/Interrupt enable register
    volatile uint32_t SR          ; // 0x10 status register
    volatile uint32_t EGR         ; // 0x14 event generation register
    volatile uint32_t CCMR1_Output; // 0x18 capture/compare mode register 1 (output mode)
    volatile uint32_t CCMR1_Input ; // 0x18 capture/compare mode register 1 (input mode)
    volatile uint32_t RESERVED_1[1]; // 0x1C - 0x1F
    volatile uint32_t CCER        ; // 0x20 capture/compare enable register
    volatile uint32_t CNT         ; // 0x24 counter
    volatile uint32_t PSC         ; // 0x28 prescaler
    volatile uint32_t ARR         ; // 0x2C auto-reload register
    volatile uint32_t RESERVED_2[1]; // 0x30 - 0x33
    volatile uint32_t CCR1        ; // 0x34 capture/compare register 1
} TIM10_TypeDef;

#define TIM10   ((TIM10_TypeDef *) 0x40014400UL)

typedef struct {
    volatile uint32_t CR1         ; // 0x00 control register 1
    volatile uint32_t RESERVED_0[2]; // 0x04 - 0x0B
    volatile uint32_t DIER        ; // 0x0C DMA/Interrupt enable register
    volatile uint32_t SR          ; // 0x10 status register
    volatile uint32_t EGR         ; // 0x14 event generation register
    volatile uint32_t CCMR1_Output; // 0x18 capture/compare mode register 1 (output mode)
    volatile uint32_t CCMR1_Input ; // 0x18 capture/compare mode register 1 (input mode)
    volatile uint32_t RESERVED_1[1]; // 0x1C - 0x1F
    volatile uint32_t CCER        ; // 0x20 capture/compare enable register
    volatile uint32_t CNT         ; // 0x24 counter
    volatile uint32_t PSC         ; // 0x28 prescaler
    volatile uint32_t ARR         ; // 0x2C auto-reload register
    volatile uint32_t RESERVED_2[1]; // 0x30 - 0x33
    volatile uint32_t CCR1        ; // 0x34 capture/compare register 1
    volatile uint32_t RESERVED_3[6]; // 0x38 - 0x4F
    volatile uint32_t OR          ; // 0x50 option register
} TIM11_TypeDef;

#define TIM11   ((TIM11_TypeDef *) 0x40014800UL)

typedef struct {
    volatile uint32_t CR1         ; // 0x00 control register 1
    volatile uint32_t CR2         ; // 0x04 control register 2
    volatile uint32_t RESERVED_0[1]; // 0x08 - 0x0B
    volatile uint32_t DIER        ; // 0x0C DMA/Interrupt enable register
    volatile uint32_t SR          ; // 0x10 status register
    volatile uint32_t EGR         ; // 0x14 event generation register
    volatile uint32_t RESERVED_1[3]; // 0x18 - 0x23
    volatile uint32_t CNT         ; // 0x24 counter
    volatile uint32_t PSC         ; // 0x28 prescaler
    volatile uint32_t ARR         ; // 0x2C auto-reload register
} TIM6_TypeDef;

#define TIM6   ((TIM6_TypeDef *) 0x40001000UL)

typedef struct {
    volatile uint32_t MACCR       ; // 0x00 Ethernet MAC configuration register
    volatile uint32_t MACFFR      ; // 0x04 Ethernet MAC frame filter register
    volatile uint32_t MACHTHR     ; // 0x08 Ethernet MAC hash table high register
    volatile uint32_t MACHTLR     ; // 0x0C Ethernet MAC hash table low register
    volatile uint32_t MACMIIAR    ; // 0x10 Ethernet MAC MII address register
    volatile uint32_t MACMIIDR    ; // 0x14 Ethernet MAC MII data register
    volatile uint32_t MACFCR      ; // 0x18 Ethernet MAC flow control register
    volatile uint32_t MACVLANTR   ; // 0x1C Ethernet MAC VLAN tag register
    volatile uint32_t RESERVED_0[3]; // 0x20 - 0x2B
    volatile uint32_t MACPMTCSR   ; // 0x2C Ethernet MAC PMT control and status register
    volatile uint32_t RESERVED_1[1]; // 0x30 - 0x33
    volatile uint32_t MACDBGR     ; // 0x34 Ethernet MAC debug register
    volatile uint32_t MACSR       ; // 0x38 Ethernet MAC interrupt status register
    volatile uint32_t MACIMR      ; // 0x3C Ethernet MAC interrupt mask register
    volatile uint32_t MACA0HR     ; // 0x40 Ethernet MAC address 0 high register
    volatile uint32_t MACA0LR     ; // 0x44 Ethernet MAC address 0 low register
    volatile uint32_t MACA1HR     ; // 0x48 Ethernet MAC address 1 high register
    volatile uint32_t MACA1LR     ; // 0x4C Ethernet MAC address1 low register
    volatile uint32_t MACA2HR     ; // 0x50 Ethernet MAC address 2 high register
    volatile uint32_t MACA2LR     ; // 0x54 Ethernet MAC address 2 low register
    volatile uint32_t MACA3HR     ; // 0x58 Ethernet MAC address 3 high register
    volatile uint32_t MACA3LR     ; // 0x5C Ethernet MAC address 3 low register
} ETHERNET_MAC_TypeDef;

#define ETHERNET_MAC   ((ETHERNET_MAC_TypeDef *) 0x40028000UL)

typedef struct {
    volatile uint32_t MMCCR       ; // 0x00 Ethernet MMC control register
    volatile uint32_t MMCRIR      ; // 0x04 Ethernet MMC receive interrupt register
    volatile uint32_t MMCTIR      ; // 0x08 Ethernet MMC transmit interrupt register
    volatile uint32_t MMCRIMR     ; // 0x0C Ethernet MMC receive interrupt mask register
    volatile uint32_t MMCTIMR     ; // 0x10 Ethernet MMC transmit interrupt mask register
    volatile uint32_t RESERVED_0[14]; // 0x14 - 0x4B
    volatile uint32_t MMCTGFSCCR  ; // 0x4C Ethernet MMC transmitted good frames after a single collision counter
    volatile uint32_t MMCTGFMSCCR ; // 0x50 Ethernet MMC transmitted good frames after more than a single collision
    volatile uint32_t RESERVED_1[5]; // 0x54 - 0x67
    volatile uint32_t MMCTGFCR    ; // 0x68 Ethernet MMC transmitted good frames counter register
    volatile uint32_t RESERVED_2[10]; // 0x6C - 0x93
    volatile uint32_t MMCRFCECR   ; // 0x94 Ethernet MMC received frames with CRC error counter register
    volatile uint32_t MMCRFAECR   ; // 0x98 Ethernet MMC received frames with alignment error counter register
    volatile uint32_t RESERVED_3[10]; // 0x9C - 0xC3
    volatile uint32_t MMCRGUFCR   ; // 0xC4 MMC received good unicast frames counter register
} ETHERNET_MMC_TypeDef;

#define ETHERNET_MMC   ((ETHERNET_MMC_TypeDef *) 0x40028100UL)

typedef struct {
    volatile uint32_t PTPTSCR     ; // 0x00 Ethernet PTP time stamp control register
    volatile uint32_t PTPSSIR     ; // 0x04 Ethernet PTP subsecond increment register
    volatile uint32_t PTPTSHR     ; // 0x08 Ethernet PTP time stamp high register
    volatile uint32_t PTPTSLR     ; // 0x0C Ethernet PTP time stamp low register
    volatile uint32_t PTPTSHUR    ; // 0x10 Ethernet PTP time stamp high update register
    volatile uint32_t PTPTSLUR    ; // 0x14 Ethernet PTP time stamp low update register
    volatile uint32_t PTPTSAR     ; // 0x18 Ethernet PTP time stamp addend register
    volatile uint32_t PTPTTHR     ; // 0x1C Ethernet PTP target time high register
    volatile uint32_t PTPTTLR     ; // 0x20 Ethernet PTP target time low register
    volatile uint32_t RESERVED_0[1]; // 0x24 - 0x27
    volatile uint32_t PTPTSSR     ; // 0x28 Ethernet PTP time stamp status register
    volatile uint32_t PTPPPSCR    ; // 0x2C Ethernet PTP PPS control register
} ETHERNET_PTP_TypeDef;

#define ETHERNET_PTP   ((ETHERNET_PTP_TypeDef *) 0x40028700UL)

typedef struct {
    volatile uint32_t DMABMR      ; // 0x00 Ethernet DMA bus mode register
    volatile uint32_t DMATPDR     ; // 0x04 Ethernet DMA transmit poll demand register
    volatile uint32_t DMARPDR     ; // 0x08 EHERNET DMA receive poll demand register
    volatile uint32_t DMARDLAR    ; // 0x0C Ethernet DMA receive descriptor list address register
    volatile uint32_t DMATDLAR    ; // 0x10 Ethernet DMA transmit descriptor list address register
    volatile uint32_t DMASR       ; // 0x14 Ethernet DMA status register
    volatile uint32_t DMAOMR      ; // 0x18 Ethernet DMA operation mode register
    volatile uint32_t DMAIER      ; // 0x1C Ethernet DMA interrupt enable register
    volatile uint32_t DMAMFBOCR   ; // 0x20 Ethernet DMA missed frame and buffer overflow counter register
    volatile uint32_t DMARSWTR    ; // 0x24 Ethernet DMA receive status watchdog timer register
    volatile uint32_t RESERVED_0[8]; // 0x28 - 0x47
    volatile uint32_t DMACHTDR    ; // 0x48 Ethernet DMA current host transmit descriptor register
    volatile uint32_t DMACHRDR    ; // 0x4C Ethernet DMA current host receive descriptor register
    volatile uint32_t DMACHTBAR   ; // 0x50 Ethernet DMA current host transmit buffer address register
    volatile uint32_t DMACHRBAR   ; // 0x54 Ethernet DMA current host receive buffer address register
} ETHERNET_DMA_TypeDef;

#define ETHERNET_DMA   ((ETHERNET_DMA_TypeDef *) 0x40029000UL)

typedef struct {
    volatile uint32_t DR          ; // 0x00 Data register
    volatile uint32_t IDR         ; // 0x04 Independent Data register
    volatile uint32_t CR          ; // 0x08 Control register
} CRC_TypeDef;

#define CRC   ((CRC_TypeDef *) 0x40023000UL)

typedef struct {
    volatile uint32_t FS_GOTGCTL  ; // 0x00 OTG_FS control and status register (OTG_FS_GOTGCTL)
    volatile uint32_t FS_GOTGINT  ; // 0x04 OTG_FS interrupt register (OTG_FS_GOTGINT)
    volatile uint32_t FS_GAHBCFG  ; // 0x08 OTG_FS AHB configuration register (OTG_FS_GAHBCFG)
    volatile uint32_t FS_GUSBCFG  ; // 0x0C OTG_FS USB configuration register (OTG_FS_GUSBCFG)
    volatile uint32_t FS_GRSTCTL  ; // 0x10 OTG_FS reset register (OTG_FS_GRSTCTL)
    volatile uint32_t FS_GINTSTS  ; // 0x14 OTG_FS core interrupt register (OTG_FS_GINTSTS)
    volatile uint32_t FS_GINTMSK  ; // 0x18 OTG_FS interrupt mask register (OTG_FS_GINTMSK)
    volatile uint32_t FS_GRXSTSR_Device; // 0x1C OTG_FS Receive status debug read(Device mode)
    volatile uint32_t FS_GRXSTSR_Host; // 0x1C OTG_FS Receive status debug read(Host mode)
    volatile uint32_t RESERVED_0[1]; // 0x20 - 0x23
    volatile uint32_t FS_GRXFSIZ  ; // 0x24 OTG_FS Receive FIFO size register (OTG_FS_GRXFSIZ)
    volatile uint32_t FS_GNPTXFSIZ_Device; // 0x28 OTG_FS non-periodic transmit FIFO size register (Device mode)
    volatile uint32_t FS_GNPTXFSIZ_Host; // 0x28 OTG_FS non-periodic transmit FIFO size register (Host mode)
    volatile uint32_t FS_GNPTXSTS ; // 0x2C OTG_FS non-periodic transmit FIFO/queue status register (OTG_FS_GNPTXSTS)
    volatile uint32_t RESERVED_1[2]; // 0x30 - 0x37
    volatile uint32_t FS_GCCFG    ; // 0x38 OTG_FS general core configuration register (OTG_FS_GCCFG)
    volatile uint32_t FS_CID      ; // 0x3C core ID register
    volatile uint32_t RESERVED_2[48]; // 0x40 - 0xFF
    volatile uint32_t FS_HPTXFSIZ ; // 0x100 OTG_FS Host periodic transmit FIFO size register (OTG_FS_HPTXFSIZ)
    volatile uint32_t FS_DIEPTXF1 ; // 0x104 OTG_FS device IN endpoint transmit FIFO size register (OTG_FS_DIEPTXF2)
    volatile uint32_t FS_DIEPTXF2 ; // 0x108 OTG_FS device IN endpoint transmit FIFO size register (OTG_FS_DIEPTXF3)
    volatile uint32_t FS_DIEPTXF3 ; // 0x10C OTG_FS device IN endpoint transmit FIFO size register (OTG_FS_DIEPTXF4)
} OTG_FS_GLOBAL_TypeDef;

#define OTG_FS_GLOBAL   ((OTG_FS_GLOBAL_TypeDef *) 0x50000000UL)

typedef struct {
    volatile uint32_t FS_HCFG     ; // 0x00 OTG_FS host configuration register (OTG_FS_HCFG)
    volatile uint32_t HFIR        ; // 0x04 OTG_FS Host frame interval register
    volatile uint32_t FS_HFNUM    ; // 0x08 OTG_FS host frame number/frame time remaining register (OTG_FS_HFNUM)
    volatile uint32_t RESERVED_0[1]; // 0x0C - 0x0F
    volatile uint32_t FS_HPTXSTS  ; // 0x10 OTG_FS_Host periodic transmit FIFO/queue status register (OTG_FS_HPTXSTS)
    volatile uint32_t HAINT       ; // 0x14 OTG_FS Host all channels interrupt register
    volatile uint32_t HAINTMSK    ; // 0x18 OTG_FS host all channels interrupt mask register
    volatile uint32_t RESERVED_1[9]; // 0x1C - 0x3F
    volatile uint32_t FS_HPRT     ; // 0x40 OTG_FS host port control and status register (OTG_FS_HPRT)
    volatile uint32_t RESERVED_2[47]; // 0x44 - 0xFF
    volatile uint32_t FS_HCCHAR0  ; // 0x100 OTG_FS host channel-0 characteristics register (OTG_FS_HCCHAR0)
    volatile uint32_t RESERVED_3[1]; // 0x104 - 0x107
    volatile uint32_t FS_HCINT0   ; // 0x108 OTG_FS host channel-0 interrupt register (OTG_FS_HCINT0)
    volatile uint32_t FS_HCINTMSK0; // 0x10C OTG_FS host channel-0 mask register (OTG_FS_HCINTMSK0)
    volatile uint32_t FS_HCTSIZ0  ; // 0x110 OTG_FS host channel-0 transfer size register
    volatile uint32_t RESERVED_4[3]; // 0x114 - 0x11F
    volatile uint32_t FS_HCCHAR1  ; // 0x120 OTG_FS host channel-1 characteristics register (OTG_FS_HCCHAR1)
    volatile uint32_t RESERVED_5[1]; // 0x124 - 0x127
    volatile uint32_t FS_HCINT1   ; // 0x128 OTG_FS host channel-1 interrupt register (OTG_FS_HCINT1)
    volatile uint32_t FS_HCINTMSK1; // 0x12C OTG_FS host channel-1 mask register (OTG_FS_HCINTMSK1)
    volatile uint32_t FS_HCTSIZ1  ; // 0x130 OTG_FS host channel-1 transfer size register
    volatile uint32_t RESERVED_6[3]; // 0x134 - 0x13F
    volatile uint32_t FS_HCCHAR2  ; // 0x140 OTG_FS host channel-2 characteristics register (OTG_FS_HCCHAR2)
    volatile uint32_t RESERVED_7[1]; // 0x144 - 0x147
    volatile uint32_t FS_HCINT2   ; // 0x148 OTG_FS host channel-2 interrupt register (OTG_FS_HCINT2)
    volatile uint32_t FS_HCINTMSK2; // 0x14C OTG_FS host channel-2 mask register (OTG_FS_HCINTMSK2)
    volatile uint32_t FS_HCTSIZ2  ; // 0x150 OTG_FS host channel-2 transfer size register
    volatile uint32_t RESERVED_8[3]; // 0x154 - 0x15F
    volatile uint32_t FS_HCCHAR3  ; // 0x160 OTG_FS host channel-3 characteristics register (OTG_FS_HCCHAR3)
    volatile uint32_t RESERVED_9[1]; // 0x164 - 0x167
    volatile uint32_t FS_HCINT3   ; // 0x168 OTG_FS host channel-3 interrupt register (OTG_FS_HCINT3)
    volatile uint32_t FS_HCINTMSK3; // 0x16C OTG_FS host channel-3 mask register (OTG_FS_HCINTMSK3)
    volatile uint32_t FS_HCTSIZ3  ; // 0x170 OTG_FS host channel-3 transfer size register
    volatile uint32_t RESERVED_10[3]; // 0x174 - 0x17F
    volatile uint32_t FS_HCCHAR4  ; // 0x180 OTG_FS host channel-4 characteristics register (OTG_FS_HCCHAR4)
    volatile uint32_t RESERVED_11[1]; // 0x184 - 0x187
    volatile uint32_t FS_HCINT4   ; // 0x188 OTG_FS host channel-4 interrupt register (OTG_FS_HCINT4)
    volatile uint32_t FS_HCINTMSK4; // 0x18C OTG_FS host channel-4 mask register (OTG_FS_HCINTMSK4)
    volatile uint32_t FS_HCTSIZ4  ; // 0x190 OTG_FS host channel-transfer size register
    volatile uint32_t RESERVED_12[3]; // 0x194 - 0x19F
    volatile uint32_t FS_HCCHAR5  ; // 0x1A0 OTG_FS host channel-5 characteristics register (OTG_FS_HCCHAR5)
    volatile uint32_t RESERVED_13[1]; // 0x1A4 - 0x1A7
    volatile uint32_t FS_HCINT5   ; // 0x1A8 OTG_FS host channel-5 interrupt register (OTG_FS_HCINT5)
    volatile uint32_t FS_HCINTMSK5; // 0x1AC OTG_FS host channel-5 mask register (OTG_FS_HCINTMSK5)
    volatile uint32_t FS_HCTSIZ5  ; // 0x1B0 OTG_FS host channel-5 transfer size register
    volatile uint32_t RESERVED_14[3]; // 0x1B4 - 0x1BF
    volatile uint32_t FS_HCCHAR6  ; // 0x1C0 OTG_FS host channel-6 characteristics register (OTG_FS_HCCHAR6)
    volatile uint32_t RESERVED_15[1]; // 0x1C4 - 0x1C7
    volatile uint32_t FS_HCINT6   ; // 0x1C8 OTG_FS host channel-6 interrupt register (OTG_FS_HCINT6)
    volatile uint32_t FS_HCINTMSK6; // 0x1CC OTG_FS host channel-6 mask register (OTG_FS_HCINTMSK6)
    volatile uint32_t FS_HCTSIZ6  ; // 0x1D0 OTG_FS host channel-6 transfer size register
    volatile uint32_t RESERVED_16[3]; // 0x1D4 - 0x1DF
    volatile uint32_t FS_HCCHAR7  ; // 0x1E0 OTG_FS host channel-7 characteristics register (OTG_FS_HCCHAR7)
    volatile uint32_t RESERVED_17[1]; // 0x1E4 - 0x1E7
    volatile uint32_t FS_HCINT7   ; // 0x1E8 OTG_FS host channel-7 interrupt register (OTG_FS_HCINT7)
    volatile uint32_t FS_HCINTMSK7; // 0x1EC OTG_FS host channel-7 mask register (OTG_FS_HCINTMSK7)
    volatile uint32_t FS_HCTSIZ7  ; // 0x1F0 OTG_FS host channel-7 transfer size register
} OTG_FS_HOST_TypeDef;

#define OTG_FS_HOST   ((OTG_FS_HOST_TypeDef *) 0x50000400UL)

typedef struct {
    volatile uint32_t FS_DCFG     ; // 0x00 OTG_FS device configuration register (OTG_FS_DCFG)
    volatile uint32_t FS_DCTL     ; // 0x04 OTG_FS device control register (OTG_FS_DCTL)
    volatile uint32_t FS_DSTS     ; // 0x08 OTG_FS device status register (OTG_FS_DSTS)
    volatile uint32_t RESERVED_0[1]; // 0x0C - 0x0F
    volatile uint32_t FS_DIEPMSK  ; // 0x10 OTG_FS device IN endpoint common interrupt mask register (OTG_FS_DIEPMSK)
    volatile uint32_t FS_DOEPMSK  ; // 0x14 OTG_FS device OUT endpoint common interrupt mask register (OTG_FS_DOEPMSK)
    volatile uint32_t FS_DAINT    ; // 0x18 OTG_FS device all endpoints interrupt register (OTG_FS_DAINT)
    volatile uint32_t FS_DAINTMSK ; // 0x1C OTG_FS all endpoints interrupt mask register (OTG_FS_DAINTMSK)
    volatile uint32_t RESERVED_1[2]; // 0x20 - 0x27
    volatile uint32_t DVBUSDIS    ; // 0x28 OTG_FS device VBUS discharge time register
    volatile uint32_t DVBUSPULSE  ; // 0x2C OTG_FS device VBUS pulsing time register
    volatile uint32_t RESERVED_2[1]; // 0x30 - 0x33
    volatile uint32_t DIEPEMPMSK  ; // 0x34 OTG_FS device IN endpoint FIFO empty interrupt mask register
    volatile uint32_t RESERVED_3[50]; // 0x38 - 0xFF
    volatile uint32_t FS_DIEPCTL0 ; // 0x100 OTG_FS device control IN endpoint 0 control register (OTG_FS_DIEPCTL0)
    volatile uint32_t RESERVED_4[1]; // 0x104 - 0x107
    volatile uint32_t DIEPINT0    ; // 0x108 device endpoint-interrupt register
    volatile uint32_t RESERVED_5[1]; // 0x10C - 0x10F
    volatile uint32_t DIEPTSIZ0   ; // 0x110 device endpoint-0 transfer size register
    volatile uint32_t RESERVED_6[1]; // 0x114 - 0x117
    volatile uint32_t DTXFSTS0    ; // 0x118 OTG_FS device IN endpoint transmit FIFO status register
    volatile uint32_t RESERVED_7[1]; // 0x11C - 0x11F
    volatile uint32_t DIEPCTL1    ; // 0x120 OTG device endpoint-1 control register
    volatile uint32_t RESERVED_8[1]; // 0x124 - 0x127
    volatile uint32_t DIEPINT1    ; // 0x128 device endpoint-1 interrupt register
    volatile uint32_t RESERVED_9[1]; // 0x12C - 0x12F
    volatile uint32_t DIEPTSIZ1   ; // 0x130 device endpoint-1 transfer size register
    volatile uint32_t RESERVED_10[1]; // 0x134 - 0x137
    volatile uint32_t DTXFSTS1    ; // 0x138 OTG_FS device IN endpoint transmit FIFO status register
    volatile uint32_t RESERVED_11[1]; // 0x13C - 0x13F
    volatile uint32_t DIEPCTL2    ; // 0x140 OTG device endpoint-2 control register
    volatile uint32_t RESERVED_12[1]; // 0x144 - 0x147
    volatile uint32_t DIEPINT2    ; // 0x148 device endpoint-2 interrupt register
    volatile uint32_t RESERVED_13[1]; // 0x14C - 0x14F
    volatile uint32_t DIEPTSIZ2   ; // 0x150 device endpoint-2 transfer size register
    volatile uint32_t RESERVED_14[1]; // 0x154 - 0x157
    volatile uint32_t DTXFSTS2    ; // 0x158 OTG_FS device IN endpoint transmit FIFO status register
    volatile uint32_t RESERVED_15[1]; // 0x15C - 0x15F
    volatile uint32_t DIEPCTL3    ; // 0x160 OTG device endpoint-3 control register
    volatile uint32_t RESERVED_16[1]; // 0x164 - 0x167
    volatile uint32_t DIEPINT3    ; // 0x168 device endpoint-3 interrupt register
    volatile uint32_t RESERVED_17[1]; // 0x16C - 0x16F
    volatile uint32_t DIEPTSIZ3   ; // 0x170 device endpoint-3 transfer size register
    volatile uint32_t RESERVED_18[1]; // 0x174 - 0x177
    volatile uint32_t DTXFSTS3    ; // 0x178 OTG_FS device IN endpoint transmit FIFO status register
    volatile uint32_t RESERVED_19[97]; // 0x17C - 0x2FF
    volatile uint32_t DOEPCTL0    ; // 0x300 device endpoint-0 control register
    volatile uint32_t RESERVED_20[1]; // 0x304 - 0x307
    volatile uint32_t DOEPINT0    ; // 0x308 device endpoint-0 interrupt register
    volatile uint32_t RESERVED_21[1]; // 0x30C - 0x30F
    volatile uint32_t DOEPTSIZ0   ; // 0x310 device OUT endpoint-0 transfer size register
    volatile uint32_t RESERVED_22[3]; // 0x314 - 0x31F
    volatile uint32_t DOEPCTL1    ; // 0x320 device endpoint-1 control register
    volatile uint32_t RESERVED_23[1]; // 0x324 - 0x327
    volatile uint32_t DOEPINT1    ; // 0x328 device endpoint-1 interrupt register
    volatile uint32_t RESERVED_24[1]; // 0x32C - 0x32F
    volatile uint32_t DOEPTSIZ1   ; // 0x330 device OUT endpoint-1 transfer size register
    volatile uint32_t RESERVED_25[3]; // 0x334 - 0x33F
    volatile uint32_t DOEPCTL2    ; // 0x340 device endpoint-2 control register
    volatile uint32_t RESERVED_26[1]; // 0x344 - 0x347
    volatile uint32_t DOEPINT2    ; // 0x348 device endpoint-2 interrupt register
    volatile uint32_t RESERVED_27[1]; // 0x34C - 0x34F
    volatile uint32_t DOEPTSIZ2   ; // 0x350 device OUT endpoint-2 transfer size register
    volatile uint32_t RESERVED_28[3]; // 0x354 - 0x35F
    volatile uint32_t DOEPCTL3    ; // 0x360 device endpoint-3 control register
    volatile uint32_t RESERVED_29[1]; // 0x364 - 0x367
    volatile uint32_t DOEPINT3    ; // 0x368 device endpoint-3 interrupt register
    volatile uint32_t RESERVED_30[1]; // 0x36C - 0x36F
    volatile uint32_t DOEPTSIZ3   ; // 0x370 device OUT endpoint-3 transfer size register
} OTG_FS_DEVICE_TypeDef;

#define OTG_FS_DEVICE   ((OTG_FS_DEVICE_TypeDef *) 0x50000800UL)

typedef struct {
    volatile uint32_t FS_PCGCCTL  ; // 0x00 OTG_FS power and clock gating control register
} OTG_FS_PWRCLK_TypeDef;

#define OTG_FS_PWRCLK   ((OTG_FS_PWRCLK_TypeDef *) 0x50000E00UL)

typedef struct {
    volatile uint32_t MCR         ; // 0x00 master control register
    volatile uint32_t MSR         ; // 0x04 master status register
    volatile uint32_t TSR         ; // 0x08 transmit status register
    volatile uint32_t RF0R        ; // 0x0C receive FIFO 0 register
    volatile uint32_t RF1R        ; // 0x10 receive FIFO 1 register
    volatile uint32_t IER         ; // 0x14 interrupt enable register
    volatile uint32_t ESR         ; // 0x18 interrupt enable register
    volatile uint32_t BTR         ; // 0x1C bit timing register
    volatile uint32_t RESERVED_0[88]; // 0x20 - 0x17F
    volatile uint32_t TI0R        ; // 0x180 TX mailboidentifier register
    volatile uint32_t TDT0R       ; // 0x184 mailbodata length control and time stamp register
    volatile uint32_t TDL0R       ; // 0x188 mailbodata low register
    volatile uint32_t TDH0R       ; // 0x18C mailbodata high register
    volatile uint32_t TI1R        ; // 0x190 mailboidentifier register
    volatile uint32_t TDT1R       ; // 0x194 mailbodata length control and time stamp register
    volatile uint32_t TDL1R       ; // 0x198 mailbodata low register
    volatile uint32_t TDH1R       ; // 0x19C mailbodata high register
    volatile uint32_t TI2R        ; // 0x1A0 mailboidentifier register
    volatile uint32_t TDT2R       ; // 0x1A4 mailbodata length control and time stamp register
    volatile uint32_t TDL2R       ; // 0x1A8 mailbodata low register
    volatile uint32_t TDH2R       ; // 0x1AC mailbodata high register
    volatile uint32_t RI0R        ; // 0x1B0 receive FIFO mailboidentifier register
    volatile uint32_t RDT0R       ; // 0x1B4 mailbodata high register
    volatile uint32_t RDL0R       ; // 0x1B8 mailbodata high register
    volatile uint32_t RDH0R       ; // 0x1BC receive FIFO mailbodata high register
    volatile uint32_t RI1R        ; // 0x1C0 mailbodata high register
    volatile uint32_t RDT1R       ; // 0x1C4 mailbodata high register
    volatile uint32_t RDL1R       ; // 0x1C8 mailbodata high register
    volatile uint32_t RDH1R       ; // 0x1CC mailbodata high register
    volatile uint32_t RESERVED_1[12]; // 0x1D0 - 0x1FF
    volatile uint32_t FMR         ; // 0x200 filter master register
    volatile uint32_t FM1R        ; // 0x204 filter mode register
    volatile uint32_t RESERVED_2[1]; // 0x208 - 0x20B
    volatile uint32_t FS1R        ; // 0x20C filter scale register
    volatile uint32_t RESERVED_3[1]; // 0x210 - 0x213
    volatile uint32_t FFA1R       ; // 0x214 filter FIFO assignment register
    volatile uint32_t RESERVED_4[1]; // 0x218 - 0x21B
    volatile uint32_t FA1R        ; // 0x21C filter activation register
    volatile uint32_t RESERVED_5[8]; // 0x220 - 0x23F
    volatile uint32_t F0R1        ; // 0x240 Filter bank 0 register 1
    volatile uint32_t F0R2        ; // 0x244 Filter bank 0 register 2
    volatile uint32_t F1R1        ; // 0x248 Filter bank 1 register 1
    volatile uint32_t F1R2        ; // 0x24C Filter bank 1 register 2
    volatile uint32_t F2R1        ; // 0x250 Filter bank 2 register 1
    volatile uint32_t F2R2        ; // 0x254 Filter bank 2 register 2
    volatile uint32_t F3R1        ; // 0x258 Filter bank 3 register 1
    volatile uint32_t F3R2        ; // 0x25C Filter bank 3 register 2
    volatile uint32_t F4R1        ; // 0x260 Filter bank 4 register 1
    volatile uint32_t F4R2        ; // 0x264 Filter bank 4 register 2
    volatile uint32_t F5R1        ; // 0x268 Filter bank 5 register 1
    volatile uint32_t F5R2        ; // 0x26C Filter bank 5 register 2
    volatile uint32_t F6R1        ; // 0x270 Filter bank 6 register 1
    volatile uint32_t F6R2        ; // 0x274 Filter bank 6 register 2
    volatile uint32_t F7R1        ; // 0x278 Filter bank 7 register 1
    volatile uint32_t F7R2        ; // 0x27C Filter bank 7 register 2
    volatile uint32_t F8R1        ; // 0x280 Filter bank 8 register 1
    volatile uint32_t F8R2        ; // 0x284 Filter bank 8 register 2
    volatile uint32_t F9R1        ; // 0x288 Filter bank 9 register 1
    volatile uint32_t F9R2        ; // 0x28C Filter bank 9 register 2
    volatile uint32_t F10R1       ; // 0x290 Filter bank 10 register 1
    volatile uint32_t F10R2       ; // 0x294 Filter bank 10 register 2
    volatile uint32_t F11R1       ; // 0x298 Filter bank 11 register 1
    volatile uint32_t F11R2       ; // 0x29C Filter bank 11 register 2
    volatile uint32_t F12R1       ; // 0x2A0 Filter bank 4 register 1
    volatile uint32_t F12R2       ; // 0x2A4 Filter bank 12 register 2
    volatile uint32_t F13R1       ; // 0x2A8 Filter bank 13 register 1
    volatile uint32_t F13R2       ; // 0x2AC Filter bank 13 register 2
    volatile uint32_t F14R1       ; // 0x2B0 Filter bank 14 register 1
    volatile uint32_t F14R2       ; // 0x2B4 Filter bank 14 register 2
    volatile uint32_t F15R1       ; // 0x2B8 Filter bank 15 register 1
    volatile uint32_t F15R2       ; // 0x2BC Filter bank 15 register 2
    volatile uint32_t F16R1       ; // 0x2C0 Filter bank 16 register 1
    volatile uint32_t F16R2       ; // 0x2C4 Filter bank 16 register 2
    volatile uint32_t F17R1       ; // 0x2C8 Filter bank 17 register 1
    volatile uint32_t F17R2       ; // 0x2CC Filter bank 17 register 2
    volatile uint32_t F18R1       ; // 0x2D0 Filter bank 18 register 1
    volatile uint32_t F18R2       ; // 0x2D4 Filter bank 18 register 2
    volatile uint32_t F19R1       ; // 0x2D8 Filter bank 19 register 1
    volatile uint32_t F19R2       ; // 0x2DC Filter bank 19 register 2
    volatile uint32_t F20R1       ; // 0x2E0 Filter bank 20 register 1
    volatile uint32_t F20R2       ; // 0x2E4 Filter bank 20 register 2
    volatile uint32_t F21R1       ; // 0x2E8 Filter bank 21 register 1
    volatile uint32_t F21R2       ; // 0x2EC Filter bank 21 register 2
    volatile uint32_t F22R1       ; // 0x2F0 Filter bank 22 register 1
    volatile uint32_t F22R2       ; // 0x2F4 Filter bank 22 register 2
    volatile uint32_t F23R1       ; // 0x2F8 Filter bank 23 register 1
    volatile uint32_t F23R2       ; // 0x2FC Filter bank 23 register 2
    volatile uint32_t F24R1       ; // 0x300 Filter bank 24 register 1
    volatile uint32_t F24R2       ; // 0x304 Filter bank 24 register 2
    volatile uint32_t F25R1       ; // 0x308 Filter bank 25 register 1
    volatile uint32_t F25R2       ; // 0x30C Filter bank 25 register 2
    volatile uint32_t F26R1       ; // 0x310 Filter bank 26 register 1
    volatile uint32_t F26R2       ; // 0x314 Filter bank 26 register 2
    volatile uint32_t F27R1       ; // 0x318 Filter bank 27 register 1
    volatile uint32_t F27R2       ; // 0x31C Filter bank 27 register 2
} CAN1_TypeDef;

#define CAN1   ((CAN1_TypeDef *) 0x40006400UL)

typedef struct {
    volatile uint32_t ACR         ; // 0x00 Flash access control register
    volatile uint32_t KEYR        ; // 0x04 Flash key register
    volatile uint32_t OPTKEYR     ; // 0x08 Flash option key register
    volatile uint32_t SR          ; // 0x0C Status register
    volatile uint32_t CR          ; // 0x10 Control register
    volatile uint32_t OPTCR       ; // 0x14 Flash option control register
} FLASH_TypeDef;

#define FLASH   ((FLASH_TypeDef *) 0x40023C00UL)

typedef struct {
    volatile uint32_t IMR         ; // 0x00 Interrupt mask register (EXTI_IMR)
    volatile uint32_t EMR         ; // 0x04 Event mask register (EXTI_EMR)
    volatile uint32_t RTSR        ; // 0x08 Rising Trigger selection register (EXTI_RTSR)
    volatile uint32_t FTSR        ; // 0x0C Falling Trigger selection register (EXTI_FTSR)
    volatile uint32_t SWIER       ; // 0x10 Software interrupt event register (EXTI_SWIER)
    volatile uint32_t PR          ; // 0x14 Pending register (EXTI_PR)
} EXTI_TypeDef;

#define EXTI   ((EXTI_TypeDef *) 0x40013C00UL)

typedef struct {
    volatile uint32_t OTG_HS_GOTGCTL; // 0x00 OTG_HS control and status register
    volatile uint32_t OTG_HS_GOTGINT; // 0x04 OTG_HS interrupt register
    volatile uint32_t OTG_HS_GAHBCFG; // 0x08 OTG_HS AHB configuration register
    volatile uint32_t OTG_HS_GUSBCFG; // 0x0C OTG_HS USB configuration register
    volatile uint32_t OTG_HS_GRSTCTL; // 0x10 OTG_HS reset register
    volatile uint32_t OTG_HS_GINTSTS; // 0x14 OTG_HS core interrupt register
    volatile uint32_t OTG_HS_GINTMSK; // 0x18 OTG_HS interrupt mask register
    volatile uint32_t OTG_HS_GRXSTSR_Host; // 0x1C OTG_HS Receive status debug read register (host mode)
    volatile uint32_t OTG_HS_GRXSTSR_Peripheral; // 0x1C OTG_HS Receive status debug read register (peripheral mode mode)
    volatile uint32_t OTG_HS_GRXSTSP_Host; // 0x20 OTG_HS status read and pop register (host mode)
    volatile uint32_t OTG_HS_GRXSTSP_Peripheral; // 0x20 OTG_HS status read and pop register (peripheral mode)
    volatile uint32_t OTG_HS_GRXFSIZ; // 0x24 OTG_HS Receive FIFO size register
    volatile uint32_t OTG_HS_GNPTXFSIZ_Host; // 0x28 OTG_HS nonperiodic transmit FIFO size register (host mode)
    volatile uint32_t OTG_HS_TX0FSIZ_Peripheral; // 0x28 Endpoint 0 transmit FIFO size (peripheral mode)
    volatile uint32_t OTG_HS_GNPTXSTS; // 0x2C OTG_HS nonperiodic transmit FIFO/queue status register
    volatile uint32_t RESERVED_0[2]; // 0x30 - 0x37
    volatile uint32_t OTG_HS_GCCFG; // 0x38 OTG_HS general core configuration register
    volatile uint32_t OTG_HS_CID  ; // 0x3C OTG_HS core ID register
    volatile uint32_t RESERVED_1[48]; // 0x40 - 0xFF
    volatile uint32_t OTG_HS_HPTXFSIZ; // 0x100 OTG_HS Host periodic transmit FIFO size register
    volatile uint32_t OTG_HS_DIEPTXF1; // 0x104 OTG_HS device IN endpoint transmit FIFO size register
    volatile uint32_t OTG_HS_DIEPTXF2; // 0x108 OTG_HS device IN endpoint transmit FIFO size register
    volatile uint32_t RESERVED_2[4]; // 0x10C - 0x11B
    volatile uint32_t OTG_HS_DIEPTXF3; // 0x11C OTG_HS device IN endpoint transmit FIFO size register
    volatile uint32_t OTG_HS_DIEPTXF4; // 0x120 OTG_HS device IN endpoint transmit FIFO size register
    volatile uint32_t OTG_HS_DIEPTXF5; // 0x124 OTG_HS device IN endpoint transmit FIFO size register
    volatile uint32_t OTG_HS_DIEPTXF6; // 0x128 OTG_HS device IN endpoint transmit FIFO size register
    volatile uint32_t OTG_HS_DIEPTXF7; // 0x12C OTG_HS device IN endpoint transmit FIFO size register
} OTG_HS_GLOBAL_TypeDef;

#define OTG_HS_GLOBAL   ((OTG_HS_GLOBAL_TypeDef *) 0x40040000UL)

typedef struct {
    volatile uint32_t OTG_HS_HCFG ; // 0x00 OTG_HS host configuration register
    volatile uint32_t OTG_HS_HFIR ; // 0x04 OTG_HS Host frame interval register
    volatile uint32_t OTG_HS_HFNUM; // 0x08 OTG_HS host frame number/frame time remaining register
    volatile uint32_t RESERVED_0[1]; // 0x0C - 0x0F
    volatile uint32_t OTG_HS_HPTXSTS; // 0x10 OTG_HS_Host periodic transmit FIFO/queue status register
    volatile uint32_t OTG_HS_HAINT; // 0x14 OTG_HS Host all channels interrupt register
    volatile uint32_t OTG_HS_HAINTMSK; // 0x18 OTG_HS host all channels interrupt mask register
    volatile uint32_t RESERVED_1[9]; // 0x1C - 0x3F
    volatile uint32_t OTG_HS_HPRT ; // 0x40 OTG_HS host port control and status register
    volatile uint32_t RESERVED_2[47]; // 0x44 - 0xFF
    volatile uint32_t OTG_HS_HCCHAR0; // 0x100 OTG_HS host channel-0 characteristics register
    volatile uint32_t OTG_HS_HCSPLT0; // 0x104 OTG_HS host channel-0 split control register
    volatile uint32_t OTG_HS_HCINT0; // 0x108 OTG_HS host channel-11 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK0; // 0x10C OTG_HS host channel-11 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ0; // 0x110 OTG_HS host channel-11 transfer size register
    volatile uint32_t OTG_HS_HCDMA0; // 0x114 OTG_HS host channel-0 DMA address register
    volatile uint32_t RESERVED_3[2]; // 0x118 - 0x11F
    volatile uint32_t OTG_HS_HCCHAR1; // 0x120 OTG_HS host channel-1 characteristics register
    volatile uint32_t OTG_HS_HCSPLT1; // 0x124 OTG_HS host channel-1 split control register
    volatile uint32_t OTG_HS_HCINT1; // 0x128 OTG_HS host channel-1 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK1; // 0x12C OTG_HS host channel-1 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ1; // 0x130 OTG_HS host channel-1 transfer size register
    volatile uint32_t OTG_HS_HCDMA1; // 0x134 OTG_HS host channel-1 DMA address register
    volatile uint32_t RESERVED_4[2]; // 0x138 - 0x13F
    volatile uint32_t OTG_HS_HCCHAR2; // 0x140 OTG_HS host channel-2 characteristics register
    volatile uint32_t OTG_HS_HCSPLT2; // 0x144 OTG_HS host channel-2 split control register
    volatile uint32_t OTG_HS_HCINT2; // 0x148 OTG_HS host channel-2 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK2; // 0x14C OTG_HS host channel-2 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ2; // 0x150 OTG_HS host channel-2 transfer size register
    volatile uint32_t OTG_HS_HCDMA2; // 0x154 OTG_HS host channel-2 DMA address register
    volatile uint32_t RESERVED_5[2]; // 0x158 - 0x15F
    volatile uint32_t OTG_HS_HCCHAR3; // 0x160 OTG_HS host channel-3 characteristics register
    volatile uint32_t OTG_HS_HCSPLT3; // 0x164 OTG_HS host channel-3 split control register
    volatile uint32_t OTG_HS_HCINT3; // 0x168 OTG_HS host channel-3 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK3; // 0x16C OTG_HS host channel-3 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ3; // 0x170 OTG_HS host channel-3 transfer size register
    volatile uint32_t OTG_HS_HCDMA3; // 0x174 OTG_HS host channel-3 DMA address register
    volatile uint32_t RESERVED_6[2]; // 0x178 - 0x17F
    volatile uint32_t OTG_HS_HCCHAR4; // 0x180 OTG_HS host channel-4 characteristics register
    volatile uint32_t OTG_HS_HCSPLT4; // 0x184 OTG_HS host channel-4 split control register
    volatile uint32_t OTG_HS_HCINT4; // 0x188 OTG_HS host channel-4 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK4; // 0x18C OTG_HS host channel-4 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ4; // 0x190 OTG_HS host channel-4 transfer size register
    volatile uint32_t OTG_HS_HCDMA4; // 0x194 OTG_HS host channel-4 DMA address register
    volatile uint32_t RESERVED_7[2]; // 0x198 - 0x19F
    volatile uint32_t OTG_HS_HCCHAR5; // 0x1A0 OTG_HS host channel-5 characteristics register
    volatile uint32_t OTG_HS_HCSPLT5; // 0x1A4 OTG_HS host channel-5 split control register
    volatile uint32_t OTG_HS_HCINT5; // 0x1A8 OTG_HS host channel-5 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK5; // 0x1AC OTG_HS host channel-5 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ5; // 0x1B0 OTG_HS host channel-5 transfer size register
    volatile uint32_t OTG_HS_HCDMA5; // 0x1B4 OTG_HS host channel-5 DMA address register
    volatile uint32_t RESERVED_8[2]; // 0x1B8 - 0x1BF
    volatile uint32_t OTG_HS_HCCHAR6; // 0x1C0 OTG_HS host channel-6 characteristics register
    volatile uint32_t OTG_HS_HCSPLT6; // 0x1C4 OTG_HS host channel-6 split control register
    volatile uint32_t OTG_HS_HCINT6; // 0x1C8 OTG_HS host channel-6 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK6; // 0x1CC OTG_HS host channel-6 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ6; // 0x1D0 OTG_HS host channel-6 transfer size register
    volatile uint32_t OTG_HS_HCDMA6; // 0x1D4 OTG_HS host channel-6 DMA address register
    volatile uint32_t RESERVED_9[2]; // 0x1D8 - 0x1DF
    volatile uint32_t OTG_HS_HCCHAR7; // 0x1E0 OTG_HS host channel-7 characteristics register
    volatile uint32_t OTG_HS_HCSPLT7; // 0x1E4 OTG_HS host channel-7 split control register
    volatile uint32_t OTG_HS_HCINT7; // 0x1E8 OTG_HS host channel-7 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK7; // 0x1EC OTG_HS host channel-7 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ7; // 0x1F0 OTG_HS host channel-7 transfer size register
    volatile uint32_t OTG_HS_HCDMA7; // 0x1F4 OTG_HS host channel-7 DMA address register
    volatile uint32_t RESERVED_10[2]; // 0x1F8 - 0x1FF
    volatile uint32_t OTG_HS_HCCHAR8; // 0x200 OTG_HS host channel-8 characteristics register
    volatile uint32_t OTG_HS_HCSPLT8; // 0x204 OTG_HS host channel-8 split control register
    volatile uint32_t OTG_HS_HCINT8; // 0x208 OTG_HS host channel-8 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK8; // 0x20C OTG_HS host channel-8 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ8; // 0x210 OTG_HS host channel-8 transfer size register
    volatile uint32_t OTG_HS_HCDMA8; // 0x214 OTG_HS host channel-8 DMA address register
    volatile uint32_t RESERVED_11[2]; // 0x218 - 0x21F
    volatile uint32_t OTG_HS_HCCHAR9; // 0x220 OTG_HS host channel-9 characteristics register
    volatile uint32_t OTG_HS_HCSPLT9; // 0x224 OTG_HS host channel-9 split control register
    volatile uint32_t OTG_HS_HCINT9; // 0x228 OTG_HS host channel-9 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK9; // 0x22C OTG_HS host channel-9 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ9; // 0x230 OTG_HS host channel-9 transfer size register
    volatile uint32_t OTG_HS_HCDMA9; // 0x234 OTG_HS host channel-9 DMA address register
    volatile uint32_t RESERVED_12[2]; // 0x238 - 0x23F
    volatile uint32_t OTG_HS_HCCHAR10; // 0x240 OTG_HS host channel-10 characteristics register
    volatile uint32_t OTG_HS_HCSPLT10; // 0x244 OTG_HS host channel-10 split control register
    volatile uint32_t OTG_HS_HCINT10; // 0x248 OTG_HS host channel-10 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK10; // 0x24C OTG_HS host channel-10 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ10; // 0x250 OTG_HS host channel-10 transfer size register
    volatile uint32_t OTG_HS_HCDMA10; // 0x254 OTG_HS host channel-10 DMA address register
    volatile uint32_t RESERVED_13[2]; // 0x258 - 0x25F
    volatile uint32_t OTG_HS_HCCHAR11; // 0x260 OTG_HS host channel-11 characteristics register
    volatile uint32_t OTG_HS_HCSPLT11; // 0x264 OTG_HS host channel-11 split control register
    volatile uint32_t OTG_HS_HCINT11; // 0x268 OTG_HS host channel-11 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK11; // 0x26C OTG_HS host channel-11 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ11; // 0x270 OTG_HS host channel-11 transfer size register
    volatile uint32_t OTG_HS_HCDMA11; // 0x274 OTG_HS host channel-11 DMA address register
    volatile uint32_t RESERVED_14[2]; // 0x278 - 0x27F
    volatile uint32_t OTG_HS_HCCHAR12; // 0x280 OTG_HS host channel-12 characteristics register
    volatile uint32_t OTG_HS_HCSPLT12; // 0x284 OTG_HS host channel-12 split control register
    volatile uint32_t OTG_HS_HCINT12; // 0x288 OTG_HS host channel-12 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK12; // 0x28C OTG_HS host channel-12 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ12; // 0x290 OTG_HS host channel-12 transfer size register
    volatile uint32_t OTG_HS_HCDMA12; // 0x294 OTG_HS host channel-12 DMA address register
    volatile uint32_t RESERVED_15[2]; // 0x298 - 0x29F
    volatile uint32_t OTG_HS_HCCHAR13; // 0x2A0 OTG_HS host channel-13 characteristics register
    volatile uint32_t OTG_HS_HCSPLT13; // 0x2A4 OTG_HS host channel-13 split control register
    volatile uint32_t OTG_HS_HCINT13; // 0x2A8 OTG_HS host channel-13 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK13; // 0x2AC OTG_HS host channel-13 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ13; // 0x2B0 OTG_HS host channel-13 transfer size register
    volatile uint32_t OTG_HS_HCDMA13; // 0x2B4 OTG_HS host channel-13 DMA address register
    volatile uint32_t RESERVED_16[2]; // 0x2B8 - 0x2BF
    volatile uint32_t OTG_HS_HCCHAR14; // 0x2C0 OTG_HS host channel-14 characteristics register
    volatile uint32_t OTG_HS_HCSPLT14; // 0x2C4 OTG_HS host channel-14 split control register
    volatile uint32_t OTG_HS_HCINT14; // 0x2C8 OTG_HS host channel-14 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK14; // 0x2CC OTG_HS host channel-14 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ14; // 0x2D0 OTG_HS host channel-14 transfer size register
    volatile uint32_t OTG_HS_HCDMA14; // 0x2D4 OTG_HS host channel-14 DMA address register
    volatile uint32_t RESERVED_17[2]; // 0x2D8 - 0x2DF
    volatile uint32_t OTG_HS_HCCHAR15; // 0x2E0 OTG_HS host channel-15 characteristics register
    volatile uint32_t OTG_HS_HCSPLT15; // 0x2E4 OTG_HS host channel-15 split control register
    volatile uint32_t OTG_HS_HCINT15; // 0x2E8 OTG_HS host channel-15 interrupt register
    volatile uint32_t OTG_HS_HCINTMSK15; // 0x2EC OTG_HS host channel-15 interrupt mask register
    volatile uint32_t OTG_HS_HCTSIZ15; // 0x2F0 OTG_HS host channel-15 transfer size register
    volatile uint32_t OTG_HS_HCDMA15; // 0x2F4 OTG_HS host channel-15 DMA address register
} OTG_HS_HOST_TypeDef;

#define OTG_HS_HOST   ((OTG_HS_HOST_TypeDef *) 0x40040400UL)

typedef struct {
    volatile uint32_t OTG_HS_DCFG ; // 0x00 OTG_HS device configuration register
    volatile uint32_t OTG_HS_DCTL ; // 0x04 OTG_HS device control register
    volatile uint32_t OTG_HS_DSTS ; // 0x08 OTG_HS device status register
    volatile uint32_t RESERVED_0[1]; // 0x0C - 0x0F
    volatile uint32_t OTG_HS_DIEPMSK; // 0x10 OTG_HS device IN endpoint common interrupt mask register
    volatile uint32_t OTG_HS_DOEPMSK; // 0x14 OTG_HS device OUT endpoint common interrupt mask register
    volatile uint32_t OTG_HS_DAINT; // 0x18 OTG_HS device all endpoints interrupt register
    volatile uint32_t OTG_HS_DAINTMSK; // 0x1C OTG_HS all endpoints interrupt mask register
    volatile uint32_t RESERVED_1[2]; // 0x20 - 0x27
    volatile uint32_t OTG_HS_DVBUSDIS; // 0x28 OTG_HS device VBUS discharge time register
    volatile uint32_t OTG_HS_DVBUSPULSE; // 0x2C OTG_HS device VBUS pulsing time register
    volatile uint32_t OTG_HS_DTHRCTL; // 0x30 OTG_HS Device threshold control register
    volatile uint32_t OTG_HS_DIEPEMPMSK; // 0x34 OTG_HS device IN endpoint FIFO empty interrupt mask register
    volatile uint32_t OTG_HS_DEACHINT; // 0x38 OTG_HS device each endpoint interrupt register
    volatile uint32_t OTG_HS_DEACHINTMSK; // 0x3C OTG_HS device each endpoint interrupt register mask
    volatile uint32_t RESERVED_2[1]; // 0x40 - 0x43
    volatile uint32_t OTG_HS_DIEPEACHMSK1; // 0x44 OTG_HS device each in endpoint-1 interrupt register
    volatile uint32_t RESERVED_3[15]; // 0x48 - 0x83
    volatile uint32_t OTG_HS_DOEPEACHMSK1; // 0x84 OTG_HS device each OUT endpoint-1 interrupt register
    volatile uint32_t RESERVED_4[30]; // 0x88 - 0xFF
    volatile uint32_t OTG_HS_DIEPCTL0; // 0x100 OTG device endpoint-0 control register
    volatile uint32_t RESERVED_5[1]; // 0x104 - 0x107
    volatile uint32_t OTG_HS_DIEPINT0; // 0x108 OTG device endpoint-0 interrupt register
    volatile uint32_t RESERVED_6[1]; // 0x10C - 0x10F
    volatile uint32_t OTG_HS_DIEPTSIZ0; // 0x110 OTG_HS device IN endpoint 0 transfer size register
    volatile uint32_t OTG_HS_DIEPDMA1; // 0x114 OTG_HS device endpoint-1 DMA address register
    volatile uint32_t OTG_HS_DTXFSTS0; // 0x118 OTG_HS device IN endpoint transmit FIFO status register
    volatile uint32_t RESERVED_7[1]; // 0x11C - 0x11F
    volatile uint32_t OTG_HS_DIEPCTL1; // 0x120 OTG device endpoint-1 control register
    volatile uint32_t RESERVED_8[1]; // 0x124 - 0x127
    volatile uint32_t OTG_HS_DIEPINT1; // 0x128 OTG device endpoint-1 interrupt register
    volatile uint32_t RESERVED_9[1]; // 0x12C - 0x12F
    volatile uint32_t OTG_HS_DIEPTSIZ1; // 0x130 OTG_HS device endpoint transfer size register
    volatile uint32_t OTG_HS_DIEPDMA2; // 0x134 OTG_HS device endpoint-2 DMA address register
    volatile uint32_t OTG_HS_DTXFSTS1; // 0x138 OTG_HS device IN endpoint transmit FIFO status register
    volatile uint32_t RESERVED_10[1]; // 0x13C - 0x13F
    volatile uint32_t OTG_HS_DIEPCTL2; // 0x140 OTG device endpoint-2 control register
    volatile uint32_t RESERVED_11[1]; // 0x144 - 0x147
    volatile uint32_t OTG_HS_DIEPINT2; // 0x148 OTG device endpoint-2 interrupt register
    volatile uint32_t RESERVED_12[1]; // 0x14C - 0x14F
    volatile uint32_t OTG_HS_DIEPTSIZ2; // 0x150 OTG_HS device endpoint transfer size register
    volatile uint32_t OTG_HS_DIEPDMA3; // 0x154 OTG_HS device endpoint-3 DMA address register
    volatile uint32_t OTG_HS_DTXFSTS2; // 0x158 OTG_HS device IN endpoint transmit FIFO status register
    volatile uint32_t RESERVED_13[1]; // 0x15C - 0x15F
    volatile uint32_t OTG_HS_DIEPCTL3; // 0x160 OTG device endpoint-3 control register
    volatile uint32_t RESERVED_14[1]; // 0x164 - 0x167
    volatile uint32_t OTG_HS_DIEPINT3; // 0x168 OTG device endpoint-3 interrupt register
    volatile uint32_t RESERVED_15[1]; // 0x16C - 0x16F
    volatile uint32_t OTG_HS_DIEPTSIZ3; // 0x170 OTG_HS device endpoint transfer size register
    volatile uint32_t OTG_HS_DIEPDMA4; // 0x174 OTG_HS device endpoint-4 DMA address register
    volatile uint32_t OTG_HS_DTXFSTS3; // 0x178 OTG_HS device IN endpoint transmit FIFO status register
    volatile uint32_t RESERVED_16[1]; // 0x17C - 0x17F
    volatile uint32_t OTG_HS_DIEPCTL4; // 0x180 OTG device endpoint-4 control register
    volatile uint32_t RESERVED_17[1]; // 0x184 - 0x187
    volatile uint32_t OTG_HS_DIEPINT4; // 0x188 OTG device endpoint-4 interrupt register
    volatile uint32_t RESERVED_18[1]; // 0x18C - 0x18F
    volatile uint32_t OTG_HS_DIEPTSIZ4; // 0x190 OTG_HS device endpoint transfer size register
    volatile uint32_t OTG_HS_DIEPDMA5; // 0x194 OTG_HS device endpoint-5 DMA address register
    volatile uint32_t OTG_HS_DTXFSTS4; // 0x198 OTG_HS device IN endpoint transmit FIFO status register
    volatile uint32_t RESERVED_19[1]; // 0x19C - 0x19F
    volatile uint32_t OTG_HS_DIEPCTL5; // 0x1A0 OTG device endpoint-5 control register
    volatile uint32_t RESERVED_20[1]; // 0x1A4 - 0x1A7
    volatile uint32_t OTG_HS_DIEPINT5; // 0x1A8 OTG device endpoint-5 interrupt register
    volatile uint32_t RESERVED_21[1]; // 0x1AC - 0x1AF
    volatile uint32_t OTG_HS_DIEPTSIZ5; // 0x1B0 OTG_HS device endpoint transfer size register
    volatile uint32_t RESERVED_22[1]; // 0x1B4 - 0x1B7
    volatile uint32_t OTG_HS_DTXFSTS5; // 0x1B8 OTG_HS device IN endpoint transmit FIFO status register
    volatile uint32_t RESERVED_23[1]; // 0x1BC - 0x1BF
    volatile uint32_t OTG_HS_DIEPCTL6; // 0x1C0 OTG device endpoint-6 control register
    volatile uint32_t RESERVED_24[1]; // 0x1C4 - 0x1C7
    volatile uint32_t OTG_HS_DIEPINT6; // 0x1C8 OTG device endpoint-6 interrupt register
    volatile uint32_t RESERVED_25[5]; // 0x1CC - 0x1DF
    volatile uint32_t OTG_HS_DIEPCTL7; // 0x1E0 OTG device endpoint-7 control register
    volatile uint32_t RESERVED_26[1]; // 0x1E4 - 0x1E7
    volatile uint32_t OTG_HS_DIEPINT7; // 0x1E8 OTG device endpoint-7 interrupt register
    volatile uint32_t RESERVED_27[69]; // 0x1EC - 0x2FF
    volatile uint32_t OTG_HS_DOEPCTL0; // 0x300 OTG_HS device control OUT endpoint 0 control register
    volatile uint32_t RESERVED_28[1]; // 0x304 - 0x307
    volatile uint32_t OTG_HS_DOEPINT0; // 0x308 OTG_HS device endpoint-0 interrupt register
    volatile uint32_t RESERVED_29[1]; // 0x30C - 0x30F
    volatile uint32_t OTG_HS_DOEPTSIZ0; // 0x310 OTG_HS device endpoint-1 transfer size register
    volatile uint32_t RESERVED_30[3]; // 0x314 - 0x31F
    volatile uint32_t OTG_HS_DOEPCTL1; // 0x320 OTG device endpoint-1 control register
    volatile uint32_t RESERVED_31[1]; // 0x324 - 0x327
    volatile uint32_t OTG_HS_DOEPINT1; // 0x328 OTG_HS device endpoint-1 interrupt register
    volatile uint32_t RESERVED_32[1]; // 0x32C - 0x32F
    volatile uint32_t OTG_HS_DOEPTSIZ1; // 0x330 OTG_HS device endpoint-2 transfer size register
    volatile uint32_t RESERVED_33[3]; // 0x334 - 0x33F
    volatile uint32_t OTG_HS_DOEPCTL2; // 0x340 OTG device endpoint-2 control register
    volatile uint32_t RESERVED_34[1]; // 0x344 - 0x347
    volatile uint32_t OTG_HS_DOEPINT2; // 0x348 OTG_HS device endpoint-2 interrupt register
    volatile uint32_t RESERVED_35[1]; // 0x34C - 0x34F
    volatile uint32_t OTG_HS_DOEPTSIZ2; // 0x350 OTG_HS device endpoint-3 transfer size register
    volatile uint32_t RESERVED_36[3]; // 0x354 - 0x35F
    volatile uint32_t OTG_HS_DOEPCTL3; // 0x360 OTG device endpoint-3 control register
    volatile uint32_t RESERVED_37[1]; // 0x364 - 0x367
    volatile uint32_t OTG_HS_DOEPINT3; // 0x368 OTG_HS device endpoint-3 interrupt register
    volatile uint32_t RESERVED_38[1]; // 0x36C - 0x36F
    volatile uint32_t OTG_HS_DOEPTSIZ3; // 0x370 OTG_HS device endpoint-4 transfer size register
    volatile uint32_t RESERVED_39[5]; // 0x374 - 0x387
    volatile uint32_t OTG_HS_DOEPINT4; // 0x388 OTG_HS device endpoint-4 interrupt register
    volatile uint32_t RESERVED_40[1]; // 0x38C - 0x38F
    volatile uint32_t OTG_HS_DOEPTSIZ4; // 0x390 OTG_HS device endpoint-5 transfer size register
    volatile uint32_t RESERVED_41[5]; // 0x394 - 0x3A7
    volatile uint32_t OTG_HS_DOEPINT5; // 0x3A8 OTG_HS device endpoint-5 interrupt register
    volatile uint32_t RESERVED_42[7]; // 0x3AC - 0x3C7
    volatile uint32_t OTG_HS_DOEPINT6; // 0x3C8 OTG_HS device endpoint-6 interrupt register
    volatile uint32_t RESERVED_43[7]; // 0x3CC - 0x3E7
    volatile uint32_t OTG_HS_DOEPINT7; // 0x3E8 OTG_HS device endpoint-7 interrupt register
} OTG_HS_DEVICE_TypeDef;

#define OTG_HS_DEVICE   ((OTG_HS_DEVICE_TypeDef *) 0x40040800UL)

typedef struct {
    volatile uint32_t OTG_HS_PCGCR; // 0x00 Power and clock gating control register
} OTG_HS_PWRCLK_TypeDef;

#define OTG_HS_PWRCLK   ((OTG_HS_PWRCLK_TypeDef *) 0x40040E00UL)

typedef struct {
    volatile uint32_t CR          ; // 0x00 control register
    volatile uint32_t DIN         ; // 0x04 data input register
    volatile uint32_t STR         ; // 0x08 start register
    volatile uint32_t HR0         ; // 0x0C digest registers
    volatile uint32_t HR1         ; // 0x10 digest registers
    volatile uint32_t HR2         ; // 0x14 digest registers
    volatile uint32_t HR3         ; // 0x18 digest registers
    volatile uint32_t HR4         ; // 0x1C digest registers
    volatile uint32_t IMR         ; // 0x20 interrupt enable register
    volatile uint32_t SR          ; // 0x24 status register
    volatile uint32_t RESERVED_0[52]; // 0x28 - 0xF7
    volatile uint32_t CSR0        ; // 0xF8 context swap registers
    volatile uint32_t CSR1        ; // 0xFC context swap registers
    volatile uint32_t CSR2        ; // 0x100 context swap registers
    volatile uint32_t CSR3        ; // 0x104 context swap registers
    volatile uint32_t CSR4        ; // 0x108 context swap registers
    volatile uint32_t CSR5        ; // 0x10C context swap registers
    volatile uint32_t CSR6        ; // 0x110 context swap registers
    volatile uint32_t CSR7        ; // 0x114 context swap registers
    volatile uint32_t CSR8        ; // 0x118 context swap registers
    volatile uint32_t CSR9        ; // 0x11C context swap registers
    volatile uint32_t CSR10       ; // 0x120 context swap registers
    volatile uint32_t CSR11       ; // 0x124 context swap registers
    volatile uint32_t CSR12       ; // 0x128 context swap registers
    volatile uint32_t CSR13       ; // 0x12C context swap registers
    volatile uint32_t CSR14       ; // 0x130 context swap registers
    volatile uint32_t CSR15       ; // 0x134 context swap registers
    volatile uint32_t CSR16       ; // 0x138 context swap registers
    volatile uint32_t CSR17       ; // 0x13C context swap registers
    volatile uint32_t CSR18       ; // 0x140 context swap registers
    volatile uint32_t CSR19       ; // 0x144 context swap registers
    volatile uint32_t CSR20       ; // 0x148 context swap registers
    volatile uint32_t CSR21       ; // 0x14C context swap registers
    volatile uint32_t CSR22       ; // 0x150 context swap registers
    volatile uint32_t CSR23       ; // 0x154 context swap registers
    volatile uint32_t CSR24       ; // 0x158 context swap registers
    volatile uint32_t CSR25       ; // 0x15C context swap registers
    volatile uint32_t CSR26       ; // 0x160 context swap registers
    volatile uint32_t CSR27       ; // 0x164 context swap registers
    volatile uint32_t CSR28       ; // 0x168 context swap registers
    volatile uint32_t CSR29       ; // 0x16C context swap registers
    volatile uint32_t CSR30       ; // 0x170 context swap registers
    volatile uint32_t CSR31       ; // 0x174 context swap registers
    volatile uint32_t CSR32       ; // 0x178 context swap registers
    volatile uint32_t CSR33       ; // 0x17C context swap registers
    volatile uint32_t CSR34       ; // 0x180 context swap registers
    volatile uint32_t CSR35       ; // 0x184 context swap registers
    volatile uint32_t CSR36       ; // 0x188 context swap registers
    volatile uint32_t CSR37       ; // 0x18C context swap registers
    volatile uint32_t CSR38       ; // 0x190 context swap registers
    volatile uint32_t CSR39       ; // 0x194 context swap registers
    volatile uint32_t CSR40       ; // 0x198 context swap registers
    volatile uint32_t CSR41       ; // 0x19C context swap registers
    volatile uint32_t CSR42       ; // 0x1A0 context swap registers
    volatile uint32_t CSR43       ; // 0x1A4 context swap registers
    volatile uint32_t CSR44       ; // 0x1A8 context swap registers
    volatile uint32_t CSR45       ; // 0x1AC context swap registers
    volatile uint32_t CSR46       ; // 0x1B0 context swap registers
    volatile uint32_t CSR47       ; // 0x1B4 context swap registers
    volatile uint32_t CSR48       ; // 0x1B8 context swap registers
    volatile uint32_t CSR49       ; // 0x1BC context swap registers
    volatile uint32_t CSR50       ; // 0x1C0 context swap registers
    volatile uint32_t CSR51       ; // 0x1C4 context swap registers
    volatile uint32_t CSR52       ; // 0x1C8 context swap registers
    volatile uint32_t CSR53       ; // 0x1CC context swap registers
    volatile uint32_t RESERVED_1[80]; // 0x1D0 - 0x30F
    volatile uint32_t HASH_HR0    ; // 0x310 HASH digest register
    volatile uint32_t HASH_HR1    ; // 0x314 read-only
    volatile uint32_t HASH_HR2    ; // 0x318 read-only
    volatile uint32_t HASH_HR3    ; // 0x31C read-only
    volatile uint32_t HASH_HR4    ; // 0x320 read-only
    volatile uint32_t HASH_HR5    ; // 0x324 read-only
    volatile uint32_t HASH_HR6    ; // 0x328 read-only
    volatile uint32_t HASH_HR7    ; // 0x32C read-only
} HASH_TypeDef;

#define HASH   ((HASH_TypeDef *) 0x50060400UL)

typedef struct {
    volatile uint32_t CR          ; // 0x00 control register
    volatile uint32_t SR          ; // 0x04 status register
    volatile uint32_t DIN         ; // 0x08 data input register
    volatile uint32_t DOUT        ; // 0x0C data output register
    volatile uint32_t DMACR       ; // 0x10 DMA control register
    volatile uint32_t IMSCR       ; // 0x14 interrupt mask set/clear register
    volatile uint32_t RISR        ; // 0x18 raw interrupt status register
    volatile uint32_t MISR        ; // 0x1C masked interrupt status register
    volatile uint32_t K0LR        ; // 0x20 key registers
    volatile uint32_t K0RR        ; // 0x24 key registers
    volatile uint32_t K1LR        ; // 0x28 key registers
    volatile uint32_t K1RR        ; // 0x2C key registers
    volatile uint32_t K2LR        ; // 0x30 key registers
    volatile uint32_t K2RR        ; // 0x34 key registers
    volatile uint32_t K3LR        ; // 0x38 key registers
    volatile uint32_t K3RR        ; // 0x3C key registers
    volatile uint32_t IV0LR       ; // 0x40 initialization vector registers
    volatile uint32_t IV0RR       ; // 0x44 initialization vector registers
    volatile uint32_t IV1LR       ; // 0x48 initialization vector registers
    volatile uint32_t IV1RR       ; // 0x4C initialization vector registers
    volatile uint32_t CSGCMCCM0R  ; // 0x50 context swap register
    volatile uint32_t CSGCMCCM1R  ; // 0x54 context swap register
    volatile uint32_t CSGCMCCM2R  ; // 0x58 context swap register
    volatile uint32_t CSGCMCCM3R  ; // 0x5C context swap register
    volatile uint32_t CSGCMCCM4R  ; // 0x60 context swap register
    volatile uint32_t CSGCMCCM5R  ; // 0x64 context swap register
    volatile uint32_t CSGCMCCM6R  ; // 0x68 context swap register
    volatile uint32_t CSGCMCCM7R  ; // 0x6C context swap register
    volatile uint32_t CSGCM0R     ; // 0x70 context swap register
    volatile uint32_t CSGCM1R     ; // 0x74 context swap register
    volatile uint32_t CSGCM2R     ; // 0x78 context swap register
    volatile uint32_t CSGCM3R     ; // 0x7C context swap register
    volatile uint32_t CSGCM4R     ; // 0x80 context swap register
    volatile uint32_t CSGCM5R     ; // 0x84 context swap register
    volatile uint32_t CSGCM6R     ; // 0x88 context swap register
    volatile uint32_t CSGCM7R     ; // 0x8C context swap register
} CRYP_TypeDef;

#define CRYP   ((CRYP_TypeDef *) 0x50060000UL)

#endif /* STM32F407_H */
