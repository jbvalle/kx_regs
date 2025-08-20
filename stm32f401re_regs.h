/* Auto-generated from SVD: cmsis-svd-stm32/stm32f4/STM32F401.svd */
#ifndef STM32F401_H
#define STM32F401_H

#include <stdint.h>

typedef struct {
    volatile uint32_t CSR         ; // 0x00 ADC Common status register
    volatile uint32_t CCR         ; // 0x04 ADC common control register
} ADC_COMMON_TypeDef;

#define ADC_COMMON   ((ADC_COMMON_TypeDef *) 0x40012300UL)

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
    volatile uint32_t DR          ; // 0x00 Data register
    volatile uint32_t IDR         ; // 0x04 Independent Data register
    volatile uint32_t CR          ; // 0x08 Control register
} CRC_TypeDef;

#define CRC   ((CRC_TypeDef *) 0x40023000UL)

typedef struct {
    volatile uint32_t DBGMCU_IDCODE; // 0x00 IDCODE
    volatile uint32_t DBGMCU_CR   ; // 0x04 Control Register
    volatile uint32_t DBGMCU_APB1_FZ; // 0x08 Debug MCU APB1 Freeze registe
    volatile uint32_t DBGMCU_APB2_FZ; // 0x0C Debug MCU APB2 Freeze registe
} DBG_TypeDef;

#define DBG   ((DBG_TypeDef *) 0xE0042000UL)

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
    volatile uint32_t ACR         ; // 0x00 Flash access control register
    volatile uint32_t KEYR        ; // 0x04 Flash key register
    volatile uint32_t OPTKEYR     ; // 0x08 Flash option key register
    volatile uint32_t SR          ; // 0x0C Status register
    volatile uint32_t CR          ; // 0x10 Control register
    volatile uint32_t OPTCR       ; // 0x14 Flash option control register
} FLASH_TypeDef;

#define FLASH   ((FLASH_TypeDef *) 0x40023C00UL)

typedef struct {
    volatile uint32_t KR          ; // 0x00 Key register
    volatile uint32_t PR          ; // 0x04 Prescaler register
    volatile uint32_t RLR         ; // 0x08 Reload register
    volatile uint32_t SR          ; // 0x0C Status register
} IWDG_TypeDef;

#define IWDG   ((IWDG_TypeDef *) 0x40003000UL)

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
    volatile uint32_t FS_PCGCCTL  ; // 0x00 OTG_FS power and clock gating control register
} OTG_FS_PWRCLK_TypeDef;

#define OTG_FS_PWRCLK   ((OTG_FS_PWRCLK_TypeDef *) 0x50000E00UL)

typedef struct {
    volatile uint32_t CR          ; // 0x00 power control register
    volatile uint32_t CSR         ; // 0x04 power control/status register
} PWR_TypeDef;

#define PWR   ((PWR_TypeDef *) 0x40007000UL)

typedef struct {
    volatile uint32_t CR          ; // 0x00 clock control register
    volatile uint32_t PLLCFGR     ; // 0x04 PLL configuration register
    volatile uint32_t CFGR        ; // 0x08 clock configuration register
    volatile uint32_t CIR         ; // 0x0C clock interrupt register
    volatile uint32_t AHB1RSTR    ; // 0x10 AHB1 peripheral reset register
    volatile uint32_t AHB2RSTR    ; // 0x14 AHB2 peripheral reset register
    volatile uint32_t RESERVED_0[2]; // 0x18 - 0x1F
    volatile uint32_t APB1RSTR    ; // 0x20 APB1 peripheral reset register
    volatile uint32_t APB2RSTR    ; // 0x24 APB2 peripheral reset register
    volatile uint32_t RESERVED_1[2]; // 0x28 - 0x2F
    volatile uint32_t AHB1ENR     ; // 0x30 AHB1 peripheral clock register
    volatile uint32_t AHB2ENR     ; // 0x34 AHB2 peripheral clock enable register
    volatile uint32_t RESERVED_2[2]; // 0x38 - 0x3F
    volatile uint32_t APB1ENR     ; // 0x40 APB1 peripheral clock enable register
    volatile uint32_t APB2ENR     ; // 0x44 APB2 peripheral clock enable register
    volatile uint32_t RESERVED_3[2]; // 0x48 - 0x4F
    volatile uint32_t AHB1LPENR   ; // 0x50 AHB1 peripheral clock enable in low power mode register
    volatile uint32_t AHB2LPENR   ; // 0x54 AHB2 peripheral clock enable in low power mode register
    volatile uint32_t RESERVED_4[2]; // 0x58 - 0x5F
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
    volatile uint32_t SR          ; // 0x00 Status register
    volatile uint32_t DR          ; // 0x04 Data register
    volatile uint32_t BRR         ; // 0x08 Baud rate register
    volatile uint32_t CR1         ; // 0x0C Control register 1
    volatile uint32_t CR2         ; // 0x10 Control register 2
    volatile uint32_t CR3         ; // 0x14 Control register 3
    volatile uint32_t GTPR        ; // 0x18 Guard time and prescaler register
} USART1_TypeDef;

#define USART1   ((USART1_TypeDef *) 0x40011000UL)

typedef struct {
    volatile uint32_t CR          ; // 0x00 Control register
    volatile uint32_t CFR         ; // 0x04 Configuration register
    volatile uint32_t SR          ; // 0x08 Status register
} WWDG_TypeDef;

#define WWDG   ((WWDG_TypeDef *) 0x40002C00UL)

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
#define GPIOH   ((GPIO_TypeDef *) 0x40021C00UL)

GPIO_TypeDef * const GpioPorts[] = {
    GPIOA,
    GPIOB,
    GPIOH
};

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
    volatile uint32_t CR1         ; // 0x00 control register 1
    volatile uint32_t CR2         ; // 0x04 control register 2
    volatile uint32_t SR          ; // 0x08 status register
    volatile uint32_t DR          ; // 0x0C data register
    volatile uint32_t CRCPR       ; // 0x10 CRC polynomial register
    volatile uint32_t RXCRCR      ; // 0x14 RX CRC register
    volatile uint32_t TXCRCR      ; // 0x18 TX CRC register
    volatile uint32_t I2SCFGR     ; // 0x1C I2S configuration register
    volatile uint32_t I2SPR       ; // 0x20 I2S prescaler register
} I2S2EXT_TypeDef;

#define I2S2EXT   ((I2S2EXT_TypeDef *) 0x40003400UL)

#endif /* STM32F401_H */
