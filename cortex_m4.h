/* Auto-generated manually */
/* Device: CORTEX_M4 */
#ifndef CORTEX_M4_H
#define CORTEX_M4_H

#include <stdint.h>

typedef struct {
    volatile uint32_t STK_CTRL    ; // 0x00 SysTick control and status register
    volatile uint32_t STK_LOAD    ; // 0x04 SysTick reload value register
    volatile uint32_t STK_VAL     ; // 0x08 SysTick current value register
    volatile uint32_t STK_CALIB   ; // 0x0C SysTick calibration value register
} SYSTICK_TypeDef;

#define SYSTICK   ((SYSTICK_TypeDef * const) 0xE000E010UL)

#define STK_CTRL_BIT_POS_ENABLE 0
#define STK_CTRL_BIT_POS_TICKINT 1
#define STK_CTRL_BIT_POS_CLKSRC 2
#define STK_CTRL_BIT_POS_CNTFLAG 16

#define AHB_FRQ_Hz 16000000

#endif /* CORTEX_M4 */
