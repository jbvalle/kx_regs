#include "cortex_m4.h"

void global_enable_IRQ(void){

    uint32_t priMASK = 0;
    __asm volatile("MSR primask, %0":: "r" (priMASK):"memory");
}
void global_disable_IRQ(void){

    uint32_t priMASK = 1;
    __asm volatile("MSR primask, %0":: "r" (priMASK):"memory");
}
