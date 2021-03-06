#pragma once

#define __NVIC_PRIO_BITS    3

typedef enum {
    Reset_IRQn = -15,
    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn = -11,
    UsageFault_IRQn = -10,
    SVCall_IRQn = -5,
    DebugMonitor_IRQn = -4,
    PendSV_IRQn = -2,
    SysTick_IRQn = -1,
#define RPM_IRQ(name,num) name##_IRQn = num,
#include <platform/irqinfo.h>
#undef RPM_IRQ
} IRQn_Type;