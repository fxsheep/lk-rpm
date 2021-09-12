#include <lk/debug.h>
#include <platform.h>
#include <arch/arm/cm.h>
#include <platform/qtimer.h>
#include <platform/uart_dm.h>

void platform_early_init(void) {
    qtimer_init();
    uart_dm_init(2, 0, BLSP1_UART1_BASE);
}

void platform_init(void) {
}

