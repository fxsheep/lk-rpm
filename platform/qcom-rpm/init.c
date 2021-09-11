#include <lk/debug.h>
#include <platform.h>
#include <arch/arm/cm.h>
#include <platform/qtimer.h>

void platform_early_init(void) {
    rpm_qtimer_init();
}

void platform_init(void) {
}

