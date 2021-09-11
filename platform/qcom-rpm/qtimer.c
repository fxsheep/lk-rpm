#include <lk/err.h>
#include <kernel/thread.h>
#include <arch/arm.h>
#include <arch/arm/cm.h>
#include <platform/timer.h>
#include <platform/sysaccess.h>
#include <platform/qtimer.h>
#include <platform/platform_cm.h>

void rpm_qtimer_init(void) {

}

status_t platform_set_periodic_timer(platform_timer_callback callback, void *arg, lk_time_t interval) {
    return NO_ERROR;
}

lk_time_t current_time(void) {
    return 0;
}

lk_bigtime_t current_time_hires(void) {
    return 0;
}

void QTimer_IRQHandler(void) {
//    sys_writel(0, 0x4ab000);
}