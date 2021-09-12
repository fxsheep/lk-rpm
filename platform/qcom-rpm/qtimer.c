#include <lk/err.h>
#include <kernel/thread.h>
#include <arch/arm.h>
#include <arch/arm/cm.h>
#include <platform/timer.h>
#include <platform/sysaccess.h>
#include <platform/qtimer.h>
#include <platform/platform_cm.h>


static uint32_t ticks_per_sec;
static spin_lock_t set_timer_spin_lock = 0;

void qtimer_init()
{
	ticks_per_sec = qtimer_get_frequency();
}

void qtimer_uninit()
{
	qtimer_disable();
}

uint32_t qtimer_tick_rate()
{
	return ticks_per_sec;
}

status_t platform_set_periodic_timer(platform_timer_callback callback, void *arg, lk_time_t interval) {
    spin_lock_saved_state_t state;

    spin_lock_irqsave(&set_timer_spin_lock, state);
    qtimer_set_physical_timer(interval, callback, arg);
    spin_unlock_irqrestore(&set_timer_spin_lock, state);

    return NO_ERROR;
}

lk_time_t current_time(void) {
	return qtimer_current_time();
}

lk_bigtime_t current_time_hires(void) {
	return qtimer_current_time() * 1000000ULL;
}
