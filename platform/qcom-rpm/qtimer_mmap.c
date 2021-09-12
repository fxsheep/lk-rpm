/* Copyright (c) 2012, The Linux Foundation. All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#include <debug.h>
#include <lk/reg.h>
//#include <compiler.h>
#include <platform/qtimer.h>
#include <arch/defines.h>
//#include <platform/irqs.h>
#include <platform/iomap.h>
//#include <platform/interrupts.h>
#include <platform/qtimer_mmap_hw.h>
#include <arch/ops.h>
#include <arch/arm/cm.h>

#define QTMR_IRQn     51

static platform_timer_callback timer_callback;
static void *timer_arg;
static lk_time_t timer_interval;
/* time in ms from start of LK. */
static volatile uint32_t current_time;
static uint32_t tick_count;

static void qtimer_enable();

void qtimer_irq(void)
{
	arm_cm_irq_entry();
	current_time += timer_interval;

	/* Program the down counter again to get
	 * an interrupt after timer_interval msecs
	 */
	writel(tick_count, QTMR_V1_CNTP_TVAL);
	mb();

	timer_callback(timer_arg, current_time);
	arm_cm_irq_exit(1);
}


/* Programs the Physical Secure Down counter timer.
 * interval : Counter ticks till expiry interrupt is fired.
 */
void qtimer_set_physical_timer(lk_time_t msecs_interval,
							   platform_timer_callback tmr_callback,
							   void *tmr_arg)
{
	qtimer_disable();

	/* Save the timer interval and call back data*/
	tick_count = msecs_interval * qtimer_tick_rate() / 1000;;
	timer_interval = msecs_interval;
	timer_arg = tmr_arg;
	timer_callback = tmr_callback;

	/* Set Physical Down Counter */
	writel(tick_count, QTMR_V1_CNTP_TVAL);
	mb();

	/* configure rpm interrupt */
	int v2;
	unsigned int v3;
	v2 = 1 << (51 & 0x1F);
  	v3 = 51 >> 5;
	*(uint32_t *)(4 * v3 + 0x80030) |= v2;
	*(uint32_t *)(4 * v3 + 0x80038) = (*(uint32_t *)(4 * v3 + 0x80038) & ~v2);

	NVIC_EnableIRQ(QTMR_IRQn);

	qtimer_enable();
}


/* Function to return the frequency of the timer */
uint32_t qtimer_get_frequency()
{
	uint32_t freq;

	/* Read the Global counter frequency */
	/* freq = readl(QTMR_V1_CNTFRQ); */
	/* TODO: remove this when bootchaint sets up the frequency. */
	freq = 19200000;

	return freq;
}

static void qtimer_enable()
{
	uint32_t ctrl;

	ctrl = readl(QTMR_V1_CNTP_CTL);

	/* Program CTRL Register */
	ctrl |= QTMR_TIMER_CTRL_ENABLE;
	ctrl &= ~QTMR_TIMER_CTRL_INT_MASK;

	writel(ctrl, QTMR_V1_CNTP_CTL);
	mb();
}

void qtimer_disable()
{
	uint32_t ctrl;

	ctrl = readl(QTMR_V1_CNTP_CTL);

	/* program cntrl register */
	ctrl &= ~QTMR_TIMER_CTRL_ENABLE;
	ctrl |= QTMR_TIMER_CTRL_INT_MASK;

	writel(ctrl, QTMR_V1_CNTP_CTL);
	mb();
}

inline __ALWAYS_INLINE uint64_t qtimer_get_phy_timer_cnt()
{
	uint32_t phy_cnt_lo;
	uint32_t phy_cnt_hi_1;
	uint32_t phy_cnt_hi_2;

	do {
		phy_cnt_hi_1 = readl(QTMR_V1_CNTPCT_HI);
		phy_cnt_lo = readl(QTMR_V1_CNTPCT_LO);
		phy_cnt_hi_2 = readl(QTMR_V1_CNTPCT_HI);
    } while (phy_cnt_hi_1 != phy_cnt_hi_2);

	return ((uint64_t)phy_cnt_hi_1 << 32) | phy_cnt_lo;
}

uint32_t qtimer_current_time()
{
	return current_time;
}
