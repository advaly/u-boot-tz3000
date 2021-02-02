/*
 * Toshiba TZ2000 Timer support
 *
 * Copyright (C) 2010,2013,2014 Toshiba Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>

#define TIMER_LOAD		TZ2000_PTMR_LOAD
#define TIMER_COUNTER		TZ2000_PTMR_COUNTER
#define TIMER_CTRL		TZ2000_PTMR_CONTROL

#define TIMER_ENABLE		(1 << 0)
#define TIMER_DISABLE		(0 << 0)
#define TIMER_AUTORELOAD	(1 << 1)

DECLARE_GLOBAL_DATA_PTR;

#define timestamp gd->arch.tbl
#define lastinc gd->arch.lastinc

#define TIMER_CLOCK		tz2000_periph_clk
#define TIMER_LOAD_VAL		0xffffffff

int timer_init(void)
{
	writel(TIMER_DISABLE, TIMER_CTRL);
	writel(TIMER_LOAD_VAL, TIMER_LOAD);
	writel(TIMER_ENABLE | TIMER_AUTORELOAD, TIMER_CTRL);

	/* reset time, capture current incrementer value time */
	lastinc = (TIMER_LOAD_VAL - readl(TIMER_COUNTER))
		/ (TIMER_CLOCK / CONFIG_SYS_HZ);
	timestamp = 0;		/* start "advancing" time stamp from 0 */

	return 0;
}

/*
 * timer without interrupts
 */
ulong get_timer(ulong base)
{
	return get_timer_masked() - base;
}

/* delay x useconds */
void __udelay(unsigned long usec)
{
	long tmo = usec * (TIMER_CLOCK / 1000000);
	unsigned long tmp, now, last = readl(TIMER_COUNTER);

	while (tmo > 0) {
		now = readl(TIMER_COUNTER);
		if (last < now) {	/* timer wrap around */
			tmp = last + (TIMER_LOAD_VAL - now);
			tmo -= tmp;
		} else {
			tmo -= last - now;
		}
		last = now;
	}
}

ulong get_timer_masked(void)
{
	/* current tick value */
	ulong now = (TIMER_LOAD_VAL - readl(TIMER_COUNTER))
		/ (TIMER_CLOCK / CONFIG_SYS_HZ);

	if (now >= lastinc)
		/* normal mode (non roll) */
		/* move stamp fordward with absoulte diff ticks */
		timestamp += (now - lastinc);
	else			/* we have rollover of incrementer */
		timestamp += ((TIMER_LOAD_VAL / (TIMER_CLOCK / CONFIG_SYS_HZ))
			      - lastinc) + now;
	lastinc = now;
	return timestamp;
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	return get_timer(0);
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	return CONFIG_SYS_HZ;
}
