/*
 * Toshiba TZ2000 SoC specific functions
 *
 * Copyright (C) 2010,2013,2014 Toshiba Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_DISPLAY_CPUINFO)
int print_cpuinfo(void)
{
	puts("Core: ");
	puts("ARM Cortex-A9\n");
	puts("CPU: ");
	puts("Toshiba TZ2000 Series\n");
	return 0;
}
#endif

#ifdef CONFIG_TZ3000_EMAC
#include <netdev.h>
int cpu_eth_init(bd_t *bis)
{
	return tz3000emac_register(bis);
}
#endif

void reset_cpu(unsigned long addr)
{
	writel(0x90, TZ2000_MBOX_BASE + 4);
	while (1)
		;
}

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif
