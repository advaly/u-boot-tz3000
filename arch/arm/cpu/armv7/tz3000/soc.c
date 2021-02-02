/*
 * Toshiba TZ3000 SoC specific functions
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
	puts("Toshiba TZ3000 Series\n");
	return 0;
}
#endif

#ifdef CONFIG_TZ3000_EMAC
#include <netdev.h>
int cpu_eth_init(bd_t *bis)
{
#ifdef CONFIG_TZ3000_EMAC_DISABLED
	char *ethprime = getenv("ethprime");
	if (!ethprime || strcmp(ethprime, "tz3000") != 0)
		return -1;
	/* deassert reset here instead of lowlevel_init */
	writel(0x11, TZ3000_PMU_SRON_ETHER);
	writel(0x01, TZ3000_PMU_CGOFF_ETHER);
	writel(0x11, TZ3000_PMU_SROFF_ETHER);
#endif
	return tz3000emac_register(bis);
}
#endif

void reset_cpu(unsigned long addr)
{
	writel(0x90, TZ3000_MBOX_BASE + 4);
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
