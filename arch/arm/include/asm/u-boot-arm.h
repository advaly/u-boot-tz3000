/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Alex Zuepke <azu@sysgo.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _U_BOOT_ARM_H_
#define _U_BOOT_ARM_H_	1

/* for the following variables, see start.S */
extern ulong IRQ_STACK_START;	/* top of IRQ stack */
extern ulong FIQ_STACK_START;	/* top of FIQ stack */
extern ulong _datarel_start_ofs;
extern ulong _datarelrolocal_start_ofs;
extern ulong _datarellocal_start_ofs;
extern ulong _datarelro_start_ofs;
extern ulong IRQ_STACK_START_IN;	/* 8 bytes in IRQ stack */

/* cpu/.../cpu.c */
int	cpu_init(void);
int	cleanup_before_linux(void);
#ifdef CONFIG_HAS_SETUP_AFTER_LINUX
int	setup_after_linux(void);
#else
static inline int setup_after_linux(void)
{
	return 0;
}
#endif

/* Set up ARMv7 MMU, caches and TLBs */
void	cpu_init_cp15(void);

/* cpu/.../arch/cpu.c */
int	arch_cpu_init(void);
int	arch_misc_init(void);
int	arch_early_init_r(void);

/* board/.../... */
int	board_init(void);
int	dram_init (void);
void	dram_init_banksize (void);

/* cpu/.../interrupt.c */
int	arch_interrupt_init	(void);
void	reset_timer_masked	(void);
ulong	get_timer_masked	(void);
void	udelay_masked		(unsigned long usec);

#endif	/* _U_BOOT_ARM_H_ */
