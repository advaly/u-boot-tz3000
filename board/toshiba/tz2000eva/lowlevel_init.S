/*
 * Board specific setup info
 *
 * (C) Copyright 2010,2013,2014 Toshiba corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm-offsets.h>
#include <config.h>
#include <linux/linkage.h>
#include <version.h>

ENTRY(lowlevel_init)

private_timer_init:
	/* Stop private timer / wdog */
	ldr	r4, =TZ2000_PTMR_BASE

	/* disable */
	mov	r5, #0
	str	r5, [r4, #0x08]
	str	r5, [r4, #0x28]

	/* clear interrupt */
	mov	r5, #1
	str	r5, [r4, #0x0c]
	str	r5, [r4, #0x2c]

	/* reset counter */
	mov	r5, #0
	str	r5, [r4, #0x00]
	str	r5, [r4, #0x20]

irq_init:
	cpsid	aif			@ disable IRQ, FIQ and data abort

set_init_sp:
	ldr	sp, =CONFIG_SYS_INIT_SP_ADDR
	bic	sp, sp, #7 /* 8-byte alignment for ABI compliance */
	sub	sp, #GD_SIZE
	bic	sp, sp, #7
	mov	r9, sp
	/*
	 * Save the old lr(passed in ip) and the current lr to stack
	 */
	push	{ip, lr}

wait_sep_boot:
	ldr	r4, =TZ2000_BOOTINFO_BOOTSTATUS
1:	ldr	r5, [r4, #0x00]
	cmp	r5, #0x0100
	bne	1b

global_timer_init:
	/* Start global timer */
	ldr	r4, =TZ2000_GTMR_BASE

	/* disable */
	mov	r5, #0x0
	str	r5, [r4, #0x08]

	/* clear interrupt */
	mov	r5, #0x1
	str	r5, [r4, #0x0c]

	/* reset counter */
	mov	r5, #0x0
	str	r5, [r4, #0x00]
	str	r5, [r4, #0x04]

	/* enable */
	mov	r5, #0x1
	str	r5, [r4, #0x08]

scu_init:
	bl	scu_disable
	bl	invalidate_scu_all
#ifdef CONFIG_SYS_DCACHE_OFF
	/* Dcache must be initialized even if DCACHE off */
	bl	v7_invalidate_dcache_all
#endif
	pop	{ip, pc}
ENDPROC(lowlevel_init)

	/* the literal pools origin */
	.ltorg
