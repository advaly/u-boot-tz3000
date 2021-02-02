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
	ldr	r4, =TZ3000_PTMR_BASE

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

find_cpu0:
	mrc	p15, 0, r0, c0, c0, 5
	ands	r0, r0, #0x03

	/* CPU0: continue to boot u-boot */
	beq	set_init_sp

not_cpu0_dcache_init:
	dmb				@ ensure ordering
					@ 	with previous memory accesses

	mrc	p15, 1, r0, c0, c0, 1	@ read clidr
	ands	r3, r0, #0x7000000	@ extract loc from clidr
	mov	r3, r3, lsr #23		@ left align loc bit field
	beq	finished		@ if loc is 0, then no need to clean
	mov	r10, #0			@ start clean at cache level 0
loop1:
	add	r2, r10, r10, lsr #1	@ work out 3x current cache level
	mov	r1, r0, lsr r2		@ extract cache type bits from clidr
	and	r1, r1, #7		@ mask of the bits
					@ 	for current cache only
	cmp	r1, #2			@ see what cache we have at this level
	blt	skip			@ skip if no cache, or just i-cache
	mcr	p15, 2, r10, c0, c0, 0	@ select current cache level in cssr
	isb				@ isb to sych the new cssr&csidr
	mrc	p15, 1, r1, c0, c0, 0	@ read the new csidr
	and	r2, r1, #7		@ extract the length of the cache lines
	add	r2, r2, #4		@ add 4 (line length offset)
	ldr	r4, =0x3ff
	ands	r4, r4, r1, lsr #3	@ find maximum number on the way size
	clz	r5, r4			@ find bit position
					@ 	of way size increment
	ldr	r7, =0x7fff
	ands	r7, r7, r1, lsr #13	@ extract max number of the index size
loop2:
	mov	r9, r4			@ create working copy of max way size
loop3:
	orr	r11, r10, r9, lsl r5	@ factor way and cache number into r11
	orr	r11, r11, r7, lsl r2	@ factor index number into r11

	mcr	p15, 0, r11, c7, c6, 2	@ invalidate by set/way
	subs	r9, r9, #1		@ decrement the way
	bge	loop3
	subs	r7, r7, #1		@ decrement the index
	bge	loop2
skip:
	add	r10, r10, #2		@ increment cache number
	cmp	r3, r10
	bgt	loop1
finished:
	mov	r10, #0			@ swith back to cache level 0
	mcr	p15, 2, r10, c0, c0, 0	@ select current cache level in cssr

	dsb
	isb

not_cpu0_get_into_wfi:
        ldr	r4, =TZ3000_IC_BASE
        mov	r5, #0x01
        str	r5, [r4, #0x00]		@ enable interrupt controller
        mov	r5, #0xf0
        str	r5, [r4, #0x04]		@ set priority mask
	ldr	pc, =0x048		@ get into wfi

	/*** Only CPU0 executes following codes ***/
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
	ldr	r4, =TZ3000_BOOTINFO_BOOTSTATUS
1:	ldr	r5, [r4, #0x00]
	cmp	r5, #0x0100
	bne	1b

global_timer_init:
	/* Start global timer */
	ldr	r4, =TZ3000_GTMR_BASE

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
