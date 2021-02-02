/*
 * Toshiba TZ2000 SCU operation code
 *
 * Copyright (C) 2010,2013,2014 Toshiba Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>

.align 5
.global scu_enable
.global scu_disable
.global invalidate_scu_all

/*
 *	void scu_enable(void)
 *
 *	Enable the SCU.
 *
 *	Corrupted registers: r0, r1
 */
scu_enable:
	ldr	r1, =TZ2000_SCU_CONTROL
	ldr	r0, [r1, #0x0]
	orr	r0, r0, #SCU_ENABLE
	str	r0, [r1, #0x0]
	mov	pc, lr

/*
 *	void scu_disable(void)
 *
 *	Disable the SCU.
 *
 *	Corrupted registers: r0, r1
 */
scu_disable:
	ldr	r1, =TZ2000_SCU_CONTROL
	ldr	r0, [r1, #0x0]
	bic	r0, r0, #SCU_ENABLE
	str	r0, [r1, #0x0]
	mov	pc, lr

/*
 *	void invalidate_scu_all(void)
 *
 *	Invalidate the SCU.
 *
 *	Corrupted registers: r0, r1
 */
invalidate_scu_all:
	ldr	r1, =TZ2000_SCU_INV_ALL
	ldr	r0, =SCU_INV_ALL_CPU_WAYS
	str	r0, [r1, #0x0]
	mov	pc, lr
