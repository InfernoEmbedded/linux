/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Allwinner A13 suspend to RAM final power-down sequence
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 *
 * This code is copied into SRAM A1 with fncpy() and runs from there,
 * because it puts the DRAM into self-refresh and afterwards nothing in
 * DRAM may be touched anymore - including the stack and page tables, so
 * it is written to use registers only, with the pages it accesses
 * preloaded into the TLB on entry.
 *
 * Called with:
 *   r0 - DRAM controller registers (virtual)
 *   r1 - TWI0 registers (virtual), controller set up by the caller
 *   r2 - value for the AXP209 power output control register
 *
 * Puts the DRAM into self-refresh, latches the DRAM pads and tells the
 * PMIC to cut the SoC power rails - which ends execution here. If the
 * PMIC write fails, the DRAM is brought back out of self-refresh and
 * the function returns nonzero so that cpu_suspend() unwinds into a
 * regular suspend abort.
 */

#include <linux/linkage.h>

#define SDR_DCR		0x004
#define SDR_DPCR	0x23c
#define SDR_HPCR	0x250

#define TWI_DATA	0x08
#define TWI_CNTR	0x0c
#define TWI_STAT	0x10

	.arm
	.align	3

ENTRY(sun5i_standby_blob)
	mov	r11, lr

	/* preload the TLB with the two device mappings */
	ldr	r3, [r0]
	ldr	r3, [r1]

	/* disable all DRAM host ports */
	add	r3, r0, #SDR_HPCR
	mov	r4, #31
1:	ldr	r5, [r3]
	bic	r5, r5, #1
	str	r5, [r3], #4
	subs	r4, r4, #1
	bne	1b

	/* precharge all, then enter self-refresh */
	mov	r6, #0x15
	bl	dcr_command
	mov	r6, #0x12
	bl	dcr_command

	/* latch the DRAM pads */
	mov	r5, #1
	str	r5, [r0, #SDR_DPCR]

	/* AXP209: single register write, addr 0x34 reg 0x12 value r2 */
	ldr	r5, [r1, #TWI_CNTR]
	orr	r5, r5, #0x20			@ M_STA
	str	r5, [r1, #TWI_CNTR]
	bl	twi_wait
	cmp	r5, #0x08
	bne	abort
	mov	r5, #0x68			@ SLA+W
	bl	twi_send
	cmp	r5, #0x18
	bne	abort
	mov	r5, #0x12			@ register address
	bl	twi_send
	cmp	r5, #0x28
	bne	abort
	mov	r5, r2				@ data
	bl	twi_send
	cmp	r5, #0x28
	bne	abort

	/* send STOP; the rails go down any moment now */
	ldr	r5, [r1, #TWI_CNTR]
	and	r5, r5, #0xc0
	orr	r5, r5, #0x10			@ M_STP
	str	r5, [r1, #TWI_CNTR]

	dsb
2:	wfi
	b	2b

abort:
	/* release the pads, exit self-refresh, refresh, ports back on */
	mov	r5, #0
	str	r5, [r0, #SDR_DPCR]
4:	ldr	r5, [r0, #SDR_DPCR]
	tst	r5, #1
	bne	4b
	mov	r6, #0x17
	bl	dcr_command
	mov	r6, #0x13
	bl	dcr_command

	add	r3, r0, #SDR_HPCR
	mov	r4, #31
3:	ldr	r5, [r3]
	orr	r5, r5, #1
	str	r5, [r3], #4
	subs	r4, r4, #1
	bne	3b

	mov	r0, #1
	bx	r11

	/* execute a DRAM controller command (r6) and wait for completion */
dcr_command:
	ldr	r5, [r0, #SDR_DCR]
	bic	r5, r5, #(0x1f << 27)
	orr	r5, r5, r6, lsl #27
	str	r5, [r0, #SDR_DCR]
1:	ldr	r5, [r0, #SDR_DCR]
	tst	r5, #(1 << 31)
	bne	1b
	mov	r7, #0x1000
2:	subs	r7, r7, #1
	bne	2b
	bx	lr

	/* write r5 to the TWI data register and continue the transfer */
twi_send:
	str	r5, [r1, #TWI_DATA]
	ldr	r5, [r1, #TWI_CNTR]
	bic	r5, r5, #0x08			@ clear INT_FLAG
	str	r5, [r1, #TWI_CNTR]

	/* wait for INT_FLAG, return the status code in r5 (0xff = timeout) */
twi_wait:
	mov	r7, #0x100000
1:	ldr	r5, [r1, #TWI_CNTR]
	tst	r5, #0x08
	bne	2f
	subs	r7, r7, #1
	bne	1b
	mov	r5, #0xff
	bx	lr
2:	ldr	r5, [r1, #TWI_STAT]
	bx	lr
ENDPROC(sun5i_standby_blob)

	.globl	sun5i_standby_blob_sz
sun5i_standby_blob_sz:
	.word	. - sun5i_standby_blob
