/*
 * TOSHIBA TZ3000 NAND_2 controller driver
 *
 * Copyright (C) 2010,2013,2014 Toshiba Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <nand.h>
#include <malloc.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <asm/io.h>

#undef TZ3000_HAM_NAND_DEBUG	/* config debug */
#undef TZ3000_HAM_NANDC_DATA_DEBUG	/* data debug */
#undef TZ3000_HAM_NAND_BIT_DEBUG	/* all 0xff bit err debug */
#define NAND_NAME	"tz3000_nand2"

#define ndwritel(val, addr)	__raw_writel(val, addr)
#define ndreadl(addr)		__raw_readl(addr)

/* Status (Read only)*/
#define TZ3000_HAM_STATUS_RAWECCINTST	(1 << 11)
#define TZ3000_HAM_STATUS_ECCINTST	(1 << 9)
#define TZ3000_HAM_STATUS_ECCINTEN	(1 << 7)
#define TZ3000_HAM_STATUS_RAWINTST	(1 << 5)
#define TZ3000_HAM_STATUS_INTST		(1 << 3)
#define TZ3000_HAM_STATUS_INTEN		(1 << 1)

/* Set Config (Write only)*/
#define TZ3000_HAM_SETCONFIG_ECCINTEN(x)	((x) << 5)
#define TZ3000_HAM_SETCONFIG_INTEN(x)		((x) << 0)

/* Clr Config (Write only) */
#define TZ3000_HAM_CLRCONFIG_ECCINTDIS(x)	((x) << 5)
#define TZ3000_HAM_CLRCONFIG_INTCLR(x)		((x) << 3)
#define TZ3000_HAM_CLRCONFIG_INTDIS(x)		((x) << 0)

/* Direct Command (Write only) */
#define TZ3000_HAM_DIRECTCMD_CMDTYPE(x) ((x) << 21)

/* Set Cycles (Write only) */
#define TZ3000_HAM_SETCYCLES_TRR(x)	((x) << 20)
#define TZ3000_HAM_SETCYCLES_TAR(x)	((x) << 17)
#define TZ3000_HAM_SETCYCLES_TCLR(x)	((x) << 14)
#define TZ3000_HAM_SETCYCLES_TWP(x)	((x) << 11)
#define TZ3000_HAM_SETCYCLES_TREA(x)	((x) << 8)
#define TZ3000_HAM_SETCYCLES_TWC(x)	((x) << 4)
#define TZ3000_HAM_SETCYCLES_TRC(x)	((x) << 0)

/* Set Opmode (Write only) */
#define TZ3000_HAM_SETOPMODE_MW(x)	((x) << 0)

/* Cycles0 (Read only) */
#define TZ3000_HAM_CYCLES0_TRR		(0xF << 20)
#define TZ3000_HAM_CYCLES0_TAR		(0x7 << 17)
#define TZ3000_HAM_CYCLES0_TCLR		(0x7 << 14)
#define TZ3000_HAM_CYCLES0_TWP		(0x7 << 10)
#define TZ3000_HAM_CYCLES0_TREA		(0x7 << 8)
#define TZ3000_HAM_CYCLES0_TWC		(0xF << 4)
#define TZ3000_HAM_CYCLES0_TRC		(0xF << 0)

/* Opmode0 (Read only) */
#define TZ3000_HAM_OPMODE0_MW		(0x3 << 0)

/* ECC status (Read only) */
#define TZ3000_HAM_ECCSTATUS_READ	(0x1F << 25)
#define TZ3000_HAM_ECCSTATUS_CAN_CRCT	(0x1F << 20)
#define TZ3000_HAM_ECCSTATUS_FAIL	(0x0F << 15)
#define TZ3000_HAM_ECCSTATUS_VAL_VALID	(0x1F << 10)
#define TZ3000_HAM_ECCSTATUS_RD_NOT_WR	(0x1  << 9)
#define TZ3000_HAM_ECCSTATUS_LAST	(0x3  << 7)
#define TZ3000_HAM_ECCSTATUS_STATUS	(0x1  << 6)
#define TZ3000_HAM_ECCSTATUS_RAW_INTST	(0x3F << 0)

/* ECC Configuration (Rd/Wr) */
#define TZ3000_HAM_ECCMEMCFG_EXBLOCK_SIZE(x)	((x) << 11)
#define TZ3000_HAM_ECCMEMCFG_EXBLOCK_EN(x)	((x) << 10)
#define TZ3000_HAM_ECCMEMCFG_INT_ABORT(x)	((x) << 9)
#define TZ3000_HAM_ECCMEMCFG_INT_PASS(x)	((x) << 8)
#define TZ3000_HAM_ECCMEMCFG_IGNORE_ADD_EIGHT	(0x1 << 7)
#define TZ3000_HAM_ECCMEMCFG_JUMP(x)		((x) << 5)
#define TZ3000_HAM_ECCMEMCFG_MODE(x)		((x) << 2)
#define TZ3000_HAM_ECCMEMCFG_PAGESIZE(x)	((x) << 0)
#define TZ3000_HAM_ECCMEMCFG_RESERVE		(0x00000010)

/* ECC Command1 (Rd/Wr) */
#define TZ3000_HAM_ECCMEMCMD1_RD_END_VAL	(0x1  << 24)
#define TZ3000_HAM_ECCMEMCMD1_RD_END		(0xFF << 16)
#define TZ3000_HAM_ECCMEMCMD1_RD_CMD		(0xFF << 8)
#define TZ3000_HAM_ECCMEMCMD1_WR_CMD		(0xFF << 0)

/* ECC Command2 (Rd/Wr) */
#define TZ3000_HAM_ECCMEMCMD2_RD_CHANGE_END_VAL	(0x1  << 24)
#define TZ3000_HAM_ECCMEMCMD2_RD_CHANGE_END		(0xFF << 16)
#define TZ3000_HAM_ECCMEMCMD2_RD_CHANGE		(0xFF << 8)
#define TZ3000_HAM_ECCMEMCMD2_WR_CHANGE		(0xFF << 0)

/* ECC Value 0-4 */
#define TZ3000_HAM_ECCVALUE_INT		(0x1 << 31)
#define TZ3000_HAM_ECCVALUE_VALID	(0x1 << 30)
#define TZ3000_HAM_ECCVALUE_READ	(0x1 << 29)
#define TZ3000_HAM_ECCVALUE_FAIL	(0x1 << 28)
#define TZ3000_HAM_ECCVALUE_CORRECT	(0x1 << 27)
#define TZ3000_HAM_ECCVALUE_WRITE	(0x00FFFFFF << 0)
#define TZ3000_HAM_ECCVALUE_READ_BYTE	(0x00FFFFF8)
#define TZ3000_HAM_ECCVALUE_READ_BIT	(0x7 << 0)

/* Command Phase set */
#define TZ3000_HAM_CMD_CHIP_ADDR	((0x0a) << 24)
#define TZ3000_HAM_CMD_ADDCYCLE(x)	((x) << 21)
#define TZ3000_HAM_CMD_END_CMD_VAL(x)	((x) << 20)
#define TZ3000_HAM_CMD_19		((0x0) << 19)
#define TZ3000_HAM_CMD_END_CMD(x)	((x) << 11)
#define TZ3000_HAM_CMD_START_CMD(x)	((x) << 3)

/* Data Phase set */
#define TZ3000_HAM_DATA_CHIP_ADDR	((0x0a) << 24)
#define TZ3000_HAM_DATA_CLEAR_CS(x)	((x) << 21)
#define TZ3000_HAM_DATA_END_CMD_VAL(x)	((x) << 20)
#define TZ3000_HAM_DATA_19		((0x1) << 19)
#define TZ3000_HAM_DATA_END_CMD(x)	((x) << 11)
#define TZ3000_HAM_DATA_ECC_LAST(x)	(host->ecc_mode == ECC_ON ? \
					(x) << 10 : 0)

#define GET_RAWINTST_TIMEOUT		1000 * (CONFIG_SYS_HZ/1000)
#define GET_ECC_STATUS_TIMEOUT		1000 * (CONFIG_SYS_HZ/1000)

#define ECC_OFF				0x0
#define ECC_ON				0x2

#define LARGE_PAGE			0x1
#define SMALL_PAGE			0x0

#define MACRONIX_MX30LF1G08_TIMING \
	(TZ3000_HAM_SETCYCLES_TRR(0x2) | \
	 TZ3000_HAM_SETCYCLES_TAR(0x2) | \
	 TZ3000_HAM_SETCYCLES_TCLR(0x2) | \
	 TZ3000_HAM_SETCYCLES_TWP(0x2) | \
	 TZ3000_HAM_SETCYCLES_TREA(0x3) | \
	 TZ3000_HAM_SETCYCLES_TWC(0x4) | \
	 TZ3000_HAM_SETCYCLES_TRC(0x5))
#define TOSHIBA_TC58DVG02D5_TIMING /* TC58BVG0S3 */	\
	(TZ3000_HAM_SETCYCLES_TRR(0x2) | \
	 TZ3000_HAM_SETCYCLES_TAR(0x1) | \
	 TZ3000_HAM_SETCYCLES_TCLR(0x1) | \
	 TZ3000_HAM_SETCYCLES_TWP(0x2) | \
	 TZ3000_HAM_SETCYCLES_TREA(0x2) | \
	 TZ3000_HAM_SETCYCLES_TWC(0x4) | \
	 TZ3000_HAM_SETCYCLES_TRC(0x4))
#ifdef CONFIG_NAND_CHIP_MACRONIX_MX30LF1G08
#define DEFAULT_AC_TIMING MACRONIX_MX30LF1G08_TIMING
#endif
#ifdef CONFIG_NAND_CHIP_TOSHIBA_TC58DVG02D5
#define DEFAULT_AC_TIMING TOSHIBA_TC58DVG02D5_TIMING
#endif

struct tz3000_ham_nand_regs {
	u32 status;
	u32 reserved00;
	u32 setconfig;
	u32 clrconfig;
	u32 directcmd;
	u32 setcycles;
	u32 setopmode;
	u32 reserved01[0x39];
	u32 cycles0;
	u32 opmode0;
	u32 reserved02[0x7E];
	u32 ecc_status;
	u32 ecc_memcfg;
	u32 ecc_memcommand1;
	u32 ecc_memcommand2;
	u32 reserved03[0x2];
	u32 ecc_value[5];
	u32 reserved04[0x335];
};

struct tz3000_ham_nand_chip_set {
	u32 full_cycle;
	u32 erase_cycle;
	u32 page_size;
	u32 column_val;
	u32 exblocksize;
	int ecc_count;
	u32 ac_timing;
	int autoset_enable;
};

struct nand_ac_timing_set {
	int man_id;
	int dev_id;
	u32 ac_timing;
};

static struct nand_ac_timing_set man_ac_chip[] = {
	{NAND_MFR_MACRONIX, 0xF1, MACRONIX_MX30LF1G08_TIMING},
	{NAND_MFR_TOSHIBA, 0xF1, TOSHIBA_TC58DVG02D5_TIMING},
	{0x0, 0x0, 0x00FFFFFF},
	{0x100,}
};

static struct tz3000_ham_nand_chip_set chip_select = {
#ifdef DEFAULT_AC_TIMING
	.ac_timing = DEFAULT_AC_TIMING,
	.autoset_enable = 0
#else
	.ac_timing = 0x00FFFFFF,
	.autoset_enable = 1
#endif
};

struct tz3000_ham_nand_host {
	struct mtd_info mtd;
	struct nand_chip *nand;
	struct tz3000_ham_nand_regs *regs;	/* NAND Register set */
	struct tz3000_ham_nand_chip_set *chip_set;	/* NAND chip set */
	int read_byte_flag;	/* read_byte address point flag */
	int page_addr;		/* keep page_address value */
	int ecc_mode;		/* set ecc mode */
	u32 read_byte_buf;	/* keep read_byte value */
	u32 command;		/* keep command value */
	u32 oob_read_flag;	/* (Data + OOB) or OOB read check flag */
	u32 oob_write_flag;	/* (Data + OOB) or OOB write check flag */
};

static struct nand_ecclayout large_nand_oob_32 = {
	.eccbytes = 12,
	.eccpos = {
		   32, 33, 34,
		   35, 36, 37,
		   38, 39, 40,
		   41, 42, 43},
	.oobfree = {
		    {.offset = 2,
		     .length = 29}
		    }
};

static struct nand_ecclayout small_nand_oob_8 = {
	.eccbytes = 3,
	.eccpos = {
		   8, 9, 10},
	.oobfree = {
		    {.offset = 0,
		     .length = 4}
		    }
};

static int tz3000_ham_nand_wait_op_done(struct tz3000_ham_nand_host *host);
static int tz3000_ham_nand_set_chip(struct mtd_info *);

static void tz3000_ham_nand_set_init(struct tz3000_ham_nand_host *host)
{
	struct tz3000_ham_nand_regs *regs = host->regs;
	u32 set_config = 0;

	u32 set_clrconfig =
		TZ3000_HAM_CLRCONFIG_ECCINTDIS(0x1) |
		TZ3000_HAM_CLRCONFIG_INTDIS(0x1);
	u32 set_cycles = 0;
	u32 set_opmode = 0;
	u32 set_directcmd = TZ3000_HAM_DIRECTCMD_CMDTYPE(0x2);

	set_cycles = host->chip_set->ac_timing;

	ndwritel(set_config, &regs->setconfig);
	ndwritel(set_clrconfig, &regs->clrconfig);
	ndwritel(set_cycles, &regs->setcycles);
	ndwritel(set_opmode, &regs->setopmode);
	/* update cycles0 and opmode0 */
	ndwritel(set_directcmd, &regs->directcmd);

#ifdef TZ3000_HAM_NAND_DEBUG
	u32 reg;
	reg = ndreadl(&regs->cycles0);
	printf("cycles0 = 0x%08x\n", reg);
	reg = ndreadl(&regs->opmode0);
	printf("opmode0 = 0x%08x\n", reg);
#endif
}

static void tz3000_ham_nand_set_ecc(struct tz3000_ham_nand_host *host,
				    char change)
{
	struct tz3000_ham_nand_regs *regs = host->regs;
	u32 set_eccmemcfg = 0;

	if (host->nand->ecc.mode == NAND_ECC_BENAND) {
		/* always ECC disable when BENAND */
		host->ecc_mode = ECC_OFF;
		goto exit;
	}

	set_eccmemcfg =
		TZ3000_HAM_ECCMEMCFG_RESERVE |
		TZ3000_HAM_ECCMEMCFG_EXBLOCK_EN(0x1) |
		TZ3000_HAM_ECCMEMCFG_INT_ABORT(0x0) |
		TZ3000_HAM_ECCMEMCFG_INT_PASS(0x0) |
		TZ3000_HAM_ECCMEMCFG_JUMP(0x0);

	if (host->chip_set->page_size != LARGE_PAGE) {
		set_eccmemcfg |=
			TZ3000_HAM_ECCMEMCFG_EXBLOCK_SIZE(0x1) |
			TZ3000_HAM_ECCMEMCFG_IGNORE_ADD_EIGHT |
			TZ3000_HAM_ECCMEMCFG_PAGESIZE(0x01);
	} else {
		set_eccmemcfg |=
			TZ3000_HAM_ECCMEMCFG_EXBLOCK_SIZE(0x3) |
			TZ3000_HAM_ECCMEMCFG_PAGESIZE(0x03);
	}

	set_eccmemcfg |= TZ3000_HAM_ECCMEMCFG_MODE(change);
exit:
#ifdef TZ3000_HAM_NAND_DEBUG
	printf("set_eccmemcfg = 0x%08x\n", set_eccmemcfg);
#endif
	ndwritel(set_eccmemcfg, &regs->ecc_memcfg);
}

static void tz3000_ham_nand_pre_command(unsigned int command,
					struct tz3000_ham_nand_host *host)
{
	u32 tz3000_ham_addr;
	u32 full_cycle;
	full_cycle = host->chip_set->full_cycle;

	tz3000_ham_addr =
		TZ3000_HAM_CMD_CHIP_ADDR | TZ3000_HAM_CMD_ADDCYCLE(full_cycle) |
		TZ3000_HAM_CMD_END_CMD_VAL(0x0) | TZ3000_HAM_CMD_19 |
		TZ3000_HAM_CMD_END_CMD(0x0) | TZ3000_HAM_CMD_START_CMD(command);
	ndwritel(0x0, tz3000_ham_addr);
#ifdef TZ3000_HAM_NANDC_DATA_DEBUG
	printf("set pre command addr = 0x%08x\n", tz3000_ham_addr);
#endif
	tz3000_ham_nand_wait_op_done(host);
}

static void tz3000_ham_nand_write_buf(struct mtd_info *mtd, const u8 *buf,
				      int len)
{
	struct nand_chip *chip = mtd->priv;
	struct tz3000_ham_nand_host *host = chip->priv;
	u32 set_data, tz3000_ham_addr;
	int i;

	if (len != mtd->oobsize) {
		tz3000_ham_addr =
			TZ3000_HAM_DATA_CHIP_ADDR |
			TZ3000_HAM_DATA_CLEAR_CS(0x1) |
			TZ3000_HAM_DATA_END_CMD_VAL(0x0) |
			TZ3000_HAM_DATA_19 |
			TZ3000_HAM_DATA_END_CMD(NAND_CMD_PAGEPROG) |
			TZ3000_HAM_DATA_ECC_LAST(0x0);

		for (i = 0; i < len; i += 4) {
			set_data = buf[i] + (buf[i + 1] << 8) +
				(buf[i + 2] << 16) + (buf[i + 3] << 24);
			ndwritel(set_data, tz3000_ham_addr);
#ifdef TZ3000_HAM_NANDC_DATA_DEBUG
			printf("set_data = 0x%08x, addr = 0x%08x\n", set_data,
			       tz3000_ham_addr);
#endif
		}

		host->oob_write_flag = 1;
	} else {
		tz3000_ham_addr =
			TZ3000_HAM_DATA_CHIP_ADDR |
			TZ3000_HAM_DATA_CLEAR_CS(0x1) |
			TZ3000_HAM_DATA_END_CMD_VAL(0x0) |
			TZ3000_HAM_DATA_19 |
			TZ3000_HAM_DATA_END_CMD(NAND_CMD_PAGEPROG) |
			TZ3000_HAM_DATA_ECC_LAST(0x0);

		for (i = 0; i < (len - 4); i += 4) {
			set_data = buf[i] + (buf[i + 1] << 8) +
				(buf[i + 2] << 16) + (buf[i + 3] << 24);
			ndwritel(set_data, tz3000_ham_addr);
#ifdef TZ3000_HAM_NANDC_DATA_DEBUG
			printf("set_data = 0x%08x, addr = 0x%08x\n",
			       set_data, tz3000_ham_addr);
#endif
		}

		tz3000_ham_addr =
			TZ3000_HAM_DATA_CHIP_ADDR |
			TZ3000_HAM_DATA_CLEAR_CS(0x1) |
			TZ3000_HAM_DATA_END_CMD_VAL(0x1) |
			TZ3000_HAM_DATA_19 |
			TZ3000_HAM_DATA_END_CMD(NAND_CMD_PAGEPROG) |
			TZ3000_HAM_DATA_ECC_LAST(0x0);
		set_data =
			buf[len - 4] + (buf[len - 3] << 8) +
			(buf[len - 2] << 16) + (buf[len - 1] << 24);
		ndwritel(set_data, tz3000_ham_addr);
#ifdef TZ3000_HAM_NANDC_DATA_DEBUG
		printf("last set_data = 0x%08x, addr = 0x%08x\n",
		       set_data, tz3000_ham_addr);
#endif

		host->oob_write_flag = 0;
		tz3000_ham_nand_wait_op_done(host);
	}
}

static int tz3000_ham_nand_write_extra_buf(struct mtd_info *mtd,
					   const u8 *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct tz3000_ham_nand_host *host = chip->priv;
	u32 set_data, tz3000_ham_addr;
	int i;

	tz3000_ham_addr =
		TZ3000_HAM_DATA_CHIP_ADDR | TZ3000_HAM_DATA_CLEAR_CS(0x1) |
		TZ3000_HAM_DATA_END_CMD_VAL(0x0) | TZ3000_HAM_DATA_19 |
		TZ3000_HAM_DATA_END_CMD(NAND_CMD_PAGEPROG) |
		TZ3000_HAM_DATA_ECC_LAST(0x0);

	for (i = 0; i < (host->chip_set->exblocksize - 4); i += 4) {
		set_data = buf[i] + (buf[i + 1] << 8) +
			(buf[i + 2] << 16) + (buf[i + 3] << 24);
		ndwritel(set_data, tz3000_ham_addr);
#ifdef TZ3000_HAM_NANDC_DATA_DEBUG
		printf("set_data = 0x%08x, addr = 0x%08x\n",
		       set_data, tz3000_ham_addr);
#endif
	}

	tz3000_ham_addr =
		TZ3000_HAM_DATA_CHIP_ADDR | TZ3000_HAM_DATA_CLEAR_CS(0x1) |
		TZ3000_HAM_DATA_END_CMD_VAL(0x1) | TZ3000_HAM_DATA_19 |
		TZ3000_HAM_DATA_END_CMD(NAND_CMD_PAGEPROG) |
		TZ3000_HAM_DATA_ECC_LAST(0x1);

	set_data = buf[i] + (buf[i + 1] << 8) +
		(buf[i + 2] << 16) + (buf[i + 3] << 24);
	ndwritel(set_data, tz3000_ham_addr);

	host->oob_write_flag = 0;
	return tz3000_ham_nand_wait_op_done(host);
}

static void tz3000_ham_nand_read_buf(struct mtd_info *mtd, u8 *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct tz3000_ham_nand_host *host = chip->priv;
	u32 tz3000_ham_addr;
	u32 *buf32;
	int i;

	buf32 = (u32 *)buf;

	/* READ Data Phase */
	tz3000_ham_addr =
		TZ3000_HAM_DATA_CHIP_ADDR | TZ3000_HAM_DATA_CLEAR_CS(0x1) |
		TZ3000_HAM_DATA_END_CMD_VAL(0x0) | TZ3000_HAM_DATA_19 |
		TZ3000_HAM_DATA_END_CMD(0x00) | TZ3000_HAM_DATA_ECC_LAST(0x0);

	if (len != mtd->oobsize) {
		for (i = 0; i < len / 4; i++)
			buf32[i] = ndreadl(tz3000_ham_addr);

		host->oob_read_flag = 1;
		return;
	}

	if (host->oob_read_flag != 1) {
		/* READOOB */
		for (i = 0; i < len / 4; i++)
			buf32[i] = ndreadl(tz3000_ham_addr);
	} else {
		/* Read Extra Block */
		for (i = 0; i < (host->chip_set->exblocksize - 4) / 4; i++)
			buf32[i] = ndreadl(tz3000_ham_addr);

		tz3000_ham_addr =
			TZ3000_HAM_DATA_CHIP_ADDR |
			TZ3000_HAM_DATA_CLEAR_CS(0x1) |
			TZ3000_HAM_DATA_END_CMD_VAL(0x0) |
			TZ3000_HAM_DATA_19 |
			TZ3000_HAM_DATA_END_CMD(0x00) |
			TZ3000_HAM_DATA_ECC_LAST(0x1);

		buf32[i] = ndreadl(tz3000_ham_addr);
	}
	host->oob_read_flag = 0;
}

static u8 tz3000_ham_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct tz3000_ham_nand_host *host = chip->priv;
	u32 tz3000_ham_addr, tmp_buf;
	int i;
	u8 buf;

	i = host->read_byte_flag;

	if (i == 0) {
		/* Data Phase */
		tz3000_ham_addr =
			TZ3000_HAM_DATA_CHIP_ADDR |
			TZ3000_HAM_DATA_CLEAR_CS(0x1) |
			TZ3000_HAM_DATA_END_CMD_VAL(0x0) |
			TZ3000_HAM_DATA_19 |
			TZ3000_HAM_DATA_END_CMD(0x00) |
			TZ3000_HAM_DATA_ECC_LAST(0x0);

		tmp_buf = ndreadl(tz3000_ham_addr);
		host->read_byte_buf = tmp_buf;
	}

	buf = (u8)((host->read_byte_buf >> (8 * i)) & 0xFF);
	host->read_byte_flag++;
	if (host->read_byte_flag == 4)
		host->read_byte_flag = 0;

	return buf;
}

static int tz3000_ham_nand_read_page_hwecc(struct mtd_info *mtd,
					   struct nand_chip *chip,
					   uint8_t *buf, int oob_required,
					   int page)
{
	chip = mtd->priv;
	struct tz3000_ham_nand_host *host = chip->priv;
	struct tz3000_ham_nand_regs *regs = host->regs;
	u32 ecc_bit, ecc_byte, reg;
	uint8_t *buf_tmp, byte;
	int i, tmp;
	unsigned long timeout;

	buf_tmp = buf;
	chip->read_buf(mtd, buf, mtd->writesize);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	timeout = get_timer(0);
	do {
		reg = ndreadl(&regs->ecc_status);
		if (get_timer(timeout) > GET_ECC_STATUS_TIMEOUT) {
			printf(NAND_NAME ":%s: ECC STATUS timeout\n",
			       __func__);
			return -ETIMEDOUT;
		}
#ifdef TZ3000_HAM_NAND_DEBUG
		printf("read ecc_status = 0x%08x\n", reg);
#endif
	} while ((reg & TZ3000_HAM_ECCSTATUS_STATUS) != 0);

	reg = ndreadl(&regs->ecc_status);
	tmp = ((reg & TZ3000_HAM_ECCSTATUS_LAST) >> 0x7);
	switch (tmp) {
	case 1:
	case 2:
	case 3:
		printf("%s: ECC STATUS ERR = %02x\n", __func__, tmp);
		return -EIO;

	default:
		break;
	}

	for (i = 0; i < host->chip_set->ecc_count; i++) {
		reg = ndreadl(&regs->ecc_value[i]);
		if (reg & TZ3000_HAM_ECCVALUE_FAIL) {
			if (!(reg & TZ3000_HAM_ECCVALUE_CORRECT)) {
				printf("%s: ecc value[%d] correct err\n",
				       __func__, i);
				mtd->ecc_stats.failed++;
				continue;
			}
			buf_tmp = buf + (0x200 * i);

			ecc_bit = reg & TZ3000_HAM_ECCVALUE_READ_BIT;
			ecc_byte =
				((reg & (TZ3000_HAM_ECCVALUE_READ_BYTE)) >> 3);
#ifdef TZ3000_HAM_NAND_DEBUG
			printf("ecc_value[%d] = 0x%08x\n", i, reg);
			printf("ecc_byte = 0x%x\n", ecc_byte);
			printf("ecc_bit = 0x%x\n", ecc_bit);
			printf("Base buf = %p\n", buf);
			printf("before:%p = 0x%02x\n", &buf_tmp[ecc_byte],
			       buf_tmp[ecc_byte]);
#endif
			byte = buf_tmp[ecc_byte];
			byte =
				((byte & ~(1 << ecc_bit)) |
				 ((~byte) & (1 << ecc_bit)));
			buf_tmp[ecc_byte] = byte;
#ifdef TZ3000_HAM_NAND_DEBUG
			printf("after:%p = 0x%02x\n", &buf_tmp[ecc_byte],
			       buf_tmp[ecc_byte]);
#endif
			mtd->ecc_stats.corrected++;
		}
	}
	return 0;
}

static int tz3000_ham_nand_write_page_hwecc(struct mtd_info *mtd,
					    struct nand_chip *chip,
					    const uint8_t *buf,
					    int oob_required)
{
	struct tz3000_ham_nand_host *host = chip->priv;
	int ret = 0;

	chip->write_buf(mtd, buf, mtd->writesize);
	if (host->oob_write_flag != 0)
		ret = tz3000_ham_nand_write_extra_buf(mtd, chip->oob_poi,
						      mtd->oobsize);
	else
		chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
	return ret;
}

static int tz3000_ham_nand_verify_buf(struct mtd_info *mtd,
				      const u_char *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct tz3000_ham_nand_host *host = chip->priv;
	u32 tz3000_ham_addr;
	u32 *buf32;
	int i;

	buf32 = (u32 *)buf;

	/* READ Data Phase */
	tz3000_ham_addr =
		TZ3000_HAM_DATA_CHIP_ADDR | TZ3000_HAM_DATA_CLEAR_CS(0x1) |
		TZ3000_HAM_DATA_END_CMD_VAL(0x0) | TZ3000_HAM_DATA_19 |
		TZ3000_HAM_DATA_END_CMD(0x00) | TZ3000_HAM_DATA_ECC_LAST(0x0);

	for (i = 0; i < len / 4; i++) {
		if (buf32[i] != ndreadl(tz3000_ham_addr))
			return -EFAULT;
	}
	return 0;
}

static int tz3000_ham_nand_wait_op_done(struct tz3000_ham_nand_host *host)
{
	struct tz3000_ham_nand_regs *regs = host->regs;
	unsigned long timeout;
	u32 set_clrconfig =
		TZ3000_HAM_CLRCONFIG_INTCLR(0x1) |
		TZ3000_HAM_CLRCONFIG_ECCINTDIS(0x1) |
		TZ3000_HAM_CLRCONFIG_INTDIS(0x1);
	int reg = 0;

	timeout = get_timer(0);
	do {
		reg = ndreadl(&regs->status);
		if (get_timer(timeout) > GET_RAWINTST_TIMEOUT) {
			printf(NAND_NAME ":%s: RAWINTST timeout\n", __func__);
			ndwritel(set_clrconfig, &regs->clrconfig);
			return -ETIMEDOUT;
		}
	} while (!(reg & TZ3000_HAM_STATUS_RAWINTST));
	ndwritel(set_clrconfig, &regs->clrconfig);
	return 0;
}

static void tz3000_ham_nand_select_chip(struct mtd_info *mtd, int chip)
{
	return;
}

static int tz3000_ham_nand_set_be_chip(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct tz3000_ham_nand_host *host = this->priv;
	int ret;

	host->chip_set->page_size = LARGE_PAGE;
	host->chip_set->exblocksize = mtd->oobsize;
	host->chip_set->ecc_count = 0;

	/* setting chip */
	switch (mtd->writesize) {
	case 2048:
	case 4096:
		host->chip_set->column_val = 0x10;
		break;
	default:
		pr_warn("ERR:Write Size is unmattch.\n");
		return -EIO;
	}

	ret = (int)(this->chipsize >> 20);
	switch (ret) {
	case 128:
		host->chip_set->full_cycle = 4;
		host->chip_set->erase_cycle = 2;
		break;
	case 256:
		host->chip_set->full_cycle = 5;
		host->chip_set->erase_cycle = 3;
		break;
	case 512:
		host->chip_set->full_cycle = 5;
		host->chip_set->erase_cycle = 3;
		break;
	default:
		pr_warn("ERR:Chip Size is unmattch.\n");
		return -EIO;
	}

	return 0;
}

static int tz3000_ham_nand_set_chip(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct tz3000_ham_nand_host *host = this->priv;
	struct tz3000_ham_nand_regs *regs = host->regs;
	int i, ret;
	u32 set_cycles = 0;
	u32 set_directcmd = TZ3000_HAM_DIRECTCMD_CMDTYPE(0x2);

	if (host->chip_set->autoset_enable != 0) {
		for (i = 0; man_ac_chip[i].man_id != 0x100; i++) {
			if (this->man_id == man_ac_chip[i].man_id &&
			    this->dev_id == man_ac_chip[i].dev_id) {
				host->chip_set->ac_timing =
				    man_ac_chip[i].ac_timing;
				break;
			}

			if (man_ac_chip[i].man_id == 0x0) {
				host->chip_set->ac_timing =
				    man_ac_chip[i].ac_timing;
				printf(" Warning :ChipID is unmatch ac_timing table.If want to New chip,");
				printf("Please add \"ac_timing\" at table. Now, using default ac_timing.\n");
				break;
			}
		}
		set_cycles = host->chip_set->ac_timing;
		ndwritel(set_cycles, &regs->setcycles);
		ndwritel(set_directcmd, &regs->directcmd);
	}
#ifdef TZ3000_HAM_NAND_DEBUG
	u32 reg;
	reg = ndreadl(&regs->cycles0);
	printf("cycles0 = 0x%08x\n", reg);
	printf("writesize = %d\n", mtd->writesize);
#endif

	if (host->nand->ecc.mode == NAND_ECC_BENAND)
		return tz3000_ham_nand_set_be_chip(mtd);

	switch (mtd->writesize) {
	case 512:
		host->chip_set->page_size = SMALL_PAGE;
		host->chip_set->column_val = 0x8;
		host->chip_set->exblocksize = 0x8;
		host->chip_set->ecc_count = 1;
		break;
	case 2048:
		host->chip_set->page_size = LARGE_PAGE;
		host->chip_set->column_val = 0x10;
		host->chip_set->exblocksize = 0x20;
		host->chip_set->ecc_count = 4;
		break;
	default:
		printf("ERR : writesize is unmatch.\n");
		return -EIO;
	}

	ret = (int)(this->chipsize >> 20);
	switch (ret) {
	case 32:
		host->chip_set->full_cycle = 3;
		host->chip_set->erase_cycle = 2;
		break;
	case 64:
		host->chip_set->full_cycle = 4;
		host->chip_set->erase_cycle = 3;
		break;
	case 128:
		host->chip_set->full_cycle = 4;
		if (host->chip_set->page_size != LARGE_PAGE)
			host->chip_set->erase_cycle = 3;
		else
			host->chip_set->erase_cycle = 2;
		break;
	case 256:
		if (host->chip_set->page_size != LARGE_PAGE)
			host->chip_set->full_cycle = 4;
		else
			host->chip_set->full_cycle = 5;
		host->chip_set->erase_cycle = 3;
		break;
	case 512:
	case 1024:
	case 2048:
		host->chip_set->full_cycle = 5;
		host->chip_set->erase_cycle = 3;
		break;
	default:
		printf("ERR; chipsize is unmatch.\n");
		return -EIO;
	}

	if (host->chip_set->page_size != LARGE_PAGE)
		this->ecc.layout = &small_nand_oob_8;
	else
		this->ecc.layout = &large_nand_oob_32;

#ifdef TZ3000_HAM_NAND_DEBUG
	printf("full_cycle = %d, erase_cycle = %d\n",
	       host->chip_set->full_cycle, host->chip_set->erase_cycle);
#endif
	return 0;
}

static int tz3000_ham_nand_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct tz3000_ham_nand_host *host = chip->priv;
	u32 tz3000_ham_addr, set_addr;
	u8 status;

	tz3000_ham_addr =
		TZ3000_HAM_CMD_CHIP_ADDR | TZ3000_HAM_CMD_ADDCYCLE(0x0) |
		TZ3000_HAM_CMD_END_CMD_VAL(0x0) | TZ3000_HAM_CMD_19 |
		TZ3000_HAM_CMD_END_CMD(0x0) |
		TZ3000_HAM_CMD_START_CMD(NAND_CMD_STATUS);

	set_addr = 0x0;
	ndwritel(set_addr, tz3000_ham_addr);

	host->read_byte_flag = 0;
	status = tz3000_ham_nand_read_byte(mtd);

	if (status & NAND_STATUS_READY)
		return 1;	/* Ready */
	return 0;		/* Busy */
}

static int tz3000_ham_nand_waitfunc(struct mtd_info *mtd,
				    struct nand_chip *chip)
{
	struct tz3000_ham_nand_host *host = chip->priv;
	u32 tz3000_ham_addr, set_addr;
	u8 status;

	tz3000_ham_addr =
		TZ3000_HAM_CMD_CHIP_ADDR | TZ3000_HAM_CMD_ADDCYCLE(0x0) |
		TZ3000_HAM_CMD_END_CMD_VAL(0x0) | TZ3000_HAM_CMD_19 |
		TZ3000_HAM_CMD_END_CMD(0x0) |
		TZ3000_HAM_CMD_START_CMD(NAND_CMD_STATUS);

	set_addr = 0x0;
	ndwritel(set_addr, tz3000_ham_addr);
	host->read_byte_flag = 0;

	status = tz3000_ham_nand_read_byte(mtd);
	return status;
}

static void tz3000_ham_nand_command(struct mtd_info *mtd,
				    unsigned int command, int column,
				    int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct tz3000_ham_nand_host *host = chip->priv;
	unsigned int cmd, end_cmd;
	u32 tz3000_ham_addr, set_addr, full_cycle, page_size, erase_cycle;

	full_cycle = host->chip_set->full_cycle;
	erase_cycle = host->chip_set->erase_cycle;
	page_size = host->chip_set->page_size;
	host->page_addr = page_addr;

	/* Command Phase */
	switch (command) {
	case NAND_CMD_READOOB:
		host->oob_read_flag = 0;
		if (host->ecc_mode != ECC_OFF) {
			host->ecc_mode = ECC_OFF;
			tz3000_ham_nand_set_ecc(host, host->ecc_mode);
		}
		if (host->chip_set->page_size != LARGE_PAGE) {
			tz3000_ham_addr =
				TZ3000_HAM_CMD_CHIP_ADDR |
				TZ3000_HAM_CMD_ADDCYCLE(full_cycle) |
				TZ3000_HAM_CMD_END_CMD_VAL(0x0) |
				TZ3000_HAM_CMD_19 |
				TZ3000_HAM_CMD_END_CMD(0x0) |
				TZ3000_HAM_CMD_START_CMD(command);
			set_addr =
				(page_addr << host->chip_set->column_val) |
				column;
			ndwritel(set_addr, tz3000_ham_addr);
			tz3000_ham_nand_wait_op_done(host);

			break;
		} else {
			host->command = NAND_CMD_READOOB;
			column += mtd->writesize;
			command = NAND_CMD_READ0;
		}
	case NAND_CMD_READ0:

		if (host->command != NAND_CMD_READOOB &&
		    host->ecc_mode != ECC_ON) {
			host->ecc_mode = ECC_ON;
			tz3000_ham_nand_set_ecc(host, host->ecc_mode);
			host->command = NAND_CMD_READ0;
		}

		if (host->chip_set->page_size != LARGE_PAGE)
			end_cmd = 0x0;
		else
			end_cmd = NAND_CMD_READSTART;

		tz3000_ham_addr =
			TZ3000_HAM_CMD_CHIP_ADDR |
			TZ3000_HAM_CMD_ADDCYCLE(full_cycle) |
			TZ3000_HAM_CMD_END_CMD_VAL(page_size) |
			TZ3000_HAM_CMD_19 |
			TZ3000_HAM_CMD_END_CMD(end_cmd) |
			TZ3000_HAM_CMD_START_CMD(command);

		set_addr = (page_addr << host->chip_set->column_val) | column;
		ndwritel(set_addr, tz3000_ham_addr);
#ifdef TZ3000_HAM_NANDC_DATA_DEBUG
		printf("set_addr = 0x%08x, addr = 0x%08x\n", set_addr,
		       tz3000_ham_addr);
#endif
		if (host->chip_set->full_cycle >= 0x5) {
			set_addr = (page_addr >> 0x10);
			ndwritel(set_addr, tz3000_ham_addr);
		}
		tz3000_ham_nand_wait_op_done(host);
		host->command = NAND_CMD_READ0;

		break;

	case NAND_CMD_READID:
		tz3000_ham_addr =
			TZ3000_HAM_CMD_CHIP_ADDR |
			TZ3000_HAM_CMD_ADDCYCLE(0x1) |
			TZ3000_HAM_CMD_END_CMD_VAL(0x0) | TZ3000_HAM_CMD_19 |
			TZ3000_HAM_CMD_END_CMD(0x0) |
			TZ3000_HAM_CMD_START_CMD(command);
		set_addr = 0x0;
		ndwritel(set_addr, tz3000_ham_addr);
		host->read_byte_flag = 0;
		break;

	case NAND_CMD_SEQIN:
		if (host->chip_set->page_size != LARGE_PAGE) {
			if (column >= mtd->writesize) {
				column -= mtd->writesize;
				cmd = NAND_CMD_READOOB;
				if (host->ecc_mode != ECC_OFF)
					host->ecc_mode = ECC_OFF;
			} else if (column < 256) {
				cmd = NAND_CMD_READ0;
				if (host->ecc_mode != ECC_ON)
					host->ecc_mode = ECC_ON;
			} else {
				column -= 256;
				cmd = NAND_CMD_READ1;
				if (host->ecc_mode != ECC_ON)
					host->ecc_mode = ECC_ON;
			}
			tz3000_ham_nand_pre_command(cmd, host);
			tz3000_ham_nand_set_ecc(host, host->ecc_mode);
		} else {
			if (column >= mtd->writesize) {
				if (host->ecc_mode != ECC_OFF)
					host->ecc_mode = ECC_OFF;
			} else {
				if (host->ecc_mode != ECC_ON)
					host->ecc_mode = ECC_ON;
			}
			tz3000_ham_nand_set_ecc(host, host->ecc_mode);
		}

		tz3000_ham_addr =
			TZ3000_HAM_CMD_CHIP_ADDR |
			TZ3000_HAM_CMD_ADDCYCLE(full_cycle) |
			TZ3000_HAM_CMD_END_CMD_VAL(0x0) | TZ3000_HAM_CMD_19 |
			TZ3000_HAM_CMD_END_CMD(0x0) |
			TZ3000_HAM_CMD_START_CMD(command);

		set_addr = (page_addr << host->chip_set->column_val) | column;
		ndwritel(set_addr, tz3000_ham_addr);
#ifdef TZ3000_HAM_NANDC_DATA_DEBUG
		printf("set_addr = 0x%08x, addr = 0x%08x\n", set_addr,
		       tz3000_ham_addr);
#endif
		if (host->chip_set->full_cycle >= 5) {
			set_addr = (page_addr >> 0x10);
			ndwritel(set_addr, tz3000_ham_addr);
		}
		break;

	case NAND_CMD_ERASE1:
		tz3000_ham_addr =
			TZ3000_HAM_CMD_CHIP_ADDR |
			TZ3000_HAM_CMD_ADDCYCLE(erase_cycle) |
			TZ3000_HAM_CMD_END_CMD_VAL(0x1) | TZ3000_HAM_CMD_19 |
			TZ3000_HAM_CMD_END_CMD(NAND_CMD_ERASE2) |
			TZ3000_HAM_CMD_START_CMD(command);
		set_addr = page_addr;
		ndwritel(set_addr, tz3000_ham_addr);
		tz3000_ham_nand_wait_op_done(host);
		break;

	case NAND_CMD_RESET:
	case NAND_CMD_STATUS:
		tz3000_ham_addr =
			TZ3000_HAM_CMD_CHIP_ADDR |
			TZ3000_HAM_CMD_ADDCYCLE(0x0) |
			TZ3000_HAM_CMD_END_CMD_VAL(0x0) | TZ3000_HAM_CMD_19 |
			TZ3000_HAM_CMD_END_CMD(0x0) |
			TZ3000_HAM_CMD_START_CMD(command);

		set_addr = 0x0;
		ndwritel(set_addr, tz3000_ham_addr);
		host->read_byte_flag = 0;

		if (command == NAND_CMD_RESET)
			tz3000_ham_nand_wait_op_done(host);
		break;

	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE2:
		break;

	default:
		printf(NAND_NAME ":Your setting command %d is Invalid\n",
		       command);
		break;
	}
}

int board_nand_init(struct nand_chip *this)
{
	struct tz3000_ham_nand_host *host;
	struct mtd_info *mtd = NULL;
	int ret;

	host = malloc(sizeof(struct tz3000_ham_nand_host));

	if (!host) {
		printf(NAND_NAME ":%s: nand malloc faild\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	memset(host, 0, sizeof(struct tz3000_ham_nand_host));

	host->chip_set = &chip_select;

	this->chip_delay = 50;	/* 50us wait */
	this->waitfunc = tz3000_ham_nand_waitfunc;
	this->cmdfunc = tz3000_ham_nand_command;
	this->read_byte = tz3000_ham_nand_read_byte;
	this->read_buf = tz3000_ham_nand_read_buf;
	this->write_buf = tz3000_ham_nand_write_buf;
	this->verify_buf = tz3000_ham_nand_verify_buf;
	this->dev_ready = tz3000_ham_nand_dev_ready;
	this->select_chip = tz3000_ham_nand_select_chip;

	this->ecc.size = 512;
	this->ecc.read_page = tz3000_ham_nand_read_page_hwecc;
	this->ecc.write_page = tz3000_ham_nand_write_page_hwecc;

	mtd = &host->mtd;
	mtd->priv = this;
	host->nand = this;
	this->priv = host;

	this->ecc.mode = NAND_ECC_HW;
	this->ecc.strength = 1;
	this->options |= NAND_NO_SUBPAGE_WRITE;

	host->regs = (struct tz3000_ham_nand_regs *)TZ3000_HAM_NANDC_BASE;
	host->oob_read_flag = 0;
	host->oob_write_flag = 0;

	if (host->chip_set->autoset_enable != 0)
		host->chip_set->ac_timing = 0x00FFFFFF;

	/* SET INIT and ECC config */
	tz3000_ham_nand_set_init(host);
	host->command = NAND_CMD_READ0;

	if (nand_scan_ident(mtd, 1, NULL)) {
		ret = -ENXIO;
		goto err;
	}

	ret = tz3000_ham_nand_set_chip(mtd);
	if (ret < 0)
		goto err;

	tz3000_ham_nand_set_ecc(host, ECC_ON);

	return 0;

err:
#ifndef CONFIG_NAND_SPL
	if (host) {
		free(host);
		host = NULL;
	}
#endif
	printf(NAND_NAME ":failed.\n");
	return ret;
}
