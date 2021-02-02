/*
 * Toshiba TZ3000 SPI controller driver
 *
 * Copyright (C) 2010,2013,2014 Toshiba Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <tz3000_spi.h>
#include <asm/io.h>

#define PRIBUF_SIZE		8
#define SECBUF_SIZE		256
#define PRIBUF_XFERSIZE(len)	(((len) - 1) << 16)
#define SECBUF_XFERSIZE(len)	(((len) - 1) << 24)
#define PRIBUF_ENABLE		0x00000010
#define SECBUF_ENABLE		0x00000020

#define XFER_START		0x00000001
#define XFER_INPROGRESS		0x00000002
#define XFER_DONE		0x00000001

#define CHIPSELECT(ch)	((ch) << 1)

#define SPR_MASK		0x001f0000

#define MAX_SCSD		0xff
#define SCSD_MASK		0x0000ff00

#define RDCNTL_MASK		0xff00f8fc
#define ADDRBYTECNT		0x00000800
#define RDCMD(cmd)		((cmd) << 24)
#define DMYCNT(cnt)		((cnt) << 12)
#define DATA_SINGLE		0x00
#define DMMY_SINGLE		0x00
#define ADDR_SINGLE		0x00
#define DATA_DUAL		0x40
#define DMMY_DUAL		0x10
#define ADDR_DUAL		0x04
#define DATA_QUAD		0x80
#define DMMY_QUAD		0x20
#define ADDR_QUAD		0x08

#define INT_ENABLE		0x1
#define STAT_ALLCLEAR		0x0

#define TZ3000_SPIC_MAX_CHIPSELECT	2
#define TZ3000_SPIC_MAX_WAIT	(CONFIG_SYS_HZ / 10)	/* 100 msec */

struct tz3000_spi_slave {
	struct spi_slave slave;
	struct tz3000_spi_data spi_data;
	u8 spr;
	u16 scsd;
};

#define to_tz3000_spi_slave(s) container_of(s, struct tz3000_spi_slave, slave)

#ifndef CONFIG_TZ3000_SPI_MAX_SPEED
#define CONFIG_TZ3000_SPI_MAX_SPEED	40000000
#endif

static void tz3000_spi_setup_direct_access(struct spi_slave *slave)
{
	u32 r, max_hz;
	int i;
	struct tz3000_spi_slave *tss = to_tz3000_spi_slave(slave);
	unsigned int map_len = TZ3000_SPI_DIR_MAP_LEN(tss->slave.cs);
	u8 fden;
	u8 dir_spr;
	u8 opcode;
	u8 dummy_count;

	/* setup memory map & size */
	for (fden = 0; fden <= 0x0f; fden++) {
		if (map_len <= 0x10000 << fden)
			break;
	}

	writel(FBA(TZ3000_SPI_DIR_MAP_BASE(tss->slave.cs)) |
	       FDEN(fden) | DIR_ENABLE,
	       TZ3000_SPIC_MEMMAP(tss->slave.cs));

	/* setup baudrate */
	max_hz = min(tss->spi_data.max_map_read_hz,
		     CONFIG_TZ3000_SPI_MAX_SPEED);

	for (i = 0; i <= SPR_MAX; i++) {
		if (max_hz >= CONFIG_TZ3000_SPI_SPR_BASE_FREQ / (i + 1)) {
			dir_spr = i;
			break;
		}
	}
	if (i > SPR_MAX)
		dir_spr = SPR_MAX;

	/* setup direct access control */
	writel((readl(TZ3000_SPIC_DIRECT(tss->slave.cs))
		& ~(SPR_MASK | SCSD_MASK))
		| SCSD(tss->scsd)
		| SPR(dir_spr), TZ3000_SPIC_DIRECT(tss->slave.cs));
	/* setup direct access read control */
	r = readl(TZ3000_SPIC_RDCTRL(tss->slave.cs)) & ~RDCNTL_MASK;
#ifdef CONFIG_SPI_FLASH_4BYTE_MODE
	if (tss->spi_data.use_4byte_mode)
		r |= ADDRBYTECNT;
#endif
	opcode = tss->spi_data.dir_read_opcode;
	dummy_count = tss->spi_data.dir_read_dummy_count;
	switch (opcode) {
	default:
		debug("%s: Not supported read operation: op=0x%02x\n",
		      __func__, opcode);
		opcode = OPCODE_FAST_READ_SINGLE;
		dummy_count = 1;
		/* FALLTHRU */
	case OPCODE_FAST_READ_SINGLE:
		r |= DATA_SINGLE | DMMY_SINGLE | ADDR_SINGLE;
		break;
	case OPCODE_FAST_READ_DUAL_OUTPUT:
		r |= DATA_DUAL | DMMY_SINGLE | ADDR_SINGLE;
		break;
	case OPCODE_FAST_READ_DUAL_IO:
		r |= DATA_DUAL | DMMY_DUAL | ADDR_DUAL;
		break;
	case OPCODE_FAST_READ_QUAD_OUTPUT:
		r |= DATA_QUAD | DMMY_SINGLE | ADDR_SINGLE;
		break;
	case OPCODE_FAST_READ_QUAD_IO:
		r |= DATA_QUAD | DMMY_QUAD | ADDR_QUAD;
		break;
	}
	r |= RDCMD(opcode) | DMYCNT(dummy_count);

	writel(r, TZ3000_SPIC_RDCTRL(tss->slave.cs));
}

static void tz3000_spi_setup(struct spi_slave *slave)
{
	int i;
	u32 max_hz;
	struct tz3000_spi_slave *tss = to_tz3000_spi_slave(slave);

	/* setup baudrate */
	max_hz = min(tss->spi_data.max_hz, CONFIG_TZ3000_SPI_MAX_SPEED);

	for (i = 0; i <= SPR_MAX; i++) {
		if (max_hz >= CONFIG_TZ3000_SPI_SPR_BASE_FREQ / (i + 1)) {
			tss->spr = i;
			break;
		}
	}
	if (i > SPR_MAX)
		tss->spr = SPR_MAX;

	/* setup chip select deassert time */
	if (tss->spi_data.deassert_time) {
		tss->scsd = NSEC_TO_SCSD(tss->spi_data.deassert_time,
					 TZ3000_HBUS_CLK);
		tss->scsd = min(tss->scsd, MAX_SCSD);
	} else {
		tss->scsd = MAX_SCSD;
	}
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	if (cs < TZ3000_SPIC_MAX_CHIPSELECT) {
		return 1;
	} else {
		debug("%s: Invalid CS\n", __func__);
		return 0;
	}
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
				  unsigned int max_hz, unsigned int mode)
{
	struct tz3000_spi_slave *tss;

	if (!spi_cs_is_valid(bus, cs))
		return NULL;

	tss = spi_alloc_slave(struct tz3000_spi_slave, bus, cs);
	if (!tss) {
		debug("%s: Failed to allocate tz3000_spi_slave\n", __func__);
		return NULL;
	}

	tss->slave.tz3000_spi_data = &tss->spi_data;
	tss->slave.max_write_size = SECBUF_SIZE;
	tss->spi_data.max_hz = max_hz;
	tss->spi_data.max_map_read_hz = max_hz;
	tss->spi_data.dir_read_opcode = OPCODE_FAST_READ_SINGLE;
	tss->spi_data.dir_read_dummy_count = 1;
	tz3000_spi_setup(&tss->slave);
	tz3000_spi_setup_direct_access(&tss->slave);
	tss->slave.memory_map = (void *)TZ3000_SPI_DIR_MAP_BASE(cs);
	return &tss->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	struct tz3000_spi_slave *tss = to_tz3000_spi_slave(slave);
	free(tss);
}

int spi_claim_bus(struct spi_slave *slave)
{
	tz3000_spi_setup(slave);
	tz3000_spi_setup_direct_access(slave);
	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
}

void spi_cs_activate(struct spi_slave *slave)
{
}

void spi_cs_deactivate(struct spi_slave *slave)
{
}

static int tz3000_spi_wait_done(void)
{
	unsigned long timebase = get_timer(0);

	do {
		if (!(readl(TZ3000_SPIC_STATUS) & XFER_INPROGRESS))
			return 0;
	} while (get_timer(timebase) < TZ3000_SPIC_MAX_WAIT);

	if (!(readl(TZ3000_SPIC_STATUS) & XFER_INPROGRESS))
		return 0;

	return 1;
}

static void tz3000_spi_set_data(u32 reg, const u8 *buf, u32 len, u32 offset)
{
	int i;

	for (i = 0; i < len; i++)
		writeb(*buf++, reg + i);
}

static void tz3000_spi_get_data(u32 reg, u8 *buf, u32 len, u32 offset)
{
	int i;

	for (i = 0; i < len; i++)
		*buf++ = readb(reg + i);
}

static int tz3000_spi_transfer(struct tz3000_spi_slave *tss,
			       unsigned int pri_len, unsigned int sec_len)
{
	u32 r;

	/* set control register 0 */
	writel((readl(TZ3000_SPIC_CONTROL0) & ~(SPR_MASK | SCSD_MASK))
	       | SCSD(tss->scsd)
	       | SPR(tss->spr), TZ3000_SPIC_CONTROL0);
	/* set control register 1 */
	r = 0;
	if (pri_len > 0)
		r |= PRIBUF_ENABLE | PRIBUF_XFERSIZE(pri_len);

	if (sec_len > 0)
		r |= SECBUF_ENABLE | SECBUF_XFERSIZE(sec_len);

	r |= CHIPSELECT(tss->slave.cs) | XFER_START;

	debug("%s: ch=%d, spr=0x%02x, pri_len=%d, sec_len=%d\n",
	      __func__, tss->slave.cs, tss->spr, pri_len, sec_len);

	writel(r, TZ3000_SPIC_CONTROL1);

	if (tz3000_spi_wait_done()) {
		debug("%s: Transfer timeout\n", __func__);
		return -1;
	}

	return 0;
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
	     void *din, unsigned long flags)
{
	static unsigned int pri_len;
	static unsigned int sec_len;
	struct tz3000_spi_slave *tss = to_tz3000_spi_slave(slave);
	const u8 *tx = dout;
	u8 *rx = din;
	uint bytes = bitlen / 8;
	int ret = 0;

	if (flags & (SPI_XFER_MMAP | SPI_XFER_MMAP_END))
		return 0;

	if (flags & SPI_XFER_BEGIN) {
		/* clear transfer size */
		pri_len = 0;
		sec_len = 0;
	}

	if (tx == NULL && rx == NULL) {
		if (bytes == 0)
			return 0;
		debug("%s: Invalid parameters\n", __func__);
		return -1;
	} else if (tx && rx) {
		if ((flags & SPI_XFER_BEGIN) == 0 ||
		    (flags & SPI_XFER_END) == 0 ||
		    sec_len + bytes > SECBUF_SIZE) {
			debug("%s: Invalid parameters\n", __func__);
			return -1;
		}
		tz3000_spi_set_data(TZ3000_SPIC_SECBUF, tx, bytes, 0);

		sec_len = bytes;
		ret = tz3000_spi_transfer(tss, pri_len, sec_len);
		if (ret)
			return -1;

		tz3000_spi_get_data(TZ3000_SPIC_SECBUF, rx, bytes, 0);
	} else if (tx) {
		if ((flags & SPI_XFER_BEGIN) && bytes <= PRIBUF_SIZE) {
			tz3000_spi_set_data(TZ3000_SPIC_PRIBUF, tx, bytes, 0);
			pri_len = bytes;
		} else {
			if (sec_len + bytes > SECBUF_SIZE) {
				debug("%s: Invalid parameters\n", __func__);
				return -1;
			}
			tz3000_spi_set_data(TZ3000_SPIC_SECBUF,
					    tx, bytes, sec_len);
			sec_len += bytes;
		}

		if (flags & SPI_XFER_END) {
			ret = tz3000_spi_transfer(tss, pri_len, sec_len);
			if (ret)
				return -1;
		}
	} else if (rx) {
		if (!(flags & SPI_XFER_END) ||
		    sec_len + bytes > SECBUF_SIZE) {
			debug("%s: Invalid parameters\n", __func__);
			return -1;
		}

		sec_len += bytes;
		ret = tz3000_spi_transfer(tss, pri_len, sec_len);
		if (ret)
			return -1;

		tz3000_spi_get_data(TZ3000_SPIC_SECBUF,
				    rx, bytes, sec_len - bytes);
	}

	return 0;
}

void spi_init(void)
{
	int i;

	for (i = 0; i < 2; i++)
		writel(readl(TZ3000_SPIC_DIRECT(i)) | SDCE(SDCE_NEGEDGE),
		       TZ3000_SPIC_DIRECT(i));
	writel(readl(TZ3000_SPIC_CONTROL0) | SDCE(SDCE_NEGEDGE),
	       TZ3000_SPIC_CONTROL0);
	writel(readl(TZ3000_SPIC_INT) & ~INT_ENABLE, TZ3000_SPIC_INT);
	writel(STAT_ALLCLEAR, TZ3000_SPIC_STATUS);
}
