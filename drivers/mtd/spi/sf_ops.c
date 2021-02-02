/*
 * SPI flash operations
 *
 * Copyright (C) 2008 Atmel Corporation
 * Copyright (C) 2010 Reinhard Meyer, EMK Elektronik
 * Copyright (C) 2013 Jagannadha Sutradharudu Teki, Xilinx Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spi.h>
#include <spi_flash.h>
#include <watchdog.h>

#include "sf_internal.h"

static int spi_flash_addr(struct spi_flash *flash, u32 addr, u8 *cmd)
{
	/* cmd[0] is actual command */
	if (spi_flash_use_4byte_mode(flash)) {
		cmd[1] = addr >> 24;
		cmd[2] = addr >> 16;
		cmd[3] = addr >> 8;
		cmd[4] = addr;
		return 5;
	}
	cmd[1] = addr >> 16;
	cmd[2] = addr >> 8;
	cmd[3] = addr >> 0;
	return 4;
}

int spi_flash_cmd_write_status(struct spi_flash *flash, u8 sr)
{
	u8 cmd;
	int ret;

	cmd = CMD_WRITE_STATUS;
	ret = spi_flash_write_common(flash, &cmd, 1, &sr, 1);
	if (ret < 0) {
		debug("SF: fail to write status register\n");
		return ret;
	}

	return 0;
}

#ifdef CONFIG_SPI_FLASH_BAR
static int spi_flash_cmd_bankaddr_write(struct spi_flash *flash, u8 bank_sel)
{
	u8 cmd;
	int ret;

	if (flash->bank_curr == bank_sel) {
		debug("SF: not require to enable bank%d\n", bank_sel);
		return 0;
	}

	cmd = flash->bank_write_cmd;
	ret = spi_flash_write_common(flash, &cmd, 1, &bank_sel, 1);
	if (ret < 0) {
		debug("SF: fail to write bank register\n");
		return ret;
	}
	flash->bank_curr = bank_sel;

	return 0;
}

static int spi_flash_bank(struct spi_flash *flash, u32 offset)
{
	u8 bank_sel;
	int ret;

	bank_sel = offset / SPI_FLASH_16MB_BOUN;

	ret = spi_flash_cmd_bankaddr_write(flash, bank_sel);
	if (ret) {
		debug("SF: fail to set bank%d\n", bank_sel);
		return ret;
	}

	return 0;
}
#endif

int spi_flash_cmd_wait_ready(struct spi_flash *flash, unsigned long timeout)
{
	struct spi_slave *spi = flash->spi;
	unsigned long timebase;
	int ret;
	u8 status;
	u8 check_status = 0x0;
	u8 poll_bit = STATUS_WIP;
	u8 cmd = flash->poll_cmd;

	if (cmd == CMD_FLAG_STATUS) {
		poll_bit = STATUS_PEC;
		check_status = poll_bit;
	}

#ifdef CONFIG_TZ3000_SPI
	/* TZ3000 SPI does not support polling without BEGIN/END */
	timebase = get_timer(0);
	do {
		WATCHDOG_RESET();
		ret = spi_flash_cmd(spi, cmd, &status, 1);
		if (ret) {
			debug("SF: fail to read %s status register\n",
			      cmd == CMD_READ_STATUS ? "read" : "flag");
			return ret;
		}
		if (cmd == CMD_FLAG_STATUS) {
			u8 status2;
			/* check twice (see n25q512a datasheet) */
			ret = spi_flash_cmd(spi, cmd, &status2, 1);
			if (ret)
				return -1;
			status &= status2;
		}
		if ((status & poll_bit) == check_status)
			break;
	} while (get_timer(timebase) < timeout);
#else /* CONFIG_TZ3000_SPI */
	ret = spi_xfer(spi, 8, &cmd, NULL, SPI_XFER_BEGIN);
	if (ret) {
		debug("SF: fail to read %s status register\n",
		      cmd == CMD_READ_STATUS ? "read" : "flag");
		return ret;
	}

	timebase = get_timer(0);
	do {
		WATCHDOG_RESET();

		ret = spi_xfer(spi, 8, NULL, &status, 0);
		if (ret)
			return -1;

		if (cmd == CMD_FLAG_STATUS) {
			u8 status2;
			/* check twice (see n25q512a datasheet) */
			ret = spi_xfer(spi, 8, NULL, &status2, 0);
			if (ret)
				return -1;
			status &= status2;
		}
		if ((status & poll_bit) == check_status)
			break;

	} while (get_timer(timebase) < timeout);

	spi_xfer(spi, 0, NULL, NULL, SPI_XFER_END);
#endif /* CONFIG_TZ3000_SPI */

	if ((status & poll_bit) == check_status)
		return 0;

	/* Timed out */
	debug("SF: time out!\n");
	return -1;
}

int spi_flash_write_common(struct spi_flash *flash, const u8 *cmd,
		size_t cmd_len, const void *buf, size_t buf_len)
{
	struct spi_slave *spi = flash->spi;
	unsigned long timeout = SPI_FLASH_PROG_TIMEOUT;
	int ret;

	if (buf == NULL)
		timeout = SPI_FLASH_PAGE_ERASE_TIMEOUT;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		debug("SF: unable to claim SPI bus\n");
		return ret;
	}

	ret = spi_flash_cmd_write_enable(flash);
	if (ret < 0) {
		debug("SF: enabling write failed\n");
		return ret;
	}

	ret = spi_flash_cmd_write(spi, cmd, cmd_len, buf, buf_len);
	if (ret < 0) {
		debug("SF: write cmd failed\n");
		return ret;
	}

	ret = spi_flash_cmd_wait_ready(flash, timeout);
	if (ret < 0) {
		debug("SF: write %s timed out\n",
		      timeout == SPI_FLASH_PROG_TIMEOUT ?
			"program" : "page erase");
		return ret;
	}

	spi_release_bus(spi);

	return ret;
}

int spi_flash_cmd_erase_ops(struct spi_flash *flash, u32 offset, size_t len)
{
	u32 erase_size;
	u8 cmd[5];
	int cmd_len;
	int ret = -1;

	erase_size = flash->erase_size;
	if (offset % erase_size || len % erase_size) {
		debug("SF: Erase offset/length not multiple of erase size\n");
		return -1;
	}

	cmd[0] = flash->erase_cmd;
	while (len) {
#ifdef CONFIG_SPI_FLASH_BAR
		ret = spi_flash_bank(flash, offset);
		if (ret < 0)
			return ret;
#endif
		cmd_len = spi_flash_addr(flash, offset, cmd);

		debug("SF: erase %2x %2x %2x %2x (%x)\n", cmd[0], cmd[1],
		      cmd[2], cmd[3], offset);

		ret = spi_flash_write_common(flash, cmd, cmd_len, NULL, 0);
		if (ret < 0) {
			debug("SF: erase failed\n");
			break;
		}

		offset += erase_size;
		len -= erase_size;
	}

	return ret;
}

int spi_flash_cmd_write_ops(struct spi_flash *flash, u32 offset,
		size_t len, const void *buf)
{
	unsigned long byte_addr, page_size;
	size_t chunk_len, actual;
	u8 cmd[5];
	int cmd_len;
	int ret = -1;

	page_size = flash->page_size;

	cmd[0] = CMD_PAGE_PROGRAM;
	for (actual = 0; actual < len; actual += chunk_len) {
#ifdef CONFIG_SPI_FLASH_BAR
		ret = spi_flash_bank(flash, offset);
		if (ret < 0)
			return ret;
#endif
		byte_addr = offset % page_size;
		chunk_len = min(len - actual, page_size - byte_addr);

		if (flash->spi->max_write_size)
			chunk_len = min(chunk_len, flash->spi->max_write_size);

		cmd_len = spi_flash_addr(flash, offset, cmd);

		debug("PP: 0x%p => cmd = { 0x%02x 0x%02x%02x%02x } chunk_len = %zu\n",
		      buf + actual, cmd[0], cmd[1], cmd[2], cmd[3], chunk_len);

		ret = spi_flash_write_common(flash, cmd, cmd_len,
					buf + actual, chunk_len);
		if (ret < 0) {
			debug("SF: write failed\n");
			break;
		}

		offset += chunk_len;
	}

	return ret;
}

int spi_flash_read_common(struct spi_flash *flash, const u8 *cmd,
		size_t cmd_len, void *data, size_t data_len)
{
	struct spi_slave *spi = flash->spi;
	int ret;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		debug("SF: unable to claim SPI bus\n");
		return ret;
	}

	ret = spi_flash_cmd_read(spi, cmd, cmd_len, data, data_len);
	if (ret < 0) {
		debug("SF: read cmd failed\n");
		return ret;
	}

	spi_release_bus(spi);

	return ret;
}

int spi_flash_cmd_read_ops(struct spi_flash *flash, u32 offset,
		size_t len, void *data)
{
	u8 cmd[6], bank_sel = 0;
	int cmd_len;
	u32 remain_len, read_len;
	int ret = -1;

	/* Handle memory-mapped SPI */
#ifndef CONFIG_SPI_FLASH_BAR
	if (flash->memory_map) {
		spi_xfer(flash->spi, 0, NULL, NULL, SPI_XFER_MMAP);
		memcpy(data, flash->memory_map + offset, len);
		spi_xfer(flash->spi, 0, NULL, NULL, SPI_XFER_MMAP_END);
		return 0;
	}
#endif /* !CONFIG_SPI_FLASH_BAR */

	cmd[0] = CMD_READ_ARRAY_FAST;
	cmd[4] = 0x00;
	cmd[5] = 0x00;

	while (len) {
#ifdef CONFIG_SPI_FLASH_BAR
		bank_sel = offset / SPI_FLASH_16MB_BOUN;

		ret = spi_flash_cmd_bankaddr_write(flash, bank_sel);
		if (ret) {
			debug("SF: fail to set bank%d\n", bank_sel);
			return ret;
		}
#endif
		remain_len = (SPI_FLASH_16MB_BOUN * (bank_sel + 1)) - offset;
		if (len < remain_len)
			read_len = len;
		else
			read_len = remain_len;

#ifdef CONFIG_SPI_FLASH_BAR
		if (flash->memory_map) {
			spi_xfer(flash->spi, 0, NULL, NULL, SPI_XFER_MMAP);
			memcpy(data,
			       flash->memory_map + offset % SPI_FLASH_16MB_BOUN,
			       read_len);
			spi_xfer(flash->spi, 0, NULL, NULL, SPI_XFER_MMAP_END);
			goto next;
		}
#endif /* CONFIG_SPI_FLASH_BAR */
		cmd_len = spi_flash_addr(flash, offset, cmd);

		ret = spi_flash_read_common(flash, cmd, cmd_len + 1,
							data, read_len);
		if (ret < 0) {
			debug("SF: read failed\n");
			break;
		}

#ifdef CONFIG_SPI_FLASH_BAR
next:
#endif /* CONFIG_SPI_FLASH_BAR */
		offset += read_len;
		len -= read_len;
		data += read_len;
	}

	return ret;
}

#ifdef CONFIG_SPI_FLASH_SST
static int sst_byte_write(struct spi_flash *flash, u32 offset, const void *buf)
{
	int ret;
	u8 cmd[4] = {
		CMD_SST_BP,
		offset >> 16,
		offset >> 8,
		offset,
	};

	debug("BP[%02x]: 0x%p => cmd = { 0x%02x 0x%06x }\n",
	      spi_w8r8(flash->spi, CMD_READ_STATUS), buf, cmd[0], offset);

	ret = spi_flash_cmd_write_enable(flash);
	if (ret)
		return ret;

	ret = spi_flash_cmd_write(flash->spi, cmd, sizeof(cmd), buf, 1);
	if (ret)
		return ret;

	return spi_flash_cmd_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
}

int sst_write_wp(struct spi_flash *flash, u32 offset, size_t len,
		const void *buf)
{
	size_t actual, cmd_len;
	int ret;
	u8 cmd[4];

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		debug("SF: Unable to claim SPI bus\n");
		return ret;
	}

	/* If the data is not word aligned, write out leading single byte */
	actual = offset % 2;
	if (actual) {
		ret = sst_byte_write(flash, offset, buf);
		if (ret)
			goto done;
	}
	offset += actual;

	ret = spi_flash_cmd_write_enable(flash);
	if (ret)
		goto done;

	cmd_len = 4;
	cmd[0] = CMD_SST_AAI_WP;
	cmd[1] = offset >> 16;
	cmd[2] = offset >> 8;
	cmd[3] = offset;

	for (; actual < len - 1; actual += 2) {
		debug("WP[%02x]: 0x%p => cmd = { 0x%02x 0x%06x }\n",
		      spi_w8r8(flash->spi, CMD_READ_STATUS), buf + actual,
		      cmd[0], offset);

		ret = spi_flash_cmd_write(flash->spi, cmd, cmd_len,
					buf + actual, 2);
		if (ret) {
			debug("SF: sst word program failed\n");
			break;
		}

		ret = spi_flash_cmd_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret)
			break;

		cmd_len = 1;
		offset += 2;
	}

	if (!ret)
		ret = spi_flash_cmd_write_disable(flash);

	/* If there is a single trailing byte, write it out */
	if (!ret && actual != len)
		ret = sst_byte_write(flash, offset, buf + actual);

 done:
	debug("SF: sst: program %s %zu bytes @ 0x%zx\n",
	      ret ? "failure" : "success", len, offset - actual);

	spi_release_bus(flash->spi);
	return ret;
}
#endif
