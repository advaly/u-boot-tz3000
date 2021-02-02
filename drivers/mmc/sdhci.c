/*
 * Copyright 2011, Marvell Semiconductor Inc.
 * Lei Wen <leiwen@marvell.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * Back ported to the 8xx platform (from the 8260 platform) by
 * Murray.Jensen@cmst.csiro.au, 27-Jan-01.
 */

#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <sdhci.h>
#include <linux/err.h>

void *aligned_buffer;

static void sdhci_reset(struct sdhci_host *host, u8 mask)
{
	unsigned long timeout;

	if (host->reset) {
		host->reset(host, mask);
		return;
	}
	/* Wait max 100 ms */
	timeout = 100;
	sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);
	while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			printf("Reset 0x%x never completed.\n", (int)mask);
			return;
		}
		timeout--;
		udelay(1000);
	}
}

static void sdhci_cmd_done(struct sdhci_host *host, struct mmc_cmd *cmd)
{
	int i;
	if (cmd->resp_type & MMC_RSP_136) {
		/* CRC is stripped so we need to do some shifting. */
		for (i = 0; i < 4; i++) {
			cmd->response[i] = sdhci_readl(host,
					SDHCI_RESPONSE + (3-i)*4) << 8;
			if (i != 3)
				cmd->response[i] |= sdhci_readb(host,
						SDHCI_RESPONSE + (3-i)*4-1);
		}
	} else {
		cmd->response[0] = sdhci_readl(host, SDHCI_RESPONSE);
	}
}

static void sdhci_transfer_pio(struct sdhci_host *host, struct mmc_data *data)
{
	int i;
	char *offs;
	for (i = 0; i < data->blocksize; i += 4) {
		offs = data->dest + i;
		if (data->flags == MMC_DATA_READ)
			*(u32 *)offs = sdhci_readl(host, SDHCI_BUFFER);
		else
			sdhci_writel(host, *(u32 *)offs, SDHCI_BUFFER);
	}
}

static int sdhci_transfer_data(struct sdhci_host *host, struct mmc_data *data,
				unsigned int start_addr)
{
	unsigned int stat, rdy, mask, timeout, block = 0;
#ifdef CONFIG_MMC_SDMA
	unsigned char ctrl;
	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	ctrl &= ~SDHCI_CTRL_DMA_MASK;
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
#endif

	timeout = 1000000;
	rdy = SDHCI_INT_SPACE_AVAIL | SDHCI_INT_DATA_AVAIL;
	mask = SDHCI_DATA_AVAILABLE | SDHCI_SPACE_AVAILABLE;
	do {
		stat = sdhci_readl(host, SDHCI_INT_STATUS);
#ifdef CONFIG_MMC_ADMA
		if (stat & SDHCI_INT_ADMA_ERROR) {
			printf("ADMA Error detected in status(0x%X)!\n", stat);
			printf("ADMA Error status 0x%x\n",
			       sdhci_readl(host, SDHCI_ADMA_ERROR));
			return -1;
		}
#endif
		if (stat & SDHCI_INT_ERROR) {
			printf("Error detected in status(0x%X)!\n", stat);
			return -1;
		}
		if (!(data->flags & MMC_DATA_READPAGES) &&
		    (stat & rdy)) {
			if (!(sdhci_readl(host, SDHCI_PRESENT_STATE) & mask))
				continue;
			sdhci_writel(host, rdy, SDHCI_INT_STATUS);
			sdhci_transfer_pio(host, data);
			data->dest += data->blocksize;
			if (++block >= data->blocks)
				break;
		}
#ifdef CONFIG_MMC_SDMA
		if (stat & SDHCI_INT_DMA_END) {
			sdhci_writel(host, SDHCI_INT_DMA_END, SDHCI_INT_STATUS);
			start_addr &= ~(SDHCI_DEFAULT_BOUNDARY_SIZE - 1);
			start_addr += SDHCI_DEFAULT_BOUNDARY_SIZE;
			sdhci_writel(host, start_addr, SDHCI_DMA_ADDRESS);
		}
#endif
		if (timeout-- > 0)
			udelay(10);
		else {
			printf("Transfer data timeout\n");
			return -1;
		}
	} while (!(stat & SDHCI_INT_DATA_END));
	return 0;
}

#ifdef CONFIG_MMC_ADMA
static void sdhci_set_adma_desc(u32 *desc, u32 addr, int len, unsigned cmd)
{
	__le32 *dataddr = (__le32 *)(desc + 1);
	__le16 *cmdlen = (__le16 *)desc;

	/* SDHCI specification says ADMA descriptors should be 4 byte
	 * aligned, so using 16 or 32bit operations should be safe. */

	cmdlen[0] = cpu_to_le16(cmd);
	cmdlen[1] = cpu_to_le16(len);

	dataddr[0] = cpu_to_le32(addr);
}

static void sdhci_adma_table_pre(struct sdhci_host *host,
				 unsigned int start_addr,
				 unsigned int trans_bytes)
{
	int i;
	unsigned int ofs;
	int ndesc = (trans_bytes + 65536 - 1) / 65536;

	/* start_addr is always 4 byte aligned */
	for (i = 0, ofs = 0; i < ndesc; i++, ofs += 65536) {
		unsigned int len = trans_bytes - ofs;
		if (len >= 65536)
			len = 0; /* 0 means 65536 bytes */
		sdhci_set_adma_desc(host->adma_desc[i],
				    start_addr + ofs, len, 0x21);
	}
	sdhci_set_adma_desc(host->adma_desc[i],
			    0, 0, 0x3); /* nop, end, valid */
	flush_cache((unsigned int)host->adma_desc,
		    sizeof(host->adma_desc[0]) * (ndesc + 1));
}

#ifndef PAGE_SIZE
#define PAGE_SIZE 4096
#endif
static int sdhci_adma_table_pre_pages(struct sdhci_host *host,
				      void **pages,
				      unsigned int trans_bytes)
{
	int i;
	unsigned int ofs;
	int ndesc = (trans_bytes + PAGE_SIZE - 1) / PAGE_SIZE;

	if (ndesc > SDHCI_ADMA_MAX_DESC) {
		printf("Too many pages for ADMA\n");
		return -1;
	}
	/* start_addr is always 4 byte aligned */
	for (i = 0, ofs = 0; i < ndesc; i++, ofs += PAGE_SIZE) {
		unsigned int len = trans_bytes - ofs;
		if (len > PAGE_SIZE)
			len = PAGE_SIZE;
		sdhci_set_adma_desc(host->adma_desc[i],
				    *pages, len, 0x21);
		pages++;
	}
	sdhci_set_adma_desc(host->adma_desc[i],
			    0, 0, 0x3); /* nop, end, valid */
	flush_cache((unsigned int)host->adma_desc,
		    sizeof(host->adma_desc[0]) * (ndesc + 1));
	return 0;
}
#endif

int sdhci_send_command(struct mmc *mmc, struct mmc_cmd *cmd,
		       struct mmc_data *data)
{
	struct sdhci_host *host = (struct sdhci_host *)mmc->priv;
	unsigned int stat = 0;
	int ret = 0;
	int trans_bytes = 0, is_aligned = 1;
	u32 mask, flags, mode;
	unsigned int timeout, start_addr = 0;
	unsigned int retry = 10000;
#ifdef CONFIG_MMC_ADMA
	unsigned char ctrl;
#endif

	/* Wait max 10 ms */
	timeout = 10;

	sdhci_writel(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);
	mask = SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT;

	/* We shouldn't wait for data inihibit for stop commands, even
	   though they might use busy signaling */
	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		mask &= ~SDHCI_DATA_INHIBIT;

	while (sdhci_readl(host, SDHCI_PRESENT_STATE) & mask) {
		if (timeout == 0) {
			printf("Controller never released inhibit bit(s).\n");
			return COMM_ERR;
		}
		timeout--;
		udelay(1000);
	}

	mask = SDHCI_INT_RESPONSE;
	if (!(cmd->resp_type & MMC_RSP_PRESENT))
		flags = SDHCI_CMD_RESP_NONE;
	else if (cmd->resp_type & MMC_RSP_136)
		flags = SDHCI_CMD_RESP_LONG;
	else if (cmd->resp_type & MMC_RSP_BUSY) {
		flags = SDHCI_CMD_RESP_SHORT_BUSY;
		mask |= SDHCI_INT_DATA_END;
	} else
		flags = SDHCI_CMD_RESP_SHORT;

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= SDHCI_CMD_CRC;
	if (cmd->resp_type & MMC_RSP_OPCODE)
		flags |= SDHCI_CMD_INDEX;
	if (data)
		flags |= SDHCI_CMD_DATA;

	/*Set Transfer mode regarding to data flag*/
	if (data != 0) {
		sdhci_writeb(host, 0xe, SDHCI_TIMEOUT_CONTROL);
		mode = SDHCI_TRNS_BLK_CNT_EN;
		trans_bytes = data->blocks * data->blocksize;
		if (data->blocks > 1)
			mode |= SDHCI_TRNS_MULTI;

		if (data->flags & MMC_DATA_READ)
			mode |= SDHCI_TRNS_READ;

#ifdef CONFIG_MMC_SDMA
		if (data->flags == MMC_DATA_READ)
			start_addr = (unsigned int)data->dest;
		else
			start_addr = (unsigned int)data->src;
		if ((host->quirks & SDHCI_QUIRK_32BIT_DMA_ADDR) &&
				(start_addr & 0x7) != 0x0) {
			is_aligned = 0;
			start_addr = (unsigned int)aligned_buffer;
			if (data->flags != MMC_DATA_READ)
				memcpy(aligned_buffer, data->src, trans_bytes);
		}

		sdhci_writel(host, start_addr, SDHCI_DMA_ADDRESS);
		mode |= SDHCI_TRNS_DMA;
#elif defined(CONFIG_MMC_ADMA)
		if (data->flags & MMC_DATA_READ)
			start_addr = (unsigned int)data->dest;
		else
			start_addr = (unsigned int)data->src;
		if (data->flags & MMC_DATA_READPAGES)
			sdhci_adma_table_pre_pages(host, (void **)start_addr,
						   trans_bytes);
		else
			sdhci_adma_table_pre(host, start_addr, trans_bytes);
		sdhci_writel(host, (unsigned int)host->adma_desc,
			     SDHCI_ADMA_ADDRESS);
		mode |= SDHCI_TRNS_DMA;
#endif
		sdhci_writew(host, SDHCI_MAKE_BLKSZ(SDHCI_DEFAULT_BOUNDARY_ARG,
				data->blocksize),
				SDHCI_BLOCK_SIZE);
		sdhci_writew(host, data->blocks, SDHCI_BLOCK_COUNT);
		sdhci_writew(host, mode, SDHCI_TRANSFER_MODE);
	}

	sdhci_writel(host, cmd->cmdarg, SDHCI_ARGUMENT);
#ifdef CONFIG_MMC_SDMA
	flush_cache(start_addr, trans_bytes);
#elif defined(CONFIG_MMC_ADMA)
	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	ctrl &= ~SDHCI_CTRL_DMA_MASK;
	if (data) {
		if (data->flags & MMC_DATA_READPAGES) {
			void **pages = (void **)start_addr;
			int ofs;
			for (ofs = 0; ofs < trans_bytes; ofs += PAGE_SIZE) {
				flush_cache((unsigned long)*pages,
					    min(trans_bytes - ofs, PAGE_SIZE));
				pages++;
			}
		} else {
			flush_cache(start_addr, trans_bytes);
		}
		ctrl |= SDHCI_CTRL_ADMA32;
	}
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
#endif
	sdhci_writew(host, SDHCI_MAKE_CMD(cmd->cmdidx, flags), SDHCI_COMMAND);
	do {
		stat = sdhci_readl(host, SDHCI_INT_STATUS);
		if (stat & SDHCI_INT_ERROR)
			break;
		if (--retry == 0)
			break;
		udelay(1);
	} while ((stat & mask) != mask);

	if (retry == 0) {
		if (host->quirks & SDHCI_QUIRK_BROKEN_R1B)
			return 0;
		else {
			printf("Timeout for status update!\n");
			return TIMEOUT;
		}
	}

	if ((stat & (SDHCI_INT_ERROR | mask)) == mask) {
		sdhci_cmd_done(host, cmd);
		sdhci_writel(host, mask, SDHCI_INT_STATUS);
	} else
		ret = -1;

	if (!ret && data)
		ret = sdhci_transfer_data(host, data, start_addr);

	if (host->quirks & SDHCI_QUIRK_WAIT_SEND_CMD)
		udelay(1000);

	stat = sdhci_readl(host, SDHCI_INT_STATUS);
	sdhci_writel(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);
	if (!ret) {
		if ((host->quirks & SDHCI_QUIRK_32BIT_DMA_ADDR) &&
				!is_aligned && (data->flags & MMC_DATA_READ))
			memcpy(data->dest, aligned_buffer, trans_bytes);
		return 0;
	}

	sdhci_reset(host, SDHCI_RESET_CMD);
	sdhci_reset(host, SDHCI_RESET_DATA);
	if (stat & SDHCI_INT_TIMEOUT)
		return TIMEOUT;
	else
		return COMM_ERR;
}

static int sdhci_set_clock(struct mmc *mmc, unsigned int clock)
{
	struct sdhci_host *host = (struct sdhci_host *)mmc->priv;
	unsigned int div, clk, timeout;

#ifdef CONFIG_TZ3000_SDHCI
	/* Do not stop clocks simultaneouosly */
	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	if (clk & SDHCI_CLOCK_CARD_EN)
		sdhci_writew(host, clk & ~SDHCI_CLOCK_CARD_EN,
			     SDHCI_CLOCK_CONTROL);
#endif
	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return 0;

	if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300) {
		/* Version 3.00 divisors must be a multiple of 2. */
		if (mmc->f_max <= clock)
			div = 1;
		else {
			for (div = 2; div < SDHCI_MAX_DIV_SPEC_300; div += 2) {
				if ((mmc->f_max / div) <= clock)
					break;
			}
		}
	} else {
		/* Version 2.00 divisors must be a power of 2. */
		for (div = 1; div < SDHCI_MAX_DIV_SPEC_200; div *= 2) {
			if ((mmc->f_max / div) <= clock)
				break;
		}
	}
	div >>= 1;

	if (host->set_clock)
		host->set_clock(host->index, div);

	clk = (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
	clk |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
		<< SDHCI_DIVIDER_HI_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			printf("Internal clock never stabilised.\n");
			return -1;
		}
		timeout--;
		udelay(1000);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
	return 0;
}

static void sdhci_set_power(struct sdhci_host *host, unsigned short power)
{
	u8 pwr = 0;

	if (power != (unsigned short)-1) {
		switch (1 << power) {
		case MMC_VDD_165_195:
			pwr = SDHCI_POWER_180;
			break;
		case MMC_VDD_29_30:
		case MMC_VDD_30_31:
			pwr = SDHCI_POWER_300;
			break;
		case MMC_VDD_32_33:
		case MMC_VDD_33_34:
			pwr = SDHCI_POWER_330;
			break;
		}
	}

	if (pwr == 0) {
		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);
		return;
	}

	if (host->quirks & SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER)
		sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);

	pwr |= SDHCI_POWER_ON;

	sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);
}

void sdhci_set_ios(struct mmc *mmc)
{
	u32 ctrl;
	struct sdhci_host *host = (struct sdhci_host *)mmc->priv;

	if (host->set_control_reg)
		host->set_control_reg(host);

	if (host->enable_tuning)
		host->enable_tuning(host, 0);

	if (mmc->clock != host->clock)
		sdhci_set_clock(mmc, mmc->clock);

	/* Set bus width */
	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	if (mmc->bus_width == 8) {
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		if ((SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300) ||
				(host->quirks & SDHCI_QUIRK_USE_WIDE8))
			ctrl |= SDHCI_CTRL_8BITBUS;
	} else {
		if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300)
			ctrl &= ~SDHCI_CTRL_8BITBUS;
		if (mmc->bus_width == 4)
			ctrl |= SDHCI_CTRL_4BITBUS;
		else
			ctrl &= ~SDHCI_CTRL_4BITBUS;
	}

	if (mmc->clock > 26000000)
		ctrl |= SDHCI_CTRL_HISPD;
	else
		ctrl &= ~SDHCI_CTRL_HISPD;

	if (host->quirks & SDHCI_QUIRK_NO_HISPD_BIT)
		ctrl &= ~SDHCI_CTRL_HISPD;

	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	if (host->enable_tuning)
		host->enable_tuning(host, 1);
}

int sdhci_init(struct mmc *mmc)
{
	struct sdhci_host *host = (struct sdhci_host *)mmc->priv;
#ifdef CONFIG_TZ3000_SDHCI
	unsigned int timeout = 1100;

	while (!(sdhci_readl(host, SDHCI_PRESENT_STATE) & SDHCI_CARD_PRESENT)) {
		if (timeout == 0)
			return -ETIMEDOUT;
		timeout--;
		udelay(1000);
	}
#endif
	if ((host->quirks & SDHCI_QUIRK_32BIT_DMA_ADDR) && !aligned_buffer) {
		aligned_buffer = memalign(8, 512*1024);
		if (!aligned_buffer) {
			printf("Aligned buffer alloc failed!!!");
			return -1;
		}
	}

	sdhci_set_power(host, fls(mmc->voltages) - 1);

	if (host->quirks & SDHCI_QUIRK_NO_CD) {
		unsigned int status;

		sdhci_writel(host, SDHCI_CTRL_CD_TEST_INS | SDHCI_CTRL_CD_TEST,
			SDHCI_HOST_CONTROL);

		status = sdhci_readl(host, SDHCI_PRESENT_STATE);
		while ((!(status & SDHCI_CARD_PRESENT)) ||
		    (!(status & SDHCI_CARD_STATE_STABLE)) ||
		    (!(status & SDHCI_CARD_DETECT_PIN_LEVEL)))
			status = sdhci_readl(host, SDHCI_PRESENT_STATE);
	}

	/* Enable only interrupts served by the SD controller */
	sdhci_writel(host, SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK
		     , SDHCI_INT_ENABLE);
	/* Mask all sdhci interrupt sources */
	sdhci_writel(host, 0x0, SDHCI_SIGNAL_ENABLE);

	return 0;
}

int add_sdhci(struct sdhci_host *host, u32 max_clk, u32 min_clk)
{
	struct mmc *mmc;
	unsigned int caps;

	mmc = malloc(sizeof(struct mmc));
	if (!mmc) {
		printf("mmc malloc fail!\n");
		return -1;
	}

	mmc->priv = host;
	host->mmc = mmc;

	sprintf(mmc->name, "%s", host->name);
	mmc->send_cmd = sdhci_send_command;
	mmc->set_ios = sdhci_set_ios;
	mmc->init = sdhci_init;
	mmc->getcd = NULL;
	mmc->getwp = NULL;

	caps = sdhci_readl(host, SDHCI_CAPABILITIES);
#ifdef CONFIG_MMC_SDMA
	if (!(caps & SDHCI_CAN_DO_SDMA)) {
		printf("Your controller don't support sdma!!\n");
		return -1;
	}
#elif defined(CONFIG_MMC_ADMA)
	if (!(caps & SDHCI_CAN_DO_ADMA2)) {
		printf("Your controller don't support adma!!\n");
		return -1;
	}
#endif

	if (max_clk)
		mmc->f_max = max_clk;
	else {
		if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300)
			mmc->f_max = (caps & SDHCI_CLOCK_V3_BASE_MASK)
				>> SDHCI_CLOCK_BASE_SHIFT;
		else
			mmc->f_max = (caps & SDHCI_CLOCK_BASE_MASK)
				>> SDHCI_CLOCK_BASE_SHIFT;
		mmc->f_max *= 1000000;
	}
	if (mmc->f_max == 0) {
		printf("Hardware doesn't specify base clock frequency\n");
		return -1;
	}
	if (min_clk)
		mmc->f_min = min_clk;
	else {
		if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300)
			mmc->f_min = mmc->f_max / SDHCI_MAX_DIV_SPEC_300;
		else
			mmc->f_min = mmc->f_max / SDHCI_MAX_DIV_SPEC_200;
	}

	mmc->voltages = 0;
	if (caps & SDHCI_CAN_VDD_330)
		mmc->voltages |= MMC_VDD_32_33 | MMC_VDD_33_34;
	if (caps & SDHCI_CAN_VDD_300)
		mmc->voltages |= MMC_VDD_29_30 | MMC_VDD_30_31;
	if (caps & SDHCI_CAN_VDD_180)
		mmc->voltages |= MMC_VDD_165_195;

	if (host->quirks & SDHCI_QUIRK_BROKEN_VOLTAGE)
		mmc->voltages |= host->voltages;

	mmc->host_caps = MMC_MODE_HS | MMC_MODE_HS_52MHz | MMC_MODE_4BIT;
	if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300) {
		if (caps & SDHCI_CAN_DO_8BIT)
			mmc->host_caps |= MMC_MODE_8BIT;
	}
	if (host->host_caps)
		mmc->host_caps |= host->host_caps;
#if defined(CONFIG_MMC_ADMA)
	mmc->host_caps |= MMC_MODE_READPAGES;
#endif

	sdhci_reset(host, SDHCI_RESET_ALL);
	mmc_register(mmc);

	return 0;
}
