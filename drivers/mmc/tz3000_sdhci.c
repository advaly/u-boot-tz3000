/*
 * Toshiba TZ3000 EMMC controller driver
 *
 * Copyright (C) 2013,2014 Toshiba Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <sdhci.h>
#include <tz3000_sdhci.h>

#define EMMC_CTRL_DLYCHG	0x50
#define EMMC_CTRL_INDLY		0x58
#define EMMC_CTRL_OUTDLY	0x60
#define EMMC_CTRL_DLL0		0x68
#define EMMC_CTRL_DLL1		0x70

struct tz3000_sdhci_host {
	struct sdhci_host host;
	struct sdhci_tz3000_platform_data *platdata;
	u16 transfer_mode;
	int tuning_enabled;
};

static void econf_emmc_updatew(struct sdhci_host *host, u16 val, int reg)
{
	struct tz3000_sdhci_host *tz3000_host =
		(struct tz3000_sdhci_host *)host;
	void *econf_emmc_addr = (void *)tz3000_host->platdata->econf_emmc_base;
	unsigned long timeout = 1000;

	writel(val, econf_emmc_addr + reg);
	while (readl(econf_emmc_addr + reg) !=
	       ((val << 16) | val)) {
		if (timeout == 0) {
			printf("%s reg %x val %x timeout.\n",
			       __func__, reg, val);
			return;
		}
		timeout--;
		udelay(1);
	}
}

/* use 32bit access for SDHC registers */
static u8 sdhci_tz3000_readb(struct sdhci_host *host, int reg)
{
	return readl(host->ioaddr + (reg & ~3)) >> (8 * (reg & 3));
}

static u16 sdhci_tz3000_readw(struct sdhci_host *host, int reg)
{
	return readl(host->ioaddr + (reg & ~3)) >> (8 * (reg & 3));
}

static void sdhci_tz3000_writeb(struct sdhci_host *host, u8 val, int reg)
{
	int reg32 = reg & ~3;
	u32 val32 = readl(host->ioaddr + reg32);
	unsigned int bitofs = 8 * (reg & 3);
	u32 mask = 0xff << bitofs;
	writel((val << bitofs) | (val32 & ~mask), host->ioaddr + reg32);
}

static void sdhci_tz3000_writew(struct sdhci_host *host, u16 val, int reg)
{
	struct tz3000_sdhci_host *tz3000_host =
		(struct tz3000_sdhci_host *)host;
	int reg32 = reg & ~3;
	u32 val32 = readl(host->ioaddr + reg32);
	unsigned int bitofs = 8 * (reg & 3);
	u32 mask = 0xffff << bitofs;

	switch (reg) {
	case SDHCI_TRANSFER_MODE:
		tz3000_host->transfer_mode = val;
		return;
	case SDHCI_COMMAND:
		val32 = tz3000_host->transfer_mode;
		break;
	}
	writel((val << bitofs) | (val32 & ~mask), host->ioaddr + reg32);
	if (reg == SDHCI_CLOCK_CONTROL) {
		/* errata 9.11, 9.12 workaround */
		readl(host->ioaddr + reg32); /* dummy read */
		udelay(10 * 20); /* at least 20 cycle (at minimum 100KHz) */
	}
}

static struct sdhci_ops sdhci_tz3000_ops = {
	.read_w = sdhci_tz3000_readw,
	.read_b = sdhci_tz3000_readb,
	.write_w = sdhci_tz3000_writew,
	.write_b = sdhci_tz3000_writeb,
};

static void sdhci_tz3000_enable_tuning(struct sdhci_host *host, int enable);

static void sdhci_tz3000_set_control_reg(struct sdhci_host *host)
{
	struct tz3000_sdhci_host *tz3000_host =
		(struct tz3000_sdhci_host *)host;
	/* limit host->mmc->clock */
	if (tz3000_host->platdata->f_max &&
	    tz3000_host->platdata->f_max < host->mmc->clock)
		host->mmc->clock = tz3000_host->platdata->f_max;
}

static void sdhci_tz3000_reset(struct sdhci_host *host, u8 mask)
{
	struct tz3000_sdhci_host *tz3000_host =
		(struct tz3000_sdhci_host *)host;
	unsigned long timeout;

	/* errata 9.3: software reset limitation */
	unsigned short clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	unsigned short orgclk = clk;
	int enabled = tz3000_host->tuning_enabled;

	if (mask == SDHCI_RESET_CMD)
		return; /* assume SDHCI_RESET_DATA follows */
	if (mask == SDHCI_RESET_DATA)
		mask = SDHCI_RESET_CMD | SDHCI_RESET_DATA;

	if (host->mmc->clock != 0 && host->mmc->clock < 25000000) {
		if (clk & SDHCI_CLOCK_CARD_EN) {
			clk &= ~SDHCI_CLOCK_CARD_EN;
			sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
		}
		if (clk & SDHCI_CLOCK_INT_EN) {
			clk &= ~SDHCI_CLOCK_INT_EN;
			sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
		}
		if (enabled &&
		    (clk & SDHCI_CLOCK_CARD_EN) && (clk & SDHCI_CLOCK_INT_EN))
			sdhci_tz3000_enable_tuning(host, 0);
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

	if (host->mmc->clock != 0 && host->mmc->clock < 25000000) {
		if (orgclk & SDHCI_CLOCK_INT_EN) {
			clk |= SDHCI_CLOCK_INT_EN;
			sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
			/* Wait max 20 ms */
			timeout = 20;
			while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
				 & SDHCI_CLOCK_INT_STABLE)) {
				if (timeout == 0) {
					printf("Internal clock never stabilised.\n");
					return;
				}
				timeout--;
				udelay(1000);
			}
		}
		if (orgclk & SDHCI_CLOCK_CARD_EN) {
			clk |= SDHCI_CLOCK_CARD_EN;
			sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
		}
		if (enabled &&
		    (clk & SDHCI_CLOCK_CARD_EN) && (clk & SDHCI_CLOCK_INT_EN))
			sdhci_tz3000_enable_tuning(host, 1);
	}
}

static void sdhci_tz3000_disable_manual_tuning(struct sdhci_host *host)
{
	/* Set TAPCHGWIN = 1 */
	econf_emmc_updatew(host, 1, EMMC_CTRL_DLYCHG);
	/* Set DLLENABLE = 0 */
	econf_emmc_updatew(host, 0, EMMC_CTRL_DLL0);
	/* Update ITAPDLY */
	econf_emmc_updatew(host, 0, EMMC_CTRL_INDLY);
	/* Update OTAPDLY */
	econf_emmc_updatew(host, 0, EMMC_CTRL_OUTDLY);
	/* Set TAPCHGWIN = 0 */
	econf_emmc_updatew(host, 0, EMMC_CTRL_DLYCHG);
}

static void sdhci_tz3000_manual_tuning(struct sdhci_host *host,
				       int otap, int itap, int usedll)
{
	u32 itapval = itap < 0 ? 0 : ((itap << 8) | 1);
	u32 otapval = otap < 0 ? 0 : ((otap << 8) | 1);

	/* Set TAPCHGWIN = 1 */
	econf_emmc_updatew(host, 1, EMMC_CTRL_DLYCHG);
	/* Set DLLENABLE = 0 */
	econf_emmc_updatew(host, 0, EMMC_CTRL_DLL0);
	/* Update DLLCLKSEL */
	econf_emmc_updatew(host, 0, EMMC_CTRL_DLL1);
	/* Set DLLENABLE = 1 */
	if (usedll)
		econf_emmc_updatew(host, 1, EMMC_CTRL_DLL0);
	/* Update ITAPDLY */
	econf_emmc_updatew(host, itapval, EMMC_CTRL_INDLY);
	/* Update OTAPDLY */
	econf_emmc_updatew(host, otapval, EMMC_CTRL_OUTDLY);
	/* Set TAPCHGWIN = 0 */
	econf_emmc_updatew(host, 0, EMMC_CTRL_DLYCHG);
}

static void sdhci_tz3000_enable_tuning(struct sdhci_host *host, int enable)
{
	struct tz3000_sdhci_host *tz3000_host =
		(struct tz3000_sdhci_host *)host;
	u16 clk;
	enum sdhci_tz3000_ec_tune_mode mode;
	const struct sdhci_tz3000_ec_tune_param *param;
	struct mmc *mmc = host->mmc;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	if (!(clk & SDHCI_CLOCK_CARD_EN) || !(clk & SDHCI_CLOCK_INT_STABLE))
		return;

	if (!enable) {
		if (tz3000_host->tuning_enabled)
			sdhci_tz3000_disable_manual_tuning(host);
		tz3000_host->tuning_enabled = 0;
		return;
	}
	tz3000_host->tuning_enabled = 1;

	if (IS_SD(mmc)) {
		if (mmc->card_caps & MMC_MODE_HS)
			mode = EC_SD_HS;
		else
			mode = EC_SD_DS;
	} else {
		if (mmc->card_caps & MMC_MODE_HS)
			mode = EC_MMC_HS;
		else
			mode = EC_SD_DS;
	}

	param = &tz3000_host->platdata->ec_tune_params[mode];
	sdhci_tz3000_manual_tuning(host, param->otap, param->itap,
				   param->usedll);
}

int tz3000_sdhci_init(char *name, unsigned long base_addr,
		      struct sdhci_tz3000_platform_data *platdata)
{
	struct tz3000_sdhci_host *tz3000_host = malloc(sizeof(*tz3000_host));
	struct sdhci_host *host;
	if (!tz3000_host) {
		printf("sdhci_host malloc fail!\n");
		return 1;
	}
	memset(tz3000_host, 0, sizeof(*tz3000_host));
	tz3000_host->platdata = platdata;
	host = &tz3000_host->host;
	host->name = name;
	host->ioaddr = (void *)base_addr;
	host->quirks = SDHCI_QUIRK_WAIT_SEND_CMD;
	host->ops = &sdhci_tz3000_ops;
	host->set_control_reg = sdhci_tz3000_set_control_reg;
	host->reset = sdhci_tz3000_reset;
	host->enable_tuning = sdhci_tz3000_enable_tuning;
	host->version = sdhci_readw(host, SDHCI_HOST_VERSION);
	host->host_caps = MMC_MODE_HC;
	return add_sdhci(host, 0, 0);
}
