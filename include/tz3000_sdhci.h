/*
 * Toshiba TZ3000 SPI definitions
 *
 * Copyright (C) 2013,2014 Toshiba Corporation
 */
#ifndef _TZ3000_SDHCI_H_
#define _TZ3000_SDHCI_H_

enum sdhci_tz3000_ec_tune_mode {
	EC_SD_DS,
	EC_SD_HS,
	EC_MMC_HS,
	EC_MMC_MAX,
};

struct sdhci_tz3000_ec_tune_param {
	int otap; /* -1 for OTAPDLYEN=0 */
	int itap; /* -1 for ITAPDLYEN=0 */
	unsigned int usedll:1;
};

struct sdhci_tz3000_platform_data {
	unsigned long econf_emmc_base;
	struct sdhci_tz3000_ec_tune_param ec_tune_params[EC_MMC_MAX];
	unsigned int f_max;
};

int tz3000_sdhci_init(char *name, unsigned long base_addr,
		      struct sdhci_tz3000_platform_data *platdata);

#endif /* _TZ3000_SDHCI_H_ */
