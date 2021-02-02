/*
 * Toshiba TZ2000 Board specific functions
 *
 * Copyright (C) 2010,2013,2014 Toshiba Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <environment.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/sizes.h>
#include <ns16550.h>
#include <tz3000_spi.h>

DECLARE_GLOBAL_DATA_PTR;

int tz2000_periph_clk = TZ2000_PERIPH_CLK;
int tz2000_board_id    = -1;
int tz2000_debug_board = -1;
int tz2000_usb_hd      = -1;

static struct {
	u32 mach_id;
	const char *board_name;
} tz2000_boards[] = {
	{ MACH_TYPE_TZ2000EVA, "TZ2000 EVA" },
	{ MACH_TYPE_TZ2000RB2, "RBTZ2000-TOUBKAL" },
	{ MACH_TYPE_TZ2000RB, "RBTZ2000-GREGORY" },
	{ MACH_TYPE_TZ2000RB3, "RBTZ2000-MONTBLANC" },
};

#ifdef CONFIG_EARLY_DEBUG
static void uart_init(unsigned long uart_base)
{
	int divisor = (CONFIG_SYS_NS16550_CLK + CONFIG_BAUDRATE * (16 / 2)) /
		(16 * CONFIG_BAUDRATE);
	NS16550_init((NS16550_t)uart_base, divisor);
}

static void uart_output_chr(unsigned long uart_base, char c)
{
	while ((readl(uart_base + 0x14) & UART_LSR_THRE) == 0)
		;
	writel(c & 0xff, uart_base + 0x00); /* thr */
	while ((readl(uart_base + 0x14) & (UART_LSR_TEMT | UART_LSR_THRE)) !=
	       (UART_LSR_TEMT | UART_LSR_THRE))
		;
}

static void uart_output_str(unsigned long uart_base, const char *s)
{
	while (*s)
		uart_output_chr(uart_base, *s++);
}
#endif

static void early_init_peripherals(void)
{
	u32 val;
	static struct init_values {
		u32 pinshare0;
		u32 pinshare1;
		u32 gp_cr0;
		u32 gp_smp0;
		u32 gp_odr0;
		u32 gp_odtp0;
		u32 gp_puden0;
		u32 gp_pudsel0;
		u32 gp_inpen0;
		u32 gp_cr1;
		u32 gp_smp1;
		u32 gp_odr1;
		u32 gp_odtp1;
		u32 gp_puden1;
		u32 gp_pudsel1;
		u32 gp_inpen1;
	} init_values[] = {
		{
			/* init_values_eva GPIO[32:33]=00 */
			0x0000f008, /* PINSHARE0 */
			0x00000001, /* PINSHARE1 */
			0x0000000e, /* GP_CR0 */
			0x00000000, /* GP_SMP0 */
			0x0000000e, /* GP_ODR0 */
			0x00000000, /* GP_ODTP0 */
			0x0000000e, /* GP_PUDEN0 */
			0x00000000, /* GP_PUDSEL0 */
			0x00000000, /* GP_INPEN0 */
			0x00005030, /* GP_CR1 */
			0x00000000, /* GP_SMP1 */
			0x00005000, /* GP_ODR1 */
			0x00000000, /* GP_ODTP1 */
			0x000070f0, /* GP_PUDEN1 */
			0x00000000, /* GP_PUDSEL1 */
			0x0000a0cf, /* GP_INPEN1 */
		}, {
			/* init_values_rb2 GPIO[32:33]=10 */
			0x0000300c, /* PINSHARE0 */
			0x00000000, /* PINSHARE1 */
			0x0000000f, /* GP_CR0 */
			0x00000000, /* GP_SMP0 */
			0x0000000e, /* GP_ODR0 */
			0x00000000, /* GP_ODTP0 */
			0x0000000f, /* GP_PUDEN0 */
			0x00000000, /* GP_PUDSEL0 */
			0x00000000, /* GP_INPEN0 */
			0x0000e308, /* GP_CR1 */
			0x00000000, /* GP_SMP1 */
			0x00006000, /* GP_ODR1 */
			0x00000000, /* GP_ODTP1 */
			0x0000ff08, /* GP_PUDEN1 */
			0x00000800, /* GP_PUDSEL1 */
			0x00001c07, /* GP_INPEN1 */
		}, {
			/* init_values_rb GPIO[32:33]=01 */
			0x0000300c, /* PINSHARE0 */
			0x00000000, /* PINSHARE1 */
			0x0000000f, /* GP_CR0 */
			0x00000000, /* GP_SMP0 */
			0x0000000e, /* GP_ODR0 */
			0x00000000, /* GP_ODTP0 */
			0x0000000f, /* GP_PUDEN0 */
			0x00000000, /* GP_PUDSEL0 */
			0x00000000, /* GP_INPEN0 */
			0x0000e308, /* GP_CR1 */
			0x00000000, /* GP_SMP1 */
			0x00006000, /* GP_ODR1 */
			0x00000000, /* GP_ODTP1 */
			0x0000fb08, /* GP_PUDEN1 */
			0x00000800, /* GP_PUDSEL1 */
			0x00001807, /* GP_INPEN1 */
		}, {
		/* init_values_rb3 GPIO[32:33]=11 */
			0x0000300c, /* PINSHARE0 */
			0x00000000, /* PINSHARE1 */
			0x00000003, /* GP_CR0 */
			0x00000000, /* GP_SMP0 */
			0x00000002, /* GP_ODR0 */
			0x00000000, /* GP_ODTP0 */
			0x00000003, /* GP_PUDEN0 */
			0x00000000, /* GP_PUDSEL0 */
			0x00000000, /* GP_INPEN0 */
			0x00008308, /* GP_CR1 */
			0x00000000, /* GP_SMP1 */
			0x00000000, /* GP_ODR1 */
			0x00000000, /* GP_ODTP1 */
			0x00008b08, /* GP_PUDEN1 */
			0x00000800, /* GP_PUDSEL1 */
			0x00000807, /* GP_INPEN1 */
		}
	};
	struct init_values *initval;
	int emmc8sd_nand;
	unsigned long uart_base;
	unsigned long sro;

	/* call here to use __udelay. */
	timer_init();

	writel(0x01110000, TZ2000_PMU_CGOFF_PERI1); /* i2c,gpio */
	writel(0x00000011, TZ2000_PMU_CGOFF_PERI3); /* spib */
	writel(0x00111101, TZ2000_PMU_CGOFF_PERI4); /* uart */
	/* gpio must be available first */
	writel(0x01110000, TZ2000_PMU_SROFF_PERI1); /* i2c,gpio */
	/* UART2 isolation must be off for reading GPIO 32,33 */
	val = readl(TZ2000_PMU_POREN); /* Isolation UART2,SPIM */
	writel(val | 0x00000400, TZ2000_PMU_POREN); /* UART2,SPIM */
	__udelay(1); /*  >100ns delay */

	/* gconf_early_init */
	val = readl(TZ2000_GCONF_PINSHARE0);
	writel(val | 0x00002000, TZ2000_GCONF_PINSHARE0); /* GPIO_UART2_DSEL */
	/* gpio_early_init */
	val = readl(TZ2000_GP_INPEN(1));
	writel(val | 0x00000003, TZ2000_GP_INPEN(1)); /* GPIO[32:33] */
	/* get_board_id */
	tz2000_board_id = readl(TZ2000_GP_DR(1)) & 0x03;

	initval = &init_values[tz2000_board_id];
	writel(initval[0].pinshare0, TZ2000_GCONF_PINSHARE0);
	writel(initval[0].pinshare1, TZ2000_GCONF_PINSHARE1);
	writel(initval[0].gp_cr0, TZ2000_GP_CR(0));
	writel(initval[0].gp_odtp0, TZ2000_GP_ODTP(0));
	writel(initval[0].gp_puden0, TZ2000_GP_PUDEN(0));
	writel(initval[0].gp_pudsel0, TZ2000_GP_PUDSEL(0));
	writel(initval[0].gp_inpen0, TZ2000_GP_INPEN(0));
	writel(initval[0].gp_smp0, TZ2000_GP_SMP(0));
	writel(initval[0].gp_odr0, TZ2000_GP_ODR(0));
	writel(initval[0].gp_cr1, TZ2000_GP_CR(1));
	writel(initval[0].gp_odtp1, TZ2000_GP_ODTP(1));
	writel(initval[0].gp_puden1, TZ2000_GP_PUDEN(1));
	writel(initval[0].gp_pudsel1, TZ2000_GP_PUDSEL(1));
	writel(initval[0].gp_inpen1, TZ2000_GP_INPEN(1));
	writel(initval[0].gp_smp1, TZ2000_GP_SMP(1));
	writel(initval[0].gp_odr1, TZ2000_GP_ODR(1));

	/* get_usb_host_ordevice */
	tz2000_debug_board = (readl(TZ2000_GP_DR(1)) >> 2) & 0x1;
	if (tz2000_board_id != 0x00)
		tz2000_usb_hd = 1; /* host */
	else
		tz2000_usb_hd = (readl(TZ2000_GP_DR(1)) >> 15) & 0x1;
	writel(0x00000011, TZ2000_PMU_SROFF_PERI3); /* spib */
	writel(0x00111101, TZ2000_PMU_SROFF_PERI4); /* uart */
#ifdef CONFIG_TZ3000_EMAC
	writel(0x11, TZ2000_PMU_SRON_ETHER);
	writel(0x01, TZ2000_PMU_CGOFF_ETHER);
#endif
	/* mmc4sd_reset */
	writel(0x01, TZ2000_PMU_SRON_EMMC);
	writel(0x03, TZ2000_PMU_CGOFF_EMMC);
	/* set emmc4sd SLOTTYPE to 'removable' */
	val = readl(TZ2000_WCONF_EMMC_BASE(0));
	writel(val & 0x3fffffff, TZ2000_WCONF_EMMC_BASE(0));
	/* CFG_CAPABILITY0:8BITSUPPORT */
	emmc8sd_nand = (readl(TZ2000_WCONF_EMMC_BASE(1)) & 0x00040000) != 0;
	/* mmc8sd_reset */
	if (emmc8sd_nand &&
	    readl(TZ2000_BOOTINFO_MAINMEM) == TZ2000_MAINMEM_DRAM) {
		/* Reset Telomere & emmc8sd */
		writel(0x030303, TZ2000_PMU_SRON_TELOMERE);
		writel(0x030111, TZ2000_PMU_CGOFF_TELOMERE);
	} else {
		/* Reset emmc8sd Only */
		writel(0x010000, TZ2000_PMU_SRON_TELOMERE);
		writel(0x030000, TZ2000_PMU_CGOFF_TELOMERE);
	}
	/* clear emmc8sd CAPABIRITY0.1P8VOLTSUPPORT if RB3 */
	if (tz2000_board_id == 0x03) {
		val = readl(TZ2000_WCONF_EMMC_BASE(1));
		writel(val & 0xfbffffff, TZ2000_WCONF_EMMC_BASE(1));
	}
	/* set emmc8sd SLOTTYPE to 'removable' if not (internal-nand or EVA) */
	if (!emmc8sd_nand && tz2000_board_id != 0x00) {
		val = readl(TZ2000_WCONF_EMMC_BASE(1));
		writel(val & 0x3fffffff, TZ2000_WCONF_EMMC_BASE(1));
	}
	writel(0x5511, TZ2000_PMU_SRON_USB2HD);
	if (tz2000_usb_hd)
		writel(0x0111, TZ2000_PMU_CGOFF_USB2HD);
	else
		writel(0x1001, TZ2000_PMU_CGOFF_USB2HD);
	writel(0x68082c08, TZ2000_WCONF_USB2PHY_CONFIG0);
	writel(0x01, TZ2000_WCONF_USB2PHY_CONFIG1);
	writel(0x51, TZ2000_WCONF_USB2HD_REFCLK);
	if (tz2000_usb_hd)
		writel(0x02, TZ2000_WCONF_USB2HD_MUX_CTL);
	else
		writel(0x03, TZ2000_WCONF_USB2HD_MUX_CTL);

	/* isolation_off */
	val = readl(TZ2000_PMU_POREN); /* CFG_CAPABILITY0:8BITSUPPORT */
	if (emmc8sd_nand)
		val |= 0x00000010; /* NAND Isolation Off */
	else
		val |= 0x00000100; /* emmc8sd Isolatopn Off */
	val |= 0x000000e0; /* USB emmc4sd, ADC Isolation Off */
	writel(val, TZ2000_PMU_POREN);
	__udelay(1); /*  >100ns delay */
#ifdef CONFIG_TZ3000_EMAC
	writel(0x11, TZ2000_PMU_SROFF_ETHER);
#endif
	writel(0x01, TZ2000_PMU_SROFF_EMMC);
	if (emmc8sd_nand &&
	    readl(TZ2000_BOOTINFO_MAINMEM) == TZ2000_MAINMEM_DRAM)
		writel(0x030303, TZ2000_PMU_SROFF_TELOMERE);
	else
		writel(0x010000, TZ2000_PMU_SROFF_TELOMERE);
	sro = TZ2000_PMU_USB_CONTROL; /* VDD33_USB Enable */
	__udelay(1);
	writel(0x00000000, sro);
	__udelay(1);
	writel(0x00000001, sro);        /* Set VDD33_USB weak */
	__udelay(100);
	writel(0x00000003, sro);        /* Set VDD33_USB strong */
	__udelay(1000);
	sro = TZ2000_PMU_SROFF_USB2HD;
	writel(0x00000001, sro);
	__udelay(225);
	if (tz2000_usb_hd) {
		writel(0x00000401, sro);
		__udelay(1);
		writel(0x00000511, sro);
	} else {
		writel(0x00004001, sro);
		__udelay(1);
		writel(0x00005001, sro);
	}

	/* if LCR was initialized, wait for txempty */
	uart_base = TZ2000_UART_BASE(CONFIG_CONS_INDEX-1);
	if (readl(uart_base + 0x0c) != 0) { /* lcr */
		do {
			do {
				__udelay(100);
				/* check lsr */
				val = readl(uart_base + 0x14);
			} while ((val & (UART_LSR_TEMT | UART_LSR_THRE)) !=
				 (UART_LSR_TEMT | UART_LSR_THRE));
			__udelay(100);
			/* check lsr again */
			val = readl(uart_base + 0x14);
		} while ((val & (UART_LSR_TEMT | UART_LSR_THRE)) !=
			 (UART_LSR_TEMT | UART_LSR_THRE));
	}
#ifdef CONFIG_EARLY_DEBUG
	uart_init(uart_base);
	uart_output_str(uart_base, "\r\n*** Booting U-Boot...\r\n");
#endif

#ifdef CONFIG_TZ3000_SPI
	writel(0x0, TZ2000_PMU_SPI_CONTROL);
#ifdef INIT_SPIC_MEMMAP_0
	writel(INIT_SPIC_MEMMAP_0, TZ3000_SPIC_MEMMAP(0));
	writel(INIT_SPIC_DIRECT_0, TZ3000_SPIC_DIRECT(0));
#endif

#ifdef INIT_SPIC_MEMMAP_1
	writel(INIT_SPIC_MEMMAP_1, TZ3000_SPIC_MEMMAP(1));
	writel(INIT_SPIC_DIRECT_1, TZ3000_SPIC_DIRECT(1));
#endif
#endif
}

static void init_env_ops(void)
{
#ifdef CONFIG_ENV_IS_SELECTABLE
	int bootdev = readl(TZ2000_BOOTINFO_BOOTDEVICE) & 7;
	struct env_ops *envops[8] = {
#ifdef CONFIG_ENV_IS_IN_FLASH
		[0] = &flash_env_ops,
		[1] = &flash_env_ops,
#endif
#ifdef CONFIG_ENV_IS_IN_SPI_FLASH
		[0] = &spi_flash_env_ops,
		[1] = &spi_flash_env_ops,
#endif
#ifdef CONFIG_ENV_IS_IN_MMC
		[2] = &mmc_env_ops,
		[5] = &mmc_env_ops,
		[6] = &mmc_env_ops,
		[7] = &mmc_env_ops,
#endif
	};
	if (envops[bootdev])
		env_ops = envops[bootdev];
#endif /* CONFIG_ENV_IS_SELECTABLE */
}

int board_early_init_f(void)
{
	early_init_peripherals();
	init_env_ops();
	return 0;
}

int board_init(void)
{
	gd->bd->bi_arch_number = tz2000_boards[tz2000_board_id].mach_id;
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
	/* to protect whole sysbin area */
	monitor_flash_len = CONFIG_SYS_MONITOR_LEN;
	return 0;
}

int dram_init(void)
{
	gd->ram_size = readl(TZ2000_BOOTINFO_MEMSIZE);
	return 0;
}

#ifdef CONFIG_DISPLAY_BOARDINFO
int checkboard(void)
{
	int bootdev = readl(TZ2000_BOOTINFO_BOOTDEVICE) & 7;
	const char *bootdev_name[] = {
		"SPI CH0 (XIP)",
		"SPI CH0",
		"Internal NAND",
		"Reserved",
		"Reserved",
		"SD Card CH0",
		"eMMC CH1",
		"SD Card CH1",
	};
	puts("Board: ");
	puts(tz2000_boards[tz2000_board_id].board_name);
	puts("\n");
	printf("BootDev: %s\n", bootdev_name[bootdev]);

	return 0;
}
#endif

#ifdef CONFIG_DISPLAY_BOOTTIME
void boottime(void)
{
	u32 upper, lower, tmp;
	u32 msec, msec_count = tz2000_periph_clk / 1000;

	upper = readl(TZ2000_GTMR_COUNTER_H);
	do {
		tmp = upper;
		lower = readl(TZ2000_GTMR_COUNTER_L);
		upper = readl(TZ2000_GTMR_COUNTER_H);
	} while (upper != tmp);

	msec = (0xffffffff / msec_count) * upper;
	msec += lower / msec_count;

	printf("\n[Boot Time] ");
	printf("%dmsec\n", msec);
}
#endif

#ifdef CONFIG_DISPLAY_PERFORMANCE
void perftime(void)
{
	u32 upper, lower, tmp;
	u32 msec, msec_count = tz2000_periph_clk / 1000;
	static u32 msec_be;

	upper = readl(TZ2000_GTMR_COUNTER_H);
	do {
		tmp = upper;
		lower = readl(TZ2000_GTMR_COUNTER_L);
		upper = readl(TZ2000_GTMR_COUNTER_H);
	} while (upper != tmp);

		msec = (0xffffffff / msec_count) * upper;
		msec += lower / msec_count;

		msec = msec - msec_be;

	if (msec_be != 0) {
		printf("\n[Perf Time] ");
		printf("%dmsec\n", msec);

		msec_be = 0;
	} else {
		msec_be = msec;
	}
}
#else
void perftime(void)
{
}
#endif

#ifdef CONFIG_CMD_NET
#include <net.h>
#include <netdev.h>

int board_eth_init(bd_t *bis)
{
	int rc = -1;
#if defined(CONFIG_TZ3000_EMAC)
	if (tz2000_board_id == 1 && tz2000_debug_board == 1) /* TZ2000RB2 */
		return rc;
	if (tz2000_board_id == 3) /* TZ2000RB3 */
		return rc;
	rc = cpu_eth_init(bis);
#endif
	return rc;
}
#endif

#ifdef CONFIG_TZ3000_SDHCI
#include <tz3000_sdhci.h>
static struct sdhci_tz3000_platform_data tz2000_emmc4sd_platdata = {
	.econf_emmc_base = TZ2000_WCONF_EMMC_BASE(0),
	.ec_tune_params = {
		[EC_SD_DS]		= { -1, -1, 0 },
		[EC_SD_HS]		= {  5, -1, 1 },
		[EC_MMC_HS]		= {  5, -1, 1 },
	},
	.f_max = 50000000,
};
static struct sdhci_tz3000_platform_data tz2000_emmc8sd_platdata = {
	.econf_emmc_base = TZ2000_WCONF_EMMC_BASE(1),
	.ec_tune_params = {
		[EC_SD_DS]		= { -1, -1, 0 },
		[EC_SD_HS]		= {  5, -1, 1 },
		[EC_MMC_HS]		= {  5, -1, 1 },
	},
	.f_max = 50000000,
};
static struct sdhci_tz3000_platform_data tz2000_emmc8sd_nand_platdata = {
	.econf_emmc_base = TZ2000_WCONF_EMMC_BASE(1),
	.ec_tune_params = {
		[EC_SD_DS]		= { -1, -1, 0 },
		[EC_SD_HS]		= { -1, -1, 0 },
		[EC_MMC_HS]		= { -1, -1, 0 },
	},
	.f_max = 100000000,
};

int tz2000_mmc_boot_dev;
int board_mmc_init(bd_t *bd)
{
	int ret;
	int dev_num = 0;
	int bootdev = readl(TZ2000_BOOTINFO_BOOTDEVICE);
	ret = tz3000_sdhci_init("eMMC4sd", TZ2000_EMMC4SD_BASE,
				&tz2000_emmc4sd_platdata);
	if (ret)
		return ret;
	if (bootdev == 5) /* SD(emmc4sd) */
		tz2000_mmc_boot_dev = dev_num;
	dev_num++;
	/* CFG_CAPABILITY0:8BITSUPPORT means internal NAND mode */
	if (readl(TZ2000_WCONF_EMMC_BASE(1)) & 0x00040000)
		ret = tz3000_sdhci_init("eMMC8sd-nand", TZ2000_EMMC8SD_BASE,
					&tz2000_emmc8sd_nand_platdata);
	else
		ret = tz3000_sdhci_init("eMMC8sd", TZ2000_EMMC8SD_BASE,
					&tz2000_emmc8sd_platdata);
	if (ret)
		return ret;
	/* 2:Internal NAND,6:eMMC,7:SD(emmc8sd) */
	if (bootdev == 2 || bootdev == 6 || bootdev == 7)
		tz2000_mmc_boot_dev = dev_num;
	return 0;
}
#endif
