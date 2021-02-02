/*
 * Toshiba TZ3000 Board specific functions
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

int tz3000_periph_clk = TZ3000_PERIPH_CLK;

#if defined(CONFIG_TZ3000_REF)
#define MACH_ID MACH_TYPE_TZ3000RB
#define BOARD_NAME "RBTZ3000-ANDES"
#elif defined(CONFIG_TZ3000_REF2)
#define MACH_ID MACH_TYPE_TZ3000RB2
#define BOARD_NAME "RBTZ3000-URAL"
#elif defined(CONFIG_TZ3000_REF3)
#define MACH_ID MACH_TYPE_TZ3000RB3
#define BOARD_NAME "RBTZ3000-CASCADE"
#elif defined(CONFIG_TZ3000_REF4)
#define MACH_ID MACH_TYPE_TZ3000RB4
#define BOARD_NAME "RBTZ3000-MANASLU"
#else
#define MACH_ID MACH_TYPE_TZ3000EVA
#define BOARD_NAME "TZ3000 EVA"
#endif

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
	unsigned long uart_base;
	unsigned long sro;

	/* call here to use __udelay. */
	timer_init();

	writel(0x01110000, TZ3000_PMU_CGOFF_PERI1); /* i2c,gpio */
	writel(0x00110011, TZ3000_PMU_CGOFF_PERI3); /* spib */
	writel(0x01111101, TZ3000_PMU_CGOFF_PERI4); /* uart */
	writel(0x01110000, TZ3000_PMU_SROFF_PERI1); /* i2c,gpio */
	writel(0x00110011, TZ3000_PMU_SROFF_PERI3); /* spib */
	writel(0x01111101, TZ3000_PMU_SROFF_PERI4); /* uart */
#if defined(CONFIG_TZ3000_EMAC) && !defined(CONFIG_TZ3000_EMAC_DISABLED)
	writel(0x11, TZ3000_PMU_SRON_ETHER);
	writel(0x01, TZ3000_PMU_CGOFF_ETHER);
#endif
#ifdef CONFIG_TZ3000_HAM_NANDC
	writel(0x101, TZ3000_PMU_SRON_NANDC);
	writel(0x101, TZ3000_PMU_CGOFF_NANDC);
#endif
#if PMU_CLK_EMMC_MASK != 0
	writel(0x010101 & PMU_CLK_EMMC_MASK, TZ3000_PMU_SRON_EMMC);
	writel(0x030303 & PMU_CLK_EMMC_MASK, TZ3000_PMU_CGOFF_EMMC);
#endif
#ifdef	CONFIG_TZ3000_REF4
	val = readl(TZ3000_ECONF_EMMCA_CAP0);
	writel(val & ~0x04000000, TZ3000_ECONF_EMMCA_CAP0);
	val = readl(TZ3000_ECONF_EMMCA_CAP1);
	writel(val & ~0x00002177, TZ3000_ECONF_EMMCA_CAP1);
#endif
	writel(0x03030311, TZ3000_PMU_SRON_HSIO);

	writel(INIT_PINSHARE0_GPIO_SEL0, TZ3000_GCONF_PINSHARE0);
	writel(INIT_PINSHARE1_GPIO_SEL1, TZ3000_GCONF_PINSHARE1);
	writel(INIT_PINSHARE3_EMMC2_SEL, TZ3000_GCONF_PINSHARE3);

#if defined(CONFIG_TZ3000_EMAC) && !defined(CONFIG_TZ3000_EMAC_DISABLED)
	writel(0x11, TZ3000_PMU_SROFF_ETHER);
#endif
#ifdef CONFIG_TZ3000_HAM_NANDC
	writel(0x101, TZ3000_PMU_SROFF_NANDC);
#endif
#if PMU_CLK_EMMC_MASK != 0
	writel(0x010101 & PMU_CLK_EMMC_MASK, TZ3000_PMU_SROFF_EMMC);
#endif

	/* if LCR was initialized, wait for txempty */
	uart_base = TZ3000_UART_BASE(CONFIG_CONS_INDEX-1);
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

	writel(INIT_GP_CR0, TZ3000_GP_CR(0));
	writel(INIT_GP_ODTP0, TZ3000_GP_ODTP(0));
	writel(INIT_GP_PUDEN0, TZ3000_GP_PUDEN(0));
	writel(INIT_GP_PUDSEL0, TZ3000_GP_PUDSEL(0));
	writel(INIT_GP_INPEN0, TZ3000_GP_INPEN(0));
	writel(INIT_GP_SMP0, TZ3000_GP_SMP(0));
	writel(INIT_GP_ODR0, TZ3000_GP_ODR(0));
	writel(INIT_GP_CR1, TZ3000_GP_CR(1));
	writel(INIT_GP_ODTP1, TZ3000_GP_ODTP(1));
	writel(INIT_GP_PUDEN1, TZ3000_GP_PUDEN(1));
	writel(INIT_GP_PUDSEL1, TZ3000_GP_PUDSEL(1));
	writel(INIT_GP_INPEN1, TZ3000_GP_INPEN(1));
	writel(INIT_GP_SMP1, TZ3000_GP_SMP(1));
	writel(INIT_GP_ODR1, TZ3000_GP_ODR(1));

#ifdef INIT_HSIO_MUX_CTL
	writel(INIT_GLOCKGATEOFF_HSIOSS, TZ3000_PMU_CGOFF_HSIO);
	writel(0x69082808, TZ3000_ECONF_USB2PHY_CONFIG0);
	writel(0x00000001, TZ3000_ECONF_USB2PHY_CONFIG1);
	writel(INIT_HSIO_MUX_CTL, TZ3000_ECONF_HSIO_MUX_CTL);
#else
	switch (readl(TZ3000_GP_DR(0)) & 0xc0) {
	case 0x00: /* Disable */
		writel(INIT_HSIO_MUX_CTL_0, TZ3000_ECONF_HSIO_MUX_CTL);
		break;
	case 0x80: /* PCIe + USB2D */
		writel(INIT_GLOCKGATEOFF_HSIOSS_1, TZ3000_PMU_CGOFF_HSIO);
		writel(0x69082808, TZ3000_ECONF_USB2PHY_CONFIG0);
		writel(0x00000001, TZ3000_ECONF_USB2PHY_CONFIG1);
		writel(INIT_HSIO_MUX_CTL_1, TZ3000_ECONF_HSIO_MUX_CTL);
		break;
	case 0x40: /* USB3H */
		writel(INIT_GLOCKGATEOFF_HSIOSS_2, TZ3000_PMU_CGOFF_HSIO);
		writel(0x69082808, TZ3000_ECONF_USB2PHY_CONFIG0);
		writel(0x00000001, TZ3000_ECONF_USB2PHY_CONFIG1);
		writel(INIT_HSIO_MUX_CTL_2, TZ3000_ECONF_HSIO_MUX_CTL);
		break;
	case 0xc0: /* PCIe + USB3H(HS) */
		writel(INIT_GLOCKGATEOFF_HSIOSS_3, TZ3000_PMU_CGOFF_HSIO);
		writel(0x69082808, TZ3000_ECONF_USB2PHY_CONFIG0);
		writel(0x00000001, TZ3000_ECONF_USB2PHY_CONFIG1);
		writel(INIT_HSIO_MUX_CTL_3, TZ3000_ECONF_HSIO_MUX_CTL);
		break;
	}
#endif

	sro = TZ3000_PMU_SROFF_HSIO;
#if defined(CONFIG_TZ3000_REF) /* PCIe + USB2H */
	writel(0x00000200, sro);
	__udelay(1);
	writel(0x03030200, sro);
	__udelay(225);
#elif defined(CONFIG_TZ3000_REF2) /* USB2D */
	writel(0x00000001, sro);
	__udelay(225);
	writel(0x00000011, sro);
	__udelay(1);
	writel(0x00000111, sro);
#elif defined(CONFIG_TZ3000_REF3) || defined(CONFIG_TZ3000_REF4) /* USB2H */
	writel(0x00000200, sro);
	__udelay(1);
	writel(0x00030200, sro);
	__udelay(225);
#else
	switch (readl(TZ3000_GP_DR(0)) & 0xc0) {
	case 0x00: /* Disable */
		break;
	case 0x80: /* PCIe + USB2D */
		writel(0x00000200, sro);
		__udelay(1);
		writel(0x03000200, sro);
		__udelay(225);
		writel(0x00000011, sro);
		__udelay(1);
		writel(0x00000111, sro);
		break;
	case 0x40: /* USB3H */
		writel(0x00000200, sro);
		__udelay(1);
		writel(0x00030200, sro);
		__udelay(225);
		break;
	case 0xc0: /* PCIe + USB3H(HS) */
		writel(0x00000200, sro);
		__udelay(1);
		writel(0x03030200, sro);
		__udelay(225);
		break;
	}
#endif

#ifdef CONFIG_TZ3000_SPI
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
	int bootdev = readl(TZ3000_BOOTINFO_BOOTDEVICE) & 7;
	struct env_ops *envops[8] = {
#ifdef CONFIG_ENV_IS_IN_FLASH
		[0] = &flash_env_ops,
		[2] = &flash_env_ops,
#endif
#ifdef CONFIG_ENV_IS_IN_SPI_FLASH
		[0] = &spi_flash_env_ops,
		[2] = &spi_flash_env_ops,
#endif
#ifdef CONFIG_ENV_IS_IN_NAND
		[4] = &nand_env_ops,
		[5] = &nand_env_ops,
#endif
#ifdef CONFIG_ENV_IS_IN_MMC
		[6] = &mmc_env_ops,
		[7] = &mmc_env_ops,
#endif
	};
	if (envops[bootdev])
		env_ops = envops[bootdev];
#endif /* CONFIG_ENV_IS_SELECTABLE */
}

static void detect_pll(void)
{
	static struct {
		int pllclk;
		unsigned int conf0, conf1;
	} pllconfigs[] = {
		{ 1000000000, 0x27183000, 0x00000027 },
		{  800000000, 0x27133000, 0x0000001f },
		{  600000000, 0x270e3000, 0x00000017 },
		{  400000000, 0x27133000, 0x1000001f },
		{  200000000, 0x27133000, 0x1001001f },
		{  100000000, 0x27133000, 0x1003001f },
	};
	unsigned int conf0, conf1;
	int i;

	conf0 = readl(TZ3000_PMU_PLLCONFIG_PLLCPU_0);
	conf1 = readl(TZ3000_PMU_PLLCONFIG_PLLCPU_1);
	for (i = 0; i < ARRAY_SIZE(pllconfigs); i++) {
		if (pllconfigs[i].conf0 == conf0 &&
		    pllconfigs[i].conf1 == conf1) {
			tz3000_periph_clk = pllconfigs[i].pllclk / 16;
			break;
		}
	}
}

int board_early_init_f(void)
{
	detect_pll();
	early_init_peripherals();
	init_env_ops();
	return 0;
}

int board_init(void)
{
	gd->bd->bi_arch_number = MACH_ID;
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
	/* to protect whole sysbin area */
	monitor_flash_len = CONFIG_SYS_MONITOR_LEN;
	return 0;
}

int dram_init(void)
{
	gd->ram_size = readl(TZ3000_BOOTINFO_MEMSIZE);
	return 0;
}

#ifdef CONFIG_DISPLAY_BOARDINFO
int checkboard(void)
{
	int bootdev = readl(TZ3000_BOOTINFO_BOOTDEVICE) & 7;
	const char *bootdev_name[] = {
		"SPI CH0 (XIP)",
		"Reserved",
		"SPI CH0",
		"Reserved",
		"NAND (4 cyc)",
		"NAND (5 cyc)",
		"eMMC",
		"SD Card",
	};
	puts("Board: ");
	puts(BOARD_NAME);
	puts("\n");
	printf("BootDev: %s\n", bootdev_name[bootdev]);

	return 0;
}
#endif

#ifdef CONFIG_DISPLAY_BOOTTIME
void boottime(void)
{
	u32 upper, lower, tmp;
	u32 msec, msec_count = tz3000_periph_clk / 1000;

	upper = readl(TZ3000_GTMR_COUNTER_H);
	do {
		tmp = upper;
		lower = readl(TZ3000_GTMR_COUNTER_L);
		upper = readl(TZ3000_GTMR_COUNTER_H);
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
	u32 msec, msec_count = tz3000_periph_clk / 1000;
	static u32 msec_be;

	upper = readl(TZ3000_GTMR_COUNTER_H);
	do {
		tmp = upper;
		lower = readl(TZ3000_GTMR_COUNTER_L);
		upper = readl(TZ3000_GTMR_COUNTER_H);
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
	rc = cpu_eth_init(bis);
#endif
	return rc;
}
#endif

#ifdef CONFIG_TZ3000_SDHCI
#include <tz3000_sdhci.h>
#if (PMU_CLK_EMMC_MASK & 0x00000f) != 0
static struct sdhci_tz3000_platform_data tz3000_emmca_platdata = {
	.econf_emmc_base = TZ3000_ECONF_EMMC_BASE(0),
	.ec_tune_params = {
		[EC_SD_DS]		= { -1, -1, 1 },
		[EC_SD_HS]		= { 15, -1, 1 },
		[EC_MMC_HS]		= { 15, -1, 1 },
	}
};
#endif
#if (PMU_CLK_EMMC_MASK & 0x000f00) != 0
static struct sdhci_tz3000_platform_data tz3000_emmcb_platdata = {
	.econf_emmc_base = TZ3000_ECONF_EMMC_BASE(1),
	.ec_tune_params = {
		[EC_SD_DS]		= { -1, -1, 1 },
		[EC_SD_HS]		= { 15, -1, 1 },
		[EC_MMC_HS]		= { 15, -1, 1 },
	}
};
#endif
#if (PMU_CLK_EMMC_MASK & 0x0f0000) != 0
static struct sdhci_tz3000_platform_data tz3000_emmcc_platdata = {
	.econf_emmc_base = TZ3000_ECONF_EMMC_BASE(2),
	.ec_tune_params = {
		[EC_SD_DS]		= { -1, -1, 1 },
		[EC_SD_HS]		= { 15, -1, 1 },
		[EC_MMC_HS]		= { 15, -1, 1 },
	}
};
#endif

int tz3000_mmc_boot_dev;
int board_mmc_init(bd_t *bd)
{
	int ret = 0;
	int dev_num = 0;
	int bootdev = readl(TZ3000_BOOTINFO_BOOTDEVICE);
#if (PMU_CLK_EMMC_MASK & 0x00000f) != 0
	ret = tz3000_sdhci_init("eMMCa", TZ3000_EMMC_BASE(0),
				&tz3000_emmca_platdata);
	if (ret)
		return ret;
	if (bootdev == 7) /* emmc-a */
		tz3000_mmc_boot_dev = dev_num;
	dev_num++;
#endif
#if (PMU_CLK_EMMC_MASK & 0x000f00) != 0
	ret = tz3000_sdhci_init("eMMCb", TZ3000_EMMC_BASE(1),
				&tz3000_emmcb_platdata);
	if (ret)
		return ret;
	dev_num++;
#endif
#if (PMU_CLK_EMMC_MASK & 0x0f0000) != 0
	ret = tz3000_sdhci_init("eMMCc", TZ3000_EMMC_BASE(2),
				&tz3000_emmcc_platdata);
	if (ret)
		return ret;
	if (bootdev == 6) /* emmc-c */
		tz3000_mmc_boot_dev = dev_num;
#endif
	return 0;
}
#endif

#ifdef CONFIG_CMD_LEDSTAT
#include <watchdog.h>
#define MAX_LEDS 8
enum led_status_t {
	LED_STATUS_OFF,
	LED_STATUS_ON,
	LED_STATUS_TOGGLE,
};
static struct ledstat {
	enum led_status_t led_status;
	enum led_status_t last_status;
	int led_gpio;
} ledstats[MAX_LEDS] = {
	[0 ... MAX_LEDS - 1] = { LED_STATUS_OFF, LED_STATUS_OFF, -1 }
};

static void update_led(int gpio, int led_on)
{
	int bank = gpio / 32;
	unsigned int bit = 1 << (gpio % 32);
	if (led_on)
		writel(bit, TZ3000_GP_ODS(bank));
	else
		writel(bit, TZ3000_GP_ODC(bank));
}

void watchdog_reset(void)
{
	static ulong last_time;
	static int toggle;
	ulong this_time;
	int i;

	if (ledstats[0].led_gpio < 0)
		return;
	for (i = 0; i < MAX_LEDS && ledstats[i].led_gpio >= 0; i++) {
		if (ledstats[i].led_status != ledstats[i].last_status)
			ledstats[i].last_status = ledstats[i].led_status;
		if (ledstats[i].led_status != LED_STATUS_TOGGLE)
			update_led(ledstats[i].led_gpio,
				   ledstats[i].led_status == LED_STATUS_ON);
	}
	this_time = get_timer(last_time);
	if (this_time < 500)
		return;
	last_time += this_time;
	toggle = !toggle;
	for (i = 0; i < MAX_LEDS && ledstats[i].led_gpio >= 0; i++)
		if (ledstats[i].led_status == LED_STATUS_TOGGLE)
			update_led(ledstats[i].led_gpio, toggle);
}

static void show_ledstat(int gpio, enum led_status_t status)
{
	printf("%d\t", gpio);
	switch (status) {
	case LED_STATUS_ON:
		printf("on");
		break;
	case LED_STATUS_TOGGLE:
		printf("toggle");
		break;
	default:
		printf("off");
		break;
	}
	printf("\n");
}

static int do_ledstat(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int i;
	int gpio;
	enum led_status_t new_status;

	if (argc == 1) {
		for (i = 0; i < MAX_LEDS && ledstats[i].led_gpio >= 0; i++)
			show_ledstat(ledstats[i].led_gpio,
				     ledstats[i].led_status);
		return 0;
	}
	gpio = simple_strtoul(argv[1], NULL, 10);
	if (gpio >= 64) {
		cmd_usage(cmdtp);
		return -1;
	}
	if (argc == 2) {
		for (i = 0; i < MAX_LEDS && ledstats[i].led_gpio >= 0; i++)
			if (ledstats[i].led_gpio == gpio)
				show_ledstat(ledstats[i].led_gpio,
					     ledstats[i].led_status);
		return 0;
	}
	if (strcmp(argv[2], "on") == 0) {
		new_status = LED_STATUS_ON;
	} else if (strcmp(argv[2], "off") == 0) {
		new_status = LED_STATUS_OFF;
	} else if (strcmp(argv[2], "toggle") == 0) {
		new_status = LED_STATUS_TOGGLE;
	} else {
		cmd_usage(cmdtp);
		return -1;
	}
	for (i = 0; i < MAX_LEDS; i++) {
		if (ledstats[i].led_gpio == gpio) {
			ledstats[i].led_status = new_status;
			return 0;
		}
	}
	for (i = 0; i < MAX_LEDS; i++) {
		if (ledstats[i].led_gpio < 0) {
			ledstats[i].led_gpio = gpio;
			ledstats[i].led_status = new_status;
			return 0;
		}
	}
	printf("no empty slot for ledstat\n");
	return -1;
}

U_BOOT_CMD(
	ledstat,	3,	0,	do_ledstat,
	"set GPIO LED status",
	"[gpio_led_number [on|off|toggle]]"
);
#endif /* CONFIG_CMD_LEDSTAT */
