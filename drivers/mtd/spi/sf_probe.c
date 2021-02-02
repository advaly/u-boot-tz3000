/*
 * SPI flash probing
 *
 * Copyright (C) 2008 Atmel Corporation
 * Copyright (C) 2010 Reinhard Meyer, EMK Elektronik
 * Copyright (C) 2013 Jagannadha Sutradharudu Teki, Xilinx Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <fdtdec.h>
#include <malloc.h>
#include <spi.h>
#include <spi_flash.h>

#include "sf_internal.h"

DECLARE_GLOBAL_DATA_PTR;

/**
 * struct spi_flash_params - SPI/QSPI flash device params structure
 *
 * @name:		Device name ([MANUFLETTER][DEVTYPE][DENSITY][EXTRAINFO])
 * @jedec:		Device jedec ID (0x[1byte_manuf_id][2byte_dev_id])
 * @ext_jedec:		Device ext_jedec ID
 * @sector_size:	Sector size of this device
 * @nr_sectors:		No.of sectors on this device
 * @flags:		Importent param, for flash specific behaviour
 */
struct spi_flash_params {
	const char *name;
	u32 jedec;
	u16 ext_jedec;
	u32 sector_size;
	u32 nr_sectors;
	u16 flags;
};

static const struct spi_flash_params spi_flash_params_table[] = {
#ifdef CONFIG_SPI_FLASH_ATMEL		/* ATMEL */
	{"AT45DB011D",	   0x1f2200, 0x0,	64 * 1024,     4,	       SECT_4K},
	{"AT45DB021D",	   0x1f2300, 0x0,	64 * 1024,     8,	       SECT_4K},
	{"AT45DB041D",	   0x1f2400, 0x0,	64 * 1024,     8,	       SECT_4K},
	{"AT45DB081D",	   0x1f2500, 0x0,	64 * 1024,    16,	       SECT_4K},
	{"AT45DB161D",	   0x1f2600, 0x0,	64 * 1024,    32,	       SECT_4K},
	{"AT45DB321D",	   0x1f2700, 0x0,	64 * 1024,    64,	       SECT_4K},
	{"AT45DB641D",	   0x1f2800, 0x0,	64 * 1024,   128,	       SECT_4K},
	{"AT25DF321",      0x1f4701, 0x0,	64 * 1024,    64,	       SECT_4K},
#endif
#ifdef CONFIG_SPI_FLASH_EON		/* EON */
	{"EN25Q32B",	   0x1c3016, 0x0,	64 * 1024,    64,	             0},
	{"EN25Q64",	   0x1c3017, 0x0,	64 * 1024,   128,	       SECT_4K},
	{"EN25Q128B",	   0x1c3018, 0x0,       64 * 1024,   256,	             0},
	{"EN25S64",	   0x1c3817, 0x0,	64 * 1024,   128,		     0},
#endif
#ifdef CONFIG_SPI_FLASH_GIGADEVICE	/* GIGADEVICE */
	{"GD25Q64B",	   0xc84017, 0x0,	64 * 1024,   128,	       SECT_4K},
	{"GD25LQ32",	   0xc86016, 0x0,	64 * 1024,    64,	       SECT_4K},
#endif
#ifdef CONFIG_SPI_FLASH_MACRONIX	/* MACRONIX */
	{"MX25L4005",	   0xc22013, 0x0,	64 * 1024,     8,	             0},
	{"MX25L8005",	   0xc22014, 0x0,	64 * 1024,    16,	             0},
	{"MX25L1605D",	   0xc22015, 0x0,	64 * 1024,    32,	             0},
	{"MX25L3205D",	   0xc22016, 0x0,	64 * 1024,    64,	             0},
	{"MX25L6405D",	   0xc22017, 0x0,	64 * 1024,   128,	             0},
	{"MX25L12805",	   0xc22018, 0x0,	64 * 1024,   256,	             0},
	{"MX25L25635F",	   0xc22019, 0x0,	64 * 1024,   512,	             0},
	{"MX25L51235F",	   0xc2201A, 0x0,	64 * 1024,  1024,	             0},
	{"MX25U6435F",	   0xc22537, 0x0,	64 * 1024,   128,	             0},
	{"MX25U12835F",	   0xc22538, 0x0,	64 * 1024,   256,	             0},
	{"MX25L12855E",	   0xc22618, 0x0,	64 * 1024,   256,	             0},
#endif
#ifdef CONFIG_SPI_FLASH_SPANSION	/* SPANSION */
	{"S25FL008A",	   0x010213, 0x0,	64 * 1024,    16,	             0},
	{"S25FL016A",	   0x010214, 0x0,	64 * 1024,    32,	             0},
	{"S25FL032A",	   0x010215, 0x0,	64 * 1024,    64,	             0},
	{"S25FL064A",	   0x010216, 0x0,	64 * 1024,   128,	             0},
	{"S25FL128P_256K", 0x012018, 0x0300,   256 * 1024,    64,	             0},
	{"S25FL128P_64K",  0x012018, 0x0301,    64 * 1024,   256,	             0},
	{"S25FL032P",	   0x010215, 0x4d00,    64 * 1024,    64,	             0},
	{"S25FL064P",	   0x010216, 0x4d00,    64 * 1024,   128,	             0},
	{"S25FL128S_64K",  0x012018, 0x4d01,    64 * 1024,   256,		     0},
	{"S25FL256S_256K", 0x010219, 0x4d00,    64 * 1024,   512,	             0},
	{"S25FL256S_64K",  0x010219, 0x4d01,    64 * 1024,   512,	             0},
	{"S25FL512S_256K", 0x010220, 0x4d00,    64 * 1024,  1024,	             0},
	{"S25FL512S_64K",  0x010220, 0x4d01,    64 * 1024,  1024,	             0},
#endif
#ifdef CONFIG_SPI_FLASH_STMICRO		/* STMICRO */
	{"M25P10",	   0x202011, 0x0,       32 * 1024,     4,	             0},
	{"M25P20",	   0x202012, 0x0,       64 * 1024,     4,	             0},
	{"M25P40",	   0x202013, 0x0,       64 * 1024,     8,	             0},
	{"M25P80",	   0x202014, 0x0,       64 * 1024,    16,	             0},
	{"M25P16",	   0x202015, 0x0,       64 * 1024,    32,	             0},
	{"M25P32",	   0x202016, 0x0,       64 * 1024,    64,	             0},
	{"M25P64",	   0x202017, 0x0,       64 * 1024,   128,	             0},
	{"M25P128",	   0x202018, 0x0,      256 * 1024,    64,	             0},
	{"N25Q32",	   0x20ba16, 0x0,       64 * 1024,    64,	       SECT_4K},
	{"N25Q32A",	   0x20bb16, 0x0,       64 * 1024,    64,	       SECT_4K},
	{"N25Q64",	   0x20ba17, 0x0,       64 * 1024,   128,	       SECT_4K},
	{"N25Q64A",	   0x20bb17, 0x0,       64 * 1024,   128,	       SECT_4K},
	{"N25Q128",	   0x20ba18, 0x0,       64 * 1024,   256,	       SECT_4K},
	{"N25Q128A",	   0x20bb18, 0x0,       64 * 1024,   256,	       SECT_4K},
	{"N25Q256",	   0x20ba19, 0x0,       64 * 1024,   512,	       SECT_4K},
	{"N25Q256A",	   0x20bb19, 0x0,       64 * 1024,   512,	       SECT_4K},
	{"N25Q512",	   0x20ba20, 0x0,       64 * 1024,  1024,      E_FSR | SECT_4K},
	{"N25Q512A",	   0x20bb20, 0x0,       64 * 1024,  1024,      E_FSR | SECT_4K},
	{"N25Q1024",	   0x20ba21, 0x0,       64 * 1024,  2048,      E_FSR | SECT_4K},
	{"N25Q1024A",	   0x20bb21, 0x0,       64 * 1024,  2048,      E_FSR | SECT_4K},
#endif
#ifdef CONFIG_SPI_FLASH_SST		/* SST */
	{"SST25VF040B",	   0xbf258d, 0x0,	64 * 1024,     8,     SECT_4K | SST_WP},
	{"SST25VF080B",	   0xbf258e, 0x0,	64 * 1024,    16,     SECT_4K | SST_WP},
	{"SST25VF016B",	   0xbf2541, 0x0,	64 * 1024,    32,     SECT_4K | SST_WP},
	{"SST25VF032B",	   0xbf254a, 0x0,	64 * 1024,    64,     SECT_4K | SST_WP},
	{"SST25VF064C",	   0xbf254b, 0x0,	64 * 1024,   128,	       SECT_4K},
	{"SST25WF512",	   0xbf2501, 0x0,	64 * 1024,     1,     SECT_4K | SST_WP},
	{"SST25WF010",	   0xbf2502, 0x0,	64 * 1024,     2,     SECT_4K | SST_WP},
	{"SST25WF020",	   0xbf2503, 0x0,	64 * 1024,     4,     SECT_4K | SST_WP},
	{"SST25WF040",	   0xbf2504, 0x0,	64 * 1024,     8,     SECT_4K | SST_WP},
	{"SST25WF080",	   0xbf2505, 0x0,	64 * 1024,    16,     SECT_4K | SST_WP},
#endif
#ifdef CONFIG_SPI_FLASH_WINBOND		/* WINBOND */
	{"W25P80",	   0xef2014, 0x0,	64 * 1024,    16,		    0},
	{"W25P16",	   0xef2015, 0x0,	64 * 1024,    32,		    0},
	{"W25P32",	   0xef2016, 0x0,	64 * 1024,    64,		    0},
	{"W25X40",	   0xef3013, 0x0,	64 * 1024,     8,	      SECT_4K},
	{"W25X16",	   0xef3015, 0x0,	64 * 1024,    32,	      SECT_4K},
	{"W25X32",	   0xef3016, 0x0,	64 * 1024,    64,	      SECT_4K},
	{"W25X64",	   0xef3017, 0x0,	64 * 1024,   128,	      SECT_4K},
	{"W25Q80BL",	   0xef4014, 0x0,	64 * 1024,    16,	      SECT_4K},
	{"W25Q16CL",	   0xef4015, 0x0,	64 * 1024,    32,	      SECT_4K},
	{"W25Q32BV",	   0xef4016, 0x0,	64 * 1024,    64,	      SECT_4K},
	{"W25Q64CV",	   0xef4017, 0x0,	64 * 1024,   128,	      SECT_4K},
	{"W25Q128BV",	   0xef4018, 0x0,	64 * 1024,   256,	      SECT_4K},
	{"W25Q256",	   0xef4019, 0x0,	64 * 1024,   512,	      SECT_4K},
	{"W25Q80BW",	   0xef5014, 0x0,	64 * 1024,    16,	      SECT_4K},
	{"W25Q16DW",	   0xef6015, 0x0,	64 * 1024,    32,	      SECT_4K},
	{"W25Q32DW",	   0xef6016, 0x0,	64 * 1024,    64,	      SECT_4K},
	{"W25Q64DW",	   0xef6017, 0x0,	64 * 1024,   128,	      SECT_4K},
	{"W25Q128FW",	   0xef6018, 0x0,	64 * 1024,   256,	      SECT_4K},
#endif
	/*
	 * Note:
	 * Below paired flash devices has similar spi_flash params.
	 * (S25FL129P_64K, S25FL128S_64K)
	 * (W25Q80BL, W25Q80BV)
	 * (W25Q16CL, W25Q16DV)
	 * (W25Q32BV, W25Q32FV_SPI)
	 * (W25Q64CV, W25Q64FV_SPI)
	 * (W25Q128BV, W25Q128FV_SPI)
	 * (W25Q32DW, W25Q32FV_QPI)
	 * (W25Q64DW, W25Q64FV_QPI)
	 * (W25Q128FW, W25Q128FV_QPI)
	 */
};

#ifdef CONFIG_TZ3000_SPI
static struct spi_flash_tune_params {
	const char *name;
	unsigned int max_hz;
	unsigned int deassert_time;
	unsigned char dir_read_opcode;
	unsigned int dir_read_max_hz;
} spi_flash_tune_table[] = {
	{ "S25FL128S_64K", 104000000, 50, OPCODE_FAST_READ_QUAD_IO, 80000000 },
	{ "MX25U12835F", 104000000, 30, OPCODE_FAST_READ_QUAD_IO, 84000000 },
	{ "MX25L12805", 133000000, 30, OPCODE_FAST_READ_QUAD_IO, 84000000 },
	{ "MX25L51235F", 133000000, 30, OPCODE_FAST_READ_QUAD_IO, 84000000 },
	{ "S25FL512S_256K", 104000000, 50, OPCODE_FAST_READ_QUAD_IO, 80000000 },
	{ "N25Q512", 108000000, 50, OPCODE_FAST_READ_QUAD_IO, 33333334 },
};

#ifdef CONFIG_SPI_FLASH_STMICRO		/* STMICRO */
#define CMD_STMICRO_RD_VCR	0x85	/* VOLATILE CONFIGURATION */
#define CMD_STMICRO_WR_VCR	0x81
static int stmicro_enable_qio(struct spi_flash *flash)
{
	int ret;
	u8 cmd, conf;
	int cycles = 8;

	if (strcmp(flash->name, "N25Q512") != 0)
		return 0;
	cmd = CMD_STMICRO_RD_VCR;
	ret = spi_flash_cmd(flash->spi, cmd, &conf, sizeof(conf));
	if (ret)
		return ret;

	ret = spi_flash_cmd_write_enable(flash);
	if (ret)
		return ret;

	conf = (conf & 0x0f) | (cycles << 4);
	cmd = CMD_STMICRO_WR_VCR;
	ret = spi_flash_cmd_write(flash->spi, &cmd, 1, &conf, 1);
	if (ret)
		return ret;

	cmd = CMD_STMICRO_RD_VCR;
	ret = spi_flash_cmd(flash->spi, cmd, &conf, sizeof(conf));
	if (ret)
		return ret;
	debug("SF: STMICRO: VCR 0x%x\n", conf);
	return 0;
}
#endif

#ifdef CONFIG_SPI_FLASH_MACRONIX	/* MACRONIX */
#define MACRONIX_SR_QE		(1 << 6)	/* Quad Mode Enable */
static int macronix_enable_qio(struct spi_flash *flash)
{
	int ret;
	u8 cmd, status;

	cmd = CMD_READ_STATUS;
	ret = spi_flash_cmd(flash->spi, cmd, &status, sizeof(status));
	if (ret)
		return ret;

	ret = spi_flash_cmd_write_enable(flash);
	if (ret)
		return ret;

	status |= MACRONIX_SR_QE;
	ret = spi_flash_cmd_write_status(flash, status);
	if (ret)
		return ret;

	return spi_flash_cmd_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
}
#endif

#ifdef CONFIG_SPI_FLASH_SPANSION	/* SPANSION */
#define SPANSION_CR_QUAD	(1 << 1)	/* Quad Mode Enable */
static int spansion_enable_qio(struct spi_flash *flash)
{
	int ret;
	u8 cmd, status[2];

	cmd = CMD_READ_STATUS;
	ret = spi_flash_cmd(flash->spi, cmd, &status[0], sizeof(status[0]));
	if (ret)
		return ret;

	cmd = CMD_READ_CONFIG;
	ret = spi_flash_cmd(flash->spi, cmd, &status[1], sizeof(status[1]));
	if (ret)
		return ret;

	ret = spi_flash_cmd_write_enable(flash);
	if (ret)
		return ret;

	status[1] |= SPANSION_CR_QUAD;
	cmd = CMD_WRITE_STATUS;
	ret = spi_flash_cmd_write(flash->spi, &cmd, 1, status, sizeof(status));
	if (ret)
		return ret;

	return spi_flash_cmd_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
}
#endif

static void spi_flash_tune_params(struct spi_slave *spi,
				  struct spi_flash *flash,
				  const struct spi_flash_params *params)
{
	struct tz3000_spi_data *data = spi->tz3000_spi_data;
	struct spi_flash_tune_params *tune = NULL;
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(spi_flash_tune_table); i++) {
		if (strcmp(flash->name, spi_flash_tune_table[i].name) == 0) {
			tune = &spi_flash_tune_table[i];
			break;
		}
	}
	if (tune) {
		data->deassert_time = tune->deassert_time;
		data->max_hz = tune->max_hz;
#ifdef CONFIG_SYS_FLASH_SPI_HIGH_PERFORMANCE_READ
		data->max_map_read_hz = tune->dir_read_max_hz;
		data->dir_read_opcode = tune->dir_read_opcode;
#else
		data->max_map_read_hz = tune->max_hz;
		data->dir_read_opcode = OPCODE_FAST_READ_SINGLE;
#endif
		if (tune->dir_read_opcode == OPCODE_FAST_READ_QUAD_IO) {
			data->dir_read_dummy_count = 3;
			if (strcmp(tune->name, "N25Q512") == 0)
				data->dir_read_dummy_count = 4;
		} else {
			data->dir_read_dummy_count = 1;
		}
		if (tune->dir_read_opcode == OPCODE_FAST_READ_QUAD_IO) {
			ret = 0;
#ifdef CONFIG_SPI_FLASH_STMICRO		/* STMICRO */
			if (params->jedec >> 16 == 0x20)
				ret = stmicro_enable_qio(flash);
#endif
#ifdef CONFIG_SPI_FLASH_MACRONIX	/* MACRONIX */
			if (params->jedec >> 16 == 0xc2)
				ret = macronix_enable_qio(flash);
#endif
#ifdef CONFIG_SPI_FLASH_SPANSION	/* SPANSION */
			if (params->jedec >> 16 == 0x01)
				ret = spansion_enable_qio(flash);
#endif
			if (ret) {
				debug("SF: Enabling QIO Failed\n");
				tune = NULL; /* fallback to single */
			}
		}
	}
	if (!tune) {
		data->max_map_read_hz = data->max_hz;
		data->dir_read_opcode = OPCODE_FAST_READ_SINGLE;
		data->dir_read_dummy_count = 1;
	}
#ifdef CONFIG_SPI_FLASH_4BYTE_MODE
	data->use_4byte_mode = spi_flash_use_4byte_mode(flash);
#endif
}
#endif

#ifdef CONFIG_SPI_FLASH_4BYTE_MODE
#define	CMD_STMICRO_EN4B		0xb7	/* Enter 4-byte mode */
static int stmicro_set_4byte_mode(struct spi_flash *flash)
{
	struct spi_slave *spi = flash->spi;
	int ret;

	ret = spi_flash_cmd_write_enable(flash);
	if (ret < 0) {
		debug("SF: Enabling Write failed\n");
		return ret;
	}
	ret = spi_flash_cmd(spi, CMD_STMICRO_EN4B, NULL, 0);
	if (ret < 0) {
		debug("SF: Enabling 4byte failed\n");
		return ret;
	}
	spi_flash_cmd_write_disable(flash);
	return ret;
}
#define	CMD_MACRONIX_EN4B		0xb7	/* Enter 4-byte mode */
static int macronix_set_4byte_mode(struct spi_flash *flash)
{
	struct spi_slave *spi = flash->spi;
	int ret;

	ret = spi_flash_cmd(spi, CMD_MACRONIX_EN4B, NULL, 0);
	if (ret < 0) {
		debug("SF: Enabling 4byte failed\n");
		return ret;
	}
	return ret;
}
#define	CMD_SPANSION_BRWR	0x17	/* Bank register write */
static int spansion_set_4byte_mode(struct spi_flash *flash)
{
	struct spi_slave *spi = flash->spi;
	int ret;
	u8 cmd;
	u8 val = 1 << 7;

	cmd = CMD_SPANSION_BRWR;
	ret = spi_flash_cmd_write(spi, &cmd, 1, &val, 1);
	if (ret < 0) {
		debug("SF: Enabling 4byte failed\n");
		return ret;
	}
	return ret;
}
#else
#define CMD_STMICRO_RESET_EN	0x66
#define CMD_STMICRO_RESET_MEMORY	0x99
static int stmicro_reset_memory(struct spi_flash *flash)
{
	struct spi_slave *spi = flash->spi;
	int ret;

	ret = spi_flash_cmd(spi, CMD_STMICRO_RESET_EN, NULL, 0);
	if (ret < 0) {
		debug("SF: Enabling Reset failed\n");
		return ret;
	}
	ret = spi_flash_cmd(spi, CMD_STMICRO_RESET_MEMORY, NULL, 0);
	if (ret < 0) {
		debug("SF: Reset Memory failed\n");
		return ret;
	}
	return ret;
}
#define CMD_MACRONIX_RESET_EN	0x66
#define CMD_MACRONIX_RESET_MEMORY	0x99
static int macronix_reset_memory(struct spi_flash *flash)
{
	struct spi_slave *spi = flash->spi;
	int ret;

	ret = spi_flash_cmd(spi, CMD_MACRONIX_RESET_EN, NULL, 0);
	if (ret < 0) {
		debug("SF: Enabling Reset failed\n");
		return ret;
	}
	ret = spi_flash_cmd(spi, CMD_MACRONIX_RESET_MEMORY, NULL, 0);
	if (ret < 0) {
		debug("SF: Reset Memory failed\n");
		return ret;
	}
	return ret;
}
#define	CMD_SPANSION_RESET	0xf0	/* Reset */
static int spansion_reset_memory(struct spi_flash *flash)
{
	struct spi_slave *spi = flash->spi;
	int ret;

	ret = spi_flash_cmd(spi, CMD_SPANSION_RESET, NULL, 0);
	if (ret < 0) {
		debug("SF: Reset Memory failed\n");
		return ret;
	}
	return ret;
}
#endif

static struct spi_flash *spi_flash_validate_params(struct spi_slave *spi,
		u8 *idcode)
{
	const struct spi_flash_params *params;
	struct spi_flash *flash;
	int i;
	u16 jedec = idcode[1] << 8 | idcode[2];
	u16 ext_jedec = idcode[3] << 8 | idcode[4];

	/* Get the flash id (jedec = manuf_id + dev_id, ext_jedec) */
	for (i = 0; i < ARRAY_SIZE(spi_flash_params_table); i++) {
		params = &spi_flash_params_table[i];
		if ((params->jedec >> 16) == idcode[0]) {
			if ((params->jedec & 0xFFFF) == jedec) {
				if (params->ext_jedec == 0)
					break;
				else if (params->ext_jedec == ext_jedec)
					break;
			}
		}
	}

	if (i == ARRAY_SIZE(spi_flash_params_table)) {
		printf("SF: Unsupported flash IDs: ");
		printf("manuf %02x, jedec %04x, ext_jedec %04x\n",
		       idcode[0], jedec, ext_jedec);
		return NULL;
	}

	flash = malloc(sizeof(*flash));
	if (!flash) {
		debug("SF: Failed to allocate spi_flash\n");
		return NULL;
	}
	memset(flash, '\0', sizeof(*flash));

	/* Assign spi data */
	flash->spi = spi;
	flash->name = params->name;
	flash->memory_map = spi->memory_map;

	/* Assign spi_flash ops */
	flash->write = spi_flash_cmd_write_ops;
#ifdef CONFIG_SPI_FLASH_SST
	if (params->flags & SST_WP)
		flash->write = sst_write_wp;
#endif
	flash->erase = spi_flash_cmd_erase_ops;
	flash->read = spi_flash_cmd_read_ops;

	/* Compute the flash size */
	flash->page_size = (ext_jedec == 0x4d00) ? 512 : 256;
	flash->sector_size = params->sector_size;
	flash->size = flash->sector_size * params->nr_sectors;

#ifdef CONFIG_SPI_FLASH_4BYTE_MODE
	if (flash->size > (1 << 24)) {
		if (params->jedec >> 16 == 0x20)
			flash->set_4byte_mode = stmicro_set_4byte_mode;
		if (params->jedec >> 16 == 0xc2)
			flash->set_4byte_mode = macronix_set_4byte_mode;
		if (params->jedec >> 16 == 0x01)
			flash->set_4byte_mode = spansion_set_4byte_mode;
	}
#else
	if (params->jedec >> 16 == 0x20)
		stmicro_reset_memory(flash);
	if (params->jedec >> 16 == 0xc2)
		macronix_reset_memory(flash);
	if (params->jedec >> 16 == 0x01)
		spansion_reset_memory(flash);
#endif
	/* Compute erase sector and command */
	if (params->flags & SECT_4K) {
		flash->erase_cmd = CMD_ERASE_4K;
		flash->erase_size = 4096;
	} else if (params->flags & SECT_32K) {
		flash->erase_cmd = CMD_ERASE_32K;
		flash->erase_size = 32768;
	} else {
		flash->erase_cmd = CMD_ERASE_64K;
		flash->erase_size = flash->sector_size;
	}

	/* Poll cmd seclection */
	flash->poll_cmd = CMD_READ_STATUS;
#ifdef CONFIG_SPI_FLASH_STMICRO
	if (params->flags & E_FSR)
		flash->poll_cmd = CMD_FLAG_STATUS;
#endif

	/* Configure the BAR - discover bank cmds and read current bank */
#ifdef CONFIG_SPI_FLASH_BAR
	u8 curr_bank = 0;
	if (flash->size > SPI_FLASH_16MB_BOUN) {
		flash->bank_read_cmd = (idcode[0] == 0x01) ?
					CMD_BANKADDR_BRRD : CMD_EXTNADDR_RDEAR;
		flash->bank_write_cmd = (idcode[0] == 0x01) ?
					CMD_BANKADDR_BRWR : CMD_EXTNADDR_WREAR;

		if (spi_flash_read_common(flash, &flash->bank_read_cmd, 1,
					  &curr_bank, 1)) {
			debug("SF: fail to read bank addr register\n");
			return NULL;
		}
		flash->bank_curr = curr_bank;
	} else {
		flash->bank_curr = curr_bank;
	}
#endif

	/* Flash powers up read-only, so clear BP# bits */
#if defined(CONFIG_SPI_FLASH_ATMEL) || \
	defined(CONFIG_SPI_FLASH_MACRONIX) || \
	defined(CONFIG_SPI_FLASH_SST)
		spi_flash_cmd_write_status(flash, 0);
#endif
	if (spi_flash_set_4byte_mode(flash)) {
		printf("SF: Failed to enable 4 byte mode\n");
		flash->set_4byte_mode = NULL;
	}
#ifdef CONFIG_TZ3000_SPI
	if (spi->tz3000_spi_data)
		spi_flash_tune_params(spi, flash, params);
#endif

	return flash;
}

#ifdef CONFIG_OF_CONTROL
int spi_flash_decode_fdt(const void *blob, struct spi_flash *flash)
{
	fdt_addr_t addr;
	fdt_size_t size;
	int node;

	/* If there is no node, do nothing */
	node = fdtdec_next_compatible(blob, 0, COMPAT_GENERIC_SPI_FLASH);
	if (node < 0)
		return 0;

	addr = fdtdec_get_addr_size(blob, node, "memory-map", &size);
	if (addr == FDT_ADDR_T_NONE) {
		debug("%s: Cannot decode address\n", __func__);
		return 0;
	}

	if (flash->size != size) {
		debug("%s: Memory map must cover entire device\n", __func__);
		return -1;
	}
	flash->memory_map = (void *)addr;

	return 0;
}
#endif /* CONFIG_OF_CONTROL */

struct spi_flash *spi_flash_probe(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int spi_mode)
{
	struct spi_slave *spi;
	struct spi_flash *flash = NULL;
	u8 idcode[5];
	int ret;

	/* Setup spi_slave */
	spi = spi_setup_slave(bus, cs, max_hz, spi_mode);
	if (!spi) {
		printf("SF: Failed to set up slave\n");
		return NULL;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		debug("SF: Failed to claim SPI bus: %d\n", ret);
		goto err_claim_bus;
	}

	/* Read the ID codes */
	ret = spi_flash_cmd(spi, CMD_READ_ID, idcode, sizeof(idcode));
	if (ret) {
		printf("SF: Failed to get idcodes\n");
		goto err_read_id;
	}

#ifdef DEBUG
	printf("SF: Got idcodes\n");
	print_buffer(0, idcode, 1, sizeof(idcode), 0);
#endif

	/* Validate params from spi_flash_params table */
	flash = spi_flash_validate_params(spi, idcode);
	if (!flash)
		goto err_read_id;

#ifdef CONFIG_OF_CONTROL
	if (spi_flash_decode_fdt(gd->fdt_blob, flash)) {
		debug("SF: FDT decode error\n");
		goto err_read_id;
	}
#endif
#ifndef CONFIG_SPL_BUILD
	printf("SF: Detected %s with page size ", flash->name);
	print_size(flash->page_size, ", erase size ");
	print_size(flash->erase_size, ", total ");
	print_size(flash->size, "");
	if (flash->memory_map)
		printf(", mapped at %p", flash->memory_map);
	puts("\n");
#endif
#ifndef CONFIG_SPI_FLASH_BAR
	if (flash->size > SPI_FLASH_16MB_BOUN &&
	    !spi_flash_use_4byte_mode(flash)) {
		puts("SF: Warning - Only lower 16MiB accessible,");
		puts(" Full access #define CONFIG_SPI_FLASH_BAR");
		puts("  or add set_4byte_mode\n");
	}
#endif

	/* Release spi bus */
	spi_release_bus(spi);

	return flash;

err_read_id:
	spi_release_bus(spi);
err_claim_bus:
	spi_free_slave(spi);
	return NULL;
}

void spi_flash_free(struct spi_flash *flash)
{
	spi_free_slave(flash->spi);
	free(flash);
}

#ifdef CONFIG_FLASH_SPI_DRIVER
#include <environment.h>
#include <asm/io.h>
#ifdef CONFIG_FLASH_CFI_MTD
static uint flash_verbose = 1;
#else
#define flash_verbose 1
#endif
#define SPI_MAX_FLASH_BANKS	CONFIG_SYS_MAX_FLASH_BANKS
flash_info_t flash_info[SPI_MAX_FLASH_BANKS];	/* FLASH chips info */
static struct spi_flash *spi_flashes[SPI_MAX_FLASH_BANKS];

static struct spi_flash *flash_info_to_spi_flash(flash_info_t *info)
{
	if (info >= &flash_info[0] && info < &flash_info[SPI_MAX_FLASH_BANKS])
		return spi_flashes[info - &flash_info[0]];
	printf("%s: bad flash info %p\n", __func__, info);
	return NULL;
}

#if defined(CONFIG_ENV_IS_IN_FLASH) || defined(CONFIG_ENV_ADDR_REDUND) || \
	defined(CONFIG_ENV_ADDR_REDUND_IN_FLASH) || \
	(CONFIG_SYS_MONITOR_BASE >= CONFIG_SYS_FLASH_BASE)
flash_info_t *flash_get_info(ulong base)
{
	int i;
	flash_info_t *info;

	for (i = 0; i < CONFIG_SYS_MAX_FLASH_BANKS; i++) {
		info = &flash_info[i];
		if (info->size && info->start[0] <= base &&
		    base <= info->start[0] + info->size - 1)
			return info;
	}

	return NULL;
}
#endif

unsigned long flash_sector_size(flash_info_t *info, flash_sect_t sect)
{
	if (sect != (info->sector_count - 1))
		return info->start[sect + 1] - info->start[sect];
	else
		return info->start[0] + info->size - info->start[sect];
}

int write_buff(flash_info_t *info, uchar *src, ulong addr, ulong cnt)
{
	u32 offset = addr - info->start[0];
	struct spi_flash *spifl = flash_info_to_spi_flash(info);
	return spi_flash_write(spifl, offset, cnt, src);
}

int flash_erase(flash_info_t *info, int s_first, int s_last)
{
	int rcode = 0;
	int prot;
	flash_sect_t sect;

	if (info->flash_id != FLASH_MAN_SPI) {
		puts("Can't erase unknown flash type - aborted\n");
		return 1;
	}
	if ((s_first < 0) || (s_first > s_last)) {
		puts("- no sectors to erase\n");
		return 1;
	}

	prot = 0;
	for (sect = s_first; sect <= s_last; ++sect) {
		if (info->protect[sect])
			prot++;
	}
	if (prot) {
		printf("- Warning: %d protected sectors will not be erased!\n",
		       prot);
	} else if (flash_verbose) {
		putc('\n');
	}

	for (sect = s_first; sect <= s_last; sect++) {
		if (ctrlc()) {
			printf("\n");
			return 1;
		}

		if (info->protect[sect] == 0) { /* not protected */
			struct spi_flash *spifl = flash_info_to_spi_flash(info);
			u32 offset = spifl->sector_size * sect;
			u32 len = spifl->sector_size;
			if (spi_flash_erase(spifl, offset, len)) {
				printf("Flash erase error at address %lx\n",
				       info->start[sect]);
				rcode = 1;
			}
		}
	}

	if (flash_verbose)
		puts(" done\n");

	return rcode;
}

void flash_print_info(flash_info_t *info)
{
	int i;
	struct spi_flash *spifl;

	if (info->flash_id != FLASH_MAN_SPI) {
		puts("missing or unknown FLASH type\n");
		return;
	}
	spifl = flash_info_to_spi_flash(info);
	printf("%s SPI FLASH", spifl->name);
	printf("  Size: %ld MB in %d Sectors\n",
	       info->size >> 20, info->sector_count);

	puts("\n  Sector Start Addresses:");
	for (i = 0; i < info->sector_count; ++i) {
		if (ctrlc())
			break;
		if ((i % 5) == 0)
			putc('\n');
#ifdef CONFIG_SYS_FLASH_EMPTY_INFO
		/* print empty and read-only info */
		printf("  %08lX %c %s ",
		       info->start[i],
		       sector_erased(info, i) ? 'E' : ' ',
		       info->protect[i] ? "RO" : "  ");
#else	/* ! CONFIG_SYS_FLASH_EMPTY_INFO */
		printf("  %08lX   %s ",
		       info->start[i],
		       info->protect[i] ? "RO" : "  ");
#endif
	}
	putc('\n');
}

#ifndef CONFIG_SYS_SPI_READID_HZ
#define CONFIG_SYS_SPI_READID_HZ	1000000
#endif

#ifndef CONFIG_SYS_SPI_FLASH_BANKS_LIST
#define CONFIG_SYS_SPI_FLASH_BANKS_LIST	{ \
	{CONFIG_SYS_FLASH_BASE, 0x01000000, 0, 0} }
#endif

#ifndef CONFIG_SYS_MAX_SPI_FLASH_BANKS
#define CONFIG_SYS_MAX_SPI_FLASH_BANKS	1
#endif

#define SPI_INVALID_BUS 0xffffffff
#define SPI_INVALID_CS  0xffffffff

struct spi_flash_map {
	phys_addr_t base;
	unsigned int len;
	unsigned int bus;
	unsigned int cs;
};
static struct spi_flash_map map[CONFIG_SYS_MAX_SPI_FLASH_BANKS] =
	CONFIG_SYS_SPI_FLASH_BANKS_LIST;

static unsigned int base2bus(phys_addr_t base)
{
	int i;
	for (i = 0; i < CONFIG_SYS_MAX_SPI_FLASH_BANKS; i++) {
		if (base == map[i].base)
			return map[i].bus;
	}
	return SPI_INVALID_BUS;
}

static unsigned int base2cs(phys_addr_t base)
{
	int i;
	for (i = 0; i < CONFIG_SYS_MAX_SPI_FLASH_BANKS; i++) {
		if (base == map[i].base)
			return map[i].cs;
	}
	return SPI_INVALID_CS;
}

static int flash_detect_spi(flash_info_t *info)
{
	int ret;
	phys_addr_t base = info->start[0];
	unsigned int bus = base2bus(base);
	unsigned int cs = base2cs(base);
	struct spi_flash *spifl;

	spifl = spi_flash_probe(bus, cs, CONFIG_SYS_SPI_READID_HZ, SPI_MODE_0);

	if (!spifl) {
		debug("%s : Failed to probe spi flash\n", __func__);
		return 0;
	}

	/* setup for direct access */
	ret = spi_claim_bus(spifl->spi);
	if (ret) {
		debug("SF: Failed to claim SPI bus: %d\n", ret);
		spi_free_slave(spifl->spi);
		return 0;
	}
	spi_release_bus(spifl->spi);
	spi_flashes[info - &flash_info[0]] = spifl;

	return 1;
}

static ulong flash_get_size(phys_addr_t base, int banknum)
{
	flash_info_t *info = &flash_info[banknum];
	int i;
	flash_sect_t sect_cnt;
	phys_addr_t sector;
	int erase_region_size;
	int erase_region_count;

	info->start[0] = (ulong)map_physmem(base, 4, MAP_NOCACHE);
	if (flash_detect_spi(info)) {
		struct spi_flash *spifl = flash_info_to_spi_flash(info);
		sect_cnt = 0;
		sector = base;
		erase_region_size = spifl->sector_size;
		erase_region_count = spifl->size / erase_region_size;
		debug("erase_region_count = %d erase_region_size = %d\n",
		      erase_region_count, erase_region_size);
		for (i = 0; i < erase_region_count; i++) {
			if (sect_cnt >= CONFIG_SYS_MAX_FLASH_SECT) {
				printf("ERROR: too many flash sectors\n");
				break;
			}
			info->start[sect_cnt] =
					(ulong)map_physmem(sector,
							   erase_region_size,
							   MAP_NOCACHE);
			sector += erase_region_size;
			info->protect[sect_cnt] = 0;
			sect_cnt++;
		}
		info->sector_count = sect_cnt;
		info->size = spifl->size;
		info->flash_id = FLASH_MAN_SPI;
	}
	return info->size;
}

#ifdef CONFIG_FLASH_CFI_MTD
void flash_set_verbose(uint v)
{
	flash_verbose = v;
}
#endif

static phys_addr_t spi_flash_bank_addr(int i)
{
	return map[i].base;
}

void flash_protect_default(void)
{
	/* Monitor protection ON by default */
#if (CONFIG_SYS_MONITOR_BASE >= CONFIG_SYS_FLASH_BASE) && \
	(!defined(CONFIG_MONITOR_IS_IN_RAM))
	flash_protect(FLAG_PROTECT_SET,
		      CONFIG_SYS_MONITOR_BASE,
		      CONFIG_SYS_MONITOR_BASE + monitor_flash_len  - 1,
		      flash_get_info(CONFIG_SYS_MONITOR_BASE));
#endif

	/* Environment protection ON by default */
#ifdef CONFIG_ENV_IS_IN_FLASH
#ifdef CONFIG_ENV_IS_SELECTABLE
	if (env_ops == &flash_env_ops) {
		ulong addr = CONFIG_ENV_ADDR_IN_FLASH;
		flash_protect(FLAG_PROTECT_SET,
			      addr, addr + CONFIG_ENV_SECT_SIZE - 1,
			      flash_get_info(addr));
	}
#else /* !CONFIG_ENV_IS_SELECTABLE */
	flash_protect(FLAG_PROTECT_SET,
		      CONFIG_ENV_ADDR,
		      CONFIG_ENV_ADDR + CONFIG_ENV_SECT_SIZE - 1,
		      flash_get_info(CONFIG_ENV_ADDR));
#endif /* !CONFIG_ENV_IS_SELECTABLE */
#endif /* CONFIG_ENV_IS_IN_FLASH */

	/* Redundant environment protection ON by default */
#if defined(CONFIG_ENV_IS_SELECTABLE) && \
	defined(CONFIG_ENV_ADDR_REDUND_IN_FLASH)
	if (env_ops == &flash_env_ops) {
		ulong addr = CONFIG_ENV_ADDR_REDUND_IN_FLASH;
		flash_protect(FLAG_PROTECT_SET,
			      addr, addr + CONFIG_ENV_SECT_SIZE - 1,
			      flash_get_info(addr));
	}
#endif /* CONFIG_ENV_IS_SELECTABLE */
#ifdef CONFIG_ENV_ADDR_REDUND
	flash_protect(FLAG_PROTECT_SET,
		      CONFIG_ENV_ADDR_REDUND,
		      CONFIG_ENV_ADDR_REDUND + CONFIG_ENV_SECT_SIZE - 1,
		      flash_get_info(CONFIG_ENV_ADDR_REDUND));
#endif
}

unsigned long flash_init(void)
{
	unsigned long size = 0;
	int i;

#ifdef CONFIG_SYS_FLASH_PROTECTION
	/* read environment from EEPROM */
	char s[64];
	getenv_f("unlock", s, sizeof(s));
#endif

	puts("SPI\n");
	/* Init: no FLASHes known */
	for (i = 0; i < CONFIG_SYS_MAX_FLASH_BANKS; ++i) {
		flash_info[i].flash_id = FLASH_UNKNOWN;
		flash_get_size(spi_flash_bank_addr(i), i);
		size += flash_info[i].size;
		if (flash_info[i].flash_id == FLASH_UNKNOWN) {
#ifndef CONFIG_SYS_FLASH_QUIET_TEST
			printf("## Unknown flash on Bank %d - Size = 0x%08lx = %ld MB\n",
			       i + 1, flash_info[i].size,
			       flash_info[i].size >> 20);
#endif /* CONFIG_SYS_FLASH_QUIET_TEST */
		}
#ifdef CONFIG_SYS_FLASH_PROTECTION
		else if ((s != NULL) && (strcmp(s, "yes") == 0)) {
			/*
			 * Only the U-Boot image and it's environment
			 * is protected, all other sectors are
			 * unprotected (unlocked) if flash hardware
			 * protection is used (CONFIG_SYS_FLASH_PROTECTION)
			 * and the environment variable "unlock" is
			 * set to "yes".
			 */
			flash_protect(FLAG_PROTECT_CLEAR,
				      flash_info[i].start[0],
				      flash_info[i].start[0]
				      + flash_info[i].size - 1,
				      &flash_info[i]);
		}
#endif /* CONFIG_SYS_FLASH_PROTECTION */
	}

	flash_protect_default();
#ifdef CONFIG_FLASH_CFI_MTD
	cfi_mtd_init();
#endif

	puts("Flash: "); /* then board.c print flash_size */
	return size;
}
#endif /* CONFIG_FLASH_SPI_DRIVER */
