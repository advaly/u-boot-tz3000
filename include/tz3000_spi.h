/*
 * Toshiba TZ3000 SPI definitions
 *
 * Copyright (C) 2010,2013,2014 Toshiba Corporation
 */
#ifndef _TZ3000_SPI_H_
#define _TZ3000_SPI_H_

#define TZ3000_SPIC_MEMMAP(x)		(TZ3000_SPIC_BASE + 0x4 * (x) + 0x000)
#define TZ3000_SPIC_DIRECT(x)		(TZ3000_SPIC_BASE + 0x4 * (x) + 0x008)
#define TZ3000_SPIC_RDCTRL(x)		(TZ3000_SPIC_BASE + 0x4 * (x) + 0x010)
#define TZ3000_SPIC_CONTROL0		(TZ3000_SPIC_BASE + 0x400 + 0x000)
#define TZ3000_SPIC_CONTROL1		(TZ3000_SPIC_BASE + 0x400 + 0x004)
#define TZ3000_SPIC_INT			(TZ3000_SPIC_BASE + 0x400 + 0x008)
#define TZ3000_SPIC_STATUS		(TZ3000_SPIC_BASE + 0x400 + 0x00c)

#define TZ3000_SPIC_PRIBUF		(TZ3000_SPIC_BASE + 0x400 + 0x100)
#define TZ3000_SPIC_SECBUF		(TZ3000_SPIC_BASE + 0x400 + 0x200)

#define FBA_MASK			0x07ff0000
#define FBA(addr)			((addr) & FBA_MASK)

#define FDEN(fden)			((fden) << 2)
#define FDEN_256MB			0x0c
#define FDEN_128MB			0x0b
#define FDEN_64MB			0x0a
#define FDEN_32MB			0x09
#define FDEN_16MB			0x08
#define FDEN_8MB			0x07
#define FDEN_4MB			0x06
#define FDEN_2MB			0x05
#define FDEN_1MB			0x04

#define DIR_ENABLE			0x1

#define SPR(spr)			((spr) << 16)
#define SPR_40MHZ			0x0e
#define SPR_MAX				0x1f

#define SDCE(sdce)			((sdce) << 4)
#define SDCE_POSEDGE			0x0
#define SDCE_NEGEDGE			0x1

#define SCSD(scsd)			((scsd) << 8)
#define NSEC_TO_SCSD(nsec, clk) \
	((((nsec) * 1000) / (1000000000 / ((clk) / 10)) + 99) / 100)

#endif /* _TZ3000_SPI_H_ */
