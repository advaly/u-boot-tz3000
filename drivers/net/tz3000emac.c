/*
 * Toshiba TZ3000 Ethernet controller driver
 *
 * Copyright (C) 2010,2013,2014 Toshiba Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>
#include <common.h>
#include <malloc.h>
#include <net.h>
#include <netdev.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/processor.h>
#include <miiphy.h>

/* independent from SWAP_IO_SPACE */
#ifdef EMAC_ETH_DEBUG_IO
static inline u32 emac_readl(void *addr)
{
	u32 ret = __raw_readl(addr);
	printf("EMAC_read (0x%08x) -> 0x%08x\n", (u32)addr, ret);
	return ret;
}

static inline void emac_writel(u32 d, void *addr)
{
	__raw_writel(d, addr);
	printf("EMAC_write(0x%08x) <- 0x%08x\n", (u32)addr, d);
}
#else
#define emac_readl(addr)	__raw_readl(addr)
#define emac_writel(d, addr)	__raw_writel(d, addr)
#endif

#define EMAC_NAME		"tz3000"
#define MII_IO_TIMEOUT		1000
#define EMAC_MSG_FLAG		(NETIF_MSG_TX_ERR|NETIF_MSG_DRV|NETIF_MSG_LINK)

#define RX_DESC_NUM		5
#define TX_DESC_NUM		5
#define TX_BUF_SIZE		1536
#define RX_BUF_SIZE		1536

#define ETH_FRAME_LEN		1536
/* mdiocycr: (200ns / (1000000000 / TZ3000_EMAC_BUSCLK) - 1) */
#define MDIOCYCR_SET	(((TZ3000_EMAC_BUSCLK + 5000000 - 1) / 5000000) - 1)

#define TZ3000_EMAC_MAX_WAIT	CONFIG_SYS_HZ	/* 1 sec for get_timer() */
#define TZ3000_ETH_MS_WAIT	TZ3000_EMAC_MAX_WAIT/1000
#define TRANSMIT_TIMEOUT	1 * TZ3000_EMAC_MAX_WAIT
#define AUTONEG_TIMEOUT		5 * TZ3000_EMAC_MAX_WAIT
#define READ_TIMEOUT		1 * TZ3000_EMAC_MAX_WAIT
#define FIFO_TIMEOUT		1 * TZ3000_EMAC_MAX_WAIT
#define DMA_TIMEOUT		1 * TZ3000_EMAC_MAX_WAIT

/* phy address */
#define TZ3000_PHY_ADDR	CONFIG_TZ3000_PHY_ADDR
/* Base address */
#define EMAC_IOBASE_ADDR	TZ3000_EMAC_BASE

#define NET_SKB_PAD		0
#define NET_IP_ALIGN		0

/* ERROR */
#define EREAD			1
#define EAUTONEG		2
#define EPHYRESET		3

struct emac_eth_options {
	int tx_desc_num;
	int rx_desc_num;
};

struct emac_eth_dev {
	struct tx_desc_s *tx_desc_base;
	struct tx_desc_s *tx_desc_dma_base;
	struct rx_desc_s *rx_desc_base;
	struct rx_desc_s *rx_desc_dma_base;

	unsigned int txdesc;
	unsigned int rxdesc;

	u32 *rx_buf_dma_base;
	u32 *rx_buf_base;
	u8 mac_addr[6];
	u8 phy_addr;
	struct eth_device *dev;

};

/* Register Map */
struct emac_eth_regs {
	u32 emacdrr;		/* 0x0000 */
	u32 emacdrtrgr;		/* 0x0004 */
	u32 emacdrenr;		/* 0x0008 */
	u32 reserved00[5];	/* 0x000c - 0x001F */
	u32 emacintr;		/* 0x0020 */
	u32 emacintenr;
	u32 emacintdcr;
	u32 reserved00_1[51];
	u32 emacfinfr;
	u32 reserved01[961];
	u32 dmacr;		/* 0x1000 */
	u32 reserved02[1];
	u32 dmatxr;
	u32 reserved03[1];
	u32 dmarxr;
	u32 reserved04[5];
	u32 dmaintr;
	u32 reserved05[1];
	u32 dmaintenr;
	u32 reserved06[1];
	u32 errmskr;
	u32 reserved07[1];
	u32 rxmiscntr;
	u32 reserved08[1];
	u32 txfifothr;
	u32 reserved09[1];
	u32 fifosizer;
	u32 reserved10[1];
	u32 dmarxmodr;
	u32 reserved11[1];
	u32 rxpadr;
	u32 reserved12[1];
	u32 rxfifothr;
	u32 reserved12_1[5];
	u32 txfifostsr;
	u32 reserved12_2[1];
	u32 rxfifostsr;
	u32 reserved13[29];
	u32 maccr;		/* 0x1100 */
	u32 reserved14[1];	/* 0x1104 */
	u32 lenlmtr;		/* 0x1108 */
	u32 reserved15[1];	/* 0x110c */
	u32 macintr;		/* 0x1110 */
	u32 reserved16[1];	/* 0x1114 */
	u32 macintenr;		/* 0x1118 */
	u32 reserved17[4];	/* 0x111c - 0x1128 */
	u32 phyintr;		/* phyintr: 0x112c */
	u32 reserved17_1[1];	/* 0x1130 */
	u32 rgibsr;		/* 0x1134 */
	u32 macstsr;		/* 0x1138 */
	u32 reserved17_2[1];	/* 0x113c */
	u32 phyifr;		/* 0x1140 */
	u32 reserved18_1[4];
	u32 apftpr;
	u32 mpftpr;
	u32 pftxcntr;
	u32 pfrxcntr;
	u32 pfrtlmtr;
	u32 pfrtcntr;
	u32 reserved19[17];
	u32 macgcr;		/* 0x11b0 */
	u32 bstlmtr;
	u32 reserved20[2];
	u32 umacar;
	u32 reserved21[1];
	u32 lmacar;
	u32 reserved22[77];
	u32 tint1cntr;		/* 0x1300 */
	u32 reserved23[1];
	u32 tint2cntr;
	u32 reserved24[1];
	u32 tint3cntr;
	u32 reserved25[11];
	u32 rint1cntr;
	u32 reserved26[1];
	u32 rint2cntr;
	u32 reserved27[1];
	u32 rint3cntr;
	u32 reserved28[1];
	u32 rint4cntr;
	u32 reserved29[1];
	u32 rint5cntr;
	u32 reserved30[1];
	u32 rint6cntr;
	u32 reserved31[1];
	u32 rint7cntr;
	u32 reserved32[1];
	u32 rint8cntr;
	u32 reserved33[1];
	u32 mdiostsr;
	u32 mdiocmdr;
	u32 mdioadrr;
	u32 mdiodatr;
	u32 mdiocycr;
	u32 reserved34[27];
	u32 desccr;		/* 0x1400 */
	u32 reserved35[3];	/* 0x1404 - 0x1408 */
	u32 dtxspar;
	u32 dtxcpar;
	u32 dtxlpar;
	u32 dtxdlr;
	u32 reserved36[4];
	u32 drxspar;
	u32 drxcpar;
	u32 drxlpar;
	u32 drxdlr;		/* 0x143c */
	u32 reserved37[48];	/* 0x1540 - 0x15dc */
};

#define EMAC_DRDTRG(x)		(((x) << 16) & 0x1ffc0000)
#define EMAC_DRDMSK(x)		((x) & 0x00001ffc)

#define EMACDRENR_DRDEN		0x00000001
#define EMACDRTRGR_DFL		EMAC_DRDTRG(TZ3000_EMAC_BASE + 0x1028)

#define EMACINTR_SERR		0x00000002
#define EMACINTR_DMAINT		0x00000001

#define DMACR_DE		0x00000040
#define DMACR_DL_16		0x00000030
#define DMACR_DL_32		0x00000010
#define DMACR_DL_64		0x00000020
#define DMACR_SWRT		0x00000002
#define DMACR_SWRR		0x00000001

#define DMATXR_START		0x00000003
#define DMATXR_STOP		0x00000000

#define DMARXR_START		0x00000001
#define DMARXR_STOP		0x00000000

#define DMAINTR_ALL		0xff3f07ff
#define DMAINTR_WB1		0x80000000
#define DMAINTR_WB0		0x40000000
#define DMAINTR_FTC1		0x20000000
#define DMAINTR_TUC		0x10000000
#define DMAINTR_ROC		0x08000000
#define DMAINTR_TABT		0x04000000
#define DMAINTR_RABT		0x02000000
#define DMAINTR_RFRMER		0x01000000
#define DMAINTR_MINT		0x00400000
#define DMAINTR_FTC0		0x00200000
#define DMAINTR_TDE		0x00100000
#define DMAINTR_TFE		0x00080000
#define DMAINTR_FRC		0x00040000
#define DMAINTR_RDE		0x00020000
#define DMAINTR_RFE		0x00010000
#define DMAINTR_TINT3		0x00000400
#define DMAINTR_TINT2		0x00000200
#define DMAINTR_TINT1		0x00000100
#define DMAINTR_RINT8		0x00000080
#define DMAINTR_RINT7		0x00000040
#define DMAINTR_RINT6		0x00000020
#define DMAINTR_RINT5		0x00000010
#define DMAINTR_RINT4		0x00000008
#define DMAINTR_RINT3		0x00000004
#define DMAINTR_RINT2		0x00000002
#define DMAINTR_RINT1		0x00000001
#define DMAINTR_ERR		(DMAINTR_RFRMER | DMAINTR_TDE | DMAINTR_TFE | \
				 DMAINTR_RDE|DMAINTR_RFE)

#define TXFIFOTHR_SF		0x00000000

#define FIFOSIZER_DFL		CONFIG_TZ3000_EMAC_FIFOSIZER

#define DMARXMODR_RCVMOD	0x00000001

#define RXPADR_PADS(x)		(((x) << 16) & 0x001f0000)
#define RXPADR_PADP(x)		((x) & 0x0000ffff)

#define RXFIFOTHR_RFF(x)	(((x) << 16) & 0x001f0000)
#define RXFIFOTHR_RFD(x)	((x) >> 8)

#define RXFIFOSTSR_FIFO_MSK	0x000F0000
#define TXFIFOSTSR_FIFO_MSK	0x740701FF

#define MACCR_TRCCM		0x04000000
#define MACCR_RCPT		0x02000000
#define MACCR_TCPT		0x01000000
#define MACCR_RCSC		0x00800000
#define MACCR_DPAD		0x00200000
#define MACCR_RZPF		0x00100000
#define MACCR_TZPF		0x00080000
#define MACCR_PFR		0x00040000
#define MACCR_RXF		0x00020000
#define MACCR_TXF		0x00010000
#define MACCR_RPE		0x00000040
#define MACCR_TPE		0x00000020
#define MACCR_DPM		0x00000002
#define MACCR_PRM		0x00000001

#define LENLMTR_LENLMT(x)	((x) & 0x0003ffff)

#define MACINTR_ALL		0x00000011
#define MACINTR_PFRI		0x00000010
#define MACINTR_PHYI		0x00000008
#define MACINTR_FCI		0x00000001

#define PHYINTR_PHYIP_HIGH	0x00000001
#define PHYINTR_PHYIP_LOW	0x00000000

#define PFRTLMTR_UNLIMIT	0x00000000

#define MACGCR_SPEED_MSK	0x00000030
#define MACGCR_SPEED_1000	0x00000020
#define MACGCR_SPEED_100	0x00000010
#define MACGCR_SPEED_10		0x00000000

#define MDIOSTSR_BSY		0x00000001

#define MDIOCMDR_READ		0x00000002
#define MDIOCMDR_WRITE		0x00000001

#define MDIOADRR_PHYADR(x)	(((x) << 5) & 0x000003e0)
#define MDIOADRR_PHYREG(x)	((x) & 0x0000001f)

#define DESCCR_ENT		0x00000002
#define DESCCR_ENR		0x00000001

#define DTXDLR_DTXDL		0x00000001

#define DRXDLR_DRXDL		0x00000001

/* Tx descripter We aloways use 3 bytes of padding */
struct tx_desc_s {
	u32 td0;
	u32 td1;
	u32 td2;
	u32 dummy;
};				/* desc size = 32byte */

struct rx_desc_s {
	u32 rd0;
	u32 rd1;
	u32 rd2;
	u32 dummy;
};

#define TD0_TACT		0x80000000
#define TD0_TDL			0x40000000
#define TD0_TFP_MASK		0x30000000
#define TD0_TFP_MIDDLE		0x00000000
#define TD0_TFP_TAIL		0x10000000
#define TD0_TFP_HEAD		0x20000000
#define TD0_TFP_ALL		0x30000000
#define TD0_TFE			0x08000000
#define TD0_TWBI		0x04000000
#define TD0_TFS_MASK		0x000003ff
#define TD0_TFS_TUC		0x00000200
#define TD0_TFS_TABT		0x00000100
#define TD0_TFS_TINT3		0x00000004
#define TD0_TFS_TINT2		0x00000002
#define TD0_TFS_TINT1		0x00000001

#define TD1_TBL_MASK		0xffff0000
#define TD1_TBL_SET(x)		(((x) << 16) & TD1_TBL_MASK)

#define TD2_TBA_SET(x)		(x)

#define RD0_RACT		0x80000000
#define RD0_RDL			0x40000000
#define RD0_RFP_MASK		0x30000000
#define RD0_RFP_MIDDLE		0x00000000
#define RD0_RFP_RAIL		0x10000000
#define RD0_RFP_HEAD		0x20000000
#define RD0_RFP_ALL		0x30000000
#define RD0_RFE			0x08000000
#define RD0_PV			0x04000000
#define RD0_RFS_MASK		0x000003ff
#define RD0_RFS_ROC		0x00000200
#define RD0_RFS_RABT		0x00000100
#define RD0_RFS_RINT8		0x00000080
#define RD0_RFS_RINT7		0x00000040
#define RD0_RFS_RINT6		0x00000020
#define RD0_RFS_RINT5		0x00000010
#define RD0_RFS_RINT4		0x00000008
#define RD0_RFS_RINT3		0x00000004
#define RD0_RFS_RINT2		0x00000002
#define RD0_RFS_RINT1		0x00000001
#define RD0_RFS_ERR		(RD0_RFS_MASK & ~RD0_RFS_RINT8)

#define RD1_RBL_MASK		0xffff0000
#define RD1_RBL_SET(x)		(((x) << 16) & RD1_RBL_MASK)

#define RD1_RFL_MASK		0x0000ffff
#define RD1_RFL_GET(x)		((x) & RD1_RFL_MASK)

#define RD2_RBA_SET(x)		(x)

/* PHY REG ADDR */
#define TZ3000_PHY_BMCR			0x00
#define TZ3000_PHY_BMSR			0x01
#define TZ3000_PHY_PHYID1		0x02
#define TZ3000_PHY_PHYID2		0x03
#define TZ3000_PHY_ANAR			0x04
#define TZ3000_PHY_ANLPAR		0x05
#define TZ3000_PHY_ANER			0x06
#define TZ3000_PHY_ANNPTR		0x07
#define TZ3000_PHY_ANNPRR		0x08
#define TZ3000_PHY_GBCR			0x09
#define TZ3000_PHY_GBSR			0x0a
#define TZ3000_PHY_GBESR		0x0f

/* TZ3000_PHY_CTRL */
#define TZ3000_PHY_BMCR_SWR		0x8000
#define TZ3000_PHY_BMCR_LOOPBK		0x4000
#define TZ3000_PHY_BMCR_ANEGE		0x1000
#define TZ3000_PHY_BMCR_POWDN		0x0800
#define TZ3000_PHY_BMCR_ISO		0x0400
#define TZ3000_PHY_BMCR_REANEG		0x0200
#define TZ3000_PHY_BMCR_DUPLEX		0x0100
#define TZ3000_PHY_BMCR_COLT		0x0080

#define TZ3000_PHY_BMCR_SPEED_1000	0x0040
#define TZ3000_PHY_BMCR_SPEED_100	0x2000
#define TZ3000_PHY_BMCR_SPEED_10	0x0000

/* TZ3000_PHY_STATUS */
#define TZ3000_PHY_BMSR_100FDX		0x4000
#define TZ3000_PHY_BMSR_100HDX		0x2000
#define TZ3000_PHY_BMSR_10FDX		0x1000
#define TZ3000_PHY_BMSR_10HDX		0x0800
#define TZ3000_PHY_BMSR_ANC		0x0020
#define TZ3000_PHY_BMSR_LINK		0x0004

/* TZ3000_PHY_ANA*/
#define TZ3000_PHY_ANAR_SET		0x01E1	/* 0b0000_0001_1110_0001 */

/* TZ3000_PHY_ANLPAR */
#define TZ3000_PHY_ANLPAR_100B		0x0380
#define TZ3000_PHY_ANLPAR_10B		0x0060
#define TZ3000_PHY_ANLPAR_TXFD		0x0140
#define TZ3000_PHY_ANLPAR_TXHD		0x00a0

/* PLL CTRL */
#define PLL_CTRL		(TZ3000_PMU_BASE + 0x4520)

static void *rx_buf_malloc;
static void *desc_malloc_addr;

static int emac_eth_config(struct eth_device *dev);
static void emac_eth_halt(struct eth_device *dev);
static int emac_eth_reset(struct emac_eth_dev *eth);
static int emac_eth_stop(struct emac_eth_dev *eth);
static int emac_eth_desc_init(struct emac_eth_dev *eth);
static int emac_eth_init(struct eth_device *dev, bd_t *bd);
static int emac_eth_send(struct eth_device *dev, void *packet, int len);
static int emac_eth_recv(struct eth_device *dev);
static int emac_eth_mii_read(const char *devname, unsigned char addr,
			     unsigned char reg, unsigned short *value);
static int emac_eth_mii_write(const char *devname, unsigned char addr,
			      unsigned char reg, unsigned short value);
#ifdef CONFIG_TZ3000_ETH_AUTONEG
static void emac_eth_sec_wait_done(int i);
static int emac_eth_phy_config(struct eth_device *dev);
#endif
static void emac_eth_ms_wait_done(int i);

#ifdef DEBUG
/* register Dump function */
static void emac_reg_dump(struct emac_eth_dev *eth)
{
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;
	u32 stat;

	/* emac */
	stat = emac_readl(&tr->emacdrr);
	printf("emacdrr:     %p  = 0x%08x\n", &tr->emacdrr, stat);
	stat = emac_readl(&tr->emacdrtrgr);
	printf("emacdrtrgr:  %p  = 0x%08x\n", &tr->emacdrtrgr, stat);
	stat = emac_readl(&tr->emacdrenr);
	printf("emacdrenr:   %p  = 0x%08x\n", &tr->emacdrenr, stat);
	stat = emac_readl(&tr->emacintr);
	printf("emaintr:     %p  = 0x%08x\n", &tr->emacintr, stat);
	stat = emac_readl(&tr->emacintenr);
	printf("emaintenr:   %p  = 0x%08x\n", &tr->emacintenr, stat);

	/* dma */
	stat = emac_readl(&tr->dmacr);
	printf("dmacr:       %p  = 0x%08x\n", &tr->dmacr, stat);
	stat = emac_readl(&tr->dmatxr);
	printf("dmatxr:      %p  = 0x%08x\n", &tr->dmatxr, stat);
	stat = emac_readl(&tr->dmarxr);
	printf("dmarxr:      %p  = 0x%08x\n", &tr->dmarxr, stat);
	stat = emac_readl(&tr->dmaintr);
	printf("dmaintr:     %p  = 0x%08x\n", &tr->dmaintr, stat);
	stat = emac_readl(&tr->dmaintenr);
	printf("dmaintenr:   %p  = 0x%08x\n", &tr->dmaintenr, stat);
	stat = emac_readl(&tr->errmskr);
	printf("errmskr:    %p  = 0x%08x\n", &tr->errmskr, stat);

	/* rx */
	stat = emac_readl(&tr->rxmiscntr);
	printf("rxmiscntr:   %p  = 0x%08x\n", &tr->rxmiscntr, stat);
	stat = emac_readl(&tr->txfifothr);
	printf("txfifothr:   %p  = 0x%08x\n", &tr->txfifothr, stat);
	stat = emac_readl(&tr->fifosizer);
	printf("fifosizer:   %p  = 0x%08x\n", &tr->fifosizer, stat);
	stat = emac_readl(&tr->dmarxmodr);
	printf("dmarxmodr:   %p  = 0x%08x\n", &tr->dmarxmodr, stat);
	stat = emac_readl(&tr->rxpadr);
	printf("rxpadr:      %p  = 0x%08x\n", &tr->rxpadr, stat);
	stat = emac_readl(&tr->rxfifothr);
	printf("rxfifothr:   %p  = 0x%08x\n", &tr->rxfifothr, stat);
	stat = emac_readl(&tr->txfifostsr);
	printf("txfifostsr:   %p  = 0x%08x\n", &tr->txfifostsr, stat);
	stat = emac_readl(&tr->rxfifostsr);
	printf("rxfifostsr:   %p  = 0x%08x\n", &tr->rxfifostsr, stat);

	stat = emac_readl(&tr->maccr);
	printf("maccr:       %p  = 0x%08x\n", &tr->maccr, stat);
	stat = emac_readl(&tr->lenlmtr);
	printf("lenlmtr:     %p  = 0x%08x\n", &tr->lenlmtr, stat);
	stat = emac_readl(&tr->macintr);
	printf("macintr:     %p  = 0x%08x\n", &tr->macintr, stat);
	stat = emac_readl(&tr->macintenr);
	printf("macintenr:   %p  = 0x%08x\n", &tr->macintenr, stat);
	stat = emac_readl(&tr->phyintr);
	printf("phyintr:     %p  = 0x%08x\n", &tr->phyintr, stat);
	stat = emac_readl(&tr->phyifr);
	printf("phyifr:     %p  = 0x%08x\n", &tr->phyifr, stat);
	stat = emac_readl(&tr->apftpr);
	printf("apftpr:      %p  = 0x%08x\n", &tr->apftpr, stat);

	stat = emac_readl(&tr->mpftpr);
	printf("mpftpr:      %p  = 0x%08x\n", &tr->mpftpr, stat);
	stat = emac_readl(&tr->pftxcntr);
	printf("pftxcntr:    %p  = 0x%08x\n", &tr->pftxcntr, stat);
	stat = emac_readl(&tr->pfrxcntr);
	printf("pfrxcntr:    %p  = 0x%08x\n", &tr->pfrxcntr, stat);
	stat = emac_readl(&tr->pfrtlmtr);
	printf("pfrtmltr:    %p  = 0x%08x\n", &tr->pfrtlmtr, stat);
	stat = emac_readl(&tr->pfrtcntr);
	printf("pfrtcntr:    %p  = 0x%08x\n", &tr->pfrtcntr, stat);
	stat = emac_readl(&tr->macgcr);
	printf("macgcr:      %p  = 0x%08x\n", &tr->macgcr, stat);
	stat = emac_readl(&tr->bstlmtr);
	printf("bstlmtr:     %p  = 0x%08x\n", &tr->bstlmtr, stat);
	stat = emac_readl(&tr->umacar);
	printf("umacar:      %p  = 0x%08x\n", &tr->umacar, stat);
	stat = emac_readl(&tr->lmacar);
	printf("lmacar:      %p  = 0x%08x\n", &tr->lmacar, stat);

	stat = emac_readl(&tr->tint1cntr);
	printf("tint1cntr:   %p  = 0x%08x\n", &tr->tint1cntr, stat);
	stat = emac_readl(&tr->tint2cntr);
	printf("tint2cntr:   %p  = 0x%08x\n", &tr->tint2cntr, stat);
	stat = emac_readl(&tr->tint3cntr);
	printf("tint3cntr:   %p  = 0x%08x\n", &tr->tint3cntr, stat);
	stat = emac_readl(&tr->rint1cntr);
	printf("rint1cntr:   %p  = 0x%08x\n", &tr->rint1cntr, stat);
	stat = emac_readl(&tr->rint2cntr);
	printf("rint2cntr:   %p  = 0x%08x\n", &tr->rint2cntr, stat);
	stat = emac_readl(&tr->rint3cntr);
	printf("rint3cntr:   %p  = 0x%08x\n", &tr->rint3cntr, stat);
	stat = emac_readl(&tr->rint4cntr);
	printf("rint4cntr:   %p  = 0x%08x\n", &tr->rint4cntr, stat);
	stat = emac_readl(&tr->rint5cntr);
	printf("rint5cntr:   %p  = 0x%08x\n", &tr->rint5cntr, stat);
	stat = emac_readl(&tr->rint6cntr);
	printf("rint6cntr:   %p  = 0x%08x\n", &tr->rint6cntr, stat);
	stat = emac_readl(&tr->rint7cntr);
	printf("rint7cntr:   %p  = 0x%08x\n", &tr->rint7cntr, stat);
	stat = emac_readl(&tr->rint8cntr);
	printf("rint8cntr:   %p  = 0x%08x\n", &tr->rint8cntr, stat);

	stat = emac_readl(&tr->mdiostsr);
	printf("mdiostsr:    %p  = 0x%08x\n", &tr->mdiostsr, stat);
	stat = emac_readl(&tr->mdiocmdr);
	printf("mdiocmdr:    %p  = 0x%08x\n", &tr->mdiocmdr, stat);
	stat = emac_readl(&tr->mdioadrr);
	printf("mdioadrr:    %p  = 0x%08x\n", &tr->mdioadrr, stat);
	stat = emac_readl(&tr->mdiodatr);
	printf("mdiodatr:    %p  = 0x%08x\n", &tr->mdiodatr, stat);
	stat = emac_readl(&tr->mdiocycr);
	printf("mdiocycr:    %p  = 0x%08x\n", &tr->mdiocycr, stat);
	stat = emac_readl(&tr->desccr);
	printf("desccr:      %p  = 0x%08x\n", &tr->desccr, stat);

	stat = emac_readl(&tr->dtxspar);
	printf("dtxspar:     %p  = 0x%08x\n", &tr->dtxspar, stat);
	stat = emac_readl(&tr->dtxcpar);
	printf("dtxcpar:     %p  = 0x%08x\n", &tr->dtxcpar, stat);
	stat = emac_readl(&tr->dtxlpar);
	printf("dtxlpar:     %p  = 0x%08x\n", &tr->dtxlpar, stat);
	stat = emac_readl(&tr->dtxdlr);
	printf("dtxdlr:      %p  = 0x%08x\n", &tr->dtxdlr, stat);

	stat = emac_readl(&tr->drxspar);
	printf("drxspar:     %p  = 0x%08x\n", &tr->drxspar, stat);
	stat = emac_readl(&tr->drxcpar);
	printf("drxcpar:     %p  = 0x%08x\n", &tr->drxcpar, stat);
	stat = emac_readl(&tr->drxlpar);
	printf("drxlpar:     %p  = 0x%08x\n", &tr->drxlpar, stat);
	stat = emac_readl(&tr->drxdlr);
	printf("drxdlr:      %p  = 0x%08x\n", &tr->drxdlr, stat);
}
#endif

static void emac_eth_ms_wait_done(int i)
{
	unsigned long timebase = get_timer(0);

	while (get_timer(timebase) < (TZ3000_ETH_MS_WAIT * i))
		;
}

#ifdef CONFIG_TZ3000_ETH_AUTONEG
static void emac_eth_sec_wait_done(int i)
{
	unsigned long timebase = get_timer(0);

	while (get_timer(timebase) < (TZ3000_EMAC_MAX_WAIT * i))
		;
}
#endif

static int emac_eth_mii_write(const char *devname, unsigned char addr,
			      unsigned char reg, unsigned short value)
{
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;
	unsigned long timeout;

	timeout = get_timer(0);
	while (emac_readl(&tr->mdiostsr) & MDIOSTSR_BSY) {
		if (get_timer(timeout) > MII_IO_TIMEOUT) {
			printf(EMAC_NAME ": %s: timeout\n", __func__);
			return -EIO;
		}
	}

	emac_writel(MDIOADRR_PHYADR(addr) | MDIOADRR_PHYREG(reg),
		    &tr->mdioadrr);
	emac_writel((u32) value, &tr->mdiodatr);
	emac_writel(MDIOCMDR_WRITE, &tr->mdiocmdr);

	timeout = get_timer(0);
	while (emac_readl(&tr->mdiostsr) & MDIOSTSR_BSY) {
		if (get_timer(timeout) > MII_IO_TIMEOUT) {
			printf(EMAC_NAME ": %s: timeout\n", __func__);
			return -EIO;
		}
	}
#ifdef CONFIG_TZ3000_RGMII
	/* workaround for RTL8211EG errata */
	{
		unsigned short dummy;
		emac_eth_mii_read(devname, addr, TZ3000_PHY_BMSR, &dummy);
	}
#endif
	return 0;
}

static int emac_eth_mii_read(const char *devname, unsigned char addr,
			      unsigned char reg, unsigned short *value)
{
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;
	unsigned long timeout;

	timeout = get_timer(0);
	while (emac_readl(&tr->mdiostsr) & MDIOSTSR_BSY) {
		if (get_timer(timeout) > MII_IO_TIMEOUT) {
			printf(EMAC_NAME ": %s: timeout\n", __func__);
			return -EIO;
		}
	}

	emac_writel(MDIOADRR_PHYADR(addr) | MDIOADRR_PHYREG(reg),
		    &tr->mdioadrr);
	emac_writel(MDIOCMDR_READ, &tr->mdiocmdr);

	timeout = get_timer(0);
	while (emac_readl(&tr->mdiostsr) & MDIOSTSR_BSY) {
		if (get_timer(timeout) > MII_IO_TIMEOUT) {
			printf(EMAC_NAME ": %s: timeout\n", __func__);
			return -EIO;
		}
	}
	*value = (unsigned short)(emac_readl(&tr->mdiodatr) & 0xffff);
	return 0;
}

/* call from Net Loop function */
static int emac_eth_init(struct eth_device *dev, bd_t *bd)
{
	struct emac_eth_dev *eth = dev->priv;
	int ret;
	u8 *data = dev->enetaddr;
	if (data == NULL) {
		printf("Please set \"ethaddr\"\n");
		return -EINVAL;
	}
	printf(EMAC_NAME ": MAC  %pM\n", data);

	ret = emac_eth_reset(eth);
	if (ret < 0)
		return ret;
	ret = emac_eth_desc_init(eth);
	if (ret < 0)
		return ret;

	ret = emac_eth_config(dev);
	if (ret < 0)
		return ret;
#ifdef DEBUG
	emac_reg_dump(eth);
#endif
	return 0;
}

static int emac_eth_config(struct eth_device *dev)
{
	u32 reg, uaddr, laddr;
#ifdef CONFIG_TZ3000_ETH_AUTONEG
	u16 lpar, adv;
#endif
	u16 bmcr;
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;
	int ret;

	/* bus block register set */
	emac_writel(CONFIG_TZ3000_EMAC_EMACDRR, &tr->emacdrr);
	emac_writel(EMACDRTRGR_DFL, &tr->emacdrtrgr);
	emac_writel(EMACDRENR_DRDEN, &tr->emacdrenr);
	emac_writel(0x0, &tr->emacintr);
	emac_writel(0x0, &tr->emacintenr);
	emac_writel(0x0, &tr->emacintdcr);

	/* DMA block register set */
	emac_writel(DMACR_DE | DMACR_DL_16, &tr->dmacr);
	emac_writel(DMAINTR_ALL, &tr->dmaintr);	/* Write Clear */
	emac_writel(0x0, &tr->dmaintenr);
	emac_writel(DMAINTR_RINT8, &tr->errmskr);
	emac_writel(TXFIFOTHR_SF, &tr->txfifothr);
	emac_writel(FIFOSIZER_DFL, &tr->fifosizer);
	emac_writel(DMARXMODR_RCVMOD, &tr->dmarxmodr);
	/* No padding */
	emac_writel(RXPADR_PADS(NET_IP_ALIGN) | RXPADR_PADP(0), &tr->rxpadr);

	/* MAC block set */
	emac_writel(LENLMTR_LENLMT(1518), &tr->lenlmtr);
	emac_writel(MACINTR_ALL, &tr->macintr);
	emac_writel(0x0, &tr->macintenr);
	uaddr = dev->enetaddr[0] << 24 | dev->enetaddr[1] << 16 |
		dev->enetaddr[2] << 8 | dev->enetaddr[3];
	laddr = dev->enetaddr[4] << 8 | dev->enetaddr[5];
	emac_writel(uaddr, &tr->umacar);
	emac_writel(laddr, &tr->lmacar);

#ifdef CONFIG_TZ3000_ETH_AUTONEG
	/* PHY Init */
	ret = emac_eth_phy_config(dev);
	if (ret == -EREAD || ret == -EPHYRESET) {
		return ret;
	} else if (ret == -EAUTONEG) {
		/* Auto-Negotiation Err */
#endif
		ret = emac_eth_mii_read(dev->name, TZ3000_PHY_ADDR,
					TZ3000_PHY_BMCR, &bmcr);
		if (ret < 0) {
			printf("MII reg read Error\n");
			return ret;
		}

#ifdef CONFIG_TZ3000_ETH_AUTONEG
		if (bmcr & (TZ3000_PHY_BMCR_SPEED_100)) {
#endif
			printf(EMAC_NAME ": 100Base/");
			emac_writel(MACGCR_SPEED_100, &tr->macgcr);
#ifdef CONFIG_TZ3000_ETH_AUTONEG
		} else {
			printf(EMAC_NAME ": 10Base/");
			emac_writel(MACGCR_SPEED_10, &tr->macgcr);
		}
#endif

		reg = emac_readl(&tr->maccr);
#ifdef CONFIG_TZ3000_ETH_AUTONEG
		if (bmcr & (TZ3000_PHY_BMCR_DUPLEX)) {
#endif
			printf("Full\n");
			reg |= MACCR_DPM | MACCR_TRCCM | MACCR_RPE | MACCR_TPE;
#ifdef CONFIG_TZ3000_ETH_AUTONEG
		} else {
			printf("Half\n");
			reg |= MACCR_TRCCM | MACCR_RPE | MACCR_TPE;
		}
#endif
		/* MAC block start */
		emac_writel(reg, &tr->maccr);
		/* Rx DMA start */
		emac_writel(DMARXR_START, &tr->dmarxr);

#ifdef CONFIG_TZ3000_ETH_AUTONEG
	} else {
		/* Auto-Negotiation Completed */
		ret = emac_eth_mii_read(dev->name, TZ3000_PHY_ADDR,
					TZ3000_PHY_ANLPAR, &lpar);
		if (ret < 0) {
			printf("MII reg read Error\n");
			return ret;
		}

		ret = emac_eth_mii_read(dev->name, TZ3000_PHY_ADDR,
					TZ3000_PHY_ANAR, &adv);
		if (ret < 0) {
			printf("MII reg read Error\n");
			return ret;
		}

		lpar &= adv;

		if (lpar & TZ3000_PHY_ANLPAR_100B) {
			printf(EMAC_NAME ": 100Base/");
			emac_writel(MACGCR_SPEED_100, &tr->macgcr);
		} else {
			printf(EMAC_NAME ": 10Base/");
			emac_writel(MACGCR_SPEED_10, &tr->macgcr);
		}

		reg = emac_readl(&tr->maccr);
		if (lpar & TZ3000_PHY_ANLPAR_TXFD) {
			printf("Full\n");
			reg |= MACCR_DPM | MACCR_TRCCM | MACCR_RPE | MACCR_TPE;
		} else {
			printf("Half\n");
			reg |= MACCR_TRCCM | MACCR_RPE | MACCR_TPE;
		}

		/* MAC block start */
		emac_writel(reg, &tr->maccr);
		/* Rx DMA start */
		emac_writel(DMARXR_START, &tr->dmarxr);
	}
#endif
	return 0;
}

#ifdef CONFIG_TZ3000_ETH_AUTONEG
static int emac_eth_phy_config(struct eth_device *dev)
{
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;
	unsigned long timeout;
	int ret;
	u16 reg;

	/* PHY init set-> 200ns */
	emac_writel(MDIOCYCR_SET, &tr->mdiocycr);

	/* PHY Software Reset */
	emac_eth_mii_write(dev->name, TZ3000_PHY_ADDR, TZ3000_PHY_BMCR,
			   TZ3000_PHY_BMCR_SWR);
	udelay(30000);		/* must wait 30ms after setting this bit */

	ret = emac_eth_mii_read(dev->name, TZ3000_PHY_ADDR, TZ3000_PHY_BMCR,
				&reg);
	if (ret < 0) {
		printf(EMAC_NAME " :MII reg read Error\n");
		return -EREAD;
	}
	if (reg & TZ3000_PHY_BMCR_SWR) {
		printf(EMAC_NAME ":phy reset timeout\n");
		return -EPHYRESET;
	}

	/* 100M-F/HDX, 10M-F/HDX, advertise selector */
	emac_eth_mii_write(dev->name, TZ3000_PHY_ADDR, TZ3000_PHY_ANAR,
			   TZ3000_PHY_ANAR_SET);
	emac_eth_mii_write(dev->name, TZ3000_PHY_ADDR, TZ3000_PHY_GBCR,
			   0x0000);
	emac_eth_mii_read(dev->name, TZ3000_PHY_ADDR, TZ3000_PHY_BMCR, &reg);

	/* Reset Auto-Negotiation */
	emac_eth_mii_write(dev->name, TZ3000_PHY_ADDR, TZ3000_PHY_BMCR,
			   reg | TZ3000_PHY_BMCR_ANEGE |
			   TZ3000_PHY_BMCR_REANEG);

	emac_eth_sec_wait_done(4);

	timeout = get_timer(0);
	do {
		if (get_timer(timeout) > AUTONEG_TIMEOUT) {
			printf(EMAC_NAME ": Autonegotiation time out\n");
			emac_eth_mii_write(dev->name, TZ3000_PHY_ADDR,
					   TZ3000_PHY_BMCR,
					   (TZ3000_PHY_BMCR_SPEED_100 |
					    TZ3000_PHY_BMCR_DUPLEX));
			return -EAUTONEG;
		}

		if ((emac_eth_mii_read(dev->name, TZ3000_PHY_ADDR,
				       TZ3000_PHY_BMSR, &reg)) != 0)
			return -EREAD;
		/* Autoneg complete */
	} while (!(reg & (TZ3000_PHY_BMSR_ANC | TZ3000_PHY_BMSR_LINK)));

	printf(EMAC_NAME ": phy initialized\n");

	return 0;
}
#endif

static int emac_eth_stop(struct emac_eth_dev *eth)
{
	u32 reg, speed;
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;
	unsigned long timeout;

	/* MAC block recieve stop */
	reg = emac_readl(&tr->maccr);
	reg &= ~MACCR_RPE;
	emac_writel(reg, &tr->maccr);
	emac_writel(DMATXR_STOP, &tr->dmatxr);

	speed = (emac_readl(&tr->macgcr) & MACGCR_SPEED_MSK);

	switch (speed) {
	case MACGCR_SPEED_100:	/* 100Mbps */
		udelay(200);
		break;

	case MACGCR_SPEED_10:	/* 10Mbps */
	default:
		emac_eth_ms_wait_done(2);
		break;
	}

	timeout = get_timer(0);
	do {
		if (get_timer(timeout) > FIFO_TIMEOUT) {
			printf(EMAC_NAME ": %s: rxfifo timeout\n", __func__);
			return -ETIME;
		}
		reg = emac_readl(&tr->rxfifostsr);
		/* waits  RX FIFO empty */
	} while ((reg & RXFIFOSTSR_FIFO_MSK) != 0);

	timeout = get_timer(0);
	do {
		if (get_timer(timeout) > DMA_TIMEOUT) {
			printf(EMAC_NAME ": %s: dmatxr timeout\n", __func__);
			return -ETIME;
		}
		reg = emac_readl(&tr->dmatxr);
		/* wait be TRNS -> 0x00 */
	} while ((reg & ~DMATXR_START) != 0);

	timeout = get_timer(0);
	do {
		if (get_timer(timeout) > FIFO_TIMEOUT) {
			printf(EMAC_NAME ": %s: txfifo timeout\n", __func__);
			return -ETIME;
		}
		reg = emac_readl(&tr->rxfifostsr);
		/* wait TX FIFO empty */
	} while ((reg & TXFIFOSTSR_FIFO_MSK) != 0);

	reg = emac_readl(&tr->maccr) & MACCR_DPM;

	if (reg != 0) {		/* Full */
		switch (speed) {
		case MACGCR_SPEED_100:	/* 100Mbps */
			udelay(200);
			break;

		case MACGCR_SPEED_10:	/* 10Mbps */
		default:
			emac_eth_ms_wait_done(2);
			break;
		}
	} else {		/* Half */
		switch (speed) {
		case MACGCR_SPEED_100:	/* 100Mbps */
			emac_eth_ms_wait_done(38);
			break;

		case MACGCR_SPEED_10:	/* 10Mbps */
		default:
			emac_eth_ms_wait_done(370);
			break;
		}
	}
	reg = emac_readl(&tr->maccr);
	emac_writel(reg & ~MACCR_TPE, &tr->maccr);	/* stop TX MAC block */

#ifdef DEBUG
	printf("reset------------->\n");
	emac_reg_dump(eth);
#endif

	/* reset */
	emac_writel(0x00, &tr->dmaintenr);
	emac_writel(0x00, &tr->macintenr);
	emac_writel(DMACR_SWRR | DMACR_SWRT, &tr->dmacr);
	return 0;
}

static int emac_eth_reset(struct emac_eth_dev *eth)
{
	int timeout;
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;

	/* reset */
	emac_writel(DESCCR_ENR | DESCCR_ENT, &tr->desccr);
	emac_writel(DMACR_SWRR | DMACR_SWRT, &tr->dmacr);

	timeout = 1000;
	while (--timeout) {
		/* reset off check */
		if (!(emac_readl(&tr->dmacr) & (DMACR_SWRR | DMACR_SWRT)))
			break;
		udelay(1);
	}

	if (!timeout) {
		printf(EMAC_NAME ": Controller soft reset failed\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void flush_txdesc(struct tx_desc_s *tx_desc)
{
	unsigned long start = (unsigned long)tx_desc;
	unsigned long end = start + sizeof(struct tx_desc_s);
	flush_dcache_range(start & ~(ARCH_DMA_MINALIGN - 1),
			   roundup(end, ARCH_DMA_MINALIGN));
}
static void invalidate_txdesc(struct tx_desc_s *tx_desc)
{
	unsigned long start = (unsigned long)tx_desc;
	unsigned long end = start + sizeof(struct tx_desc_s);
	invalidate_dcache_range(start & ~(ARCH_DMA_MINALIGN - 1),
				roundup(end, ARCH_DMA_MINALIGN));
}
static void flush_rxdesc(struct rx_desc_s *rx_desc)
{
	unsigned long start = (unsigned long)rx_desc;
	unsigned long end = start + sizeof(struct rx_desc_s);
	flush_dcache_range(start & ~(ARCH_DMA_MINALIGN - 1),
			   roundup(end, ARCH_DMA_MINALIGN));
}
static void invalidate_rxdesc(struct rx_desc_s *rx_desc)
{
	unsigned long start = (unsigned long)rx_desc;
	unsigned long end = start + sizeof(struct rx_desc_s);
	invalidate_dcache_range(start & ~(ARCH_DMA_MINALIGN - 1),
				roundup(end, ARCH_DMA_MINALIGN));
}

static int emac_eth_desc_init(struct emac_eth_dev *eth)
{
	int i;
	void *rx_buf;
	void *desc_malloc;
	size_t desc_size;
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;
	unsigned long desc_addr, desc_dma_addr;

	/* receive buffer entry */
	if (rx_buf_malloc) {
		free(rx_buf_malloc);
		rx_buf_malloc = NULL;
	}
	rx_buf_malloc = malloc(RX_BUF_SIZE * RX_DESC_NUM +
			       CONFIG_SYS_CACHELINE_SIZE);
	if (!rx_buf_malloc) {
		printf(EMAC_NAME ": %s:  buf_malloc failed\n", __func__);
		return -ENOMEM;
	}

	eth->rx_buf_base = (u32 *)ALIGN((unsigned long)rx_buf_malloc,
					CONFIG_SYS_CACHELINE_SIZE);
	eth->rx_buf_dma_base = (u32 *)&eth->rx_buf_base[0];
	rx_buf = eth->rx_buf_dma_base;

	/* descripter entry */
	desc_size = TX_DESC_NUM * sizeof(struct tx_desc_s) +
		RX_DESC_NUM * sizeof(struct rx_desc_s) +
		CONFIG_SYS_CACHELINE_SIZE;
	if (desc_malloc_addr) {
		free(desc_malloc_addr);
		desc_malloc_addr = NULL;
	}
	desc_malloc_addr = malloc(desc_size);
	if (!desc_malloc_addr) {
		printf(EMAC_NAME ": %s:  malloc failed\n", __func__);
		return -ENOMEM;
	}
	desc_malloc = (void *)ALIGN((unsigned long)desc_malloc_addr,
				    CONFIG_SYS_CACHELINE_SIZE);

	desc_addr = (unsigned long)desc_malloc;
	desc_dma_addr = (unsigned long)desc_malloc;

	/* RX descripter entry */
	eth->rx_desc_base = (struct rx_desc_s *)desc_addr;
	eth->rx_desc_dma_base = (struct rx_desc_s *)desc_dma_addr;

	desc_addr += sizeof(struct rx_desc_s) * RX_DESC_NUM;
	desc_dma_addr += sizeof(struct rx_desc_s) * RX_DESC_NUM;

	for (i = 0; i < RX_DESC_NUM; i++, rx_buf += RX_BUF_SIZE) {
		eth->rx_desc_base[i].rd2 = RD2_RBA_SET((u32)rx_buf);
		eth->rx_desc_base[i].rd1 = RD1_RBL_SET(RX_BUF_SIZE);
		eth->rx_desc_base[i].rd0 = RD0_RACT;
	}

	eth->rx_desc_base[RX_DESC_NUM - 1].rd0 |= RD0_RDL;
	eth->rxdesc = 0;

	/* TX descripter entry */
	eth->tx_desc_base = (struct tx_desc_s *)desc_addr;
	eth->tx_desc_dma_base = (struct tx_desc_s *)desc_dma_addr;

	for (i = 0; i < TX_DESC_NUM; i++) {
		eth->tx_desc_base[i].td0 = 0x00;
		eth->tx_desc_base[i].td1 = 0x00;
		eth->tx_desc_base[i].td2 = 0x00;
	}

	eth->tx_desc_base[TX_DESC_NUM - 1].td0 |= TD0_TDL;
	eth->txdesc = 0;

	/* Descripter Table entry */
	flush_cache((ulong)desc_malloc_addr, (ulong)desc_size);
	emac_writel((u32)&eth->rx_desc_dma_base[0], &tr->drxspar);
	emac_writel((u32)&eth->rx_desc_dma_base[0], &tr->drxcpar);
	emac_writel((u32)&eth->rx_desc_dma_base[RX_DESC_NUM - 1],
		    &tr->drxlpar);
	emac_writel(DRXDLR_DRXDL, &tr->drxdlr);
	emac_writel((u32)&eth->tx_desc_dma_base[0], &tr->dtxspar);
	emac_writel((u32)&eth->tx_desc_dma_base[0], &tr->dtxcpar);
	emac_writel((u32)&eth->tx_desc_dma_base[TX_DESC_NUM - 1],
		    &tr->dtxlpar);
	emac_writel(DTXDLR_DTXDL, &tr->dtxdlr);

	return 0;
}

static void emac_eth_halt(struct eth_device *dev)
{
	struct emac_eth_dev *eth = dev->priv;
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;
	int ret;
	u32 reg;

	reg = (emac_readl(&tr->dmatxr) & DMATXR_START)
		| (emac_readl(&tr->dmarxr) & DMARXR_START)
		| (emac_readl(&tr->maccr) & (MACCR_TPE | MACCR_RPE));

	if (reg != 0) {
		ret = emac_eth_stop(eth);
		if (ret < 0)
			printf(EMAC_NAME ": %s: stop failed\n", __func__);
	} else {
		ret = emac_eth_reset(eth);
		if (ret < 0)
			printf(EMAC_NAME ": %s :reset failed\n", __func__);
	}

	if (desc_malloc_addr) {
		free(desc_malloc_addr);
		desc_malloc_addr = NULL;
	}
	if (rx_buf_malloc) {
		free(rx_buf_malloc);
		rx_buf_malloc = NULL;
	}
	/* PHY init set-> 200ns */
	emac_writel(MDIOCYCR_SET, &tr->mdiocycr);
}

static int emac_eth_send(struct eth_device *dev, void *packet, int len)
{
	unsigned int i;
	u32 stat;
	struct emac_eth_dev *eth = dev->priv;
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;
	struct tx_desc_s *tx_desc;
	unsigned long timeout;

	debug("start send tz3000emac\n");
	if (!packet || len > ETH_FRAME_LEN) {
		printf(EMAC_NAME ":%s; Invalid argument.\n", __func__);
		return -EIO;
	}

	flush_dcache_range((ulong)packet & ~(ARCH_DMA_MINALIGN - 1),
			   roundup((ulong)packet + len, ARCH_DMA_MINALIGN));

	i = eth->txdesc;
	tx_desc = &eth->tx_desc_base[i];

	invalidate_txdesc(tx_desc);
	tx_desc->td0 &= TD0_TDL;
	tx_desc->td2 = (u32)packet;
	tx_desc->td1 = TD1_TBL_SET(len);
	tx_desc->td0 |= TD0_TACT | TD0_TFP_ALL;
	emac_writel((DMAINTR_FTC0 | DMAINTR_FTC1), &tr->dmaintr);

	flush_txdesc(tx_desc);
	/* TX DMA Start (TxBuffer -> TXFIFO) */
	emac_writel(DMATXR_START, &tr->dmatxr);

	timeout = get_timer(0);
	while (1) {
		stat = emac_readl(&tr->dmaintr);
		if (stat & (DMAINTR_FTC1 | DMAINTR_FTC0)) {
			invalidate_txdesc(tx_desc);

			tx_desc = &eth->tx_desc_base[i];
			if (!(tx_desc->td0 & TD0_TACT)) {
				eth->txdesc = (eth->txdesc + 1) % TX_DESC_NUM;
				i = eth->txdesc;
				flush_txdesc(&eth->tx_desc_base[i]);
				tx_desc = &eth->tx_desc_base[i];
				break;
			}
		}
		if (get_timer(timeout) > TRANSMIT_TIMEOUT) {
			printf(EMAC_NAME ":%s: transmit timeout\n", __func__);
			goto err;
		}
	}

	stat =
		(emac_readl(&tr->dmaintr) &
		 (DMAINTR_TABT | DMAINTR_TDE | DMAINTR_TFE | DMAINTR_TINT3 |
		  DMAINTR_TINT2 | DMAINTR_TINT1));
	if (!stat)
		return 0;
err:
	printf(EMAC_NAME ": failed to send packet: %s:%s:%s:%s:%s:%s\n",
	       stat & DMAINTR_TABT ? "TABT" : "",
	       stat & DMAINTR_TDE ? "TDE" : "",
	       stat & DMAINTR_TFE ? "TFE" : "",
	       stat & DMAINTR_TINT3 ? "TINT3" : "",
	       stat & DMAINTR_TINT2 ? "TINT2" : "",
	       stat & DMAINTR_TINT1 ? "TINT1" : "");

	return -EIO;
}

static int emac_eth_recv(struct eth_device *dev)
{
	struct emac_eth_dev *eth = dev->priv;
	int len = 0, i;
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;
	uchar *packet;
	struct rx_desc_s *rx_desc;
	unsigned long timeout;
	u32 stat;

	debug("emac_eth_recv start\n");

	i = eth->rxdesc;
	rx_desc = &eth->rx_desc_base[i];
	invalidate_rxdesc(rx_desc);

	timeout = get_timer(0);
	while ((rx_desc->rd0 & RD0_RACT) != 0) {
		rx_desc = &eth->rx_desc_base[i];
		invalidate_rxdesc(rx_desc);
		if (get_timer(timeout) > READ_TIMEOUT)
			break;
	}
	if (!(rx_desc->rd0 & RD0_RFE)) {
		len = rx_desc->rd1 & 0xffff;
		packet = (uchar *)rx_desc->rd2;
		invalidate_dcache_range((ulong)packet,
					(ulong)packet +
					roundup(RX_BUF_SIZE,
						ARCH_DMA_MINALIGN));
#ifdef DEBUG
		int j;
		printf("-----------------------------------\n");
		printf("rx_packet = %p, length = %d,\n", packet, len);
		for (j = 0; j < len; j++) {
			printf("0x%02x ", packet[j]);
			if (j % 16 == 15)
				printf("\n");
		}
		printf("\n-----------------------------------\n");
#endif
		NetReceive(packet, len);	/* packet -> eth core */
		debug("NetRecieve done\n");

		invalidate_rxdesc(rx_desc);
		rx_desc->rd0 &= RD0_RDL;	/* clear RD0 */
		rx_desc->rd0 |= RD0_RACT;

		flush_rxdesc(rx_desc);

		/* descripter ring */
		eth->rxdesc = (eth->rxdesc + 1) % RX_DESC_NUM;
		i = eth->rxdesc;

		flush_rxdesc(rx_desc);
		rx_desc = &eth->rx_desc_base[i];
	}

	stat = (emac_readl(&tr->dmaintr) &
		(DMAINTR_RFRMER | DMAINTR_RABT | DMAINTR_RDE | DMAINTR_RFE |
		 DMAINTR_RINT7 | DMAINTR_RINT6 | DMAINTR_RINT5 |
		 DMAINTR_RINT4 | DMAINTR_RINT3 | DMAINTR_RINT2 |
		 DMAINTR_RINT1));
	if (stat == DMAINTR_RDE) {
		/* handle RDE */
		emac_writel(DMAINTR_RDE, &tr->dmaintr);
		/* Rx DMA restart */
		emac_writel(DMARXR_START, &tr->dmarxr);
		stat = 0;
	}
	if (!stat)
		return 0;
	printf(EMAC_NAME
	       ": failed to recv packet: %s:%s:%s:%s:%s:%s:%s:%s:%s:%s:%s\n",
	       stat & DMAINTR_RFRMER ? "RFRMER" : "",
	       stat & DMAINTR_RABT ? "RABT" : "",
	       stat & DMAINTR_RDE ? "RDE" : "",
	       stat & DMAINTR_RFE ? "RFE" : "",
	       stat & DMAINTR_RINT7 ? "RINT7" : "",
	       stat & DMAINTR_RINT6 ? "RINT6" : "",
	       stat & DMAINTR_RINT5 ? "RINT5" : "",
	       stat & DMAINTR_RINT4 ? "RINT4" : "",
	       stat & DMAINTR_RINT3 ? "RINT3" : "",
	       stat & DMAINTR_RINT2 ? "RINT2" : "",
	       stat & DMAINTR_RINT1 ? "RINT1" : "");

	return -EIO;
}

int tz3000emac_register(bd_t *bd)
{
	int ret = 0;
	struct emac_eth_dev *eth = NULL;
	struct eth_device *dev = NULL;
	struct emac_eth_regs *tr = (struct emac_eth_regs *)EMAC_IOBASE_ADDR;

	eth = malloc(sizeof(struct emac_eth_dev));
	if (!eth) {
		printf(EMAC_NAME ":%s:eth malloc failed\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	dev = malloc(sizeof(struct eth_device));
	if (!dev) {
		printf(EMAC_NAME ":%s:dev malloc failed\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	memset(dev, 0, sizeof(struct eth_device));
	memset(eth, 0, sizeof(struct emac_eth_dev));

	/* EMAC device entry to u-boot */
	dev->priv = eth;
	dev->iobase = EMAC_IOBASE_ADDR;
	dev->init = emac_eth_init;
	dev->halt = emac_eth_halt;
	dev->send = emac_eth_send;
	dev->recv = emac_eth_recv;
	sprintf(dev->name, EMAC_NAME);
	eth_register(dev);
	debug(EMAC_NAME ": MAC %pM\n", dev->enetaddr);

#ifdef CONFIG_TZ3000_RGMII
	/* PLLCTRL setting */
	emac_writel(0, PLL_CTRL);
	udelay(100);
	emac_writel(0x05000000, &tr->phyifr);

#endif

	/* PHY init set-> 200ns */
	emac_writel(MDIOCYCR_SET, &tr->mdiocycr);

#if defined(CONFIG_MII) || defined(CONFIG_CMD_MII)
	miiphy_register(EMAC_NAME, emac_eth_mii_read, emac_eth_mii_write);
#endif
	return ret;

err:
	if (dev) {
		free(dev);
		dev = NULL;
	}

	if (eth) {
		free(eth);
		eth = NULL;
	}

	printf(EMAC_NAME ":failed\n");
	return ret;
}
