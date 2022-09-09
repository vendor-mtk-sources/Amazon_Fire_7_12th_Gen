/*
 * Copyright (c) 2014-2015 MediaTek Inc.
 * Author: Chaotian.Jing <chaotian.jing@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>

#ifdef CONFIG_PM_DEVFREQ
#include <linux/pm_qos.h>
#ifdef CONFIG_MACH_MT8168
#include "helio-dvfsrc-opp-mt8168.h"
#endif
#endif
#ifdef CONFIG_MTK_EMMC_HW_CQ
#include "cmdq_hci.h"
#include "../core/card.h"
#endif
#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#endif

#ifdef CONFIG_AMZN_METRICS_LOG
#include <linux/amzn_metricslog.h>
#endif

#include <asm-generic/div64.h>
#define MAX_BD_NUM          1024

/*--------------------------------------------------------------------------*/
/* Common Definition                                                        */
/*--------------------------------------------------------------------------*/
#define MSDC_BUS_1BITS          0x0
#define MSDC_BUS_4BITS          0x1
#define MSDC_BUS_8BITS          0x2

#define MSDC_BURST_64B          0x6

/*--------------------------------------------------------------------------*/
/* Register Offset                                                          */
/*--------------------------------------------------------------------------*/
#define MSDC_CFG         0x0
#define MSDC_IOCON       0x04
#define MSDC_PS          0x08
#define MSDC_INT         0x0c
#define MSDC_INTEN       0x10
#define MSDC_FIFOCS      0x14
#define SDC_CFG          0x30
#define SDC_CMD          0x34
#define SDC_ARG          0x38
#define SDC_STS          0x3c
#define SDC_RESP0        0x40
#define SDC_RESP1        0x44
#define SDC_RESP2        0x48
#define SDC_RESP3        0x4c
#define SDC_BLK_NUM      0x50
#define SDC_ADV_CFG0     0x64
#define EMMC_IOCON       0x7c
#define SDC_ACMD_RESP    0x80
#define DMA_SA_H4BIT     0x8c
#define MSDC_DMA_SA      0x90
#define MSDC_DMA_CTRL    0x98
#define MSDC_DMA_CFG     0x9c
#define MSDC_PATCH_BIT   0xb0
#define MSDC_PATCH_BIT1  0xb4
#define MSDC_PATCH_BIT2  0xb8
#define MSDC_PAD_TUNE    0xec
#define MSDC_PAD_TUNE0   0xf0
#define PAD_DS_TUNE      0x188
#define PAD_CMD_TUNE     0x18c
#define EMMC50_CFG0      0x208
#define EMMC50_CFG3      0x220
#define SDC_FIFO_CFG     0x228

/*--------------------------------------------------------------------------*/
/* Top Register Offset                                                      */
/*--------------------------------------------------------------------------*/
#define EMMC_TOP_CONTROL		(0x00)
#define EMMC_TOP_CMD			(0x04)
#define EMMC50_PAD_DS_TUNE		(0x0c)

/*--------------------------------------------------------------------------*/
/* Register Mask                                                            */
/*--------------------------------------------------------------------------*/

/* MSDC_CFG mask */
#define MSDC_CFG_MODE           (0x1 << 0)	/* RW */
#define MSDC_CFG_CKPDN          (0x1 << 1)	/* RW */
#define MSDC_CFG_RST            (0x1 << 2)	/* RW */
#define MSDC_CFG_PIO            (0x1 << 3)	/* RW */
#define MSDC_CFG_CKDRVEN        (0x1 << 4)	/* RW */
#define MSDC_CFG_BV18SDT        (0x1 << 5)	/* RW */
#define MSDC_CFG_BV18PSS        (0x1 << 6)	/* R  */
#define MSDC_CFG_CKSTB          (0x1 << 7)	/* R  */
#define MSDC_CFG_CKDIV          (0xff << 8)	/* RW */
#define MSDC_CFG_CKMOD          (0x3 << 16)	/* RW */
#define MSDC_CFG_HS400_CK_MODE  (0x1 << 18)	/* RW */
#define MSDC_CFG_HS400_CK_MODE_EXTRA  (0x1 << 22)	/* RW */
#define MSDC_CFG_CKDIV_EXTRA    (0xfff << 8)	/* RW */
#define MSDC_CFG_CKMOD_EXTRA    (0x3 << 20)	/* RW */

/* MSDC_IOCON mask */
#define MSDC_IOCON_SDR104CKS    (0x1 << 0)	/* RW */
#define MSDC_IOCON_RSPL         (0x1 << 1)	/* RW */
#define MSDC_IOCON_DSPL         (0x1 << 2)	/* RW */
#define MSDC_IOCON_DDLSEL       (0x1 << 3)	/* RW */
#define MSDC_IOCON_DDR50CKD     (0x1 << 4)	/* RW */
#define MSDC_IOCON_DSPLSEL      (0x1 << 5)	/* RW */
#define MSDC_IOCON_W_DSPL       (0x1 << 8)	/* RW */
#define MSDC_IOCON_D0SPL        (0x1 << 16)	/* RW */
#define MSDC_IOCON_D1SPL        (0x1 << 17)	/* RW */
#define MSDC_IOCON_D2SPL        (0x1 << 18)	/* RW */
#define MSDC_IOCON_D3SPL        (0x1 << 19)	/* RW */
#define MSDC_IOCON_D4SPL        (0x1 << 20)	/* RW */
#define MSDC_IOCON_D5SPL        (0x1 << 21)	/* RW */
#define MSDC_IOCON_D6SPL        (0x1 << 22)	/* RW */
#define MSDC_IOCON_D7SPL        (0x1 << 23)	/* RW */
#define MSDC_IOCON_RISCSZ       (0x3 << 24)	/* RW */

/* MSDC_PS mask */
#define MSDC_PS_CDEN            (0x1 << 0)	/* RW */
#define MSDC_PS_CDSTS           (0x1 << 1)	/* R  */
#define MSDC_PS_CDDEBOUNCE      (0xf << 12)	/* RW */
#define MSDC_PS_DAT             (0xff << 16)	/* R  */
#define MSDC_PS_CMD             (0x1 << 24)	/* R  */
#define MSDC_PS_WP              (0x1 << 31)	/* R  */

/* MSDC_INT mask */
#define MSDC_INT_MMCIRQ         (0x1 << 0)	/* W1C */
#define MSDC_INT_CDSC           (0x1 << 1)	/* W1C */
#define MSDC_INT_ACMDRDY        (0x1 << 3)	/* W1C */
#define MSDC_INT_ACMDTMO        (0x1 << 4)	/* W1C */
#define MSDC_INT_ACMDCRCERR     (0x1 << 5)	/* W1C */
#define MSDC_INT_DMAQ_EMPTY     (0x1 << 6)	/* W1C */
#define MSDC_INT_SDIOIRQ        (0x1 << 7)	/* W1C */
#define MSDC_INT_CMDRDY         (0x1 << 8)	/* W1C */
#define MSDC_INT_CMDTMO         (0x1 << 9)	/* W1C */
#define MSDC_INT_RSPCRCERR      (0x1 << 10)	/* W1C */
#define MSDC_INT_CSTA           (0x1 << 11)	/* R */
#define MSDC_INT_XFER_COMPL     (0x1 << 12)	/* W1C */
#define MSDC_INT_DXFER_DONE     (0x1 << 13)	/* W1C */
#define MSDC_INT_DATTMO         (0x1 << 14)	/* W1C */
#define MSDC_INT_DATCRCERR      (0x1 << 15)	/* W1C */
#define MSDC_INT_ACMD19_DONE    (0x1 << 16)	/* W1C */
#define MSDC_INT_DMA_BDCSERR    (0x1 << 17)	/* W1C */
#define MSDC_INT_DMA_GPDCSERR   (0x1 << 18)	/* W1C */
#define MSDC_INT_DMA_PROTECT    (0x1 << 19)	/* W1C */
#define MSDC_INT_CMDQ           (0x1  << 28)    /* W1C */

/* MSDC_INTEN mask */
#define MSDC_INTEN_MMCIRQ       (0x1 << 0)	/* RW */
#define MSDC_INTEN_CDSC         (0x1 << 1)	/* RW */
#define MSDC_INTEN_ACMDRDY      (0x1 << 3)	/* RW */
#define MSDC_INTEN_ACMDTMO      (0x1 << 4)	/* RW */
#define MSDC_INTEN_ACMDCRCERR   (0x1 << 5)	/* RW */
#define MSDC_INTEN_DMAQ_EMPTY   (0x1 << 6)	/* RW */
#define MSDC_INTEN_SDIOIRQ      (0x1 << 7)	/* RW */
#define MSDC_INTEN_CMDRDY       (0x1 << 8)	/* RW */
#define MSDC_INTEN_CMDTMO       (0x1 << 9)	/* RW */
#define MSDC_INTEN_RSPCRCERR    (0x1 << 10)	/* RW */
#define MSDC_INTEN_CSTA         (0x1 << 11)	/* RW */
#define MSDC_INTEN_XFER_COMPL   (0x1 << 12)	/* RW */
#define MSDC_INTEN_DXFER_DONE   (0x1 << 13)	/* RW */
#define MSDC_INTEN_DATTMO       (0x1 << 14)	/* RW */
#define MSDC_INTEN_DATCRCERR    (0x1 << 15)	/* RW */
#define MSDC_INTEN_ACMD19_DONE  (0x1 << 16)	/* RW */
#define MSDC_INTEN_DMA_BDCSERR  (0x1 << 17)	/* RW */
#define MSDC_INTEN_DMA_GPDCSERR (0x1 << 18)	/* RW */
#define MSDC_INTEN_DMA_PROTECT  (0x1 << 19)	/* RW */
#define MSDC_INTEN_CMDQ         (0x1  << 28)    /* W1C */

/* MSDC_FIFOCS mask */
#define MSDC_FIFOCS_RXCNT       (0xff << 0)	/* R */
#define MSDC_FIFOCS_TXCNT       (0xff << 16)	/* R */
#define MSDC_FIFOCS_CLR         (0x1 << 31)	/* RW */

/* SDC_CFG mask */
#define SDC_CFG_SDIOINTWKUP     (0x1 << 0)	/* RW */
#define SDC_CFG_INSWKUP         (0x1 << 1)	/* RW */
#define SDC_CFG_WRDTOC          (0xfff << 2)	/* RW */
#define SDC_CFG_BUSWIDTH        (0x3 << 16)	/* RW */
#define SDC_CFG_SDIO            (0x1 << 19)	/* RW */
#define SDC_CFG_SDIOIDE         (0x1 << 20)	/* RW */
#define SDC_CFG_INTATGAP        (0x1 << 21)	/* RW */
#define SDC_CFG_DTOC            (0xff << 24)	/* RW */

/* SDC_STS mask */
#define SDC_STS_SDCBUSY         (0x1 << 0)	/* RW */
#define SDC_STS_CMDBUSY         (0x1 << 1)	/* RW */
#define SDC_STS_SWR_COMPL       (0x1 << 31)	/* RW */

/* SDC_ADV_CFG0 mask */
#define SDC_RX_ENHANCE_EN	(0x1 << 20)	/* RW */

/* DMA_SA_H4BIT mask */
#define DMA_ADDR_HIGH_4BIT      (0xf << 0)      /* RW */

/* MSDC_DMA_CTRL mask */
#define MSDC_DMA_CTRL_START     (0x1 << 0)	/* W */
#define MSDC_DMA_CTRL_STOP      (0x1 << 1)	/* W */
#define MSDC_DMA_CTRL_RESUME    (0x1 << 2)	/* W */
#define MSDC_DMA_CTRL_MODE      (0x1 << 8)	/* RW */
#define MSDC_DMA_CTRL_LASTBUF   (0x1 << 10)	/* RW */
#define MSDC_DMA_CTRL_BRUSTSZ   (0x7 << 12)	/* RW */

/* MSDC_DMA_CFG mask */
#define MSDC_DMA_CFG_STS        (0x1 << 0)	/* R */
#define MSDC_DMA_CFG_DECSEN     (0x1 << 1)	/* RW */
#define MSDC_DMA_CFG_AHBHPROT2  (0x2 << 8)	/* RW */
#define MSDC_DMA_CFG_ACTIVEEN   (0x2 << 12)	/* RW */
#define MSDC_DMA_CFG_CS12B16B   (0x1 << 16)	/* RW */

/* MSDC_PATCH_BIT mask */
#define MSDC_PATCH_BIT_ODDSUPP    (0x1 <<  1)	/* RW */
#define MSDC_INT_DAT_LATCH_CK_SEL (0x7 <<  7)
#define MSDC_CKGEN_MSDC_DLY_SEL   (0x1f << 10)
#define MSDC_PATCH_BIT_IODSSEL    (0x1 << 16)	/* RW */
#define MSDC_PATCH_BIT_IOINTSEL   (0x1 << 17)	/* RW */
#define MSDC_PATCH_BIT_BUSYDLY    (0xf << 18)	/* RW */
#define MSDC_PATCH_BIT_WDOD       (0xf << 22)	/* RW */
#define MSDC_PATCH_BIT_IDRTSEL    (0x1 << 26)	/* RW */
#define MSDC_PATCH_BIT_CMDFSEL    (0x1 << 27)	/* RW */
#define MSDC_PATCH_BIT_INTDLSEL   (0x1 << 28)	/* RW */
#define MSDC_PATCH_BIT_SPCPUSH    (0x1 << 29)	/* RW */
#define MSDC_PATCH_BIT_DECRCTMO   (0x1 << 30)	/* RW */

#define MSDC_PATCH_BIT1_STOP_DLY  (0xf << 8)    /* RW */

#define MSDC_PATCH_BIT2_CFGRESP   (0x1 << 15)   /* RW */
#define MSDC_PATCH_BIT2_CFGCRCSTS (0x1 << 28)   /* RW */
#define MSDC_PB2_SUPPORT_64G      (0x1 << 1)    /* RW */
#define MSDC_PB2_RESPWAIT         (0x3 << 2)    /* RW */
#define MSDC_PB2_RESPSTSENSEL     (0x7 << 16)   /* RW */
#define MSDC_PB2_CRCSTSENSEL      (0x7 << 29)   /* RW */
#define MSDC_PATCH_BIT1_CMDTA     (0x7 << 3)    /* RW */

#define MSDC_PAD_TUNE_DATWRDLY	  (0x1f <<  0)	/* RW */
#define MSDC_PAD_TUNE_DATRRDLY	  (0x1f <<  8)	/* RW */
#define MSDC_PAD_TUNE_CMDRDLY	  (0x1f << 16)  /* RW */
#define MSDC_PAD_TUNE_CMDRRDLY	  (0x1f << 22)	/* RW */
#define MSDC_PAD_TUNE_CLKTDLY	  (0x1f << 27)  /* RW */
#define MSDC_PAD_TUNE_RXDLYSEL	  (0x1 << 15)   /* RW */
#define MSDC_PAD_TUNE_RD_SEL	  (0x1 << 13)   /* RW */
#define MSDC_PAD_TUNE_CMD_SEL	  (0x1 << 21)   /* RW */

#define PAD_DS_TUNE_DLY1	  (0x1f << 2)   /* RW */
#define PAD_DS_TUNE_DLY2	  (0x1f << 7)   /* RW */
#define PAD_DS_TUNE_DLY3	  (0x1f << 12)  /* RW */

#define PAD_CMD_TUNE_RX_DLY3	  (0x1f << 1)  /* RW */

#define EMMC50_CFG_PADCMD_LATCHCK (0x1 << 0)   /* RW */
#define EMMC50_CFG_CRCSTS_EDGE    (0x1 << 3)   /* RW */
#define EMMC50_CFG_CFCSTS_SEL     (0x1 << 4)   /* RW */

#define EMMC50_CFG3_OUTS_WR       (0x1f << 0)  /* RW */

#define SDC_FIFO_CFG_WRVALIDSEL   (0x1 << 24)  /* RW */
#define SDC_FIFO_CFG_RDVALIDSEL   (0x1 << 25)  /* RW */

/* EMMC_TOP_CONTROL mask */
#define PAD_RXDLY_SEL           (0x1 << 0)      /* RW */
#define DELAY_EN                (0x1 << 1)      /* RW */
#define PAD_DAT_RD_RXDLY2       (0x1F << 2)     /* RW */
#define PAD_DAT_RD_RXDLY        (0x1F << 7)     /* RW */
#define PAD_DAT_RD_RXDLY2_SEL   (0x1 << 12)     /* RW */
#define PAD_DAT_RD_RXDLY_SEL    (0x1 << 13)     /* RW */
#define DATA_K_VALUE_SEL        (0x1 << 14)     /* RW */
#define SDC_RX_ENH_EN           (0x1 << 15)     /* TW */

/* EMMC_TOP_CMD mask */
#define PAD_CMD_RXDLY2          (0x1F << 0)     /* RW */
#define PAD_CMD_RXDLY           (0x1F << 5)     /* RW */
#define PAD_CMD_RD_RXDLY2_SEL   (0x1 << 10)     /* RW */
#define PAD_CMD_RD_RXDLY_SEL    (0x1 << 11)     /* RW */
#define PAD_CMD_TX_DLY          (0x1F << 12)    /* RW */

#define REQ_CMD_EIO  (0x1 << 0)
#define REQ_CMD_TMO  (0x1 << 1)
#define REQ_DAT_ERR  (0x1 << 2)
#define REQ_STOP_EIO (0x1 << 3)
#define REQ_STOP_TMO (0x1 << 4)
#define REQ_CMD_BUSY (0x1 << 5)

#define MSDC_PREPARE_FLAG (0x1 << 0)
#define MSDC_ASYNC_FLAG (0x1 << 1)
#define MSDC_MMAP_FLAG (0x1 << 2)

#define MTK_MMC_AUTOSUSPEND_DELAY	50
#define CMD_TIMEOUT         (HZ/10 * 5)	/* 100ms x5 */
#define DAT_TIMEOUT         (HZ    * 5)	/* 1000ms x5 */
#ifdef CONFIG_AMZN_METRICS_LOG
#define METRICS_DELAY       HZ
#define SDCARD_HOST_NAME	"mmc1"
#endif

#define PAD_DELAY_MAX	32 /* PAD delay cells */
/*--------------------------------------------------------------------------*/
/* Descriptor Structure                                                     */
/*--------------------------------------------------------------------------*/
struct mt_gpdma_desc {
	u32 gpd_info;
#define GPDMA_DESC_HWO		(0x1 << 0)
#define GPDMA_DESC_BDP		(0x1 << 1)
#define GPDMA_DESC_CHECKSUM	(0xff << 8) /* bit8 ~ bit15 */
#define GPDMA_DESC_INT		(0x1 << 16)
#define GPDMA_DESC_NEXT_H4	(0xf << 24)
#define GPDMA_DESC_PTR_H4	(0xf << 28)
	u32 next;
	u32 ptr;
	u32 gpd_data_len;
#define GPDMA_DESC_BUFLEN	(0xffff) /* bit0 ~ bit15 */
#define GPDMA_DESC_EXTLEN	(0xff << 16) /* bit16 ~ bit23 */
	u32 arg;
	u32 blknum;
	u32 cmd;
};

struct mt_bdma_desc {
	u32 bd_info;
#define BDMA_DESC_EOL		(0x1 << 0)
#define BDMA_DESC_CHECKSUM	(0xff << 8) /* bit8 ~ bit15 */
#define BDMA_DESC_BLKPAD	(0x1 << 17)
#define BDMA_DESC_DWPAD		(0x1 << 18)
#define BDMA_DESC_NEXT_H4	(0xf << 24)
#define BDMA_DESC_PTR_H4	(0xf << 28)
	u32 next;
	u32 ptr;
	u32 bd_data_len;
#define BDMA_DESC_BUFLEN	(0xffff) /* bit0 ~ bit15 */
#define BDMA_DESC_BUFLEN_EXT	(0xffffff) /* bit0 ~ bit23 */
};

struct msdc_dma {
	struct scatterlist *sg;	/* I/O scatter list */
	struct mt_gpdma_desc *gpd;		/* pointer to gpd array */
	struct mt_bdma_desc *bd;		/* pointer to bd array */
	dma_addr_t gpd_addr;	/* the physical address of gpd array */
	dma_addr_t bd_addr;	/* the physical address of bd array */
};

struct msdc_save_para {
	u32 msdc_cfg;
	u32 iocon;
	u32 sdc_cfg;
	u32 pad_tune;
	u32 patch_bit0;
	u32 patch_bit1;
	u32 patch_bit2;
	u32 pad_ds_tune;
	u32 pad_cmd_tune;
	u32 emmc50_cfg0;
	u32 emmc50_cfg3;
	u32 sdc_fifo_cfg;
	u32 emmc_top_control;
	u32 emmc_top_cmd;
	u32 emmc50_pad_ds_tune;
};

struct mtk_mmc_compatible {
	u8 clk_div_bits;
	bool hs400_tune; /* only used for MT8173 */
	u32 pad_tune_reg;
	bool async_fifo;
	bool data_tune;
	bool busy_check;
	bool stop_clk_fix;
	bool enhance_rx;
	bool support_64g;
	bool tune_resp_data_together;
	u8 ckgen_delay;
};

struct msdc_tune_para {
	u32 iocon;
	u32 pad_tune;
	u32 pad_cmd_tune;
	u32 emmc_top_control;
	u32 emmc_top_cmd;
};

struct msdc_delay_phase {
	u8 maxlen;
	u8 start;
	u8 final_phase;
};

struct msdc_host {
	struct device *dev;
	const struct mtk_mmc_compatible *dev_comp;
	struct mmc_host *mmc;	/* mmc structure */
#ifdef CONFIG_MTK_EMMC_HW_CQ
	struct cmdq_host *cq_host;
#endif
	int cmd_rsp;

	spinlock_t lock;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;
	int error;

	void __iomem *base;		/* host base address */
	void __iomem *top_base;		/* host top register base address */

	struct msdc_dma dma;	/* dma channel */
	u64 dma_mask;

	u32 timeout_ns;		/* data timeout ns */
	u32 timeout_clks;	/* data timeout clks */

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_uhs;
	struct delayed_work req_timeout;
	int irq;		/* host interrupt */

	struct clk *src_clk;	/* msdc source clock */
	struct clk *h_clk;      /* msdc h_clk */
	struct clk *src_clk_cg; /* msdc source clock control gate */
	u32 mclk;		/* mmc subsystem clock frequency */
	u32 src_clk_freq;	/* source clock frequency */
	u32 sclk;		/* SD/MS bus clock frequency */
	unsigned char timing;
	bool vqmmc_enabled;
	u32 latch_ck;
	u32 hs400_ds_delay;
	u32 hs200_cmd_int_delay; /* cmd internal delay for HS200/SDR104 */
	u32 hs400_cmd_int_delay; /* cmd internal delay for HS400 */
	bool hs400_cmd_resp_sel_rising;
				 /* cmd response sample selection for HS400 */
	bool hs400_mode;	/* current eMMC will run at hs400 mode */
	struct msdc_save_para save_para; /* used when gate HCLK */
	struct msdc_tune_para def_tune_para; /* default tune setting */
	struct msdc_tune_para saved_tune_para; /* tune result of CMD21/CMD19 */
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	u32 tune_flag; /* flag defined for emmc window */
#endif
#ifdef CONFIG_PM_DEVFREQ
	struct pm_qos_request pm_qos;
#endif

	u32 crc_count;	/* total crc count */
	u32 crc_invalid_count;	/* total crc invalid count eg CMD19 */
	u32 req_count; /* total request count */
	u32 datatimeout_count; /* total data timeout count */
	u32 cmdtimeout_count; /* total cmd timeout count */
	u32 reqtimeout_count; /* total req timeout count */
	u32 pc_count;	/* total power cycle count */
	u32 pc_suspend;	/* suspend/resume count */
	u32 cmd19_fail; /* cmd19 tune failed count */

#ifdef CONFIG_AMZN_METRICS_LOG
	struct delayed_work metrics_work;
	bool metrics_enable;
	u32 crc_count_p;	/* reported crc count */
	u32 crc_invalid_count_p;	/* reported crc invalid count eg CMD19 */
	u32 req_count_p; /* reported request count */
	u32 datatimeout_count_p; /* reported data timeout count */
	u32 cmdtimeout_count_p; /* reported cmd timeout count */
	u32 reqtimeout_count_p; /* reported req timeout count */
	u32 pc_count_p;	/* reported power cycle count */
	u32 pc_suspend_p;	/* reported suspend/resume count */
	u32 cmd19_fail_p; /* reported cmd19 tune failed count */
	u32 inserted_p; /* reported card detection count */
	u32 inserted; /* total card detection could */
#endif
};

#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
#define MMC_TUNE_FLAG_NONE      0x0
#define MMC_TUNE_FLAG_RSP       0x1
#define MMC_TUNE_FLAG_DAT       0x2
#define MMC_TUNE_FLAG_RETUNE    0x4
#define MMC_TUNE_FLAG_RSP_DAT   (MMC_TUNE_FLAG_RSP | MMC_TUNE_FLAG_DAT)

#define NEED_TUNE_RSP(host)     (((host)->tune_flag & MMC_TUNE_FLAG_RSP) ? false : true)
#define NEED_TUNE_DAT(host)     (((host)->tune_flag & MMC_TUNE_FLAG_DAT) ? false : true)
#define NEED_TUNE_RSP_DAT(host) (((host)->tune_flag & MMC_TUNE_FLAG_RSP_DAT) ? false : true)
#define SET_RSP_TUNED(host)     ((host)->tune_flag |= MMC_TUNE_FLAG_RSP)
#define SET_DAT_TUNED(host)     ((host)->tune_flag |= MMC_TUNE_FLAG_DAT)
#define SET_RSP_DAT_TUNED(host) do {                \
	if ((host)->tune_flag == MMC_TUNE_FLAG_NONE) {  \
		(host)->tune_flag |= MMC_TUNE_FLAG_RETUNE;  \
	} else {                                        \
		(host)->tune_flag |= MMC_TUNE_FLAG_RSP_DAT; \
	}                                               \
} while (0)
#endif

#ifdef CONFIG_AMZN_METRICS_LOG
#define MSDC_LOG_COUNTER_TO_VITALS(name, value) \
	do { \
		if (value != value##_p) { \
			log_counter_to_vitals(ANDROID_LOG_INFO, "Kernel", "Kernel", \
				((host->mmc) && (host->mmc->caps & MMC_CAP_SD_HIGHSPEED)) ? "SD" : "EMMC", \
				#name, value - value##_p, "count", NULL, VITALS_NORMAL); \
			value##_p = value; \
		} \
	} while (0)
#endif

#define FILTER_INVALIDCMD(opcode) \
((opcode == MMC_SLEEP_AWAKE) || (opcode == MMC_SEND_EXT_CSD) || (opcode == MMC_BUS_TEST_W) \
	|| (opcode == MMC_SEND_TUNING_BLOCK_HS200) || (opcode == SD_IO_RW_DIRECT) \
	|| (opcode == MMC_APP_CMD))

#define MSDC_DEV_ATTR(name, fmt, val, fmt_type)					\
static ssize_t msdc_attr_##name##_show(struct device *dev, struct device_attribute *attr, char *buf)	\
{										\
	struct mmc_host *mmc = dev_get_drvdata(dev);				\
	struct msdc_host *host = mmc_priv(mmc);					\
	return sprintf(buf, fmt "\n", (fmt_type)val);				\
}										\
static ssize_t msdc_attr_##name##_store(struct device *dev,			\
	struct device_attribute *attr, const char *buf, size_t count)		\
{										\
	fmt_type tmp;								\
	struct mmc_host *mmc = dev_get_drvdata(dev);				\
	struct msdc_host *host = mmc_priv(mmc);					\
	int n = sscanf(buf, fmt, &tmp);						\
	val = (typeof(val))tmp;							\
	return n ? count : -EINVAL;						\
}										\
static DEVICE_ATTR(name, 0664, msdc_attr_##name##_show, msdc_attr_##name##_store)


MSDC_DEV_ATTR(crc_count, "%d", host->crc_count, u32);
MSDC_DEV_ATTR(crc_invalid_count, "%d", host->crc_invalid_count, u32);
MSDC_DEV_ATTR(req_count, "%d", host->req_count, u32);
MSDC_DEV_ATTR(datatimeout_count, "%d", host->datatimeout_count, u32);
MSDC_DEV_ATTR(cmdtimeout_count, "%d", host->cmdtimeout_count, u32);
MSDC_DEV_ATTR(reqtimeout_count, "%d", host->reqtimeout_count, u32);
MSDC_DEV_ATTR(pc_count, "%d", host->pc_count, u32);
MSDC_DEV_ATTR(pc_suspend, "%d", host->pc_suspend, u32);
MSDC_DEV_ATTR(cmd19_fail, "%d", host->cmd19_fail, u32);

static struct device_attribute *msdc_attrs[] = {
	&dev_attr_crc_count,
	&dev_attr_crc_invalid_count,
	&dev_attr_req_count,
	&dev_attr_datatimeout_count,
	&dev_attr_cmdtimeout_count,
	&dev_attr_reqtimeout_count,
	&dev_attr_pc_count,
	&dev_attr_pc_suspend,
	&dev_attr_cmd19_fail,
	NULL,
};

static void msdc_add_device_attrs(struct msdc_host *host, struct device_attribute *attrs[])
{
	int i, ret;

	if (!attrs)
		return;

	for (i = 0; attrs[i]; ++i) {
		ret = device_create_file(host->dev, attrs[i]);
		if (ret)
			dev_err(host->dev, "failed to register attribute: %s; err=%d\n",
				attrs[i]->attr.name, ret);
	}
}

static void msdc_remove_device_attrs(struct msdc_host *host, struct device_attribute *attrs[])
{
	int i;

	if (!attrs)
		return;

	for (i = 0; attrs[i]; ++i)
		device_remove_file(host->dev, attrs[i]);
}

#ifdef CONFIG_AMZN_METRICS_LOG
static void msdc_metrics_work(struct work_struct *work)
{
	struct msdc_host *host = container_of(work, struct msdc_host, metrics_work.work);

	MSDC_LOG_COUNTER_TO_VITALS(crc, host->crc_count);
	MSDC_LOG_COUNTER_TO_VITALS(crc_invalid, host->crc_invalid_count);
	MSDC_LOG_COUNTER_TO_VITALS(req, host->req_count);
	MSDC_LOG_COUNTER_TO_VITALS(datato, host->datatimeout_count);
	MSDC_LOG_COUNTER_TO_VITALS(cmdto, host->cmdtimeout_count);
	MSDC_LOG_COUNTER_TO_VITALS(reqto, host->reqtimeout_count);
	MSDC_LOG_COUNTER_TO_VITALS(pc_count, host->pc_count);
	MSDC_LOG_COUNTER_TO_VITALS(pc_suspend, host->pc_suspend);
	MSDC_LOG_COUNTER_TO_VITALS(cmd19_fail, host->cmd19_fail);
	MSDC_LOG_COUNTER_TO_VITALS(inserted, host->inserted);
}
#endif

static const struct mtk_mmc_compatible mt8135_compat = {
	.clk_div_bits = 8,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE,
	.async_fifo = false,
	.data_tune = false,
	.busy_check = false,
	.stop_clk_fix = false,
	.enhance_rx = false,
	.support_64g = false,
	.tune_resp_data_together = false,
	.ckgen_delay = 1,
};

static const struct mtk_mmc_compatible mt8173_compat = {
	.clk_div_bits = 8,
	.hs400_tune = true,
	.pad_tune_reg = MSDC_PAD_TUNE,
	.async_fifo = false,
	.data_tune = false,
	.busy_check = false,
	.stop_clk_fix = false,
	.enhance_rx = false,
	.support_64g = false,
	.tune_resp_data_together = false,
	.ckgen_delay = 1,
};

static const struct mtk_mmc_compatible mt8168_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
	.tune_resp_data_together = true,
	.ckgen_delay = 0,
};

static const struct mtk_mmc_compatible mt2701_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = false,
	.stop_clk_fix = false,
	.enhance_rx = false,
	.support_64g = false,
	.tune_resp_data_together = false,
	.ckgen_delay = 1,
};

static const struct mtk_mmc_compatible mt2712_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
	.tune_resp_data_together = true,
	.ckgen_delay = 1,
};

static const struct mtk_mmc_compatible mt7622_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = false,
	.tune_resp_data_together = false,
	.ckgen_delay = 1,
};

static const struct mtk_mmc_compatible mt8183_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
	.tune_resp_data_together = true,
	.ckgen_delay = 1,
};

static const struct of_device_id msdc_of_ids[] = {
	{ .compatible = "mediatek,mt8135-mmc", .data = &mt8135_compat},
	{ .compatible = "mediatek,mt8173-mmc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt8168-mmc", .data = &mt8168_compat},
	{ .compatible = "mediatek,mt2701-mmc", .data = &mt2701_compat},
	{ .compatible = "mediatek,mt2712-mmc", .data = &mt2712_compat},
	{ .compatible = "mediatek,mt7622-mmc", .data = &mt7622_compat},
	{ .compatible = "mediatek,mt8183-mmc", .data = &mt8183_compat},
	{}
};
MODULE_DEVICE_TABLE(of, msdc_of_ids);

static void sdr_set_bits(void __iomem *reg, u32 bs)
{
	u32 val = readl(reg);

	val |= bs;
	writel(val, reg);
}

static void sdr_clr_bits(void __iomem *reg, u32 bs)
{
	u32 val = readl(reg);

	val &= ~bs;
	writel(val, reg);
}

static void sdr_set_field(void __iomem *reg, u32 field, u32 val)
{
	unsigned int tv = readl(reg);

	tv &= ~field;
	tv |= ((val) << (ffs((unsigned int)field) - 1));
	writel(tv, reg);
}

static void sdr_get_field(void __iomem *reg, u32 field, u32 *val)
{
	unsigned int tv = readl(reg);

	*val = ((tv & field) >> (ffs((unsigned int)field) - 1));
}

static void msdc_reset_hw(struct msdc_host *host)
{
	u32 val;

	sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_RST);
	while (readl(host->base + MSDC_CFG) & MSDC_CFG_RST)
		cpu_relax();

	sdr_set_bits(host->base + MSDC_FIFOCS, MSDC_FIFOCS_CLR);
	while (readl(host->base + MSDC_FIFOCS) & MSDC_FIFOCS_CLR)
		cpu_relax();

	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);
}

static void msdc_cmd_next(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd);

static const u32 cmd_ints_mask = MSDC_INTEN_CMDRDY | MSDC_INTEN_RSPCRCERR |
			MSDC_INTEN_CMDTMO | MSDC_INTEN_ACMDRDY |
			MSDC_INTEN_ACMDCRCERR | MSDC_INTEN_ACMDTMO;
static const u32 data_ints_mask = MSDC_INTEN_XFER_COMPL | MSDC_INTEN_DATTMO |
			MSDC_INTEN_DATCRCERR | MSDC_INTEN_DMA_BDCSERR |
			MSDC_INTEN_DMA_GPDCSERR | MSDC_INTEN_DMA_PROTECT;

static u8 msdc_dma_calcs(u8 *buf, u32 len)
{
	u32 i, sum = 0;

	for (i = 0; i < len; i++)
		sum += buf[i];
	return 0xff - (u8) sum;
}

static inline void msdc_dma_setup(struct msdc_host *host, struct msdc_dma *dma,
		struct mmc_data *data)
{
	unsigned int j, dma_len;
	dma_addr_t dma_address;
	u32 dma_ctrl;
	struct scatterlist *sg;
	struct mt_gpdma_desc *gpd;
	struct mt_bdma_desc *bd;

	sg = data->sg;

	gpd = dma->gpd;
	bd = dma->bd;

	/* modify gpd */
	gpd->gpd_info |= GPDMA_DESC_HWO;
	gpd->gpd_info |= GPDMA_DESC_BDP;
	/* need to clear first. use these bits to calc checksum */
	gpd->gpd_info &= ~GPDMA_DESC_CHECKSUM;
	gpd->gpd_info |= msdc_dma_calcs((u8 *) gpd, 16) << 8;

	/* modify bd */
	for_each_sg(data->sg, sg, data->sg_count, j) {
		dma_address = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);

		/* init bd */
		bd[j].bd_info &= ~BDMA_DESC_BLKPAD;
		bd[j].bd_info &= ~BDMA_DESC_DWPAD;
		bd[j].ptr = lower_32_bits(dma_address);
		if (host->dev_comp->support_64g) {
			bd[j].bd_info &= ~BDMA_DESC_PTR_H4;
			bd[j].bd_info |= (upper_32_bits(dma_address) & 0xf)
					 << 28;
		}

		if (host->dev_comp->support_64g) {
			bd[j].bd_data_len &= ~BDMA_DESC_BUFLEN_EXT;
			bd[j].bd_data_len |= (dma_len & BDMA_DESC_BUFLEN_EXT);
		} else {
			bd[j].bd_data_len &= ~BDMA_DESC_BUFLEN;
			bd[j].bd_data_len |= (dma_len & BDMA_DESC_BUFLEN);
		}

		if (j == data->sg_count - 1) /* the last bd */
			bd[j].bd_info |= BDMA_DESC_EOL;
		else
			bd[j].bd_info &= ~BDMA_DESC_EOL;

		/* checksume need to clear first */
		bd[j].bd_info &= ~BDMA_DESC_CHECKSUM;
		bd[j].bd_info |= msdc_dma_calcs((u8 *)(&bd[j]), 16) << 8;
	}

	sdr_set_field(host->base + MSDC_DMA_CFG, MSDC_DMA_CFG_DECSEN, 1);
	dma_ctrl = readl_relaxed(host->base + MSDC_DMA_CTRL);
	dma_ctrl &= ~(MSDC_DMA_CTRL_BRUSTSZ | MSDC_DMA_CTRL_MODE);
	dma_ctrl |= (MSDC_BURST_64B << 12 | 1 << 8);
	writel_relaxed(dma_ctrl, host->base + MSDC_DMA_CTRL);
	if (host->dev_comp->support_64g)
		sdr_set_field(host->base + DMA_SA_H4BIT, DMA_ADDR_HIGH_4BIT,
			      upper_32_bits(dma->gpd_addr) & 0xf);
	writel(lower_32_bits(dma->gpd_addr), host->base + MSDC_DMA_SA);
}

static void msdc_prepare_data(struct msdc_host *host, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;

	if (!(data->host_cookie & MSDC_PREPARE_FLAG)) {
		data->host_cookie |= MSDC_PREPARE_FLAG;
		data->sg_count = dma_map_sg(host->dev, data->sg, data->sg_len,
					    mmc_get_dma_dir(data));
	}
}

static void msdc_unprepare_data(struct msdc_host *host, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;

	if (data->host_cookie & MSDC_ASYNC_FLAG)
		return;

	if (data->host_cookie & MSDC_PREPARE_FLAG) {
		dma_unmap_sg(host->dev, data->sg, data->sg_len,
			     mmc_get_dma_dir(data));
		data->host_cookie &= ~MSDC_PREPARE_FLAG;
	}
}

/* clock control primitives */
static void msdc_set_timeout(struct msdc_host *host, u32 ns, u32 clks)
{
	u32 timeout, clk_ns;
	u32 mode = 0;

	host->timeout_ns = ns;
	host->timeout_clks = clks;
	if (host->sclk == 0) {
		timeout = 0;
	} else {
		clk_ns  = 1000000000UL / host->sclk;
		timeout = (ns + clk_ns - 1) / clk_ns + clks;
		/* in 1048576 sclk cycle unit */
		timeout = (timeout + (0x1 << 20) - 1) >> 20;
		if (host->dev_comp->clk_div_bits == 8)
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD, &mode);
		else
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD_EXTRA, &mode);
		/*DDR mode will double the clk cycles for data timeout */
		timeout = mode >= 2 ? timeout * 2 : timeout;
		timeout = timeout > 1 ? timeout - 1 : 0;
		timeout = timeout > 255 ? 255 : timeout;
	}
	sdr_set_field(host->base + SDC_CFG, SDC_CFG_DTOC, timeout);
}

#ifdef CONFIG_MTK_EMMC_HW_CQ
static void msdc_set_busy_timeout_ms(struct msdc_host *host, u32 ms)
{
	u64 timeout, clk_ns, us;
	u32 mode = 0;

	us = (u64)ms * 1000;

	if (host->sclk == 0) {
		timeout = 0;
	} else {
		clk_ns  = 1000000000ULL;
		do_div(clk_ns, host->sclk);
		timeout = us * 1000 + clk_ns - 1;
		do_div(timeout, clk_ns);
		/* in 1048576 sclk cycle unit */
		timeout = (timeout + (1 << 20) - 1) >> 20;
		if (host->dev_comp->clk_div_bits == 8)
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD, &mode);
		else
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD_EXTRA, &mode);

		/*DDR mode will double the clk cycles for data timeout*/
		timeout = mode >= 2 ? timeout * 2 : timeout;
		timeout = timeout > 1 ? timeout - 1 : 0;
		timeout = timeout > 8191 ? 8191 : timeout;
	}

	sdr_set_field(host->base + SDC_CFG, SDC_CFG_WRDTOC, (u32)timeout);
}
#endif

static void msdc_gate_clock(struct msdc_host *host)
{
	clk_disable_unprepare(host->src_clk_cg);
	clk_disable_unprepare(host->src_clk);
	clk_disable_unprepare(host->h_clk);
}

static void msdc_ungate_clock(struct msdc_host *host)
{
	clk_prepare_enable(host->h_clk);
	clk_prepare_enable(host->src_clk);
	clk_prepare_enable(host->src_clk_cg);
	while (!(readl(host->base + MSDC_CFG) & MSDC_CFG_CKSTB))
		cpu_relax();
}

static void msdc_set_mclk(struct msdc_host *host, unsigned char timing, u32 hz)
{
	u32 mode;
	u32 flags;
	u32 div;
	u32 sclk;
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	if (!hz) {
		dev_dbg(host->dev, "set mclk to 0\n");
		host->mclk = 0;
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
		return;
	}

	flags = readl(host->base + MSDC_INTEN);
	sdr_clr_bits(host->base + MSDC_INTEN, flags);
	if (host->dev_comp->clk_div_bits == 8)
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_HS400_CK_MODE);
	else
		sdr_clr_bits(host->base + MSDC_CFG,
			     MSDC_CFG_HS400_CK_MODE_EXTRA);
	if (timing == MMC_TIMING_UHS_DDR50 ||
	    timing == MMC_TIMING_MMC_DDR52 ||
	    timing == MMC_TIMING_MMC_HS400) {
		if (timing == MMC_TIMING_MMC_HS400)
			mode = 0x3;
		else
			mode = 0x2; /* ddr mode and use divisor */

		if (hz >= (host->src_clk_freq >> 2)) {
			div = 0; /* mean div = 1/4 */
			sclk = host->src_clk_freq >> 2; /* sclk = clk / 4 */
		} else {
			div = (host->src_clk_freq + ((hz << 2) - 1)) / (hz << 2);
			sclk = (host->src_clk_freq >> 2) / div;
			div = (div >> 1);
		}

		if (timing == MMC_TIMING_MMC_HS400 &&
		    hz >= (host->src_clk_freq >> 1)) {
			if (host->dev_comp->clk_div_bits == 8)
				sdr_set_bits(host->base + MSDC_CFG,
					     MSDC_CFG_HS400_CK_MODE);
			else
				sdr_set_bits(host->base + MSDC_CFG,
					     MSDC_CFG_HS400_CK_MODE_EXTRA);
			sclk = host->src_clk_freq >> 1;
			div = 0; /* div is ignore when bit18 is set */
		}
	} else if (hz >= host->src_clk_freq) {
		mode = 0x1; /* no divisor */
		div = 0;
		sclk = host->src_clk_freq;
	} else {
		mode = 0x0; /* use divisor */
		if (hz >= (host->src_clk_freq >> 1)) {
			div = 0; /* mean div = 1/2 */
			sclk = host->src_clk_freq >> 1; /* sclk = clk / 2 */
		} else {
			div = (host->src_clk_freq + ((hz << 2) - 1)) / (hz << 2);
			sclk = (host->src_clk_freq >> 2) / div;
		}
	}
	sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
	/*
	 * As src_clk/HCLK use the same bit to gate/ungate,
	 * So if want to only gate src_clk, need gate its parent(mux).
	 */
	if (host->src_clk_cg)
		clk_disable_unprepare(host->src_clk_cg);
	else
		clk_disable_unprepare(clk_get_parent(host->src_clk));
	if (host->dev_comp->clk_div_bits == 8)
		sdr_set_field(host->base + MSDC_CFG,
			      MSDC_CFG_CKMOD | MSDC_CFG_CKDIV,
			      (mode << 8) | div);
	else
		sdr_set_field(host->base + MSDC_CFG,
			      MSDC_CFG_CKMOD_EXTRA | MSDC_CFG_CKDIV_EXTRA,
			      (mode << 12) | div);
	if (host->src_clk_cg)
		clk_prepare_enable(host->src_clk_cg);
	else
		clk_prepare_enable(clk_get_parent(host->src_clk));

	while (!(readl(host->base + MSDC_CFG) & MSDC_CFG_CKSTB))
		cpu_relax();
	sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
	host->sclk = sclk;
	host->mclk = hz;
	host->timing = timing;
	/* need because clk changed. */
	msdc_set_timeout(host, host->timeout_ns, host->timeout_clks);
	sdr_set_bits(host->base + MSDC_INTEN, flags);

	/*
	 * mmc_select_hs400() will drop to 50Mhz and High speed mode,
	 * tune result of hs200/200Mhz is not suitable for 50Mhz
	 */
	if (host->sclk <= 52000000) {
		writel(host->def_tune_para.iocon, host->base + MSDC_IOCON);
		writel(host->def_tune_para.pad_tune, host->base + tune_reg);
		if (host->top_base != NULL) {
			writel(host->def_tune_para.emmc_top_control,
			       host->top_base + EMMC_TOP_CONTROL);
			writel(host->def_tune_para.emmc_top_cmd,
			       host->top_base + EMMC_TOP_CMD);
		}
	} else {
		writel(host->saved_tune_para.iocon, host->base + MSDC_IOCON);
		writel(host->saved_tune_para.pad_tune, host->base + tune_reg);
		writel(host->saved_tune_para.pad_cmd_tune,
		       host->base + PAD_CMD_TUNE);
		if (host->top_base != NULL) {
			writel(host->saved_tune_para.emmc_top_control,
			       host->top_base + EMMC_TOP_CONTROL);
			writel(host->saved_tune_para.emmc_top_cmd,
			       host->top_base + EMMC_TOP_CMD);
		}
	}

	if (timing == MMC_TIMING_MMC_HS400 &&
	    host->dev_comp->hs400_tune)
		sdr_set_field(host->base + PAD_CMD_TUNE,
			      MSDC_PAD_TUNE_CMDRRDLY,
			      host->hs400_cmd_int_delay);
	dev_dbg(host->dev, "sclk: %d, timing: %d\n", host->sclk, timing);
}

static inline u32 msdc_cmd_find_resp(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	u32 resp;

	switch (mmc_resp_type(cmd)) {
		/* Actually, R1, R5, R6, R7 are the same */
	case MMC_RSP_R1:
		resp = 0x1;
		break;
	case MMC_RSP_R1B:
		resp = 0x7;
		break;
	case MMC_RSP_R2:
		resp = 0x2;
		break;
	case MMC_RSP_R3:
		resp = 0x3;
		break;
	case MMC_RSP_NONE:
	default:
		resp = 0x0;
		break;
	}

	return resp;
}

static inline u32 msdc_cmd_prepare_raw_cmd(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	/* rawcmd :
	 * vol_swt << 30 | auto_cmd << 28 | blklen << 16 | go_irq << 15 |
	 * stop << 14 | rw << 13 | dtype << 11 | rsptyp << 7 | brk << 6 | opcode
	 */
	u32 opcode = cmd->opcode;
	u32 resp = msdc_cmd_find_resp(host, mrq, cmd);
	u32 rawcmd = (opcode & 0x3f) | ((resp & 0x7) << 7);

	host->cmd_rsp = resp;

	if ((opcode == SD_IO_RW_DIRECT && cmd->flags == (unsigned int) -1) ||
	    opcode == MMC_STOP_TRANSMISSION)
		rawcmd |= (0x1 << 14);
	else if (opcode == SD_SWITCH_VOLTAGE)
		rawcmd |= (0x1 << 30);
	else if (opcode == SD_APP_SEND_SCR ||
		 opcode == SD_APP_SEND_NUM_WR_BLKS ||
		 (opcode == SD_SWITCH && mmc_cmd_type(cmd) == MMC_CMD_ADTC) ||
		 (opcode == SD_APP_SD_STATUS && mmc_cmd_type(cmd) == MMC_CMD_ADTC) ||
		 (opcode == MMC_SEND_EXT_CSD && mmc_cmd_type(cmd) == MMC_CMD_ADTC))
		rawcmd |= (0x1 << 11);

	if (cmd->data) {
		struct mmc_data *data = cmd->data;

		if (mmc_op_multi(opcode)) {
			if (mmc_card_mmc(host->mmc->card) && mrq->sbc &&
			    !(mrq->sbc->arg & 0xFFFF0000))
				rawcmd |= 0x2 << 28; /* AutoCMD23 */
		}

		rawcmd |= ((data->blksz & 0xFFF) << 16);
		if (data->flags & MMC_DATA_WRITE)
			rawcmd |= (0x1 << 13);
		if (data->blocks > 1)
			rawcmd |= (0x2 << 11);
		else
			rawcmd |= (0x1 << 11);
		/* Always use dma mode */
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_PIO);

		if (host->timeout_ns != data->timeout_ns ||
		    host->timeout_clks != data->timeout_clks)
			msdc_set_timeout(host, data->timeout_ns,
					data->timeout_clks);

		writel(data->blocks, host->base + SDC_BLK_NUM);
	}
	return rawcmd;
}

static void msdc_start_data(struct msdc_host *host, struct mmc_request *mrq,
			    struct mmc_command *cmd, struct mmc_data *data)
{
	bool read;

	WARN_ON(host->data);
	host->data = data;
	read = data->flags & MMC_DATA_READ;

	mod_delayed_work(system_wq, &host->req_timeout, DAT_TIMEOUT);
	host->req_count++;
#ifdef CONFIG_AMZN_METRICS_LOG
	if (!strcmp(mmc_hostname(host->mmc), SDCARD_HOST_NAME))
		atomic64_inc(&host->mmc->data_count);
#endif
	dev_dbg(host->dev, "crc/total %d/%d invalcrc %d datato %d cmdto %d reqto %d total_pc %d pc_sus %d\n",
			host->crc_count, host->req_count, host->crc_invalid_count, host->datatimeout_count,
			host->cmdtimeout_count, host->reqtimeout_count, host->pc_count, host->pc_suspend);
	msdc_dma_setup(host, &host->dma, data);
	sdr_set_bits(host->base + MSDC_INTEN, data_ints_mask);
	sdr_set_field(host->base + MSDC_DMA_CTRL, MSDC_DMA_CTRL_START, 1);
	dev_dbg(host->dev, "DMA start\n");
	dev_dbg(host->dev, "%s: cmd=%d DMA data: %d blocks; read=%d\n",
			__func__, cmd->opcode, data->blocks, read);
}

static int msdc_auto_cmd_done(struct msdc_host *host, int events,
		struct mmc_command *cmd)
{
	u32 *rsp = cmd->resp;

	rsp[0] = readl(host->base + SDC_ACMD_RESP);

	if (events & MSDC_INT_ACMDRDY) {
		cmd->error = 0;
	} else {
		msdc_reset_hw(host);
		if (events & MSDC_INT_ACMDCRCERR) {
			cmd->error = -EILSEQ;
			host->error |= REQ_STOP_EIO;
		} else if (events & MSDC_INT_ACMDTMO) {
			cmd->error = -ETIMEDOUT;
			host->error |= REQ_STOP_TMO;
		}
		dev_err(host->dev,
			"%s: AUTO_CMD%d arg=%08X; rsp %08X; cmd_error=%d\n",
			__func__, cmd->opcode, cmd->arg, rsp[0], cmd->error);
	}
	return cmd->error;
}

static void msdc_track_cmd_data(struct msdc_host *host,
				struct mmc_command *cmd, struct mmc_data *data)
{
	if (host->error)
		dev_dbg(host->dev, "%s: cmd=%d arg=%08X; host->error=0x%08X\n",
			__func__, cmd->opcode, cmd->arg, host->error);
}

static void msdc_request_done(struct msdc_host *host, struct mmc_request *mrq)
{
	unsigned long flags;

	cancel_delayed_work(&host->req_timeout);
	spin_lock_irqsave(&host->lock, flags);
	host->mrq = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	msdc_track_cmd_data(host, mrq->cmd, mrq->data);
	if (mrq->data)
		msdc_unprepare_data(host, mrq);
	mmc_request_done(host->mmc, mrq);
}

/* returns true if command is fully handled; returns false otherwise */
static bool msdc_cmd_done(struct msdc_host *host, int events,
			  struct mmc_request *mrq, struct mmc_command *cmd)
{
	bool done = false;
	bool sbc_error;
	unsigned long flags;
	u32 *rsp;

	if (mrq->sbc && cmd == mrq->cmd &&
	    (events & (MSDC_INT_ACMDRDY | MSDC_INT_ACMDCRCERR
				   | MSDC_INT_ACMDTMO)))
		msdc_auto_cmd_done(host, events, mrq->sbc);

	sbc_error = mrq->sbc && mrq->sbc->error;

	if (!sbc_error && !(events & (MSDC_INT_CMDRDY
					| MSDC_INT_RSPCRCERR
					| MSDC_INT_CMDTMO)))
		return done;

	spin_lock_irqsave(&host->lock, flags);
	done = !host->cmd;
	host->cmd = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	if (done)
		return true;
	rsp = cmd->resp;

	sdr_clr_bits(host->base + MSDC_INTEN, cmd_ints_mask);

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			rsp[0] = readl(host->base + SDC_RESP3);
			rsp[1] = readl(host->base + SDC_RESP2);
			rsp[2] = readl(host->base + SDC_RESP1);
			rsp[3] = readl(host->base + SDC_RESP0);
		} else {
			rsp[0] = readl(host->base + SDC_RESP0);
		}
	}

	if (!sbc_error && !(events & MSDC_INT_CMDRDY)) {
		if (events & MSDC_INT_CMDTMO ||
		    (cmd->opcode != MMC_SEND_TUNING_BLOCK &&
		     cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200))
			/*
			 * should not clear fifo/interrupt as the tune data
			 * may have alreay come when cmd19/cmd21 gets response
			 * CRC error.
			 */
			msdc_reset_hw(host);
		if (events & MSDC_INT_RSPCRCERR) {
			cmd->error = -EILSEQ;
			host->error |= REQ_CMD_EIO;
			if (FILTER_INVALIDCMD(cmd->opcode))
				host->crc_invalid_count++;
			else
				host->crc_count++;
		} else if (events & MSDC_INT_CMDTMO) {
			cmd->error = -ETIMEDOUT;
			host->error |= REQ_CMD_TMO;
			host->cmdtimeout_count++;
		}
#ifdef CONFIG_AMZN_METRICS_LOG
		if (host->metrics_enable)
			mod_delayed_work(system_wq, &host->metrics_work, METRICS_DELAY);
#endif
	}
	if (cmd->error &&
	    cmd->opcode != MMC_SEND_TUNING_BLOCK &&
	    cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200)
		dev_info(host->dev,
				"%s: cmd=%d arg=%08X; rsp %08X; cmd_error=%d\n",
				__func__, cmd->opcode, cmd->arg, rsp[0],
				cmd->error);

	msdc_cmd_next(host, mrq, cmd);
	return true;
}

/* It is the core layer's responsibility to ensure card status
 * is correct before issue a request. but host design do below
 * checks recommended.
 */
static inline bool msdc_cmd_is_ready(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	/* The max busy time we can endure is 20ms */
	unsigned long tmo = jiffies + msecs_to_jiffies(20);

	while ((readl(host->base + SDC_STS) & SDC_STS_CMDBUSY) &&
			time_before(jiffies, tmo))
		cpu_relax();
	if (readl(host->base + SDC_STS) & SDC_STS_CMDBUSY) {
		dev_err(host->dev, "CMD bus busy detected\n");
		host->error |= REQ_CMD_BUSY;
		msdc_cmd_done(host, MSDC_INT_CMDTMO, mrq, cmd);
		return false;
	}

	if (mmc_resp_type(cmd) == MMC_RSP_R1B || cmd->data) {
		tmo = jiffies + msecs_to_jiffies(20);
		/* R1B or with data, should check SDCBUSY */
		while ((readl(host->base + SDC_STS) & SDC_STS_SDCBUSY) &&
				time_before(jiffies, tmo))
			cpu_relax();
		if (readl(host->base + SDC_STS) & SDC_STS_SDCBUSY) {
			dev_err(host->dev, "Controller busy detected\n");
			host->error |= REQ_CMD_BUSY;
			msdc_cmd_done(host, MSDC_INT_CMDTMO, mrq, cmd);
			return false;
		}
	}
	return true;
}

static void msdc_start_command(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	u32 rawcmd;
	unsigned long flags;

	WARN_ON(host->cmd);
	host->cmd = cmd;

	mod_delayed_work(system_wq, &host->req_timeout, DAT_TIMEOUT);
	if (!msdc_cmd_is_ready(host, mrq, cmd))
		return;

	if ((readl(host->base + MSDC_FIFOCS) & MSDC_FIFOCS_TXCNT) >> 16 ||
	    readl(host->base + MSDC_FIFOCS) & MSDC_FIFOCS_RXCNT) {
		dev_err(host->dev, "TX/RX FIFO non-empty before start of IO. Reset\n");
		msdc_reset_hw(host);
	}

	cmd->error = 0;
	rawcmd = msdc_cmd_prepare_raw_cmd(host, mrq, cmd);
	mod_delayed_work(system_wq, &host->req_timeout, DAT_TIMEOUT);
	host->req_count++;
	spin_lock_irqsave(&host->lock, flags);
	sdr_set_bits(host->base + MSDC_INTEN, cmd_ints_mask);
	spin_unlock_irqrestore(&host->lock, flags);

	writel(cmd->arg, host->base + SDC_ARG);
	writel(rawcmd, host->base + SDC_CMD);
}

static void msdc_cmd_next(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	if ((cmd->error &&
	    !(cmd->error == -EILSEQ &&
	      (cmd->opcode == MMC_SEND_TUNING_BLOCK ||
	       cmd->opcode == MMC_SEND_TUNING_BLOCK_HS200))) ||
	    (mrq->sbc && mrq->sbc->error))
		msdc_request_done(host, mrq);
	else if (cmd == mrq->sbc)
		msdc_start_command(host, mrq, mrq->cmd);
	else if (!cmd->data)
		msdc_request_done(host, mrq);
	else
		msdc_start_data(host, mrq, cmd, cmd->data);
}

static void msdc_ops_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);

	host->error = 0;
	WARN_ON(host->mrq);
	host->mrq = mrq;

	if (mrq->data)
		msdc_prepare_data(host, mrq);

	/* if SBC is required, we have HW option and SW option.
	 * if HW option is enabled, and SBC does not have "special" flags,
	 * use HW option,  otherwise use SW option
	 */
	if (mrq->sbc && (!mmc_card_mmc(mmc->card) ||
	    (mrq->sbc->arg & 0xFFFF0000)))
		msdc_start_command(host, mrq, mrq->sbc);
	else
		msdc_start_command(host, mrq, mrq->cmd);
}

static void msdc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	msdc_prepare_data(host, mrq);
	data->host_cookie |= MSDC_ASYNC_FLAG;
}

static void msdc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
		int err)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_data *data;

	data = mrq->data;
	if (!data)
		return;
	if (data->host_cookie) {
		data->host_cookie &= ~MSDC_ASYNC_FLAG;
		msdc_unprepare_data(host, mrq);
	}
}

static void msdc_data_xfer_next(struct msdc_host *host,
				struct mmc_request *mrq, struct mmc_data *data)
{
	if (mmc_op_multi(mrq->cmd->opcode) && mrq->stop && !mrq->stop->error &&
	    !mrq->sbc)
		msdc_start_command(host, mrq, mrq->stop);
	else
		msdc_request_done(host, mrq);
}

static bool msdc_data_xfer_done(struct msdc_host *host, u32 events,
				struct mmc_request *mrq, struct mmc_data *data)
{
	struct mmc_command *stop;
	unsigned long flags;
	bool done;
	unsigned int check_data = events &
	    (MSDC_INT_XFER_COMPL | MSDC_INT_DATCRCERR | MSDC_INT_DATTMO
	     | MSDC_INT_DMA_BDCSERR | MSDC_INT_DMA_GPDCSERR
	     | MSDC_INT_DMA_PROTECT);
	u32 val;
	int ret;

	spin_lock_irqsave(&host->lock, flags);
	done = !host->data;
	if (check_data)
		host->data = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	if (done)
		return true;
	stop = data->stop;

	if (check_data || (stop && stop->error)) {
		dev_dbg(host->dev, "DMA status: 0x%8X\n",
				readl(host->base + MSDC_DMA_CFG));
		sdr_set_field(host->base + MSDC_DMA_CTRL, MSDC_DMA_CTRL_STOP,
				1);
		ret = readl_poll_timeout_atomic(host->base + MSDC_DMA_CFG, val,
						!(val & MSDC_DMA_CFG_STS), 1, 20000);
		sdr_clr_bits(host->base + MSDC_INTEN, data_ints_mask);
		dev_dbg(host->dev, "DMA stop\n");

		if ((events & MSDC_INT_XFER_COMPL) && (!stop || !stop->error)) {
			data->bytes_xfered = data->blocks * data->blksz;
		} else {
			dev_dbg(host->dev, "interrupt events: %x\n", events);
			msdc_reset_hw(host);
			host->error |= REQ_DAT_ERR;
			data->bytes_xfered = 0;

			if (events & MSDC_INT_DATTMO) {
				data->error = -ETIMEDOUT;
				host->datatimeout_count++;
#ifdef CONFIG_AMZN_METRICS_LOG
				if (!strcmp(mmc_hostname(host->mmc), SDCARD_HOST_NAME))
					atomic64_inc(&host->mmc->data_timeout_count);
#endif
			} else if (events & MSDC_INT_DATCRCERR) {
				data->error = -EILSEQ;
				if (FILTER_INVALIDCMD(mrq->cmd->opcode))
					host->crc_invalid_count++;
				else
					host->crc_count++;
			}
#ifdef CONFIG_AMZN_METRICS_LOG
			if (host->metrics_enable)
				mod_delayed_work(system_wq, &host->metrics_work, METRICS_DELAY);
#endif
			if (data->error != -EILSEQ &&
			    mrq->cmd->opcode != MMC_SEND_TUNING_BLOCK &&
			    mrq->cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200) {
				dev_err(host->dev, "%s: cmd=%d; blocks=%d",
					__func__, mrq->cmd->opcode, data->blocks);
				dev_err(host->dev, "data_error=%d xfer_size=%d\n",
					(int)data->error, data->bytes_xfered);
			}
		}

		msdc_data_xfer_next(host, mrq, data);
		done = true;
	}
	return done;
}

static void msdc_set_buswidth(struct msdc_host *host, u32 width)
{
	u32 val = readl(host->base + SDC_CFG);

	val &= ~SDC_CFG_BUSWIDTH;

	switch (width) {
	default:
	case MMC_BUS_WIDTH_1:
		val |= (MSDC_BUS_1BITS << 16);
		break;
	case MMC_BUS_WIDTH_4:
		val |= (MSDC_BUS_4BITS << 16);
		break;
	case MMC_BUS_WIDTH_8:
		val |= (MSDC_BUS_8BITS << 16);
		break;
	}

	writel(val, host->base + SDC_CFG);
	dev_dbg(host->dev, "Bus Width = %d", width);
}

static int msdc_ops_switch_volt(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret = 0;

	if (!IS_ERR(mmc->supply.vqmmc)) {
		if (ios->signal_voltage != MMC_SIGNAL_VOLTAGE_330 &&
		    ios->signal_voltage != MMC_SIGNAL_VOLTAGE_180) {
			dev_err(host->dev, "Unsupported signal voltage!\n");
			return -EINVAL;
		}

		ret = mmc_regulator_set_vqmmc(mmc, ios);
		if (ret) {
			dev_dbg(host->dev, "Regulator set error %d (%d)\n",
				ret, ios->signal_voltage);
		} else {
			/* Apply different pinctrl settings for different signal voltage */
			if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180)
				pinctrl_select_state(host->pinctrl, host->pins_uhs);
			else
				pinctrl_select_state(host->pinctrl, host->pins_default);
		}
	}
	return ret;
}

static int msdc_card_busy(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 status = readl(host->base + MSDC_PS);

	/* only check if data0 is low */
	return !(status & BIT(16));
}

static void msdc_request_timeout(struct work_struct *work)
{
	struct msdc_host *host = container_of(work, struct msdc_host,
			req_timeout.work);

	/* simulate HW timeout status */
	dev_err(host->dev, "%s: aborting cmd/data/mrq\n", __func__);
	host->reqtimeout_count++;
	if (host->mrq) {
		dev_err(host->dev, "%s: aborting mrq=%p cmd=%d\n", __func__,
				host->mrq, host->mrq->cmd->opcode);
		if (host->cmd) {
			dev_err(host->dev, "%s: aborting cmd=%d\n",
					__func__, host->cmd->opcode);
			msdc_cmd_done(host, MSDC_INT_CMDTMO, host->mrq,
					host->cmd);
		} else if (host->data) {
			dev_err(host->dev, "%s: abort data: cmd%d; %d blocks\n",
					__func__, host->mrq->cmd->opcode,
					host->data->blocks);
			msdc_data_xfer_done(host, MSDC_INT_DATTMO, host->mrq,
					host->data);
		}
	}
}

static void msdc_enable_sdio_irq(struct mmc_host *mmc, int enb)
{
	unsigned long flags;
	struct msdc_host *host = mmc_priv(mmc);

	if (enb)
		pm_runtime_get_sync(host->dev);

	spin_lock_irqsave(&host->lock, flags);
	if (enb)
		sdr_set_bits(host->base + MSDC_INTEN, MSDC_INTEN_SDIOIRQ);
	else
		sdr_clr_bits(host->base + MSDC_INTEN, MSDC_INTEN_SDIOIRQ);
	spin_unlock_irqrestore(&host->lock, flags);

	if (!enb) {
		pm_runtime_mark_last_busy(host->dev);
		pm_runtime_put_autosuspend(host->dev);
	}
}

#ifdef CONFIG_MTK_EMMC_HW_CQ
static irqreturn_t msdc_cmdq_irq(struct msdc_host *host, u32 intsts)
{
	int err = 0;

	if (intsts & MSDC_INT_RSPCRCERR) {
		err = -EILSEQ;
		dev_err(host->dev, "XXX CMD CRC");
	} else if (intsts & MSDC_INT_CMDTMO) {
		err = -ETIMEDOUT;
		dev_err(host->dev, "XXX CMD TIMEOUT");
	}

	if (intsts & MSDC_INT_DATCRCERR) {
		err = -EILSEQ;
		dev_err(host->dev, "XXX DATA CRC");
	} else if (intsts & MSDC_INT_DATTMO) {
		err = -ETIMEDOUT;
		dev_err(host->dev, "XXX DATA TIMEOUT");
	}

	if (err) {
		dev_err(host->dev, "err = %d, intsts = 0x%x", err, intsts);
		writel(intsts, host->base + MSDC_INT); /* clear interrupts */
	}

	return cmdq_irq(host->mmc, err);
}
#endif

static irqreturn_t msdc_irq(int irq, void *dev_id)
{
	struct msdc_host *host = (struct msdc_host *) dev_id;

	while (true) {
		unsigned long flags;
		struct mmc_request *mrq;
		struct mmc_command *cmd;
		struct mmc_data *data;
		u32 events, event_mask;

		spin_lock_irqsave(&host->lock, flags);
		events = readl(host->base + MSDC_INT);
		event_mask = readl(host->base + MSDC_INTEN);
		/* clear interrupts */
		writel(events & event_mask, host->base + MSDC_INT);

		mrq = host->mrq;
		cmd = host->cmd;
		data = host->data;
		spin_unlock_irqrestore(&host->lock, flags);

#ifdef CONFIG_MTK_EMMC_HW_CQ
		if (mmc_card_cmdq(host->mmc->card) &&
		    (events & MSDC_INT_CMDQ)) {
			writel(events, host->base + MSDC_INT);
			msdc_cmdq_irq(host, events);
			break;
	}
#endif

		if ((events & event_mask) & MSDC_INT_SDIOIRQ) {
			msdc_enable_sdio_irq(host->mmc, 0);
			sdio_signal_irq(host->mmc);
		}

		if (!(events & (event_mask & ~MSDC_INT_SDIOIRQ)))
			break;

		if (!mrq) {
			dev_err(host->dev,
				"%s: MRQ=NULL; events=%08X; event_mask=%08X\n",
				__func__, events, event_mask);
			WARN_ON(1);
			break;
		}

		dev_dbg(host->dev, "%s: events=%08X\n", __func__, events);

		if (cmd)
			msdc_cmd_done(host, events, mrq, cmd);
		else if (data)
			msdc_data_xfer_done(host, events, mrq, data);
	}

	return IRQ_HANDLED;
}

static void msdc_init_hw(struct msdc_host *host)
{
	u32 val;
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	/* Configure to MMC/SD mode, clock free running */
	sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_MODE | MSDC_CFG_CKPDN);

	/* Reset */
	msdc_reset_hw(host);

	/* Disable card detection */
	sdr_clr_bits(host->base + MSDC_PS, MSDC_PS_CDEN);

	/* Disable and clear all interrupts */
	writel(0, host->base + MSDC_INTEN);
	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);

	writel(0, host->base + tune_reg);
	if (host->top_base != NULL) {
		writel(0, host->top_base + EMMC_TOP_CONTROL);
		writel(0, host->top_base + EMMC_TOP_CMD);
	}
	writel(0, host->base + MSDC_IOCON);
	sdr_set_field(host->base + MSDC_IOCON, MSDC_IOCON_DDLSEL, 0);
	writel(0x403c0046, host->base + MSDC_PATCH_BIT);
	sdr_set_field(host->base + MSDC_PATCH_BIT, MSDC_CKGEN_MSDC_DLY_SEL,
		      host->dev_comp->ckgen_delay);
	writel(0xffff4089, host->base + MSDC_PATCH_BIT1);
	sdr_set_bits(host->base + EMMC50_CFG0, EMMC50_CFG_CFCSTS_SEL);

	if (host->dev_comp->stop_clk_fix) {
		sdr_set_field(host->base + MSDC_PATCH_BIT1,
			      MSDC_PATCH_BIT1_STOP_DLY, 3);
		sdr_clr_bits(host->base + SDC_FIFO_CFG,
			     SDC_FIFO_CFG_WRVALIDSEL);
		sdr_clr_bits(host->base + SDC_FIFO_CFG,
			     SDC_FIFO_CFG_RDVALIDSEL);
	}

	if (host->dev_comp->busy_check)
		sdr_clr_bits(host->base + MSDC_PATCH_BIT1, (1 << 7));

	if (host->dev_comp->async_fifo) {
		sdr_set_field(host->base + MSDC_PATCH_BIT2,
			      MSDC_PB2_RESPWAIT, 3);
		if (host->dev_comp->enhance_rx) {
			if (host->top_base != NULL)
				sdr_set_bits(host->top_base + EMMC_TOP_CONTROL,
					     SDC_RX_ENH_EN);
			else
				sdr_set_bits(host->base + SDC_ADV_CFG0,
					     SDC_RX_ENHANCE_EN);
		} else {
			sdr_set_field(host->base + MSDC_PATCH_BIT2,
				      MSDC_PB2_RESPSTSENSEL, 2);
			sdr_set_field(host->base + MSDC_PATCH_BIT2,
				      MSDC_PB2_CRCSTSENSEL, 2);
		}
		/* use async fifo, then no need tune internal delay */
		sdr_clr_bits(host->base + MSDC_PATCH_BIT2,
			     MSDC_PATCH_BIT2_CFGRESP);
		sdr_set_bits(host->base + MSDC_PATCH_BIT2,
			     MSDC_PATCH_BIT2_CFGCRCSTS);
	}

	if (host->dev_comp->support_64g)
		sdr_set_bits(host->base + MSDC_PATCH_BIT2,
			     MSDC_PB2_SUPPORT_64G);
	if (host->dev_comp->data_tune) {
		if (host->top_base != NULL) {
			sdr_set_bits(host->top_base + EMMC_TOP_CONTROL,
				     PAD_DAT_RD_RXDLY_SEL);
			sdr_clr_bits(host->top_base + EMMC_TOP_CONTROL,
				     DATA_K_VALUE_SEL);
			sdr_set_bits(host->top_base + EMMC_TOP_CMD,
				     PAD_CMD_RD_RXDLY_SEL);
		} else {
			sdr_set_bits(host->base + tune_reg,
				     MSDC_PAD_TUNE_RD_SEL |
				     MSDC_PAD_TUNE_CMD_SEL);
		}
	} else {
		/* choose clock tune */
		if (host->top_base != NULL)
			sdr_set_bits(host->top_base + EMMC_TOP_CONTROL,
				     PAD_RXDLY_SEL);
		else
			sdr_set_bits(host->base + tune_reg,
				     MSDC_PAD_TUNE_RXDLYSEL);
	}

	/* Configure to enable SDIO mode.
	 * it's must otherwise sdio cmd5 failed
	 */
	sdr_set_bits(host->base + SDC_CFG, SDC_CFG_SDIO);

	/* Config SDIO device detect interrupt function */
	if (host->mmc->caps & MMC_CAP_SDIO_IRQ)
		sdr_set_bits(host->base + SDC_CFG, SDC_CFG_SDIOIDE);
	else
		sdr_clr_bits(host->base + SDC_CFG, SDC_CFG_SDIOIDE);

	/* Configure to default data timeout */
	sdr_set_field(host->base + SDC_CFG, SDC_CFG_DTOC, 3);

	host->def_tune_para.iocon = readl(host->base + MSDC_IOCON);
	host->def_tune_para.pad_tune = readl(host->base + tune_reg);
	host->saved_tune_para.iocon = readl(host->base + MSDC_IOCON);
	host->saved_tune_para.pad_tune = readl(host->base + tune_reg);
	if (host->top_base != NULL) {
		host->def_tune_para.emmc_top_control =
			readl(host->top_base + EMMC_TOP_CONTROL);
		host->def_tune_para.emmc_top_cmd =
			readl(host->top_base + EMMC_TOP_CMD);
		host->saved_tune_para.emmc_top_control =
			readl(host->top_base + EMMC_TOP_CONTROL);
		host->saved_tune_para.emmc_top_cmd =
			readl(host->top_base + EMMC_TOP_CMD);
	}
	dev_dbg(host->dev, "init hardware done!");
}

static void msdc_deinit_hw(struct msdc_host *host)
{
	u32 val;
	/* Disable and clear all interrupts */
	writel(0, host->base + MSDC_INTEN);

	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);
}

/* init gpd and bd list in msdc_drv_probe */
static void msdc_init_gpd_bd(struct msdc_host *host, struct msdc_dma *dma)
{
	struct mt_gpdma_desc *gpd = dma->gpd;
	struct mt_bdma_desc *bd = dma->bd;
	dma_addr_t dma_addr;
	int i;

	memset(gpd, 0, sizeof(struct mt_gpdma_desc) * 2);

	dma_addr = dma->gpd_addr + sizeof(struct mt_gpdma_desc);
	gpd->gpd_info = GPDMA_DESC_BDP; /* hwo, cs, bd pointer */
	/* gpd->next is must set for desc DMA
	 * That's why must alloc 2 gpd structure.
	 */
	gpd->next = lower_32_bits(dma_addr);
	if (host->dev_comp->support_64g)
		gpd->gpd_info |= (upper_32_bits(dma_addr) & 0xf) << 24;

	dma_addr = dma->bd_addr;
	gpd->ptr = lower_32_bits(dma->bd_addr); /* physical address */
	if (host->dev_comp->support_64g)
		gpd->gpd_info |= (upper_32_bits(dma_addr) & 0xf) << 28;

	memset(bd, 0, sizeof(struct mt_bdma_desc) * MAX_BD_NUM);
	for (i = 0; i < (MAX_BD_NUM - 1); i++) {
		dma_addr = dma->bd_addr + sizeof(*bd) * (i + 1);
		bd[i].next = lower_32_bits(dma_addr);
		if (host->dev_comp->support_64g)
			bd[i].bd_info |= (upper_32_bits(dma_addr) & 0xf) << 24;
	}
}

static void msdc_ops_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret;

	msdc_set_buswidth(host, ios->bus_width);

	/* Suspend/Resume will do power off/on */
	switch (ios->power_mode) {
	case MMC_POWER_UP:
		if (!IS_ERR(mmc->supply.vmmc)) {
			msdc_init_hw(host);
			ret = mmc_regulator_set_ocr(mmc, mmc->supply.vmmc,
					ios->vdd);
			if (ret) {
				dev_err(host->dev, "Failed to set vmmc power!\n");
				return;
			}
		}
		break;
	case MMC_POWER_ON:
		if (!IS_ERR(mmc->supply.vqmmc) && !host->vqmmc_enabled) {
			ret = regulator_enable(mmc->supply.vqmmc);
			if (ret)
				dev_err(host->dev, "Failed to set vqmmc power!\n");
			else
				host->vqmmc_enabled = true;
		}
		break;
	case MMC_POWER_OFF:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);

		if (!IS_ERR(mmc->supply.vqmmc) && host->vqmmc_enabled) {
			regulator_disable(mmc->supply.vqmmc);
			host->vqmmc_enabled = false;
			host->pc_count++;
#ifdef CONFIG_AMZN_METRICS_LOG
			if (host->metrics_enable)
				mod_delayed_work(system_wq, &host->metrics_work, METRICS_DELAY);
#endif
			dev_info(host->dev, "crc/total %d/%d invalcrc %d datato %d cmdto %d reqto %d total_pc %d pc_sus %d\n",
				host->crc_count, host->req_count, host->crc_invalid_count, host->datatimeout_count,
				host->cmdtimeout_count, host->reqtimeout_count, host->pc_count, host->pc_suspend);
		}
		break;
	default:
		break;
	}

	if (host->mclk != ios->clock || host->timing != ios->timing)
		msdc_set_mclk(host, ios->timing, ios->clock);
}

static u32 test_delay_bit(u32 delay, u32 bit)
{
	bit %= PAD_DELAY_MAX;
	return delay & (1 << bit);
}

static int get_delay_len(u32 delay, u32 start_bit)
{
	int i;

	for (i = 0; i < (PAD_DELAY_MAX - start_bit); i++) {
		if (test_delay_bit(delay, start_bit + i) == 0)
			return i;
	}
	return PAD_DELAY_MAX - start_bit;
}

static struct msdc_delay_phase get_best_delay(struct msdc_host *host, u32 delay)
{
	int start = 0, len = 0;
	int start_final = 0, len_final = 0;
	u8 final_phase = 0xff;
	struct msdc_delay_phase delay_phase = { 0, };

	if (delay == 0) {
		dev_err(host->dev, "phase error: [map:%x]\n", delay);
		delay_phase.final_phase = final_phase;
		return delay_phase;
	}

	while (start < PAD_DELAY_MAX) {
		len = get_delay_len(delay, start);
		if (len_final < len) {
			start_final = start;
			len_final = len;
		}
		start += len ? len : 1;
		if (len >= 12 && start_final < 4)
			break;
	}

	/* The rule is that to find the smallest delay cell */
	if (start_final == 0)
		final_phase = (start_final + len_final / 3) % PAD_DELAY_MAX;
	else
		final_phase = (start_final + len_final / 2) % PAD_DELAY_MAX;
	dev_info(host->dev, "phase: [map:%x] [maxlen:%d] [final:%d]\n",
		 delay, len_final, final_phase);

	delay_phase.maxlen = len_final;
	delay_phase.start = start_final;
	delay_phase.final_phase = final_phase;
	return delay_phase;
}

static int msdc_tune_resp_data(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 rise_delay = 0, fall_delay = 0;
	struct msdc_delay_phase final_rise_delay, final_fall_delay = { 0,};
	u8 final_delay, final_maxlen;
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	int err;
	int i, j;
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	struct msdc_delay_phase debug_fall_delay = { 0, };
#endif

	sdr_set_field(host->base + MSDC_PATCH_BIT, MSDC_INT_DAT_LATCH_CK_SEL,
		      host->latch_ck);
	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		if (host->top_base != NULL) {
			sdr_set_field(host->top_base + EMMC_TOP_CMD,
				      PAD_CMD_RXDLY, i);
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
				      PAD_DAT_RD_RXDLY, i);
		} else {
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_CMDRDLY, i);
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_DATRRDLY, i);
		}
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters 3 times.
		 */
		for (j = 0; j < 3; j++) {
			err = mmc_send_tuning(mmc, opcode, NULL);
			if (!err) {
				rise_delay |= (1 << i);
			} else {
				rise_delay &= ~(1 << i);
				break;
			}
		}
	}
	final_rise_delay = get_best_delay(host, rise_delay);
	/* if rising edge has enough margin, then do not scan falling edge */
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	if ((final_rise_delay.maxlen >= 12) && !NEED_TUNE_RSP_DAT(host))
#else
	if (final_rise_delay.maxlen >= 12)
#endif
		goto skip_fall;

	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
	for (i = 0; i < PAD_DELAY_MAX; i++) {
		if (host->top_base != NULL) {
			sdr_set_field(host->top_base + EMMC_TOP_CMD,
				      PAD_CMD_RXDLY, i);
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
				      PAD_DAT_RD_RXDLY, i);
		} else {
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_CMDRDLY, i);
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_DATRRDLY, i);
		}

		for (j = 0; j < 3; j++) {
			err = mmc_send_tuning(mmc, opcode, NULL);
			if (!err) {
				fall_delay |= (1 << i);
			} else {
				fall_delay &= ~(1 << i);
				break;
			}
		}
	}
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	if (NEED_TUNE_RSP_DAT(host))
		debug_fall_delay = get_best_delay(host, fall_delay);
	if (final_rise_delay.maxlen < 12)
#endif
		final_fall_delay = get_best_delay(host, fall_delay);

skip_fall:
	final_maxlen = max(final_rise_delay.maxlen, final_fall_delay.maxlen);
	if (final_fall_delay.maxlen >= 12 && final_fall_delay.start < 4)
		final_maxlen = final_fall_delay.maxlen;
	if (final_maxlen == final_rise_delay.maxlen) {
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
		if (host->top_base != NULL) {
			sdr_set_field(host->top_base + EMMC_TOP_CMD,
				      PAD_CMD_RXDLY,
				      final_rise_delay.final_phase);
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
				      PAD_DAT_RD_RXDLY,
				      final_rise_delay.final_phase);
		} else {
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_CMDRDLY,
				      final_rise_delay.final_phase);
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_DATRRDLY,
				      final_rise_delay.final_phase);
		}
		final_delay = final_rise_delay.final_phase;
	} else {
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
		if (host->top_base != NULL) {
			sdr_set_field(host->top_base + EMMC_TOP_CMD,
				      PAD_CMD_RXDLY,
				      final_fall_delay.final_phase);
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
				      PAD_DAT_RD_RXDLY,
				      final_fall_delay.final_phase);
		} else {
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_CMDRDLY,
				      final_fall_delay.final_phase);
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_DATRRDLY,
				      final_fall_delay.final_phase);
		}
		final_delay = final_fall_delay.final_phase;
	}

	dev_info(host->dev, "Final cmd/data pad delay: %x\n", final_delay);
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	/* dump the debug message of rise/fall window */
	if (NEED_TUNE_RSP_DAT(host)) {
		dev_info(host->dev, "[%d]CALIB_RSLT: rise 0x%x,%d/%d, fall 0x%x,%d/%d\n", mmc->index,
			rise_delay, final_rise_delay.final_phase, final_rise_delay.maxlen,
			fall_delay, debug_fall_delay.final_phase, debug_fall_delay.maxlen);
		SET_RSP_DAT_TUNED(host);
	}
#endif
	return final_delay == 0xff ? -EIO : 0;
}

static int msdc_tune_response(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 rise_delay = 0, fall_delay = 0;
	struct msdc_delay_phase final_rise_delay, final_fall_delay = { 0, };
	struct msdc_delay_phase internal_delay_phase = { 0, };
	u8 final_delay, final_maxlen;
	u32 internal_delay = 0;
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	int cmd_err;
	int i, j;
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	struct msdc_delay_phase debug_fall_delay = { 0, };
#endif

	if (mmc->ios.timing == MMC_TIMING_MMC_HS200 ||
	    mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		sdr_set_field(host->base + tune_reg,
			      MSDC_PAD_TUNE_CMDRRDLY,
			      host->hs200_cmd_int_delay);

	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		if (host->top_base != NULL)
			sdr_set_field(host->top_base + EMMC_TOP_CMD,
				      PAD_CMD_RXDLY, i);
		else
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_CMDRDLY, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters 3 times.
		 */
		for (j = 0; j < 3; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err) {
				rise_delay |= (1 << i);
			} else {
				rise_delay &= ~(1 << i);
				break;
			}
		}
	}
	final_rise_delay = get_best_delay(host, rise_delay);
	/* if rising edge has enough margin, then do not scan falling edge */
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	if ((final_rise_delay.maxlen >= 12) && !NEED_TUNE_RSP(host))
#else
	if (final_rise_delay.maxlen >= 12)
#endif
		goto skip_fall;

	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	for (i = 0; i < PAD_DELAY_MAX; i++) {
		if (host->top_base != NULL)
			sdr_set_field(host->top_base + EMMC_TOP_CMD,
				      PAD_CMD_RXDLY, i);
		else
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_CMDRDLY, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters 3 times.
		 */
		for (j = 0; j < 3; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err) {
				fall_delay |= (1 << i);
			} else {
				fall_delay &= ~(1 << i);
				break;
			}
		}
	}
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	if (NEED_TUNE_RSP(host))
		debug_fall_delay = get_best_delay(host, fall_delay);
	if (final_rise_delay.maxlen < 12)
#endif
		final_fall_delay = get_best_delay(host, fall_delay);

skip_fall:
	final_maxlen = max(final_rise_delay.maxlen, final_fall_delay.maxlen);
	if (final_fall_delay.maxlen >= 12 && final_fall_delay.start < 4)
		final_maxlen = final_fall_delay.maxlen;
	if (final_maxlen == final_rise_delay.maxlen) {
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		if (host->top_base != NULL)
			sdr_set_field(host->top_base + EMMC_TOP_CMD,
				      PAD_CMD_RXDLY,
				      final_rise_delay.final_phase);
		else
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_CMDRDLY,
				      final_rise_delay.final_phase);
		final_delay = final_rise_delay.final_phase;
	} else {
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		if (host->top_base != NULL)
			sdr_set_field(host->top_base + EMMC_TOP_CMD,
				      PAD_CMD_RXDLY,
				      final_fall_delay.final_phase);
		else
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_CMDRDLY,
				      final_fall_delay.final_phase);
		final_delay = final_fall_delay.final_phase;
	}
	if (host->dev_comp->async_fifo || host->hs200_cmd_int_delay)
		goto skip_internal;

	for (i = 0; i < PAD_DELAY_MAX; i++) {
		sdr_set_field(host->base + tune_reg,
			      MSDC_PAD_TUNE_CMDRRDLY, i);
		mmc_send_tuning(mmc, opcode, &cmd_err);
		if (!cmd_err)
			internal_delay |= (1 << i);
	}
	dev_dbg(host->dev, "Final internal delay: 0x%x\n", internal_delay);
	internal_delay_phase = get_best_delay(host, internal_delay);
	sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_CMDRRDLY,
		      internal_delay_phase.final_phase);
skip_internal:
	dev_dbg(host->dev, "Final cmd pad delay: %x\n", final_delay);
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	/* dump the debug message of rise/fall window, note that internal is meaningless for us */
	if (NEED_TUNE_RSP(host)) {
		dev_info(host->dev, "[%d]CALIB_CMD: rise 0x%x,%d/%d, fall 0x%x,%d/%d\n", mmc->index,
			rise_delay, final_rise_delay.final_phase, final_rise_delay.maxlen,
			fall_delay, debug_fall_delay.final_phase, debug_fall_delay.maxlen);
		SET_RSP_TUNED(host);
	}
#endif
	return final_delay == 0xff ? -EIO : 0;
}

static int hs400_tune_response(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 cmd_delay = 0;
	struct msdc_delay_phase final_cmd_delay = { 0,};
	u8 final_delay;
	int cmd_err;
	int i, j;

	/* select EMMC50 PAD CMD tune */
	sdr_set_bits(host->base + PAD_CMD_TUNE, BIT(0));
	sdr_set_field(host->base + MSDC_PATCH_BIT1, MSDC_PATCH_BIT1_CMDTA, 2);

	if (mmc->ios.timing == MMC_TIMING_MMC_HS200 ||
	    mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		sdr_set_field(host->base + MSDC_PAD_TUNE,
			      MSDC_PAD_TUNE_CMDRRDLY,
			      host->hs200_cmd_int_delay);

	if (host->hs400_cmd_resp_sel_rising)
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	else
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		sdr_set_field(host->base + PAD_CMD_TUNE,
			      PAD_CMD_TUNE_RX_DLY3, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters 3 times.
		 */
		for (j = 0; j < 3; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err) {
				cmd_delay |= (1 << i);
			} else {
				cmd_delay &= ~(1 << i);
				break;
			}
		}
	}
	final_cmd_delay = get_best_delay(host, cmd_delay);
	sdr_set_field(host->base + PAD_CMD_TUNE, PAD_CMD_TUNE_RX_DLY3,
		      final_cmd_delay.final_phase);
	final_delay = final_cmd_delay.final_phase;

	dev_dbg(host->dev, "Final cmd pad delay: %x\n", final_delay);
	return final_delay == 0xff ? -EIO : 0;
}

static int msdc_tune_data(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 rise_delay = 0, fall_delay = 0;
	struct msdc_delay_phase final_rise_delay, final_fall_delay = { 0, };
	u8 final_delay, final_maxlen;
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	int i, ret;
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	struct msdc_delay_phase debug_fall_delay = { 0, };
#endif

	sdr_set_field(host->base + MSDC_PATCH_BIT, MSDC_INT_DAT_LATCH_CK_SEL,
		      host->latch_ck);
	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		if (host->top_base != NULL)
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
				      PAD_DAT_RD_RXDLY, i);
		else
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_DATRRDLY, i);
		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (!ret)
			rise_delay |= (1 << i);
	}
	final_rise_delay = get_best_delay(host, rise_delay);
	/* if rising edge has enough margin, then do not scan falling edge */
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	if ((final_rise_delay.maxlen >= 12) && !NEED_TUNE_DAT(host))
#else
	if (final_rise_delay.maxlen >= 12)
#endif
		goto skip_fall;

	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
	for (i = 0; i < PAD_DELAY_MAX; i++) {
		if (host->top_base != NULL)
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
				      PAD_DAT_RD_RXDLY, i);
		else
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_DATRRDLY, i);
		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (!ret)
			fall_delay |= (1 << i);
	}
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	if (NEED_TUNE_DAT(host))
		debug_fall_delay = get_best_delay(host, fall_delay);
	if (final_rise_delay.maxlen < 12)
#endif
		final_fall_delay = get_best_delay(host, fall_delay);

skip_fall:
	final_maxlen = max(final_rise_delay.maxlen, final_fall_delay.maxlen);
	if (final_maxlen == final_rise_delay.maxlen) {
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
		if (host->top_base != NULL)
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
				      PAD_DAT_RD_RXDLY,
				      final_rise_delay.final_phase);
		else
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_DATRRDLY,
				      final_rise_delay.final_phase);
		final_delay = final_rise_delay.final_phase;
	} else {
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
		if (host->top_base != NULL)
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
				      PAD_DAT_RD_RXDLY,
				      final_fall_delay.final_phase);
		else
			sdr_set_field(host->base + tune_reg,
				      MSDC_PAD_TUNE_DATRRDLY,
				      final_fall_delay.final_phase);
		final_delay = final_fall_delay.final_phase;
	}

	dev_dbg(host->dev, "Final pad delay: %x\n", final_delay);
#ifdef CONFIG_MMC_TUNE_WINDOW_LOGS
	/* dump the debug message of rise/fall window */
	if (NEED_TUNE_DAT(host)) {
		dev_info(host->dev, "[%d]CALIB_DAT: rise 0x%x,%d/%d, fall 0x%x,%d/%d\n", mmc->index,
			rise_delay, final_rise_delay.final_phase, final_rise_delay.maxlen,
			fall_delay, debug_fall_delay.final_phase, debug_fall_delay.maxlen);
		SET_DAT_TUNED(host);
	}
#endif
	return final_delay == 0xff ? -EIO : 0;
}

static int msdc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret;
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	if (host->dev_comp->tune_resp_data_together) {
		ret = msdc_tune_resp_data(mmc, opcode);
		if (ret == -EIO)
			dev_err(host->dev, "Tune cmd/data fail!\n");
		if (host->hs400_mode) {
			sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
			sdr_clr_bits(host->base + MSDC_IOCON,
				     MSDC_IOCON_W_DSPL);
			if (host->top_base != NULL)
				sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
					      PAD_DAT_RD_RXDLY, 0);
			else
				sdr_set_field(host->base + tune_reg,
					      MSDC_PAD_TUNE_DATRRDLY, 0);
		}
	} else {
		if (host->hs400_mode &&
		    host->dev_comp->hs400_tune)
			ret = hs400_tune_response(mmc, opcode);
		else
			ret = msdc_tune_response(mmc, opcode);
		if (ret == -EIO) {
			dev_err(host->dev, "Tune response fail!\n");
			return ret;
		}
		if (host->hs400_mode == false) {
			ret = msdc_tune_data(mmc, opcode);
			if (ret == -EIO)
				dev_err(host->dev, "Tune data fail!\n");
			if (ret) {
				host->cmd19_fail++;
#ifdef CONFIG_AMZN_METRICS_LOG
			if (host->metrics_enable)
					mod_delayed_work(system_wq, &host->metrics_work, METRICS_DELAY);
#endif
			}
		}
	}

	host->saved_tune_para.iocon = readl(host->base + MSDC_IOCON);
	host->saved_tune_para.pad_tune = readl(host->base + tune_reg);
	host->saved_tune_para.pad_cmd_tune = readl(host->base + PAD_CMD_TUNE);
	if (host->top_base != NULL) {
		host->saved_tune_para.emmc_top_control = readl(host->top_base +
				EMMC_TOP_CONTROL);
		host->saved_tune_para.emmc_top_cmd = readl(host->top_base +
				EMMC_TOP_CMD);
	}
	return ret;
}

static int msdc_prepare_hs400_tuning(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	host->hs400_mode = true;

	if (host->top_base != NULL)
		writel(host->hs400_ds_delay,
			host->top_base + EMMC50_PAD_DS_TUNE);
	else
		writel(host->hs400_ds_delay, host->base + PAD_DS_TUNE);
	/* hs400 mode must set it to 0 */
	sdr_clr_bits(host->base + MSDC_PATCH_BIT2, MSDC_PATCH_BIT2_CFGCRCSTS);
	/* to improve read performance, set outstanding to 2 */
	sdr_set_field(host->base + EMMC50_CFG3, EMMC50_CFG3_OUTS_WR, 2);

	return 0;
}

static void msdc_hw_reset(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	sdr_set_bits(host->base + EMMC_IOCON, 1);
	udelay(10); /* 10us is enough */
	sdr_clr_bits(host->base + EMMC_IOCON, 1);
}

static void msdc_ack_sdio_irq(struct mmc_host *mmc)
{
	msdc_enable_sdio_irq(mmc, 1);
}

#ifdef CONFIG_AMZN_METRICS_LOG
static void msdc_cd_irq(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	host->inserted++;
	if (host->metrics_enable)
		mod_delayed_work(system_wq, &host->metrics_work, METRICS_DELAY);
}
#endif

static const struct mmc_host_ops mt_msdc_ops = {
	.post_req = msdc_post_req,
	.pre_req = msdc_pre_req,
	.request = msdc_ops_request,
	.set_ios = msdc_ops_set_ios,
	.get_ro = mmc_gpio_get_ro,
	.get_cd = mmc_gpio_get_cd,
	.enable_sdio_irq = msdc_enable_sdio_irq,
	.ack_sdio_irq = msdc_ack_sdio_irq,
	.start_signal_voltage_switch = msdc_ops_switch_volt,
	.card_busy = msdc_card_busy,
	.execute_tuning = msdc_execute_tuning,
	.prepare_hs400_tuning = msdc_prepare_hs400_tuning,
	.hw_reset = msdc_hw_reset,
#ifdef CONFIG_AMZN_METRICS_LOG
	.cd_irq = msdc_cd_irq,
#endif
};

static void msdc_of_property_parse(struct platform_device *pdev,
				   struct msdc_host *host)
{
	of_property_read_u32(pdev->dev.of_node, "mediatek,latch-ck",
			     &host->latch_ck);

	of_property_read_u32(pdev->dev.of_node, "hs400-ds-delay",
			     &host->hs400_ds_delay);

	of_property_read_u32(pdev->dev.of_node, "mediatek,hs200-cmd-int-delay",
			     &host->hs200_cmd_int_delay);

	of_property_read_u32(pdev->dev.of_node, "mediatek,hs400-cmd-int-delay",
			     &host->hs400_cmd_int_delay);

	if (of_property_read_bool(pdev->dev.of_node,
				  "mediatek,hs400-cmd-resp-sel-rising"))
		host->hs400_cmd_resp_sel_rising = true;
	else
		host->hs400_cmd_resp_sel_rising = false;
}

#ifdef CONFIG_MTK_EMMC_HW_CQ
static void msdc_cqhci_post_cqe_halt(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	sdr_set_field(host->base + MSDC_DMA_CTRL, MSDC_DMA_CTRL_STOP, 1);
	msdc_reset_hw(host);
}

static void msdc_cqhci_pre_cqe_enable(struct mmc_host *mmc, bool en)
{
	struct msdc_host *host = mmc_priv(mmc);

	if (en) {
		/* enable busy check */
		sdr_set_bits(host->base + MSDC_PATCH_BIT1, BIT(7));
		/* default write data / busy timeout 20 * 1000ms */
		msdc_set_busy_timeout_ms(host, 20 * 1000);
		/* default read data timeout 100ms */
		msdc_set_timeout(host, (100 * 1000 * 1000UL), 0);
	} else {
		/* disable busy check */
		sdr_clr_bits(host->base + MSDC_PATCH_BIT1, BIT(7));
	}
}

static int msdc_cqhci_pre_irq_complete(struct mmc_host *mmc, unsigned int err)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret = 0;
	u32 msdc_int, msdc_int_err = MSDC_INT_DATTMO |
		MSDC_INT_DATCRCERR | MSDC_INT_CMDTMO | MSDC_INT_RSPCRCERR;

	msdc_int = readl(host->base + MSDC_INT);
	if (!err && (msdc_int & msdc_int_err)) {
		writel(msdc_int, host->base + MSDC_INT);
		ret = (unsigned int)-ETIMEDOUT;
		pr_err("%s %d: msdc_int = 0x%x, err = %d\n",
			__func__, __LINE__, msdc_int, err);
	}

	return ret;
}

static const struct cmdq_host_ops msdc_cmdq_ops = {
	.post_cqe_halt = msdc_cqhci_post_cqe_halt,
	.pre_irq_complete = msdc_cqhci_pre_irq_complete,
	.pre_cqe_enable = msdc_cqhci_pre_cqe_enable,
};
#endif
static int msdc_drv_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct msdc_host *host;
	struct resource *res;
	const struct of_device_id *of_id;
	int ret;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

	of_id = of_match_node(msdc_of_ids, pdev->dev.of_node);
	if (!of_id)
		return -EINVAL;
	/* Allocate MMC host for this device */
	mmc = mmc_alloc_host(sizeof(struct msdc_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	ret = mmc_of_parse(mmc);
	if (ret)
		goto host_free;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto host_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	host->top_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->top_base))
		host->top_base = NULL;

	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		goto host_free;

	host->src_clk = devm_clk_get(&pdev->dev, "source");
	if (IS_ERR(host->src_clk)) {
		ret = PTR_ERR(host->src_clk);
		goto host_free;
	}

	host->h_clk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(host->h_clk)) {
		ret = PTR_ERR(host->h_clk);
		goto host_free;
	}

	/*source clock control gate is optional clock*/
	host->src_clk_cg = devm_clk_get(&pdev->dev, "source_cg");
	if (IS_ERR(host->src_clk_cg))
		host->src_clk_cg = NULL;

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = -EINVAL;
		goto host_free;
	}

	host->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(host->pinctrl)) {
		ret = PTR_ERR(host->pinctrl);
		dev_err(&pdev->dev, "Cannot find pinctrl!\n");
		goto host_free;
	}

	host->pins_default = pinctrl_lookup_state(host->pinctrl, "default");
	if (IS_ERR(host->pins_default)) {
		ret = PTR_ERR(host->pins_default);
		dev_err(&pdev->dev, "Cannot find pinctrl default!\n");
		goto host_free;
	}

	host->pins_uhs = pinctrl_lookup_state(host->pinctrl, "state_uhs");
	if (IS_ERR(host->pins_uhs)) {
		ret = PTR_ERR(host->pins_uhs);
		dev_err(&pdev->dev, "Cannot find pinctrl uhs!\n");
		goto host_free;
	}

	msdc_of_property_parse(pdev, host);

	host->dev = &pdev->dev;
	host->dev_comp = of_id->data;
	host->mmc = mmc;
	host->src_clk_freq = clk_get_rate(host->src_clk);
	/* Set host parameters to mmc */
	mmc->ops = &mt_msdc_ops;
	if (host->dev_comp->clk_div_bits == 8)
		mmc->f_min = DIV_ROUND_UP(host->src_clk_freq, 4 * 255);
	else
		mmc->f_min = DIV_ROUND_UP(host->src_clk_freq, 4 * 4095);

	if (mmc->caps & MMC_CAP_SDIO_IRQ)
		mmc->caps2 |= MMC_CAP2_SDIO_IRQ_NOTHREAD;

	mmc->caps |= MMC_CAP_ERASE | MMC_CAP_CMD23;
	/* MMC core transfer sizes tunable parameters */
	mmc->max_segs = MAX_BD_NUM;
	if (host->dev_comp->support_64g)
		mmc->max_seg_size = BDMA_DESC_BUFLEN_EXT;
	else
		mmc->max_seg_size = BDMA_DESC_BUFLEN;
	mmc->max_blk_size = 2048;
	mmc->max_req_size = 512 * 1024;
	mmc->max_blk_count = mmc->max_req_size / 512;
	if (host->dev_comp->support_64g)
		host->dma_mask = DMA_BIT_MASK(36);
	else
		host->dma_mask = DMA_BIT_MASK(32);
	mmc_dev(mmc)->dma_mask = &host->dma_mask;

#ifdef CONFIG_MTK_EMMC_HW_CQ
	if (mmc->caps2 & MMC_CAP2_CQE) {
		host->cq_host = cmdq_pltfm_init(pdev);
		host->cq_host->caps |= CMDQ_TASK_DESC_SZ_128;
		host->cq_host->mmio = host->base + 0x800;
		cmdq_init(host->cq_host, mmc, true);
		host->cq_host->ops = &msdc_cmdq_ops;
		host->cq_host->mmc->max_segs = 128;
		/* cqhci describes data buffer length by 16bits
		 * 0 means 65536 bytes, so we don't have to -1 here
		 */
		mmc->max_seg_size = min_t(unsigned int, mmc->max_seg_size,
					  64 * 1024);
	}
#endif

	host->timeout_clks = 3 * 1048576;
	host->dma.gpd = dma_alloc_coherent(&pdev->dev,
				2 * sizeof(struct mt_gpdma_desc),
				&host->dma.gpd_addr, GFP_KERNEL);
	host->dma.bd = dma_alloc_coherent(&pdev->dev,
				MAX_BD_NUM * sizeof(struct mt_bdma_desc),
				&host->dma.bd_addr, GFP_KERNEL);
	if (!host->dma.gpd || !host->dma.bd) {
		ret = -ENOMEM;
		goto release_mem;
	}
	msdc_init_gpd_bd(host, &host->dma);
	INIT_DELAYED_WORK(&host->req_timeout, msdc_request_timeout);
	spin_lock_init(&host->lock);
#ifdef CONFIG_AMZN_METRICS_LOG
	host->metrics_enable = true;
	INIT_DELAYED_WORK(&host->metrics_work, msdc_metrics_work);
#endif

	platform_set_drvdata(pdev, mmc);
	msdc_ungate_clock(host);
	msdc_init_hw(host);

	ret = devm_request_irq(&pdev->dev, host->irq, msdc_irq,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT, pdev->name, host);
	if (ret)
		goto release;

#ifdef CONFIG_PM_DEVFREQ
	pm_qos_add_request(&host->pm_qos, PM_QOS_VCORE_OPP,
			   PM_QOS_VCORE_OPP_DEFAULT_VALUE);
#endif
	pm_runtime_set_active(host->dev);
	pm_runtime_set_autosuspend_delay(host->dev, MTK_MMC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(host->dev);
	pm_runtime_enable(host->dev);
	msdc_add_device_attrs(host, msdc_attrs);
	ret = mmc_add_host(mmc);

	if (ret)
		goto end;

	return 0;
end:
	pm_runtime_disable(host->dev);
release:
	platform_set_drvdata(pdev, NULL);
	msdc_deinit_hw(host);
	msdc_gate_clock(host);
release_mem:
	if (host->dma.gpd)
		dma_free_coherent(&pdev->dev,
			2 * sizeof(struct mt_gpdma_desc),
			host->dma.gpd, host->dma.gpd_addr);
	if (host->dma.bd)
		dma_free_coherent(&pdev->dev,
			MAX_BD_NUM * sizeof(struct mt_bdma_desc),
			host->dma.bd, host->dma.bd_addr);
host_free:
	mmc_free_host(mmc);

	return ret;
}

static int msdc_drv_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct msdc_host *host;

	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);

	pm_runtime_get_sync(host->dev);

	platform_set_drvdata(pdev, NULL);
	mmc_remove_host(host->mmc);
	msdc_remove_device_attrs(host, msdc_attrs);
	msdc_deinit_hw(host);
	msdc_gate_clock(host);

	pm_runtime_disable(host->dev);
	pm_runtime_put_noidle(host->dev);
	dma_free_coherent(&pdev->dev,
			2 * sizeof(struct mt_gpdma_desc),
			host->dma.gpd, host->dma.gpd_addr);
	dma_free_coherent(&pdev->dev, MAX_BD_NUM * sizeof(struct mt_bdma_desc),
			host->dma.bd, host->dma.bd_addr);

#ifdef CONFIG_PM_DEVFREQ
	pm_qos_remove_request(&host->pm_qos);
#endif
	mmc_free_host(host->mmc);

	return 0;
}

#ifdef CONFIG_PM
static void msdc_save_reg(struct msdc_host *host)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	host->save_para.msdc_cfg = readl(host->base + MSDC_CFG);
	host->save_para.iocon = readl(host->base + MSDC_IOCON);
	host->save_para.sdc_cfg = readl(host->base + SDC_CFG);
	host->save_para.pad_tune = readl(host->base + tune_reg);
	host->save_para.patch_bit0 = readl(host->base + MSDC_PATCH_BIT);
	host->save_para.patch_bit1 = readl(host->base + MSDC_PATCH_BIT1);
	host->save_para.patch_bit2 = readl(host->base + MSDC_PATCH_BIT2);
	host->save_para.pad_ds_tune = readl(host->base + PAD_DS_TUNE);
	host->save_para.pad_cmd_tune = readl(host->base + PAD_CMD_TUNE);
	host->save_para.emmc50_cfg0 = readl(host->base + EMMC50_CFG0);
	host->save_para.emmc50_cfg3 = readl(host->base + EMMC50_CFG3);
	host->save_para.sdc_fifo_cfg = readl(host->base + SDC_FIFO_CFG);
	if (host->top_base != NULL) {
		host->save_para.emmc_top_control =
			readl(host->top_base + EMMC_TOP_CONTROL);
		host->save_para.emmc_top_cmd =
			readl(host->top_base + EMMC_TOP_CMD);
		host->save_para.emmc50_pad_ds_tune =
			readl(host->top_base + EMMC50_PAD_DS_TUNE);

	}
}

static void msdc_restore_reg(struct msdc_host *host)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	writel(host->save_para.msdc_cfg, host->base + MSDC_CFG);
	writel(host->save_para.iocon, host->base + MSDC_IOCON);
	writel(host->save_para.sdc_cfg, host->base + SDC_CFG);
	writel(host->save_para.pad_tune, host->base + tune_reg);
	writel(host->save_para.patch_bit0, host->base + MSDC_PATCH_BIT);
	writel(host->save_para.patch_bit1, host->base + MSDC_PATCH_BIT1);
	writel(host->save_para.patch_bit2, host->base + MSDC_PATCH_BIT2);
	writel(host->save_para.pad_ds_tune, host->base + PAD_DS_TUNE);
	writel(host->save_para.pad_cmd_tune, host->base + PAD_CMD_TUNE);
	writel(host->save_para.emmc50_cfg0, host->base + EMMC50_CFG0);
	writel(host->save_para.emmc50_cfg3, host->base + EMMC50_CFG3);
	writel(host->save_para.sdc_fifo_cfg, host->base + SDC_FIFO_CFG);
	if (host->top_base != NULL) {
		writel(host->save_para.emmc_top_control,
				host->top_base + EMMC_TOP_CONTROL);
		writel(host->save_para.emmc_top_cmd,
				host->top_base + EMMC_TOP_CMD);
		writel(host->save_para.emmc50_pad_ds_tune,
				host->top_base + EMMC50_PAD_DS_TUNE);
	}
}

static int msdc_runtime_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);

	msdc_save_reg(host);
	msdc_gate_clock(host);
#ifdef CONFIG_PM_DEVFREQ
	pm_qos_update_request(&host->pm_qos, VCORE_OPP_UNREQ);
#endif
	return 0;
}

static int msdc_runtime_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);

#ifdef CONFIG_PM_DEVFREQ
	pm_qos_update_request(&host->pm_qos, VCORE_OPP_0);
#endif
	msdc_ungate_clock(host);
	msdc_restore_reg(host);
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int msdc_sys_suspend(struct device *dev)
{
	int ret = pm_runtime_force_suspend(dev);

	if (ret < 0)
		return ret;

	return pinctrl_pm_select_sleep_state(dev);
}

static int msdc_sys_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host =  mmc_priv(mmc);
	int ret = pinctrl_pm_select_default_state(dev);

	if (ret)
		return ret;
	if (host != NULL)
		host->pc_suspend++;

	return pm_runtime_force_resume(dev);
}
#endif

static const struct dev_pm_ops msdc_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(msdc_sys_suspend, msdc_sys_resume)
	SET_RUNTIME_PM_OPS(msdc_runtime_suspend, msdc_runtime_resume, NULL)
};


static struct platform_driver mt_msdc_driver = {
	.probe = msdc_drv_probe,
	.remove = msdc_drv_remove,
	.driver = {
		.name = "mtk-msdc",
		.of_match_table = msdc_of_ids,
		.pm = &msdc_dev_pm_ops,
	},
};

module_platform_driver(mt_msdc_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek SD/MMC Card Driver");
