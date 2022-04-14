/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PSE_HW_DEFS_H_
#define _PSE_HW_DEFS_H_

#include "bsp_config.h"

#define WAKEUP_DASHBOARD_IRQ    5       /**< Dashboard IRQ */
#define RESET_PREP_AVAIL_IRQ    6
#define VNN_ACK_IRQ             8
#define PMU_PCIEDEV_IRQ         9
#define PMU_TO_IOAPIC_IRQ       10
#define HBW_PER_FABRIC_IRQ      14
#define SBEP_MSG_RCVD_IRQ       22

#define NVIC_ISER_START         0xE000E100
#define NVIC_ISER_END           0xE000E11C
#define NVIC_ICER_START         0xE000E180
#define NVIC_ICER_END           0xE000E19C
#define NVIC_ISPR_START         0xE000E200
#define NVIC_ISPR_END           0xE000E21C
#define NVIC_ICPR_START         0xE000E280
#define NVIC_ICPR_END           0xE000E29C
#define NVIC_IABR_START         0xE000E300
#define NVIC_IABR_END           0xE000E31C
#define NVIC_IP_START           0xE000E400
#define NVIC_IP_END             0xE000E4EF
#define NVIC_STIR               0xE000EF00
#define SCB_AIRCR               0xE000ED0C
	#define VECTRESET       0x1
	#define SYSRESETREQ     0x4
	#define PRIGRP_MSK      0x700
	#define VECTKEY         0x5FA0000

#define LMT_FB_EN_REG   0x50001068
#define PER0_FB_EN_REG 0x40001068
#define PER1_FB_EN_REG 0x48001068

#define SCB_CM7_ITCMCR          0xE000EF90
#define SCB_CM7_DTCMCR          0xE000EF94
#define SCB_CM7_TCMCR_EN        BIT(0)
#define SCB_CM7_TCMCR_RMW       BIT(1)
#define SCB_CM7_TCMCR_RETEN     BIT(2)

/* CCU */
#define CCU_BASE                0x40600000
#define CCU_TCG_EN              CCU_BASE
	#define CCU_TCG_S0IX_XTAL       1
	#define CCU_TCG_CPU_FAST        2
	#define CCU_TCG_MAIN            4
	#define CCU_TCG_EN_MASK (0xFFFF)
#define CCU_BCG_HPET            (CCU_BASE + 0x4)
#define CCU_BCG_MSK_HPET        (1 << 1)
#define CCU_BCG_ARM_SS          (1 << 0)
#define CCU_BCG_UART            (CCU_BASE + 0x8)
#define CCU_BCG_I2C             (CCU_BASE + 0xc)
#define CCU_BCG_I2C0            (1 << 0)
#define CCU_BCG_I2C1            (1 << 1)
#define CCU_BCG_I2C2            (1 << 2)
#define CCU_BCG_SPI             (CCU_BASE + 0x10)
#define CCU_BCG_SPI1            (1 << 0)
#define CCU_BCG_SPI2            (1 << 1)
#define CCU_BCG_GPIO            (CCU_BASE + 0x14)
#define CCU_BCG_I2S             (CCU_BASE + 0x18)
#define CCU_BCG_TSN             (CCU_BASE + 0x1c)
#define CCU_BCG_SRAM            (CCU_BASE + 0x20)
#define CCU_BCG_QEP             (CCU_BASE + 0x24)
#define CCU_BCG_DMA             (CCU_BASE + 0x28)
#define CCU_BCG_PWM             (CCU_BASE + 0x2c)
#define CCU_BCG_ADC             (CCU_BASE + 0x30)
#define CCU_BCG_CANBUS          (CCU_BASE + 0x34)
#define CCU_RST_HIS             (CCU_BASE + 0x3c)
	#define CCU_RST_HST_COLD_BOOT   0
	#define CCU_RST_HST_SW          (1 << 0)        /* IPC reset */
	#define CCU_RST_HST_WD          (1 << 1)        /* WD exipary */
	#define CCU_RST_HST_MIA         (1 << 2)        /* triple fault */
	#define CCU_RST_HST_ECC         (1 << 3)        /* SRAM ECC reset */
#define CCU_BLK_CG_DIS                  (CCU_BASE + 0x44)
	#define ASYNC_PATH              0x100
#define CCU_TCG_STS                     (CCU_BASE + 0x88)
#define CCU_BCG_GBE                     (CCU_BASE + 0x90)
#define CCU_CLKACK_RISE_INTR_MASK       (CCU_BASE + 0x98)
#define CCU_CLKACK_FALL_INTR_MASK       (CCU_BASE + 0x9C)
#define CCU_BCG_ETHCAT                  (CCU_BASE + 0xa0)
#define CCU_BCG_TGPIO_OCP               (CCU_BASE + 0xcc)
#define CCU_BCG_TGPIO_XTAL              (CCU_BASE + 0xd0)
#define CCU_BCG_TGPIO_PTP               (CCU_BASE + 0xd4)
#define CCU_HW_CLK_SEL                  (CCU_BASE + 0xa8)
#define CCU_CORE_CLK_SEL                (CCU_BASE + 0xd8)
	#define CCU_CLK_SEL_MSK         3       /* Clock select mask */
	#define CCU_CLK_S0IX            0       /* 100MHz - for both
						 * CCU_HW_CLK_SEL & CCU_CORE_CLK_SEL
						 */
	#define CCU_CLK_MAIN_DIV2       1       /* 200MHz - for both
						 * CCU_HW_CLK_SEL & CCU_CORE_CLK_SEL
						 */
	#define CCU_CLK_MAIN            2       /* 400MHz - only for CCU_CORE_CLK_SEL */
	#define CCU_CLK_FAST            3       /* 500MHz - only for CCU_CORE_CLK_SEL */
	#define CCU_CLK_SWTCH_EN        0x100   /* both CCU_HW_CLK_SEL & CCU_CORE_CLK_SEL: Clock
						 * switch enable: FW write 1, HW auto clear
						 */
#define CCU_BCG_I3C             (CCU_BASE + 0xe0)
#define CCU_BCG_SPROLE          (CCU_BASE + 0xe8)
#define CCU_TCG_DIS             (CCU_BASE + 0x40)
#define CCU_BCG_DIS             (CCU_BASE + 0x44)
#define CCU_TCG_RISE_INTR_STS   (CCU_BASE + 0xd0)
#define CCU_TCG_FALL_INTR_STS   (CCU_BASE + 0xd4)

#define HBW_FABRIC_BASE         0x50000000
#define STATUS0_HBW_LOWER       (HBW_FABRIC_BASE + 0x510)
#define ARM_AXI_INBAND_ERR      (1 << 3)
#define ARM_IOSF2AXI_INBAND_ERR (1 << 24)
#define STATUS0_HBW_UPPER       (HBW_FABRIC_BASE + 0x514)
#define STATUS1_HBW_LOWER       (HBW_FABRIC_BASE + 0x530)
#define STATUS1_HBW_UPPER       (HBW_FABRIC_BASE + 0x534)
#define ARM_AXIM_IOCP_IA_AGENT_STATUS_LOWER     (HBW_FABRIC_BASE + 0x1428)
#define INBAND_PRIMARY_ERROR                    (1 << 28)
#define ARM_AXIM_IOCP_IA_ERROR_LOG_LOWER        (HBW_FABRIC_BASE + 0x1458)
#define ARM_AXIM_IOCP_IA_ERROR_LOG_UPPER        (HBW_FABRIC_BASE + 0x145C)
#define ARM_AXIM_IOCP_IA_ERROR_LOG_ADDR_LOWER   (HBW_FABRIC_BASE + 0x1460)
#define IOSF2AXI_IAXI_IA_AGENT_STATUS           (HBW_FABRIC_BASE + 0x9828)
#define IOSF2AXI_VC0_TAXI_TA_AGENT_STS          (HBW_FABRIC_BASE + 0x9C28)
#define IOSF2AXI_VC1_TAXI_TA_AGENT_STS          (HBW_FABRIC_BASE + 0xA028)
#define IOSF2AXI_IAXI_IA_ERR_LOG_L      (HBW_FABRIC_BASE + 0x9858)
#define IOSF2AXI_IAXI_IA_ERR_LOG_H      (HBW_FABRIC_BASE + 0x985C)
#define IOSF2AXI_VC0_TAXI_TA_ERR_LOG_L  (HBW_FABRIC_BASE + 0x9C58)
#define IOSF2AXI_VC0_TAXI_TA_ERR_LOG_H  (HBW_FABRIC_BASE + 0x9C5C)
#define IOSF2AXI_VC1_TAXI_TA_ERR_LOG_L  (HBW_FABRIC_BASE + 0xA058)
#define IOSF2AXI_VC1_TAXI_TA_ERR_LOG_H  (HBW_FABRIC_BASE + 0xA05C)
#define IOSF2AXI_IAXI_IA_ERR_ADDR_L     (HBW_FABRIC_BASE + 0x9860)
#define IOSF2AXI_VC0TAXI_TA_ERR_ADDR_L  (HBW_FABRIC_BASE + 0x9C60)
#define IOSF2AXI_VC1TAXI_TA_ERR_ADDR_L  (HBW_FABRIC_BASE + 0xA060)


#define PER0_FABRIC_IOCP_IA_AGENT_STATUS  (HBW_FABRIC_BASE + 0x1028)
#define PER0_FABRIC_IOCP_IA_ERROT_LOG_LOWER (HBW_FABRIC_BASE + 0x1058)
#define PER0_FABRIC_IOCP_IA_ERROT_LOG_UPPER (HBW_FABRIC_BASE + 0x105C)
#define PER0_FABRIC_IOCP_IA_ERROT_LOG_ADDR (HBW_FABRIC_BASE + 0x1060)

#define PER0_FABRIC_BASE        0x40000000
#define STATUS0_PER0_LOWER      (PER0_FABRIC_BASE + 0x510)
#define STATUS0_PER0_UPPER      (PER0_FABRIC_BASE + 0x514)
#define PER0_INBAND_ERR         (1 << 0)

#define PER1_FABRIC_BASE        0x48000000
#define STATUS0_PER1_LOWER      (PER1_FABRIC_BASE + 0x510)
#define STATUS0_PER1_UPPER      (PER1_FABRIC_BASE + 0x514)

#define DMA_BASE                0x50100000
#define DMA_REG_BASE(n)         (DMA_BASE + 0x4000 * n)

#define DMA_CH_REGS_SIZE        0x58

#define DMA_CH0_REGS(n)         (DMA_REG_BASE(n) + 0 * DMA_CH_REGS_SIZE)
#define DMA_CH1_REGS(n)         (DMA_REG_BASE(n) + 1 * DMA_CH_REGS_SIZE)
#define DMA_CH2_REGS(n)         (DMA_REG_BASE(n) + 2 * DMA_CH_REGS_SIZE)
#define DMA_CH3_REGS(n)         (DMA_REG_BASE(n) + 3 * DMA_CH_REGS_SIZE)
#define DMA_CH4_REGS(n)         (DMA_REG_BASE(n) + 4 * DMA_CH_REGS_SIZE)
#define DMA_CH5_REGS(n)         (DMA_REG_BASE(n) + 5 * DMA_CH_REGS_SIZE)
#define DMA_CH6_REGS(n)         (DMA_REG_BASE(n) + 6 * DMA_CH_REGS_SIZE)
#define DMA_CH7_REGS(n)         (DMA_REG_BASE(n) + 7 * DMA_CH_REGS_SIZE)

#define DMA_SAR                 0x000
#define DMA_DAR                 0x008

#define DMA_LLP                 0x010
#define DMA_LLP_LMS_MASK        0x3
#define DMA_LLP_LMS_SHIFT       0
#define DMA_LLP_LOC_MASK        0xFFFFFFFC
#define DMA_LLP_LOC_SHIFT       2

#define DMA_CTL                 0x018
#define DMA_CTL_INT_EN_BIT      0
#define DMA_CTL_INT_EN_MASK     (1 << DMA_CTL_INT_EN_BIT)

#define DMA_CTL_DST_TR_WIDTH_MASK       0x0000000E
#define DMA_CTL_DST_TR_WIDTH_SHIFT      1
#define DMA_CTL_SRC_TR_WIDTH_MASK       0x00000070
#define DMA_CTL_SRC_TR_WIDTH_SHIFT      4
#define DMA_CTL_TR_WIDTH8               0
#define DMA_CTL_TR_WIDTH16              1
#define DMA_CTL_TR_WIDTH32              2
#define DMA_CTL_TR_WIDTH64              3
#define DMA_CTL_TR_WIDTH128             4
#define DMA_CTL_TR_WIDTH256             5

#define DMA_CTL_DINC_MASK               0x00000180
#define DMA_CTL_DINC_SHIFT              7
#define DMA_CTL_SINC_MASK               0x00000600
#define DMA_CTL_SINC_SHIFT              9
#define DMA_CTL_ADDR_INC                0
#define DMA_CTL_ADDR_DEC                1
#define DMA_CTL_ADDR_CONST              2

#define DMA_CTL_DEST_MSIZE_MASK         0x00003800
#define DMA_CTL_DEST_MSIZE_SHIFT        11
#define DMA_CTL_SRC_MSIZE_MASK          0x0001C000
#define DMA_CTL_SRC_MSIZE_SHIFT         14
#define DMA_CTL_MSIZE1                  0
#define DMA_CTL_MSIZE4                  1
#define DMA_CTL_MSIZE8                  2
#define DMA_CTL_MSIZE16                 3
#define DMA_CTL_MSIZE32                 4
#define DMA_CTL_MSIZE64                 5
#define DMA_CTL_MSIZE128                6
#define DMA_CTL_MSIZE256                7

#define DMA_CTL_SRC_GATHER_EN_BIT       17
#define DMA_CTL_SRC_GATHER_EN_MASK      (1 << DMA_CTL_SRC_GATHER_EN_BIT)
#define DMA_CTL_DST_SCATTER_EN_BIT      18
#define DMA_CTL_DST_SCATTER_EN_MASK     (1 << DMA_CTL_DST_SCATTER_EN_BIT)

#define DMA_CTL_TT_FC_MASK              0x00700000
#define DMA_CTL_TT_FC_SHIFT             20
#define DMA_CTL_TT_FC_M2M_DMAC          0
#define DMA_CTL_TT_FC_M2P_DMAC          1
#define DMA_CTL_TT_FC_P2M_DMAC          2
#define DMA_CTL_TT_FC_P2P_DMAC          3
#define DMA_CTL_TT_FC_P2M_SRCP          4
#define DMA_CTL_TT_FC_P2P_SRCP          5
#define DMA_CTL_TT_FC_M2P_DSTP          6
#define DMA_CTL_TT_FC_P2P_DSTP          7

#define DMA_CTL_DMS_MASK                0x01800000
#define DMA_CTL_DMS_SHIFT               23
#define DMA_CTL_SMS_MASK                0x06000000
#define DMA_CTL_SMS_SHIFT               25
#define DMA_CTL_LLP_DST_EN_BIT          27
#define DMA_CTL_LLP_DST_EN_MASK         (1 << DMA_CTL_LLP_DST_EN_BIT)
#define DMA_CTL_LLP_SRC_EN_BIT          28
#define DMA_CTL_LLP_SRC_EN_MASK         (1 << DMA_CTL_LLP_SRC_EN_BIT)

#define DMA_CTL_BLOCK_TS_MASK           0x00000FFF
#define DMA_CTL_BLOCK_TS_SHIFT          0

#define DMA_CTL_DONE_BIT                44
#define DMA_CTL_DONE_MASK               (1 << (DMA_CTL_DONE_BIT - 32))

#define DMA_CTL_HI_BLOCK_TS_MASK        0x1ffff
#define MISC_DMA_CTL_CH0_READ_RS_POS    0x3
#define MISC_DMA_CTL_CH0_WRITE_RS_POS   0x5

#define DMA_CFG_LO_DST_BURST_ALIGN      (1 << 0)
#define DMA_CFG_LO_SRC_BURST_ALIGN      (1 << 1)
#define DMA_CFG_LO_RD_LLP_SNP           (1 << 14)
#define DMA_CFG_LO_RD_STAT_SNP          (1 << 15)
#define DMA_CFG_LO_WR_STAT_SNP          (1 << 16)
#define DMA_CFG_LO_WR_CTLHI_SNP         (1 << 17)


#define DMA_CFG_CH_SUSP_BIT             8
#define DMA_CFG_CH_SUSP_MASK            (1 << DMA_CFG_CH_SUSP_BIT)
#define DMA_CFG_FIFO_EMPTY_BIT          9
#define DMA_CFG_FIFO_EMPTY_MASK         (1 << DMA_CFG_FIFO_EMPTY_BIT)

#define DMA_CFG_HS_SEL_DST_BIT          10
#define DMA_CFG_HS_SEL_DST_MASK         (1 << DMA_CFG_HS_SEL_DST_BIT)
#define DMA_CFG_HS_SEL_SRC_BIT          11
#define DMA_CFG_HS_SEL_SRC_MASK         (1 << DMA_CFG_HS_SEL_SRC_BIT)
#define DMA_CFG_HS_SEL_HW               0
#define DMA_CFG_HS_SEL_SW               1

#define DMA_CFG_DST_HS_POL_BIT          18
#define DMA_CFG_DST_HS_POL_MASK         (1 << DMA_CFG_DST_HS_POL_BIT)
#define DMA_CFG_SRC_HS_POL_BIT          19
#define DMA_CFG_SRC_HS_POL_MASK         (1 << DMA_CFG_SRC_HS_POL_BIT)
#define DMA_CFG_HS_POL_HIGH             0
#define DMA_CFG_HS_POL_LOW              1

#define DMA_CFG_FCMODE_BIT              0
#define DMA_CFG_FCMODE_MASK             (1 << DMA_CFG_FCMODE_BIT)
#define DMA_CFG_FCMODE_SRC              0
#define DMA_CFG_FCMODE_DST              1

#define DMA_CFG_FIFO_MODE_BIT           1
#define DMA_CFG_FIFO_MODE_MASK          (1 << DMA_CFG_FIFO_MODE_BIT)
#define DMA_CFG_FIFO_MODE_SINGLE        0
#define DMA_CFG_FIFO_MODE_HALF          1

#define DMA_CFG_PROTCTL_BIT             2
#define DMA_CFG_PROTCTL_MASK            (0x7 << DMA_CFG_PROTCTL_BIT)

#define DMA_CFG_DS_UPD_EN_BIT           5
#define DMA_CFG_DS_UPD_EN_MASK          (1 << DMA_CFG_DS_UPD_EN_BIT)

#define DMA_CFG_SS_UPD_EN_BIT           6
#define DMA_CFG_SS_UPD_EN_MASK          (1 << DMA_CFG_SS_UPD_EN_BIT)

#define DMA_CFG_SRC_PER_BIT             7
#define DMA_CFG_SRC_PER_MASK            (1 << DMA_CFG_SRC_PER_BIT)

#define DMA_CFG_DST_PER_BIT             11
#define DMA_CFG_DST_PER_MASK            (1 << DMA_CFG_DST_PER_BIT)

#define DMA_SSTAT               0x020
#define DMA_DSTAT               0x028
#define DMA_SSTATAR             0x030
#define DMA_DSTATAR             0x038
#define DMA_CFG                 0x040
#define DMA_SGR                 0x048
#define DMA_DSR                 0x050

/* Miscellaneous Registers */
#define DMA_CFGREG(n)                   (DMA_REG_BASE(n) + 0x398)
#define DMA_EN_BIT                      0
#define DMA_EN_MASK                     (1 << DMA_EN_BIT)

#define DMA_CHENREG(n)                  (DMA_REG_BASE(n) + 0x3A0)
#define DMA_CH_EN_BIT(n)                (1 << (n))
#define DMA_CH_EN_MASK                  0xFF
#define DMA_CH_EN_WE_BIT(n)             (1 << (8 + (n)))

#define DMA_IDREG(n)                    (DMA_REG_BASE(n) + 0x3A8)
#define DMA_TESTREG(n)                  (DMA_REG_BASE(n) + 0x3B0)

/* Both SRAM bus and DDR bus are 32-bit wide */
#define SRC_WIDTH                       4
#define DEST_WIDTH                      4

/* Maximum 16 dwords in one burst */
#define DMA_MAX_BURST_SIZE      64
/* Maximum block size = 128K-1 we will allow up to 64K */
#define DMA_MAX_BLOCK_SIZE      (0x10000)

#if SRC_WIDTH == 4
#define SRC_TR_WIDTH DMA_CTL_TR_WIDTH32
#if (DMA_MAX_BURST_SIZE / SRC_WIDTH) == 16
#define SRC_BURST_SIZE DMA_CTL_MSIZE16
#endif
#endif /* SRC_WIDTH */

#if DEST_WIDTH == 4
#define DEST_TR_WIDTH DMA_CTL_TR_WIDTH32
#if (DMA_MAX_BURST_SIZE / DEST_WIDTH) == 16
#define DEST_BURST_SIZE DMA_CTL_MSIZE16
#endif
#endif /* DEST_WIDTH */

/* Registers Addresses */
#define DMA_MISC_BASE(n)                (DMA_REG_BASE(n) + 0x1000)

#define DMA_MISC_DMA_CTL_REG(n, ch)     (DMA_MISC_BASE(n) + 4 * (ch))
#define DMA_MISC_SRC_FILLIN_DMA(n, ch)  (DMA_MISC_BASE(n) + 0x100 + (4 * (ch)))
#define DMA_MISC_DST_FILLIN_DMA(n, ch)  (DMA_MISC_BASE(n) + 0x200 + (4 * (ch)))
#define DMA_MISC_CHID_CFG_REG(n)        (DMA_MISC_BASE(n) + 0x400)
#define DMA_MISC_ECC_ERR_SRESP(n)       (DMA_MISC_BASE(n) + 0x404)

#define UPPER_32_BITS(adrs) ((uint32_t)((adrs) >> 32))
#define LOWER_32_BITS(adrs) ((uint32_t)((adrs) & 0x00000000ffffffff))

#define DMA_STS_TFR_REG(n)      (DMA_REG_BASE(n) + 0x2E8)
#define DMA_STS_BLOCK_REG(n)    (DMA_REG_BASE(n) + 0x2F0)
#define DMA_STS_ERR_REG(n)      (DMA_REG_BASE(n) + 0x308)
#define DMA_MASK_TFR_REG(n)     (DMA_REG_BASE(n) + 0x310)
#define DMA_MASK_BLOCK_REG(n)   (DMA_REG_BASE(n) + 0x318)
#define DMA_MASK_ERR_REG(n)     (DMA_REG_BASE(n) + 0x330)
#define DMA_CLR_TFR_REG(n)      (DMA_REG_BASE(n) + 0x338)
#define DMA_CLR_BLOCK_REG(n)    (DMA_REG_BASE(n) + 0x340)
#define DMA_CLR_ERR_REG(n)      (DMA_REG_BASE(n) + 0x358)
#define DMA_EN_REG(n)           (DMA_REG_BASE(n) + 0x3A0)
#define DMA_CFG_REG(n)          (DMA_REG_BASE(n) + 0x398)
#define DMA_STATUS_INT_REG(n)   (DMA_REG_BASE(n) + 0x360)
#define DMA_PSIZE_01(n)         (DMA_REG_BASE(n) + 0x400)
#define DMA_PSIZE_23(n)         (DMA_REG_BASE(n) + 0x404)
#define DMA_PSIZE_45(n)         (DMA_REG_BASE(n) + 0x408)
#define DMA_PSIZE_67(n)         (DMA_REG_BASE(n) + 0x40c)

#define DMA_PSIZE_CHAN0_SIZE    512
#define DMA_PSIZE_CHAN0_OFFSET  0
#define DMA_PSIZE_CHAN1_SIZE    128
#define DMA_PSIZE_CHAN1_OFFSET  13
/* in PSIZE_01 only! */
#define DMA_PSIZE_UPDATE        (1 << 26)

#define DMA_LLP_REG(n, ch)      (DMA_REG_BASE(n) + DMA_CH_REGS_SIZE * (ch) + DMA_LLP)
#define DMA_CTL_LO_REG(n, ch)   (DMA_REG_BASE(n) + DMA_CH_REGS_SIZE * (ch) + DMA_CTL)
#define DMA_CTL_UP_REG(n, ch)   (DMA_REG_BASE(n) + DMA_CH_REGS_SIZE * (ch) + DMA_CTL + 4)
#define DMA_CFG_LO_REG(n, ch)   (DMA_REG_BASE(n) + DMA_CH_REGS_SIZE * (ch) + DMA_CFG)
#define DMA_CFG_UP_REG(n, ch)   (DMA_REG_BASE(n) + DMA_CH_REGS_SIZE * (ch) + DMA_CFG + 4)
#define DMA_DAR_REG(n, ch)      (DMA_REG_BASE(n) + DMA_CH_REGS_SIZE * (ch) + DMA_DAR)
#define DMA_SAR_REG(n, ch)      (DMA_REG_BASE(n) + DMA_CH_REGS_SIZE * (ch) + DMA_SAR)

#define DMA_INT_ERR_MASK        (1 << 4)
#define DMA_INT_BLOCK_MASK      (1 << 1)

/* PSE has 8 DMA channels */
#define DMA_MAX_CHANNEL                 7

#define SIZE_64K                0x10000
#define SRAM_TO_DRAM            0x1
#define DRAM_TO_SRAM            0x2
#define DMAC_RS0                0x0
#define DMAC_RS3                0x3
#define DMAC_NON_SNOOP          ((1 << 8) | (1 << 9))
#define DMAC_RS_RD_OFFSET       0x3
#define DMAC_RS_WR_OFFSET       0x5

#define DMA_ENABLED_MASK                (1 << 0)
#define DMA_IN_PROGRESS_MASK(ch)        (1 << (16 + (ch)))
#define CSR_SRAM_CLAIM                  (1 << 31)
#define PMU_RST_PREP_AVAIL              (1 << 1)

#define WDT_BASE                0x40B00000
#define WDT_WDTC                (WDT_BASE + 0)
#define WDT_WDRP                (WDT_BASE + 4)
#define WDT_WDTV                (WDT_BASE + 8)

/* IPC */
#define IPC_HOST_BASE   0x40400000
#define IPC_SEC_BASE    0x40410000
#define IPC_PMC_BASE    0x40411000

/* Doorbells regs */
#define IPC_HOST2OSE_DOORBELL   (IPC_HOST_BASE + 0x48)
#define IPC_OSE2HOST_DOORBELL   (IPC_HOST_BASE + 0x54)
#define IPC_SEC2OSE_DOORBELL    (IPC_SEC_BASE + 0x48)
#define IPC_OSE2SEC_DOORBELL    (IPC_SEC_BASE + 0x54)
#define IPC_PMC2OSE_DOORBELL    (IPC_PMC_BASE + 0x48)
#define IPC_OSE2PMC_DOORBELL    (IPC_PMC_BASE + 0x54)
/* MSG Regs */
#define IPC_OSE2HOST_MSG_REGS   (IPC_HOST_BASE + 0x60)
#define IPC_HOST2OSE_MSG_REGS   (IPC_HOST_BASE + 0xE0)
#define IPC_OSE2SEC_MSG_REGS    (IPC_SEC_BASE + 0x60)
#define IPC_SEC2OSE_MSG_REGS    (IPC_SEC_BASE + 0xE0)

/* Remap Regs (scratchpad) */
#define IPC_PSE_RMP_REGS        (IPC_HOST_BASE + 0x360)
#define IPC_PSE_RMP0            IPC_PSE_RMP_REGS
#define IPC_PSE_RMP1            (IPC_PSE_RMP0 + 4)
#define IPC_PSE_RMP2            (IPC_PSE_RMP1 + 4)
#define IPC_PSE_RMP3            (IPC_PSE_RMP2 + 4)
#define IPC_PSE_RMP4            (IPC_PSE_RMP3 + 4)
#define IPC_PSE_RMP5            (IPC_PSE_RMP4 + 4)

#define IPC_PISR_2OSE_OFFSET    0
#define IPC_PIMR_2OSE_OFFSET    4
#define IPC_PIMR_OSE2_OFFSET    8
#define IPC_PIMR_CIM_OFFSET     0x10
#define IPC_PISR_CIM_OFFSET     0x14
#define IPC_PIMR_OSE2_BC_BIT    11
#define IPC_PIMR_HOST2OSE       (IPC_HOST_BASE + IPC_PIMR_2OSE_OFFSET)
#define IPC_PIMR_SEC2OSE        (IPC_SEC_BASE + IPC_PIMR_2OSE_OFFSET)
#define IPC_PIMR_OSE2SEC        (IPC_SEC_BASE + IPC_PIMR_OSE2_OFFSET)
#define IPC_PIMR_PMC2OSE        (IPC_PMC_BASE + IPC_PIMR_2OSE_OFFSET)
#define IPC_PIMR_OSE2PMC        (IPC_PMC_BASE + IPC_PIMR_OSE2_OFFSET)

#define IPC_PSE_HOST_FWST_REG           (IPC_HOST_BASE + 0x34)

#define FWST_DOOMSDAY                   0x00001038
#define FWST_DOOMSDAY_N_POOLDB          0x0000b038
#define FWST_INVALID_FUSE               0x0000002c
#define FWST_DMA0_MASK                  (1 << 16)
#define FWST_FAILURE_MASK       0x0000003C
#define FWST_FAIL_DISABLE       0x00000000      /* Disabled */
#define FWST_FAIL_AUTH          0x00000004      /* ISH authentication fail */
#define FWST_FAIL_EXPT          0x00000008      /* exception */
#define FWST_FAIL_INV_REGS      0x0000000C      /* boundary check fail */
#define FWST_FAIL_RST_WAIT      0x00000010      /* Platform reset wait */
#define FWST_FAIL_INV_MSG       0x00000014      /* INV IPC msg */
#define FWST_FAIL_DMA           0x00000018      /* DMA copy failed */
#define FWST_FAIL_OTHER         0x0000001C
#define FWST_FAIL_STACKOVFL     0x00000024
#define FWST_FAIL_INVFUSE       0x0000002C
#define FWST_FAIL_PMC_BUSY      0x00000030
#define FWST_FAIL_NONE          0x00000038
#define FWST_MSK_RST_PRP        0x00000040      /**< New added flag */
#define FWST_MSK_BOOT_PRP       0x00000080      /**< New added flag */
#define FWST_RSTCOUNT_MASK      0x00000F00
#define FWST_MSK_STATE          0x0000F000
#define FWST_STT_AFTER_RST      0x00000000
#define FWST_STT_WAIT_PATCH     0x00001000
#define FWST_RESET_SYNC         0x0000a000


/* Reset- */
#define IPC_PSE_RST_REG                 (IPC_SEC_BASE + 0x44)

/* MISC */
#define MISC_BASE               0x40700000
#define MISC_PSE_FUSE0          (MISC_BASE + 0x50)
	#define PSE_DISABLE_FUSE        (1 << 0)
	#define HVM_FUSE                (1 << 1)
	#define SECURE_FUSE             (1 << 2)
	#define DEBUG_FUSE              (1 << 3)
	#define SX_DISABLED_FUSE        (1 << 7)
	#define LOAD_MODES_MASK         (SECURE_FUSE | DEBUG_FUSE | HVM_FUSE)
#define MISC_SOFTSTRAP          (MISC_BASE + 0x78)
	#define MISC_MASK_CLKSWTCH_ROM  0x2000000       /* bit 25: 1 - ROM switch to 400MHz;
							 *	   0 - BUP switch to 400MHz
							 */
	#define MISC_MASK_DBGLOG_EN     0x4000000       /* bit 26: 1 - Log output enable */
#define RTC_COUNTER0            (MISC_BASE + 0x94)
#define RTC_COUNTER1            (MISC_BASE + 0x98)
#define RTC_CNT_PER_MS          (32768 / 1000)
#define MISC_REVISION_ID                (MISC_BASE + 0x100)
#define MISC_REVISION_ID_A0             0xB
#define MISC_REVISION_ID_B0             0x1
#define FUSA_HBW_FABRIC_PARITY_LOG      (MISC_BASE + 0x800)
#define FUSA_PER0_FABRIC_PARITY_LOG     (MISC_BASE + 0x804)
#define FUSA_PER1_FABRIC_PARITY_LOG     (MISC_BASE + 0x808)
#define FUSA_MEM_PARITY_LOG             (MISC_BASE + 0x80C)
#define FUSA_ARM_DC_ERROR_LOG           (MISC_BASE + 0x810)
#define FUSA_ARM_IC_ERROR_LOG           (MISC_BASE + 0x814)
#define FUSA_BRIDGE_PARITY_LOG0         (MISC_BASE + 0x818)
#define FUSA_BRIDGE_PARITY_LOG1         (MISC_BASE + 0x81C)
#define FUSA_CTRL_REG           (MISC_BASE + 0x820)
#define PARITY_LOGIC_EN         (1 << 1)
#define MISC_NMI_STATUS         (MISC_BASE + 0x908)
#define MISC_NMI_TMOUT          (MISC_BASE + 0x904)
#define MISC_MNI_ALIVE          (MISC_BASE + 0x900)
#define MISC_MNI_ALIVE_CODE     0xDEADBEEF

/* PMU */
#define PMU_BASE                0x40500000
#define PMU_SRAM_PG_EN          (PMU_BASE + 0x00)
#define PMU_PSE_MASK_EVENT      (PMU_BASE + 0x10)
#define PMU_MASK_EVENT_TIMER    (1 << 16)
#define PMU_MASK_EVENT_IPC      (1 << 17)
#define PMU_MASK_EVENT_SBEP     (1 << 26)
#define PMU_PSE_FABRIC_CNT      (PMU_BASE + 0x18)
#define PMU_PSE_FABRIC_CNT_TIMEOUT_MASK 0xffff0000
#define PMU_PSE_FABRIC_CNT_TIMEOUT      50
#define PMU_RF_ROM_PWR_CTRL     (PMU_BASE + 0x30)
#define L2_SRAM_MASK_LSBIT      0x400   /* The least significant bit for L2 SRAM is bit 10
					 * The bit0-bit9 is for the CCM
					 */

#define PMU_VNN_REQ             (PMU_BASE + 0x3c)
#define PMU_VNN_REQ2            (PMU_BASE + 0x6c)

#define PMU_VNN_REQ_ACK                 (PMU_BASE + 0x40)
#define VNN_ACK_OK                      (1 << 0)
#define VNN_REQ_ACK_CLR_RISE            (1 << 2)
#define VNN_REQ_ACK_CLR_FALL            (1 << 3)
#define VNN_REQ_ACK_UNMASK_RISE         (0 << 4)
#define VNN_REQ_ACK_MASK_RISE           (1 << 4)
#define VNN_REQ_ACK_MASK_FALL           (1 << 5)
#define VNN_CLR_AND_MASK_INTERRUPTS     (VNN_REQ_ACK_MASK_RISE |   \
					 VNN_REQ_ACK_MASK_FALL |   \
					 VNN_REQ_ACK_CLR_RISE    | \
					 VNN_REQ_ACK_CLR_FALL)
#define PMU_LDO_CTRL                    (PMU_BASE + 0x44)
#define PMU_LDO_ON_VAL                  (1 << 0)
#define PMU_LDO_RETENTION_ON            (1 << 1)
#define PMU_LDO_READY_MASK              (1 << 3)
#define PMU_PSE_MASK_EVENT2             (PMU_BASE + 0x4c)
#define PMU_MASK_EVENT2_RESET_PREP      (1 << 5)
#define PMU_MASK_EVENT2_PCE_CHANGED     (1 << 18)
#define PMU_PGCB_CLKGATE_CTRL           (PMU_BASE + 0x54)
#define PMU_PGCB_T_CG_TIMER_VALUE_MAX   0xf
#define PMU_RESET_PREP                  (PMU_BASE + 0x5c)
#define PMU_RESET_PREP_GET              0x1
#define PMU_RESET_PREP_AVAIL            0x2
#define PMU_RESET_PREP_MASK             (1 << 31)
#define PMU_FABRIC_SNAPSHOT             (PMU_BASE + 0x60)
#define PMU_FABRIC_SNAPSHOT_HSU_ACTIVE  (1 << 4)
#define PMU_CG_PG_STATUS                (PMU_BASE + 0x68)
#define PMU_PCE_D3_STATUS_0             (PMU_BASE + 0x100)
#define PMU_PCE_D3_STATUS_1             (PMU_BASE + 0x104)
	#define PMU_PCE_PMCREN          (1 << 0)
	#define PMU_PCE_IDLEN           (1 << 1)
	#define PMU_PCE_D3HEN           (1 << 2)
	#define PMU_PCE_SLEEPEN         (1 << 3)
	#define PMU_PCE_HWAEN           (1 << 4)
	#define PMU_PCE_SHADOW_MASK     0x1f

	#define PMU_PCE_PMCRE_MASK      (1 << 8)
	#define PMU_PCE_IDLEN_MASK      (1 << 9)
	#define PMU_PCE_D3HEN_MASK      (1 << 10)
	#define PMU_PCE_SE_MASK         (1 << 11)
	#define PMU_PCE_PG_ALLOWED_MASK (1 << 12)

	#define PMU_D3_LIVESTS                  (1 << 16)
	#define PMU_D3_RISING_EDGE_STATUS       (1 << 17)
	#define PMU_D3_FALLING_EDGE_STATUS      (1 << 18)
	#define PMU_D3_RISING_EDGE_MASK         (1 << 19)
	#define PMU_D3_FALLING_EDGE_MASK        (1 << 20)
	#define PMU_D3_MASK                     (0xf << 17)

	#define PMU_D0I3_CIP_LIVE_STATUS        (1 << 21)
	#define PMU_D0I3_LIVE_STATUS            (1 << 22)
	#define PMU_D0I3_ENABLE_MASK            (1 << 23)

	#define PMU_BME_LIVESTS                 (1 << 24)
	#define PMU_BME_RISING_EDGE_STATUS      (1 << 25)
	#define PMU_BME_FALLING_EDGE_STATUS     (1 << 26)
	#define PMU_BME_RISING_EDGE_MASK        (1 << 27)
	#define PMU_BME_FALLING_EDGE_MASK       (1 << 28)
	#define PMU_BME_MASK            (0xf << 25)

#define PMU_PCE_SHADOW                  PMU_PCE_D3_STATUS_0
#define PMU_D3_STATUS                   PMU_PCE_D3_STATUS_0

#define PMU_BRIDGE_INTR_MASK_D3_RISE    (PMU_BASE + 0x200)
#define PMU_BRIDGE_INTR_MASK_D3_FALL    (PMU_BASE + 0x208)
#define PMU_BRIDGE_INTR_MASK_BME_RISE   (PMU_BASE + 0x220)
#define PMU_BRIDGE_INTR_MASK_BME_FALL   (PMU_BASE + 0x228)
#define PMU_INTR_MASK_GPIO0             (PMU_BASE + 0x250)
#define PMU_MASK_GPIO0                  0xffffffff
#define PMU_INTR_MASK_GPIO1             (PMU_BASE + 0x254)
#define PMU_MASK_GPIO1                  0x7
#define PMU_UART_IDLE_MASK              (PMU_BASE + 0x2d8)
#define PMU_UART_IDLE_MASK_ALL          0x3f
#define PMU_CLK_SWITCH_INTR             (PMU_BASE + 0x2c4)
	#define PMU_CLK_SWITCH_INTR_STS         1
	#define PMU_CLK_SWITCH_INTR_MASK        2

#define PMU_STATUS_REG                  (PMU_BASE + 0xf00)
#define PMU_STATUS_PG_BIT               (1 << 4)
#define PMU_PWR_STATUS_D0IX_STATE_2     0x2
#define PMU_PWR_STATUS_D0IX_STATE_3     0x3
#define PMU_SCRATCHPAD0                 (PMU_BASE + 0xf04)
#define PMU_SCRATCHPAD1                 (PMU_BASE + 0xf08)
#define PMU_PG_EN_REG                   (PMU_BASE + 0xf10)
#define PMU_PG_EN_BIT                   (1 << 0)
#define PMU_PG_EXIT_COMPLETE_BIT        (1 << 8)
#define PMU_SW_PG_REQ_INTR              (PMU_BASE + 0xf14)
#define PMU_SW_PG_REQ_B_LIVESTS         (1 << 0)
#define PMU_SW_PG_REQ_B_INTR_RISE       (1 << 1)
#define PMU_SW_PG_REQ_B_INTR_FALL       (1 << 2)
#define PMU_PMC_PG_WAKE_INTR            (PMU_BASE + 0xf18)
#define PMU_PMC_PG_WAKE_LIVESTS         (1 << 0)
#define PMU_PMC_PG_WAKE_INTR_RISE       (1 << 1)
#define PMU_PMC_PG_WAKE_INTR_FALL       (1 << 2)
#define PMU_HOST_RST_CTL                (PMU_BASE + 0xf20)
#define PMU_HOST_RST_B                  (1 << 0)
#define PMU_HOST_RST_B_INTR_RISE        (1 << 1)
#define PMU_HOST_RST_B_INTR_FALL        (1 << 2)
#define PMU_BRIDGE_ISOL                 (PMU_BASE + 0xF24)
#define PMU_BR_ISOL_REQ                 (1 << 0)
#define PMU_BR_ISOL_ACK                 (1 << 1)
#define PMU_CDC_CGDIS                   (PMU_BASE + 0xf28)
#define PMU_CDC_CGDIS_DIS               (1 << 0)
#define PMU_PCE_LOCAL                   (PMU_BASE + 0xf30)
#define PMU_PCE_LOCAL_CHG_DET_INTR_STS  (1 << 8)
#define PMU_PCE_SHADOW_MASK             0x1f
#define PMU_TOTAL_PCI_FUNCTION          36
#define PMU_IPC_PCI_DEV_FUNC            35
#define PMU_SRAM_BITLINE_FLOAT_EN       (PMU_BASE + 0xf34)
#define PMU_SRAM_BITLINE_FLOAT_EN_ALL   0x3ffffff
#define PMU_SRAM_ARSLEEP_EN             (PMU_BASE + 0xf38)
#define PMU_SRAM_ARSLEEP_EN_ALL         0x3ffffff

#define PMU_D3_BME_BITS()       (read32(PMU_D3_STATUS) & \
				 (PMU_D0I3_ENABLE_MASK | PMU_D3_LIVESTS | PMU_BME_LIVESTS))

/* SB */
#define SBEP_BASE                       0x40800000
/* Registers for upstream request */
#define SBEP_DOWN_MSG_REQ_ATTR          (SBEP_BASE + 0x0000)
#define SBEP_DOWN_MSG_SRC_ID_MASK       0xFF00
#define SBEP_DOWN_MSG_SRC_ID_LOC        8
#define SBEP_BOOT_PREP_ACK_ATTR         0xF2900
#define SBEP_DOWN_MSG_REQ_ADDR_LOW      (SBEP_BASE + 0x0004)
#define SBEP_DOWN_MSG_REQ_EH            (SBEP_BASE + 0x000C)
#define SBEP_DOWN_MSG_DATA0             (SBEP_BASE + 0x0010)
#define SBEP_DOWN_MSG_STATUS            (SBEP_BASE + 0x0014)
#define SBEP_DOWN_INT_MASK              (1 << 30)
#define SBEP_DOWN_MSG_RCVD              (1 << 31)
#define SBEP_DOWN_CMPL_CTRL             (SBEP_BASE + 0x0028)
#define SBEP_DOWN_CMPL_INT_MASK         (1 << 30)
#define SBEP_UP_MSG_STATUS              (SBEP_BASE + 0x0040)
#define UP_STATUS_BUSY_MASK             0x01
#define UP_STATUS_MSG_SENT_MASK         0x02
#define UP_STATUS_MSG_SENT_CLR          0x02
#define SBEP_UP_MSG_COMMAND             (SBEP_BASE + 0x0044)
#define SBEP_UP_INT_MASK                (1 << 3)
#define SBEP_UP_MSG_REQ_ADDR_LOW        (SBEP_BASE + 0x0048)
#define SBEP_UP_MSG_REQ_ADDR_HIGH       (SBEP_BASE + 0x004C)
#define SBEP_UP_MSG_REQ_DATA            (SBEP_BASE + 0x0050)

/* Generic Registers required for upstream messages */
#define SBEP_UP_MSG_REQ_ATTR    (SBEP_BASE + 0x0054)
#define SBEP_UP_MSG_REQ_EH      (SBEP_BASE + 0x0058)

/* Upstream completion registers */
#define SBEP_UP_MSG_CMPL_DATA           (SBEP_BASE + 0x005C)
#define SBEP_UP_MSG_CMPL_STATUS         (SBEP_BASE + 0x0060)
#define SBEP_UP_GENERATE_CMPL           0x80000000
#define SBEP_UP_MSG_CMPL_RSP_EH         (SBEP_BASE + 0x0064)
#define SBEP_BOOT_PREP_CTRL             (SBEP_BASE + 0x0084)
#define SBEP_BOOT_PREP_RCVD             (1 << 0)
#define SBEP_CLK_GATE_ENABLE            (SBEP_BASE + 0x006C)
#define SBEP_SIDECLK_GATE               0x3

/* UPSTREAM_COMMAND_REG values */
#define SBEP_CMD_ACTION         0x1
#define SBEP_CMD_TYPE_WRITE     0x0
#define SBEP_CMD_POSTED         0x1
#define SBEP_CMD_INT_DISABLED   0x0

/* UPSTREAM_COMMAND_REG offsets */
#define SBEP_CMD_ACTION_OFF     0
#define SBEP_CMD_TYPE_OFF       1
#define SBEP_CMD_POSTED_OFF     2
#define SBEP_CMD_INT_OFF        3

#define SBEP_CMD_VAL    ((SBEP_CMD_ACTION << SBEP_CMD_ACTION_OFF) |	   \
			 (SBEP_CMD_TYPE_WRITE << SBEP_CMD_TYPE_OFF)      | \
			 (SBEP_CMD_POSTED << SBEP_CMD_POSTED_OFF)        | \
			 (SBEP_CMD_INT_DISABLED << SBEP_CMD_INT_OFF))

#define SBEP_DOWN_MSG_DATA1             (SBEP_BASE + 0x0074)

/* HPET timer */
#define HPET_BASE               0x40a00000
#define HPET_GCFG_LOW           (HPET_BASE + 0x10)
#define HPET_GCFG_ENABLE_CNF    (1 << 0)

/* GPIO */
#define GPIO0_BASE              0x40200000
#define GPIO_GRER0              (GPIO0_BASE + 0x64)
#define GPIO_GFER0              (GPIO0_BASE + 0x7c)
#define GPIO_GISR0              (GPIO0_BASE + 0xc4)

#define SRAM_BASE               0x60000000UL
#define SRAM_SIZE               0x100000        /* 1280K L2 SRAM*/
#define SRAM_BANK_SIZE          (0x8000)

/*
 * bank addresses.
 * for example: bank 5 starts at BANK_N_START(5) and
 * ends at (=address of last byte is) BANK_N_END(5)
 */
#define BANK_N_START(n)         (SRAM_BASE + SRAM_BANK_SIZE * (n))
#define BANK_N_END(n)           (BANK_N_START(n) + SRAM_BANK_SIZE - 1)

/*------------------------------------------------------------
 *---- SRAM Registers ----
 *------------------------------------------------------------
 */
#define SRAM_CTRL_BASE                          0x50400000
#define SRAM_CTRL_SCFGR                         (SRAM_CTRL_BASE + 0x00)
	#define SRAM_ECC_DISABLE                0x10    /* ECC Correction Disable */
#define SRAM_CTRL_INTR                          (SRAM_CTRL_BASE + 0x04)
#define SRAM_CTRL_INTR_MASK                     (SRAM_CTRL_BASE + 0x08)
#define SRAM_CTRL_ICCM_ERASE_CTRL               (SRAM_CTRL_BASE + 0x0C)
#define SRAM_CTRL_CCM_ERASE_SIZE_BIT            2
#define SRAM_CTRL_ERASE_START_BIT               BIT(0)
#define SRAM_CTRL_ICCM_ERASE_ADDR               (SRAM_CTRL_BASE + 0x10)
#define SRAM_CTRL_BANK_STATUS                   (SRAM_CTRL_BASE + 0x2c)
#define SRAM_CTRL_CCM_LIMIT                     (SRAM_CTRL_BASE + 0x40)
#define SRAM_CTRL_CCM_LIMIT_BIT                 20
#define SRAM_CTRL_CCM_NUM_ENTRIES               28
#define SRAM_CTRL_CCM_BANK_TABLE0               (SRAM_CTRL_BASE + 0x50)
#define SRAM_CTRL_CCM_BANK_TABLE_PBLI_BIT       0
#define SRAM_CTRL_CCM_BANK_TABLE_PBHI_BIT       8
#define SRAM_CTRL_CCM_BANK_TABLE_LB_BIT         16
#define SRAM_CTRL_CCM_BANK_TABLE_PBTYP_BIT      24
#define SRAM_CTRL_CCM_BANK_TABLE_PBTYP_HI_BIT   28
#define SRAM_CTRL_L2_ERASE_CTRL         (SRAM_CTRL_BASE + 0x200)
#define SRAM_CTRL_L2_ERASE_SIZE_SHIFT           3 /* QDWORD */
#define SRAM_CTRL_L2_ERASE_SIZE_BIT             2
#define SRAM_CTRL_L2_ERASE_START_BIT            BIT(0)
#define SRAM_CTRL_L2_ERASE_ADDR                 (SRAM_CTRL_BASE + 0x204)
#define SRAM_CTRL_L2_ERASE_SIZE_SET(size)	      \
	(((size) >> SRAM_CTRL_L2_ERASE_SIZE_SHIFT) << \
		SRAM_CTRL_L2_ERASE_SIZE_BIT)
#define SRAM_CTRL_DCCM_ERASE_CTRL               (SRAM_CTRL_BASE + 0x220)
#define SRAM_CTRL_DCCM_ERASE_ADDR               (SRAM_CTRL_BASE + 0x224)

#define SRAM_CTRL_INTR_MASK_BITMAP      0xFFFFFFFF
#define SRAM_WAIT_LOOP_CNT              30

/* [ROM FAS], Figure 2: SRAM Controller Registers & Banks */
#define BANK_STATUS_MASK_CCM    (0x3FF)
#define BANK_STATUS_MASK_L2     (0x3FFFC00)

#define CCM_SIZE        (PSE_CONFIG_CCM_SIZE + PSE_CONFIG_REDUNDANT_CCM_SIZE)
#define ICCM_BASE       PSE_CONFIG_ICCM_BASE
#define IMR_SIZE_LIMIT  0x400000        /* IMR size limitation is 4MB according to spec */

#endif
