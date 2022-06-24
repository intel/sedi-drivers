/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _EMMC_5P1_H_
#define _EMMC_5P1_H_

#include <intel/hal_driver_common.h>
#include <intel/hal_emmc.h>

/* Bit Map and length details for Command Register */
#define EMMC_CMD_RESP_TYPE_LOC      0
#define EMMC_CMD_CRC_CHECK_EN_LOC   3
#define EMMC_CMD_IDX_CHECK_EN_LOC   4
#define EMMC_CMD_DATA_PRESENT_LOC   5
#define EMMC_CMD_TYPE_LOC           6
#define EMMC_CMD_INDEX_LOC          8

#define EMMC_CMD_RESP_TYPE_LEN      2
#define EMMC_CMD_CRC_CHECK_EN_LEN   1
#define EMMC_CMD_IDX_CHECK_EN_LEN   1
#define EMMC_CMD_DATA_PRESENT_LEN   1
#define EMMC_CMD_TYPE_LEN           2
#define EMMC_CMD_INDEX_LEN          6

/* Bit Map and length details for Transfer Mode Register */
#define EMMC_XFER_DMA_EN_LOC            0
#define EMMC_XFER_BLOCK_CNT_EN_LOC      1
#define EMMC_XFER_AUTO_CMD_EN_LOC       2
#define EMMC_XFER_DATA_DIR_LOC          4
#define EMMC_XFER_MULTI_BLOCK_SEL_LOC   5


#define EMMC_XFER_DMA_EN_LEN            1
#define EMMC_XFER_BLOCK_CNT_EN_LEN      1
#define EMMC_XFER_AUTO_CMD_EN_LEN       2
#define EMMC_XFER_DATA_DIR_LEN          1
#define EMMC_XFER_MULTI_BLOCK_SEL_LEN   1

/* Bit Map and length details for Block Size and GAP Register */
#define EMMC_BLOCK_SIZE_LOC             0
#define EMMC_SDMA_BUF_SIZE_LOC		12

#define EMMC_BLOCK_SIZE_LEN		12
#define EMMC_SDMA_BUF_SIZE_LEN		3

#define EMMC_BLOCK_GAP_LOC              3
#define EMMC_BLOCK_GAP_LEN              1

/* Bit Map and length details for Normal Interrupt Status Register */
#define EMMC_NRML_INT_STAT_CMD_CMPLT_LOC        0
#define EMMC_NRML_INT_STAT_XFER_CMPLT_LOC       1
#define EMMC_NRML_INT_STAT_BLCK_GAP_LOC         2
#define EMMC_NRML_INT_STAT_DMA_INT_LOC          3
#define EMMC_NRML_INT_STAT_BUF_WR_RDY_LOC       4
#define EMMC_NRML_INT_STAT_BUF_RD_RDY_LOC       5

#define EMMC_NRML_INT_STAT_CMD_CMPLT_LEN        1
#define EMMC_NRML_INT_STAT_XFER_CMPLT_LEN       1
#define EMMC_NRML_INT_STAT_BLCK_GAP_LEN         1
#define EMMC_NRML_INT_STAT_DMA_INT_LEN          1
#define EMMC_NRML_INT_STAT_BUF_WR_RDY_LEN       1
#define EMMC_NRML_INT_STAT_BUF_RD_RDY_LEN       1

/* Bit Map and length details for Clock Control Register */
#define EMMC_CLOCK_CONTROL_SDCLCK_FREQ_SEL_LOC		8
#define EMMC_CLOCK_CONTROL_SDCLCK_FREQ_SEL_UPPER_LOC	6

#define EMMC_CLOCK_CONTROL_SDCLCK_FREQ_SEL_LEN		8
#define EMMC_CLOCK_CONTROL_SDCLCK_FREQ_SEL_UPPER_LEN	2

/* Bit Map for Host Control 1 Register */
#define EMMC_HOST_CTRL1_DAT_WIDTH_LOC		1
#define EMMC_HOST_CTRL1_DMA_SEL_LOC		3
#define EMMC_HOST_CTRL1_EXT_DAT_WIDTH_LOC	5

#define EMMC_HOST_CTRL1_DMA_SEL_LEN		2
#define EMMC_HOST_CTRL1_EXT_DAT_WIDTH_LEN	1

/* The CMD6 EXT_CSD access mode constants. */
#define EMMC_HOST_ACCESS_EXTCSD_COMMAND_SET  (0x0u)
#define EMMC_HOST_ACCESS_EXTCSD_SET_BITS     (0x1u)
#define EMMC_HOST_ACCESS_EXTCSD_CLEAR_BITS   (0x2u)
#define EMMC_HOST_ACCESS_EXTCSD_WRITE_BYTE   (0x3uL)

/* The CMD6 SWITCH command. */
#define EMMC_SWITCH_CMD6_ACCESS_OFFSET  (24u)     /* [25:24] Access */
#define EMMC_SWITCH_CMD6_IDX_OFFSET     (16u)     /* [23:16] Index */
#define EMMC_SWITCH_CMD6_VALUE_OFFSET   (8u)      /* [15:8] Value */
#define EMMC_SWITCH_CMD6_CMD_SET_OFFSET (0u)      /* [2:0] Cmd Set */

#define EMMC_RELIABLE_WRITE_CMD23_OFFSET (31u)

#define CMD_ARG_BUS_WIDTH_EXT_CSD_REG(bus_width)          \
	(EMMC_HOST_ACCESS_EXTCSD_WRITE_BYTE << EMMC_SWITCH_CMD6_ACCESS_OFFSET) | \
	(EMMC_HOST_EXTCSD_BUS_WIDTH_ADDR << EMMC_SWITCH_CMD6_IDX_OFFSET) |	\
	(bus_width << EMMC_SWITCH_CMD6_VALUE_OFFSET) |				\
	(0x0uL << EMMC_SWITCH_CMD6_CMD_SET_OFFSET)

#define CMD_ARG_BUS_SPEED_EXT_CSD_REG(bus_speed)          \
	(EMMC_HOST_ACCESS_EXTCSD_WRITE_BYTE << EMMC_SWITCH_CMD6_ACCESS_OFFSET) |\
	(EMMC_HOST_EXTCSD_HS_TIMING_ADDR << EMMC_SWITCH_CMD6_IDX_OFFSET) |\
	(bus_speed << EMMC_SWITCH_CMD6_VALUE_OFFSET) |\
	(0x0uL << EMMC_SWITCH_CMD6_CMD_SET_OFFSET);

/** Constants Software Reset register */
#define EMMC_SW_RESET_REG_ALL   BIT(0)
#define EMMC_SW_RESET_REG_CMD   BIT(1)
#define EMMC_SW_RESET_REG_DATA  BIT(2)

/* Commands Codes. */
#define EMMC_HOST_CMD0                  0
#define EMMC_HOST_CMD1                  1
#define EMMC_HOST_CMD2                  2
#define EMMC_HOST_CMD3                  3
#define EMMC_HOST_CMD4                  4
#define EMMC_HOST_CMD5                  5
#define EMMC_HOST_CMD6                  6
#define EMMC_HOST_CMD7                  7
#define EMMC_HOST_CMD8                  8
#define EMMC_HOST_CMD9                  9
#define EMMC_HOST_CMD10                 10

#define EMMC_HOST_CMD12                 12
#define EMMC_HOST_CMD13                 13
#define EMMC_HOST_CMD15                 15

#define EMMC_HOST_CMD16                 16
#define EMMC_HOST_CMD17                 17
#define EMMC_HOST_CMD18                 18
#define EMMC_HOST_CMD23                 23
#define EMMC_HOST_CMD24                 24
#define EMMC_HOST_CMD25                 25
#define EMMC_HOST_CMD35                 35
#define EMMC_HOST_CMD36                 36
#define EMMC_HOST_CMD38                 38

/* Constants */
#define EMMC_HOST_RESPONSE_SIZE         4
#define EMMC_HOST_OCR_BUSY_BIT          BIT(31)         /* The Card power up status bit */
#define EMMC_HOST_OCR_CAPACITY_MASK     0x40000000U     /* The OCR sector access mode bit */
#define EMMC_DUAL_VOLTAGE_RANGE         0x40FF8080U     /** voltage 1.8 supported */
#define EMMC_BLOCK_SIZE			512

#define EMMC_HOST_RCA_SHIFT			(16u)   /** RCA of device is at [31:16] */
#define EMMC_HOST_EXTCSD_SEC_COUNT		(53u)   /** Sector Count[212:215] EXTCSD register */
#define EMMC_HOST_EXTCSD_GENERIC_CMD6_TIME	(62u)   /** Generic CMD6 Timeout [248] */
#define EMMC_HOST_EXTCSD_BUS_WIDTH_ADDR      (0xB7uL)   /* Ext CSD Bus Width [183] */
#define EMMC_HOST_EXTCSD_HS_TIMING_ADDR      (0xB9uL)   /* Ext CSD HS Timing [185] */

#define EMMC_HOST_BUS_SPEED_HIGHSPEED   1u

#define EMMC_CMD_COMPLETE_RETRY		1000
#define EMMC_CMD1_RETRY_TIMEOUT		1000
#define EMMC_HOST_CMD6_TIMEOUT_MULT	10

#define EMMC_NORMAL_INTR_MASK		0x7fff
#define EMMC_ERROR_INTR_MASK		0x13ff
#define EMMC_NORMAL_INTR_MASK_CLR	0x60ff

#define EMMC_POWER_CTRL_SD_BUS_POWER	0x1
#define EMMC_POWER_CTRL_SD_BUS_VOLT_SEL	0x5     /* 1.8V voltage select */

#define EMMC_HOST_CTRL2_UHSMODE_SDR12	0x0u
#define EMMC_HOST_CTRL2_UHSMODE_SDR25	0x1u
#define EMMC_HOST_CTRL2_UHSMODE_SDR50	0x2u
#define EMMC_HOST_CTRL2_UHSMODE_SDR104	0x3u
#define EMMC_HOST_CTRL2_UHSMODE_DDR50	0x4u
#define EMMC_HOST_CTRL2_UHSMODE_HS400	0x5u

#define EMMC_HOST_CTRL2_1P8V_SIG_EN	0x1u
#define EMMC_HOST_CTRL2_1P8V_SIG_LOC	3
#define EMMC_HOST_CTRL2_UHS_MODE_SEL_LOC	0
#define EMMC_HOST_CTRL2_UHS_MODE_SEL_LEN	3

/** Below constants is used to read event status from
 *  NORMALINTRSTS Reg
 */
#define EMMC_HOST_CMD_COMPLETE          BIT(0)
#define EMMC_HOST_XFER_COMPLETE         BIT(1)
#define EMMC_HOST_BLOCK_GAP_INTR        BIT(2)
#define EMMC_HOST_DMA_INTR              BIT(3)
#define EMMC_HOST_BUF_WR_READY          BIT(4)
#define EMMC_HOST_BUF_RD_READY          BIT(5)

/** Below constants is used to read event status from
 *  ERRORINSTRSTS Reg
 */
#define EMMC_HOST_CMD_TIMEOUT_ERR       BIT(0)
#define EMMC_HOST_CMD_CRC_ERR           BIT(1)
#define EMMC_HOST_CMD_END_BIT_ERR       BIT(2)
#define EMMC_HOST_CMD_IDX_ERR           BIT(3)
#define EMMC_HOST_DATA_TIMEOUT_ERR      BIT(4)
#define EMMC_HOST_DATA_CRC_ERR          BIT(5)
#define EMMC_HOST_DATA_END_BIT_ERR      BIT(6)
#define EMMC_HOST_CUR_LMT_ERR           BIT(7)
#define EMMC_HOST_ERR                   BIT(12)

/** Constants for PState register */
#define EMMC_HOST_PSTATE_REG_CMD_INHIBIT   BIT(0)
#define EMMC_HOST_PSTATE_REG_DAT_INHIBIT   BIT(1)
#define EMMC_HOST_PSTATE_REG_BUF_READ_EN   BIT(11)

#define EMMC_HOST_MAX_TIMEOUT           0xe

/** Constants for Clock Control register */
#define EMMC_HOST_INTERNAL_CLOCK_EN		BIT(0)
#define EMMC_HOST_INTERNAL_CLOCK_STABLE		BIT(1)
#define EMMC_HOST_SD_CLOCK_EN			BIT(2)

/** Clock frequency */
#define EMMC_HOST_CLK_FREQ_400K		0.4
#define EMMC_HOST_CLK_FREQ_25M		25
#define EMMC_HOST_CLK_FREQ_50M		50

typedef struct {
	__IO_RW uint32_t sdma_sysaddr;          /**< SDMA System Address */
	__IO_RW uint16_t block_size;            /**< Block Size */
	__IO_RW uint16_t block_count;           /**< Block Count */
	__IO_RW uint32_t argument;              /**< Argument */
	__IO_RW uint16_t transfer_mode;         /**< Transfer Mode */
	__IO_RW uint16_t cmd;                   /**< Command */

	__IO_R uint32_t resp_01;                /**< Response Register 0 & 1 */
	__IO_R uint32_t resp_23;                /**< Response Register 2 & 3*/
	__IO_R uint32_t resp_45;                /**< Response Register 4 & 5 */
	__IO_R uint32_t resp_67;                /**< Response Register 6 & 7 */
	__IO_RW uint32_t data_port;             /**< Buffer Data Port */
	__IO_R uint32_t present_state;          /**< Present State */
	__IO_RW uint8_t host_ctrl1;             /**< Host Control 1 */
	__IO_RW uint8_t power_ctrl;             /**< Power Control */
	__IO_RW uint8_t block_gap_ctrl;         /**< Block Gap Control */
	__IO_RW uint8_t wake_up_ctrl;           /**< Wakeup Control */
	__IO_RW uint16_t clock_ctrl;            /**< Clock Control */
	__IO_RW uint8_t timeout_ctrl;           /**< Timeout Control */
	__IO_RW uint8_t sw_reset;               /**< Software Reset */
	__IO_RW uint16_t normal_int_stat;       /**< Normal Interrupt Status */
	__IO_RW uint16_t err_int_stat;          /**< Error Interrupt Status */
	__IO_RW uint16_t normal_int_stat_en;    /**< Normal Interrupt Status Enable */
	__IO_RW uint16_t err_int_stat_en;       /**< Error Interrupt Status Enable */
	__IO_RW uint16_t normal_int_signal_en;  /**< Normal Interrupt Signal Enable */
	__IO_RW uint16_t err_int_signal_en;     /**< Error Interrupt Signal Enable */
	__IO_R uint16_t auto_cmd_err_stat;      /**< Auto CMD Error Status */
	__IO_RW uint16_t host_ctrl2;            /**< Host Control 2 */
	__IO_R uint64_t capabilities;           /**< Capabilities */

	__IO_W uint64_t max_current_cap;        /**< Max Current Capabilities */
	__IO_W uint16_t force_err_autocmd_stat; /**< Force Event for Auto CMD Error Status*/
	__IO_W uint16_t force_err_int_stat;     /**< Force Event for Error Interrupt Status */
	__IO_R uint8_t adma_err_stat;           /**< ADMA Error Status */
	__IO_R uint8_t reserved[3];
	__IO_RW uint32_t adma_sys_addr1;        /**< ADMA System Address1 */
	__IO_RW uint16_t adma_sys_addr2;        /**< ADMA System Address2 */
	__IO_RW uint16_t adma_sys_addr3;        /**< ADMA System Address3 */
	__IO_R uint16_t preset_val_0;           /**< Preset Value 0 */
	__IO_R uint16_t preset_val_1;           /**< Preset Value 1 */
	__IO_R uint16_t preset_val_2;           /**< Preset Value 2 */
	__IO_R uint16_t preset_val_3;           /**< Preset Value 3 */
	__IO_R uint16_t preset_val_4;           /**< Preset Value 4 */
	__IO_R uint16_t preset_val_5;           /**< Preset Value 5 */
	__IO_R uint16_t preset_val_6;           /**< Preset Value 6 */
	__IO_R uint16_t preset_val_7;           /**< Preset Value 7 */
	__IO_R uint32_t boot_timeout;           /**< Boot Timeout */
	__IO_R uint16_t preset_val_8;           /**< Preset Value 8 */
	__IO_R uint16_t reserved3;
	__IO_RW uint16_t vendor_reg;            /**< Vendor Enhanced strobe */
	__IO_R uint16_t reserved4[56];
	__IO_R uint32_t reserved5[4];
	__IO_R uint16_t slot_intr_stat;         /**< Slot Interrupt Status */
	__IO_R uint16_t host_cntrl_version;     /**< Host Controller Version */
	__IO_R uint32_t reserved6[64];
	__IO_R uint32_t cq_ver;                 /**< Command Queue Version */
	__IO_R uint32_t cq_cap;                 /**< Command Queue Capabilities */
	__IO_RW uint32_t cq_cfg;                /**< Command Queue Configuration */
	__IO_RW uint32_t cq_ctrl;               /**< Command Queue Control */
	__IO_RW uint32_t cq_intr_stat;          /**< Command Queue Interrupt Status */
	__IO_RW uint32_t cq_intr_stat_en;       /**< Command Queue Interrupt Status Enable */
	__IO_RW uint32_t cq_intr_sig_en;        /**< Command Queue Interrupt Signal Enable */
	__IO_RW uint32_t cq_intr_coalesc;       /**< Command Queue Interrupt Coalescing */
	__IO_RW uint32_t cq_tdlba;              /**< Command Queue Task Desc List Base Addr */
	__IO_RW uint32_t cq_tdlba_upr;          /**< Command Queue Task Desc List Base Addr Upr */
	__IO_RW uint32_t cq_task_db;            /**< Command Queue Task DoorBell */
	__IO_RW uint32_t cq_task_db_notify;     /**< Command Queue Task DoorBell Notify */
	__IO_R uint32_t cq_dev_qstat;           /**< Command Queue Device queue status */
	__IO_R uint32_t cq_dev_pend_task;       /**< Command Queue Device pending tasks */
	__IO_RW uint32_t cq_task_clr;           /**< Command Queue Task Clr */
	__IO_R uint32_t reserved7;
	__IO_RW uint32_t cq_ssc1;               /**< Command Queue Send Status Configuration 1 */
	__IO_RW uint32_t cq_ssc2;               /**< Command Queue Send Status Configuration 2 */
	__IO_R uint32_t cq_crdct;               /**< Command response for direct command */
	__IO_R uint32_t reserved8;
	__IO_RW uint32_t cq_rmem;               /**< Command response mode error mask */
	__IO_R uint32_t cq_terri;               /**< Command Queue Task Error Information */
	__IO_R uint32_t cq_cri;                 /**< Command Queue Command response index */
	__IO_R uint32_t cq_cra;                 /**< Command Queue Command response argument */
	__IO_R uint32_t reserved9[425];
} emmc_5p1_regs_t;

typedef enum {
	EMMC_HOST_CMD_NORMAL = 0,
	EMMC_HOST_CMD_SUSPEND,
	EMMC_HOST_CMD_RESUME,
	EMMC_HOST_CMD_ABORT,
} emmc_host_cmd_type_t;

typedef enum {
	EMMC_HOST_RESP_NONE = 0,        /* No Response */
	EMMC_HOST_RESP_LEN_136,         /* Response Length 136 bits */
	EMMC_HOST_RESP_LEN_48,          /* Response Length 48 bits */
	EMMC_HOST_RESP_LEN_48B,         /* Response Length 48 bits with busy signal */
} emmc_host_response_type_t;

typedef struct {
	uint32_t cmd_arg;                               /* Argument for the command. */
	uint32_t cmd_idx;                               /* Index of the command. */
	emmc_host_cmd_type_t cmd_type;                  /* Command type*/
	bool data_present;                              /* True if data is present. */
	bool idx_check_en;                              /* Checks the index of the response. */
	bool crc_check_en;                              /* Enables CRC check in the response. */
	emmc_host_response_type_t resp_type;            /* Response type. */
} emmc_host_cmd_config_t;

typedef struct {
	uint32_t block_size;                            /* The size of the data block. */
	uint32_t num_of_block;                          /* The number of blocks to send. */
	bool read;                                      /* true = Read, false = Write. */
	uint32_t *data;                                 /* The pointer to data to send/receive */
	uint32_t data_timeout;                          /* The timeout value for the transfer. */
	bool intr_block_gap_en;                         /* Enables interrupt at the block gap. */
	bool reliable_write_en;                         /* Enables reliable write. */
	bool dma_en;                                    /* DMA enable. */
} emmc_host_data_config_t;


#endif /* _EMMC_5P1_H_ */
