/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SEDI_I2S_H_
#define _SEDI_I2S_H_

#define I2S_BIT_HIGH (1)
#define I2S_BIT_LOW (0)
#define I2S_SET_BIT (1)
#define I2S_CLEAR_BIT (0)

#define TDM_ACTIVE_TX_6_CHNL_SLOT (0x3F)
#define TDM_ACTIVE_RX_6_CHNL_SLOT (0x3F)
#define TDM_ACTIVE_TX_8_CHNL_SLOT (0xFF)
#define TDM_ACTIVE_RX_8_CHNL_SLOT (0xFF)

#define SEDI_I2S_DRIVER_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

#define I2S_RESET_VAL (0x1B8UL)

#define I2S_TX_FIFO 0x040
#define I2S_RX_FIFO 0x040

#define TX_FIFO_LEVEL 0x20
#define RX_FIFO_LEVEL 0x20
#define KEEP_FIFO_LEVEL_EMPTY (TX_FIFO_LEVEL - 0x03)

#define TX_FIFO_AFULL 0x20
#define RX_FIFO_AFULL 0x10

#define TX_FIFO_AEMPTY 0x10
#define RX_FIFO_AEMPTY 0

#define INTR_STAT_FIFO_EMPTY BIT(2)
#define INTR_STAT_FIFO_AEMPTY BIT(3)
#define INTR_STAT_FIFO_FULL BIT(4)
#define INTR_STAT_FIFO_AFULL BIT(5)
#define INTR_STAT_RFIFO_EMPTY BIT(16)
#define INTR_STAT_RFIFO_AEMPTY BIT(17)
#define INTR_STAT_RFIFO_FULL BIT(18)
#define INTR_STAT_RFIFO_AFULL BIT(19)

#define I2S_INTR_I2S_MASK BIT(26)
#define I2S_INTR_TX_EMPTY BIT(27)
#define I2S_INTR_TX_AEMPTY BIT(28)
#define I2S_INTR_TX_FULL BIT(29)
#define I2S_INTR_TX_AFULL BIT(30)
#define I2S_INTR_MASK						      \
	(I2S_INTR_I2S_MASK | I2S_INTR_TX_EMPTY | I2S_INTR_TX_AEMPTY | \
	 I2S_INTR_TX_FULL | I2S_INTR_TX_AFULL)

#define ENABLE_I2S_INTR (0)

#define I2S_12_BIT_SAMPLE (12)
#define I2S_16_BIT_SAMPLE (16)
#define I2S_24_BIT_SAMPLE (24)
#define I2S_32_BIT_SAMPLE (32)
#define BIT_SAMPLE_MAX (4)

#define I2S_8K_SFR (8000)
#define I2S_11K_SFR (11025)
#define I2S_16K_SFR (16000)
#define I2S_22K_SFR (22050)
#define I2S_32K_SFR (32000)
#define I2S_44K_SFR (44100)
#define I2S_48K_SFR (48000)
#define I2S_88K_SFR (88200)
#define I2S_96K_SFR (96000)
#define I2S_176K_SFR (176400)
#define I2S_192K_SFR (192000)
#define SFR_MAX (11)

#define I2S_CHNL_MONO (1)
#define I2S_CHNL_STERIO (2)
#define I2S_CHNL_6 (6)
#define I2S_CHNL_8 (8)
#define I2S_CHNL_MAX (4)

#define I2S_CHNL_WIDTH_12 (1)
#define I2S_CHNL_WIDTH_16 (2)
#define I2S_CHNL_WIDTH_24 (5)
#define I2S_CHNL_WIDTH_32 (7)

#define SAMPLE_RATE_CAL_ITRN (2)
#define SAMPLE_FREQ_LOOKUP_TABLE (0)

typedef union {

	struct {
		uint32_t i2s_en : 1;
		uint32_t dir_cfg : 1;
		uint32_t ms_cfg : 1;
		uint32_t sfr_rst : 1;
		uint32_t fifo_rst : 1;
		uint32_t chn_width : 3;
		uint32_t ws_mode : 4;
		uint32_t mono_mode : 1;
		uint32_t audio_mode : 1;
		uint32_t sck_polar : 1;
		uint32_t ws_polar : 1;
		uint32_t data_ws_del : 5;
		uint32_t data_align : 1;
		uint32_t data_order : 1;
		uint32_t host_data_align : 1;
		uint32_t i2s_stb : 1;
		uint32_t intreq_mask : 1;
		uint32_t i2s_mask : 1;
		uint32_t fifo_empty_mask : 1;
		uint32_t fifo_aempty_mask : 1;
		uint32_t fifo_full_mask : 1;
		uint32_t fifo_afull_mask : 1;
		uint32_t full_duplex : 1;
	};
	uint32_t bits;
} i2s_ctrl_t;

typedef union {

	struct {
		uint32_t res_0_to_3 : 4;
		uint32_t fifo_rst : 1;
		uint32_t res_5_to_25 : 21;
		uint32_t ri2s_mask : 1;
		uint32_t rfifo_empty_mask : 1;
		uint32_t rfifo_aempty_mask : 1;
		uint32_t rfifo_full_mask : 1;
		uint32_t rfifo_afull_mask : 1;
		uint32_t res_31 : 1;
	};
	uint32_t bits;
} i2s_ctrl_fdr_t;

typedef union {

	struct {
		uint32_t resolution : 5;
		uint32_t res_5_to_31 : 27;
	};
	uint32_t bits;
} i2s_sres_t;

typedef union {

	struct {
		uint32_t rresolution : 5;
		uint32_t res_5_to_31 : 27;
	};
	uint32_t bits;
} i2s_sres_fdr_t;

typedef union {

	struct {
		uint32_t sample_rate : 20;
		uint32_t res_21_to_31 : 12;
	};
	uint32_t bits;
} i2s_srate_t;

typedef volatile union {

	struct {
		uint32_t tdata_underr : 1;
		uint32_t rdata_ovrerr : 1;
		uint32_t fifo_empty : 1;
		uint32_t fifo_aempty : 1;
		uint32_t fifo_full : 1;
		uint32_t fifo_afull : 1;
		uint32_t res_6_to_15 : 10;
		uint32_t rfifo_empty : 1;
		uint32_t rfifo_aempty : 1;
		uint32_t rfifo_full : 1;
		uint32_t rfifo_afull : 1;
		uint32_t res_20_to_31 : 12;
	};
	uint32_t bits;
} i2s_stat_t;

typedef union {

	struct {
		uint32_t tdm_en : 1;
		uint32_t chn_no : 4;
		uint32_t res_5_to_15 : 11;
		uint32_t ch_slot : 16;
	};
	uint32_t bits;
} i2s_tdm_ctrl_t;

typedef union {

	struct {
		uint32_t ch_tx_slot : 16;
		uint32_t ch_rx_slot : 16;
	};
	uint32_t bits;
} i2s_tdm_fd_dir_t;

typedef struct {
	__IO_RW i2s_ctrl_t ctrl;
	__IO_RW i2s_ctrl_fdr_t ctrl_fdr;
	__IO_RW i2s_sres_t sres;
	__IO_RW i2s_sres_fdr_t sres_fdr;
	__IO_RW i2s_srate_t srate;
	__IO_RW i2s_stat_t stat;
	__IO_RW uint32_t fifo_level;
	__IO_RW uint32_t fifo_aempty;
	__IO_RW uint32_t fifo_afull;
	__IO_RW uint32_t fifo_level_fdr;
	__IO_RW uint32_t fifo_aempty_fdr;
	__IO_RW uint32_t fifo_afull_fdr;
	__IO_RW i2s_tdm_ctrl_t tdm_ctrl;
	__IO_RW i2s_tdm_fd_dir_t tdm_fd_dir;
} i2s_regs_t;

typedef union {
	struct {
		uint32_t val : 24;
		uint32_t res_24_31 : 8;
	};
	uint32_t bits;
} i2s_mn_div_val_t;
typedef union {
	struct {
		uint32_t enable : 1;
		uint32_t res_01_31 : 31;
	};
	uint32_t bits;
} i2s_mn_div_enable_t;

typedef struct {
	__IO_RW i2s_mn_div_val_t mval;
	__IO_RW i2s_mn_div_val_t nval;
	__IO_RW i2s_mn_div_enable_t mn_div_en;
} i2s_mn_div_regs_t;

typedef struct {
	i2s_mn_div_val_t mval;
	i2s_mn_div_val_t nval;
	i2s_mn_div_enable_t mn_div_en;
} i2s_mn_div_t;

typedef struct {
	i2s_mn_div_regs_t *regs;
} i2s_mn_div_resource_t;

struct stream {
	uint32_t *src;
	uint32_t *dst;
	uint32_t size;
	uint32_t cur_idx;
};

typedef struct {
	i2s_regs_t *regs;
	i2s_config_t i2s_config;
	uint32_t tx_dma_channel;
	uint32_t rx_dma_channel;
	i2s_ctrl_t ctrl;
	i2s_ctrl_fdr_t ctrl_fdr;
	i2s_tdm_ctrl_t tdm_ctrl;
	i2s_tdm_fd_dir_t tdm_fd_dir;
	i2s_mn_div_t i2s_mn_div;
	struct stream tx;
	struct stream rx;
} i2s_resource_t;

#endif
