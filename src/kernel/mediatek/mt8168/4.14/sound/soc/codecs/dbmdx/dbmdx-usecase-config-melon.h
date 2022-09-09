/*
 * dbmdx-usecase-config-melon.h  --  Preset USE CASE configurations
 *
 * Copyright (C) 2021 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef DBMDX_MELON_USECASES_SUPPORTED

#ifndef _DBMDX_USECASE_CONFIG_MELON_H
#define _DBMDX_USECASE_CONFIG_MELON_H

#include "dbmdx-interface.h"
#include "dbmdx-usecase-config-def.h"

#define DBMDX_MELON_OPTIMIZED 1
#define NONE_OPTIMIZED_TL_FREQ_VT_CORE_MELON	124000000

int uc_lp_model_config_dummy(struct dbmdx_private *p,
				struct usecase_config *uc);

int melon_uc_lp_model_config(struct dbmdx_private *p,
				struct usecase_config *uc);

int melon_uc_nr_model_config(struct dbmdx_private *p,
				struct usecase_config *uc);

int melon_uc_production_model_config(struct dbmdx_private *p,
				struct usecase_config *uc);

int uc_load_models_general(struct dbmdx_private *p,
					struct usecase_config *uc);

int uc_load_models_unload_all(struct dbmdx_private *p,
					struct usecase_config *uc);

int uc_stop_all_models(struct dbmdx_private *p,
					struct usecase_config *uc);

static struct usecase_config config_uc_melon_idle = {
	.usecase_name = "uc_melon_idle",
	.id	= (DBMDX_USECASE_ID_UC_IDX_FF |
			DBMDX_USECASE_ID_PRJ_MELON |
			DBMDX_USECASE_ID_HWREV_00),
	.hw_rev = 0,
	.change_clock_src = true,
	.tdm_clock_freq = TDM_CLK_FREQ_48,
	.number_of_bits = 16,
#ifdef FW_AUTO_CONFIGURE_CLOCK
	.clock_op_va = DBMDX_CLOCK_OP_DO_NOT_CONFIG,
#else
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK,
#endif
	.clock_config_va = {
			.wanted_pll = 81000000,
			.wanted_tl3_clk = 81000000,
			.wanted_ahb_clk = 0,
			.wanted_apb_clk = 0,
			.use_pll_post_div = true,
		},
	.usecase_requires_amodel = false,
	.usecase_sets_detection_mode = false,
	.usecase_supports_us_buffering = false,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,

	.num_of_va_cfg_values = 0,
	.num_of_va_ve_cfg_values = 0,
	.va_start_cmd_type = START_CMD_TYPE_OPMODE,
	.va_start_cmd = (DBMDX_IDLE),
	.va_ve_start_cmd_type = START_CMD_TYPE_OPMODE,
	.send_va_ve_start_cmd = false,
	.va_ve_start_cmd = (DBMDX_IDLE),
};




/* Melon Low Power Mode Usecase */
static struct usecase_config config_uc_melon_low_power = {
	.usecase_name = "uc_melon_low_power",
	.id	= (DBMDX_USECASE_ID_UC_IDX_00 |
			DBMDX_USECASE_ID_PRJ_MELON |
			DBMDX_USECASE_ID_HWREV_00),
	.hw_rev = 0,
	.change_clock_src = true,
	.tdm_clock_freq = TDM_CLK_FREQ_48,
	.number_of_bits = 16,
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK,
	.clock_config_va = {
			.wanted_pll = 81000000,
#ifndef DBMDX_MELON_OPTIMIZED
			.wanted_tl3_clk = 98000000,
#else
			.wanted_tl3_clk = 20000000,
#endif
			.wanted_tl3_clk_debug = 98000000,
			.wanted_ahb_clk = 0,
			.wanted_apb_clk = 0,
			.use_pll_post_div = true,
		},
	.usecase_requires_amodel = true,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
	.model_cp = 0,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.complex_clb_1 = uc_load_models_general,
	.complex_clb_2 = melon_uc_lp_model_config,
	.num_of_output_channels = 1,

	.va_cfg_values = (u32 []){
		(DBMDX_REGN_VT1_REGS_OFFSET |
			DBMDX_REGN_VT_REG_LPSD | 0x0001),
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_1),
		(DBMDX_REGN_UART_SPEED |
			DBMDX_REGV_UART_BAUD_RATE_460_800_hz),
#ifndef FW_AUTO_CONFIGURE_CLOCK
		(DBMDX_UC_SEQ_CMD_CHANGE_CLK_SRC),
#endif
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_2),
		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
			DBMDX_REGV_NO_STREAM_CH_2 |
			DBMDX_REGV_STREAM_CH_1_CP_0),
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG | 0x1000),
		(DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE |
			DBMDX_REGV_USE_PHRASE_LEN_FROM_WWE |
			DBMDX_REGV_NORMALIZE_TO_MINUS_6dB),
		(DBMDX_UC_SEQ_CMD_CONFIG_MICS),
	},
#ifdef FW_AUTO_CONFIGURE_CLOCK
	.num_of_va_cfg_values = 8,
#else
	.num_of_va_cfg_values = 9,
#endif
	.config_mics = MIC_CONFIG_BY_USECASE,

	.mic_config = {
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
#else
		(DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_SAR_IIR_FILTER_128 |
#ifdef DBMDX_SAR_ADC_SELECT_SEC
		 DBMDX_REGV_SAR_ADC_SEL_SEC |
#else
		 DBMDX_REGV_SAR_ADC_SEL_MAIN |
#endif
		DBMDX_REGV_DDF_AND_DM_CONFIG_SD_DDF_SD_ADC),
#endif
		0x0000,
		0x0000,
		0x0000},

#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
	.mic_freq = { 1536000, 0, 0, 0 },
#else
	.mic_freq = { 512000, 0, 0, 0 },
#endif
	.num_of_va_ve_cfg_values = 0,

	.audio_routing_config = {
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,},

	.tdm_configs_va = {
		/* DBMD4 TDM0_TX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM0_RX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM1_TX is disabled */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM1_RX is disabled */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},

	},
	.va_start_cmd_type = START_CMD_TYPE_OPMODE,
	.send_va_start_cmd = true,
	.va_start_cmd = (DBMDX_DETECTION),

	.va_ve_start_cmd_type = START_CMD_TYPE_OPMODE,
	.send_va_ve_start_cmd = false,
	.va_ve_start_cmd = (DBMDX_IDLE),
};

/***********************************************************************/

static struct usecase_config config_uc_melon_production = {
	.usecase_name = "uc_melon_production",
	.id	= (DBMDX_USECASE_ID_UC_IDX_08 |
			DBMDX_USECASE_ID_PRJ_MELON |
			DBMDX_USECASE_ID_HWREV_00),
	.hw_rev = 0,
	.change_clock_src = true,
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK,
	.tdm_clock_freq = TDM_CLK_FREQ_16,
	.number_of_bits = 16,

	.clock_config_va = {
		.wanted_pll = 81000000,
#ifndef DBMDX_MELON_OPTIMIZED
			.wanted_tl3_clk = 98000000,
#else
			.wanted_tl3_clk = 16000000,
#endif
		.wanted_tl3_clk_debug = 98000000,
		.wanted_ahb_clk = 0,
		.wanted_apb_clk = 0,
		.use_pll_post_div = true,
	},

	.usecase_requires_amodel = true,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
	.model_cp = 0,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.complex_clb_1 = uc_load_models_general,
	.complex_clb_2 = melon_uc_production_model_config,
	.num_of_output_channels = 1,

	.va_cfg_values = (u32 []){
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_1),
		(DBMDX_REGN_UART_SPEED |
			DBMDX_REGV_UART_BAUD_RATE_460_800_hz),
#ifndef FW_AUTO_CONFIGURE_CLOCK
		(DBMDX_UC_SEQ_CMD_CHANGE_CLK_SRC),
#endif
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_2),
		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
			DBMDX_REGV_NO_STREAM_CH_2 |
			DBMDX_REGV_STREAM_CH_1_CP_0),
		(DBMDX_UC_SEQ_CMD_CONFIG_MICS),
	},
#ifdef FW_AUTO_CONFIGURE_CLOCK
	.num_of_va_cfg_values = 5,
#else
	.num_of_va_cfg_values = 6,
#endif

	.config_mics = MIC_CONFIG_BY_USECASE,

	.mic_config = {
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
#else
		(DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_SAR_IIR_FILTER_128 |
#ifdef DBMDX_SAR_ADC_SELECT_SEC
		 DBMDX_REGV_SAR_ADC_SEL_SEC |
#else
		 DBMDX_REGV_SAR_ADC_SEL_MAIN |
#endif
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SD_DDF_SD_ADC),
#endif
		 0x0000,
		 0x0000,
		 0x0000 },

#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
	.mic_freq = { 1536000, 0, 0, 0 },
#else
	.mic_freq = { 512000, 0, 0, 0 },
#endif
	.num_of_va_ve_cfg_values = 0,
	.tdm_configs_va = {
		/* DBMD4 TDM0_TX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM0_RX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM1_TX is disabled */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM1_RX is disabled */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
	},
	.va_start_cmd_type = START_CMD_TYPE_OPMODE,
	.send_va_start_cmd = true,
	/* Enable:
	 *		TDM0_RX (HOST CODEC==>D4)
	 *		TDM0_TX (CLK) MASTER
	 */
	.va_start_cmd = DBMDX_DETECTION,

	.send_va_ve_start_cmd = false,
};



#endif /* _DBMDX_USECASE_CONFIG_MELON_H */

#endif /* DBMDX_MELON_USECASES_SUPPORTED */
