/*
 * dbmdx-customer.h  --  DBMDX customer definitions
 *
 * Copyright (C) 2021 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_CUSTOMER_DEF_H
#define _DBMDX_CUSTOMER_DEF_H

#define DBMD2_VA_FIRMWARE_NAME			"dbmd2_va_fw.bin"

#define DBMD4_VA_FIRMWARE_NAME			"dbmd4_va_fw.bin"

#define DBMDX_VT_GRAM_NAME			"voice_grammar.bin"
#define DBMDX_VT_NET_NAME			"voice_net.bin"
#define DBMDX_VT_AMODEL_NAME			"voice_amodel.bin"

#define DBMDX_VC_GRAM_NAME			"vc_grammar.bin"
#define DBMDX_VC_NET_NAME			"vc_net.bin"
#define DBMDX_VC_AMODEL_NAME			"vc_amodel.bin"

/* ================ Defines related to kernel vesion ===============*/
/* comment out if not define */
#if 0
#define USE_KERNEL_ABOVE_4_17_XX	1		/* snd_soc_component, timer_list */
#define USE_KERNEL_ABOVE_4_19_10	1		/* wakelock handling */
#define USE_KERNEL_ABOVE_5_4		1		/* of_dev_node_match, snd_soc_dai_link */
#define USE_KERNEL_ABOVE_5_10		1		/* asoc_rtd_to_codec, snd_pcm_ops */
#endif

#define USE_ALSA_API_3_10_XX	0

#define SOC_BYTES_EXT_HAS_KCONTROL_FIELD	1

#define DBMDX_UNLOAD_MODELS_ON_ENTER		1


/* ==================================================================*/
#define DBMDX_MELON_USECASES_SUPPORTED 1


#define HOST_HW_TDM_CONF_ASYNC_DOWN_DATA_DOWN 0
#define HOST_HW_TDM_CONF_ASYNC_DOWN_DATA_UP 0
#define HOST_HW_TDM_CONF_ASYNC_UP_DATA_DOWN 0
#define HOST_HW_TDM_CONF_ASYNC_UP_DATA_UP 1

#define DEFAULT_D4_CLOCK_HZ	92160000
/* #define DEFAULT_D4_CLOCK_HZ	49152000 */
/* #define DEFAULT_D4_CLOCK_HZ	73728000 */

/* ================ Custom Configuration ===============*/
#define DBMDX_VA_VE_SUPPORT 1


#define DBMDX_DEFER_IF_SND_CARD_ID_0 1

#define SV_FW_DETECTION_STATS 1
#define FW_SUPPORTS_DSP_CLOCK_32BIT_CMD			1
#define SET_PLL_OSC_SEL_USE_OSC 1

#define AMZ_REQ
#ifdef AMZ_REQ
#define FW_AUTO_CONFIGURE_CLOCK
#define NOT_AUTOLOAD_FW
#endif
//#define AMODEL_LOAD_USES_NORMAL_SPEED		1
/* ==================================================================*/

#endif
