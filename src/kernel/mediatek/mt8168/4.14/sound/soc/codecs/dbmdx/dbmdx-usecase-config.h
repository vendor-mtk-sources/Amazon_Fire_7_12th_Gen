/*
 * dbmdx-usecase-config  --  Preset USE CASE configurations
 *
 * Copyright (C) 2021 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_USECASE_CONFIG_H
#define _DBMDX_USECASE_CONFIG_H

#include "dbmdx-usecase-config-def.h"

#ifdef DBMDX_MELON_USECASES_SUPPORTED
#include "dbmdx-usecase-config-melon.h"
#endif

static struct usecase_config *usecases_map[] = {
#ifdef DBMDX_MELON_USECASES_SUPPORTED
	&config_uc_melon_idle,
	&config_uc_melon_low_power,
	&config_uc_melon_production,
#endif

};

static struct usecase_config config_usecase_external = {
	.usecase_name = NULL,
	.num_of_va_cfg_values = 0,
	.va_cfg_values = NULL,
	.num_of_va_ve_cfg_values = 0,
	.va_ve_cfg_values = NULL,
};

#endif
