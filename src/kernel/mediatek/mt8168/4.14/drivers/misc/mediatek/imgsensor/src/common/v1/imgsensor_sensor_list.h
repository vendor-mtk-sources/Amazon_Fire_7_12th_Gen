/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __KD_SENSORLIST_H__
#define __KD_SENSORLIST_H__

#include "kd_camera_typedef.h"
#include "imgsensor_sensor.h"

/*IMX*/
UINT32 IMX576_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX350_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX398_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX268_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX386_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX386_MIPI_MONO_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX376_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX362_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX338_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX318_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX377_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX230_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX220_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX219_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX214_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX214_MIPI_MONO_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX179_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX178_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX132_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX135_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX105_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX073_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX258_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX258_MIPI_MONO_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*OV*/
UINT32 OV13855_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV13855SUB_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV16880MIPISensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV16825MIPISensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV13870_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV13850_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV12830_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV9760MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV9740_MIPI_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV9726_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV9726MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV8865_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV8858_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV8856_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV8830SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV8825_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV7675_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5693_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5670_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5670_MIPI_RAW_SensorInit_2(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV2281_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5650SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5650MIPISensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5648MIPISensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5647MIPISensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5647SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5645_MIPI_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5642_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5642_MIPI_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5642_MIPI_RGB_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5642_MIPI_JPG_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5642_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV5642_YUV_SWI2C_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV4688_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV3640SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV3640_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV2722MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV2680MIPISensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV2659_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV2655_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV2650SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV23850_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV20880_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*S5K*/
UINT32 S5K2T7SP_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K3P8SP_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K3P8SX_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K2T7SP_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K2L7_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K3L8_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K3M3_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K2P7_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K2P8_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K3P3SX_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K2X8_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K3M2_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K4E6_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K3H2YX_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K3H7Y_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K4H5YC_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K4H5YX_2LANE_MIPI_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K5E2YA_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K5CAGX_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K4E1GA_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K4ECGX_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K4ECGX_MIPI_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K4ECGX_MIPI_JPG_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K8AAYX_MIPI_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K8AAYX_PVI_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K5E8YX_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*HI*/
UINT32 HI841_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 HI707_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 HI704_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 HI551_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 HI545_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 HI544_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 HI542_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 HI542_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 HI253_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 HI191MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*MT*/
UINT32 MT9P012SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 MT9P015SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 MT9P017SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 MT9P017MIPISensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 MT9T113_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 MT9V113_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 MT9T113MIPI_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 MT9V114_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 MT9D115MIPISensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 MT9V115_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*GC*/
UINT32 GC2375_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC2375SUB_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC2375HMIPI_CXT_REAR_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC2375HMIPI_CXT_FRONT_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC02M1MIPI_CXT_REAR_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC02M1MIPI_CXT_FRONT_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC02M1_QUARTZ_CXT_REAR_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC02M1_QUARTZ_CXT_FRONT_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC02M1_QUARTZ_LCE_REAR_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC02M1_QUARTZ_LCE_FRONT_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV02B10_QUARTZ_SJC_REAR_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV02B10_QUARTZ_SJC_FRONT_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC2365_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC2366_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC2355_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC2235_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC2035_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC0330_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC0329_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC0313MIPI_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC0310_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 IMX215_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*SP*/
UINT32 SP0A19_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 SP2509_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);

/*A*/
UINT32 A5141_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 A5142_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*HM*/
UINT32 HM3451SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*AR*/
UINT32 AR0833_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*SIV*/
UINT32 SIV120B_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 SIV121D_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*PAS (PixArt Image)*/
UINT32 PAS6180_SERIAL_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*Panasoic*/
UINT32 MN34152_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*Toshiba*/
UINT32 T4K28_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 T4KA7_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
/*Others*/
UINT32 ISX012_MIPI_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 T8EV5_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 SC202CS_QUARTZ_RAW_CXT_REAR_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 SC202CS_QUARTZ_RAW_CXT_FRONT_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc);

extern struct IMGSENSOR_SENSOR_LIST gimgsensor_sensor_list[];

#endif

