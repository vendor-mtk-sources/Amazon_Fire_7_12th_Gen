/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/version.h>
#include <linux/iio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <mt-plat/aee.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/uaccess.h>
#include "mt-plat/mtk_thermal_monitor.h"
#include "mach/mtk_thermal.h"
#include "mtk_thermal_timer.h"
#include "mt-plat/mtk_thermal_platform.h"
#include <linux/uidgid.h>
#include <tmp_bts.h>
#include <linux/slab.h>
#include <linux/reboot.h>
#ifdef CONFIG_AMZN_SIGN_OF_LIFE
#include <linux/amzn_sign_of_life.h>
#endif
#ifdef CONFIG_THERMAL_SHUTDOWN_LAST_KMESG
#include <linux/thermal_framework.h>
#endif
#include <charger_class.h>

struct mtkts_bts_channel_param {
	int g_RAP_pull_up_R;
	int g_TAP_over_critical_low;
	int g_RAP_pull_up_voltage;
	int g_RAP_ntc_table;
	int g_RAP_ADC_channel;
	int g_AP_TemperatureR;
	char channelName[THERMAL_NAME_LENGTH];
#ifdef CONFIG_THERMAL_DEBOUNCE
	int pre_temp;
	int BTS_counter;
	int BTS_temp_change;
#endif
};

struct BTS_TEMPERATURE {
	__s32 BTS_Temp;
	__s32 TemperatureR;
};

#include "ntc_table.h"
static int auxadc_channel_num;
static struct mtkts_bts_channel_param *bts_channel_param;

#define RESERVED_TZS (4)
#define NUM_TRIP_MAX (10)

static kuid_t uid = KUIDT_INIT(0);
static kgid_t gid = KGIDT_INIT(1000);

struct thz_data {
	struct thermal_zone_device *thz_dev;
	char thz_name[20];
	int trip_temp[10];
	int trip_type[10];	/*ACTIVE, PASSIVE, HOT, and Critical*/
	char bind[10][20];
	int num_trip;
	unsigned int interval;	/* mseconds, 0 : no auto polling */
	int kernelmode;
	struct semaphore sem_mutex;
};

static struct thz_data g_tsData[RESERVED_TZS];

static unsigned int interval;	/* seconds, 0 : no auto polling */
static int trip_temp[10] = { 120000, 110000, 100000, 90000, 80000,
				70000, 65000, 60000, 55000, 50000 };

static struct thermal_zone_device **thz_dev;
static int mtkts_bts_debug_log;

/**
 * If curr_temp >= polling_trip_temp1, use interval
 * else if cur_temp >= polling_trip_temp2 && curr_temp < polling_trip_temp1,
 *	use interval*polling_factor1
 * else, use interval*polling_factor2
 */
static int polling_trip_temp1 = 40000;
static int polling_trip_temp2 = 20000;
static int polling_factor1 = 5000;
static int polling_factor2 = 10000;

static int bts_cur_temp = 1;


#define MTKTS_BTS_TEMP_CRIT 60000	/* 60.000 degree Celsius */

#define mtkts_bts_dprintk(fmt, args...)   \
do {                                    \
	if (mtkts_bts_debug_log) {                \
		pr_debug("[Thermal/TZ/BTS]" fmt, ##args); \
	}                                   \
} while (0)


#define mtkts_bts_printk(fmt, args...) \
pr_debug("[Thermal/TZ/BTS]" fmt, ##args)

struct gadc_thermal_info {
	struct device *dev;
	struct iio_channel *channel;
};
static struct gadc_thermal_info *gti_ntc;
/* BTS_TEMPERATURE BTS_Temperature_Table[] = {0}; */

static struct BTS_TEMPERATURE *BTS_Temperature_Table;
static int ntc_tbl_size;

#ifdef CONFIG_THERMAL_DEBOUNCE
#define BTS_VALID_CHANGE_1		30000
#define BTS_VALID_CHANGE_5		60000
#define BTS_VALID_CHANGE_10		80000
#endif

/* convert register to temperature  */
static __s16 mtkts_bts_thermistor_conver_temp(__s32 Res)
{
	int i = 0;
	int asize = 0;
	__s32 RES1 = 0, RES2 = 0;
	__s32 TAP_Value = -200, TMP1 = 0, TMP2 = 0;

	asize = (ntc_tbl_size / sizeof(struct BTS_TEMPERATURE));

	/* mtkts_bts_dprintk("mtkts_bts_thermistor_conver_temp() :
	 * asize = %d, Res = %d\n",asize,Res);
	 */
	if (Res >= BTS_Temperature_Table[0].TemperatureR) {
		TAP_Value = -40;	/* min */
	} else if (Res <= BTS_Temperature_Table[asize - 1].TemperatureR) {
		TAP_Value = 125;	/* max */
	} else {
		RES1 = BTS_Temperature_Table[0].TemperatureR;
		TMP1 = BTS_Temperature_Table[0].BTS_Temp;
		/* mtkts_bts_dprintk("%d : RES1 = %d,TMP1 = %d\n",
		 * __LINE__,RES1,TMP1);
		 */

		for (i = 0; i < asize; i++) {
			if (Res >= BTS_Temperature_Table[i].TemperatureR) {
				RES2 = BTS_Temperature_Table[i].TemperatureR;
				TMP2 = BTS_Temperature_Table[i].BTS_Temp;
				/* mtkts_bts_dprintk("%d :i=%d, RES2 = %d,
				 * TMP2 = %d\n",__LINE__,i,RES2,TMP2);
				 */
				break;
			}
			RES1 = BTS_Temperature_Table[i].TemperatureR;
			TMP1 = BTS_Temperature_Table[i].BTS_Temp;
			/* mtkts_bts_dprintk("%d :i=%d, RES1 = %d,TMP1 = %d\n",
			 * __LINE__,i,RES1,TMP1);
			 */
		}

		TAP_Value = (((Res - RES2) * TMP1) + ((RES1 - Res) * TMP2))
								/ (RES1 - RES2);
	}

#if 0
	mtkts_bts_dprintk(
		"%s() : TAP_Value = %d\n", __func__,
								TAP_Value);

	mtkts_bts_dprintk("%s() : Res = %d\n", __func__,
									Res);

	mtkts_bts_dprintk("%s() : RES1 = %d\n", __func__
									RES1);

	mtkts_bts_dprintk("%s() : RES2 = %d\n", __func__,
									RES2);

	mtkts_bts_dprintk("%s() : TMP1 = %d\n", __func__,
									TMP1);

	mtkts_bts_dprintk("%s() : TMP2 = %d\n", __func__,
									TMP2);
#endif

	return TAP_Value;
}

/* convert ADC_AP_temp_volt to register */
/*Volt to Temp formula same with 6589*/
static __s16 mtk_ts_bts_volt_to_temp(__s32 index, __u32 dwVolt)
{
	__s32 TRes;
	__s32 dwVCriAP = 0;
	__s32 BTS_TMP = -100;

	/* SW workaround-----------------------------------------------------
	 * dwVCriAP = (TAP_OVER_CRITICAL_LOW * 1800) /
	 *		(TAP_OVER_CRITICAL_LOW + 39000);
	 * dwVCriAP = (TAP_OVER_CRITICAL_LOW * RAP_PULL_UP_VOLT) /
	 *		(TAP_OVER_CRITICAL_LOW + RAP_PULL_UP_R);
	 */

	dwVCriAP =
	    (bts_channel_param[index].g_TAP_over_critical_low *
		bts_channel_param[index].g_RAP_pull_up_voltage) /
			(bts_channel_param[index].g_TAP_over_critical_low +
		bts_channel_param[index].g_RAP_pull_up_R);

	if (dwVolt > dwVCriAP) {
		TRes = bts_channel_param[index].g_TAP_over_critical_low;
	} else {
		/* TRes = (39000*dwVolt) / (1800-dwVolt); */
		/* TRes = (RAP_PULL_UP_R*dwVolt) / (RAP_PULL_UP_VOLT-dwVolt); */
		TRes = (bts_channel_param[index].g_RAP_pull_up_R * dwVolt) /
		(bts_channel_param[index].g_RAP_pull_up_voltage - dwVolt);
	}
	/* ------------------------------------------------------------------ */
	mtkts_bts_dprintk("index=%d, dwVCriAP=%d, TRes=%d\n",
		index, dwVCriAP, TRes);

	bts_channel_param[index].g_AP_TemperatureR = TRes;

	#ifdef CONFIG_THERMAL_CROWN
		mtkts_bts_prepare_table(bts_channel_param[index].g_RAP_ntc_table);
	#endif

	/* convert register to temperature */
	BTS_TMP = mtkts_bts_thermistor_conver_temp(TRes);

	return BTS_TMP;
}

static DEFINE_MUTEX(BTS_lock);

static int gadc_thermal_get_temp(void *data, int index, int *temp)
{
	struct gadc_thermal_info *gti = data;
	struct iio_channel *channel;
	int val;
	int ret;
	int higher_temp;

	if (gti == NULL)
		return 0;

	if (index >= auxadc_channel_num) {
		dev_err(gti->dev, "%s: error: index:%d > auxadc_channel_num:%d\n",
				__func__, index, auxadc_channel_num);
		return -EINVAL;
	}

	channel = &gti->channel[index];

	ret = iio_read_channel_processed(channel, &val);
	if (ret < 0) {
		dev_dbg(gti->dev, "IIO channel read failed %d\n", ret);
		return ret;
	}

	val = val * 1500 / 4096;

	higher_temp = mtk_ts_bts_volt_to_temp(index, val);
	if (higher_temp >= *temp)
		*temp = higher_temp;
	mtkts_bts_dprintk("channel = %d mV\n",  val);
	mtkts_bts_dprintk("channel = %d degree Celsius\n", higher_temp);
	return 0;
}

/*int ts_AP_at_boot_time = 0;*/
int mtkts_bts_get_hw_temp(int index)
{
	int t_ret = 0;
	int t_ret2 = 0;

	mutex_lock(&BTS_lock);

	/* get HW AP temp (TSAP) */
	/* cat /sys/class/power_supply/AP/AP_temp */
	gadc_thermal_get_temp(gti_ntc, index, &t_ret);
	t_ret = t_ret * 1000;

	mutex_unlock(&BTS_lock);


	if (tsatm_thermal_get_catm_type() == 2)
		t_ret2 = wakeup_ta_algo(TA_CATMPLUS_TTJ);

	if (t_ret2 < 0)
		pr_notice("[Thermal/TZ/BTS]wakeup_ta_algo out of memory\n");

	bts_cur_temp = t_ret;

	if (t_ret > 40000)	/* abnormal high temp */
		mtkts_bts_printk("T_AP=%d\n", t_ret);

	mtkts_bts_dprintk("[%s] T_AP, %d\n", __func__, t_ret);
	return t_ret;
}

static int mtkts_bts_get_temp(struct thermal_zone_device *thermal, int *t)
{
#ifdef CONFIG_THERMAL_DEBOUNCE
	int ret = 0;
#endif
	int i = thermal->type[strlen(thermal->type) - 1] - '0';

	mtkts_bts_dprintk("[mtkts_bts_get_temp]index = %d\n", i);

	if (i < 0 || i > auxadc_channel_num) {
		pr_err("%s bad channel index %d, name=%s\n",
			__func__, i, thermal->type);
		return -EINVAL;
	}

	*t = mtkts_bts_get_hw_temp(i);
#ifdef CONFIG_THERMAL_DEBOUNCE
	if ((*t > 145000) || (*t < -60000)) {
		pr_err("%s temp(%d) too high, drop this data!\n",
					__func__, *t);
		return -1;
	}

	ret = thermal_zone_debounce(&bts_channel_param[i].pre_temp, t,
				    bts_channel_param[i].BTS_temp_change,
				    &bts_channel_param[i].BTS_counter,
				    thermal->type);
#endif
	if ((int)*t >= polling_trip_temp1) {
		thermal->polling_delay = interval * 1000;
#ifdef CONFIG_THERMAL_DEBOUNCE
		bts_channel_param[i].BTS_temp_change = BTS_VALID_CHANGE_1;
#endif
	} else if ((int)*t < polling_trip_temp2) {
		thermal->polling_delay = interval * polling_factor2;
#ifdef CONFIG_THERMAL_DEBOUNCE
		bts_channel_param[i].BTS_temp_change = BTS_VALID_CHANGE_10;
#endif
	} else {
		thermal->polling_delay = interval * polling_factor1;
#ifdef CONFIG_THERMAL_DEBOUNCE
		bts_channel_param[i].BTS_temp_change = BTS_VALID_CHANGE_5;
#endif
	}
#ifdef CONFIG_THERMAL_DEBOUNCE
	return ret;
#else
	return 0;
#endif
}

static int mtkts_bts_get_index(struct thermal_zone_device *thermal)
{
	/* ex: mtkts_bts0, mtkts_bts1, mtkts_bts2, mtkts_bts3 */
	int index;
	index = thermal->type[strlen(thermal->type) - 1] - '0';

	if (index < 0 || index >= auxadc_channel_num)
		index = -1;

	return index;
}

static int mtkts_bts_bind(
struct thermal_zone_device *thermal, struct thermal_cooling_device *cdev)
{
	int table_val = -1, index, i;

	index = mtkts_bts_get_index(thermal);
	if (index == -1) {
		return -EINVAL;
	}

	mtkts_bts_dprintk("[%s mtkts_bts%d]\n", __func__, index);

	for (i = 0; i < NUM_TRIP_MAX; i++) {
		if (!strcmp(cdev->type, g_tsData[index].bind[i])) {
			table_val = i;
			break;
		}
	}

	if (table_val == -1)
		return 0;

	mtkts_bts_dprintk("[%s mtkts_bts %d] %s\n", __func__, index, cdev->type);

	if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) {
		mtkts_bts_dprintk(
			"[%s mtkts_bts %d] error binding cooling dev\n", __func__, index);
		return -EINVAL;
	}

	mtkts_bts_dprintk("[%s mtkts_bts %d] binding OK, %d\n", __func__, index, table_val);

	return 0;
}

static int mtkts_bts_unbind(struct thermal_zone_device *thermal,
			    struct thermal_cooling_device *cdev)
{
	int table_val = -1, index, i;

	index = mtkts_bts_get_index(thermal);
	if (index == -1) {
		return -EINVAL;
	}

	mtkts_bts_dprintk("[%s mtkts_bts%d]\n", __func__, index);

	for (i = 0; i < NUM_TRIP_MAX; i++) {
		if (!strcmp(cdev->type, g_tsData[index].bind[i])) {
			table_val = i;
			break;
		}
	}

	if (table_val == -1)
		return 0;

	mtkts_bts_dprintk("[%s mtkts_bts %d] %s\n", __func__, index, cdev->type);

	if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) {
		mtkts_bts_dprintk(
			"[%s mtkts_bts %d] error unbinding cooling dev\n", __func__,
			index);

		return -EINVAL;
	}

	mtkts_bts_dprintk("[%s mtkts_bts %d] unbinding OK\n", __func__, index);
	return 0;
}

static int mtkts_bts_get_mode(
struct thermal_zone_device *thermal, enum thermal_device_mode *mode)
{
	int index;

	index = mtkts_bts_get_index(thermal);
	if (index == -1) {
		return -EINVAL;
	}

	*mode = (g_tsData[index].kernelmode) ?
			THERMAL_DEVICE_ENABLED : THERMAL_DEVICE_DISABLED;
	return 0;
}

static int mtkts_bts_set_mode(
struct thermal_zone_device *thermal, enum thermal_device_mode mode)
{
	int index;

	index = mtkts_bts_get_index(thermal);
	if (index == -1) {
		return -EINVAL;
	}

	g_tsData[index].kernelmode = mode;
	return 0;
}

static int mtkts_bts_get_trip_type(
struct thermal_zone_device *thermal, int trip, enum thermal_trip_type *type)
{
	int index;

	index = mtkts_bts_get_index(thermal);
	if (index == -1) {
		return -EINVAL;
	}

	*type = g_tsData[index].trip_type[trip];
	return 0;
}

static int mtkts_bts_get_trip_temp(
struct thermal_zone_device *thermal, int trip, int *temp)
{
	int index;

	index = mtkts_bts_get_index(thermal);
	if (index == -1) {
		return -EINVAL;
	}

	*temp = g_tsData[index].trip_temp[trip];
	return 0;
}

static int mtkts_bts_get_crit_temp(
struct thermal_zone_device *thermal, int *temperature)
{
	*temperature = MTKTS_BTS_TEMP_CRIT;
	return 0;
}

static int mtkts_bts_thermal_notify(struct thermal_zone_device *thermal, int trip, enum thermal_trip_type type)
{

#ifdef CONFIG_AMZN_SIGN_OF_LIFE
	if (type == THERMAL_TRIP_CRITICAL) {
		pr_err("[%s] [%s] type: [%s] Thermal shutdown bts, current temp=%d, trip=%d, trip_temp = %d\n",
			__func__, dev_name(&thermal->device), thermal->type,
			thermal->temperature, trip, trip_temp[trip]);
		life_cycle_set_thermal_shutdown_reason(THERMAL_SHUTDOWN_REASON_BTS);
	}
#endif

#ifdef CONFIG_THERMAL_SHUTDOWN_LAST_KMESG
	if (type == THERMAL_TRIP_CRITICAL) {
		pr_err("%s: thermal_shutdown notify\n", __func__);
		last_kmsg_thermal_shutdown();
		pr_err("%s: thermal_shutdown notify end\n", __func__);
	}
#endif
	if (type == THERMAL_TRIP_CRITICAL)
		set_shutdown_enable_dcap(&thermal->device);

	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops mtkts_BTS_dev_ops = {
	.bind = mtkts_bts_bind,
	.unbind = mtkts_bts_unbind,
	.get_temp = mtkts_bts_get_temp,
	.get_mode = mtkts_bts_get_mode,
	.set_mode = mtkts_bts_set_mode,
	.get_trip_type = mtkts_bts_get_trip_type,
	.get_trip_temp = mtkts_bts_get_trip_temp,
	.get_crit_temp = mtkts_bts_get_crit_temp,
	.notify = mtkts_bts_thermal_notify,
};

static void mtkts_bts_unregister_thermal(void);

void mtkts_bts_prepare_table(int table_num)
{

	switch (table_num) {
	case 1:		/* AP_NTC_BL197 */
		BTS_Temperature_Table = BTS_Temperature_Table1;
		ntc_tbl_size = sizeof(BTS_Temperature_Table1);
		break;
	case 2:		/* AP_NTC_TSM_1 */
		BTS_Temperature_Table = BTS_Temperature_Table2;
		ntc_tbl_size = sizeof(BTS_Temperature_Table2);
		break;
	case 3:		/* AP_NTC_10_SEN_1 */
		BTS_Temperature_Table = BTS_Temperature_Table3;
		ntc_tbl_size = sizeof(BTS_Temperature_Table3);
		break;
	case 4:		/* AP_NTC_10 */
		BTS_Temperature_Table = BTS_Temperature_Table4;
		ntc_tbl_size = sizeof(BTS_Temperature_Table4);
		break;
	case 5:		/* AP_NTC_47 */
		BTS_Temperature_Table = BTS_Temperature_Table5;
		ntc_tbl_size = sizeof(BTS_Temperature_Table5);
		break;
	case 6:		/* NTCG104EF104F */
		BTS_Temperature_Table = BTS_Temperature_Table6;
		ntc_tbl_size = sizeof(BTS_Temperature_Table6);
		break;
	case 7:		/* NCP15WF104F03RC */
		BTS_Temperature_Table = BTS_Temperature_Table7;
		ntc_tbl_size = sizeof(BTS_Temperature_Table7);
		break;
	default:		/* AP_NTC_10 */
		BTS_Temperature_Table = BTS_Temperature_Table4;
		ntc_tbl_size = sizeof(BTS_Temperature_Table4);
		break;
	}

	pr_notice("[Thermal/TZ/BTS] %s table_num=%d\n", __func__, table_num);

#if 0
	{
		int i = 0;

		for (i = 0; i < (ntc_tbl_size
			/ sizeof(struct BTS_TEMPERATURE)); i++) {
			pr_notice(
				"BTS_Temperature_Table[%d].APteryTemp =%d\n", i,
				BTS_Temperature_Table[i].BTS_Temp);
			pr_notice(
				"BTS_Temperature_Table[%d].TemperatureR=%d\n",
				i, BTS_Temperature_Table[i].TemperatureR);
		}
	}
#endif
}

static int mtkts_bts_param_read(struct seq_file *m, void *v)
{
	__s32 i;

	for (i = 0; i < auxadc_channel_num; i++) {
		seq_printf(m, "%d\t", bts_channel_param[i].g_RAP_pull_up_R);
		seq_printf(m, "%d\t",
		bts_channel_param[i].g_RAP_pull_up_voltage);
		seq_printf(m, "%d\t",
		bts_channel_param[i].g_TAP_over_critical_low);
		seq_printf(m, "%d\t", bts_channel_param[i].g_RAP_ntc_table);
		seq_printf(m, "%d\n", bts_channel_param[i].g_RAP_ADC_channel);
	}

	return 0;
}

static ssize_t mtkts_bts_param_write(
struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	int len = 0;
	struct mtktsbts_param_data {
		char desc[512];
		char pull_R[10], pull_V[10];
		char overcrilow[16];
		char NTC_TABLE[10];
		unsigned int valR, valV, over_cri_low, ntc_table;
		unsigned int adc_channel;
	};

	struct mtktsbts_param_data *ptr_mtktsbts_parm_data;

	ptr_mtktsbts_parm_data = kmalloc(
				sizeof(*ptr_mtktsbts_parm_data), GFP_KERNEL);

	if (ptr_mtktsbts_parm_data == NULL)
		return -ENOMEM;

	/* external pin: 0/1/12/13/14/15, can't use pin:2/3/4/5/6/7/8/9/10/11,
	 *choose "adc_channel=11" to check if there is any param input
	 */
	ptr_mtktsbts_parm_data->adc_channel = 11;

	len = (count < (sizeof(ptr_mtktsbts_parm_data->desc) - 1)) ?
			count : (sizeof(ptr_mtktsbts_parm_data->desc) - 1);

	if (copy_from_user(ptr_mtktsbts_parm_data->desc, buffer, len)) {
		kfree(ptr_mtktsbts_parm_data);
		return 0;
	}

	ptr_mtktsbts_parm_data->desc[len] = '\0';

	mtkts_bts_dprintk("[%s]\n", __func__);

	if (sscanf
	    (ptr_mtktsbts_parm_data->desc, "%9s %d %9s %d %15s %d %9s %d %d",
		ptr_mtktsbts_parm_data->pull_R, &ptr_mtktsbts_parm_data->valR,
		ptr_mtktsbts_parm_data->pull_V, &ptr_mtktsbts_parm_data->valV,
		ptr_mtktsbts_parm_data->overcrilow,
		&ptr_mtktsbts_parm_data->over_cri_low,
		ptr_mtktsbts_parm_data->NTC_TABLE,
		&ptr_mtktsbts_parm_data->ntc_table,
		&ptr_mtktsbts_parm_data->adc_channel) >= 8) {

		__s32 i;

		for (i = 0; i < auxadc_channel_num; i++) {
			if (bts_channel_param[i].g_RAP_ADC_channel == ptr_mtktsbts_parm_data->adc_channel)
				break;
		}
		if (i == auxadc_channel_num) {
			pr_err("%s bad channel argument %d\n", __func__, ptr_mtktsbts_parm_data->adc_channel);
			kfree(ptr_mtktsbts_parm_data);
			return -EINVAL;
		}

		if (!strcmp(ptr_mtktsbts_parm_data->pull_R, "PUP_R")) {
			bts_channel_param[i].g_RAP_pull_up_R = ptr_mtktsbts_parm_data->valR;
			mtkts_bts_dprintk("g_RAP_pull_up_R=%d\n",
							bts_channel_param[i].g_RAP_pull_up_R);
		} else {
			mtkts_bts_printk(
				"[%s] bad PUP_R argument\n", __func__);
			kfree(ptr_mtktsbts_parm_data);
			return -EINVAL;
		}

		if (!strcmp(ptr_mtktsbts_parm_data->pull_V, "PUP_VOLT")) {
			bts_channel_param[i].g_RAP_pull_up_voltage = ptr_mtktsbts_parm_data->valV;
			mtkts_bts_dprintk("g_Rat_pull_up_voltage=%d\n",
							bts_channel_param[i].g_RAP_pull_up_voltage);
		} else {
			mtkts_bts_printk(
				"[%s] bad PUP_VOLT argument\n", __func__);
			kfree(ptr_mtktsbts_parm_data);
			return -EINVAL;
		}

		if (!strcmp(ptr_mtktsbts_parm_data->overcrilow,
			"OVER_CRITICAL_L")) {
			bts_channel_param[i].g_TAP_over_critical_low = ptr_mtktsbts_parm_data->over_cri_low;
			mtkts_bts_dprintk("g_TAP_over_critical_low=%d\n",
						bts_channel_param[i].g_TAP_over_critical_low);
		} else {
			mtkts_bts_printk(
				"[%s] bad OVERCRIT_L argument\n", __func__);
			kfree(ptr_mtktsbts_parm_data);
			return -EINVAL;
		}

		if (!strcmp(ptr_mtktsbts_parm_data->NTC_TABLE, "NTC_TABLE")) {
			bts_channel_param[i].g_RAP_ntc_table = ptr_mtktsbts_parm_data->ntc_table;
			mtkts_bts_dprintk("g_RAP_ntc_table=%d\n",
						bts_channel_param[i].g_RAP_ntc_table);
		} else {
			mtkts_bts_printk(
				"[%s] bad NTC_TABLE argument\n", __func__);
			kfree(ptr_mtktsbts_parm_data);
			return -EINVAL;
		}

		mtkts_bts_dprintk("adc_channel=%d\n",
					ptr_mtktsbts_parm_data->adc_channel);
		mtkts_bts_dprintk("g_RAP_ADC_channel=%d\n",
					bts_channel_param[i].g_RAP_ADC_channel);

		mtkts_bts_prepare_table(bts_channel_param[i].g_RAP_ntc_table);

		kfree(ptr_mtktsbts_parm_data);
		return count;
	}

	mtkts_bts_printk("[%s] bad argument\n", __func__);
	kfree(ptr_mtktsbts_parm_data);
	return -EINVAL;
}

#if 0
static void mtkts_bts_cancel_thermal_timer(void)
{
	/* cancel timer
	 * mtkts_bts_printk("mtkts_bts_cancel_thermal_timer\n");

	 * stop thermal framework polling when entering deep idle

	 *
	 *   We cannot cancel the timer during deepidle and SODI, because
	 *   the battery may suddenly heat up by 3A fast charging.
	 *
	 *
	 *if (thz_dev)
	 *	cancel_delayed_work(&(thz_dev->poll_queue));
	 */
}


static void mtkts_bts_start_thermal_timer(void)
{
	/* mtkts_bts_printk("mtkts_bts_start_thermal_timer\n");
	 * resume thermal framework polling when leaving deep idle
	 *
	 *if (thz_dev != NULL && interval != 0)
	 *	mod_delayed_work(system_freezable_power_efficient_wq,
	 *				&(thz_dev->poll_queue),
	 *				round_jiffies(msecs_to_jiffies(3000)));
	 */
}
#endif

static void mtkts_bts_unregister_thermal(void)
{
	__s32 i;
	mtkts_bts_dprintk("[%s]\n", __func__);

	for (i = 0; i < auxadc_channel_num; i++) {
		if (thz_dev[i]) {
			mtk_thermal_zone_device_unregister(thz_dev[i]);
			thz_dev[i] = NULL;
		}
	}
}

#define FOPS_RW(num)	\
static int mtkts_bts ## num ## _read(struct seq_file *m, void *v)	\
{	\
	int i;	\
\
	for (i = 0; i < NUM_TRIP_MAX; i++) {	\
		seq_printf(m, "Trip_%d_temp=%d ", i,	\
			g_tsData[num].trip_temp[i]);	\
		if ((i + 1) % 5 == 0)	\
			seq_printf(m, "\n");	\
	}	\
\
	for (i = 0; i < NUM_TRIP_MAX; i++) {	\
		seq_printf(m, "Trip_type%d=%d ", i,	\
			g_tsData[num].trip_type[i]);	\
		if ((i + 1) % 5 == 0)	\
			seq_printf(m, "\n");	\
	}	\
\
	for (i = 0; i < NUM_TRIP_MAX; i++) {	\
		seq_printf(m, "Cool_dev%d=%s ", i,	\
			g_tsData[num].bind[i]);	\
		if ((i + 1) % 5 == 0)	\
			seq_printf(m, "\n");	\
	}	\
	seq_printf(m, "Time_ms=%d\n", g_tsData[num].interval);	\
	return 0;	\
}	\
\
static ssize_t mtkts_bts ## num ## _write(	\
struct file *file, const char __user *buffer, size_t count, loff_t *data)	\
{	\
	int len = 0, i, j;	\
	struct mtktsbts_data {	\
		int num_trip;	\
		int time_msec;	\
		int trip[10];	\
		int t_type[10];	\
		char bind[10][20];	\
		char desc[512];	\
	};	\
\
	struct mtktsbts_data *ptr_mtktsbts_data = kmalloc(	\
					sizeof(*ptr_mtktsbts_data), GFP_KERNEL);	\
\
	if (ptr_mtktsbts_data == NULL)	\
		return -ENOMEM;	\
\
	len = (count < (sizeof(ptr_mtktsbts_data->desc) - 1)) ?	\
				count : (sizeof(ptr_mtktsbts_data->desc) - 1);	\
\
	if (copy_from_user(ptr_mtktsbts_data->desc, buffer, len)) {	\
		kfree(ptr_mtktsbts_data);	\
		return 0;	\
	}	\
\
	ptr_mtktsbts_data->desc[len] = '\0';	\
\
	i = sscanf(ptr_mtktsbts_data->desc,	\
"%d %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d"	\
"%19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d",	\
		&ptr_mtktsbts_data->num_trip,	\
		&ptr_mtktsbts_data->trip[0], &ptr_mtktsbts_data->t_type[0], ptr_mtktsbts_data->bind[0],	\
		&ptr_mtktsbts_data->trip[1], &ptr_mtktsbts_data->t_type[1], ptr_mtktsbts_data->bind[1],	\
		&ptr_mtktsbts_data->trip[2], &ptr_mtktsbts_data->t_type[2], ptr_mtktsbts_data->bind[2],	\
		&ptr_mtktsbts_data->trip[3], &ptr_mtktsbts_data->t_type[3], ptr_mtktsbts_data->bind[3],	\
		&ptr_mtktsbts_data->trip[4], &ptr_mtktsbts_data->t_type[4], ptr_mtktsbts_data->bind[4],	\
		&ptr_mtktsbts_data->trip[5], &ptr_mtktsbts_data->t_type[5], ptr_mtktsbts_data->bind[5],	\
		&ptr_mtktsbts_data->trip[6], &ptr_mtktsbts_data->t_type[6], ptr_mtktsbts_data->bind[6],	\
		&ptr_mtktsbts_data->trip[7], &ptr_mtktsbts_data->t_type[7], ptr_mtktsbts_data->bind[7],	\
		&ptr_mtktsbts_data->trip[8], &ptr_mtktsbts_data->t_type[8], ptr_mtktsbts_data->bind[8],	\
		&ptr_mtktsbts_data->trip[9], &ptr_mtktsbts_data->t_type[9], ptr_mtktsbts_data->bind[9],	\
		&ptr_mtktsbts_data->time_msec);	\
\
	if (i == 32) {	\
		down(&g_tsData[num].sem_mutex);	\
		mtkts_bts_dprintk("[mtkts_bts_write_"__stringify(num)	\
					"]thermal unregister\n");	\
		if (g_tsData[num].thz_dev) {	\
			mtk_thermal_zone_device_unregister(	\
				g_tsData[num].thz_dev);	\
				g_tsData[num].thz_dev = NULL;	\
		}	\
\
		if (ptr_mtktsbts_data->num_trip < 0 || ptr_mtktsbts_data->num_trip > NUM_TRIP_MAX) {	\
			aee_kernel_warning_api(__FILE__, __LINE__,	\
					DB_OPT_DEFAULT, "mtkts_bts_write",	\
					"Bad argument");	\
			mtkts_bts_dprintk("[%s] bad argument\n", __func__);	\
			kfree(ptr_mtktsbts_data);	\
			up(&g_tsData[num].sem_mutex);	\
			return -EINVAL;	\
		}	\
\
		g_tsData[num].num_trip = ptr_mtktsbts_data->num_trip;	\
\
		for (i = 0; i < g_tsData[num].num_trip; i++) {	\
			g_tsData[num].trip_type[i] =	\
						ptr_mtktsbts_data->t_type[i];	\
			g_tsData[num].trip_temp[i] =	\
						ptr_mtktsbts_data->trip[i];	\
		}	\
\
		for (i = 0; i < NUM_TRIP_MAX; i++) {	\
			g_tsData[num].bind[i][0] = '\0';	\
			for (j = 0; j < 20; j++)	\
				g_tsData[num].bind[i][j] =	\
						ptr_mtktsbts_data->bind[i][j];	\
		}	\
\
		if (mtkts_bts_debug_log) {	\
			for (i = 0; i < NUM_TRIP_MAX; i++) {	\
				mtkts_bts_dprintk("Trip_%d_temp=%d ", i,	\
				g_tsData[num].trip_temp[i]);	\
				if ((i + 1) % 5 == 0)	\
					mtkts_bts_dprintk("\n");	\
			}	\
\
			for (i = 0; i < NUM_TRIP_MAX; i++) {	\
				mtkts_bts_dprintk("Trip_type%d=%d ", i,	\
				g_tsData[num].trip_type[i]);	\
				if ((i + 1) % 5 == 0)	\
					mtkts_bts_dprintk("\n");	\
			}	\
\
			for (i = 0; i < NUM_TRIP_MAX; i++) {	\
				mtkts_bts_dprintk("Cool_dev%d=%s ", i,	\
				g_tsData[num].bind[i]);	\
				if ((i + 1) % 5 == 0)	\
					mtkts_bts_dprintk("\n");	\
			}	\
			mtkts_bts_dprintk("Time_ms=%d\n",	\
					g_tsData[num].interval);	\
		}	\
\
		for (i = 0; i < ptr_mtktsbts_data->num_trip; i++)	\
			trip_temp[i] = ptr_mtktsbts_data->trip[i];	\
\
		interval = ptr_mtktsbts_data->time_msec / 1000;	\
\
		mtkts_bts_dprintk("[mtkts_bts_write_"__stringify(num)	\
					"] thermal register\n");	\
		if (g_tsData[num].thz_dev == NULL) {	\
			g_tsData[num].thz_dev =	\
				mtk_thermal_zone_device_register(	\
					bts_channel_param[num].channelName, g_tsData[num].num_trip, NULL,	\
		&mtkts_BTS_dev_ops, 0, 0, 0, interval * 1000);	\
		}	\
\
		up(&g_tsData[num].sem_mutex);	\
		kfree(ptr_mtktsbts_data);	\
		/* AP_write_flag=1; */	\
		return count;	\
	}	\
\
	mtkts_bts_dprintk("[%s] bad argument\n", __func__);	\
	aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,	\
							"mtkts_bts_write",	\
							"Bad argument");	\
	kfree(ptr_mtktsbts_data);	\
	return -EINVAL;	\
}	\
\
static int mtkts_bts ## num ## _open(struct inode *inode, struct file *file)	\
{	\
	return single_open(file, mtkts_bts ## num ## _read, PDE_DATA(inode));	\
}	\
\
static const struct file_operations mtkts_bts ## num ## _fops = {	\
	.owner = THIS_MODULE,	\
	.open = mtkts_bts ## num ## _open,	\
	.read = seq_read,	\
	.llseek = seq_lseek,	\
	.write = mtkts_bts ## num ## _write,	\
	.release = single_release,	\
};

#define FOPS(num)	(&mtkts_bts ## num ## _fops)

FOPS_RW(0);
FOPS_RW(1);
FOPS_RW(2);
FOPS_RW(3);

static const struct file_operations *thz_fops[RESERVED_TZS] = {
	FOPS(0),
	FOPS(1),
	FOPS(2),
	FOPS(3),
};

static int mtkts_bts_param_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtkts_bts_param_read, NULL);
}

static const struct file_operations mtkts_AP_param_fops = {
	.owner = THIS_MODULE,
	.open = mtkts_bts_param_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtkts_bts_param_write,
	.release = single_release,
};

static int __init mtkts_bts_init(void)
{
	int i, j;
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtkts_AP_dir = NULL;

	mtkts_bts_dprintk("[%s]\n", __func__);

	for (i = 0; i < RESERVED_TZS; i++) {
		g_tsData[i].thz_dev = NULL;
		g_tsData[i].thz_name[0] = '\0';
		for (j = 0; j < NUM_TRIP_MAX; j++) {
			g_tsData[i].trip_temp[j] = 0;
			g_tsData[i].trip_type[j] = 0;
			g_tsData[i].bind[j][0] = '\0';
		}
		g_tsData[i].num_trip = 0;
		g_tsData[i].interval = 0;
		g_tsData[i].kernelmode = 0;
		sema_init(&g_tsData[i].sem_mutex, 1);
	}

	/* setup default table */
	mtkts_bts_prepare_table(bts_channel_param[0].g_RAP_ntc_table);

	mtkts_AP_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtkts_AP_dir) {
		mtkts_bts_dprintk("[%s]: mkdir /proc/driver/thermal failed\n",
								__func__);
	} else {
		for (i = 0; i < auxadc_channel_num; i++) {
			sprintf(g_tsData[i].thz_name, "tzbts%d", i);
			entry = proc_create(g_tsData[i].thz_name, 0664, mtkts_AP_dir,
							thz_fops[i]);
			if (entry)
				proc_set_user(entry, uid, gid);
		}

		entry = proc_create("tzbts_param", 0664, mtkts_AP_dir,
							&mtkts_AP_param_fops);
		if (entry)
			proc_set_user(entry, uid, gid);

	}
#if 0
	mtkTTimer_register("mtktsAP", mtkts_bts_start_thermal_timer,
					mtkts_bts_cancel_thermal_timer);
#endif
	return 0;
}

static void __exit mtkts_bts_exit(void)
{
	mtkts_bts_dprintk("[%s]\n", __func__);
	mtkts_bts_unregister_thermal();
#if 0
	mtkTTimer_unregister("mtktsAP");
#endif
}

late_initcall(mtkts_bts_init);
module_exit(mtkts_bts_exit);

static void parse_node_int(const struct device_node *np,
			    const char *node_name, int *cust_val)
{
	u32 val = 0;

	if (of_property_read_u32(np, node_name, &val) == 0) {
		(*cust_val) = (int)val;
		pr_debug("Get %s %d\n", node_name, *cust_val);
	} else
		pr_notice("Get %s failed\n", node_name);
}

static void parse_node_string_index(const struct device_node *np,
				const char *node_name,
				int index,
				int max_len,
				char *cust_string)
{
	const char *string;
	if (of_property_read_string_index(np, node_name, index, &string) == 0) {
		strncpy(cust_string, string, max_len);
		pr_debug("Get %s %s\n", node_name, cust_string);
	} else
		pr_notice("Get %s failed\n", node_name);
}

static void of_get_bts_init_data(struct device_node *np,
			struct mtkts_bts_channel_param *bts_param)
{
	parse_node_int(np, "pull_up_r", &(bts_param->g_RAP_pull_up_R));
	parse_node_int(np, "over_critical_low", &(bts_param->g_TAP_over_critical_low));
	parse_node_int(np, "pull_up_voltage", &(bts_param->g_RAP_pull_up_voltage));
	parse_node_int(np, "ntc_table", &(bts_param->g_RAP_ntc_table));
	parse_node_int(np, "adc_channel", &(bts_param->g_RAP_ADC_channel));
	parse_node_int(np, "temperature_r", &(bts_param->g_AP_TemperatureR));
	parse_node_string_index(np, "channel_name", 0,
		THERMAL_NAME_LENGTH - 1, bts_param->channelName);
}

static int init_cust_data_from_dt(struct platform_device *dev)
{
	__s32 i = 0;
	struct device_node *np = dev->dev.of_node;
	struct device_node *sub_np;

	auxadc_channel_num = of_get_child_count(np);
	if (!auxadc_channel_num) {
		return -EINVAL;
	}
	pr_notice("%s: auxadc_channel_num = %d\n", __func__, auxadc_channel_num);

	bts_channel_param = devm_kzalloc(&dev->dev,
			 sizeof(struct mtkts_bts_channel_param) * auxadc_channel_num,
			 GFP_KERNEL);
	if (!bts_channel_param) {
		pr_err("%s: bts_channel_param failed to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	thz_dev = devm_kzalloc(&dev->dev,
			 sizeof(struct thermal_zone_device *) * auxadc_channel_num,
			 GFP_KERNEL);
	if (!thz_dev) {
		pr_err("%s: thz_dev failed to allocate memory!\n", __func__);
		goto thz_dev_err;
	}

	for_each_child_of_node(np, sub_np) {
		of_get_bts_init_data(sub_np, &(bts_channel_param[i]));

		pr_notice("bts_channel_param[%d].g_RAP_pull_up_R %d\n",
			i, bts_channel_param[i].g_RAP_pull_up_R);
		pr_notice("bts_channel_param[%d].g_TAP_over_critical_low %d\n",
			i, bts_channel_param[i].g_TAP_over_critical_low);
		pr_notice("bts_channel_param[%d].g_RAP_pull_up_voltage %d\n",
			i, bts_channel_param[i].g_RAP_pull_up_voltage);
		pr_notice("bts_channel_param[%d].g_RAP_ntc_table %d\n", i,
			bts_channel_param[i].g_RAP_ntc_table);
		pr_notice("bts_channel_param[%d].g_RAP_ADC_channel %d\n",
			i, bts_channel_param[i].g_RAP_ADC_channel);
		pr_notice("bts_channel_param[%d].g_AP_TemperatureR %d\n",
			i, bts_channel_param[i].g_AP_TemperatureR);
		pr_notice("bts_channel_param[%d].channelName %s\n",
			i, bts_channel_param[i].channelName);

#ifdef CONFIG_THERMAL_DEBOUNCE
		bts_channel_param[i].BTS_temp_change = BTS_VALID_CHANGE_1;
#endif

		i++;
	}

	return 0;

thz_dev_err:
	devm_kfree(&dev->dev, bts_channel_param);
	return -ENOMEM;
}

static int gadc_thermal_probe(struct platform_device *pdev)
{
	struct gadc_thermal_info *gti;
	int ret;

	if (!pdev->dev.of_node) {
		dev_dbg(&pdev->dev, "Only DT based supported\n");
		return -ENODEV;
	}
	gti = devm_kzalloc(&pdev->dev, sizeof(*gti), GFP_KERNEL);
	if (!gti)
		return -ENOMEM;
	gti_ntc = gti;
	gti->dev = &pdev->dev;
	platform_set_drvdata(pdev, gti);
	gti->channel = iio_channel_get_all(&pdev->dev);
	if (IS_ERR(gti->channel)) {
		ret = PTR_ERR(gti->channel);
		dev_dbg(&pdev->dev, "IIO channel not found: %d\n", ret);
		return ret;
	}

	ret = init_cust_data_from_dt(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: init dts data error\n", __func__);
		iio_channel_release_all(gti->channel);
		return ret;
	}
#if 0
	gti->tz_dev = thermal_zone_of_sensor_register(&pdev->dev, 0,
						      gti, &gadc_thermal_ops);
	if (IS_ERR(gti->tz_dev)) {
		ret = PTR_ERR(gti->tz_dev);
		dev_dbg(&pdev->dev, "Thermal zone sensor register failed: %d\n",
			ret);
		goto sensor_fail;
	}
#endif
	pr_debug("%s OK\n", __func__);
	return 0;
#if 0
sensor_fail:
	iio_channel_release(gti->channel);
	return ret;
#endif
}
static int gadc_thermal_remove(struct platform_device *pdev)
{
	struct gadc_thermal_info *gti = platform_get_drvdata(pdev);

	iio_channel_release(gti->channel);
	return 0;
}
static const struct of_device_id of_adc_thermal_match[] = {
	{ .compatible = "generic-adc-thermal", },
	{},
};
MODULE_DEVICE_TABLE(of, of_adc_thermal_match);
static struct platform_driver gadc_thermal_driver = {
	.driver = {
		.name = "generic-adc-thermal",
		.of_match_table = of_adc_thermal_match,
	},
	.probe = gadc_thermal_probe,
	.remove = gadc_thermal_remove,
};
module_platform_driver(gadc_thermal_driver);
