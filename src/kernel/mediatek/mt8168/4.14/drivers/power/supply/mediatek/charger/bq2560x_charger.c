/*
 * BQ2560x battery charging driver
 *
 * Copyright (C) 2021 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define pr_fmt(fmt)	"[bq2560x]:%s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/timekeeping.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/syscore_ops.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/battery_metrics.h>

#include "mtk_charger_intf.h"
#include "bq2560x_reg.h"
#include "bq2560x.h"

#define ENABLE_BQ2560X_20HOURS_SAFETYTIMER
#define SGM41511_ITERM_VAL                    300
#define BQ25601_ITERM_VAL                     240
#define AN_SY6974_ITERM_VAL                   240

enum {
	PN_BQ25600,
	PN_BQ25600D,
	PN_BQ25601,
	PN_BQ25601D,
};

static int pn_data[] = {
	[PN_BQ25600] = 0x00,
	[PN_BQ25600D] = 0x01,
	[PN_BQ25601] = 0x02,
	[PN_BQ25601D] = 0x07,
};

static char *pn_str[] = {
	[PN_BQ25600] = "bq25600",
	[PN_BQ25600D] = "bq25600d",
	[PN_BQ25601] = "bq25601",
	[PN_BQ25601D] = "bq25601d",
};

enum bq2560x_part_no {
     BQ25601 = 0x02,
};

struct bq2560x {
	struct device *dev;
	struct i2c_client *client;

	enum bq2560x_part_no part_no;
	int revision;
	u8 reg0b_bit2;

	const char *chg_dev_name;
	const char *eint_name;

	bool chg_det_enable;

	enum charger_type chg_type;

	int status;
	int irq;

	struct mutex i2c_rw_lock;

	bool charge_enabled;	/* Register bit status */
	bool power_good;

	struct bq2560x_platform_data *platform_data;
	struct charger_device *chg_dev;

	struct power_supply *psy;
	enum ic_list cur_ic;
#ifdef ENABLE_BQ2560X_20HOURS_SAFETYTIMER
	unsigned int rst_safetytimer_times;
	unsigned int rst_safetytimer_times_remain;
	unsigned int rst_safetytimer_interval;
	struct timespec rst_safetytimer_record_time;
	bool rst_safetytimer_flag;
#endif
};

static struct bq2560x *g_bq;

static const struct charger_properties bq2560x_chg_props = {
	.alias_name = "bq2560x",
};

static int __bq2560x_read_reg(struct bq2560x *bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __bq2560x_write_reg(struct bq2560x *bq, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq2560x_read_byte(struct bq2560x *bq, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2560x_read_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2560x_write_byte(struct bq2560x *bq, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2560x_write_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}

static int bq2560x_update_bits(struct bq2560x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2560x_read_reg(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2560x_write_reg(bq, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int bq2560x_enable_otg(struct bq2560x *bq)
{
	u8 val = REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_01, REG01_OTG_CONFIG_MASK,
				   val);

}

static int bq2560x_disable_otg(struct bq2560x *bq)
{
	u8 val = REG01_OTG_DISABLE << REG01_OTG_CONFIG_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_01, REG01_OTG_CONFIG_MASK,
				   val);

}

#ifdef ENABLE_BQ2560X_20HOURS_SAFETYTIMER
/*
 * BQ2560x: 5H*2 + 10H = 20H
 * SGM41511: 3.5H*4 + 6H = 20H
 * AN_SY6974: 5H*2 + 10H = 20H
 */
#define BQ2560X_SAFETIMER_INTERVAL                     18000
#define SGM41511_SAFETIMER_INTERVAL                    12600
#define AN_SY6974_SAFETIMER_INTERVAL                   18000

#define BQ2560X_SAFETIMER_TIMES                         2
#define SGM41511_SAFETIMER_TIMES                        4
#define AN_SY6974_SAFETIMER_TIMES                       2

static int bq2560x_disable_safety_timer(struct bq2560x *bq);
static int bq2560x_enable_safety_timer(struct bq2560x *bq);

static void bq2560x_config_safetytimer(struct bq2560x *bq)
{
	int ret = 0;

	/* BQ2560x:10H  SGM41511:6H  AN_SY6974:10H */
	ret = bq2560x_update_bits(bq, BQ2560X_REG_05, REG05_CHG_TIMER_MASK,
		 REG05_CHG_TIMER_10HOURS << REG05_CHG_TIMER_SHIFT);
	if (ret)
		pr_err("%s:Failed to set BQ2560X_REG_05, ret = %d\n", __func__, ret);
	ret = bq2560x_update_bits(bq, BQ2560X_REG_07, REG07_TMR2X_EN_MASK,
		 REG07_TMR2X_DISABLE << REG07_TMR2X_EN_SHIFT);
	if (ret)
		pr_err("%s:Failed to disable  safety timer2X, ret = %d\n", __func__, ret);
}

static void bq2560x_start_reset_safetytimer(struct bq2560x *bq)
{
	struct timespec now_time;
	struct timespec diff;

	if (bq->rst_safetytimer_flag == false) {
		bq->rst_safetytimer_flag = true;
		bq->rst_safetytimer_times_remain = bq->rst_safetytimer_times;
		bq2560x_config_safetytimer(bq);
		get_monotonic_boottime(&bq->rst_safetytimer_record_time);
	} else {
		if (bq->rst_safetytimer_times_remain > 0) {
			get_monotonic_boottime(&now_time);
			diff = timespec_sub(now_time, bq->rst_safetytimer_record_time);
			pr_info("diff_time = %d, interval = %d, times_remain = %d\n",
				 diff.tv_sec, bq->rst_safetytimer_interval,
				 bq->rst_safetytimer_times_remain);
			if (diff.tv_sec >= bq->rst_safetytimer_interval) {
				bq->rst_safetytimer_times_remain--;
				bq2560x_disable_safety_timer(bq);
				msleep(100);
				bq2560x_enable_safety_timer(bq);
				(bq->rst_safetytimer_record_time).tv_sec +=
					 bq->rst_safetytimer_interval;
			}
		}
	}
}

static void bq2560x_stop_reset_safetytimer(struct bq2560x *bq)
{
	bq->rst_safetytimer_flag = false;
}

static void bq2560x_init_reset_safetimer(struct bq2560x *bq)
{
	bq2560x_config_safetytimer(bq);
	bq->rst_safetytimer_flag = false;
	if (bq->part_no == PN_VALUE_BQ25601_SGM41511) {
		if (bq->reg0b_bit2 == BIT2_VALUE_BQ25601) {
			bq->rst_safetytimer_times = BQ2560X_SAFETIMER_TIMES;
			bq->rst_safetytimer_interval = BQ2560X_SAFETIMER_INTERVAL;
		} else {
			bq->rst_safetytimer_times = SGM41511_SAFETIMER_TIMES;
			bq->rst_safetytimer_interval = SGM41511_SAFETIMER_INTERVAL;
		}
	} else if (bq->part_no == PN_VALUE_ANSY6974) {
		bq->rst_safetytimer_times = AN_SY6974_SAFETIMER_TIMES;
		bq->rst_safetytimer_interval = AN_SY6974_SAFETIMER_INTERVAL;
	}
	pr_info("[%s] times= %d interval = %d\n", __func__,
		 bq->rst_safetytimer_times, bq->rst_safetytimer_interval);
}
#else

static inline void bq2560x_start_reset_safetytimer(struct bq2560x *bq)
{
}

static inline void bq2560x_stop_reset_safetytimer(struct bq2560x *bq)
{
}

static inline void bq2560x_init_reset_safetimer(struct bq2560x *bq)
{
}
#endif

static int bq2560x_enable_charger(struct bq2560x *bq)
{
	int ret;
	u8 val = REG01_CHG_ENABLE << REG01_CHG_CONFIG_SHIFT;

	ret =
	    bq2560x_update_bits(bq, BQ2560X_REG_01, REG01_CHG_CONFIG_MASK, val);

	bq2560x_start_reset_safetytimer(bq);

	return ret;
}

static int bq2560x_disable_charger(struct bq2560x *bq)
{
	int ret;
	u8 val = REG01_CHG_DISABLE << REG01_CHG_CONFIG_SHIFT;

	ret =
	    bq2560x_update_bits(bq, BQ2560X_REG_01, REG01_CHG_CONFIG_MASK, val);

	bq2560x_stop_reset_safetytimer(bq);

	return ret;
}

int bq2560x_set_chargecurrent(struct bq2560x *bq, int curr)
{
	u8 ichg;

	if (curr < REG02_ICHG_BASE)
		curr = REG02_ICHG_BASE;

	ichg = (curr - REG02_ICHG_BASE) / REG02_ICHG_LSB;
	return bq2560x_update_bits(bq, BQ2560X_REG_02, REG02_ICHG_MASK,
				   ichg << REG02_ICHG_SHIFT);

}

int bq2560x_set_term_current(struct bq2560x *bq, int curr)
{
	u8 iterm;

	if (curr < REG03_ITERM_BASE)
		curr = REG03_ITERM_BASE;

	iterm = (curr - REG03_ITERM_BASE) / REG03_ITERM_LSB;

	return bq2560x_update_bits(bq, BQ2560X_REG_03, REG03_ITERM_MASK,
				   iterm << REG03_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2560x_set_term_current);

int bq2560x_set_prechg_current(struct bq2560x *bq, int curr)
{
	u8 iprechg;

	if (curr < REG03_IPRECHG_BASE)
		curr = REG03_IPRECHG_BASE;

	iprechg = (curr - REG03_IPRECHG_BASE) / REG03_IPRECHG_LSB;

	return bq2560x_update_bits(bq, BQ2560X_REG_03, REG03_IPRECHG_MASK,
				   iprechg << REG03_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2560x_set_prechg_current);

int bq2560x_set_chargevolt(struct bq2560x *bq, int volt)
{
	u8 val;

	if (volt < REG04_VREG_BASE)
		volt = REG04_VREG_BASE;

	val = (volt - REG04_VREG_BASE) / REG04_VREG_LSB;
	return bq2560x_update_bits(bq, BQ2560X_REG_04, REG04_VREG_MASK,
				   val << REG04_VREG_SHIFT);
}

int bq2560x_set_input_volt_limit(struct bq2560x *bq, int volt)
{
	u8 val;

	if (volt < REG06_VINDPM_BASE)
		volt = REG06_VINDPM_BASE;

	val = (volt - REG06_VINDPM_BASE) / REG06_VINDPM_LSB;
	return bq2560x_update_bits(bq, BQ2560X_REG_06, REG06_VINDPM_MASK,
				   val << REG06_VINDPM_SHIFT);
}

int bq2560x_set_input_current_limit(struct bq2560x *bq, int curr)
{
	u8 val;

	if (curr < REG00_IINLIM_BASE)
		curr = REG00_IINLIM_BASE;

	val = (curr - REG00_IINLIM_BASE) / REG00_IINLIM_LSB;
	return bq2560x_update_bits(bq, BQ2560X_REG_00, REG00_IINLIM_MASK,
				   val << REG00_IINLIM_SHIFT);
}

int bq2560x_set_watchdog_timer(struct bq2560x *bq, u8 timeout)
{
	u8 temp;

	temp = (u8) (((timeout -
		       REG05_WDT_BASE) / REG05_WDT_LSB) << REG05_WDT_SHIFT);

	return bq2560x_update_bits(bq, BQ2560X_REG_05, REG05_WDT_MASK, temp);
}
EXPORT_SYMBOL_GPL(bq2560x_set_watchdog_timer);

int bq2560x_disable_watchdog_timer(struct bq2560x *bq)
{
	u8 val = REG05_WDT_DISABLE << REG05_WDT_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_05, REG05_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2560x_disable_watchdog_timer);

int bq2560x_reset_watchdog_timer(struct bq2560x *bq)
{
	u8 val = REG01_WDT_RESET << REG01_WDT_RESET_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_01, REG01_WDT_RESET_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(bq2560x_reset_watchdog_timer);

int bq2560x_reset_chip(struct bq2560x *bq)
{
	int ret;
	u8 val = REG0B_REG_RESET << REG0B_REG_RESET_SHIFT;

	ret =
	    bq2560x_update_bits(bq, BQ2560X_REG_0B, REG0B_REG_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2560x_reset_chip);

int bq2560x_enter_hiz_mode(struct bq2560x *bq)
{
	u8 val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2560x_enter_hiz_mode);

int bq2560x_exit_hiz_mode(struct bq2560x *bq)
{

	u8 val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2560x_exit_hiz_mode);

int bq2560x_get_hiz_mode(struct bq2560x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_00, &val);
	if (ret)
		return ret;
	*state = (val & REG00_ENHIZ_MASK) >> REG00_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2560x_get_hiz_mode);

static int bq2560x_enable_term(struct bq2560x *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = REG05_TERM_ENABLE << REG05_EN_TERM_SHIFT;
	else
		val = REG05_TERM_DISABLE << REG05_EN_TERM_SHIFT;

	ret = bq2560x_update_bits(bq, BQ2560X_REG_05, REG05_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2560x_enable_term);

int bq2560x_set_boost_current(struct bq2560x *bq, int curr)
{
	u8 val;

	val = REG02_BOOST_LIM_0P5A;
	if (curr == BOOSTI_1200)
		val = REG02_BOOST_LIM_1P2A;

	return bq2560x_update_bits(bq, BQ2560X_REG_02, REG02_BOOST_LIM_MASK,
				   val << REG02_BOOST_LIM_SHIFT);
}

int bq2560x_set_boost_voltage(struct bq2560x *bq, int volt)
{
	u8 val;

	if (volt == BOOSTV_4850)
		val = REG06_BOOSTV_4P85V;
	else if (volt == BOOSTV_5150)
		val = REG06_BOOSTV_5P15V;
	else if (volt == BOOSTV_5300)
		val = REG06_BOOSTV_5P3V;
	else
		val = REG06_BOOSTV_5V;

	return bq2560x_update_bits(bq, BQ2560X_REG_06, REG06_BOOSTV_MASK,
				   val << REG06_BOOSTV_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2560x_set_boost_voltage);

static int bq2560x_set_acovp_threshold(struct bq2560x *bq, int volt)
{
	u8 val;

	if (volt == VAC_OVP_14000)
		val = REG06_OVP_14P0V;
	else if (volt == VAC_OVP_10500)
		val = REG06_OVP_10P5V;
	else if (volt == VAC_OVP_6500)
		val = REG06_OVP_6P5V;
	else
		val = REG06_OVP_5P5V;

	return bq2560x_update_bits(bq, BQ2560X_REG_06, REG06_OVP_MASK,
				   val << REG06_OVP_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2560x_set_acovp_threshold);

static int bq2560x_set_stat_ctrl(struct bq2560x *bq, int ctrl)
{
	u8 val;

	val = ctrl;

	return bq2560x_update_bits(bq, BQ2560X_REG_00, REG00_STAT_CTRL_MASK,
				   val << REG00_STAT_CTRL_SHIFT);
}

static int bq2560x_set_int_mask(struct bq2560x *bq, int mask)
{
	u8 val;

	val = mask;

	return bq2560x_update_bits(bq, BQ2560X_REG_0A, REG0A_INT_MASK_MASK,
				   val << REG0A_INT_MASK_SHIFT);
}

static int bq2560x_enable_batfet(struct bq2560x *bq)
{
	const u8 val = REG07_BATFET_ON << REG07_BATFET_DIS_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_07, REG07_BATFET_DIS_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(bq2560x_enable_batfet);

static int bq2560x_disable_batfet(struct bq2560x *bq)
{
	const u8 val = REG07_BATFET_OFF << REG07_BATFET_DIS_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_07, REG07_BATFET_DIS_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(bq2560x_disable_batfet);

static int bq2560x_set_batfet_delay(struct bq2560x *bq, uint8_t delay)
{
	u8 val;

	if (delay == 0)
		val = REG07_BATFET_DLY_0S;
	else
		val = REG07_BATFET_DLY_10S;

	val <<= REG07_BATFET_DLY_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_07, REG07_BATFET_DLY_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(bq2560x_set_batfet_delay);

static int bq2560x_enable_safety_timer(struct bq2560x *bq)
{
	const u8 val = REG05_CHG_TIMER_ENABLE << REG05_EN_TIMER_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_05, REG05_EN_TIMER_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(bq2560x_enable_safety_timer);

static int bq2560x_disable_safety_timer(struct bq2560x *bq)
{
	const u8 val = REG05_CHG_TIMER_DISABLE << REG05_EN_TIMER_SHIFT;

	return bq2560x_update_bits(bq, BQ2560X_REG_05, REG05_EN_TIMER_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(bq2560x_disable_safety_timer);

static struct bq2560x_platform_data *bq2560x_parse_dt(struct device_node *np,
						      struct bq2560x *bq)
{
	int ret;
	struct bq2560x_platform_data *pdata;
	struct device *dev = bq->dev;

	pdata = devm_kzalloc(dev, sizeof(struct bq2560x_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (of_property_read_string(np, "charger_name", &bq->chg_dev_name) < 0) {
		bq->chg_dev_name = "primary_chg";
		pr_warn("no charger name\n");
	}

	if (of_property_read_string(np, "eint_name", &bq->eint_name) < 0) {
		bq->eint_name = "chr_stat";
		pr_warn("no eint name\n");
	}

	bq->chg_det_enable =
	    of_property_read_bool(np, "ti,bq2560x,charge-detect-enable");

	ret = of_property_read_u32(np, "ti,bq2560x,usb-vlim", &pdata->usb.vlim);
	if (ret) {
		pdata->usb.vlim = 4500;
		pr_err("Failed to read node of ti,bq2560x,usb-vlim\n");
	}

	ret = of_property_read_u32(np, "ti,bq2560x,usb-ilim", &pdata->usb.ilim);
	if (ret) {
		pdata->usb.ilim = 2000;
		pr_err("Failed to read node of ti,bq2560x,usb-ilim\n");
	}

	ret = of_property_read_u32(np, "ti,bq2560x,usb-vreg", &pdata->usb.vreg);
	if (ret) {
		pdata->usb.vreg = 4200;
		pr_err("Failed to read node of ti,bq2560x,usb-vreg\n");
	}

	ret = of_property_read_u32(np, "ti,bq2560x,usb-ichg", &pdata->usb.ichg);
	if (ret) {
		pdata->usb.ichg = 2000;
		pr_err("Failed to read node of ti,bq2560x,usb-ichg\n");
	}

	ret = of_property_read_u32(np, "ti,bq2560x,stat-pin-ctrl",
				   &pdata->statctrl);
	if (ret) {
		pdata->statctrl = 0;
		pr_err("Failed to read node of ti,bq2560x,stat-pin-ctrl\n");
	}

	ret = of_property_read_u32(np, "ti,bq2560x,precharge-current",
				   &pdata->iprechg);
	if (ret) {
		pdata->iprechg = 180;
		pr_err("Failed to read node of ti,bq2560x,precharge-current\n");
	}

	ret = of_property_read_u32_array(np, "ti,bq2560x,termination-current",
		pdata->iterm, ARRAY_SIZE(pdata->iterm));
	if (ret) {
		pdata->iterm[IC_SGM41511] = SGM41511_ITERM_VAL;
		pdata->iterm[IC_BQ25601] = BQ25601_ITERM_VAL;
		pdata->iterm[IC_AN_SY6974] = AN_SY6974_ITERM_VAL;
		pr_err("Failed to read node of ti,bq2560x,termination-current\n");
	}

	ret =
	    of_property_read_u32(np, "ti,bq2560x,boost-voltage",
				 &pdata->boostv);
	if (ret) {
		pdata->boostv = 5000;
		pr_err("Failed to read node of ti,bq2560x,boost-voltage\n");
	}

	ret =
	    of_property_read_u32(np, "ti,bq2560x,boost-current",
				 &pdata->boosti);
	if (ret) {
		pdata->boosti = 1200;
		pr_err("Failed to read node of ti,bq2560x,boost-current\n");
	}

	ret = of_property_read_u32(np, "ti,bq2560x,vac-ovp-threshold",
				   &pdata->vac_ovp);
	if (ret) {
		pdata->vac_ovp = 6500;
		pr_err("Failed to read node of ti,bq2560x,vac-ovp-threshold\n");
	}

	return pdata;
}

static int check_fault_register(struct bq2560x *bq)
{
	int ret;
	u8 fault_val = 0;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_09, &fault_val);
	if (ret) {
		pr_err("%s: read fault register failed\n", __func__);
		return -1;
	}

	/*
	 * Before the host reads REG09 and all the faults are cleared, the charger device would not send
	 * any INT upon new faults. To read the current fault status, the host has to read REG09 two times consecutively
	 */
	ret = bq2560x_read_byte(bq, BQ2560X_REG_09, &fault_val);
	if (ret) {
		pr_err("%s: read fault register failed\n", __func__);
		return -1;
	}

	if (fault_val == 0)
		return 0;

	switch (fault_val) {
	case REG09_FAULT_BOOST_MASK:
		pr_err("%s: vbus ovp\n", __func__);
		bat_metrics_chg_fault(METRICS_FAULT_VBUS_OVP);
		break;
	case REG09_FAULT_BAT_MASK:
		pr_err("%s: vbat ovp\n", __func__);
		bat_metrics_chg_fault(METRICS_FAULT_VBAT_OVP);
		break;
	case REG09_FAULT_CHRG_MASK:
		pr_err("%s: safety timer timeout\n", __func__);
		bat_metrics_chg_fault(METRICS_FAULT_SAFETY_TIMEOUT);
		break;
	case REG09_FAULT_WDT_MASK:
		pr_err("%s: Watchdog timer expiration\n", __func__);
		break;
	case REG09_FAULT_NTC_COLD:
		pr_err("%s: NTC cold fault\n", __func__);
		break;
	case REG09_FAULT_NTC_HOT:
		pr_err("%s: NTC hot fault\n", __func__);
		break;
	case REG09_FAULT_NTC_COOL:
		pr_err("%s: NTC cool fault\n", __func__);
		break;
	case REG09_FAULT_NTC_WARM:
		pr_err("%s: NTC warm fault\n", __func__);
		break;
	default:
		break;
	}

	return fault_val;
}

static irqreturn_t bq2560x_irq_handler(int irq, void *data)
{
	int ret;
	u8 reg_val;
	bool prev_pg;
	struct bq2560x *bq = data;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_08, &reg_val);
	if (ret)
		return IRQ_HANDLED;

	charger_dev_notify(bq->chg_dev, CHARGER_DEV_NOTIFY_VBUS_EVENT);

	prev_pg = bq->power_good;

	bq->power_good = !!(reg_val & REG08_PG_STAT_MASK);

	if (!prev_pg && bq->power_good)
		pr_notice("adapter/usb inserted\n");
	else if (prev_pg && !bq->power_good)
		pr_notice("adapter/usb removed\n");


	ret = check_fault_register(bq);
	if (ret < 0)
		pr_err("%s: Fail to check fault register\n", __func__);

	pr_info("[%s] bq2560x_irq_handler ok\n", __func__);

	return IRQ_HANDLED;
}

static int bq2560x_register_interrupt(struct bq2560x *bq)
{
	int ret = 0;

    if(!bq->client->irq){
		pr_info("bq2560x bq->client->irq is null\n");
			return -ENODEV;
    	}
	ret = devm_request_threaded_irq(bq->dev, bq->client->irq, NULL,
					bq2560x_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"ti_irq", bq);
	if (ret < 0) {
		pr_err("request thread irq failed:%d\n", ret);
		return ret;
	}

	enable_irq_wake(bq->irq);
	pr_info("bq2560x_register_interrupt ok\n");

	return 0;
}

static int bq2560x_init_device(struct bq2560x *bq)
{
	int ret;
	pr_info("[%s] cwy1110 bq2560x_init_device open\n", __func__);

	bq2560x_disable_watchdog_timer(bq);

	ret = bq2560x_set_stat_ctrl(bq, bq->platform_data->statctrl);
	if (ret)
		pr_err("Failed to set stat pin control mode, ret = %d\n", ret);

	ret = bq2560x_set_prechg_current(bq, bq->platform_data->iprechg);
	if (ret)
		pr_err("Failed to set prechg current, ret = %d\n", ret);

	ret = bq2560x_set_term_current(bq, bq->platform_data->iterm[bq->cur_ic]);
	if (ret)
		pr_err("Failed to set termination current, ret = %d\n", ret);

	ret = bq2560x_set_boost_voltage(bq, bq->platform_data->boostv);
	if (ret)
		pr_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = bq2560x_set_boost_current(bq, bq->platform_data->boosti);
	if (ret)
		pr_err("Failed to set boost current, ret = %d\n", ret);

	ret = bq2560x_set_acovp_threshold(bq, bq->platform_data->vac_ovp);
	if (ret)
		pr_err("Failed to set acovp threshold, ret = %d\n", ret);

	ret = bq2560x_set_int_mask(bq,
				   REG0A_IINDPM_INT_MASK |
				   REG0A_VINDPM_INT_MASK);
	if (ret)
		pr_err("Failed to set vindpm and iindpm int mask\n");

	pr_info("[%s] bq2560x_init_device oK\n", __func__);

	return 0;
}

static void determine_initial_status(struct bq2560x *bq)
{
	bq2560x_irq_handler(bq->irq, (void *) bq);
}

static int bq2560x_detect_device(struct bq2560x *bq)
{
	int ret;
	u8 data;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_0B, &data);
	if(ret) {
		pr_err("%s Read reg0b is err\n",__func__);
		return ret;
	}
	bq->part_no = (data >> REG0B_PN_SHIFT)&REG0B_PN_MASK;
	bq->reg0b_bit2 = (data >> REG0B_BIT2_SHIFT)&REG0B_BIT2_MASK;
	bq->revision = (data & REG0B_DEV_REV_MASK) >> REG0B_DEV_REV_SHIFT;
	if (bq->part_no == PN_VALUE_BQ25601_SGM41511) {
		if (bq->reg0b_bit2 == BIT2_VALUE_BQ25601) {
			pr_notice("[%s] Charge IC is BQ25601\n", __func__);
			bq->cur_ic = IC_BQ25601;
		} else {
			pr_notice("[%s] Charge IC is SGM41511\n", __func__);
			bq->cur_ic = IC_SGM41511;
		}
	} else if (bq->part_no == PN_VALUE_ANSY6974) {
		pr_notice("[%s] Charge IC is AN_SY6974\n", __func__);
		bq->cur_ic = IC_AN_SY6974;
	} else {
		pr_err("[%s] Charge IC detect is error\n", __func__);
		return -1;
	}
	pr_notice("%s[ret:%.2x], [bit2:%.2x], [pn:%.2x], [rev%.2x]\n", __func__,
			 ret, bq->reg0b_bit2, bq->part_no, bq->revision);
	return ret;
}

static void bq2560x_dump_regs(struct bq2560x *bq)
{
	int addr, ret, len, idx = 0;
	u8 val, tmpbuf[10];
	char buf[100] = {0};

	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = bq2560x_read_byte(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, 10, "%.1x:%.2x ", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}
	pr_info("%s\n", buf);
}

static int bq2560x_get_registers(struct charger_device *chg_dev, char *buf, int length)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 addr;
	u8 val;
	int len;
	int idx = 0;
	int ret;
	char tmpbuf[80] = {0};

	for (addr = 0x0; addr <= 0x0a; addr++) {
		ret = bq2560x_read_byte(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(&tmpbuf[idx], length - idx, "%.1x:%.2x ", addr, val);
			idx += len;
		}
	}

	return snprintf(buf, length, "%s", tmpbuf);
}

static int bq2560x_get_vendor(struct charger_device *chg_dev, char *buf, int length)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	int idx = 0;

	if (bq->part_no == PN_VALUE_BQ25601_SGM41511) {
		if (bq->reg0b_bit2 == BIT2_VALUE_BQ25601)
			idx = snprintf(buf, length, "%s", "BQ25601\n");
		else
			idx = snprintf(buf, length, "%s", "SGM41511\n");
	} else if (bq->part_no == PN_VALUE_ANSY6974) {
		idx = snprintf(buf, length, "%s", "AN_SY6974\n");
	} else {
		idx = snprintf(buf, length, "%s", "ERROR\n");
	}
	return idx;
}

static ssize_t
bq2560x_show_registers(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct bq2560x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[10];
	int len;
	int idx = 0;
	int ret;

	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = bq2560x_read_byte(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, 10, "%.1x:%.2x ", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t
bq2560x_show_vendor(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct bq2560x *bq = dev_get_drvdata(dev);
	int idx = 0;

	if (bq->part_no == PN_VALUE_BQ25601_SGM41511) {
		if (bq->reg0b_bit2 == BIT2_VALUE_BQ25601)
			idx = snprintf(buf, PAGE_SIZE, "%s", "BQ25601\n");
		else
			idx = snprintf(buf, PAGE_SIZE, "%s", "SGM41511\n");
	} else if (bq->part_no == PN_VALUE_ANSY6974)
		idx = snprintf(buf, PAGE_SIZE, "%s", "AN_SY6974\n");
	else {
		idx = snprintf(buf, PAGE_SIZE, "%s", "ERROR\n");
	}
	return idx;
}

static ssize_t
bq2560x_store_registers(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct bq2560x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < 0x0B) {
		bq2560x_write_byte(bq, (unsigned char) reg,
				   (unsigned char) val);
	}

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, bq2560x_show_registers,
		   bq2560x_store_registers);

static DEVICE_ATTR(vendor, S_IRUGO, bq2560x_show_vendor, NULL);

static struct attribute *bq2560x_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_vendor.attr,
	NULL,
};

static const struct attribute_group bq2560x_attr_group = {
	.attrs = bq2560x_attributes,
};

static int bq2560x_charging(struct charger_device *chg_dev, bool enable)
{

	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;

	if (enable)
		ret = bq2560x_enable_charger(bq);
	else
		ret = bq2560x_disable_charger(bq);

	pr_err("%s charger %s\n", enable ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	ret = bq2560x_read_byte(bq, BQ2560X_REG_01, &val);

	if (!ret)
		bq->charge_enabled = !!(val & REG01_CHG_CONFIG_MASK);

	return ret;
}

static int bq2560x_plug_in(struct charger_device *chg_dev)
{

	int ret;

	ret = bq2560x_charging(chg_dev, true);

	if (ret)
		pr_err("Failed to enable charging:%d\n", ret);

	return ret;
}

static int bq2560x_plug_out(struct charger_device *chg_dev)
{
	int ret;

	ret = bq2560x_charging(chg_dev, false);

	if (ret)
		pr_err("Failed to disable charging:%d\n", ret);

	return ret;
}

static int bq2560x_dump_register(struct charger_device *chg_dev)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);

	bq2560x_dump_regs(bq);

	return 0;
}

static int bq2560x_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);

	*en = bq->charge_enabled;

	return 0;
}

static int bq2560x_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_08, &val);
	if (!ret) {
		val = val & REG08_CHRG_STAT_MASK;
		val = val >> REG08_CHRG_STAT_SHIFT;
		*done = (val == REG08_CHRG_STAT_CHGDONE);
	}

	return ret;
}

static int bq2560x_enable_power_path(struct charger_device *chg_dev, bool en)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 val;

	if (en) {
		val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;
		return bq2560x_update_bits(bq, BQ2560X_REG_00, REG00_ENHIZ_MASK, val);
	} else {
		val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;
		return bq2560x_update_bits(bq, BQ2560X_REG_00, REG00_ENHIZ_MASK, val);
	}
}

static int bq2560x_is_power_path_enable(struct charger_device *chg_dev, bool *en)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_00, &val);
	if (!ret) {
		val = val & REG00_ENHIZ_MASK;
		val = val >> REG00_ENHIZ_SHIFT;
		*en = (val == REG00_HIZ_DISABLE);
	}
	return ret;
}

static int bq2560x_set_ichg(struct charger_device *chg_dev, u32 curr)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);

	pr_err("charge curr = %d\n", curr);

	return bq2560x_set_chargecurrent(bq, curr / 1000);
}

static int bq2560x_get_ichg(struct charger_device *chg_dev, u32 *curr)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int ichg;
	int ret;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_02, &reg_val);
	if (!ret) {
		ichg = (reg_val & REG02_ICHG_MASK) >> REG02_ICHG_SHIFT;
		ichg = ichg * REG02_ICHG_LSB + REG02_ICHG_BASE;
		*curr = ichg * 1000;
	}

	return ret;
}

static int bq2560x_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	*curr = 60 * 1000;

	return 0;
}

static int bq2560x_set_vchg(struct charger_device *chg_dev, u32 volt)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);

	pr_err("charge volt = %d\n", volt);

	return bq2560x_set_chargevolt(bq, volt / 1000);
}

static int bq2560x_get_vchg(struct charger_device *chg_dev, u32 *volt)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int vchg;
	int ret;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_04, &reg_val);
	if (!ret) {
		vchg = (reg_val & REG04_VREG_MASK) >> REG04_VREG_SHIFT;
		vchg = vchg * REG04_VREG_LSB + REG04_VREG_BASE;
		*volt = vchg * 1000;
	}

	return ret;
}

static int bq2560x_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);

	pr_err("vindpm volt = %d\n", volt);

	return bq2560x_set_input_volt_limit(bq, volt / 1000);
}

static int bq2560x_get_ivl(struct charger_device *chg_dev, u32 *uV)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int vchg;
	int ret;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_06, &reg_val);
	if (!ret) {
		vchg = (reg_val & REG06_VINDPM_MASK) >> REG06_VINDPM_SHIFT;
		vchg = vchg * REG06_VINDPM_LSB + REG06_VINDPM_BASE;
		*uV = vchg * 1000;
	}

	return ret;
}

static int bq2560x_get_indpm_state(struct charger_device *chg_dev,
				bool *vdpm, bool *idpm)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 val;
	int ret;

	*vdpm = false;
	*idpm = false;
	if (bq->charge_enabled) {
		ret = bq2560x_read_byte(bq, BQ2560X_REG_0A, &val);
		if (ret)
			return ret;
		*vdpm = !!(val & REG0A_VINDPM_STAT_MASK);
		*idpm = !!(val & REG0A_IINDPM_STAT_MASK);
	}

	return 0;
}

static int bq2560x_set_icl(struct charger_device *chg_dev, u32 curr)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);

	pr_err("indpm curr = %d\n", curr);

	return bq2560x_set_input_current_limit(bq, curr / 1000);
}

static int bq2560x_get_icl(struct charger_device *chg_dev, u32 *curr)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int icl;
	int ret;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_00, &reg_val);
	if (!ret) {
		icl = (reg_val & REG00_IINLIM_MASK) >> REG00_IINLIM_SHIFT;
		icl = icl * REG00_IINLIM_LSB + REG00_IINLIM_BASE;
		*curr = icl * 1000;
	}

	return ret;

}

static int bq2560x_kick_wdt(struct charger_device *chg_dev)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);

	return bq2560x_reset_watchdog_timer(bq);
}

static int bq2560x_set_otg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);

	if (en)
		ret = bq2560x_enable_otg(bq);
	else
		ret = bq2560x_disable_otg(bq);

	pr_err("%s OTG %s\n", en ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	return ret;
}

static int bq2560x_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;

	if (en)
		ret = bq2560x_enable_safety_timer(bq);
	else
		ret = bq2560x_disable_safety_timer(bq);

	return ret;
}

static int bq2560x_is_safety_timer_enabled(struct charger_device *chg_dev,
					   bool *en)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = bq2560x_read_byte(bq, BQ2560X_REG_05, &reg_val);

	if (!ret)
		*en = !!(reg_val & REG05_EN_TIMER_MASK);

	return ret;
}

static int bq2560x_set_boost_ilmt(struct charger_device *chg_dev, u32 curr)
{
	struct bq2560x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;

	pr_err("otg curr = %d\n", curr);

	ret = bq2560x_set_boost_current(bq, curr / 1000);

	return ret;
}

static int bq2560x_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}
	return 0;
}

static struct charger_ops bq2560x_chg_ops = {
	/* Normal charging */
	.plug_in = bq2560x_plug_in,
	.plug_out = bq2560x_plug_out,
	.dump_registers = bq2560x_dump_register,
	.enable = bq2560x_charging,
	.is_enabled = bq2560x_is_charging_enable,
	.get_charging_current = bq2560x_get_ichg,
	.set_charging_current = bq2560x_set_ichg,
	.get_input_current = bq2560x_get_icl,
	.set_input_current = bq2560x_set_icl,
	.get_constant_voltage = bq2560x_get_vchg,
	.set_constant_voltage = bq2560x_set_vchg,
	.kick_wdt = bq2560x_kick_wdt,
	.set_mivr = bq2560x_set_ivl,
	.get_mivr = bq2560x_get_ivl,
	.get_indpm_state = bq2560x_get_indpm_state,
	.is_charging_done = bq2560x_is_charging_done,
	.get_min_charging_current = bq2560x_get_min_ichg,

	/* Safety timer */
	.enable_safety_timer = bq2560x_set_safety_timer,
	.is_safety_timer_enabled = bq2560x_is_safety_timer_enabled,
	.enable_chg_type_det = NULL,
	/* Power path */
	.enable_powerpath = bq2560x_enable_power_path,
	.is_powerpath_enabled = bq2560x_is_power_path_enable,

	/* OTG */
	.enable_otg = bq2560x_set_otg,
	.set_boost_current_limit = bq2560x_set_boost_ilmt,

	/* Event */
	.event = bq2560x_do_event,

	.enable_discharge = NULL,

	/* PE+/PE+20 */
	.send_ta_current_pattern = NULL,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
	.enable_cable_drop_comp = NULL,

	/* ADC */
	.get_tchg_adc = NULL,

	/*vendor&reg*/
	.get_vendor = bq2560x_get_vendor,
	.get_reg = bq2560x_get_registers,
};

static struct of_device_id bq2560x_charger_match_table[] = {
	{
	 .compatible = "ti,bq25600",
	 .data = &pn_data[PN_BQ25600],
	 },
	{
	 .compatible = "ti,bq25600D",
	 .data = &pn_data[PN_BQ25600D],
	 },
	{
	 .compatible = "ti,bq25601",
	 .data = &pn_data[PN_BQ25601],
	 },
	{
	 .compatible = "ti,bq25601D",
	 .data = &pn_data[PN_BQ25601D],
	 },
	{},
};
MODULE_DEVICE_TABLE(of, bq2560x_charger_match_table);

static void bq2560x_charger_shutdown(void)
{
	int ret = 0;

	devm_free_irq(g_bq->dev, g_bq->client->irq, g_bq);
	ret = bq2560x_disable_otg(g_bq);
	if (ret) {
		pr_err("Failed to bq2560x disable otg\n", ret);
	}
	mutex_destroy(&g_bq->i2c_rw_lock);
	pr_info("[%s] OK\n", __func__);
}

static struct syscore_ops bq2560x_syscore_ops = {
	.shutdown = bq2560x_charger_shutdown,
};

static int bq2560x_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq2560x *bq;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;

	int ret = 0;

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2560x), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);

	ret = bq2560x_detect_device(bq);
	if (ret) {
		pr_err("No bq2560x device found!\n");
		devm_kfree(&client->dev, bq);
		return -ENODEV;
	}
	pr_notice("bq2560x device found!\n");
	bq2560x_init_reset_safetimer(bq);
	match = of_match_node(bq2560x_charger_match_table, node);
	if (match == NULL) {
		pr_err("device tree match not found\n");
		devm_kfree(&client->dev, bq);
		return -EINVAL;
	}

	if (bq->part_no != *(int *)match->data)
		pr_info("part no mismatch, hw:%s, devicetree:%s\n",
			pn_str[bq->part_no], pn_str[*(int *) match->data]);

	bq->platform_data = bq2560x_parse_dt(node, bq);

	if (!bq->platform_data) {
		pr_err("No platform data provided.\n");
		return -EINVAL;
	}

	ret = bq2560x_init_device(bq);
	if (ret) {
		pr_err("Failed to init device\n");
		return ret;
	}

	bq->chg_dev = charger_device_register(bq->chg_dev_name,
					      &client->dev, bq,
					      &bq2560x_chg_ops,
					      &bq2560x_chg_props);
	if (IS_ERR_OR_NULL(bq->chg_dev)) {
		ret = PTR_ERR(bq->chg_dev);
		pr_info("[%s] charger_device_register failed (%d)\n", __func__,ret);
		return ret;
	}
	pr_info("[%s] charger_device_register OK(%d)\n", __func__,ret);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2560x_attr_group);
	if (ret)
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);

	register_syscore_ops(&bq2560x_syscore_ops);
	g_bq = bq;
	bq2560x_register_interrupt(g_bq);
	determine_initial_status(g_bq);

	pr_info("[%s] bq2560x probe successfully, Part Num:%d, Revision:%d\n!",
		 __func__, bq->part_no, bq->revision);

	return 0;
}

static int bq2560x_charger_remove(struct i2c_client *client)
{
	struct bq2560x *bq = i2c_get_clientdata(client);

	sysfs_remove_group(&bq->dev->kobj, &bq2560x_attr_group);

	return 0;
}

static struct i2c_driver bq2560x_charger_driver = {
	.driver = {
		   .name = "bq2560x-charger",
		   .owner = THIS_MODULE,
		   .of_match_table = bq2560x_charger_match_table,
		   },

	.probe = bq2560x_charger_probe,
	.remove = bq2560x_charger_remove,
};

module_i2c_driver(bq2560x_charger_driver);

MODULE_DESCRIPTION("TI BQ2560x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
