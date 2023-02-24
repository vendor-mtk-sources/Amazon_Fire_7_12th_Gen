/* SPDX-License-Identifier: GPL-2.0-only */
// SGM4154X Charger Driver
/*
 * Copyright (C) 2022 Amazon.com Inc. or its affiliates.  All Rights Reserved.
 */

#define pr_fmt(fmt)	"[sgm4154x]:%s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
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
#include <mt-plat/battery_metrics.h>
#include "mtk_charger_intf.h"
#include "sgm4154x.h"

struct sgm4154x {
	struct device *dev;
	struct i2c_client *client;
	struct charger_device *chg_dev;

	const char *chg_dev_name;
	const char *eint_name;
	struct mutex i2c_rw_lock;

	bool charge_enabled;	/* Register bit status */
	bool power_good;
	int status;
	int irq;

	int iprechg;
	int iterm;
	int statctrl;
	int boostv;
	int boosti;
	int vac_ovp;
	int safety_hours;
};

static struct sgm4154x *g_sgm;

static const struct charger_properties sgm4154x_chg_props = {
	.alias_name = "sgm4154x",
};

#define SGM4154X_REG_NUM    (0xF)

/* SGM4154X REG06 BOOST_LIM[5:4], uV */
static const unsigned int BOOST_VOLT_LIMIT[] = {
	4850000, 5000000, 5150000, 5300000
};

/* SGM4154X REG06 VAC OVP[7:6], uV */
static const unsigned int VAC_OVP[] = {
	5500000, 6500000, 10500000, 12000000
};

 /* SGM4154X REG02 BOOST_LIM[7:7], uA */
#if (defined(__SGM41542_CHIP_ID__) || defined(__SGM41541_CHIP_ID__) || defined(__SGM41543_CHIP_ID__) || defined(__SGM41543D_CHIP_ID__))
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	1200000, 2000000
};
#else
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	500000, 1200000
};
#endif

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
static const unsigned int IPRECHG_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};

static const unsigned int ITERM_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};
#endif

static int __sgm4154x_read_reg(struct sgm4154x *sgm, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(sgm->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read reg 0x%02X: %d\n", reg, ret);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __sgm4154x_write_reg(struct sgm4154x *sgm, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(sgm->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

static int sgm4154x_read_byte(struct sgm4154x *sgm, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_read_reg(sgm, reg, data);
	mutex_unlock(&sgm->i2c_rw_lock);

	return ret;
}

static int sgm4154x_write_byte(struct sgm4154x *sgm, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_write_reg(sgm, reg, data);
	mutex_unlock(&sgm->i2c_rw_lock);

	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}

static int sgm4154x_update_bits(struct sgm4154x *sgm, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_read_reg(sgm, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sgm4154x_write_reg(sgm, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&sgm->i2c_rw_lock);
	return ret;
}

static int sgm4154x_enable_otg(struct sgm4154x *sgm)
{
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_1,
			SGM4154X_OTG_EN, SGM4154X_OTG_EN);
}

static int sgm4154x_disable_otg(struct sgm4154x *sgm)
{
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_1,
			SGM4154X_OTG_EN, 0);
}

static int sgm4154x_enable_charger(struct sgm4154x *sgm)
{
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_1,
			SGM4154X_CHRG_EN, SGM4154X_CHRG_EN);
}

static int sgm4154x_disable_charger(struct sgm4154x *sgm)
{
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_1,
			SGM4154X_CHRG_EN, 0);
}

static int sgm4154x_set_chargecurrent(struct sgm4154x *sgm, int uA)
{
	u8 reg_val;

	if (uA < SGM4154X_ICHRG_I_MIN_uA)
		uA = SGM4154X_ICHRG_I_MIN_uA;
	if (uA > SGM4154X_ICHRG_I_MAX_uA)
		uA = SGM4154X_ICHRG_I_MAX_uA;

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	if (uA <= 40000)
		reg_val = uA / 5000;
	else if (uA <= 110000)
		reg_val = 0x08 + (uA - 40000) / 10000;
	else if (uA <= 270000)
		reg_val = 0x0F + (uA - 110000) / 20000;
	else if (uA <= 540000)
		reg_val = 0x17 + (uA - 270000) / 30000;
	else if (uA <= 1500000)
		reg_val = 0x20 + (uA - 540000) / 60000;
	else if (uA <= 2940000)
		reg_val = 0x30 + (uA - 1500000) / 120000;
	else
		reg_val = 0x3d;
#else
	reg_val = uA / SGM4154X_ICHRG_I_STEP_uA;
#endif

	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_2,
				  SGM4154X_ICHRG_I_MASK, reg_val);
}

static int sgm4154x_set_term_current(struct sgm4154x *sgm, int uA)
{
	u8 reg_val;

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	for (reg_val = 1; reg_val < sizeof(ITERM_CURRENT_STABLE)/sizeof(ITERM_CURRENT_STABLE[0]) && uA >= ITERM_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#else
	if (uA < SGM4154X_TERMCHRG_I_MIN_uA)
		uA = SGM4154X_TERMCHRG_I_MIN_uA;
	if (uA > SGM4154X_TERMCHRG_I_MAX_uA)
		uA = SGM4154X_TERMCHRG_I_MAX_uA;

	reg_val = (uA - SGM4154X_TERMCHRG_I_MIN_uA) / SGM4154X_TERMCHRG_CU_STEP_uA;
#endif

	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_3,
				  SGM4154X_TERMCHRG_CUR_MASK, reg_val);
}

static int sgm4154x_set_prechg_current(struct sgm4154x *sgm, int uA)
{
	u8 reg_val;

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	for (reg_val = 1; reg_val < sizeof(IPRECHG_CURRENT_STABLE)/sizeof(IPRECHG_CURRENT_STABLE[0]) && uA >= IPRECHG_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#else
	if (uA < SGM4154X_PRECHRG_I_MIN_uA)
		uA = SGM4154X_PRECHRG_I_MIN_uA;
	if (uA > SGM4154X_PRECHRG_I_MAX_uA)
		uA = SGM4154X_PRECHRG_I_MAX_uA;

	reg_val = (uA - SGM4154X_PRECHRG_I_MIN_uA) / SGM4154X_PRECHRG_CURRENT_STEP_uA;
#endif
	reg_val = reg_val << SGM4154X_PRECHRG_I_SHIFT;
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_3,
				  SGM4154X_PRECHRG_CUR_MASK, reg_val);
}

static int sgm4154x_set_chargevolt(struct sgm4154x *sgm, int uV)
{
	u8 reg_val;

	if (uV < SGM4154X_VREG_V_MIN_uV)
		uV = SGM4154X_VREG_V_MIN_uV;
	if (uV > SGM4154X_VREG_V_MAX_uV)
		uV = SGM4154X_VREG_V_MAX_uV;

	reg_val = (uV - SGM4154X_VREG_V_MIN_uV) / SGM4154X_VREG_V_STEP_uV;
	reg_val = reg_val << 3;
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_4,
				  SGM4154X_VREG_V_MASK, reg_val);
}

static int sgm4154x_set_input_volt_limit(struct sgm4154x *sgm, int uV)
{
	int ret;
	u8 reg_mivr = 0;
	u8 os_val;
	unsigned int offset;

	if (uV < SGM4154X_VINDPM_V_MIN_uV)
		uV = SGM4154X_VINDPM_V_MIN_uV;
	if (uV > SGM4154X_VINDPM_V_MAX_uV)
		uV = SGM4154X_VINDPM_V_MAX_uV;

	if (uV < 5900000) {
		os_val = 0;
		offset = 3900000;
	} else if (uV >= 5900000 && uV < 7500000) {
		os_val = 1;
		offset = 5900000;
	} else if (uV >= 7500000 && uV < 10500000) {
		os_val = 2;
		offset = 7500000;
	} else {
		os_val = 3;
		offset = 10500000;
	}

	ret = sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_f,
				  SGM4154X_VINDPM_OS_MASK, os_val);
	if (ret) {
		pr_err("%s set vindpm offset fail\n", __func__);
		return ret;
	}

	reg_mivr = (uV - offset) / SGM4154X_VINDPM_STEP_uV;

	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_6,
				  SGM4154X_VINDPM_V_MASK, reg_mivr);
}

static int sgm4154x_set_input_current_limit(struct sgm4154x *sgm, int uA)
{
	u8 reg_val;

	if (uA < SGM4154X_IINDPM_I_MIN_uA)
		uA = SGM4154X_IINDPM_I_MIN_uA;
	if (uA > SGM4154X_IINDPM_I_MAX_uA)
		uA = SGM4154X_IINDPM_I_MAX_uA;

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	reg_val = (uA - SGM4154X_IINDPM_I_MIN_uA) / SGM4154X_IINDPM_STEP_uA;
#else
	if (uA >= SGM4154X_IINDPM_I_MIN_uA && uA <= 3100000)//default
		reg_val = (uA - SGM4154X_IINDPM_I_MIN_uA) / SGM4154X_IINDPM_STEP_uA;
	else if (uA > 3100000 && uA < SGM4154X_IINDPM_I_MAX_uA)
		reg_val = 0x1E;
	else
		reg_val = SGM4154X_IINDPM_I_MASK;
#endif
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_0,
				SGM4154X_ICHRG_I_MASK, reg_val);
}

static int sgm4154x_disable_watchdog_timer(struct sgm4154x *sgm)
{
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_5,
			SGM4154X_WDT_TIMER_MASK, SGM4154X_WDT_TIMER_DISABLE);
}

static int sgm4154x_reset_watchdog_timer(struct sgm4154x *sgm)
{
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_1,
			SGM4154X_WDT_RST_MASK, SGM4154X_WDT_RST_MASK);
}

static int sgm4154x_enable_term(struct sgm4154x *sgm, bool enable)
{
	u8 val;

	if (enable)
		val = SGM4154X_TERM_EN;
	else
		val = SGM4154X_TERM_DISABLE;

	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_5,
			SGM4154X_TERM_EN, val);
}

static int sgm4154x_set_boost_current(struct sgm4154x *sgm, int uA)
{
	int ret = 0;

	if (uA == BOOST_CURRENT_LIMIT[0]) {
		ret = sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_2,
				SGM4154X_BOOST_LIM, 0);
	} else if (uA == BOOST_CURRENT_LIMIT[1]) {
		ret = sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_2,
				SGM4154X_BOOST_LIM, BIT(7));
	}
	return ret;
}

static int sgm4154x_set_boost_voltage(struct sgm4154x *sgm, int uV)
{
	int ret = 0;
	char reg_val = -1;
	int i = 0;

	while (i < 4) {
		if (uV == BOOST_VOLT_LIMIT[i]) {
			reg_val = i;
			break;
		}
		i++;
	}
	if (reg_val < 0)
		return reg_val;
	reg_val = reg_val << 4;
	ret = sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_6,
				  SGM4154X_BOOSTV, reg_val);

	return ret;
}

static int sgm4154x_set_acovp_threshold(struct sgm4154x *sgm, int uV)
{
	int ret = 0;
	char reg_val = -1;
	int i = 0;

	while (i < 4) {
		if (uV == VAC_OVP[i]) {
			reg_val = i;
			break;
		}
		i++;
	}
	if (reg_val < 0)
		return reg_val;
	reg_val = reg_val << 6;
	ret = sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_6,
				  SGM4154X_VAC_OVP_MASK, reg_val);

	return ret;
}

static int sgm4154x_set_stat_ctrl(struct sgm4154x *sgm, int ctrl)
{
	u8 val;

	val = ctrl;

	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_0,
			SGM4154X_STAT_CTRL_MASK, val << SGM4154X_STAT_CTRL_SHIFT);
}

static int sgm4154x_set_int_mask(struct sgm4154x *sgm, int mask)
{
	u8 val;

	val = mask;

	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_a,
			SGM4154X_INT_MASK_MASK, val);
}

static int sgm4154x_enable_safety_timer(struct sgm4154x *sgm)
{
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_5,
			SGM4154X_SAFETY_TIMER_EN, SGM4154X_SAFETY_TIMER_EN);
}

static int sgm4154x_disable_safety_timer(struct sgm4154x *sgm)
{
	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_5,
			SGM4154X_SAFETY_TIMER_EN, SGM4154X_SAFETY_TIMER_DISABLE);
}

static int sgm4154x_set_safety_timer_hours(struct sgm4154x *sgm, int hours)
{
	u8 val = SGM4154X_SAFETY_TIMER_10H;

	if (hours == 20)
		val = SGM4154X_SAFETY_TIMER_20H;

	return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_5,
			SGM4154X_SAFETY_TIMER_SET_MASK, val);
}

static void sgm4154x_parse_dt(struct device_node *np, struct sgm4154x *sgm)
{
	int ret;

	if (of_property_read_string(np, "charger_name", &sgm->chg_dev_name) < 0) {
		sgm->chg_dev_name = "primary_chg";
		pr_warn("no charger name, use default\n");
	}

	if (of_property_read_string(np, "eint_name", &sgm->eint_name) < 0) {
		sgm->eint_name = "chr_stat";
		pr_warn("no eint name, use default\n");
	}

	ret = of_property_read_u32(np, "sgm4154x,stat-pin-ctrl", &sgm->statctrl);
	if (ret) {
		sgm->statctrl = 0;
		pr_info("no sgm4154x,stat-pin-ctrl, use default value = %d\n", sgm->statctrl);
	}

	ret = of_property_read_u32(np, "sgm4154x,precharge-current",
				   &sgm->iprechg);
	if (ret) {
		sgm->iprechg = 180;
		pr_info("no sgm4154x,precharge-current, use default value = \n", sgm->iprechg);
	}

	ret = of_property_read_u32(np, "sgm4154x,termination-current",
				   &sgm->iterm);
	if (ret) {
		sgm->iterm = 240;
		pr_info("no sgm4154x,termination-current, use default value = %d\n", sgm->iterm);
	}

	ret = of_property_read_u32(np, "sgm4154x,boost-voltage", &sgm->boostv);
	if (ret) {
		sgm->boostv = 5000;
		pr_info("no sgm4154x,boost-voltage, use default value = %d\n", sgm->boostv);
	}

	ret = of_property_read_u32(np, "sgm4154x,boost-current", &sgm->boosti);
	if (ret) {
		sgm->boosti = 1200;
		pr_info("no sgm4154x,boost-current, use default value = %d\n", sgm->boosti);
	}

	ret = of_property_read_u32(np, "sgm4154x,vac-ovp-threshold", &sgm->vac_ovp);
	if (ret) {
		sgm->vac_ovp = 6500;
		pr_info("no sgm4154x,vac-ovp-threshold, use default value = %d\n", sgm->vac_ovp);
	}

	ret = of_property_read_u32(np, "sgm4154x,safety-hours", &sgm->safety_hours);
	if (ret) {
		sgm->safety_hours = 20;
		pr_info("no sgm4154x,safety-hours, use default value = %d\n", sgm->safety_hours);
	}
}

static int check_fault_register(struct sgm4154x *sgm)
{
	int ret;
	u8 fault_val = 0;

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_FAULT, &fault_val);
	if (ret) {
		pr_err("%s: read fault register failed\n", __func__);
		return -1;
	}

	/*
	 * Before the host reads REG09 and all the faults are cleared, the charger device would not send
	 * any INT upon new faults. To read the current fault status, the host has to read REG09 two times consecutively
	 */
	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_FAULT, &fault_val);
	if (ret) {
		pr_err("%s: read fault register failed\n", __func__);
		return -1;
	}

	if (fault_val == 0)
		return 0;

	pr_err("%s: read fault register is 0x%02X\n", __func__, fault_val);
	if (fault_val & SGM4154X_FAULT_NTC_MASK) {
		switch (fault_val & SGM4154X_FAULT_NTC_MASK) {
		case SGM4154X_FAULT_NTC_WARM:
			pr_err("%s: NTC warm fault\n", __func__);
			break;
		case SGM4154X_FAULT_NTC_COOL:
			pr_err("%s: NTC cool fault\n", __func__);
			break;
		case SGM4154X_FAULT_NTC_COLD:
			pr_err("%s: NTC cold fault\n", __func__);
			break;
		case SGM4154X_FAULT_NTC_HOT:
			pr_err("%s: NTC hot fault\n", __func__);
			break;
		default:
			break;
		}
	}

	if (fault_val & SGM4154X_FAULT_BAT_MASK) {
		pr_err("%s: vbat ovp\n", __func__);
		bat_metrics_chg_fault(METRICS_FAULT_VBAT_OVP);
	}

	if (fault_val & SGM4154X_FAULT_CHRG_MASK) {
		switch (fault_val & SGM4154X_FAULT_CHRG_MASK) {
		case SGM4154X_FAULT_CHRG_INPUT:
			pr_err("%s:Input fault(VAC OVP or Vbat<Vbus<3.8V)\n",
				__func__);
			break;
		case SGM4154X_FAULT_CHRG_THERMAL:
			pr_err("%s: Thermal shutdown\n", __func__);
			break;
		case SGM4154X_FAULT_CHRG_TIMER:
			pr_err("%s: Charge safety timer expired\n", __func__);
			bat_metrics_chg_fault(METRICS_FAULT_SAFETY_TIMEOUT);
		default:
			break;
		}
	}

	if (fault_val & SGM4154X_FAULT_BOOST_MASK) {
		pr_err("%s: VBUS overloaded in OTG, or VBUS OVP, or Vbat too low\n", __func__);
		bat_metrics_chg_fault(METRICS_FAULT_VBUS_OVP);
	}

	if (fault_val & SGM4154X_FAULT_WDT_MASK) {
		pr_err("%s: Watchdog timer expired\n", __func__);
	}

	return fault_val;
}

static irqreturn_t sgm4154x_irq_handler(int irq, void *data)
{
	int ret;
	u8 reg_val;
	bool prev_pg;
	struct sgm4154x *sgm = data;

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_STAT, &reg_val);
	if (ret)
		return IRQ_HANDLED;

	charger_dev_notify(sgm->chg_dev, CHARGER_DEV_NOTIFY_VBUS_EVENT);

	prev_pg = sgm->power_good;
	sgm->power_good = !!(reg_val & SGM4154X_PG_STAT);

	if (!prev_pg && sgm->power_good)
		pr_notice("adapter/usb inserted\n");
	else if (prev_pg && !sgm->power_good)
		pr_notice("adapter/usb removed\n");


	ret = check_fault_register(sgm);
	if (ret < 0)
		pr_err("%s: Fail to check fault register\n", __func__);

	pr_info("[%s] sgm4154x_irq_handler ok\n", __func__);

	return IRQ_HANDLED;
}

static int sgm4154x_register_interrupt(struct sgm4154x *sgm)
{
	int ret = 0;

	if (!sgm->client->irq) {
		pr_info("sgm4154x sgm->client->irq is null\n");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(sgm->dev, sgm->client->irq, NULL,
					sgm4154x_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					sgm->eint_name, sgm);
	if (ret < 0) {
		pr_err("request thread irq failed:%d\n", ret);
		return ret;
	}

	enable_irq_wake(sgm->irq);
	pr_info("sgm4154x_register_interrupt ok\n");

	return 0;
}

static int sgm4154x_init_device(struct sgm4154x *sgm)
{
	int ret;

	pr_info("[%s] open\n", __func__);

	sgm4154x_disable_watchdog_timer(sgm);
	ret = sgm4154x_set_safety_timer_hours(sgm, sgm->safety_hours);
	if (ret) {
		pr_err("Failed to set safety timer hours, ret = %d\n", ret);
		goto out_init_device;
	}

	ret = sgm4154x_set_stat_ctrl(sgm, sgm->statctrl);
	if (ret) {
		pr_err("Failed to set stat pin control mode, ret = %d\n", ret);
		goto out_init_device;
	}
	ret = sgm4154x_set_prechg_current(sgm, sgm->iprechg);
	if (ret) {
		pr_err("Failed to set prechg current, ret = %d\n", ret);
		goto out_init_device;
	}

	ret = sgm4154x_enable_term(sgm, true);
	if (ret) {
		pr_err("Failed to enable termination, ret = %d\n", ret);
		goto out_init_device;
	}

	ret = sgm4154x_set_term_current(sgm, sgm->iterm);
	if (ret) {
		pr_err("Failed to set termination current, ret = %d\n", ret);
		goto out_init_device;
	}

	ret = sgm4154x_set_boost_voltage(sgm, sgm->boostv);
	if (ret) {
		pr_err("Failed to set boost voltage, ret = %d\n", ret);
		goto out_init_device;
	}

	ret = sgm4154x_set_boost_current(sgm, sgm->boosti);
	if (ret) {
		pr_err("Failed to set boost current, ret = %d\n", ret);
		goto out_init_device;
	}

	ret = sgm4154x_set_acovp_threshold(sgm, sgm->vac_ovp);
	if (ret) {
		pr_err("Failed to set acovp threshold, ret = %d\n", ret);
		goto out_init_device;
	}

	ret = sgm4154x_set_int_mask(sgm,
			SGM4154X_IINDPM_INT_MASK | SGM4154X_VINDPM_INT_MASK);
	if (ret) {
		pr_err("Failed to set vindpm and iindpm int mask\n");
		goto out_init_device;
	}

	pr_info("[%s] OK\n", __func__);

out_init_device:
	return ret;
}

static void determine_initial_status(struct sgm4154x *sgm)
{
	sgm4154x_irq_handler(sgm->irq, (void *)sgm);
}

static int sgm4154x_hw_chipid_detect(struct sgm4154x *sgm)
{
	int ret = 0;
	u8 val = 0;

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_CTRL_b, &val);
	if (ret < 0) {
		pr_info("[%s] read SGM4154X_CHRG_CTRL_b fail\n", __func__);
		return ret;
	}
	val = val & SGM4154X_PN_MASK;
	pr_info("%s: 0x0B=0x%02X\n", __func__, val);

	return val;
}

static void sgm4154x_dump_regs(struct sgm4154x *sgm)
{
	int addr, ret, len, idx = 0;
	u8 val, tmpbuf[16];
	char buf[128] = {0};

	for (addr = SGM4154X_CHRG_CTRL_0; addr <= SGM4154X_CHRG_CTRL_f; addr++) {
		ret = sgm4154x_read_byte(sgm, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, 10, "%.1x:%.2x ", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}
	pr_info("%s\n", buf);
}

static int sgm4154x_get_vendor(struct charger_device *chg_dev, char *buf, int length)
{
	return snprintf(buf, length, "%s", "SGM_SG41513\n");
}

static int sgm4154x_get_registers(struct charger_device *chg_dev, char *buf, int length)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int addr, ret, len, idx = 0;
	char tmpbuf[80] = {0};
	u8 val;

	for (addr = SGM4154X_CHRG_CTRL_0; addr <= SGM4154X_CHRG_CTRL_a; addr++) {
		ret = sgm4154x_read_byte(sgm, addr, &val);
		if (ret == 0) {
			len = snprintf(&tmpbuf[idx], length - idx, "%.1x:%.2x ", addr, val);
			idx += len;
		}
	}

	return snprintf(buf, length, "%s", tmpbuf);
}

static ssize_t sgm4154x_show_registers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sgm4154x *sgm = dev_get_drvdata(dev);
	int addr, ret, len, idx = 0;
	u8 val, tmpbuf[16];

	for (addr = SGM4154X_CHRG_CTRL_0; addr <= SGM4154X_CHRG_CTRL_f; addr++) {
		ret = sgm4154x_read_byte(sgm, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, 10, "%.1x:%.2x ", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t sgm4154x_show_vendor(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s", "SGM_SG41513\n");
}

static ssize_t sgm4154x_store_registers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sgm4154x *sgm = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < SGM4154X_REG_NUM)
		sgm4154x_write_byte(sgm, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, sgm4154x_show_registers,
		   sgm4154x_store_registers);

static DEVICE_ATTR(vendor, S_IRUGO, sgm4154x_show_vendor, NULL);

static struct attribute *sgm4154x_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_vendor.attr,
	NULL,
};

static const struct attribute_group sgm4154x_attr_group = {
	.attrs = sgm4154x_attributes,
};

static int sgm4154x_charging(struct charger_device *chg_dev, bool enable)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;

	if (enable)
		ret = sgm4154x_enable_charger(sgm);
	else
		ret = sgm4154x_disable_charger(sgm);

	pr_err("%s charger %s\n", enable ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_CTRL_1, &val);
	if (!ret)
		sgm->charge_enabled = !!(val & SGM4154X_CHRG_EN);

	return ret;
}

static int sgm4154x_plug_in(struct charger_device *chg_dev)
{
	int ret;

	ret = sgm4154x_charging(chg_dev, true);
	if (ret)
		pr_err("Failed to enable charging:%d\n", ret);

	return ret;
}

static int sgm4154x_plug_out(struct charger_device *chg_dev)
{
	int ret;

	ret = sgm4154x_charging(chg_dev, false);
	if (ret)
		pr_err("Failed to disable charging:%d\n", ret);

	return ret;
}

static int sgm4154x_dump_register(struct charger_device *chg_dev)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);

	sgm4154x_dump_regs(sgm);

	return 0;
}

static int sgm4154x_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);

	*en = sgm->charge_enabled;

	return 0;
}

static int sgm4154x_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_STAT, &val);
	if (!ret) {
		val = val & SGM4154X_CHG_STAT_MASK;
		*done = (val == SGM4154X_TERM_CHRG);
	}

	return ret;
}

static int sgm4154x_enable_power_path(struct charger_device *chg_dev, bool en)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);

	if (en) {
		return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_0,
				SGM4154X_HIZ_EN, 0);
	} else {
		return sgm4154x_update_bits(sgm, SGM4154X_CHRG_CTRL_0,
			SGM4154X_HIZ_EN, SGM4154X_HIZ_EN);
	}
}

static int sgm4154x_is_power_path_enable(struct charger_device *chg_dev, bool *en)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_CTRL_0, &val);
	if (!ret) {
		val = val & SGM4154X_HIZ_EN;
		*en = !(val == SGM4154X_HIZ_EN);
	}

	return ret;
}

static int sgm4154x_set_ichg(struct charger_device *chg_dev, u32 uA)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);

	pr_err("charge curr = %d\n", uA);
	return sgm4154x_set_chargecurrent(sgm, uA);
}

static int sgm4154x_get_ichg(struct charger_device *chg_dev, u32 *chg_uA)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 ichg = 0;
	int curr;

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_CTRL_2, &ichg);
	if (ret)
		return ret;

	ichg &= SGM4154X_ICHRG_I_MASK;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	if (ichg <= 0x8)
		curr = ichg * 5000;
	else if (ichg <= 0xF)
		curr = 40000 + (ichg - 0x8) * 10000;
	else if (ichg <= 0x17)
		curr = 110000 + (ichg - 0xF) * 20000;
	else if (ichg <= 0x20)
		curr = 270000 + (ichg - 0x17) * 30000;
	else if (ichg <= 0x30)
		curr = 540000 + (ichg - 0x20) * 60000;
	else if (ichg <= 0x3C)
		curr = 1500000 + (ichg - 0x30) * 120000;
	else
		curr = 3000000;
#else
	curr = ichg * SGM4154X_ICHRG_I_STEP_uA;
#endif
	*chg_uA = curr;

	return ret;
}

static int sgm4154x_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	*curr = SGM4154X_ICHRG_I_MIN_uA;

	return 0;
}

static int sgm4154x_set_vchg(struct charger_device *chg_dev, u32 volt)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);

	pr_err("charge volt = %d\n", volt);

	return sgm4154x_set_chargevolt(sgm, volt);
}

static int sgm4154x_get_vchg(struct charger_device *chg_dev, u32 *volt)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 vreg_val;

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_CTRL_4, &vreg_val);
	if (ret)
		return ret;

	vreg_val = (vreg_val & SGM4154X_VREG_V_MASK) >> 3;
	if (15 == vreg_val)
		*volt = 4352000; //default
	else if (vreg_val < 25)
		*volt = vreg_val * SGM4154X_VREG_V_STEP_uV
			+ SGM4154X_VREG_V_MIN_uV;

	return 0;
}

static int sgm4154x_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);

	pr_err("vindpm volt = %d\n", volt);

	return sgm4154x_set_input_volt_limit(sgm, volt);
}

static int sgm4154x_get_ivl(struct charger_device *chg_dev, u32 *uV)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int ret;
	int offset;
	u8 vlim, os_reg;

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_CTRL_6, &vlim);
	if (ret)
		return ret;
	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_CTRL_f, &os_reg);
	if (ret)
		return ret;

	os_reg = os_reg & SGM4154X_VINDPM_OS_MASK;
	if (0 == os_reg)
		offset = 3900000;
	else if (1 == os_reg)
		offset = 5900000;
	else if (2 == os_reg)
		offset = 7500000;
	else if (3 == os_reg)
		offset = 10500000;

	*uV = offset + (vlim & 0x0F) * SGM4154X_VINDPM_STEP_uV;

	return 0;
}

static int sgm4154x_get_indpm_state(struct charger_device *chg_dev,
				bool *vdpm, bool *idpm)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	u8 val;
	int ret;

	*vdpm = false;
	*idpm = false;
	if (sgm->charge_enabled) {
		ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_CTRL_a, &val);
		if (ret)
			return ret;
		*vdpm = !!(val & SGM4154X_VINDPM_STAT);
		*idpm = !!(val & SGM4154X_IINDPM_STAT);
	}

	return 0;
}

static int sgm4154x_set_icl(struct charger_device *chg_dev, u32 curr)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);

	pr_err("indpm curr = %d\n", curr);

	return sgm4154x_set_input_current_limit(sgm, curr);
}

static int sgm4154x_get_icl(struct charger_device *chg_dev, u32 *ilim)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_CTRL_0, &reg_val);
	if (ret)
		return ret;
	if (SGM4154X_IINDPM_I_MASK == (reg_val & SGM4154X_IINDPM_I_MASK))
		*ilim =  SGM4154X_IINDPM_I_MAX_uA;
	else
		*ilim = (reg_val & SGM4154X_IINDPM_I_MASK) * SGM4154X_IINDPM_STEP_uA
			+ SGM4154X_IINDPM_I_MIN_uA;

	return 0;
}

static int sgm4154x_kick_wdt(struct charger_device *chg_dev)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);

	return sgm4154x_reset_watchdog_timer(sgm);
}

static int sgm4154x_set_otg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);

	if (en)
		ret = sgm4154x_enable_otg(sgm);
	else
		ret = sgm4154x_disable_otg(sgm);

	pr_err("%s OTG %s\n", en ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	return ret;
}

static int sgm4154x_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int ret;

	if (en)
		ret = sgm4154x_enable_safety_timer(sgm);
	else
		ret = sgm4154x_disable_safety_timer(sgm);

	return ret;
}

static int sgm4154x_is_safety_timer_enabled(struct charger_device *chg_dev,
					   bool *en)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = sgm4154x_read_byte(sgm, SGM4154X_CHRG_CTRL_5, &reg_val);

	if (!ret)
		*en = !!(reg_val & SGM4154X_SAFETY_TIMER_MASK);

	return ret;
}

static int sgm4154x_set_boost_ilmt(struct charger_device *chg_dev, u32 curr)
{
	struct sgm4154x *sgm = dev_get_drvdata(&chg_dev->dev);
	int ret;

	pr_err("otg curr = %d\n", curr);

	ret = sgm4154x_set_boost_current(sgm, curr);

	return ret;
}

static int sgm4154x_do_event(struct charger_device *chg_dev, u32 event, u32 args)
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

static struct charger_ops sgm4154x_chg_ops = {
	/* Normal charging */
	.plug_in = sgm4154x_plug_in,
	.plug_out = sgm4154x_plug_out,
	.dump_registers = sgm4154x_dump_register,
	.enable = sgm4154x_charging,
	.is_enabled = sgm4154x_is_charging_enable,
	.get_charging_current = sgm4154x_get_ichg,
	.set_charging_current = sgm4154x_set_ichg,
	.get_input_current = sgm4154x_get_icl,
	.set_input_current = sgm4154x_set_icl,
	.get_constant_voltage = sgm4154x_get_vchg,
	.set_constant_voltage = sgm4154x_set_vchg,
	.kick_wdt = sgm4154x_kick_wdt,
	.set_mivr = sgm4154x_set_ivl,
	.get_mivr = sgm4154x_get_ivl,
	.get_indpm_state = sgm4154x_get_indpm_state,
	.is_charging_done = sgm4154x_is_charging_done,
	.get_min_charging_current = sgm4154x_get_min_ichg,

	/* Safety timer */
	.enable_safety_timer = sgm4154x_set_safety_timer,
	.is_safety_timer_enabled = sgm4154x_is_safety_timer_enabled,
	.enable_chg_type_det = NULL,
	/* Power path */
	.enable_powerpath = sgm4154x_enable_power_path,
	.is_powerpath_enabled = sgm4154x_is_power_path_enable,

	/* OTG */
	.enable_otg = sgm4154x_set_otg,
	.set_boost_current_limit = sgm4154x_set_boost_ilmt,

	/* Event */
	.event = sgm4154x_do_event,

	.enable_discharge = NULL,

	/* PE+/PE+20 */
	.send_ta_current_pattern = NULL,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
	.enable_cable_drop_comp = NULL,

	/* ADC */
	.get_tchg_adc = NULL,

	/*vendor&reg*/
	.get_vendor = sgm4154x_get_vendor,
	.get_reg = sgm4154x_get_registers,
};

static struct of_device_id sgm4154x_match_table[] = {
	{ .compatible = "sgm,sgm4154x" },
	{},
};
MODULE_DEVICE_TABLE(of, sgm4154x_match_table);

static void sgm4154x_shutdown(void)
{
	int ret = 0;

	devm_free_irq(g_sgm->dev, g_sgm->client->irq, g_sgm);
	ret = sgm4154x_disable_otg(g_sgm);
	if (ret) {
		pr_err("Failed to sgm4154x disable otg\n", ret);
	}
	mutex_destroy(&g_sgm->i2c_rw_lock);
	pr_info("[%s] OK\n", __func__);
}

static struct syscore_ops sgm4154x_syscore_ops = {
	.shutdown = sgm4154x_shutdown,
};

static int sgm4154x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sgm4154x *sgm;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;

	int ret = 0;

	sgm = devm_kzalloc(&client->dev, sizeof(struct sgm4154x), GFP_KERNEL);
	if (!sgm)
		return -ENOMEM;

	sgm->dev = &client->dev;
	sgm->client = client;
	i2c_set_clientdata(client, sgm);

	mutex_init(&sgm->i2c_rw_lock);

	ret = sgm4154x_hw_chipid_detect(sgm);
	if (ret != SGM4154X_PN_ID) {
		pr_info("%s: No sgm4154x device not found !!!\n", __func__);
		return -ENODEV;
	}
	pr_notice("sgm4154x device found!\n");

	match = of_match_node(sgm4154x_match_table, node);
	if (match == NULL) {
		pr_err("%s: device tree match not found\n", __func__);
		return -EINVAL;
	}
	sgm4154x_parse_dt(node, sgm);

	ret = sgm4154x_init_device(sgm);
	if (ret) {
		pr_err("%s: Failed to init device\n", __func__);
		return ret;
	}

	sgm->chg_dev = charger_device_register(sgm->chg_dev_name,
					      &client->dev, sgm,
					      &sgm4154x_chg_ops,
					      &sgm4154x_chg_props);
	if (IS_ERR_OR_NULL(sgm->chg_dev)) {
		ret = PTR_ERR(sgm->chg_dev);
		pr_err("%s:charger_device_register failed (%d)\n",
			__func__, ret);
		return ret;
	}
	pr_info("%s: charger_device_register OK(%d)\n", __func__, ret);

	ret = sysfs_create_group(&sgm->dev->kobj, &sgm4154x_attr_group);
	if (ret) {
		dev_err(sgm->dev, "%s: failed to register sysfs. err: %d\n",
			__func__, ret);
			goto out;
	}

	register_syscore_ops(&sgm4154x_syscore_ops);
	g_sgm = sgm;
	sgm4154x_register_interrupt(g_sgm);
	determine_initial_status(g_sgm);

	pr_info("%s: sgm4154x probe successfully\n!", __func__);

	return 0;
out:
	mutex_destroy(&g_sgm->i2c_rw_lock);
	charger_device_unregister(sgm->chg_dev);
	return ret;
}

static int sgm4154x_remove(struct i2c_client *client)
{
	struct sgm4154x *sgm = i2c_get_clientdata(client);

	charger_device_unregister(sgm->chg_dev);
	sysfs_remove_group(&sgm->dev->kobj, &sgm4154x_attr_group);

	return 0;
}

static struct i2c_driver sgm4154x_driver = {
	.driver = {
		   .name = "sgm4154x",
		   .owner = THIS_MODULE,
		   .of_match_table = sgm4154x_match_table,
	},

	.probe = sgm4154x_probe,
	.remove = sgm4154x_remove,
};

module_i2c_driver(sgm4154x_driver);

MODULE_DESCRIPTION("SG Micro SGM41513 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Amazon");
