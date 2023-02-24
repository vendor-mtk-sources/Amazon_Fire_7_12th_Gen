// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#endif
#endif
#include "lcm_drv.h"

#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#endif

#if defined(CONFIG_AMZN_METRICS_LOG) || defined(CONFIG_AMZN_MINERVA_METRICS_LOG)
#include <linux/amzn_metricslog.h>
#endif

/* pcba info */
#define ONLY_4POWER_PCBA		1
#define ONLY_3POWER_PCBA		2
#define BOTH_MODE_PCBA			3

/*
* Before HVT, Lcm power mode is contrled by ldo gpio.
* Since HVT, Lcm power mode is contrled by pmic vcn33.
* 0    #ldo gpio
* 1    #pmic vcn33
*/
#define LCM_POWER_PMIC_VCN33		1

#define OTA7290X_4POWER_UNKNOWN		0xFF

static struct lcm_platform_data {
	struct pinctrl *lcmctrl;
	unsigned char lcm_id;
	unsigned char pcba_mode;
	unsigned int lcm_power_mode;
	int lcm_id_voltage;
	struct regulator *vcn33_ldo;
	struct pinctrl_state *lcm_pins_rst0;
	struct pinctrl_state *lcm_pins_rst1;
	struct pinctrl_state *lcm_pins_bias_en0;
	struct pinctrl_state *lcm_pins_bias_en1;
	struct pinctrl_state *lcm_pins_pwr_en0;
	struct pinctrl_state *lcm_pins_pwr_en1;
} lcm_data;

static int __init setup_lcm_id(char *str)
{
	int id;

	if (get_option(&str, &id)) {
		lcm_data.lcm_id = (unsigned char)id;
		pr_notice("[LCM] %s: get lcm_id from cmdline, lcm_id = %d.\n",
		__func__, lcm_data.lcm_id);
	} else {
		pr_notice("[LCM] %s: get lcm_id failed, lcm_id = %d.\n",
		__func__, lcm_data.lcm_id);
	}

	return 0;
}
__setup("LCM_VENDOR_ID=", setup_lcm_id);

static int __init setup_pcba_mode(char *str)
{
	int mode;

	lcm_data.pcba_mode = 0;

	if (get_option(&str, &mode)) {
		lcm_data.pcba_mode = (unsigned char)mode;
		pr_notice("[LCM] %s: get pcba mode from cmdline, pcba_mode = %d.\n",
		__func__, lcm_data.pcba_mode);
	} else {
		pr_notice("[LCM] %s: get pcba mode failed, pcba_mode = %d.\n",
		__func__, lcm_data.pcba_mode);
	}

	return 0;
}
__setup("LCM_PCBA_POWER_MODE=", setup_pcba_mode);

static int __init setup_lcm_id_voltage(char *str)
{
	int voltage;

	lcm_data.lcm_id_voltage = 0;

	if (get_option(&str, &voltage)) {
		lcm_data.lcm_id_voltage = voltage;
		pr_notice("[LCM] %s: get lcm voltage from cmdline, lcm_id_voltage = %d.\n",
		__func__, lcm_data.lcm_id_voltage);
	} else {
		pr_notice("[LCM] %s: get lcm voltage failed, lcm_id_voltage = %d.\n",
		__func__, lcm_data.lcm_id_voltage);
	}

	return 0;
}
__setup("LCM_ID_VOLTAGE=", setup_lcm_id_voltage);

static ssize_t get_lcm_vendor_id(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", lcm_data.lcm_id);
}
static DEVICE_ATTR(vendor_id, 0444, get_lcm_vendor_id, NULL);

static ssize_t get_pcba_power_mode(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	size_t data_len = 0;

	if (lcm_data.pcba_mode == ONLY_4POWER_PCBA)
		data_len = scnprintf(buf, PAGE_SIZE, "Only_4Power_Mode_PCBA\n");
	else if (lcm_data.pcba_mode == ONLY_3POWER_PCBA)
		data_len = scnprintf(buf, PAGE_SIZE, "Only_3Power_Mode_PCBA\n");
	else if (lcm_data.pcba_mode == BOTH_MODE_PCBA)
		data_len = scnprintf(buf, PAGE_SIZE, "Both_3&4Power_PCBA\n");
	else
		data_len = scnprintf(buf, PAGE_SIZE, "Unknow_Mode_PCBA\n");
	return data_len;
}
static DEVICE_ATTR(pcba_power_mode, 0444, get_pcba_power_mode, NULL);

static ssize_t get_lcm_id_voltage(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", lcm_data.lcm_id_voltage);
}
static DEVICE_ATTR(lcm_id_voltage, 0444, get_lcm_id_voltage, NULL);

static struct attribute *lcm_attrs[] = {
	&dev_attr_vendor_id.attr,
	&dev_attr_pcba_power_mode.attr,
	&dev_attr_lcm_id_voltage.attr,
	NULL
};

static const struct attribute_group lcm_attr_group = {
	.attrs = lcm_attrs,
};

static int lcm_driver_probe(struct device *dev)
{
	int ret = 0;
	struct kobject *lcm_kobj = &dev->kobj;

	pr_notice("[LCM][Ota7290x] %s Default transmission mode: mipi\n",
					__func__);

	lcm_data.lcmctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lcm_data.lcmctrl)) {
		dev_err(dev, "[LCM]Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcm_data.lcmctrl);
		return ret;
	}

	/*lcm power pin lookup */
	lcm_data.lcm_pins_rst0 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_rst0");
	if (IS_ERR(lcm_data.lcm_pins_rst0)) {
		ret = PTR_ERR(lcm_data.lcm_pins_rst0);
		pr_err("[LCM]pinctrl err, lcm_pins_rst0\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_rst1 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_rst1");
	if (IS_ERR(lcm_data.lcm_pins_rst1)) {
		ret = PTR_ERR(lcm_data.lcm_pins_rst1);
		pr_err("[LCM]pinctrl err, lcm_pins_rst1\n");
		goto lcm_pinctrl_free;
	}

	lcm_data.lcm_pins_bias_en0 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_bias_en0");
	if (IS_ERR(lcm_data.lcm_pins_bias_en0)) {
		ret = PTR_ERR(lcm_data.lcm_pins_bias_en0);
		pr_err("[LCM]pinctrl err, lcm_pins_bias_en0\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_bias_en1 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_bias_en1");
	if (IS_ERR(lcm_data.lcm_pins_bias_en1)) {
		ret = PTR_ERR(lcm_data.lcm_pins_bias_en1);
		pr_err("[LCM]pinctrl err, lcm_pins_bias_en1\n");
		goto lcm_pinctrl_free;
	}

	if (lcm_data.lcm_power_mode == LCM_POWER_PMIC_VCN33) {
		pr_notice("[LCM]: lcm_power_mode is pmic vcn33.\n");
		lcm_data.vcn33_ldo = devm_regulator_get(dev, "reg-lcm");
		if (IS_ERR(lcm_data.vcn33_ldo)) {
			ret = PTR_ERR(lcm_data.vcn33_ldo);
			dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
			goto lcm_pinctrl_free;
		}

		ret = regulator_enable(lcm_data.vcn33_ldo);
		if (ret < 0) {
			dev_err(dev, "failed to enable vcn33_ldo\n");
			goto lcm_power_free;
		}
	} else {
		pr_notice("[LCM]: lcm_power_mode is ldo gpio.\n");
		lcm_data.lcm_pins_pwr_en0 = pinctrl_lookup_state(lcm_data.lcmctrl,
								"lcm_pins_pwr_en0");
		if (IS_ERR(lcm_data.lcm_pins_pwr_en0)) {
			ret = PTR_ERR(lcm_data.lcm_pins_pwr_en0);
			pr_err("[LCM]pinctrl err, lcm_pins_pwr_en0\n");
			goto lcm_pinctrl_free;
		}
		lcm_data.lcm_pins_pwr_en1 = pinctrl_lookup_state(lcm_data.lcmctrl,
								"lcm_pins_pwr_en1");
		if (IS_ERR(lcm_data.lcm_pins_pwr_en1)) {
			ret = PTR_ERR(lcm_data.lcm_pins_pwr_en1);
			pr_err("[LCM]pinctrl err, lcm_pins_pwr_en1\n");
			goto lcm_pinctrl_free;
		}
	}

	ret = sysfs_create_group(&dev->kobj, &lcm_attr_group);
	if (ret < 0) {
		pr_err("[LCM]Failure create sysfs group.\n");
		if (lcm_data.lcm_power_mode == LCM_POWER_PMIC_VCN33)
			goto lcm_power_enalbe_free;
		else
			goto lcm_pinctrl_free;
	}

	lcm_kobj = kobject_create_and_add("device", NULL);
	if (lcm_kobj != NULL) {
		ret = sysfs_create_link(lcm_kobj,
				&dev->kobj,
				"lcm");
		if (ret < 0) {
			kobject_put(lcm_kobj);
			pr_err("[LCM]Failed to create sysfs link for lcm.\n");
			if (lcm_data.lcm_power_mode == LCM_POWER_PMIC_VCN33)
				goto lcm_power_enalbe_free;
			else
				goto lcm_pinctrl_free;
		}
	}

	return ret;

lcm_power_enalbe_free:
	regulator_disable(lcm_data.vcn33_ldo);

lcm_power_free:
	regulator_put(lcm_data.vcn33_ldo);
	lcm_data.vcn33_ldo = NULL;

lcm_pinctrl_free:
	devm_pinctrl_put(lcm_data.lcmctrl);
	lcm_data.lcmctrl = NULL;

	pr_err("[LCM] %s get lcm dev res failed.\n", __func__);
	return ret;
}

static void lcm_set_rst(int val)
{
	if (!lcm_data.lcmctrl) {
		pr_err("[LCM]cannot find lcm pinctrl.\n");
		return;
	}

	if (val == 0)
		pinctrl_select_state(lcm_data.lcmctrl,
					lcm_data.lcm_pins_rst0);
	else
		pinctrl_select_state(lcm_data.lcmctrl,
					lcm_data.lcm_pins_rst1);
}

static void lcm_set_bias_en(int val)
{
	if (!lcm_data.lcmctrl) {
		pr_err("[LCM]cannot find lcm pinctrl.\n");
		return;
	}

	if (val == 0)
		pinctrl_select_state(lcm_data.lcmctrl,
					lcm_data.lcm_pins_bias_en0);
	else
		pinctrl_select_state(lcm_data.lcmctrl,
					lcm_data.lcm_pins_bias_en1);
}

static void lcm_set_pwr_en(int val)
{
	if (!lcm_data.lcmctrl) {
		pr_err("[LCM]cannot find lcm pinctrl.\n");
		return;
	}

	if (val == 0)
		pinctrl_select_state(lcm_data.lcmctrl,
					lcm_data.lcm_pins_pwr_en0);
	else
		pinctrl_select_state(lcm_data.lcmctrl,
					lcm_data.lcm_pins_pwr_en1);
}

static int lcm_platform_remove(struct platform_device *pdev)
{
	/* free gpio */
	if (lcm_data.lcmctrl != NULL) {
		devm_pinctrl_put(lcm_data.lcmctrl);
		lcm_data.lcmctrl = NULL;
	}
	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{.compatible = "ota,ota7290b_4power_hsd_kd",},
	{.compatible = "ota,ota7290n_4power_hsd_kd",},
	{.compatible = "ota,ota7290n_4power_inx_kd",},
	{},
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	unsigned int panel_id = OTA7290X_4POWER_UNKNOWN;
	int ret = 0;

	ret = of_property_read_u32(np, "vendor_id", &panel_id);
	if (ret < 0) {
		pr_notice("[LCM]:get dts vendor_id failed\n");
		return -ENODEV;
	}

	if (panel_id != lcm_data.lcm_id) {
		pr_notice("[LCM][%s] panel_id:%d != lcm_id:%d\n",
			__func__, panel_id, lcm_data.lcm_id);
		return -ENODEV;
	}
	pr_notice("[LCM][%s] panel_id = %d, lcm_id = %d, pcab_mode =%d\n",
			__func__, panel_id,
			lcm_data.lcm_id, lcm_data.pcba_mode);

	ret = of_property_read_u32(np, "lcm_power_mode_sel", &lcm_data.lcm_power_mode);
	if (ret < 0) {
		pr_notice("[LCM]:get dts lcm_power_mode failed.\n");
		return -ENODEV;
	}

	pr_notice("[LCM]: lcm_power_mode = %d.\n", lcm_data.lcm_power_mode);

	return lcm_driver_probe(&pdev->dev);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		   .name = "ota7290x",
		   .owner = THIS_MODULE,
		   .of_match_table = lcm_platform_of_match,
	},
	.remove = lcm_platform_remove,
};

static int __init lcm_drv_init(void)
{
	pr_notice("[LCM][Ota7290x]: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_notice("[LCM][Ota7290x]: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_drv_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("[LCM][Ota7290x]: Unregister lcm driver done\n");
}

late_initcall(lcm_drv_init);
module_exit(lcm_drv_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

/* ----------------------------------------------------------------- */
/* Local Constants */
/* ----------------------------------------------------------------- */

#define FRAME_WIDTH			(600)
#define FRAME_HEIGHT			(1024)
#define GPIO_OUT_HIGH			1
#define GPIO_OUT_LOW			0
#define TRUE				1
#define FALSE				0

#define OTA7290N_4POWER_KD_HSD		16
#define OTA7290B_4POWER_KD_HSD		17
#define OTA7290N_4POWER_KD_INX		18

#define REGFLAG_DELAY			0xFC
#define REGFLAG_END_OF_TABLE		0xFD

/* ----------------------------------------------------------------- */
/*  Local Variables */
/* ----------------------------------------------------------------- */

static struct LCM_UTIL_FUNCS lcm_util = {
	.set_reset_pin = NULL,
	.set_gpio_out = NULL,
	.udelay = NULL,
	.mdelay = NULL,
};


#define SET_RESET_PIN(v)		(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)		(lcm_util.set_gpio_out((n), (v)))
#define UDELAY(n)			(lcm_util.udelay(n))
#define MDELAY(n)			(lcm_util.mdelay(n))

/* ----------------------------------------------------------------- */
/* Local Functions */
/* ----------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		 (lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update))
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		 (lcm_util.dsi_set_cmdq(pdata, queue_size, force_update))
#define wrtie_cmd(cmd) \
		 (lcm_util.dsi_write_cmd(cmd))
#define write_regs(addr, pdata, byte_nums) \
		 (lcm_util.dsi_write_regs(addr, pdata, byte_nums))
#define read_reg \
		 (lcm_util.dsi_read_reg())
#define read_reg_v2(cmd, buffer, buffer_size) \
		 (lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size))

/* ----------------------------------------------------------------- */
/* LCM Driver Implementations */
/* ----------------------------------------------------------------- */
struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static void push_table(struct LCM_setting_table *table,
			unsigned int count,
			unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned int cmd;

		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static struct LCM_setting_table lcm_init_kd_hsd_ota7290b[] = {
	{0x11,	0,	{ } },
	{REGFLAG_DELAY, 120, { } },
	{0x29,  0,      { } },
	{REGFLAG_DELAY, 20, {} },
	{REGFLAG_END_OF_TABLE, 0x00, { } },
};

static struct LCM_setting_table lcm_init_kd_hsd_ota7290n[] = {
	{0x11,	0,	{ } },
	{REGFLAG_DELAY, 120, { } },
	{0x29,  0,      { } },
	{REGFLAG_DELAY, 20, {} },
	{REGFLAG_END_OF_TABLE, 0x00, { } },
};

static struct LCM_setting_table lcm_init_kd_inx_ota7290n[] = {
	{0x11,	0,	{ } },
	{REGFLAG_DELAY, 120, { } },
	{0x29,  0,      { } },
	{REGFLAG_DELAY, 20, {} },
	{REGFLAG_END_OF_TABLE, 0x00, { } },
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,  0, { } },
	{REGFLAG_DELAY, 20, { } },
	{0x10, 0, { } },
	{REGFLAG_DELAY, 120, { } },
	{REGFLAG_END_OF_TABLE, 0x00, { } },
};

static void lcm_get_4power_ota7290x_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode   = BURST_VDO_MODE;

	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;

	params->dsi.intermediat_buffer_num = 0;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	//params->dsi.word_count=FRAME_WIDTH*3;
	if (lcm_data.lcm_id == OTA7290B_4POWER_KD_HSD) {
		params->dsi.vertical_sync_active	= 1;
		params->dsi.vertical_backporch		= 25;
		params->dsi.vertical_frontporch		= 35;
		params->dsi.vertical_active_line	= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active	= 1;
		params->dsi.horizontal_backporch	= 60;
		params->dsi.horizontal_frontporch	= 76;
		params->dsi.horizontal_active_pixel	= FRAME_WIDTH;
	} else if (lcm_data.lcm_id == OTA7290N_4POWER_KD_HSD) {
		params->dsi.vertical_sync_active	= 1;
		params->dsi.vertical_backporch		= 25;
		params->dsi.vertical_frontporch		= 35;
		params->dsi.vertical_active_line	= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active	= 1;
		params->dsi.horizontal_backporch	= 60;
		params->dsi.horizontal_frontporch	= 76;
		params->dsi.horizontal_active_pixel	= FRAME_WIDTH;
	} else if (lcm_data.lcm_id == OTA7290N_4POWER_KD_INX) {
		params->dsi.vertical_sync_active	= 1;
		params->dsi.vertical_backporch		= 25;
		params->dsi.vertical_frontporch		= 35;
		params->dsi.vertical_active_line	= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active	= 1;
		params->dsi.horizontal_backporch	= 60;
		params->dsi.horizontal_frontporch	= 76;
		params->dsi.horizontal_active_pixel	= FRAME_WIDTH;
	} else {
		params->dsi.vertical_sync_active	= 1;
		params->dsi.vertical_backporch		= 25;
		params->dsi.vertical_frontporch		= 35;
		params->dsi.vertical_active_line	= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active	= 1;
		params->dsi.horizontal_backporch	= 60;
		params->dsi.horizontal_frontporch	= 76;
		params->dsi.horizontal_active_pixel	= FRAME_WIDTH;
	}

	params->dsi.PLL_CLOCK = 149;
	params->dsi.ssc_disable = 1;		//1:disable ssc, 0:enable ssc
	params->dsi.HS_TRAIL = 4;
	params->dsi.CLK_TRAIL = 3;
	params->dsi.hstx_voltage = 0xB; /* set mipitx voltage at 0.46v */

	params->physical_width = 89;
	params->physical_height = 152;
}

static void lcm_power_on(void)
{
	int ret;

	if (lcm_data.lcm_power_mode == LCM_POWER_PMIC_VCN33) {
		pr_info("[LCM]: lcm_power_mode is pmic vcn33.\n");
		if (lcm_data.vcn33_ldo == NULL) {
			pr_err("vcn33_ldo is NULL!\n");
			return;
		}

		ret = regulator_enable(lcm_data.vcn33_ldo);
	} else {
		pr_info("[LCM]: lcm_power_mode is ldo gpio.\n");
		lcm_set_pwr_en(TRUE);
	}
}

static void init_ota7290x_lcm(void)
{
	if (lcm_data.lcm_id == OTA7290B_4POWER_KD_HSD)
		push_table(lcm_init_kd_hsd_ota7290b,
			sizeof(lcm_init_kd_hsd_ota7290b) / sizeof(struct LCM_setting_table),
			1);
	else if (lcm_data.lcm_id == OTA7290N_4POWER_KD_HSD)
		push_table(lcm_init_kd_hsd_ota7290n,
			sizeof(lcm_init_kd_hsd_ota7290n) / sizeof(struct LCM_setting_table),
			1);
	else if (lcm_data.lcm_id == OTA7290N_4POWER_KD_INX)
		push_table(lcm_init_kd_inx_ota7290n,
			sizeof(lcm_init_kd_inx_ota7290n) / sizeof(struct LCM_setting_table),
			1);
	else {
		pr_warn("[LCM]cannot match a correct lcm id,use defualt initial.\n");
		push_table(lcm_init_kd_hsd_ota7290b,
			sizeof(lcm_init_kd_hsd_ota7290b) / sizeof(struct LCM_setting_table),
			1);
	}
}

static void lcm_reset(void)
{
	pr_notice("[LCM][Ota7290x]%s enter\n", __func__);

	lcm_set_rst(GPIO_OUT_HIGH);
	MDELAY(3);
	lcm_set_rst(GPIO_OUT_LOW);
	MDELAY(2);
	lcm_set_rst(GPIO_OUT_HIGH);
	MDELAY(2);
}

static void lcm_init(void)
{
	pr_notice("[LCM][Ota7290x] %s enter.\n", __func__);

	init_ota7290x_lcm();
}

static void lcm_resume(void)
{
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG) || defined(CONFIG_AMZN_MINERVA_METRICS_LOG)
	char buf[512];
#endif

#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG)
	snprintf(buf, sizeof(buf), "%s:lcd:resume=1;CT;1:NR", __func__);
	log_to_metrics(ANDROID_LOG_INFO, "LCDEvent", buf);
#endif

#ifdef CONFIG_AMZN_MINERVA_METRICS_LOG
	minerva_metrics_log(buf, 512, "%s:%s:100:%s,%s,%s,%s,lcm_state=lcm_resume;SY,"
			"ESD_Recovery=0;IN:us-east-1",
			METRICS_LCD_GROUP_ID, METRICS_LCD_SCHEMA_ID,
			PREDEFINED_ESSENTIAL_KEY, PREDEFINED_MODEL_KEY,
			PREDEFINED_TZ_KEY, PREDEFINED_DEVICE_LANGUAGE_KEY);
#endif
	pr_notice("[LCM][Ota7290x] %s enter\n", __func__);

	init_ota7290x_lcm();
}

static void lcm_init_power(void)
{
	pr_notice("[LCM][Ota7290x] %s enter\n", __func__);

	lcm_power_on();
	MDELAY(3);
	lcm_reset();
	lcm_set_bias_en(TRUE);
	MDELAY(11);
}

static void lcm_resume_power(void)
{
	pr_notice("[LCM][Ota7290x] %s enter\n", __func__);

	lcm_power_on();
	MDELAY(3);
	lcm_reset();
	lcm_set_bias_en(TRUE);
	MDELAY(11);
}

static void lcm_suspend(void)
{
#if defined(CONFIG_AMZN_METRICS_LOG) || defined(CONFIG_AMZN_MINERVA_METRICS_LOG)
	char buf[512];
#endif

#if defined(CONFIG_AMZN_METRICS_LOG)
	snprintf(buf, sizeof(buf), "%s:lcd:suspend=1;CT;1:NR", __func__);
	log_to_metrics(ANDROID_LOG_INFO, "LCDEvent", buf);
#endif

#ifdef CONFIG_AMZN_MINERVA_METRICS_LOG
	minerva_metrics_log(buf, 512, "%s:%s:100:%s,%s,%s,%s,lcm_state=lcm_suspend;SY,"
			"ESD_Recovery=0;IN:us-east-1",
			METRICS_LCD_GROUP_ID, METRICS_LCD_SCHEMA_ID,
			PREDEFINED_ESSENTIAL_KEY, PREDEFINED_MODEL_KEY,
			PREDEFINED_TZ_KEY, PREDEFINED_DEVICE_LANGUAGE_KEY);
#endif
	pr_notice("[LCM][Ota7290x] %s enter\n", __func__);
	push_table(lcm_suspend_setting,
			sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
			1);
}

static void lcm_suspend_power(void)
{
	pr_notice("[LCM][Ota7290x] %s enter\n", __func__);

	if (lcm_data.lcm_power_mode == LCM_POWER_PMIC_VCN33) {
		if (lcm_data.vcn33_ldo == NULL) {
			pr_err("vcn33_ldo is NULL!\n");
			return;
		}
	}
	MDELAY(21);
	lcm_set_bias_en(FALSE);

	MDELAY(12);
	lcm_set_rst(FALSE);
	if (lcm_data.lcm_power_mode == LCM_POWER_PMIC_VCN33) {
		pr_info("[LCM]: lcm_power_mode is pmic vcn33.\n");
		regulator_disable(lcm_data.vcn33_ldo);
	} else {
		pr_info("[LCM]: lcm_power_mode is ldo gpio.\n");
		lcm_set_pwr_en(FALSE);
	}
}

struct LCM_DRIVER ota7290x_wsvga_dsi_vdo_4power_quartz_lcm_drv = {
	.name           = "ota7290x_wsvga_dsi_vdo_4power_quartz",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_4power_ota7290x_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
};
