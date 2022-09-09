/************************************************************
 * wusb3801 Type-C Port Control Driver
 *------------------------------------------------------------
 * Copyright (c) 2021, WillSemi Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/cpu.h>
#include <linux/version.h>
#include <linux/pm_wakeup.h>
#include <linux/sched/clock.h>
#include <uapi/linux/sched/types.h>

#include <pd_dbg_info.h>
#include <tcpci.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
#include <linux/sched/rt.h>
#endif

/*
 *Bit operations if we don't want to include #include <linux/bitops.h>
 */

#undef  __CONST_FFS
#define __CONST_FFS(_x) \
	((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) :\
					((_x) & 0x04 ? 2 : 3)) :\
			((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) :\
					((_x) & 0x40 ? 6 : 7)))

#undef  FFS
#define FFS(_x) \
	((_x) ? __CONST_FFS(_x) : 0)

#undef  BITS
#define BITS(_end, _start) \
	((BIT(_end) - BIT(_start)) + BIT(_end))

#undef  __BITS_GET
#define __BITS_GET(_byte, _mask, _shift) \
	(((_byte) & (_mask)) >> (_shift))

#undef  BITS_GET
#define BITS_GET(_byte, _bit) \
	__BITS_GET(_byte, _bit, FFS(_bit))

#undef  __BITS_SET
#define __BITS_SET(_byte, _mask, _shift, _val) \
	(((_byte) & ~(_mask)) | (((_val) << (_shift)) & (_mask)))

#undef  BITS_SET
#define BITS_SET(_byte, _bit, _val) \
	__BITS_SET(_byte, _bit, FFS(_bit), _val)

#undef  BITS_MATCH
#define BITS_MATCH(_byte, _bit) \
	(((_byte) & (_bit)) == (_bit))

/* Register Map */
#define WUSB3801_DRV_VERSION	"3.1.0_MTK"

#define WUSB3801_REG_VERSION_ID         0x01
#define WUSB3801_REG_CONTROL0           0x02
#define WUSB3801_REG_INTERRUPT          0x03
#define WUSB3801_REG_STATUS             0x04
#define WUSB3801_REG_CONTROL1           0x05
#define WUSB3801_REG_TEST0              0x06
#define WUSB3801_REG_TEST_01            0x07
#define WUSB3801_REG_TEST_02            0x08
#define WUSB3801_REG_TEST_03            0x09
#define WUSB3801_REG_TEST_04            0x0A
#define WUSB3801_REG_TEST_05            0x0B
#define WUSB3801_REG_TEST_06            0x0C
#define WUSB3801_REG_TEST_07            0x0D
#define WUSB3801_REG_TEST_08            0x0E
#define WUSB3801_REG_TEST_09            0x0F
#define WUSB3801_REG_TEST_0A            0x10
#define WUSB3801_REG_TEST_0B            0x11
#define WUSB3801_REG_TEST_0C            0x12
#define WUSB3801_REG_TEST_0D            0x13
#define WUSB3801_REG_TEST_0E            0x14
#define WUSB3801_REG_TEST_0F            0x15
#define WUSB3801_REG_TEST_10            0x16
#define WUSB3801_REG_TEST_11            0x17
#define WUSB3801_REG_TEST_12            0x18

#define WUSB3801_SLAVE_ADDR0            0xc0
#define WUSB3801_SLAVE_ADDR1            0xd0

/*Available modes*/
#define WUSB3801_DRP_ACC                (BIT_REG_CTRL0_RLE_DRP)
#define WUSB3801_DRP                    (BIT_REG_CTRL0_RLE_DRP | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_SNK_ACC                (BIT_REG_CTRL0_RLE_SNK)
#define WUSB3801_SNK                    (BIT_REG_CTRL0_RLE_SNK | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_SRC_ACC                (BIT_REG_CTRL0_RLE_SRC)
#define WUSB3801_SRC                    (BIT_REG_CTRL0_RLE_SRC | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_DRP_PREFER_SRC_ACC     (WUSB3801_DRP_ACC | BIT_REG_CTRL0_TRY_SRC)
#define WUSB3801_DRP_PREFER_SRC         (WUSB3801_DRP     | BIT_REG_CTRL0_TRY_SRC)
#define WUSB3801_DRP_PREFER_SNK_ACC     (WUSB3801_DRP_ACC | BIT_REG_CTRL0_TRY_SNK)
#define WUSB3801_DRP_PREFER_SNK         (WUSB3801_DRP     | BIT_REG_CTRL0_TRY_SNK)

/*TODO: redefine your prefer role here*/
#define WUSB3801_INIT_MODE              (WUSB3801_DRP_PREFER_SNK_ACC)

/*Registers relevant values*/
#define WUSB3801_VENDOR_ID              0x06
#define WUSB3801_ENABLE_INT              0x24

/*Switch to enable/disable feature of specified Registers*/
#define BIT_REG_CTRL0_DIS_ACC           (0x01 << 7)
#define BIT_REG_CTRL0_TRY_SRC           (0x02 << 5)
#define BIT_REG_CTRL0_TRY_SNK           (0x01 << 5)
#define BIT_REG_CTRL0_CUR_DEF           (0x00 << 3)
#define BIT_REG_CTRL0_CUR_1P5           (0x01 << 3)
#define BIT_REG_CTRL0_CUR_3P0           (0x02 << 3)
#define BIT_REG_CTRL0_RLE_SNK           (0x00 << 1)
#define BIT_REG_CTRL0_RLE_SRC           (0x01 << 1)
#define BIT_REG_CTRL0_RLE_DRP           (0x02 << 1)
#define BIT_REG_CTRL0_INT_MSK           (0x01 << 0)

#define BIT_REG_STATUS_VBUS             (0x01 << 7)
#define BIT_REG_STATUS_STANDBY          (0x00 << 5)
#define BIT_REG_STATUS_CUR_DEF          (0x01 << 5)
#define BIT_REG_STATUS_CUR_MID          (0x02 << 5)
#define BIT_REG_STATUS_CUR_HIGH         (0x03 << 5)

#define BIT_REG_STATUS_ATC_STB          (0x00 << 1)
#define BIT_REG_STATUS_ATC_SNK          (0x01 << 1)
#define BIT_REG_STATUS_ATC_SRC          (0x02 << 1)
#define BIT_REG_STATUS_ATC_ACC          (0x03 << 1)
#define BIT_REG_STATUS_ATC_DACC         (0x04 << 1)

#define BIT_REG_STATUS_PLR_STB          (0x00 << 0)
#define BIT_REG_STATUS_PLR_CC1          (0x01 << 0)
#define BIT_REG_STATUS_PLR_CC2          (0x02 << 0)
#define BIT_REG_STATUS_PLR_BOTH         (0x03 << 0)

#define BIT_REG_CTRL1_SW02_DIN          (0x01 << 4)
#define BIT_REG_CTRL1_SW02_EN           (0x01 << 3)
#define BIT_REG_CTRL1_SW01_DIN          (0x01 << 2)
#define BIT_REG_CTRL1_SW01_EN           (0x01 << 1)
#define BIT_REG_CTRL1_SM_RST            (0x01 << 0)
#define BIT_REG_TEST02_FORCE_ERR_RCY    (0x01)

#define WUSB3801_WAIT_VBUS               0x40
/*Fixed duty cycle period. 40ms:40ms*/
#define WUSB3801_TGL_40MS                0
#define WUSB3801_HOST_DEFAULT            0
#define WUSB3801_HOST_1500MA             1
#define WUSB3801_HOST_3000MA             2
#define WUSB3801_INT_ENABLE              0x00
#define WUSB3801_INT_DISABLE             0x01
#define WUSB3801_DISABLED                0x0A
#define WUSB3801_ERR_REC                 0x01
#define WUSB3801_VBUS_OK                 0x80

#define WUSB3801_SNK_0MA                (0x00 << 5)
#define WUSB3801_SNK_DEFAULT            (0x01 << 5)
#define WUSB3801_SNK_1500MA             (0x02 << 5)
#define WUSB3801_SNK_3000MA             (0x03 << 5)
#define WUSB3801_ATTACH                  0x1C

//#define WUSB3801_TYPE_PWR_ACC           (0x00 << 2) /*Ra/Rd treated as Open*/
#define WUSB3801_TYPE_INVALID           (0x00)
#define WUSB3801_TYPE_SNK               (0x01 << 2)
#define WUSB3801_TYPE_SRC               (0x02 << 2)
#define WUSB3801_TYPE_AUD_ACC           (0x03 << 2)
#define WUSB3801_TYPE_DBG_ACC           (0x04 << 2)

#define WUSB3801_INT_DETACH              (0x01 << 1)
#define WUSB3801_INT_ATTACH              (0x01 << 0)

#define WUSB3801_REV20                   0x02

/* Masks for Read-Modified-Write operations*/
#define WUSB3801_HOST_CUR_MASK           0x18  /*Host current for IIC*/
#define WUSB3801_INT_MASK                0x01
#define WUSB3801_BCLVL_MASK              0x60
#define WUSB3801_TYPE_MASK               0x1C
#define WUSB3801_MODE_MASK               0xE6  /*Roles relevant bits*/
#define WUSB3801_INT_STS_MASK            0x03
#define WUSB3801_FORCE_ERR_RCY_MASK      0x80  /*Force Error recovery*/
#define WUSB3801_ROLE_MASK               0x06
#define WUSB3801_VENDOR_ID_MASK          0x07
#define WUSB3801_VERSION_ID_MASK         0xF8
#define WUSB3801_VENDOR_SUB_ID_MASK         0xA0
#define WUSB3801_POLARITY_CC_MASK        0x03
#define WUSB3801_CC_STS_MASK            0x03
#define WUSB3801_CONTROL0_DEFAULT        0x80

/* WUSB3801 STATES MACHINES */
#define WUSB3801_STATE_DISABLED             0x00
#define WUSB3801_STATE_ERROR_RECOVERY       0x01
#define WUSB3801_STATE_UNATTACHED_SNK       0x02
#define WUSB3801_STATE_UNATTACHED_SRC       0x03
#define WUSB3801_STATE_ATTACHWAIT_SNK       0x04
#define WUSB3801_STATE_ATTACHWAIT_SRC       0x05
#define WUSB3801_STATE_ATTACHED_SNK         0x06
#define WUSB3801_STATE_ATTACHED_SRC         0x07
#define WUSB3801_STATE_AUDIO_ACCESSORY      0x08
#define WUSB3801_STATE_DEBUG_ACCESSORY      0x09
#define WUSB3801_STATE_TRY_SNK              0x0A
#define WUSB3801_STATE_TRYWAIT_SRC          0x0B
#define WUSB3801_STATE_TRY_SRC              0x0C
#define WUSB3801_STATE_TRYWAIT_SNK          0x0D

#define WUSB3801_CC2_CONNECTED 1
#define WUSB3801_CC1_CONNECTED 0
#define WUSB3801_IRQ_WAKE_TIME	(1000) /* ms */
/*1.5 Seconds timeout for force detection*/
#define ROLE_SWITCH_TIMEOUT		              1500
#define DEBOUNCE_TIME_OUT				50

#define REVERSE_CHG_SOURCE				0X01
#define REVERSE_CHG_SINK				0X02
#define REVERSE_CHG_DRP					0X03

#ifdef CONFIG_ARM64
#define IS_ERR_VALUE_64(x) IS_ERR_VALUE((unsigned long)x)
#else
#define IS_ERR_VALUE_64(x) IS_ERR_VALUE(x)
#endif

/*Private data*/
typedef struct wusb3801_data {
	uint32_t  int_gpio;
	uint8_t  init_mode;
	uint8_t  dfp_power;
	uint8_t  dttime;
} wusb3801_data_t;

struct wusb3801_chip {
	struct i2c_client *client;
	struct device *dev;
	struct semaphore suspend_lock;
	struct tcpc_desc *tcpc_desc;
	struct tcpc_device *tcpc;
	struct wusb3801_data *pdata;
	int ufp_power;
	uint8_t mode;
	uint8_t init_state;
	uint8_t type;
	uint8_t state;
	uint8_t cc_mode;
	uint8_t bc_lvl;
	uint8_t dfp_power;
	uint8_t dttime;
	uint8_t attached;
	uint8_t defer_init;
	int try_attcnt;
	int irq_gpio;
	uint8_t dev_id;
	uint8_t dev_sub_id;
	int irq;
	int chip_id;
};

static int wusb3801_read_device(void *client, u32 reg, int len, void *dst)
{
	struct i2c_client *i2c = (struct i2c_client *)client;
	struct wusb3801_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0, count = 5;

	down(&chip->suspend_lock);
	while (count) {
		if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(i2c, reg, len, dst);
			if (ret < 0)
				count--;
			else
				goto out;
		} else {
			ret = i2c_smbus_read_byte_data(i2c, reg);
			if (ret < 0)
				count--;
			else {
				*(u8 *)dst = (u8)ret;
				goto out;
			}
		}
		usleep_range(100, 100);
	}
out:
	up(&chip->suspend_lock);
	return ret;
}

static int wusb3801_write_device(void *client, u32 reg, int len, const void *src)
{
	const u8 *data;
	struct i2c_client *i2c = (struct i2c_client *)client;
	struct wusb3801_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0, count = 5;

	down(&chip->suspend_lock);
	while (count) {
		if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(i2c,
							reg, len, src);
			if (ret < 0)
				count--;
			else
				goto out;
		} else {
			data = src;
			ret = i2c_smbus_write_byte_data(i2c, reg, *data);
			if (ret < 0)
				count--;
			else
				goto out;
		}
		usleep_range(100, 100);
	}
out:
	up(&chip->suspend_lock);
	return ret;
}

static int wusb3801_reg_read(struct i2c_client *i2c, u8 reg)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(i2c);
	u8 val = 0;
	int ret = 0;

	ret = wusb3801_read_device(chip->client, reg, 1, &val);
	if (ret < 0) {
		dev_err(chip->dev, "wusb3801 reg read fail\n");
		return ret;
	}
	return val;
}

static int wusb3801_reg_write(struct i2c_client *i2c, u8 reg, const u8 data)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;

	ret = wusb3801_write_device(chip->client, reg, 1, &data);
	if (ret < 0)
		dev_err(chip->dev, "wusb3801 reg write fail\n");
	return ret;
}

static inline int wusb3801_i2c_write8(
	struct tcpc_device *tcpc, u8 reg, const u8 data)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);

	return wusb3801_reg_write(chip->client, reg, data);
}

static inline int wusb3801_i2c_read8(struct tcpc_device *tcpc, u8 reg)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);

	return wusb3801_reg_read(chip->client, reg);
}

static void wusb3801_do_work_handler(struct wusb3801_chip *chip)
{
	struct tcpc_device *tcpc;
	int int_sts, rc;
	uint8_t status, type;

	pr_debug("%s enter...\n", __func__);
	tcpc = chip->tcpc;
	tcpci_lock_typec(tcpc);

	/* get interrupt */
	rc = wusb3801_i2c_read8(chip->tcpc, WUSB3801_REG_INTERRUPT);
	if (rc < 0) {
		pr_err("%s: failed to read interrupt\n", __func__);
		goto skip;
	}
	pr_info("%s : 0x%02x\n", __func__, rc);
	int_sts = rc & WUSB3801_INT_STS_MASK;
	rc = wusb3801_i2c_read8(chip->tcpc, WUSB3801_REG_STATUS);
	if (rc < 0) {
		pr_err("%s: failed to read reg status\n", __func__);
		goto skip;
	}
	pr_info("%s:int_sts[0x%02x], status_reg:0x%02x\n", __func__, int_sts, rc);
	status = (rc & WUSB3801_ATTACH) ? true : false;
	type = status ? rc & WUSB3801_TYPE_MASK : WUSB3801_TYPE_INVALID;
	pr_info("sts[0x%02x], type[0x%02x]\n", status, type);
	if (int_sts & WUSB3801_INT_DETACH) {
		tcpc->typec_attach_new = TYPEC_UNATTACHED;
		pr_info("%s attach type %d, %d\n", __func__, chip->tcpc->typec_attach_new,
							chip->tcpc->typec_attach_old);
		tcpci_notify_typec_state(tcpc);
		if (tcpc->typec_attach_old == TYPEC_ATTACHED_SRC)
			tcpci_source_vbus(tcpc, TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_0V, 0);

		tcpc->typec_attach_old = TYPEC_UNATTACHED;
	}
	if (int_sts & WUSB3801_INT_ATTACH) {
		switch (type) {
		case WUSB3801_TYPE_SNK:
			if (tcpc->typec_attach_new != TYPEC_ATTACHED_SRC) {
				tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
				tcpci_source_vbus(tcpc, TCP_VBUS_CTRL_TYPEC,
					TCPC_VBUS_SOURCE_5V, 0);
				pr_info("%s attach type snk %d, %d\n", __func__,
					chip->tcpc->typec_attach_new, chip->tcpc->typec_attach_old);
				tcpci_notify_typec_state(tcpc);
				tcpc->typec_attach_old = TYPEC_ATTACHED_SRC;
			}
			break;
		case WUSB3801_TYPE_SRC:
			if (tcpc->typec_attach_new != TYPEC_ATTACHED_SNK) {
				tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
				pr_info("%s: attach type src %d, %d\n", __func__,
					chip->tcpc->typec_attach_new, chip->tcpc->typec_attach_old);
				tcpci_notify_typec_state(tcpc);
				tcpc->typec_attach_old = TYPEC_ATTACHED_SNK;
			}
			break;
		default:
			pr_info("%s: Unknwon type[0x%02x]\n", __func__, type);
			break;
		}
	}

skip:
	tcpci_unlock_typec(tcpc);
}

static irqreturn_t wusb3801_typec_thread(int irq, void *ptr)
{
	struct wusb3801_chip *chip = ptr;

	pr_info("%s enter\n", __func__);
	wusb3801_do_work_handler(chip);

	return IRQ_HANDLED;
}

static int wusb3801_init_alert(struct tcpc_device *tcpc)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);
	int ret;
	char *name;
	int len;

	len = strlen(chip->tcpc_desc->name);
	name = devm_kzalloc(chip->dev, len+5, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	snprintf(name, PAGE_SIZE, "%s-IRQ", chip->tcpc_desc->name);

	pr_info("%s name = %s, gpio = %d\n", __func__,
				chip->tcpc_desc->name, chip->irq_gpio);

	ret = devm_gpio_request(chip->dev, chip->irq_gpio, name);
	if (ret < 0) {
		pr_err("Error: failed to request GPIO%d (ret = %d)\n",
		chip->irq_gpio, ret);
		goto init_alert_err;
	}

	ret = gpio_direction_input(chip->irq_gpio);
	if (ret < 0) {
		pr_err("Error: failed to set GPIO%d as input pin(ret = %d)\n",
		chip->irq_gpio, ret);
		goto init_alert_err;
	}

	chip->irq = gpio_to_irq(chip->irq_gpio);
	if (chip->irq <= 0) {
		pr_err("%s gpio to irq fail, chip->irq(%d)\n",
						__func__, chip->irq);
		goto init_alert_err;
	}

	pr_info("%s : IRQ number = %d\n", __func__, chip->irq);
	i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
	ret = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_TEST_02);

	wusb3801_i2c_write8(chip->tcpc, WUSB3801_REG_TEST_02, 0x00);
	wusb3801_i2c_write8(chip->tcpc, WUSB3801_REG_TEST_09, 0x00);

	ret = request_threaded_irq(chip->irq, NULL, wusb3801_typec_thread,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT, name, chip);

	if (ret < 0) {
		pr_err("Error: failed to request irq%d (gpio = %d, ret = %d)\n",
			chip->irq, chip->irq_gpio, ret);
		goto init_alert_err;
	}

	enable_irq_wake(chip->irq);
	return 0;
init_alert_err:
	return -EINVAL;
}

int wusb3801_alert_status_clear(struct tcpc_device *tcpc, uint32_t mask)
{
	pr_info("%s enter\n", __func__);
	return 0;
}

static int wusb3801_tcpc_init(struct tcpc_device *tcpc, bool sw_reset)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);
	int  ret;

	pr_info("%s enter...\n", __func__);
	wusb3801_i2c_write8(chip->tcpc,
			WUSB3801_REG_CONTROL0, 0x24);
	ret = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_CONTROL0);
	pr_info("%s WUSB3801_REG_CONTROL0 0x%02x\n", __func__, ret);

	return 0;
}

int wusb3801_fault_status_clear(struct tcpc_device *tcpc, uint8_t status)
{
	pr_info("%s enter\n", __func__);
	return 0;
}

int wusb3801_get_alert_mask(struct tcpc_device *tcpc, uint32_t *mask)
{
	pr_info("%s enter\n", __func__);
	return 0;
}

int wusb3801_get_alert_status(struct tcpc_device *tcpc, uint32_t *alert)
{
	pr_info("%s enter\n", __func__);
	return 0;
}

static int wusb3801_get_power_status(
		struct tcpc_device *tcpc, uint16_t *pwr_status)
{
	pr_info("%s enter\n", __func__);
	return 0;
}

int wusb3801_get_fault_status(struct tcpc_device *tcpc, uint8_t *status)
{
	pr_info("%s enter\n", __func__);
	return 0;
}

static int wusb3801_get_cc(struct tcpc_device *tcpc, int *cc1, int *cc2)
{

	pr_info("%s enter\n", __func__);
	return 0;
}

static int wusb3801_set_cc(struct tcpc_device *tcpc, int pull)
{
	pr_info("%s enter\n", __func__);
	return 0;
}

static int wusb3801_set_polarity(struct tcpc_device *tcpc, int polarity)
{

	pr_info("%s enter\n", __func__);
	return 0;
}

static int wusb3801_set_low_rp_duty(struct tcpc_device *tcpc, bool low_rp)
{
	pr_info("%s enter\n", __func__);
	return 0;
}

static int wusb3801_set_vconn(struct tcpc_device *tcpc, int enable)
{
	pr_info("%s enter\n", __func__);
	return 0;
}

static int wusb3801_tcpc_deinit(struct tcpc_device *tcpc_dev)
{
	pr_info("%s enter\n", __func__);
	return 0;
}

static int wusb3801_set_mode(struct wusb3801_chip *chip, uint8_t mode)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_CONTROL0);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: fail to read mode\n", __func__);
		return rc;
	}

	rc &= ~WUSB3801_MODE_MASK;

	rc |= (mode | WUSB3801_INT_MASK);/*Disable the chip interrupt*/
	rc = i2c_smbus_write_byte_data(chip->client,
			   WUSB3801_REG_CONTROL0, rc);

	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "failed to write mode(%d)\n", rc);
		return rc;
	}

	/* Clear the chip interrupt */
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: fail to clear chip interrupt\n", __func__);
		return rc;
	}

	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_CONTROL0);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: fail to read chip interrupt\n", __func__);
		return rc;
	}
	rc &= ~WUSB3801_INT_MASK;/*enable the chip interrupt*/
	rc = i2c_smbus_write_byte_data(chip->client,
			   WUSB3801_REG_CONTROL0, rc);

	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "failed to enable chip interrupt(%d)\n", rc);
		return rc;
	}

	return mode;
}

static int wusb3801_set_cc_mode(struct tcpc_device *tcpc, int mode)
{

	struct wusb3801_chip *chip = NULL;
	unsigned int rc;

	pr_info("%s: wusb3801 mode = %d\n", __func__, mode);
	chip = tcpc_get_dev_data(tcpc);
	chip->cc_mode = (mode == TYPEC_ROLE_SNK) ? TYPEC_ROLE_SNK : TYPEC_ROLE_TRY_SNK;

	if (mode == TYPEC_ROLE_SNK) {
		rc = wusb3801_set_mode(chip, WUSB3801_SNK);
		if (IS_ERR_VALUE_64(rc)) {
			pr_info("%s: failed to set mode\n",
				__func__);
			return rc;
		}
	} else {
		rc = wusb3801_set_mode(chip, WUSB3801_DRP_PREFER_SRC);
		if (IS_ERR_VALUE_64(rc)) {
			pr_info("%s: failed to set mode\n",
			__func__);
			return rc;
		}
	}


	return 0;
}

static struct tcpc_ops wusb3801_tcpc_ops = {
	.init = wusb3801_tcpc_init,
	.alert_status_clear = wusb3801_alert_status_clear,
	.fault_status_clear = wusb3801_fault_status_clear,
	.get_alert_mask = wusb3801_get_alert_mask,
	.get_alert_status = wusb3801_get_alert_status,
	.get_power_status = wusb3801_get_power_status,
	.get_fault_status = wusb3801_get_fault_status,
	.get_cc = wusb3801_get_cc,
	.set_cc = wusb3801_set_cc,
	.set_mode = wusb3801_set_cc_mode,
	.set_polarity = wusb3801_set_polarity,
	.set_low_rp_duty = wusb3801_set_low_rp_duty,
	.set_vconn = wusb3801_set_vconn,
	.deinit = wusb3801_tcpc_deinit,
};

static int mt_parse_dt(struct wusb3801_chip *chip, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;

	if (!np)
		return -EINVAL;

	np = of_find_node_by_name(NULL, "wusb3801");
	if (!np) {
		pr_err("%s find node type_c_port0 fail\n", __func__);
		return -ENODEV;
	}

	ret = of_get_named_gpio(np, "wusb3801,int-gpio", 0);
	if (ret < 0) {
		pr_err("wusb3801 int-gpio is not available\n");
		return ret;
	}
	chip->irq_gpio = ret;
	pr_err("wusb3801 %s intr_gpio =%d,=%d\n", __func__, ret, chip->irq_gpio);

	return ret;
}

static int wusb3801_tcpcdev_init(struct wusb3801_chip *chip, struct device *dev)
{
	struct tcpc_desc *desc;
	struct device_node *np;
	u32 val, len;
	const char *name = "default";

	np = of_find_node_by_name(NULL, "type_c_port0");
	if (!np) {
		pr_err("%s find node mt6370 fail\n", __func__);
		return -ENODEV;
	}

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	if (of_property_read_u32(np, "mt-tcpc,role_def", &val) >= 0) {
		if (val >= TYPEC_ROLE_NR)
			desc->role_def = TYPEC_ROLE_DRP;
		else
			desc->role_def = val;
	} else {
		dev_info(dev, "use default Role DRP\n");
		desc->role_def = TYPEC_ROLE_DRP;
	}

	if (of_property_read_u32(
		np, "mt-tcpc,notifier_supply_num", &val) >= 0) {
		if (val < 0)
			desc->notifier_supply_num = 0;
		else
			desc->notifier_supply_num = val;
	} else
		desc->notifier_supply_num = 0;

	if (of_property_read_u32(np, "mt-tcpc,rp_level", &val) >= 0) {
		switch (val) {
		case 0: /* RP Default */
			desc->rp_lvl = TYPEC_CC_RP_DFT;
			break;
		case 1: /* RP 1.5V */
			desc->rp_lvl = TYPEC_CC_RP_1_5;
			break;
		case 2: /* RP 3.0V */
			desc->rp_lvl = TYPEC_CC_RP_3_0;
			break;
		default:
			break;
		}
	}

	of_property_read_string(np, "mt-tcpc,name", (char const **)&name);
	len = strlen(name);
	desc->name = kzalloc(len+1, GFP_KERNEL);
	if (!desc->name)
		return -ENOMEM;
	strlcpy((char *)desc->name, name, len+1);
	chip->tcpc_desc = desc;
	chip->tcpc = tcpc_device_register(dev,
			desc, &wusb3801_tcpc_ops, chip);
	if (IS_ERR(chip->tcpc))
		return -EINVAL;
	chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
	chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;

	return 0;
}

static uint8_t dev_sub_id;
static inline int wusb3801_check_revision(struct i2c_client *client)
{
	int rc;

	rc = i2c_smbus_read_byte_data(client, WUSB3801_REG_VERSION_ID);
	if (rc < 0)
		return -EIO;
	pr_info("VendorID register: 0x%02x\n", rc);
	if ((rc & WUSB3801_VENDOR_ID_MASK) != WUSB3801_VENDOR_ID)
		return -ENODEV;
	pr_info("Vendor id: 0x%02x, Version id: 0x%02x\n", rc & WUSB3801_VENDOR_ID_MASK,
						(rc & WUSB3801_VERSION_ID_MASK) >> 3);

	rc = i2c_smbus_read_byte_data(client, WUSB3801_REG_TEST_01);
	if (rc > 0)
		dev_sub_id = rc & WUSB3801_VENDOR_SUB_ID_MASK;
	pr_info("VendorSUBID register: 0x%02x\n", rc & WUSB3801_VENDOR_SUB_ID_MASK);

	return WUSB3801_VENDOR_ID;
}

static int wusb3801_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct wusb3801_chip *chip;
	int ret = 0, chip_id;
	bool use_dt = client->dev.of_node;

	pr_info("%s\n", __func__);
	if (i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_I2C_BLOCK | I2C_FUNC_SMBUS_BYTE_DATA))
		pr_info("I2C functionality : OK...\n");
	else
		pr_err("I2C functionality check : failuare...\n");

	chip_id = wusb3801_check_revision(client);
	if (chip_id < 0)
		return chip_id;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (use_dt)
		mt_parse_dt(chip, &client->dev);
	else {
		dev_err(&client->dev, "no dts node\n");
		return -ENODEV;
	}
	chip->dev = &client->dev;
	chip->client = client;
	chip->cc_mode = TYPEC_ROLE_DRP;
	sema_init(&chip->suspend_lock, 1);
	i2c_set_clientdata(client, chip);

	chip->chip_id = chip_id;
	pr_info("wusb3801_chipID = 0x%0x\n", chip_id);

	ret = wusb3801_tcpcdev_init(chip, &client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "wusb3801 tcpc dev init fail\n");
		goto err_tcpc_reg;
	}
	pr_info("wusb3801_tcpcdev_init\n");

	ret = wusb3801_init_alert(chip->tcpc);
	if (ret < 0) {
		pr_err("wusb3801 init alert fail\n");
		goto err_irq_init;
	}

	wusb3801_i2c_write8(chip->tcpc, WUSB3801_REG_CONTROL0, WUSB3801_CONTROL0_DEFAULT);
	wusb3801_i2c_write8(chip->tcpc, WUSB3801_REG_TEST_02, 0x00);
	wusb3801_i2c_write8(chip->tcpc, WUSB3801_REG_TEST_09, 0x00);

	pr_info("%s probe OK!\n", __func__);
	return 0;

err_irq_init:
	tcpc_device_unregister(chip->dev, chip->tcpc);
err_tcpc_reg:
	return ret;
}

static int wusb3801_i2c_remove(struct i2c_client *client)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(client);

	if (chip)
		tcpc_device_unregister(chip->dev, chip->tcpc);

	pr_info("%s ok!\n", __func__);

	return 0;
}

static int wusb3801_write_masked_byte(struct i2c_client *client,
					uint8_t addr, uint8_t mask, uint8_t val)
{
	int rc;

	if (!mask) {
		rc = -EINVAL;
		goto out;
	}
	rc = i2c_smbus_read_byte_data(client, addr);
	if (!IS_ERR_VALUE_64(rc)) {
		rc = i2c_smbus_write_byte_data(client,
			addr, BITS_SET((uint8_t)rc, mask, val));
	}
out:
	return rc;
}

static int wusb3801_init_force_dfp_power(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	rc = wusb3801_write_masked_byte(chip->client,
					WUSB3801_REG_CONTROL0,
					WUSB3801_HOST_CUR_MASK,
					WUSB3801_HOST_1500MA);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "failed to write current\n");
		return rc;
	}

	chip->dfp_power = WUSB3801_HOST_1500MA;

	dev_dbg(cdev, "%s: host current (%d)\n", __func__, rc);

	return rc;
}

static void wusb3801_detach(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;

	dev_err(cdev, "%s: type[0x%02x] chipstate[0x%02x]\n",
			__func__, chip->type, chip->state);

	switch (chip->state) {
	case WUSB3801_STATE_ATTACHED_SRC:
		break;
	case WUSB3801_STATE_ATTACHED_SNK:
		break;
	case WUSB3801_STATE_DEBUG_ACCESSORY:
		break;
	case WUSB3801_STATE_AUDIO_ACCESSORY:
		break;
	case WUSB3801_STATE_DISABLED:
	case WUSB3801_STATE_ERROR_RECOVERY:
		break;
	default:
		dev_err(cdev, "%s: Invaild chipstate[0x%02x]\n",
				__func__, chip->state);
		break;
	}
	chip->type = WUSB3801_TYPE_INVALID;
	chip->bc_lvl = WUSB3801_SNK_0MA;
	chip->ufp_power  = 0;
	chip->try_attcnt = 0;
	chip->attached   = 0;
}

static int wusb3801_set_chip_state(struct wusb3801_chip *chip, uint8_t state)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	if (state > WUSB3801_STATE_UNATTACHED_SRC)
		return -EINVAL;

	rc = i2c_smbus_write_byte_data(chip->client,
				WUSB3801_REG_CONTROL1,
			   (state == WUSB3801_STATE_DISABLED) ? \
				WUSB3801_DISABLED :        \
				0);

	if (IS_ERR_VALUE_64(rc))
		dev_err(cdev, "failed to write state machine(%d)\n", rc);

	chip->init_state = state;

	return rc;
}

static int wusb3801_init_reg(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	/* change current */
	rc = wusb3801_init_force_dfp_power(chip);
	if (IS_ERR_VALUE_64(rc))
		dev_err(cdev, "%s: failed to force dfp power\n",
				__func__);

	rc = wusb3801_set_mode(chip, chip->pdata->init_mode);
	if (IS_ERR_VALUE_64(rc))
		dev_err(cdev, "%s: failed to set mode\n",
				__func__);

	rc = wusb3801_set_chip_state(chip,
				WUSB3801_STATE_ERROR_RECOVERY);

	if (IS_ERR_VALUE_64(rc))
		dev_err(cdev, "%s: Reset state failed.\n",
				__func__);

	return rc;
}

static int wusb3801_update_status(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;

	/* Get control0 register */
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_CONTROL0);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: fail to read mode\n", __func__);
		return rc;
	}

	chip->mode      =  rc & WUSB3801_MODE_MASK;
	chip->dfp_power =  BITS_GET(rc, WUSB3801_HOST_CUR_MASK);
	chip->dttime    =  WUSB3801_TGL_40MS;

	return 0;
}

static int wusb3801_get_mode(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;

	/* Get control0 register */
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_CONTROL0);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: fail to read mode\n", __func__);
		return rc;
	}

	chip->mode      =  rc & WUSB3801_MODE_MASK;

	return chip->mode;
}

static int wusb3801_reset_device(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	rc = wusb3801_update_status(chip);
	if (IS_ERR_VALUE_64(rc))
		dev_err(cdev, "fail to read status\n");

	rc = wusb3801_init_reg(chip);
	if (IS_ERR_VALUE_64(rc))
		dev_err(cdev, "fail to init reg\n");

	wusb3801_detach(chip);

	/* clear global interrupt mask */
	rc = wusb3801_write_masked_byte(chip->client,
				WUSB3801_REG_CONTROL0,
				WUSB3801_INT_MASK,
				WUSB3801_INT_ENABLE);
	if (IS_ERR_VALUE_64(rc)) {
		dev_err(cdev, "%s: fail to init\n", __func__);
		return rc;
	}
	dev_info(cdev, "mode[0x%02x], host_cur[0x%02x], dttime[0x%02x]\n",
			chip->mode, chip->dfp_power, chip->dttime);

	return rc;
}

static int wusb3801_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = (struct wusb3801_chip *)i2c_get_clientdata(client);
	int rc, status;

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(chip->irq_gpio);
	disable_irq(chip->irq_gpio);

	status = wusb3801_get_mode(chip);
	pr_info("%s: wusb3801 status = 0x%2x\n", __func__, status);
	if (status != WUSB3801_SNK) {
		rc = wusb3801_set_mode(chip, WUSB3801_SNK);
		pr_info("%s rc = 0x%2x\n", __func__, rc);

		if (IS_ERR_VALUE_64(rc)) {
			if (IS_ERR_VALUE_64(wusb3801_reset_device(chip)))
				pr_info("%s  failed to reset\n", __func__);
			enable_irq(chip->irq_gpio);
			if (device_may_wakeup(&client->dev))
				disable_irq_wake(chip->irq_gpio);
			return rc;
		}
	}

	return 0;
}

static int wusb3801_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip  = (struct wusb3801_chip *)i2c_get_clientdata(client);
	int rc, status;

	enable_irq(chip->irq_gpio);
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(chip->irq_gpio);

	status = wusb3801_get_mode(chip);
	pr_info("%s: wusb3801 status = 0x%2x, chip->cc_mode = 0x%2x\n",
		__func__, status, chip->cc_mode);
	if (status == WUSB3801_SNK && chip->cc_mode >= TYPEC_ROLE_DRP) {
		rc = wusb3801_set_mode(chip, WUSB3801_DRP_PREFER_SRC);

		if (IS_ERR_VALUE_64(rc)) {
			if (IS_ERR_VALUE_64(wusb3801_reset_device(chip)))
				pr_info("%s  failed to reset\n", __func__);
			if (device_may_wakeup(&client->dev))
				enable_irq_wake(chip->irq_gpio);
			disable_irq(chip->irq_gpio);
			return rc;
		}
	}

	return 0;
}

static void wusb3801_shutdown(struct i2c_client *client)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int rc;

	/* Please reset IC here */
	if (chip && chip->irq)
		disable_irq(chip->irq);

	rc = wusb3801_set_mode(chip, WUSB3801_SNK);
	if (IS_ERR_VALUE_64(rc)) {
		if (IS_ERR_VALUE_64(wusb3801_reset_device(chip)))
			pr_info("%s  failed to reset\n", __func__);
	}
	pr_info("%s ok!\n", __func__);
}

#ifdef CONFIG_PM_RUNTIME
static int wusb3801_pm_suspend_runtime(struct device *device)
{
	pr_info("%s pm_suspend\n", __func__);
	return 0;
}

static int wusb3801_pm_resume_runtime(struct device *device)
{
	pr_info("%s pm_runtime\n", __func__);
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static SIMPLE_DEV_PM_OPS(wusb3801_PM_OPS,
			wusb3801_i2c_suspend, wusb3801_i2c_resume);



static const struct i2c_device_id wusb3801_id_table[] = {
	{"wusb3801", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, wusb3801_id_table);

static const struct of_device_id rt_match_table[] = {
	{.compatible = "mediatek,usb_typec_c",},
	{},
};

static struct i2c_driver wusb3801_driver = {
	.driver = {
		.name = "usb_typec_c",
		.owner = THIS_MODULE,
		.of_match_table = rt_match_table,
		.pm = &wusb3801_PM_OPS,
	},
	.probe = wusb3801_i2c_probe,
	.remove = wusb3801_i2c_remove,
	.shutdown = wusb3801_shutdown,
	.id_table = wusb3801_id_table,
};

static int __init wusb3801_init(void)
{
	struct device_node *np;

	pr_info("%s (%s): initializing...\n", __func__, WUSB3801_DRV_VERSION);
	np = of_find_node_by_name(NULL, "wusb3801");

	if (np != NULL)
		pr_info("wusb3801 node found...\n");
	else
		pr_info("wusb3801 node not found...\n");

	return i2c_add_driver(&wusb3801_driver);
}
subsys_initcall(wusb3801_init);

static void __exit wusb3801_exit(void)
{
	i2c_del_driver(&wusb3801_driver);
}
module_exit(wusb3801_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("wusb3801 TCPC Driver");
MODULE_VERSION(WUSB3801_DRV_VERSION);
