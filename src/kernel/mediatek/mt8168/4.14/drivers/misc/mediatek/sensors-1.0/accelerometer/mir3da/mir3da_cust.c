/* For MTK android platform.
 *
 * mir3da.c - Linux kernel modules for 3-Axis Accelerometer
 *
 * Copyright (C) 2011-2013 MiraMEMS Sensing Technology Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "cust_acc.h"
#include "accel.h"
#include "mir3da_core.h"
#include "mir3da_cust.h"

#define MIR3DA_DRV_NAME	"mir3da"

#define MIR3DA_ERR_CLIENT	-2
#define MIR3DA_AXIS_X	0
#define MIR3DA_AXIS_Y	1
#define MIR3DA_AXIS_Z	2
#define MIR3DA_AXES_NUM	3
#define MIR3DA_DATA_BUF_NUM	3
#define MIR3DA_ACC_AXES_NUM	3
#define ACCEL_SELF_TEST_MIN_VAL	0
#define ACCEL_SELF_TEST_MAX_VAL	13000

#define MIR3DA_ACC_CALI_FILE "/data/inv_cal_data.bin"



#define CALI_SIZE 3 /*CALI_SIZE should not less than 3*/
static s16 accel_xyz_offset[MIR3DA_AXES_NUM] = { 0 };
static int accel_self_test[MIR3DA_ACC_AXES_NUM] = {0};
static int accel_cali_tolerance = 20;

struct mir3da_i2c_data {
	struct i2c_client	*client;
	struct acc_hw	hw;
	struct hwmsen_convert	cvt;

	bool flush;

	atomic_t	trace;
	atomic_t	suspend;
	atomic_t	selftest;
	atomic_t	enabled;
	s16	cali_sw[MIR3DA_AXES_NUM+1];
	s16	offset[MIR3DA_AXES_NUM + 1];
	s16	data[MIR3DA_AXES_NUM+1];
};
static bool sensor_power;
static struct GSENSOR_VECTOR3D gsensor_gain;
static MIR_HANDLE              mir_handle;
static int mir3da_init_flag;

#ifdef CONFIG_CUSTOM_KERNEL_ACCELEROMETER_MODULE
extern bool success_Flag;
#endif

static struct i2c_client		*mir3da_i2c_client;
static struct mir3da_i2c_data   *mir3da_obj_i2c_data;

#define MIR3DA_SUPPORT_CONCURRENCY_PROTECTION_ 1
extern int Log_level;
/*----------------------------------------------------------------------------*/
#define MI_DATA(format, ...)         \
	do { \
		if (DEBUG_DATA&Log_level) \
			printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);\
	} while (0)

#define MI_MSG(format, ...)           \
	do { \
		if (DEBUG_MSG&Log_level) \
			printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);\
	} while (0)

#define MI_ERR(format, ...)           \
	do { \
		if (DEBUG_ERR&Log_level) \
			printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__); \
	} while (0)

#define MI_FUN                        \
	do { \
		if (DEBUG_FUNC&Log_level) { \
			printk(KERN_ERR MI_TAG "%s is called, line: %d\n", __func__, __LINE__); } \
	} while (0)
#define MI_ASSERT(expr)	\
		do { \
			if (!(expr)) { \
				printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n", __FILE__, __LINE__, __func__, #expr); } \
		} while (0)


enum MIR3DA_TRC {
	MIR3DA_TRC_INFO = 0X01,
	MIR3DA_TRC_RAWDATA = 0x02,
};

static int mir3da_flush(void);
static int mir3da_local_init(void);
static int mir3da_local_remove(void);

#ifdef MIR3DA_SUPPORT_CONCURRENCY_PROTECTION_

static struct semaphore s_tSemaProtect;

static void mir3da_mutex_init(void)
{
	sema_init(&s_tSemaProtect, 1);
}

static int mir3da_mutex_lock(void)
{
	if (down_interruptible(&s_tSemaProtect))
		return (-ERESTARTSYS);

	return 0;
}

static void mir3da_mutex_unlock(void)
{
	up(&s_tSemaProtect);
}
#else
	#define mir3da_mutex_init()				do {} while (0)
	#define mir3da_mutex_lock()				do {} while (0)
	#define mir3da_mutex_unlock()			do {} while (0)
#endif

/*----------------------------------------------------------------------------*/
static int get_address(PLAT_HANDLE handle)
{
	if (handle == NULL) {
		MI_ERR("chip init failed !\n");
		return -1;
	}

	return ((struct i2c_client *)handle)->addr;
}
/*----------------------------------------------------------------------------*/
static int mir3da_resetCalibration(struct i2c_client *client)
{
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);

	MI_FUN;

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_readCalibration(struct i2c_client *client, int *dat)
{
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);

	MI_FUN;

	dat[MIR3DA_AXIS_X] = obj->cali_sw[MIR3DA_AXIS_X];
	dat[MIR3DA_AXIS_Y] = obj->cali_sw[MIR3DA_AXIS_Y];
	dat[MIR3DA_AXIS_Z] = obj->cali_sw[MIR3DA_AXIS_Z];

	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_writeCalibration(struct i2c_client *client, int dat[MIR3DA_AXES_NUM])
{
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[MIR3DA_AXES_NUM];

	MI_FUN;
	if (!obj || !dat) {
		MI_ERR("null ptr!!\n");
		return -EINVAL;

	} else {
		err = mir3da_readCalibration(client, cali);
		if (err != 0) {
			MI_ERR("read offset fail, %d\n", err);
			return err;
		}

		MI_MSG("write_cali  raw cali_sw[%d][%d][%d] dat[%d][%d][%d]", cali[0], cali[1], cali[2], dat[0], dat[1], dat[2]);

		cali[MIR3DA_AXIS_X] += dat[MIR3DA_AXIS_X];
		cali[MIR3DA_AXIS_Y] += dat[MIR3DA_AXIS_Y];
		cali[MIR3DA_AXIS_Z] += dat[MIR3DA_AXIS_Z];

		obj->cali_sw[MIR3DA_AXIS_X] = cali[MIR3DA_AXIS_X];

		obj->cali_sw[MIR3DA_AXIS_Y] = cali[MIR3DA_AXIS_Y];

		obj->cali_sw[MIR3DA_AXIS_Z] = cali[MIR3DA_AXIS_Z];

		MI_MSG("write_cali  new cali_sw[%d][%d][%d] ", obj->cali_sw[0], obj->cali_sw[1], obj->cali_sw[2]);
	}

	mdelay(1);

	return err;
}
/*----------------------------------------------------------------------------*/
static int mir3da_setPowerMode(struct i2c_client *client, bool enable)
{
	int ret;
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);

	MI_MSG("%s, enable = %d\n", __func__, enable);

	if (enable == sensor_power) {
		MI_ERR("Sensor power status should not be set again!!!\n");
		return 0;
	}

	ret = mir3da_set_enable(client, enable);
	if (ret == 0)
		sensor_power = enable;
	else if (atomic_read(&obj->trace) & MIR3DA_TRC_INFO) {
		MI_MSG("mir3da set power mode failed!\n");
		return -1;
	}

	if (mir3da_obj_i2c_data->flush) {
		if (sensor_power) {
			MI_MSG("remain flush, will call mir3da_flush in setPowerMode\n");
			mir3da_flush();
			} else	{
				mir3da_obj_i2c_data->flush = false;
			}
	}
	atomic_set(&obj->enabled, enable);
	return ret;
}
/*----------------------------------------------------------------------------*/
static int mir3da_readSensorData(struct i2c_client *client, int *pdata_x, int *pdata_y, int *pdata_z)
{
	struct mir3da_i2c_data *obj = (struct
				mir3da_i2c_data *)i2c_get_clientdata(client);
	unsigned char databuf[20];
	int acc[MIR3DA_AXES_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(unsigned char)*10);

	if (atomic_read(&obj->suspend)) {
		MI_ERR("Chip is in suspend mode, skip!!\n");
		return -EINVAL;
	}

	if (!obj) {
		*pdata_x = 0;
		*pdata_y = 0;
		*pdata_z = 0;
		return MIR3DA_ERR_CLIENT;
	}

	if (client == NULL) {
		*pdata_x = 0;
		*pdata_y = 0;
		*pdata_z = 0;
		return MIR3DA_ERR_CLIENT;
	}
	if (sensor_power == false) {
		res = mir3da_setPowerMode(client, true);
		if (res)
			MI_ERR("Power on mir3da error %d!\n", res);

			msleep(20);
	}

	res = mir3da_read_data(client, &(obj->data[MIR3DA_AXIS_X]),
						   &(obj->data[MIR3DA_AXIS_Y]),
						   &(obj->data[MIR3DA_AXIS_Z]));

	if (res) {
		MI_ERR("I2C error: ret value=%d", res);
		return -3;

	} else {

		if (atomic_read(&obj->trace) & MIR3DA_TRC_RAWDATA) {
			MI_MSG("read_sensor_data map[%d][%d][%d] sign[%d][%d][%d]",
			obj->cvt.map[0], obj->cvt.map[1], obj->cvt.map[2],
			obj->cvt.sign[0], obj->cvt.sign[1], obj->cvt.sign[2]);
			MI_MSG("read_sensor_data xyz_0[%d][%d][%d] cali_sw[%d][%d][%d]",
			   obj->data[0], obj->data[1],
			   obj->data[2], obj->cali_sw[0],
			   obj->cali_sw[1], obj->cali_sw[2]);
		}
		obj->data[MIR3DA_AXIS_X] = (obj->data[MIR3DA_AXIS_X] * GRAVITY_EARTH_1000 / gsensor_gain.x);
		obj->data[MIR3DA_AXIS_Y] = (obj->data[MIR3DA_AXIS_Y] * GRAVITY_EARTH_1000 / gsensor_gain.y);
		obj->data[MIR3DA_AXIS_Z] = (obj->data[MIR3DA_AXIS_Z] * GRAVITY_EARTH_1000 / gsensor_gain.z);

		obj->data[MIR3DA_AXIS_X] += (obj->cvt.sign[MIR3DA_AXIS_X] * obj->cali_sw[obj->cvt.map[MIR3DA_AXIS_X]]);
		obj->data[MIR3DA_AXIS_Y] += (obj->cvt.sign[MIR3DA_AXIS_Y] * obj->cali_sw[obj->cvt.map[MIR3DA_AXIS_Y]]);
		obj->data[MIR3DA_AXIS_Z] += (obj->cvt.sign[MIR3DA_AXIS_Z] * obj->cali_sw[obj->cvt.map[MIR3DA_AXIS_Z]]);

		if (atomic_read(&obj->trace) & MIR3DA_TRC_RAWDATA)
			MI_MSG("read_sensor_data xyz_1[%d][%d][%d]",
			   obj->data[0], obj->data[1], obj->data[2]);

		acc[obj->cvt.map[MIR3DA_AXIS_X]] = (obj->cvt.sign[MIR3DA_AXIS_X] * obj->data[MIR3DA_AXIS_X]);
		acc[obj->cvt.map[MIR3DA_AXIS_Y]] = (obj->cvt.sign[MIR3DA_AXIS_Y] * obj->data[MIR3DA_AXIS_Y]);
		acc[obj->cvt.map[MIR3DA_AXIS_Z]] = (obj->cvt.sign[MIR3DA_AXIS_Z] * obj->data[MIR3DA_AXIS_Z]);

		*pdata_x = acc[MIR3DA_AXIS_X];
		*pdata_y = acc[MIR3DA_AXIS_Y];
		*pdata_z = acc[MIR3DA_AXIS_Z];

		if (atomic_read(&obj->trace) & MIR3DA_TRC_RAWDATA)
			MI_MSG("read_sensor_data xyz_2[%d][%d][%d]",
			   acc[obj->cvt.map[MIR3DA_AXIS_X]],
			   acc[obj->cvt.map[MIR3DA_AXIS_Y]],
			   acc[obj->cvt.map[MIR3DA_AXIS_Z]]);

		if (atomic_read(&obj->trace) & MIR3DA_TRC_RAWDATA)
			MI_DATA("mir3da data map: %d, %d, %d!\n",
				acc[MIR3DA_AXIS_X],
				acc[MIR3DA_AXIS_Y],
				acc[MIR3DA_AXIS_Z]);

		if (atomic_read(&obj->trace) & MIR3DA_TRC_RAWDATA)
			MI_DATA("mir3da data mg: x= %d, y=%d, z=%d\n",
			acc[MIR3DA_AXIS_X],
			acc[MIR3DA_AXIS_Y],
			acc[MIR3DA_AXIS_Z]);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_readRawData(struct i2c_client *client, char *buf)
{
	struct mir3da_i2c_data *obj = (struct mir3da_i2c_data *)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
		return -EINVAL;

	res = mir3da_read_data(client, &(obj->data[MIR3DA_AXIS_X]), &(obj->data[MIR3DA_AXIS_Y]), &(obj->data[MIR3DA_AXIS_Z]));
	if (res) {
		MI_ERR("I2C error: ret value=%d", res);
		return -EIO;
	} else
		sprintf(buf, "%04x %04x %04x", (obj->data[MIR3DA_AXIS_X]), (obj->data[MIR3DA_AXIS_Y]), (obj->data[MIR3DA_AXIS_Z]));


	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t enable_show(struct device_driver *ddri, char *buf)
{
	int ret;
	char bEnable;
	struct i2c_client *client = mir_handle;

	MI_FUN;

	ret = mir3da_get_enable(client, &bEnable);
	if (ret < 0)
		ret = -EINVAL;
	else
		ret = sprintf(buf, "%d\n", bEnable);

	return ret;
}
/*----------------------------------------------------------------------------*/
static ssize_t enable_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	int ret;
	char bEnable;
	unsigned long enable;
	struct i2c_client *client = mir_handle;

	if (buf == NULL)
		return -1;

	enable = simple_strtoul(buf, NULL, 10);
	bEnable = (enable > 0) ? true : false;

	ret = mir3da_set_enable(client, bEnable);
	if (ret < 0)
		ret = -EINVAL;
	else
		ret = count;

	return ret;
}
/*----------------------------------------------------------------------------*/
static ssize_t sensordata_show(struct device_driver *ddri, char *buf)
{
	int result;
	int x, y, z;
	int count = 0;

	result = mir3da_readSensorData(mir_handle, &x, &y, &z);
	if (result == 0)
		count += sprintf(buf+count, "%d %d %d\n", x, y, z);
	else
		count += sprintf(buf+count, "reading failed!");

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t reg_data_show(struct device_driver *ddri, char *buf)
{
	MIR_HANDLE          handle = mir_handle;

	return mir3da_get_reg_data(handle, buf);
}
/*----------------------------------------------------------------------------*/
static ssize_t reg_data_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	int                 addr, data;
	int                 result;

	result = sscanf(buf, "0x%x, 0x%x\n", &addr, &data);
	if (result != 2)
		return count;

	result = mir3da_register_write(mir_handle, addr, data);

	MI_ASSERT(result == 0);

	MI_MSG("set[0x%x]->[0x%x]\n", addr, data);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t log_level_show(struct device_driver *ddri, char *buf)
{
	int ret;

	ret = sprintf(buf, "%d\n", Log_level);

	return ret;
}
/*----------------------------------------------------------------------------*/
static ssize_t log_level_store(struct device_driver *ddri, const char *buf, size_t count)
{
	Log_level = simple_strtoul(buf, NULL, 10);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t primary_offset_show(struct device_driver *ddri, char *buf)
{
	int x = 0, y = 0, z = 0;

	mir3da_get_primary_offset(mir_handle, &x, &y, &z);

	return sprintf(buf, "x=%d ,y=%d ,z=%d\n", x, y, z);
}
/*----------------------------------------------------------------------------*/
static ssize_t version_show(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%s_%s\n", DRI_VER, CORE_VER);
}
/*----------------------------------------------------------------------------*/
static ssize_t vendor_show(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%s\n", "MiraMEMS");
}
/*----------------------------------------------------------------------------*/

static void get_accel_idme_cali(void)
{

	s16 idmedata[CALI_SIZE] = {0};
#ifdef CONFIG_AMZN_IDME
	char *gsensor_cal = NULL;
	gsensor_cal = idme_get_sensorcal();
	if (gsensor_cal == NULL) {
		MI_ERR("idme get sensorcal fail!\n");
		return;
	}
	acc_idme_get_gsensorcal(gsensor_cal, idmedata, CALI_SIZE);
#endif

	accel_xyz_offset[0] = idmedata[0];
	accel_xyz_offset[1] = idmedata[1];
	accel_xyz_offset[2] = idmedata[2];

	MI_MSG("accel_xyz_offset =%d, %d, %d\n", accel_xyz_offset[0], accel_xyz_offset[1], accel_xyz_offset[2]);
}

static ssize_t accelgetidme_show(struct device_driver *ddri, char *buf)
{
	get_accel_idme_cali();
	return scnprintf(buf, PAGE_SIZE,
					"offset_x=%d , offset_y=%d , offset_z=%d\nPass\n",
					accel_xyz_offset[0],
					accel_xyz_offset[1],
					accel_xyz_offset[2]);
}

static int acc_store_offset_in_file(const char *filename, s16 *offset)
{
	struct file *cali_file;
	char w_buf[MIR3DA_DATA_BUF_NUM*sizeof(s16)*2+1] = {0};
	char r_buf[MIR3DA_DATA_BUF_NUM*sizeof(s16)*2+1] = {0};
	int i;
	char *dest = w_buf;
	mm_segment_t fs;

	cali_file = filp_open(filename, O_CREAT | O_RDWR, 0777);
	if (IS_ERR(cali_file)) {
		MI_ERR("open error! exit!\n");
		return -1;
	}
	fs = get_fs();
	set_fs(get_ds());
	for (i = 0; i < MIR3DA_DATA_BUF_NUM; i++) {
		sprintf(dest, "%02X", offset[i]&0x00FF);
		dest += 2;
		sprintf(dest, "%02X", (offset[i] >> 8)&0x00FF);
		dest += 2;
	};
	MI_MSG("w_buf: %s\n", w_buf);
	vfs_write(cali_file, (void *)w_buf,
		MIR3DA_DATA_BUF_NUM*sizeof(s16)*2, &cali_file->f_pos);
	cali_file->f_pos = 0x00;
	vfs_read(cali_file, (void *)r_buf,
		MIR3DA_DATA_BUF_NUM*sizeof(s16)*2, &cali_file->f_pos);
	for (i = 0; i < MIR3DA_DATA_BUF_NUM*sizeof(s16)*2; i++) {
		if (r_buf[i] != w_buf[i]) {
			set_fs(fs);
			filp_close(cali_file, NULL);
			MI_ERR("read back error! exit!\n");
			return -1;
		}
	}
	set_fs(fs);

	filp_close(cali_file, NULL);
	MI_MSG("store_offset_in_file ok exit\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t accelsetselftest_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mir3da_i2c_client;
	int avg[3] = {0}, out_nost[3] = {0};
	int err = -1, num = 0, count = 5;
	int data_x = 0, data_y = 0, data_z = 0;

	if (mir3da_i2c_client == NULL) {
		MI_ERR("i2c_client obj is null!!\n");
		return 0;
	}

	accel_self_test[0] = accel_self_test[1] = accel_self_test[2] = 0;

	err = mir3da_setPowerMode(client, true);
	if (err) {
		MI_ERR("enable gsensor fail: %d\n", err);
		goto MIR3DA_accel_self_test_exit;
	}

	msleep(100);

	while (num < count) {
		msleep(20);

		/* read gsensor data */
		err = mir3da_readSensorData(client, &data_x, &data_y, &data_z);

		if (err) {
			MI_ERR("read data fail: %d\n", err);
			goto MIR3DA_accel_self_test_exit;
		}

		avg[MIR3DA_AXIS_X] = data_x + avg[MIR3DA_AXIS_X];
		avg[MIR3DA_AXIS_Y] = data_y + avg[MIR3DA_AXIS_Y];
		avg[MIR3DA_AXIS_Z] = data_z + avg[MIR3DA_AXIS_Z];

		num++;
	}

	out_nost[0] = avg[MIR3DA_AXIS_X] / count;
	out_nost[1] = avg[MIR3DA_AXIS_Y] / count;
	out_nost[2] = avg[MIR3DA_AXIS_Z] / count;

	accel_self_test[0] = abs(out_nost[0]);
	accel_self_test[1] = abs(out_nost[1]);
	accel_self_test[2] = abs(out_nost[2]);

	/* disable sensor */
	err = mir3da_setPowerMode(client, false);
	if (err < 0)
		goto MIR3DA_accel_self_test_exit;

	return scnprintf(buf, PAGE_SIZE,
					"[G Sensor] set_accel_self_test PASS\n");

MIR3DA_accel_self_test_exit:

	mir3da_setPowerMode(client, false);

	return scnprintf(buf, PAGE_SIZE, "[G Sensor] exit - Fail , err=%d\n", err);

}

static ssize_t accelgetselftest_show(struct device_driver *ddri, char *buf)
{
	if (accel_self_test[0] < ACCEL_SELF_TEST_MIN_VAL ||
			accel_self_test[0] > ACCEL_SELF_TEST_MAX_VAL)
		return sprintf(buf, "X=%d , out of range\nFail\n",
					accel_self_test[0]);

	if (accel_self_test[1] < ACCEL_SELF_TEST_MIN_VAL ||
			accel_self_test[1] > ACCEL_SELF_TEST_MAX_VAL)
		return sprintf(buf, "Y=%d , out of range\nFail\n",
					accel_self_test[1]);

	if (accel_self_test[2] < ACCEL_SELF_TEST_MIN_VAL ||
			accel_self_test[2] > ACCEL_SELF_TEST_MAX_VAL)
		return sprintf(buf, "Z=%d , out of range\nFail\n",
			accel_self_test[2]);
	else
		return sprintf(buf, "%d , %d , %d\nPass\n", accel_self_test[0],
					accel_self_test[1], accel_self_test[2]);
}

static ssize_t accelsetcali_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mir3da_i2c_client;
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);
	int avg[3] = {0};
	int cali[3] = {0};
	int golden_x = 0;
	int golden_y = 0;
	int golden_z = -9800;
	int cali_last[3] = {0};
	int err = -1, num = 0, times = 20;
	int data_x = 0, data_y = 0, data_z = 0;

	if (!obj) {
		MI_ERR("i2c client obj is null!!\n");
		return 0;
	}

	obj->cali_sw[MIR3DA_AXIS_X] = 0;
	obj->cali_sw[MIR3DA_AXIS_Y] = 0;
	obj->cali_sw[MIR3DA_AXIS_Z] = 0;

	while (num < times) {
		msleep(20);
		/* read gsensor data */
		err = mir3da_readSensorData(client, &data_x, &data_y, &data_z);

		if (err) {
			MI_ERR("read data fail: %d\n", err);
			return 0;
		}

		if (data_z > 8500)
			golden_z = 9800;
		else if (data_z < -8500)
			golden_z = -9800;
		else
			return 0;

		avg[MIR3DA_AXIS_X] = data_x + avg[MIR3DA_AXIS_X];
		avg[MIR3DA_AXIS_Y] = data_y + avg[MIR3DA_AXIS_Y];
		avg[MIR3DA_AXIS_Z] = data_z + avg[MIR3DA_AXIS_Z];

		num++;

	}

	avg[MIR3DA_AXIS_X] /= times;
	avg[MIR3DA_AXIS_Y] /= times;
	avg[MIR3DA_AXIS_Z] /= times;

	cali[MIR3DA_AXIS_X] = golden_x - avg[MIR3DA_AXIS_X];
	cali[MIR3DA_AXIS_Y] = golden_y - avg[MIR3DA_AXIS_Y];
	cali[MIR3DA_AXIS_Z] = golden_z - avg[MIR3DA_AXIS_Z];


		if ((abs(cali[MIR3DA_AXIS_X]) >
			abs(accel_cali_tolerance * golden_z / 100))
			|| (abs(cali[MIR3DA_AXIS_Y]) >
			abs(accel_cali_tolerance * golden_z / 100))
			|| (abs(cali[MIR3DA_AXIS_Z]) >
			abs(accel_cali_tolerance * golden_z / 100))) {

			MI_ERR("X/Y/Z out of range  tolerance:	[%d] avg_x:[%d] avg_y:[%d] avg_z:[%d]\n", accel_cali_tolerance, avg[MIR3DA_AXIS_X], avg[MIR3DA_AXIS_Y], avg[MIR3DA_AXIS_Z]);

		return scnprintf(buf, PAGE_SIZE,
						"Please place the Pad to a horizontal level.\ntolerance:[%d] avg_x:[%d] avg_y:[%d] avg_z:[%d]\n",
						accel_cali_tolerance,
						avg[MIR3DA_AXIS_X],
						avg[MIR3DA_AXIS_Y],
						avg[MIR3DA_AXIS_Z]);
	}

	cali_last[0] = cali[MIR3DA_AXIS_X];
	cali_last[1] = cali[MIR3DA_AXIS_Y];
	cali_last[2] = cali[MIR3DA_AXIS_Z];

	mir3da_writeCalibration(client, cali_last);

	cali[MIR3DA_AXIS_X] = obj->cali_sw[MIR3DA_AXIS_X];
	cali[MIR3DA_AXIS_Y] = obj->cali_sw[MIR3DA_AXIS_Y];
	cali[MIR3DA_AXIS_Z] = obj->cali_sw[MIR3DA_AXIS_Z];

	accel_xyz_offset[0] = (s16)cali[MIR3DA_AXIS_X];
	accel_xyz_offset[1] = (s16)cali[MIR3DA_AXIS_Y];
	accel_xyz_offset[2] = (s16)cali[MIR3DA_AXIS_Z];

	if (acc_store_offset_in_file(MIR3DA_ACC_CALI_FILE, accel_xyz_offset)) {
		return scnprintf(buf, PAGE_SIZE,
						"[G Sensor] set_accel_cali ERROR %d, %d, %d\n",
						accel_xyz_offset[0],
						accel_xyz_offset[1],
						accel_xyz_offset[2]);
	}

	return scnprintf(buf, PAGE_SIZE,
					"[G Sensor] set_accel_cali PASS  %d, %d, %d\n",
					accel_xyz_offset[0],
					accel_xyz_offset[1],
					accel_xyz_offset[2]);
}

static ssize_t accelgetcali_show(struct device_driver *ddri, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"x=%d , y=%d , z=%d\nx=0x%04x , y=0x%04x , z=0x%04x\nPass\n",
			accel_xyz_offset[0], accel_xyz_offset[1],
			accel_xyz_offset[2], accel_xyz_offset[0],
			accel_xyz_offset[1], accel_xyz_offset[2]);
}

static ssize_t power_mode_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mir3da_i2c_client;
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);

	if (!obj) {
		MI_ERR("i2c_client is null!!\n");
		return 0;
	}

	MI_MSG("[%s] MIR3DA_sensor_power: %d\n", __func__, atomic_read(&obj->enabled));
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->enabled));
}

static ssize_t power_mode_store(struct device_driver *ddri, const char *buf, size_t tCount)
{
	struct i2c_client *client = mir3da_obj_i2c_data->client;
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);
	bool power_enable = false;
	int power_mode = 0;
	int ret = 0;

	if (!obj) {
		MI_ERR("i2c_client is null!!\n");
		return 0;
	}

	ret = kstrtoint(buf, 10, &power_mode);

	power_enable = (power_mode ? true:false);

	if (ret == 0)
		ret = mir3da_setPowerMode(client, power_enable);

	if (ret) {
		MI_ERR("set power %s failed %d\n",
			(power_enable?"on":"off"), ret);
		return 0;

	} else
		MI_ERR("set power %s ok\n", (atomic_read(&obj->enabled)?"on":"off"));

	return tCount;
}

static ssize_t cali_tolerance_show(struct device_driver *ddri, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "accel_cali_tolerance=%d\n", accel_cali_tolerance);
}

static ssize_t cali_tolerance_store(struct device_driver *ddri, const char *buf, size_t tCount)
{
	int temp_cali_tolerance = 0;
	int ret = 0;

	ret = kstrtoint(buf, 10, &temp_cali_tolerance);

	if (ret == 0) {
		if (temp_cali_tolerance > 100)
			temp_cali_tolerance = 100;
		if (temp_cali_tolerance <= 0)
			temp_cali_tolerance = 1;

		accel_cali_tolerance = temp_cali_tolerance;
	}

	if (ret) {
		MI_ERR("set accel_cali_tolerance failed %d\n", ret);
		return 0;

	} else
		MI_ERR("set accel_cali_tolerance %d ok\n", accel_cali_tolerance);

	return tCount;
}

static ssize_t trace_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct i2c_client *client = mir3da_obj_i2c_data->client;
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);

	if (obj == NULL) {
		MI_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	res = scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t trace_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = mir3da_obj_i2c_data->client;
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);
	int trace;

	if (obj == NULL) {
		MI_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	if (sscanf(buf, "0x%11x", &trace) == 1)
		atomic_set(&obj->trace, trace);
	else
		MI_ERR("invalid content: '%s'\n", buf);

	return count;
}
static DRIVER_ATTR_RW(enable);
static DRIVER_ATTR_RO(sensordata);
static DRIVER_ATTR_RW(reg_data);
static DRIVER_ATTR_RW(log_level);
static DRIVER_ATTR_RO(primary_offset);
static DRIVER_ATTR_RO(vendor);
static DRIVER_ATTR_RO(version);

static DRIVER_ATTR_RO(accelsetselftest);
static DRIVER_ATTR_RO(accelgetselftest);
static DRIVER_ATTR_RO(accelsetcali);
static DRIVER_ATTR_RO(accelgetcali);
static DRIVER_ATTR_RW(power_mode);
static DRIVER_ATTR_RW(cali_tolerance);
static DRIVER_ATTR_RO(accelgetidme);
static DRIVER_ATTR_RW(trace);

static struct driver_attribute *mir3da_attributes[] = {
	&driver_attr_enable,
	&driver_attr_sensordata,
	&driver_attr_reg_data,
	&driver_attr_log_level,
	&driver_attr_primary_offset,
	&driver_attr_vendor,
	&driver_attr_version,
	/*add for diag*/
	&driver_attr_accelsetselftest,
	&driver_attr_accelgetselftest,
	&driver_attr_accelsetcali,
	&driver_attr_accelgetcali,
	&driver_attr_power_mode,
	&driver_attr_cali_tolerance,
	&driver_attr_accelgetidme,
	&driver_attr_trace,
};
/*----------------------------------------------------------------------------*/
static int mir3da_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mir3da_attributes)/sizeof(mir3da_attributes[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, mir3da_attributes[idx]);
		if (err) {
			MI_MSG("driver_create_file (%s) = %d\n",
				   mir3da_attributes[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int mir3da_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mir3da_attributes)/sizeof(mir3da_attributes[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, mir3da_attributes[idx]);

	return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
int i2c_smbus_read(PLAT_HANDLE handle, u8 addr, u8 *data)
{
	int                 res = 0;
	struct i2c_client   *client = (struct i2c_client *)handle;

	*data = i2c_smbus_read_byte_data(client, addr);

	return res;
}
/*----------------------------------------------------------------------------*/
int i2c_smbus_read_block(PLAT_HANDLE handle, u8 addr, u8 count, u8 *data)
{
	int                 res = 0;
	struct i2c_client   *client = (struct i2c_client *)handle;

	res = i2c_smbus_read_i2c_block_data(client, addr, count, data);

	return res;
}
/*----------------------------------------------------------------------------*/
int i2c_smbus_write(PLAT_HANDLE handle, u8 addr, u8 data)
{
	int                 res = 0;
	struct i2c_client   *client = (struct i2c_client *)handle;

	res = i2c_smbus_write_byte_data(client, addr, data);

	return res;
}
/*----------------------------------------------------------------------------*/
void msdelay(int ms)
{
	mdelay(ms);
}
/*----------------------------------------------------------------------------*/
MIR_GENERAL_OPS_DECLARE(ops_handle, i2c_smbus_read, i2c_smbus_read_block, i2c_smbus_write, NULL, NULL, NULL, get_address, NULL, msdelay, printk, sprintf);
/*----------------------------------------------------------------------------*/
static void mir3da_SetGain(void)
{
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 1024;
}
/*----------------------------------------------------------------------------*/
static int mir3da_open_report_data(int open)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_enable_nodata(int en)
{
	int res = 0;
	int retry = 0;
	char bEnable = false;

	MI_ERR("%s 0\n", __func__);

	if (en == 1)
		bEnable = true;
	if (en == 0)
		bEnable = false;

	for (retry = 0; retry < 3; retry++) {
		res = mir3da_setPowerMode(mir3da_i2c_client, bEnable);
		if (res == 0) {
			MI_ERR("mir3da_SetPowerMode done\n");
			break;
		}
		MI_ERR("mir3da_SetPowerMode fail\n");
	}
	if (res != 0) {
		MI_ERR("mir3da_SetPowerMode fail!\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_set_delay(u64 ns)
{
	int value = 0;

	MI_ERR("%s\n", __func__);
	value = (int)ns/1000/1000;
	MI_MSG("mir3daset_delay (%d), chip only use 1024HZ\n", value);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_get_data(int *x, int *y, int *z, int *status)
{

	int ret;

	mir3da_mutex_lock();

	ret = mir3da_readSensorData(mir3da_i2c_client, x, y, z);
	if (ret) {
		MI_MSG("read data fail: %d\n", ret);
		mir3da_mutex_unlock();
		return -1;
	}
	mir3da_mutex_unlock();

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	MI_ERR("%s\n", __func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_flush(void)
{
	int err = 0;

	MI_ERR("%s\n", __func__);
	if (!sensor_power) {
		mir3da_obj_i2c_data->flush = true;
		return 0;
	}

	err = acc_flush_report();
	if (err >= 0)
		mir3da_obj_i2c_data->flush = false;
		MI_ERR("%s\n", __func__);
	return err;
}
/*----------------------------------------------------------------------------*/
static int mir3da_factory_enable_sensor(bool enabledisable,
		int64_t sample_periods_ms)
{
	int err;

	MI_FUN;

	err = mir3da_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		MI_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = mir3da_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		MI_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_factory_get_data(int32_t data[3], int *status)
{
	int ret;

	MI_FUN;

	ret = mir3da_get_data(&data[0], &data[1], &data[2], status);

	MI_MSG("%s %d %d %d\n", __func__, data[0], data[1], data[2]);

	return ret;

}
/*----------------------------------------------------------------------------*/
static int mir3da_factory_get_raw_data(int32_t data[3])
{
	char strbuf[MIR3DA_BUFSIZE] = { 0 };

	MI_FUN;

	mir3da_readRawData(mir3da_i2c_client, strbuf);
	MI_MSG("%s!!!\n", __func__);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_factory_enable_calibration(void)
{
	MI_FUN;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_factory_clear_cali(void)
{
	int err = 0;

	MI_FUN;

	err = mir3da_resetCalibration(mir3da_i2c_client);
	if (err) {
		MI_ERR("mir3da_ResetCalibration failed!\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_factory_set_cali(int32_t data[3])
{
	int err = 0;
	int cali[3] = { 0 };

	MI_FUN;

	MI_MSG("%s ori %d %d %d\n", __func__, data[0], data[1], data[2]);

	cali[MIR3DA_AXIS_X] = data[0] * gsensor_gain.x / GRAVITY_EARTH_1000;
	cali[MIR3DA_AXIS_Y] = data[1] * gsensor_gain.y / GRAVITY_EARTH_1000;
	cali[MIR3DA_AXIS_Z] = data[2] * gsensor_gain.z / GRAVITY_EARTH_1000;

	MI_MSG("%s new %d %d %d", __func__, cali[0], cali[1], cali[2]);

	err = mir3da_writeCalibration(mir3da_i2c_client, cali);
	if (err) {
		MI_ERR("mir3da_WriteCalibration failed!\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_factory_get_cali(int32_t data[3])
{
	data[0] = (mir3da_obj_i2c_data->cali_sw[MIR3DA_AXIS_X]
		* GRAVITY_EARTH_1000 / gsensor_gain.x);
	data[1] = (mir3da_obj_i2c_data->cali_sw[MIR3DA_AXIS_Y]
		* GRAVITY_EARTH_1000 / gsensor_gain.x);
	data[2] = (mir3da_obj_i2c_data->cali_sw[MIR3DA_AXIS_Z]
		* GRAVITY_EARTH_1000 / gsensor_gain.x);


	MI_MSG("%s %d %d %d", __func__, data[0], data[1], data[2]);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_factory_do_self_test(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct accel_factory_fops mir3da_factory_fops = {
	.enable_sensor = mir3da_factory_enable_sensor,
	.get_data = mir3da_factory_get_data,
	.get_raw_data = mir3da_factory_get_raw_data,
	.enable_calibration = mir3da_factory_enable_calibration,
	.clear_cali = mir3da_factory_clear_cali,
	.set_cali = mir3da_factory_set_cali,
	.get_cali = mir3da_factory_get_cali,
	.do_self_test = mir3da_factory_do_self_test,
};

static struct accel_factory_public mir3da_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &mir3da_factory_fops,
};

/*----------------------------------------------------------------------------*/
static struct acc_init_info mir3da_init_info = {
	.name = MIR3DA_DRV_NAME,
	.init = mir3da_local_init,
	.uninit = mir3da_local_remove,
};
/*----------------------------------------------------------------------------*/
static int mir3da_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int result = 0;
	int err = 0;
	struct mir3da_i2c_data *obj = NULL;
	unsigned char chip_id = 0;
	unsigned char i = 0;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};
	s16 idmedata[CALI_SIZE] = {0};
#ifdef CONFIG_AMZN_IDME
	char *gsensor_cal = NULL;
#endif

	MI_FUN;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		MI_ERR("kzalloc failed!");
		result = -ENOMEM;
		goto exit;
	}

	result = get_accel_dts_func(client->dev.of_node, &obj->hw);
	if (result < 0) {
		MI_ERR("get cust_baro dts info fail\n");
		goto exit_kfree_failed;
	}

	result = hwmsen_get_convert(obj->hw.direction, &obj->cvt);
	if (result) {
		MI_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit_kfree_failed;
	}

	obj->client = client;
	i2c_set_clientdata(client, obj);
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

	mir3da_obj_i2c_data = obj;
	mir3da_i2c_client = obj->client;

	mir3da_mutex_init();

	if (mir3da_install_general_ops(&ops_handle)) {
		MI_ERR("Install ops failed !\n");
		goto exit_init_failed;
	}

	i2c_smbus_read((PLAT_HANDLE)mir3da_i2c_client,
				NSA_REG_WHO_AM_I, &chip_id);
	if (chip_id != 0x13) {
		for (i = 0; i < 5 ; i++) {
			msleep(5);
			i2c_smbus_read((PLAT_HANDLE)mir3da_i2c_client,
						   NSA_REG_WHO_AM_I, &chip_id);
			if (chip_id == 0x13)
				break;
		}
		if (chip_id != 0x13) {
			result = -ENOMEM;
			goto exit_init_failed;
		}
	}

	mir_handle = mir3da_core_init((PLAT_HANDLE)mir3da_i2c_client);
	if (mir_handle == NULL) {
		MI_ERR("chip init failed !\n");
		goto exit_init_failed;
	}

	mir3da_SetGain();

	ctl.is_use_common_factory = false;

	result = accel_factory_device_register(&mir3da_factory_device);
	if (result) {
		MI_ERR("acc_factory register failed.\n");
		goto exit_factory_device_register_failed;
	}

	result = mir3da_create_attr(
		&(mir3da_init_info.platform_diver_addr->driver));
	if (result) {
		MI_ERR("create attribute result = %d\n", result);
		result = -EINVAL;
		goto exit_create_attr_failed;
	}

	ctl.open_report_data = mir3da_open_report_data;
	ctl.enable_nodata = mir3da_enable_nodata;
	ctl.batch = mir3da_batch;
	ctl.flush = mir3da_flush;
	ctl.set_delay  = mir3da_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = false;
	result = acc_register_control_path(&ctl);
	if (result) {
		MI_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = mir3da_get_data;
	data.vender_div = 1000;
	result = acc_register_data_path(&data);
	if (result) {
		MI_ERR("register acc data path err= %d\n", result);
		goto exit_kfree;
	}

#ifdef CONFIG_AMZN_IDME
	gsensor_cal = idme_get_sensorcal();
	if (gsensor_cal == NULL) {
		MI_ERR("idme get sensorcal fail!\n");
		goto exit_kfree;
	}
	err = acc_idme_get_gsensorcal(gsensor_cal, idmedata, CALI_SIZE);
	if (err)
		MI_ERR("Get gsensor offset fail,default offset x=y=z=0\n");
#endif

	accel_xyz_offset[0] = idmedata[0];
	accel_xyz_offset[1] = idmedata[1];
	accel_xyz_offset[2] = idmedata[2];
	obj->cali_sw[MIR3DA_AXIS_X] = accel_xyz_offset[0];
	obj->cali_sw[MIR3DA_AXIS_Y] = accel_xyz_offset[1];
	obj->cali_sw[MIR3DA_AXIS_Z] = accel_xyz_offset[2];
	MI_MSG("idme to read:%d %d %d\n",
			accel_xyz_offset[0],
			accel_xyz_offset[1],
			accel_xyz_offset[2]);

	mir3da_init_flag = 0;
	MI_MSG("%s: OK\n", __func__);


	return result;



exit_kfree:
	mir3da_delete_attr(
		&(mir3da_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
	accel_factory_device_deregister(&mir3da_factory_device);
exit_factory_device_register_failed:
exit_init_failed:
exit_kfree_failed:
	kfree(obj);
exit:
	MI_ERR("%s: err = %d\n", __func__, result);
	obj = NULL;
	mir_handle = NULL;
	mir3da_i2c_client = NULL;
	mir3da_obj_i2c_data = NULL;
	mir3da_init_flag = -1;
	return result;
}
/*----------------------------------------------------------------------------*/
static int  mir3da_remove(struct i2c_client *client)
{
	int err = 0;

	err = mir3da_delete_attr(
			  &(mir3da_init_info.platform_diver_addr->driver));
	if (err)
		MI_ERR("mir3da_delete_attr fail: %d\n", err);

	mir3da_i2c_client = NULL;
	i2c_unregister_device(client);
	accel_factory_device_deregister(&mir3da_factory_device);
	kfree(i2c_get_clientdata(client));

	return 0;
}



#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,mir3da"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static int mir3da_i2c_suspend(struct device *dev);
static int mir3da_i2c_resume(struct device *dev);

static const struct dev_pm_ops mir3da_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mir3da_i2c_suspend, mir3da_i2c_resume)
};
#endif



static const struct i2c_device_id mir3da_id[] = {
	{ MIR3DA_DRV_NAME, 0 },
	{ }
};

static struct i2c_driver mir3da_driver = {
	.driver = {
		.name    = MIR3DA_DRV_NAME,
#ifdef CONFIG_OF
		.of_match_table = accel_of_match,
#endif
#ifdef CONFIG_PM_SLEEP
		.pm             = &mir3da_pm_ops,
#endif
	},
	.probe       = mir3da_probe,
	.remove      = mir3da_remove,

	.id_table    = mir3da_id,
};
#ifdef CONFIG_PM_SLEEP

static int mir3da_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	MI_FUN;
	if (obj == NULL) {
		MI_ERR("null pointer!!\n");
		return -EINVAL;
	}
	acc_driver_pause_polling(1);
	atomic_set(&obj->suspend, 1);
	err = mir3da_setPowerMode(client, false);
	if (err) {
		acc_driver_pause_polling(0);
		atomic_set(&obj->suspend, 0);
		MI_ERR("write power control fail!!\n");
		return err;
	}

	return err;
}

static int mir3da_i2c_resume(struct device *dev)
{
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct mir3da_i2c_data *obj = i2c_get_clientdata(client);

	MI_FUN;
	if (obj == NULL) {
		MI_ERR("null pointer!!\n");
		return -EINVAL;
	}

	if (acc_driver_query_polling_state() == 1) {
		err = mir3da_setPowerMode(obj->client, true);
		if (err)
			MI_ERR("initialize client fail!!\n");
	}

	atomic_set(&obj->suspend, 0);
	acc_driver_pause_polling(0);

	return err;
}
#endif
/*----------------------------------------------------------------------------*/
static int mir3da_local_init(void)
{
	MI_FUN;

	if (i2c_add_driver(&mir3da_driver)) {
		MI_ERR("add driver error\n");
		return -1;
	}

	if (-1 == mir3da_init_flag)
		return -1;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int mir3da_local_remove(void)
{
	MI_FUN;

	i2c_del_driver(&mir3da_driver);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init mir3da_init(void)
{
	MI_FUN;

	acc_driver_add(&mir3da_init_info);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit mir3da_exit(void)
{
	MI_FUN;

#ifdef CONFIG_CUSTOM_KERNEL_ACCELEROMETER_MODULE
	success_Flag = false;
#endif
}
/*----------------------------------------------------------------------------*/

module_init(mir3da_init);
module_exit(mir3da_exit);

MODULE_AUTHOR("MiraMEMS <lschen@miramems.com>");
MODULE_DESCRIPTION("MirMEMS 3-Axis Accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

