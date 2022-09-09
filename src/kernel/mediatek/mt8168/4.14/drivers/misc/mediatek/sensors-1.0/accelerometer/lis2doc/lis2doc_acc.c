/********************************************************************
 * Copyright (c) 2021, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the STMicroelectronics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************************
 * REVISON HISTORY
 *
 * VERSION | DATE          | DESCRIPTION
 *
 * 3.0.2   | 06/22/2017    | MTK driver initial version
 * 3.0.3   | 07/05/2017    | fixed ATA test bug
 *
 ********************************************************************************************/


#include "lis2doc.h"

static struct data_resolution lis2doc_acc_data_resolution[] = {
	/* combination by {FULL_RES,RANGE}*/
	{{0, 0}, 244},	/*2g   1LSB=61ug*/
	{{0, 0}, 488},	/*Not vailable*/
	{{0, 0}, 976},	/*4g*/
	{{0, 0}, 1952},	/*8g*/
};

#define LIS2DOC_ACC_CALI_FILE "/data/inv_cal_data.bin"
#define CALI_SIZE 3 /*CALI_SIZE should not less than 3*/
static int accel_cali_tolerance = 20;
static s16 accel_xyz_offset[LIS2DOC_AXES_NUM] = {0};
static int accel_self_test[LIS2DOC_AXES_NUM] = {0};
static int change_caculate;

static struct data_resolution lis2doc_offset_resolution = {{15, 6}, 64};
static struct GSENSOR_VECTOR3D gsensor_gain;
struct acc_hw lis2doc_acc_cust_hw;

/*For driver get cust info*/
struct acc_hw *lis2doc_get_cust_acc_hw(void)
{
	return &lis2doc_acc_cust_hw;
}

static int lis2doc_acc_set_resolution(struct lis2doc_acc *acc_obj)
{
	struct lis2doc_data *obj = container_of(acc_obj, struct lis2doc_data, lis2doc_acc_data);
	struct i2c_client *client = obj->client;
	int ret;
	u8 dat, reso;

	ret = lis2doc_i2c_read_block(client, LIS2DOC_REG_CTRL6, &dat, 0x01);
	if (ret < 0) {
		ST_ERR("write data format fail!!\n");
		return ret;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
	reso = (dat & LIS2DOC_REG_CTRL6_MASK_FS) >> 4;
	if (reso >= 0x3)
		reso = 0x3;

	ST_LOG("LIS2DOC_REG_CTRL4:0x%02x, reso:%d\n", dat, reso);

	if (reso < sizeof(lis2doc_acc_data_resolution)/sizeof(lis2doc_acc_data_resolution[0])) {
		acc_obj->reso = &lis2doc_acc_data_resolution[reso];
		return LIS2DOC_SUCCESS;
	} else {
		return -EINVAL;
	}
}

static int lis2doc_acc_read_rawdata(struct lis2doc_acc *acc_obj, s16 data[LIS2DOC_AXES_NUM])
{
	struct lis2doc_data *obj = container_of(acc_obj, struct lis2doc_data, lis2doc_acc_data);
	struct i2c_client *client = obj->client;
	u8 buf[LIS2DOC_DATA_LEN] = {0};
	int ret = 0;

	if (NULL == client) {
		ret = -EINVAL;
	} else {
		if ((lis2doc_i2c_read_block(client, LIS2DOC_REG_OUT_X_L, buf, LIS2DOC_DATA_LEN)) < 0) {
			ST_ERR("read  G sensor data register err!\n");
			return -1;
		}

		data[LIS2DOC_AXIS_X] = ((s16)((buf[LIS2DOC_AXIS_X*2+1] << 8) | buf[LIS2DOC_AXIS_X*2])>>2);
		data[LIS2DOC_AXIS_Y] = ((s16)((buf[LIS2DOC_AXIS_Y*2+1] << 8) | buf[LIS2DOC_AXIS_Y*2])>>2);
		data[LIS2DOC_AXIS_Z] = ((s16)((buf[LIS2DOC_AXIS_Z*2+1] << 8) | buf[LIS2DOC_AXIS_Z*2])>>2);

		if (atomic_read(&acc_obj->trace) & ADX_TRC_RAWDATA) {
			ST_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LIS2DOC_AXIS_X], data[LIS2DOC_AXIS_Y], data[LIS2DOC_AXIS_Z],
				data[LIS2DOC_AXIS_X], data[LIS2DOC_AXIS_Y], data[LIS2DOC_AXIS_Z]);
		}

		if (atomic_read(&acc_obj->trace) & ADX_TRC_RAWDATA) {
			ST_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[LIS2DOC_AXIS_X], data[LIS2DOC_AXIS_Y], data[LIS2DOC_AXIS_Z],
				data[LIS2DOC_AXIS_X], data[LIS2DOC_AXIS_Y], data[LIS2DOC_AXIS_Z]);
		}
	}

	return ret;
}

static int lis2doc_acc_reset_calibration(struct lis2doc_acc *acc_obj)
{
	memset(acc_obj->cali_sw, 0x00, sizeof(acc_obj->cali_sw));
	return LIS2DOC_SUCCESS;
}

static int lis2doc_acc_read_calibration(struct lis2doc_acc *acc_obj, int dat[LIS2DOC_AXES_NUM])
{
	dat[acc_obj->cvt.map[LIS2DOC_AXIS_X]] = acc_obj->cvt.sign[LIS2DOC_AXIS_X]*acc_obj->cali_sw[LIS2DOC_AXIS_X];
	dat[acc_obj->cvt.map[LIS2DOC_AXIS_Y]] = acc_obj->cvt.sign[LIS2DOC_AXIS_Y]*acc_obj->cali_sw[LIS2DOC_AXIS_Y];
	dat[acc_obj->cvt.map[LIS2DOC_AXIS_Z]] = acc_obj->cvt.sign[LIS2DOC_AXIS_Z]*acc_obj->cali_sw[LIS2DOC_AXIS_Z];

	return LIS2DOC_SUCCESS;
}

static int lis2doc_acc_write_calibration(struct lis2doc_acc *acc_obj, int dat[LIS2DOC_AXES_NUM])
{
	ST_FUN();

	if (!acc_obj || !dat) {
		ST_ERR("null ptr!!\n");
		return -EINVAL;
	} else {
		s16 cali[LIS2DOC_AXES_NUM];

		cali[acc_obj->cvt.map[LIS2DOC_AXIS_X]] = acc_obj->cvt.sign[LIS2DOC_AXIS_X]*acc_obj->cali_sw[LIS2DOC_AXIS_X];
		cali[acc_obj->cvt.map[LIS2DOC_AXIS_Y]] = acc_obj->cvt.sign[LIS2DOC_AXIS_Y]*acc_obj->cali_sw[LIS2DOC_AXIS_Y];
		cali[acc_obj->cvt.map[LIS2DOC_AXIS_Z]] = acc_obj->cvt.sign[LIS2DOC_AXIS_Z]*acc_obj->cali_sw[LIS2DOC_AXIS_Z];

		cali[LIS2DOC_AXIS_X] += dat[LIS2DOC_AXIS_X];
		cali[LIS2DOC_AXIS_Y] += dat[LIS2DOC_AXIS_Y];
		cali[LIS2DOC_AXIS_Z] += dat[LIS2DOC_AXIS_Z];

		acc_obj->cali_sw[LIS2DOC_AXIS_X] = acc_obj->cvt.sign[LIS2DOC_AXIS_X]*cali[acc_obj->cvt.map[LIS2DOC_AXIS_X]];
		acc_obj->cali_sw[LIS2DOC_AXIS_Y] = acc_obj->cvt.sign[LIS2DOC_AXIS_Y]*cali[acc_obj->cvt.map[LIS2DOC_AXIS_Y]];
		acc_obj->cali_sw[LIS2DOC_AXIS_Z] = acc_obj->cvt.sign[LIS2DOC_AXIS_Z]*cali[acc_obj->cvt.map[LIS2DOC_AXIS_Z]];
	}

	return 0;
}

static int lis2doc_acc_set_full_scale(struct lis2doc_acc *acc_obj, u8 dataformat)
{
	struct lis2doc_data *obj = container_of(acc_obj, struct lis2doc_data, lis2doc_acc_data);
	struct i2c_client *client = obj->client;
	int ret = 0;

	ret = lis2doc_i2c_write_with_mask(client, LIS2DOC_REG_CTRL6, LIS2DOC_REG_CTRL6_MASK_FS, dataformat);
	if (ret < 0) {
		ST_ERR("read LIS2DOC_REG_CTRL1 register err!\n");
		return LIS2DOC_ERR_I2C;
	}

	return lis2doc_acc_set_resolution(acc_obj);
}

static int lis2doc_acc_set_odr(struct lis2doc_acc *acc_obj, u8 odr)
{
	struct lis2doc_data *obj = container_of(acc_obj, struct lis2doc_data, lis2doc_acc_data);
	struct i2c_client *client = obj->client;
	int ret = 0;

	ret = lis2doc_i2c_write_with_mask(client, LIS2DOC_REG_CTRL1, LIS2DOC_REG_CTRL1_MASK_ODR, odr);
	if (ret < 0)
		return LIS2DOC_ERR_I2C;

	return LIS2DOC_SUCCESS;
}

int lis2doc_acc_set_power_mode(struct lis2doc_acc *acc_obj, bool state)
{
	int ret = 0;

	if (state == acc_obj->lis2doc_acc_power) {
		ST_LOG("Sensor power status is newest!\n");
		return LIS2DOC_SUCCESS;
	}

	if (state == true) {
		if (acc_obj->odr == 0)
			acc_obj->odr = LIS2DOC_REG_CTRL1_ODR_100HZ;

		ret = lis2doc_acc_set_odr(acc_obj, acc_obj->odr);
	} else if (state == false) {
		ret = lis2doc_acc_set_odr(acc_obj, LIS2DOC_REG_CTRL1_ODR_0HZ);
	} else {
		ST_ERR("set power state error!\n");
		return LIS2DOC_ERR_SETUP_FAILURE;
	}

	if (ret < 0) {
		ST_ERR("set power mode failed!\n");
		return LIS2DOC_ERR_I2C;
	} else if (atomic_read(&acc_obj->trace) & ADX_TRC_INFO) {
		ST_LOG("set power mode ok %d!\n", state);
	}

	acc_obj->lis2doc_acc_power = state;

	atomic_set(&acc_obj->enabled, state);

	return LIS2DOC_SUCCESS;
}

int lis2doc_acc_init(struct lis2doc_acc *acc_obj, int reset_cali)
{
	struct lis2doc_data *obj = container_of(acc_obj, struct lis2doc_data, lis2doc_acc_data);
	struct i2c_client *client = obj->client;
	int ret = 0;
	u8 buf[2] = {0, 0};

	ST_FUN();

	/*High performance mode 0000,0110*/
	buf[0] = 0x06;
	ret = lis2doc_i2c_write_block(client, LIS2DOC_REG_CTRL1, buf, 0x01);
	if (ret < 0) {
		ST_ERR("LIS2DOC_REG_CTRL1 step 1!\n");
		return ret;
	}

	/*BDU+IF_ADD_INC 0000,1100*/
	buf[0] = 0x0c;
	ret = lis2doc_i2c_write_block(client, LIS2DOC_REG_CTRL2, buf, 0x01);
	if (ret < 0) {
		ST_ERR("LIS2DOC_REG_CTRL1 step 1!\n");
		return ret;
	}

	/*set FS as 4g*/
	ret = lis2doc_acc_set_full_scale(acc_obj, LIS2DOC_REG_CTRL1_FS_4G);
	if (ret < 0) {
		ST_ERR("%s 3!\n", __func__);
		return ret;
	}

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = acc_obj->reso->sensitivity;

	if (0 != reset_cali) {
		/*reset calibration only in power on*/
		ret = lis2doc_acc_reset_calibration(acc_obj);
		if (ret < 0)
			return ret;
	}

	return LIS2DOC_SUCCESS;
}

static int lis2doc_acc_read_chip_name(struct lis2doc_acc *acc_obj, u8 *buf, int bufsize)
{
	sprintf(buf, "%s", acc_obj->name);
	return LIS2DOC_SUCCESS;
}

static int lis2doc_acc_read_data(struct lis2doc_acc *acc_obj, int *pdata_x, int *pdata_y, int *pdata_z)
{
	struct lis2doc_data *obj = container_of(acc_obj, struct lis2doc_data, lis2doc_acc_data);
	struct i2c_client *client = obj->client;
	int acc[LIS2DOC_AXES_NUM];
	int ret = 0;

	if (NULL == client) {
		*pdata_x = 0;
		*pdata_y = 0;
		*pdata_z = 0;
		return LIS2DOC_ERR_SETUP_FAILURE;
	}

	if (atomic_read(&acc_obj->suspend)) {
		ST_LOG("sensor in suspend read not data!\n");
		return LIS2DOC_SUCCESS;
	}

	if (atomic_read(&acc_obj->enabled) == false) {
		ret = lis2doc_acc_set_power_mode(acc_obj, true);
		if (ret)
			ST_ERR("Power on lis2doc error %d!\n", ret);
	}

	ret = lis2doc_acc_read_rawdata(acc_obj, acc_obj->data);
	if (ret) {
		ST_ERR("I2C error: ret value=%d", ret);
		return ret;
	} else {
		if (change_caculate == 1) {
			acc_obj->data[LIS2DOC_AXIS_X] = (acc_obj->data[LIS2DOC_AXIS_X] * acc_obj->reso->sensitivity / 1000) * GRAVITY_EARTH_1000 / 1000;
			acc_obj->data[LIS2DOC_AXIS_Y] = (acc_obj->data[LIS2DOC_AXIS_Y] * acc_obj->reso->sensitivity / 1000) * GRAVITY_EARTH_1000 / 1000;
			acc_obj->data[LIS2DOC_AXIS_Z] = (acc_obj->data[LIS2DOC_AXIS_Z] * acc_obj->reso->sensitivity / 1000) * GRAVITY_EARTH_1000 / 1000;
		} else {
			acc_obj->data[LIS2DOC_AXIS_X] = acc_obj->data[LIS2DOC_AXIS_X] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
			acc_obj->data[LIS2DOC_AXIS_Y] = acc_obj->data[LIS2DOC_AXIS_Y] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
			acc_obj->data[LIS2DOC_AXIS_Z] = acc_obj->data[LIS2DOC_AXIS_Z] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
		}

		acc_obj->data[LIS2DOC_AXIS_X] += acc_obj->cali_sw[LIS2DOC_AXIS_X];
		acc_obj->data[LIS2DOC_AXIS_Y] += acc_obj->cali_sw[LIS2DOC_AXIS_Y];
		acc_obj->data[LIS2DOC_AXIS_Z] += acc_obj->cali_sw[LIS2DOC_AXIS_Z];

		/*remap coordinate*/
		acc[acc_obj->cvt.map[LIS2DOC_AXIS_X]] = acc_obj->cvt.sign[LIS2DOC_AXIS_X]*acc_obj->data[LIS2DOC_AXIS_X];
		acc[acc_obj->cvt.map[LIS2DOC_AXIS_Y]] = acc_obj->cvt.sign[LIS2DOC_AXIS_Y]*acc_obj->data[LIS2DOC_AXIS_Y];
		acc[acc_obj->cvt.map[LIS2DOC_AXIS_Z]] = acc_obj->cvt.sign[LIS2DOC_AXIS_Z]*acc_obj->data[LIS2DOC_AXIS_Z];


		*pdata_x = acc[LIS2DOC_AXIS_X];
		*pdata_y = acc[LIS2DOC_AXIS_Y];
		*pdata_z = acc[LIS2DOC_AXIS_Z];
		if (atomic_read(&acc_obj->trace) & ADX_TRC_IOCTL) {
			ST_LOG("gsensor data: x = %d, y = %d, z = %d\n", *pdata_x, *pdata_y, *pdata_z);
			dumpReg(obj);
		}
	}

	return LIS2DOC_SUCCESS;
}

static int lis2doc_acc_read_rawdata_string(struct lis2doc_acc *acc_obj, u8 *buf)
{
	struct lis2doc_data *obj = container_of(acc_obj, struct lis2doc_data, lis2doc_acc_data);
	struct i2c_client *client = obj->client;
	int ret = 0;

	if (!buf || !client)
		return LIS2DOC_ERR_SETUP_FAILURE;

	ret = lis2doc_acc_read_rawdata(acc_obj, acc_obj->data);
	if (ret) {
		ST_ERR("I2C error: ret value=%d", ret);
		return ret;
	} else {
		sprintf(buf, "%04x %04x %04x", acc_obj->data[LIS2DOC_AXIS_X],
		acc_obj->data[LIS2DOC_AXIS_Y], acc_obj->data[LIS2DOC_AXIS_Z]);
	}

	return LIS2DOC_SUCCESS;
}

static ssize_t chipinfo_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	u8 strbuf[LIS2DOC_BUFSIZE];

	if (NULL == obj->client) {
		ST_ERR("i2c client is null!!\n");
		return LIS2DOC_SUCCESS;
	}

	lis2doc_acc_read_chip_name(acc_obj, strbuf, LIS2DOC_BUFSIZE);

	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t chipid_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;

	if (NULL == obj->client) {
		ST_ERR("i2c client is null!!\n");
		return LIS2DOC_SUCCESS;
	}

	return snprintf(buf, PAGE_SIZE, "0x%x\n", obj->chip_id);
}

static ssize_t sensordata_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	int x = 0, y = 0, z = 0;
	char strbuf[LIS2DOC_BUFSIZE];

	if (NULL == obj->client) {
		ST_ERR("i2c client is null!!\n");
		return LIS2DOC_SUCCESS;
	}

	lis2doc_acc_read_data(acc_obj, &x, &y, &z);

	scnprintf(strbuf, LIS2DOC_BUFSIZE, "%d %d %d", x, y, z);

	return scnprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t rawdata_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	u8 databuf[LIS2DOC_BUFSIZE];

	if (NULL == obj->client) {
		ST_ERR("i2c client is null!!\n");
		return LIS2DOC_SUCCESS;
	}

	lis2doc_acc_read_rawdata_string(acc_obj, databuf);

	return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}

static ssize_t cali_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	int ret, len = 0, mul;
	int tmp[LIS2DOC_AXES_NUM];

	if (NULL == obj->client) {
		ST_ERR("i2c client is null!!\n");
		return 0;
	}

	ret = lis2doc_acc_read_calibration(acc_obj, tmp);
	if (ret) {
		return -EINVAL;
	} else {
		mul = acc_obj->reso->sensitivity/lis2doc_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n",
			mul, acc_obj->offset[LIS2DOC_AXIS_X], acc_obj->offset[LIS2DOC_AXIS_Y], acc_obj->offset[LIS2DOC_AXIS_Z],
			acc_obj->offset[LIS2DOC_AXIS_X], acc_obj->offset[LIS2DOC_AXIS_Y], acc_obj->offset[LIS2DOC_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
			acc_obj->cali_sw[LIS2DOC_AXIS_X], acc_obj->cali_sw[LIS2DOC_AXIS_Y], acc_obj->cali_sw[LIS2DOC_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
			acc_obj->offset[LIS2DOC_AXIS_X]*mul + acc_obj->cali_sw[LIS2DOC_AXIS_X],
			acc_obj->offset[LIS2DOC_AXIS_Y]*mul + acc_obj->cali_sw[LIS2DOC_AXIS_Y],
			acc_obj->offset[LIS2DOC_AXIS_Z]*mul + acc_obj->cali_sw[LIS2DOC_AXIS_Z],
			tmp[LIS2DOC_AXIS_X], tmp[LIS2DOC_AXIS_Y], tmp[LIS2DOC_AXIS_Z]);

		return len;
	}
}

static ssize_t cali_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	int ret, x, y, z;
	int dat[LIS2DOC_AXES_NUM];

	if (!strncmp(buf, "rst", 3)) {
		ret = lis2doc_acc_reset_calibration(acc_obj);
		if (ret)
			ST_ERR("reset offset err = %d\n", ret);
	} else if (3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
		dat[LIS2DOC_AXIS_X] = x;
		dat[LIS2DOC_AXIS_Y] = y;
		dat[LIS2DOC_AXIS_Z] = z;
		ret = lis2doc_acc_write_calibration(acc_obj, dat);
		if (ret)
			ST_ERR("write calibration err = %d\n", ret);
	} else {
		ST_ERR("invalid format\n");
	}

	return count;
}

static ssize_t power_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct i2c_client *client = obj->client;
	u8 data;

	if (NULL == obj->client) {
		ST_ERR("i2c client is null!!\n");
		return 0;
	}

	lis2doc_i2c_read_block(client, LIS2DOC_REG_CTRL1, &data, 0x01);

	return snprintf(buf, PAGE_SIZE, "%x\n", data);
}

static ssize_t firlen_show(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "not support\n");

}

static ssize_t firlen_store(struct device_driver *ddri, const char *buf, size_t count)
{
	return count;
}

static ssize_t trace_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	ssize_t ret;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	ret = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&acc_obj->trace));

	return ret;
}

static ssize_t trace_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	int trace;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&acc_obj->trace, trace);
	else
		ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}

static ssize_t status_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	ssize_t len = 0;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (acc_obj->lis2doc_acc_hw) {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n",
			acc_obj->lis2doc_acc_hw->i2c_num, acc_obj->lis2doc_acc_hw->direction, acc_obj->reso->sensitivity,
			acc_obj->lis2doc_acc_hw->power_id, acc_obj->lis2doc_acc_hw->power_vol);
		dumpReg(obj);
	} else {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	return len;
}

static ssize_t chipinit_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	ssize_t ret;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	ret = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&acc_obj->trace));

	return ret;
}

static ssize_t chipinit_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return count;
	}

	lis2doc_acc_init(acc_obj, 0);

	return count;
}

static ssize_t layout_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;

	if (NULL == obj) {
		ST_LOG("lis2doc_acc is null!!\n");
		return -1;
	}

	return sprintf(buf, "(%d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
			acc_obj->lis2doc_acc_hw->direction, acc_obj->cvt.sign[0], acc_obj->cvt.sign[1],
			acc_obj->cvt.sign[2], acc_obj->cvt.map[0], acc_obj->cvt.map[1], acc_obj->cvt.map[2]);
}

static ssize_t layout_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	int layout = 0;

	if (NULL == obj) {
		ST_ERR("lis2doc_acc is null!!\n");
		return count;
	}

	if (1 == sscanf(buf, "%d", &layout)) {
		if (!hwmsen_get_convert(layout, &acc_obj->cvt)) {
			ST_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(acc_obj->lis2doc_acc_hw->direction, &acc_obj->cvt)) {
			ST_LOG("invalid layout: %d, restore to %d\n", layout, acc_obj->lis2doc_acc_hw->direction);
		} else {
			ST_ERR("invalid layout: (%d, %d)\n", layout, acc_obj->lis2doc_acc_hw->direction);
			hwmsen_get_convert(0, &acc_obj->cvt);
		}
	} else {
		ST_LOG("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/

static void lis2doc_get_accel_idme_cali(void)
{

	s16 idmedata[CALI_SIZE] = {0};
#ifdef CONFIG_AMZN_IDME
	char *gsensor_cal = NULL;
	gsensor_cal = idme_get_sensorcal();
	if (gsensor_cal == NULL) {
		ST_ERR("idme get sensorcal fail!\n");
		return;
	}
	acc_idme_get_gsensorcal(gsensor_cal, idmedata, CALI_SIZE);
#endif

	accel_xyz_offset[0] = idmedata[0];
	accel_xyz_offset[1] = idmedata[1];
	accel_xyz_offset[2] = idmedata[2];

	ST_LOG("accel_xyz_offset =%d, %d, %d\n", accel_xyz_offset[0],
			accel_xyz_offset[1], accel_xyz_offset[2]);
}

static ssize_t accelgetidme_show(struct device_driver *ddri, char *buf)
{
	lis2doc_get_accel_idme_cali();
	return scnprintf(buf, PAGE_SIZE,
			"offset_x=%d , offset_y=%d , offset_z=%d\nPass\n",
			accel_xyz_offset[0],
			accel_xyz_offset[1],
			accel_xyz_offset[2]);
}

static int acc_store_offset_in_file(const char *filename, s16 *offset)
{
	struct file *cali_file;
	char w_buf[LIS2DOC_DATA_BUF_NUM*sizeof(s16)*2+1] = {0};
	char r_buf[LIS2DOC_DATA_BUF_NUM*sizeof(s16)*2+1] = {0};
	int i;
	char *dest = w_buf;
	mm_segment_t fs;

	cali_file = filp_open(filename, O_CREAT | O_RDWR, 0777);
	if (IS_ERR(cali_file)) {
		ST_ERR("open error! exit!\n");
		return -1;
	}
	fs = get_fs();
	set_fs(get_ds());
	for (i = 0; i < LIS2DOC_DATA_BUF_NUM; i++) {
		sprintf(dest, "%02X", offset[i]&0x00FF);
		dest += 2;
		sprintf(dest, "%02X", (offset[i] >> 8)&0x00FF);
		dest += 2;
	};
	ST_LOG("w_buf: %s\n", w_buf);
	vfs_write(cali_file, (void *)w_buf, LIS2DOC_DATA_BUF_NUM*sizeof(s16)*2, &cali_file->f_pos);
	cali_file->f_pos = 0x00;
	vfs_read(cali_file, (void *)r_buf, LIS2DOC_DATA_BUF_NUM*sizeof(s16)*2, &cali_file->f_pos);
	for (i = 0; i < LIS2DOC_DATA_BUF_NUM*sizeof(s16)*2; i++) {
		if (r_buf[i] != w_buf[i]) {
			set_fs(fs);
			filp_close(cali_file, NULL);
			ST_ERR("read back error! exit!\n");
			return -1;
		}
	}
	set_fs(fs);

	filp_close(cali_file, NULL);
	ST_LOG("store_offset_in_file ok exit\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t accelsetselftest_show(struct device_driver *ddri, char *buf)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;

	int avg[3] = {0}, out_nost[3] = {0};
	int err = -1, num = 0, count = 5;
	int data_x = 0, data_y = 0, data_z = 0;

	if (obj->client == NULL) {
		ST_ERR("i2c_client obj is null!!\n");
		return 0;
	}

	accel_self_test[0] = accel_self_test[1] = accel_self_test[2] = 0;

	err = lis2doc_acc_set_power_mode(acc_obj, true);
	if (err) {
		ST_ERR("enable gsensor fail: %d\n", err);
		goto LIS2DOC_accel_self_test_exit;
	}

	msleep(100);

	while (num < count) {
		msleep(20);

		/* read gsensor data */
		err = lis2doc_acc_read_data(acc_obj, &data_x, &data_y, &data_z);

		if (err) {
			ST_ERR("read data fail: %d\n", err);
			goto LIS2DOC_accel_self_test_exit;
		}

		avg[LIS2DOC_AXIS_X] = data_x + avg[LIS2DOC_AXIS_X];
		avg[LIS2DOC_AXIS_Y] = data_y + avg[LIS2DOC_AXIS_Y];
		avg[LIS2DOC_AXIS_Z] = data_z + avg[LIS2DOC_AXIS_Z];

		num++;
	}

	out_nost[0] = avg[LIS2DOC_AXIS_X] / count;
	out_nost[1] = avg[LIS2DOC_AXIS_Y] / count;
	out_nost[2] = avg[LIS2DOC_AXIS_Z] / count;

	accel_self_test[0] = abs(out_nost[0]);
	accel_self_test[1] = abs(out_nost[1]);
	accel_self_test[2] = abs(out_nost[2]);

	/* disable sensor */
	err = lis2doc_acc_set_power_mode(acc_obj, false);
	if (err < 0)
		goto LIS2DOC_accel_self_test_exit;

	return scnprintf(buf, PAGE_SIZE,
			"[G Sensor] set_accel_self_test PASS\n");

LIS2DOC_accel_self_test_exit:

	lis2doc_acc_set_power_mode(acc_obj, false);

	return scnprintf(buf, PAGE_SIZE,
			"[G Sensor] exit - Fail , err=%d\n", err);

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
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;

	int avg[3] = {0};
	int cali[3] = {0};
	int golden_x = 0;
	int golden_y = 0;
	int golden_z = -9800;
	int cali_last[3] = {0};
	int err = -1, num = 0, times = 20;
	int data_x = 0, data_y = 0, data_z = 0;

	if (!acc_obj) {
		ST_ERR("i2c client obj is null!!\n");
		return 0;
	}

	acc_obj->cali_sw[LIS2DOC_AXIS_X] = 0;
	acc_obj->cali_sw[LIS2DOC_AXIS_Y] = 0;
	acc_obj->cali_sw[LIS2DOC_AXIS_Z] = 0;

	while (num < times) {
		msleep(20);
		/* read gsensor data */
		err = lis2doc_acc_read_data(acc_obj, &data_x, &data_y, &data_z);
		if (err) {
			ST_ERR("read data fail: %d\n", err);
			return 0;
		}

		if (data_z > 8500)
			golden_z = 9800;
		else if (data_z < -8500)
			golden_z = -9800;
		else
			return 0;

		avg[LIS2DOC_AXIS_X] = data_x + avg[LIS2DOC_AXIS_X];
		avg[LIS2DOC_AXIS_Y] = data_y + avg[LIS2DOC_AXIS_Y];
		avg[LIS2DOC_AXIS_Z] = data_z + avg[LIS2DOC_AXIS_Z];

		num++;

	}

	avg[LIS2DOC_AXIS_X] /= times;
	avg[LIS2DOC_AXIS_Y] /= times;
	avg[LIS2DOC_AXIS_Z] /= times;

	cali[LIS2DOC_AXIS_X] = golden_x - avg[LIS2DOC_AXIS_X];
	cali[LIS2DOC_AXIS_Y] = golden_y - avg[LIS2DOC_AXIS_Y];
	cali[LIS2DOC_AXIS_Z] = golden_z - avg[LIS2DOC_AXIS_Z];


	if ((abs(cali[LIS2DOC_AXIS_X]) >
		abs(accel_cali_tolerance * golden_z / 100))
		|| (abs(cali[LIS2DOC_AXIS_Y]) >
		abs(accel_cali_tolerance * golden_z / 100))
		|| (abs(cali[LIS2DOC_AXIS_Z]) >
		abs(accel_cali_tolerance * golden_z / 100))) {

		ST_ERR("X/Y/Z out of range  tolerance:[%d] avg_x:[%d] avg_y:[%d] avg_z:[%d]\n",
				accel_cali_tolerance,
				avg[LIS2DOC_AXIS_X],
				avg[LIS2DOC_AXIS_Y],
				avg[LIS2DOC_AXIS_Z]);

		return scnprintf(buf, PAGE_SIZE,
				"Please place the Pad to a horizontal level.\ntolerance:[%d] avg_x:[%d] avg_y:[%d] avg_z:[%d]\n",
				accel_cali_tolerance,
				avg[LIS2DOC_AXIS_X],
				avg[LIS2DOC_AXIS_Y],
				avg[LIS2DOC_AXIS_Z]);
	}

	cali_last[0] = cali[LIS2DOC_AXIS_X];
	cali_last[1] = cali[LIS2DOC_AXIS_Y];
	cali_last[2] = cali[LIS2DOC_AXIS_Z];

	lis2doc_acc_write_calibration(acc_obj, cali_last);

	cali[LIS2DOC_AXIS_X] = acc_obj->cali_sw[LIS2DOC_AXIS_X];
	cali[LIS2DOC_AXIS_Y] = acc_obj->cali_sw[LIS2DOC_AXIS_Y];
	cali[LIS2DOC_AXIS_Z] = acc_obj->cali_sw[LIS2DOC_AXIS_Z];


	accel_xyz_offset[0] = (s16)cali[LIS2DOC_AXIS_X];
	accel_xyz_offset[1] = (s16)cali[LIS2DOC_AXIS_Y];
	accel_xyz_offset[2] = (s16)cali[LIS2DOC_AXIS_Z];

	if (acc_store_offset_in_file(LIS2DOC_ACC_CALI_FILE, accel_xyz_offset)) {
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
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;

	if (!obj) {
		ST_ERR("i2c_client is null!!\n");
		return 0;
	}

	ST_LOG("[%s] LIS2DOC_sensor_power: %d\n", __func__, atomic_read(&acc_obj->enabled));
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&acc_obj->enabled));
}

static ssize_t power_mode_store(struct device_driver *ddri, const char *buf, size_t tCount)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	bool power_enable = false;
	int power_mode = 0;
	int ret = 0;

	if (!obj) {
		ST_ERR("i2c_client is null!!\n");
		return 0;
	}

	ret = kstrtoint(buf, 10, &power_mode);

	power_enable = (power_mode ? true:false);

	if (0 == ret)
		ret = lis2doc_acc_set_power_mode(acc_obj, power_enable);

	if (ret) {
		ST_ERR("set power %s failed %d\n", (power_enable?"on":"off"), ret);
		return 0;
	} else
		ST_ERR("set power %s ok\n", (atomic_read(&acc_obj->enabled)?"on":"off"));

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
		ST_ERR("set accel_cali_tolerance failed %d\n", ret);
		return 0;
	} else
		ST_ERR("set accel_cali_tolerance %d ok\n", accel_cali_tolerance);

	return tCount;
}
/*----------------------------------------------------------------------------*/

static DRIVER_ATTR_RO(chipinfo);
static DRIVER_ATTR_RO(chipid);
static DRIVER_ATTR_RO(sensordata);
static DRIVER_ATTR_RO(rawdata);
static DRIVER_ATTR_RW(cali);
static DRIVER_ATTR_RO(power);
static DRIVER_ATTR_RW(firlen);
static DRIVER_ATTR_RW(trace);
static DRIVER_ATTR_RO(status);
static DRIVER_ATTR_RW(chipinit);
static DRIVER_ATTR_RW(layout);
/* add for diag */
static DRIVER_ATTR_RO(accelsetselftest);
static DRIVER_ATTR_RO(accelgetselftest);
static DRIVER_ATTR_RO(accelsetcali);
static DRIVER_ATTR_RO(accelgetcali);
static DRIVER_ATTR_RW(power_mode);
static DRIVER_ATTR_RW(cali_tolerance);
static DRIVER_ATTR_RO(accelgetidme);


static struct driver_attribute *lis2doc_attr_acc_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_chipid,       /*chip id*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_rawdata,      /*dump sensor raw data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_power,        /*show power reg*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_chipinit,
	&driver_attr_layout,
	/*add for diag*/
	&driver_attr_accelsetselftest,
	&driver_attr_accelgetselftest,
	&driver_attr_accelsetcali,
	&driver_attr_accelgetcali,
	&driver_attr_power_mode,
	&driver_attr_cali_tolerance,
	&driver_attr_accelgetidme,
};

int lis2doc_acc_create_attr(struct device_driver *driver)
{
	int idx, ret = 0;
	int num = (int)(sizeof(lis2doc_attr_acc_list)/sizeof(lis2doc_attr_acc_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		ret = driver_create_file(driver, lis2doc_attr_acc_list[idx]);
		if (ret) {
			ST_ERR("driver_create_file (%s) = %d\n", lis2doc_attr_acc_list[idx]->attr.name, ret);
			break;
		}
	}

	return ret;
}

int lis2doc_acc_delete_attr(struct device_driver *driver)
{
	int idx, ret = 0;
	int num = (int)(sizeof(lis2doc_attr_acc_list)/sizeof(lis2doc_attr_acc_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, lis2doc_attr_acc_list[idx]);

	return ret;
}

static int lis2doc_acc_open_report_data_intf(int open)
{
	return LIS2DOC_SUCCESS;
}

/*if use this typ of enable, Gsensor only enabled but not report inputEvent to HAL*/
static int lis2doc_acc_enable_nodata_intf(int en)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	int ret = 0;
	bool power = false;

	if (en == 1)
		power = true;
	else if (en == 0)
		power = false;


	ret = lis2doc_acc_set_power_mode(acc_obj, power);
	if (ret != LIS2DOC_SUCCESS) {
		ST_ERR("lis2doc_acc_set_power_mode fail!\n");
		return ret;
	}

	return LIS2DOC_SUCCESS;
}

static int lis2doc_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs/1000/1000;
	ST_LOG("%s (%d), chip only use 1024HZ\n", __func__, value);
	return 0;
}

static int lis2doc_flush(void)
{
	return acc_flush_report();
}

static int lis2doc_acc_set_delay_intf(u64 ns)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	int ms = 0;
	int odr = 0;
	int ret;

	ms = (int)ns/1000/1000;
	if (ms <= 5)
		odr = LIS2DOC_REG_CTRL1_ODR_200HZ;
	else if (ms <= 10)
		odr = LIS2DOC_REG_CTRL1_ODR_100HZ;
	else if (ms <= 20)
		odr = LIS2DOC_REG_CTRL1_ODR_50HZ;
	else
		odr = LIS2DOC_REG_CTRL1_ODR_12P5HZ;

	acc_obj->odr = odr;

	ret = lis2doc_acc_set_odr(acc_obj, acc_obj->odr);
	if (ret != LIS2DOC_SUCCESS)
		ST_ERR("Set delay parameter error!\n");

	ST_LOG("%s (%d)\n", __func__, ms);

	return LIS2DOC_SUCCESS;
}

static int lis2doc_acc_get_data_intf(int *x, int *y, int *z, int *status)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;

	lis2doc_acc_read_data(acc_obj, x, y, z);

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return LIS2DOC_SUCCESS;
}

static int lis2doc_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;

	int err;

	err = lis2doc_acc_set_power_mode(acc_obj, 1);
	if (err) {
		ST_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = lis2doc_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		ST_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}

static int lis2doc_factory_get_data(int32_t data[3], int *status)
{
	return lis2doc_acc_get_data_intf(&data[0], &data[1], &data[2], status);
}

static int lis2doc_factory_get_raw_data(int32_t data[3])
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	s16 strbuf[LIS2DOC_AXES_NUM] = {0};
	int res = 0;

	res = lis2doc_acc_read_rawdata(acc_obj, strbuf);
	data[0] = strbuf[0];
	data[1] = strbuf[1];
	data[2] = strbuf[2];

	return 0;
}

static int lis2doc_factory_enable_cali(void)
{
	return 0;
}

static int lis2doc_factory_clear_cali(void)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	int err = 0;

	err = lis2doc_acc_reset_calibration(acc_obj);
	return 0;
}

static int lis2doc_factory_set_cali(int32_t data[3])
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;

	int err = 0;
	int cali[3] = { 0 };

	cali[LIS2DOC_AXIS_X] = data[0]
		* acc_obj->reso->sensitivity / GRAVITY_EARTH_1000;
	cali[LIS2DOC_AXIS_Y] = data[1]
		* acc_obj->reso->sensitivity / GRAVITY_EARTH_1000;
	cali[LIS2DOC_AXIS_Z] = data[2]
		* acc_obj->reso->sensitivity / GRAVITY_EARTH_1000;
	err = lis2doc_acc_write_calibration(acc_obj, cali);
	if (err) {
		ST_ERR("LSM6DS3H_WriteCalibration failed!\n");
		return -1;
	}
	return 0;
}

static int lis2doc_factory_get_cali(int32_t data[3])
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	int err = 0;
	int cali[3] = { 0 };

	err = lis2doc_acc_read_calibration(acc_obj, cali);
	if (err) {
		ST_ERR("BMA2x2_ReadCalibration failed!\n");
		return -1;
	}
	data[0] = cali[LIS2DOC_AXIS_X]
			* GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
	data[1] = cali[LIS2DOC_AXIS_Y]
			* GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
	data[2] = cali[LIS2DOC_AXIS_Z]
			* GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
	return 0;
}

static int lis2doc_factory_do_self_test(void)
{
	return 0;
}

static struct accel_factory_fops lis2doc_factory_fops = {
	.enable_sensor = lis2doc_factory_enable_sensor,
	.get_data = lis2doc_factory_get_data,
	.get_raw_data = lis2doc_factory_get_raw_data,
	.enable_calibration = lis2doc_factory_enable_cali,
	.clear_cali = lis2doc_factory_clear_cali,
	.set_cali = lis2doc_factory_set_cali,
	.get_cali = lis2doc_factory_get_cali,
	.do_self_test = lis2doc_factory_do_self_test,
};

static struct accel_factory_public lis2doc_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &lis2doc_factory_fops,
};

static int lis2doc_acc_local_init(void)
{
	struct lis2doc_data *obj = obj_i2c_data;
	struct lis2doc_acc *acc_obj = &obj->lis2doc_acc_data;
	struct i2c_client *client = obj->client;
	int ret = 0, retry = 0;
	struct acc_control_path ctl = { 0 };
	struct acc_data_path data = { 0 };
	s16 idmedata[CALI_SIZE] = {0};
#ifdef CONFIG_AMZN_IDME
	char *gsensor_cal = NULL;
#endif

	ST_FUN();


	ret = get_accel_dts_func(client->dev.of_node, &lis2doc_acc_cust_hw);
	if (ret < 0) {
		ST_ERR("get lis2doc dts info fail\n");
		return -EFAULT;
	}

	acc_obj->lis2doc_acc_hw = &lis2doc_acc_cust_hw;


	ret = hwmsen_get_convert(acc_obj->lis2doc_acc_hw->direction, &acc_obj->cvt);
	if (ret) {
		ST_ERR("invalid direction: %d\n", acc_obj->lis2doc_acc_hw->direction);
		goto exit_get_direction_failed;
	}

	check_caculate_by_dts(client->dev.of_node, &change_caculate);

	atomic_set(&acc_obj->trace, 0);
	atomic_set(&acc_obj->suspend, 0);

	for (retry = 0; retry < 3; retry++) {
		ret = lis2doc_acc_init(acc_obj, 1);
		if (!ret) {
			ST_LOG("lis2doc_acc_init successfully\n");
			break;
		}

		ST_ERR("lis2doc_acc_device init cilent fail time: %d\n", retry);
	}

	if (ret != 0)
		goto exit_init_failed;

	sprintf(acc_obj->name, "%s_ACC", obj->name);


	ret = lis2doc_acc_create_attr(&(lis2doc_acc_init_info.platform_diver_addr->driver));
	if (ret) {
		ST_ERR("create attribute err = %d\n", ret);
		goto exit_create_attr_failed;
	}

	ctl.is_use_common_factory  = false;
	ret = accel_factory_device_register(&lis2doc_factory_device);
	if (ret) {
		ST_ERR("accel_factory_device_register failed");
		goto exit_create_factory_attr_failed;
	}

	ctl.open_report_data       = lis2doc_acc_open_report_data_intf;
	ctl.enable_nodata          = lis2doc_acc_enable_nodata_intf;
	ctl.batch = lis2doc_batch;
	ctl.flush = lis2doc_flush;
	ctl.set_delay              = lis2doc_acc_set_delay_intf;
	ctl.is_report_input_direct = false;

	ret = acc_register_control_path(&ctl);
	if (ret) {
		ST_ERR("register acc control path ret:%d\n", ret);
		goto exit_register_control_path_failed;
	}

	data.get_data = lis2doc_acc_get_data_intf;
	data.vender_div = 1000;
	ret = acc_register_data_path(&data);
	if (ret) {
		ST_ERR("register acc data path err\n");
		goto exit_register_data_path_failed;
	}

#ifdef CONFIG_AMZN_IDME
	gsensor_cal = idme_get_sensorcal();
	if (gsensor_cal == NULL) {
		ST_ERR("idme get sensorcal fail!\n");
		goto exit_register_data_path_failed;
	}
	ret = acc_idme_get_gsensorcal(gsensor_cal, idmedata, CALI_SIZE);
	if (ret)
		ST_ERR("Get gsensor offset fail,default offset x=y=z=0\n");
#endif

	accel_xyz_offset[0] = idmedata[0];
	accel_xyz_offset[1] = idmedata[1];
	accel_xyz_offset[2] = idmedata[2];
	acc_obj->cali_sw[LIS2DOC_AXIS_X] = accel_xyz_offset[0];
	acc_obj->cali_sw[LIS2DOC_AXIS_Y] = accel_xyz_offset[1];
	acc_obj->cali_sw[LIS2DOC_AXIS_Z] = accel_xyz_offset[2];
	ST_LOG("idme to read:%d %d %d\n", accel_xyz_offset[0], accel_xyz_offset[1], accel_xyz_offset[2]);

	ST_LOG("%s: OK\n", __func__);
	return 0;


exit_register_data_path_failed:
exit_register_control_path_failed:
	accel_factory_device_deregister(&lis2doc_factory_device);
exit_create_factory_attr_failed:
	lis2doc_acc_delete_attr(&(lis2doc_acc_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
exit_init_failed:
exit_get_direction_failed:
	ST_ERR("%s: err = %d\n", __func__, ret);
	return -1;
}

static int lis2doc_acc_local_remove(void)
{
	ST_FUN();
	lis2doc_acc_delete_attr(&(lis2doc_acc_init_info.platform_diver_addr->driver));

	return LIS2DOC_SUCCESS;
}

struct acc_init_info lis2doc_acc_init_info = {
	.name = "lis2doc_acc",
	.init = lis2doc_acc_local_init,
	.uninit = lis2doc_acc_local_remove,
};
