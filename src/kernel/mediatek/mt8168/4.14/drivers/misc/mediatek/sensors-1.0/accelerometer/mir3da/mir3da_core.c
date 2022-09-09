/* Core file for MiraMEMS 3-Axis Accelerometer's driver.
 *
 * mir3da_core.c - Linux kernel modules for 3-Axis Accelerometer
 *
 * Copyright (C) 2011-2021 MiraMEMS Sensing Technology Co., Ltd.
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
#include "mir3da_core.h"
#include "mir3da_cust.h"

#define MIR3DA_REG_ADDR(REG)                ((REG)&0xFF)

#define MIR3DA_OFFSET_THRESHOLD             20
#define PEAK_LVL                            800
#define STICK_LSB                           2000
#define AIX_HISTORY_SIZE                    20

typedef struct reg_obj_s {
	short	addr;
	unsigned char	mask;
	unsigned char	value;
} reg_obj_t;

struct gsensor_data_fmt_s {
	unsigned char	msbw;
	unsigned char	lsbw;
	unsigned char	endian;                         /* 0: little endian; 1: big endian */
};

struct gsensor_data_obj_s {
	reg_obj_t	data_sect[MIR3DA_DATA_LEN];
	struct gsensor_data_fmt_s	data_fmt;
};

struct gsensor_obj_s {
	char	asic[10];

	reg_obj_t	chip_id;
	reg_obj_t	mod_id;
	reg_obj_t	soft_reset;
	reg_obj_t	power;
	reg_obj_t	init_sect[MIR3DA_INIT_SECT_LEN];
	reg_obj_t	offset_sect[MIR3DA_OFF_SECT_LEN];
	reg_obj_t	odr_sect[MIR3DA_ODR_SECT_LEN];

	struct gsensor_data_obj_s	data;

	int		(*get_reg_data)(MIR_HANDLE handle, char *buf);
};

struct gsensor_drv_s {
	struct general_op_s		*method;
	struct gsensor_obj_s	*obj;
};

typedef enum _asic_type{
	ASIC_NONE,
	ASIC_2511,
	ASIC_2512B,
	ASIC_2513A,
	ASIC_2516,
} asic_type;

typedef enum _mems_type{
	MEMS_NONE,
	MEMS_T4,
	MEMS_T9,
	MEMS_TV03,
	MEMS_RTO3,
	MEMS_GT2,
	MEMS_GT3,
} mems_type;

typedef enum _package_type{
	PACKAGE_NONE,
	PACKAGE_2X2_12PIN,
	PACKAGE_3X3_10PIN,
	PACKAGE_3X3_16PIN,
} package_type;

struct  chip_info_s{
	unsigned char    reg_value;
	package_type     package;
	asic_type        asic;
	mems_type        mems;
};

struct  chip_info_s gsensor_chip_info;

static struct chip_info_s         mir3da_chip_info_list[] = {
	{0x00, PACKAGE_2X2_12PIN, ASIC_2512B, MEMS_TV03},
	{0x01, PACKAGE_2X2_12PIN, ASIC_2511, MEMS_T4},
	{0x02, PACKAGE_2X2_12PIN, ASIC_2511, MEMS_T9},
	{0x03, PACKAGE_3X3_10PIN, ASIC_2511, MEMS_T4},
	{0x04, PACKAGE_3X3_10PIN, ASIC_2511, MEMS_T9},
	{0x05, PACKAGE_3X3_10PIN, ASIC_2511, MEMS_T4},
	{0x06, PACKAGE_3X3_10PIN, ASIC_2511, MEMS_T9},
	{0x07, PACKAGE_3X3_16PIN, ASIC_2511, MEMS_T4},
	{0x08, PACKAGE_3X3_16PIN, ASIC_2511, MEMS_T9},
	{0x09, PACKAGE_2X2_12PIN, ASIC_2511, MEMS_T4},
	{0x0c, PACKAGE_2X2_12PIN, ASIC_2512B, MEMS_T9},
	{0x33, PACKAGE_2X2_12PIN, ASIC_2511, MEMS_T9},
	{0x34, PACKAGE_2X2_12PIN, ASIC_2511, MEMS_T9},
	{0x35, PACKAGE_2X2_12PIN, ASIC_2511, MEMS_T9},
};

#define MIR3DA_NSA_INIT_SECTION	{ NSA_REG_G_RANGE, 		0x03,   0x01    }, \
				{ NSA_REG_POWERMODE_BW, 	0xFF,   0x3e    }, \
				{ NSA_REG_ODR_AXIS_DISABLE,     0xFF,   0x07    }, \
				{ NSA_REG_INTERRUPT_SETTINGS2,  0xFF,   0x00    }, \
				{ NSA_REG_INTERRUPT_MAPPING2,   0xFF,   0x00    }, \
				{ NSA_REG_ENGINEERING_MODE,     0xFF,   0x83    }, \
				{ NSA_REG_ENGINEERING_MODE,     0xFF,   0x69    }, \
				{ NSA_REG_ENGINEERING_MODE,     0xFF,   0xBD    }, \
				{ NSA_REG_INT_PIN_CONFIG,       0x0F,   0x05    }, \
				{ -1,                           0x00,   0x00    }, \
				{ -1,                           0x00,   0x00    }, \


#define MIR3DA_NSA_OFFSET_SECTION	{ NSA_REG_COARSE_OFFSET_TRIM_X, 0xFF,   0x00    }, \
					{ NSA_REG_COARSE_OFFSET_TRIM_Y, 0xFF,   0x00    }, \
					{ NSA_REG_COARSE_OFFSET_TRIM_Z, 0xFF,   0x00    }, \
					{ NSA_REG_FINE_OFFSET_TRIM_X,   0xFF,   0x00    }, \
					{ NSA_REG_FINE_OFFSET_TRIM_Y,   0xFF,   0x00    }, \
					{ NSA_REG_FINE_OFFSET_TRIM_Z,   0xFF,   0x00    }, \
					{ NSA_REG_CUSTOM_OFFSET_X,      0xFF,   0x00    }, \
					{ NSA_REG_CUSTOM_OFFSET_Y,      0xFF,   0x00    }, \
					{ NSA_REG_CUSTOM_OFFSET_Z,      0xFF,   0x00    }, \

#define MIR3DA_NSA_ODR_SECTION	{ NSA_REG_ODR_AXIS_DISABLE,     0x0F,   0x06    }, \
				{ NSA_REG_ODR_AXIS_DISABLE,     0x0F,   0x07    }, \
				{ NSA_REG_ODR_AXIS_DISABLE,     0x0F,   0x08    }, \


#define MIR3DA_NSA_DATA_SECTION	{{ NSA_REG_ACC_X_LSB,           0xFF,   0x00  }, \
				{ NSA_REG_ACC_X_MSB,            0xFF,   0x00  }, \
				{ NSA_REG_ACC_Y_LSB,            0xFF,   0x00  }, \
				{ NSA_REG_ACC_Y_MSB,            0xFF,   0x00  }, \
				{ NSA_REG_ACC_Z_LSB,            0xFF,   0x00  }, \
				{ NSA_REG_ACC_Z_MSB,            0xFF,   0x00  } }, \
				{ 8,                            5,      0     }

static int NSA_get_reg_data(MIR_HANDLE handle, char *buf);

#define MIR_NSA_NTO		{ "NSA_NTO", { NSA_REG_WHO_AM_I, 0xFF, 0x13 }, \
					{ NSA_REG_FIFO_CTRL,        0xFF,   0x00    }, \
					{ NSA_REG_SPI_I2C,          0x24,   0x24    }, \
					{ NSA_REG_POWERMODE_BW,     0x80,   0x80    }, \
					{ MIR3DA_NSA_INIT_SECTION                   }, \
					{ MIR3DA_NSA_OFFSET_SECTION                 }, \
					{ MIR3DA_NSA_ODR_SECTION                    }, \
					{ MIR3DA_NSA_DATA_SECTION                   }, \
					NSA_get_reg_data,                              \
					}
/**************************************************************** COMMON ***************************************************************************/
#define MIR3DA_GSENSOR_SCHEME           MIR3DA_SUPPORT_CHIP_LIST

/* this level can be modified while runtime through system attribute */
int                                 Log_level = DEBUG_MSG|DEBUG_ERR|DEBUG_ASSERT|DEBUG_MSG|DEBUG_FUNC|DEBUG_DATA;
int                                gsensor_mod = -1;        /* Initial value */
int                                gsensor_type = -1;        /* Initial value */
static struct gsensor_obj_s         mir3da_gsensor[] = { MIR3DA_GSENSOR_SCHEME };
struct gsensor_drv_s                mir3da_gsensor_drv;

#define MI_DATA(format, ...) \
	do { \
		if (DEBUG_DATA&Log_level) { \
			mir3da_gsensor_drv.method->myprintf(MI_TAG format "\n", ## __VA_ARGS__); } \
	} while (0)
#define MI_MSG(format, ...) \
	do { \
		if (DEBUG_MSG&Log_level) { \
			mir3da_gsensor_drv.method->myprintf(MI_TAG format "\n", ## __VA_ARGS__); } \
	} while (0)
#define MI_ERR(format, ...) \
	do { \
		if (DEBUG_ERR&Log_level) { \
			mir3da_gsensor_drv.method->myprintf(MI_TAG format "\n", ## __VA_ARGS__); } \
	} while (0)
#define MI_FUN \
	do { \
		if (DEBUG_FUNC&Log_level) { \
			mir3da_gsensor_drv.method->myprintf(MI_TAG "%s is called, line: %d\n", __FUNCTION__, __LINE__); } \
	} while (0)
#define MI_ASSERT(expr) \
	do { \
		if (!(expr))  { \
			mir3da_gsensor_drv.method->myprintf("Assertion failed! %s,%d,%s,%s\n", __FILE__, __LINE__, __func__, #expr); } \
	} while (0)

#ifndef MTK_ANDROID_M
#define abs(x) ({ long __x = (x); (__x < 0) ? -__x : __x; })
#endif

int squareRoot(int val)
{
	int r = 0;
	int shift;

	if (val < 0)
		return 0;

	for (shift = 0; shift < 32; shift += 2) {
		int x = 0x40000000l >> shift;

		if (x + r <= val) {
			val -= x + r;
			r = (r >> 1) | x;
		} else {
			r = r >> 1;
		}
	}

	return r;
}

int mir3da_register_read(MIR_HANDLE handle, short addr, unsigned char *data)
{
	int     res = 0;

	res = mir3da_gsensor_drv.method->smi.read(handle, MIR3DA_REG_ADDR(addr), data);

	return res;
}

int mir3da_register_read_continuously(MIR_HANDLE handle, short addr, unsigned char count, unsigned char *data)
{
	int     res = 0;

	res = (count == mir3da_gsensor_drv.method->smi.read_block(handle, MIR3DA_REG_ADDR(addr), count, data)) ? 0 : 1;

	return res;
}

int mir3da_register_write(MIR_HANDLE handle, short addr, unsigned char data)
{
	int     res = 0;

	res = mir3da_gsensor_drv.method->smi.write(handle, MIR3DA_REG_ADDR(addr), data);

	return res;
}

int mir3da_register_mask_write(MIR_HANDLE handle, short addr, unsigned char mask, unsigned char data)
{
	int     res = 0;
	unsigned char      tmp_data;

	res = mir3da_register_read(handle, addr, &tmp_data);
	if (res)
		return res;

	tmp_data &= ~mask;
	tmp_data |= data & mask;
	res = mir3da_register_write(handle, addr, tmp_data);

	return res;
}

static int mir3da_read_raw_data(MIR_HANDLE handle, short *x, short *y, short *z)
{
	unsigned char    tmp_data[6] = {0};

	if (mir3da_register_read_continuously(handle, mir3da_gsensor_drv.obj[gsensor_mod].data.data_sect[0].addr, 6, tmp_data) != 0) {
		MI_ERR("i2c block read failed\n");
		return -1;
	}

	*x = ((short)(tmp_data[1] << mir3da_gsensor_drv.obj[gsensor_mod].data.data_fmt.msbw | tmp_data[0])) >> (8-mir3da_gsensor_drv.obj[gsensor_mod].data.data_fmt.lsbw);
	*y = ((short)(tmp_data[3] << mir3da_gsensor_drv.obj[gsensor_mod].data.data_fmt.msbw | tmp_data[2])) >> (8-mir3da_gsensor_drv.obj[gsensor_mod].data.data_fmt.lsbw);
	*z = ((short)(tmp_data[5] << mir3da_gsensor_drv.obj[gsensor_mod].data.data_fmt.msbw | tmp_data[4])) >> (8-mir3da_gsensor_drv.obj[gsensor_mod].data.data_fmt.lsbw);

	return 0;
}

static int remap[8][4] = {{0, 0, 0, 0},
						{0, 1, 0, 1},
						{1, 1, 0, 0},
						{1, 0, 0, 1},
						{1, 0, 1, 0},
						{0, 0, 1, 1},
						{0, 1, 1, 0},
						{1, 1, 1, 1} };

int mir3da_direction_remap(short *x, short *y, short *z, int direction)
{
	short temp = 0;

	*x = *x - ((*x) * remap[direction][0]*2);
	*y = *y - ((*y) * remap[direction][1]*2);
	*z = *z - ((*z) * remap[direction][2]*2);

	if (remap[direction][3]) {
		temp = *x;
		*x = *y;
		*y = temp;
	}

	if (remap[direction][2])
		return -1;

	return 1;
}

int mir3da_read_data(MIR_HANDLE handle, short *x, short *y, short *z)
{
	int    rst = 0;

	rst = mir3da_read_raw_data(handle, x, y, z);
	if (rst != 0) {
		MI_ERR("mir3da_read_raw_data failed, rst = %d", rst);
		return rst;
	}

	return 0;
}

int cycle_read_xyz(MIR_HANDLE handle, int *x, int *y, int *z, int ncycle)
{
	unsigned int j = 0;
	short raw_x, raw_y, raw_z;

	*x = *y = *z = 0;

	for (j = 0; j < ncycle; j++) {
		raw_x = raw_y = raw_z = 0;
		mir3da_read_raw_data(handle, &raw_x, &raw_y, &raw_z);

		(*x) += raw_x;
		(*y) += raw_y;
		(*z) += raw_z;

		mir3da_gsensor_drv.method->msdelay(5);
	}

	(*x) /= ncycle;
	(*y) /= ncycle;
	(*z) /= ncycle;

	return 0;
}

int mir3da_read_offset(MIR_HANDLE handle, unsigned char *offset)
{
	int     i, res = 0;

	for (i = 0; i < MIR3DA_OFF_SECT_LEN; i++) {
		if (mir3da_gsensor_drv.obj[gsensor_mod].offset_sect[i].addr < 0)
			break;

		res = mir3da_register_read(handle, mir3da_gsensor_drv.obj[gsensor_mod].offset_sect[i].addr, &offset[i]);
		if (res != 0)
			return res;
	}

	return res;
}

int mir3da_write_offset(MIR_HANDLE handle, unsigned char *offset)
{
	int     i, res = 0;

	for (i = 0; i < MIR3DA_OFF_SECT_LEN; i++) {
		if (mir3da_gsensor_drv.obj[gsensor_mod].offset_sect[i].addr < 0)
			break;

		res = mir3da_register_write(handle, mir3da_gsensor_drv.obj[gsensor_mod].offset_sect[i].addr, offset[i]);
		if (res != 0)
			return res;
	}

	return res;
}

int mir3da_get_enable(MIR_HANDLE handle, char *enable)
{
	int             res = 0;
	unsigned char   reg_data = 0;

	res = mir3da_register_read(handle, mir3da_gsensor_drv.obj[gsensor_mod].power.addr, &reg_data);
	if (res != 0)
		return res;

	*enable = (reg_data & mir3da_gsensor_drv.obj[gsensor_mod].power.mask) ? 0 : 1;

	return res;
}

int mir3da_set_enable(MIR_HANDLE handle, char enable)
{
	int             res = 0;
	unsigned char   reg_data = 0;

	if (!enable)
		reg_data = mir3da_gsensor_drv.obj[gsensor_mod].power.value;

	res = mir3da_register_mask_write(handle, mir3da_gsensor_drv.obj[gsensor_mod].power.addr, mir3da_gsensor_drv.obj[gsensor_mod].power.mask, reg_data);

	return res;
}

static int NSA_get_reg_data(MIR_HANDLE handle, char *buf)
{
	int                 count = 0;
	int                 i;
	unsigned char       val;

	count += mir3da_gsensor_drv.method->mysprintf(buf+count, "---------start---------");
	for (i = 0; i <= 0xd2; i++) {
		if (i%16 == 0)
			count += mir3da_gsensor_drv.method->mysprintf(buf+count, "\n%02x\t", i);
		mir3da_register_read(handle, i, &val);
		count += mir3da_gsensor_drv.method->mysprintf(buf+count, "%02X ", val);
	}

	count += mir3da_gsensor_drv.method->mysprintf(buf+count, "\n--------end---------\n");
	return count;
}

int mir3da_get_reg_data(MIR_HANDLE handle, char *buf)
{
	return mir3da_gsensor_drv.obj[gsensor_mod].get_reg_data(handle, buf);
}

int mir3da_set_odr(MIR_HANDLE handle, int delay)
{
	int     res = 0;
	int     odr = 0;

	if (delay <= 5)
		odr = MIR3DA_ODR_200HZ;
	else if (delay <= 10)
		odr = MIR3DA_ODR_100HZ;
	else
		odr = MIR3DA_ODR_50HZ;

	res = mir3da_register_mask_write(handle, mir3da_gsensor_drv.obj[gsensor_mod].odr_sect[odr].addr,
			mir3da_gsensor_drv.obj[gsensor_mod].odr_sect[odr].mask, mir3da_gsensor_drv.obj[gsensor_mod].odr_sect[odr].value);
	if (res != 0)
		return res;

	return res;
}

static int mir3da_soft_reset(MIR_HANDLE handle)
{
	int             res = 0;
	unsigned char   reg_data;

	reg_data = mir3da_gsensor_drv.obj[gsensor_mod].soft_reset.value;
	res = mir3da_register_mask_write(handle, mir3da_gsensor_drv.obj[gsensor_mod].soft_reset.addr, mir3da_gsensor_drv.obj[gsensor_mod].soft_reset.mask, reg_data);
	mir3da_gsensor_drv.method->msdelay(5);

	return res;
}

int mir3da_module_detect(PLAT_HANDLE handle)
{
	int             i, res = 0;
	unsigned char   cid, mid;
	int             is_find = -1;

	/* Probe gsensor module */
	for (i = 0; i < sizeof(mir3da_gsensor)/sizeof(mir3da_gsensor[0]); i++) {
		res = mir3da_register_read(handle, mir3da_gsensor[i].chip_id.addr, &cid);
		if (res != 0)
			return res;

		cid &= mir3da_gsensor[i].chip_id.mask;
		if (mir3da_gsensor[i].chip_id.value == cid) {
			res = mir3da_register_read(handle, mir3da_gsensor[i].mod_id.addr, &mid);
			if (res != 0)
				return res;

			mid &= mir3da_gsensor[i].mod_id.mask;
			if (mir3da_gsensor[i].mod_id.value == mid) {
				MI_MSG("Found Gsensor MIR3DA !");
				gsensor_mod = i;
				is_find = 0;
				break;
			}
		}
	}

	return is_find;
}

int mir3da_parse_chip_info(PLAT_HANDLE handle)
{
	unsigned char i = 0, tmp = 0;
	unsigned char reg_value = -1, reg_value1 = -1, reg_value2 = -1;
	char res = -1;

	if (-1 == gsensor_mod)
		return res;

	res = mir3da_register_read(handle, NSA_REG_CHIP_INFO, &reg_value);
	if (res != 0)
		return res;

	gsensor_chip_info.reg_value    = reg_value;

	if (0 == (reg_value>>6))
		return -1;

	if (!(reg_value&0xc0)) {
		gsensor_chip_info.asic = ASIC_2511;
		gsensor_chip_info.mems = MEMS_T9;
		gsensor_chip_info.package = PACKAGE_NONE;

		for (i = 0; i < sizeof(mir3da_chip_info_list)/sizeof(mir3da_chip_info_list[0]); i++) {
			if (reg_value == mir3da_chip_info_list[i].reg_value) {
				gsensor_chip_info.package = mir3da_chip_info_list[i].package;
				gsensor_chip_info.asic = mir3da_chip_info_list[i].asic;
				gsensor_chip_info.mems = mir3da_chip_info_list[i].mems;
				break;
			}
		}
	} else {
		gsensor_chip_info.asic = ASIC_2512B;
		gsensor_chip_info.mems = MEMS_T9;
		gsensor_chip_info.package = PACKAGE_NONE;

		gsensor_chip_info.package = (package_type)((reg_value&0xc0)>>6);

		if ((reg_value&0x38)>>3 == 0x01)
			gsensor_chip_info.asic = ASIC_2512B;
		else if ((reg_value&0x38)>>3 == 0x02)
			gsensor_chip_info.asic = ASIC_2513A;
		else if ((reg_value&0x38)>>3 == 0x03)
			gsensor_chip_info.asic = ASIC_2516;

		res = mir3da_register_read(handle, NSA_REG_CHIP_INFO_SECOND, &reg_value1);
		if (res != 0)
			return res;

		if (gsensor_chip_info.asic == ASIC_2512B) {
			res = mir3da_register_read(handle, NSA_REG_MEMS_OPTION, &reg_value);
			if (res != 0)
				return res;
			tmp = ((reg_value&0x01)<<2) | ((reg_value1&0xc0)>>6);
		} else {
			tmp = (reg_value1&0xe0)>>5;
		}

		res = mir3da_register_read(handle, NSA_REG_MEMS_OPTION, &reg_value2);
		if (res != 0)
			return res;

	if (tmp == 0x00) {
		if (reg_value2&0x80)
			gsensor_chip_info.mems = MEMS_TV03;
		else
			gsensor_chip_info.mems = MEMS_T9;
		} else if (tmp == 0x01) {
			gsensor_chip_info.mems = MEMS_RTO3;
		} else if (tmp == 0x03) {
			gsensor_chip_info.mems = MEMS_GT2;
		if ((gsensor_chip_info.reg_value != 0x5A) && (gsensor_chip_info.asic == ASIC_2516))
			gsensor_chip_info.mems = MEMS_GT3;
		} else if (tmp == 0x04) {
			gsensor_chip_info.mems = MEMS_GT3;
		}
	}

	return 0;
}


int mir3da_install_general_ops(struct general_op_s *ops)
{
	if (ops == 0)
		return -1;

	mir3da_gsensor_drv.method = ops;
	return 0;
}

MIR_HANDLE mir3da_core_init(PLAT_HANDLE handle)
{
	int             res = 0;

	mir3da_gsensor_drv.obj = mir3da_gsensor;

	if (gsensor_mod < 0) {
		res = mir3da_module_detect(handle);
		if (res) {
			MI_ERR("Can't find Mir3da gsensor!!");
			return 0;
		}

		/* No miramems gsensor instance found */
		if (gsensor_mod < 0)
			return 0;
	}

	MI_MSG("Probe gsensor module: %s", mir3da_gsensor[gsensor_mod].asic);

	res = mir3da_chip_resume(handle);
	if (res) {
		MI_ERR("chip resume fail!!\n");
		return 0;
	}

	return handle;
}

int mir3da_chip_resume(MIR_HANDLE handle)
{
	int     res = 0;
	unsigned char      reg_data;
	unsigned char      i = 0;

	res = mir3da_soft_reset(handle);
	if (res) {
		MI_ERR("Do softreset failed !");
		return res;
	}

	for (i = 0; i < MIR3DA_INIT_SECT_LEN; i++) {
		if (mir3da_gsensor_drv.obj[gsensor_mod].init_sect[i].addr < 0)
			break;

		reg_data = mir3da_gsensor_drv.obj[gsensor_mod].init_sect[i].value;
		res = mir3da_register_mask_write(handle, mir3da_gsensor_drv.obj[gsensor_mod].init_sect[i].addr, mir3da_gsensor_drv.obj[gsensor_mod].init_sect[i].mask, reg_data);
		if (res != 0)
			return res;
	}

	mir3da_gsensor_drv.method->msdelay(10);

	if (gsensor_type < 0) {
		gsensor_type = mir3da_parse_chip_info(handle);

		if (gsensor_type < 0) {
			MI_ERR("Can't parse Mir3da gsensor chipinfo!!");
			return -1;
		}
	}

	if (gsensor_chip_info.asic == ASIC_2512B) {

		reg_data = mir3da_gsensor_drv.method->get_address(handle);


		if (reg_data == 0x26 || reg_data == 0x4c)
			mir3da_register_mask_write(handle, NSA_REG_SENS_COMP, 0xc0, 0x00);
	}

	return res;
}

int mir3da_get_primary_offset(MIR_HANDLE handle, int *x, int *y, int *z)
{
	int     res = 0;
	unsigned char      reg_data;
	unsigned char      i = 0;
	unsigned char      offset[9] = {0};

	res = mir3da_read_offset(handle, offset);
	if (res != 0) {
		MI_ERR("Read offset failed !");
		return -1;
	}

	res = mir3da_soft_reset(handle);
	if (res) {
		MI_ERR("Do softreset failed !");
		return -1;
	}

	for (i = 0; i < MIR3DA_INIT_SECT_LEN; i++) {
		if (mir3da_gsensor_drv.obj[gsensor_mod].init_sect[i].addr < 0)
			break;


		reg_data = mir3da_gsensor_drv.obj[gsensor_mod].init_sect[i].value;
		res = mir3da_register_mask_write(handle, mir3da_gsensor_drv.obj[gsensor_mod].init_sect[i].addr, mir3da_gsensor_drv.obj[gsensor_mod].init_sect[i].mask, reg_data);
		if (res != 0) {
			MI_ERR("Write register[0x%x] error!", mir3da_gsensor_drv.obj[gsensor_mod].init_sect[i].addr);
			goto EXIT;
		}
	}

	mir3da_gsensor_drv.method->msdelay(100);

	res = cycle_read_xyz(handle, x, y, z, 20);
	if (res) {
		MI_ERR("i2c block read failed\n");
		goto EXIT;
	}

	mir3da_write_offset(handle, offset);

	if ((gsensor_chip_info.reg_value == 0x4B)
		|| (gsensor_chip_info.reg_value == 0x8C)
		|| (gsensor_chip_info.reg_value == 0xCA)
		|| (gsensor_chip_info.mems == MEMS_GT2)) {
		*z = 0;
	}

	return 0;

EXIT:
	mir3da_write_offset(handle, offset);
	return -1;
}
