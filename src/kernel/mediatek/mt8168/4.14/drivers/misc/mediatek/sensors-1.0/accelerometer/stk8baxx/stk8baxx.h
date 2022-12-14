/*!
 * @section LICENSE
 * (C) Copyright 2011~2016 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 */

#ifndef STK8BAXX_H
#define STK8BAXX_H
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>

/*****************************************************************************
 * stk8baxx register, start
 *****************************************************************************/
#define STK8BAXX_SLAVE_ADDR             0x18

#define STK8BAXX_REG_CHIPID             0x00
#define STK8BAXX_REG_XOUT1              0x02
#define STK8BAXX_REG_XOUT2              0x03
#define STK8BAXX_REG_YOUT1              0x04
#define STK8BAXX_REG_YOUT2              0x05
#define STK8BAXX_REG_ZOUT1              0x06
#define STK8BAXX_REG_ZOUT2              0x07
#define STK8BAXX_REG_INTSTS1            0x09
#define STK8BAXX_REG_INTSTS2            0x0A
#define STK8BAXX_REG_RANGESEL           0x0F
#define STK8BAXX_REG_BWSEL              0x10
#define STK8BAXX_REG_POWMODE            0x11
#define STK8BAXX_REG_SWRST              0x14
#define STK8BAXX_REG_INTEN1             0x16
#define STK8BAXX_REG_INTEN2             0x17
#define STK8BAXX_REG_INTMAP1            0x19
#define STK8BAXX_REG_INTMAP2            0x1A
#define STK8BAXX_REG_INTCFG1            0x20
#define STK8BAXX_REG_INTCFG2            0x21
#define STK8BAXX_REG_SLOPEDLY           0x27
#define STK8BAXX_REG_SLOPETHD           0x28
#define STK8BAXX_REG_SIGMOT1            0x29
#define STK8BAXX_REG_SIGMOT2            0x2A
#define STK8BAXX_REG_SIGMOT3            0x2B
#define STK8BAXX_REG_INTFCFG            0x34
#define STK8BAXX_REG_OFSTCOMP1          0x36
#define STK8BAXX_REG_OFSTX              0x38
#define STK8BAXX_REG_OFSTY              0x39
#define STK8BAXX_REG_OFSTZ              0x3A

/* STK8BAXX_REG_CHIPID */
#define STK8BA50_R_ID                   0x86
#define STK8BA53_ID                     0x87

/* STK8BAXX_REG_INTSTS1 */
#define STK8BAXX_INTSTS1_SIG_MOT_STS     0x1
#define STK8BAXX_INTSTS1_ANY_MOT_STS     0x4

/* STK8BAXX_REG_RANGESEL */
#define STK8BAXX_RANGESEL_2G             0x3
#define STK8BAXX_RANGESEL_4G             0x5
#define STK8BAXX_RANGESEL_8G             0x8
#define STK8BAXX_RANGESEL_BW_MASK        0x1F
#define STK8BAXX_RANGESEL_DEF            STK8BAXX_RANGESEL_2G

#define STK8BAXX_SPTIME_BASE             0x9     /* for 32000, ODR:31.25 */
#define STK8BAXX_SPTIME_BOUND            0xB     /* for 8000, ODR:125 */

/* STK8BAXX_REG_POWMODE */
#define STK8BAXX_PWMD_SUSPEND            0x80
#define STK8BAXX_PWMD_LOWPOWER           0x40
#define STK8BAXX_PWMD_NORMAL             0x00
#define STK8BAXX_PWMD_SLP_MASK           0x3E

/* STK8BAXX_REG_SWRST */
#define STK8BAXX_SWRST_VAL               0xB6

/* STK8BAXX_REG_INTEN1 */
#define STK8BAXX_INTEN1_SLP_EN_XYZ       0x07

/* STK8BAXX_REG_INTEN2 */
#define STK8BAXX_INTEN2_DATA_EN          0x10

/* STK8BAXX_REG_INTMAP1 */
#define STK8BAXX_INTMAP1_SIGMOT2INT1         0x01
#define STK8BAXX_INTMAP1_ANYMOT2INT1         0x04

/* STK8BAXX_REG_INTMAP2 */
#define STK8BAXX_INTMAP2_DATA2INT1           0x01

/* STK8BAXX_REG_INTCFG1 */
#define STK8BAXX_INTCFG1_INT1_ACTIVE_H       0x01
#define STK8BAXX_INTCFG1_INT1_OD_PUSHPULL    0x00

/* STK8BAXX_REG_INTCFG2 */
#define STK8BAXX_INTCFG2_NOLATCHED           0x00
#define STK8BAXX_INTCFG2_LATCHED             0x0F
#define STK8BAXX_INTCFG2_INT_RST             0x80

/* STK8BAXX_REG_SLOPETHD */
#define STK8BAXX_SLOPETHD_DEF                0x14

/* STK8BAXX_REG_SIGMOT1 */
#define STK8BAXX_SIGMOT1_SKIP_TIME_3SEC      0x96    /* default value */

/* STK8BAXX_REG_SIGMOT2 */
#define STK8BAXX_SIGMOT2_SIG_MOT_EN          0x02
#define STK8BAXX_SIGMOT2_ANY_MOT_EN          0x04

/* STK8BAXX_REG_SIGMOT3 */
#define STK8BAXX_SIGMOT3_PROOF_TIME_1SEC     0x32    /* default value */

/* STK8BAXX_REG_INTFCFG */
#define STK8BAXX_INTFCFG_I2C_WDT_EN          0x04

/* STK8BAXX_REG_OFSTCOMP1 */
#define STK8BAXX_OFSTCOMP1_OFST_RST          0x80

/* STK8BAXX_REG_OFSTx */
#define STK8BAXX_OFST_LSB                    128     /* 8 bits for +-1G */

/* selftest usage */
#define STK_SELFTEST_SAMPLE_NUM             100
#define STK_SELFTEST_RESULT_NA              0
#define STK_SELFTEST_RESULT_RUNNING         (1 << 0)
#define STK_SELFTEST_RESULT_NO_ERROR        (1 << 1)
#define STK_SELFTEST_RESULT_DRIVER_ERROR    (1 << 2)
#define STK_SELFTEST_RESULT_FAIL_X          (1 << 3)
#define STK_SELFTEST_RESULT_FAIL_Y          (1 << 4)
#define STK_SELFTEST_RESULT_FAIL_Z          (1 << 5)
#define STK_SELFTEST_RESULT_NO_OUTPUT       (1 << 6)

#define STK8BAXX_SUCCESS                           0
#define STK8BAXX_ERR_I2C                          -1
#define STK8BAXX_ERR_CLIENT                       -2
#define STK8BAXX_ERR_STATUS                       -3
#define STK8BAXX_ERR_SETUP_FAILURE                -4
#define STK8BAXX_ERR_GETGSENSORDATA               -5
#define STK8BAXX_ERR_IDENTIFICATION               -6
/*****************************************************************************
 * stk8baxx register, end
 *****************************************************************************/


#endif
