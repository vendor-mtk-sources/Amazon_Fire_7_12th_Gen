/* SPDX-License-Identifier: GPL-2.0-only */
// SGM4154X Charger Driver
/*
 * Copyright (C) 2022 Amazon.com Inc. or its affiliates.  All Rights Reserved.
 */
#ifndef __SGM4154X_HEADER__
#define __SGM4154X_HEADER__

#include <linux/i2c.h>
#include <linux/bits.h>

#define SGM4154X_MANUFACTURER    "SG Micro"
//#define __SGM41541_CHIP_ID__
//#define __SGM41542_CHIP_ID__
#define __SGM41513_CHIP_ID__
//#define __SGM41513A_CHIP_ID__
//#define __SGM41513D_CHIP_ID__
//#define __SGM41516_CHIP_ID__
//#define __SGM41516D_CHIP_ID__
//#define __SGM41543_CHIP_ID__
//#define __SGM41543D_CHIP_ID__

#ifdef __SGM41541_CHIP_ID__
#define SGM4154X_NAME       "sgm41541"
#define SGM4154X_PN_ID      (BIT(6) | BIT(5))
#endif

#ifdef __SGM41542_CHIP_ID__
#define SGM4154X_NAME        "sgm41542"
#define SGM4154X_PN_ID       (BIT(6) | BIT(5) | BIT(3))
#endif

#ifdef __SGM41513_CHIP_ID__
#define SGM4154X_NAME        "sgm41513"
#define SGM4154X_PN_ID       0
#define SGM4154X_I2C_ADDR    0x1A
#endif

#ifdef __SGM41513A_CHIP_ID__
#define SGM4154X_NAME       "sgm41513A"
#define SGM4154X_PN_ID       BIT(3)
#define SGM4154X_I2C_ADDR    0x1A
#endif

#ifdef __SGM41513D_CHIP_ID__
#define SGM4154X_NAME        "sgm41513D"
#define SGM4154X_PN_ID       BIT(3)
#define SGM4154X_I2C_ADDR    0x1A
#endif

#ifdef __SGM41516_CHIP_ID__
#define SGM4154X_NAME        "sgm41516"
#define SGM4154X_PN_ID       (BIT(6) | BIT(5))
#endif

#ifdef __SGM41516D_CHIP_ID__
#define SGM4154X_NAME        "sgm41516D"
#define SGM4154X_PN_ID       (BIT(6) | BIT(5) | BIT(3))
#endif

#ifdef __SGM41543_CHIP_ID__
#define SGM4154X_NAME        "sgm41543"
#define SGM4154X_PN_ID       BIT(6)
#endif

#ifdef __SGM41543D_CHIP_ID__
#define SGM4154X_NAME        "sgm41543D"
#define SGM4154X_PN_ID      (BIT(6) | BIT(3))
#endif

/*define register*/
#define SGM4154X_CHRG_CTRL_0           0x00
#define SGM4154X_CHRG_CTRL_1           0x01
#define SGM4154X_CHRG_CTRL_2           0x02
#define SGM4154X_CHRG_CTRL_3           0x03
#define SGM4154X_CHRG_CTRL_4           0x04
#define SGM4154X_CHRG_CTRL_5           0x05
#define SGM4154X_CHRG_CTRL_6           0x06
#define SGM4154X_CHRG_CTRL_7           0x07
#define SGM4154X_CHRG_STAT             0x08
#define SGM4154X_CHRG_FAULT            0x09
#define SGM4154X_CHRG_CTRL_a           0x0a
#define SGM4154X_CHRG_CTRL_b           0x0b
#define SGM4154X_CHRG_CTRL_c           0x0c
#define SGM4154X_CHRG_CTRL_d           0x0d
#define SGM4154X_INPUT_DET             0x0e
#define SGM4154X_CHRG_CTRL_f           0x0f

/* charge flags  */
#define SGM4154X_CHRG_EN               BIT(4)
#define SGM4154X_HIZ_EN                BIT(7)
#define SGM4154X_TERM_EN               BIT(7)
#define SGM4154X_VAC_OVP_MASK          GENMASK(7, 6)
#define SGM4154X_DPDM_ONGOING          BIT(7)
#define SGM4154X_VBUS_GOOD             BIT(7)
#define SGM4154X_TERM_DISABLE          0

#define SGM4154X_STAT_CTRL_MASK        GENMASK(6, 5)
#define SGM4154X_STAT_CTRL_SHIFT       5

#define SGM4154X_BOOSTV                GENMASK(5, 4)
#define SGM4154X_BOOST_LIM             BIT(7)
#define SGM4154X_OTG_EN                BIT(5)

/* Part ID  */
#define SGM4154X_PN_MASK               GENMASK(6, 3)

/* WDT TIMER SET  */
#define SGM4154X_WDT_TIMER_MASK        GENMASK(5, 4)
#define SGM4154X_WDT_TIMER_DISABLE     0
#define SGM4154X_WDT_TIMER_40S         BIT(4)
#define SGM4154X_WDT_TIMER_80S         BIT(5)
#define SGM4154X_WDT_TIMER_160S        (BIT(4) | BIT(5))

#define SGM4154X_WDT_RST_MASK          BIT(6)

/* SAFETY TIMER SET  */
#define SGM4154X_SAFETY_TIMER_MASK     GENMASK(3, 3)
#define SGM4154X_SAFETY_TIMER_DISABLE  0
#define SGM4154X_SAFETY_TIMER_EN       BIT(3)
#define SGM4154X_SAFETY_TIMER_SET_MASK BIT(2)
#define SGM4154X_SAFETY_TIMER_10H      0
#define SGM4154X_SAFETY_TIMER_20H      BIT(2)

/* recharge voltage  */
#define SGM4154X_VRECHARGE             BIT(0)
#define SGM4154X_VRECHRG_STEP_mV       100
#define SGM4154X_VRECHRG_OFFSET_mV     100

/* charge status  */
#define SGM4154X_VSYS_STAT             BIT(0)
#define SGM4154X_THERM_STAT            BIT(1)
#define SGM4154X_PG_STAT               BIT(2)
#define SGM4154X_CHG_STAT_MASK         GENMASK(4, 3)
#define SGM4154X_PRECHRG               BIT(3)
#define SGM4154X_FAST_CHRG             BIT(4)
#define SGM4154X_TERM_CHRG             (BIT(3) | BIT(4))

/* charge type  */
#define SGM4154X_VBUS_STAT_MASK        GENMASK(7, 5)
#define SGM4154X_NOT_CHRGING           0
#define SGM4154X_USB_SDP               BIT(5)
#define SGM4154X_USB_CDP               BIT(6)
#define SGM4154X_USB_DCP               (BIT(5) | BIT(6))
#define SGM4154X_UNKNOWN               (BIT(7) | BIT(5))
#define SGM4154X_NON_STANDARD          (BIT(7) | BIT(6))
#define SGM4154X_OTG_MODE              (BIT(7) | BIT(6) | BIT(5))

/* TEMP Status  */
#define SGM4154X_TEMP_MASK             GENMASK(2, 0)
#define SGM4154X_TEMP_NORMAL           BIT(0)
#define SGM4154X_TEMP_WARM             BIT(1)
#define SGM4154X_TEMP_COOL             (BIT(0) | BIT(1))
#define SGM4154X_TEMP_COLD             (BIT(0) | BIT(3))
#define SGM4154X_TEMP_HOT              (BIT(2) | BIT(3))

/* precharge current  */
#define SGM4154X_PRECHRG_CUR_MASK      GENMASK(7, 4)
#define SGM4154X_PRECHRG_CUR_STEP_uA   60000
#define SGM4154X_PRECHRG_I_MIN_uA      60000
#define SGM4154X_PRECHRG_I_MAX_uA      780000
#define SGM4154X_PRECHRG_I_DEF_uA      180000
#define SGM4154X_PRECHRG_I_SHIFT       4

/* termination current  */
#define SGM4154X_TERMCHRG_CUR_MASK     GENMASK(3, 0)
#define SGM4154X_TERMCHRG_CU_STEP_uA   60000
#define SGM4154X_TERMCHRG_I_MIN_uA     60000
#define SGM4154X_TERMCHRG_I_MAX_uA     960000
#define SGM4154X_TERMCHRG_I_DEF_uA     180000

/* charge current  */
#define SGM4154X_ICHRG_I_MASK          GENMASK(5, 0)

#define SGM4154X_ICHRG_I_MIN_uA        0
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
#define SGM4154X_ICHRG_I_MAX_uA        3000000
#define SGM4154X_ICHRG_I_DEF_uA        1980000
#else
#define SGM4154X_ICHRG_I_STEP_uA       60000
#define SGM4154X_ICHRG_I_MAX_uA        3780000
#define SGM4154X_ICHRG_I_DEF_uA        2040000
#endif
/* charge voltage  */
#define SGM4154X_VREG_V_MASK           GENMASK(7, 3)
#define SGM4154X_VREG_V_MAX_uV         4624000
#define SGM4154X_VREG_V_MIN_uV         3856000
#define SGM4154X_VREG_V_DEF_uV         4208000
#define SGM4154X_VREG_V_STEP_uV        32000

/* iindpm current  */
#define SGM4154X_IINDPM_I_MASK         GENMASK(4, 0)
#define SGM4154X_IINDPM_I_MIN_uA       100000
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
#define SGM4154X_IINDPM_I_MAX_uA       3200000
#else
#define SGM4154X_IINDPM_I_MAX_uA       3800000
#endif
#define SGM4154X_IINDPM_STEP_uA        100000
#define SGM4154X_IINDPM_DEF_uA         2400000

/* vindpm voltage  */
#define SGM4154X_VINDPM_V_MASK         GENMASK(3, 0)
#define SGM4154X_VINDPM_V_MIN_uV       3900000
#define SGM4154X_VINDPM_V_MAX_uV       12000000
#define SGM4154X_VINDPM_STEP_uV        100000
#define SGM4154X_VINDPM_DEF_uV         3600000
#define SGM4154X_VINDPM_OS_MASK        GENMASK(1, 0)

/* fault status */
#define SGM4154X_FAULT_NTC_MASK        GENMASK(2, 0)
#define SGM4154X_FAULT_NTC_WARM        BIT(2)
#define SGM4154X_FAULT_NTC_COOL        (BIT(1) | BIT(0))
#define SGM4154X_FAULT_NTC_COLD        (BIT(2) | BIT(0))
#define SGM4154X_FAULT_NTC_HOT         (BIT(2) | BIT(1))
#define SGM4154X_FAULT_BAT_MASK        BIT(3)
#define SGM4154X_FAULT_CHRG_MASK       GENMASK(5, 4)
#define SGM4154X_FAULT_CHRG_INPUT      BIT(4)
#define SGM4154X_FAULT_CHRG_THERMAL    BIT(5)
#define SGM4154X_FAULT_CHRG_TIMER      (BIT(5) | BIT(4))
#define SGM4154X_FAULT_BOOST_MASK      BIT(6)
#define SGM4154X_FAULT_WDT_MASK        BIT(7)

/* indpm int mask & stat */
#define SGM4154X_IINDPM_INT_MASK       BIT(0)
#define SGM4154X_VINDPM_INT_MASK       BIT(1)
#define SGM4154X_INT_MASK_MASK         GENMASK(1, 0)
#define SGM4154X_IINDPM_STAT           BIT(5)
#define SGM4154X_VINDPM_STAT           BIT(6)

/* DP DM SEL  */
#define SGM4154X_DP_VSEL_MASK          GENMASK(4, 3)
#define SGM4154X_DP_VOLT_SHIFT         3
#define SGM4154X_DM_VSEL_MASK          GENMASK(2, 1)
#define SGM4154X_DM_VOLT_SHIFT         1

/* PUMPX SET  */
#define SGM4154X_EN_PUMPX              BIT(7)
#define SGM4154X_PUMPX_UP              BIT(6)
#define SGM4154X_PUMPX_DN              BIT(5)

#endif /* __SGM4154X_HEADER__ */

