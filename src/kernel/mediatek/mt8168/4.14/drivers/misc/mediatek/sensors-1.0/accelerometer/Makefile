# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include

obj-y   +=  accel.o accel_factory.o

obj-$(CONFIG_MTK_ACCELHUB) += accelhub/
obj-$(CONFIG_MTK_BMI160_I2C)   += bmi160-i2c/
obj-$(CONFIG_MTK_BMA2X2) += bma2x2/
obj-$(CONFIG_MTK_MC3410_I2C) += mc3410-i2c/
obj-$(CONFIG_MTK_MC34XX)   +=  mc34xx/
obj-$(CONFIG_MTK_STK8BAXX)   +=  stk8baxx/
obj-$(CONFIG_MTK_MIR3DA)   +=  mir3da/
obj-$(CONFIG_MTK_LIS2DOC)  +=  lis2doc/
