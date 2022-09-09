$(info $$CONFIG_SND_SOC_DBMDX is [${CONFIG_SND_SOC_DBMDX}])
ifeq ($(CONFIG_SND_SOC_DBMDX),y)
	obj-$(CONFIG_SND_SOC_DBMDX) := dbmdx.o
	obj-$(CONFIG_SND_SOC_DBMDX) += dbmdx-spi.o
	obj-$(CONFIG_SND_SOC_DBMDX) += dbmdx-spi-d4.o
	obj-$(CONFIG_SND_SOC_DBMDX) += dbmdx-customer.o
	obj-$(CONFIG_SND_SOC_DBMDX) += dbmdx-cdev.o
	ifeq ($(CONFIG_SND_SOC_DBMDX_SND_CAPTURE),y)
		obj-$(CONFIG_SND_SOC_DBMDX_SND_CAPTURE) += dbmdx-snd.o
		obj-$(CONFIG_SND_SOC_DBMDX_SND_CAPTURE) += dbmdx-snd-pcm.o
	endif
else
	ifeq ($(CONFIG_SND_SOC_DBMDX),m)
		obj-$(CONFIG_SND_SOC_DBMDX) := dbmdx-codec.o
		dbmdx-codec-objs := dbmdx.o dbmdx-spi.o dbmdx-spi-d4.o dbmdx-customer.o dbmdx-cdev.o
		ifeq ($(CONFIG_SND_SOC_DBMDX_SND_CAPTURE),y)
			dbmdx-codec-objs += dbmdx-snd.o
			dbmdx-codec-objs += dbmdx-snd-pcm.o
		endif
	endif
endif

# Adf dsp firmware
ccflags-$(CONFIG_AMAZON_DSP_FRAMEWORK) += -I$(srctree)/drivers/staging/amazon/dspframework/include/
