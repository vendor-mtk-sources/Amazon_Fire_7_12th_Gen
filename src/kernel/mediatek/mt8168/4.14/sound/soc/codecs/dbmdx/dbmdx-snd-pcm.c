/*
 * snd-dbmdx-pcm.c -- DBMDX ASoC platform driver
 *
 * Copyright (C) 2021 DSP Group
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#define DEBUG
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#include <linux/dma-mapping.h>
#include "dbmdx-interface.h"
#if defined(CONFIG_AMZN_METRICS_LOG) || defined(CONFIG_AMZN_MINERVA_METRICS_LOG)
/*add include file for metrics logging through logcat_vital-->KDM*/
#include <linux/amzn_metricslog.h>
#define DBMDX_METRICS_STR_LEN 512
#endif

#define DRV_NAME "dbmdx-snd-soc-platform"

/* defaults */
/* must be a multiple of 4 */
#define MAX_BUFFER_SIZE		(131072*4) /* 3 seconds for each channel */
#define MIN_PERIOD_SIZE		4096
#define MAX_PERIOD_SIZE		(MAX_BUFFER_SIZE / 64)
#define USE_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE)

#ifdef DBMDX_PCM_RATE_8000_SUPPORTED
#define USE_RATE_MIN		8000
#else
#define USE_RATE_MIN		16000
#endif
#define USE_RATE_MAX		48000
#define USE_CHANNELS_MIN	1
#ifdef DBMDX_4CHANNELS_SUPPORT
#define USE_CHANNELS_MAX	4
#else
#define USE_CHANNELS_MAX	2
#endif
#define USE_PERIODS_MIN		1
#define USE_PERIODS_MAX		1024
/* 3 seconds + 4 bytes for position */
#define REAL_BUFFER_SIZE	(MAX_BUFFER_SIZE + 4)

struct snd_dbmdx {
	struct snd_soc_card *card;
	struct snd_pcm_hardware pcm_hw;
};

struct snd_dbmdx_runtime_data {
	struct snd_pcm_substream *substream;
#ifdef TIMER_LIST_PTR
	struct timer_list timer;
#else
	struct timer_list *timer;
#endif /* TIMER_LIST_PTR */
	bool timer_is_active;
	struct delayed_work pcm_start_capture_work;
	struct delayed_work pcm_stop_capture_work;
	struct workqueue_struct		*dbmdx_pcm_workq;
	unsigned int capture_in_progress;
	atomic_t command_in_progress;
	atomic_t number_of_cmds_in_progress;
};

static struct snd_pcm_hardware dbmdx_pcm_hardware = {
	.info =			(SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID |
				 SNDRV_PCM_INFO_BATCH),
	.formats =		USE_FORMATS,
	.rates =		(SNDRV_PCM_RATE_16000 |
#ifdef DBMDX_PCM_RATE_8000_SUPPORTED
				SNDRV_PCM_RATE_8000  |
#endif
#ifdef DBMDX_PCM_RATE_32000_SUPPORTED
				SNDRV_PCM_RATE_32000 |
#endif
#ifdef DBMDX_PCM_RATE_44100_SUPPORTED
				SNDRV_PCM_RATE_44100 |
#endif
				SNDRV_PCM_RATE_48000),
	.rate_min =		USE_RATE_MIN,
	.rate_max =		USE_RATE_MAX,
	.channels_min =		USE_CHANNELS_MIN,
	.channels_max =		USE_CHANNELS_MAX,
	.buffer_bytes_max =	MAX_BUFFER_SIZE,
	.period_bytes_min =	MIN_PERIOD_SIZE,
	.period_bytes_max =	MAX_PERIOD_SIZE,
	.periods_min =		USE_PERIODS_MIN,
	.periods_max =		USE_PERIODS_MAX,
	.fifo_size =		0,
};

extern struct dbmdx_private *dbmdx_data;

static DECLARE_WAIT_QUEUE_HEAD(dbmdx_wq);

int pcm_command_in_progress(struct snd_dbmdx_runtime_data *prtd,
	bool is_command_in_progress)
{
	if (is_command_in_progress) {
		if (!atomic_add_unless(&prtd->command_in_progress, 1, 1))
			return -EBUSY;
	} else {
		atomic_set(&prtd->command_in_progress, 0);
		atomic_dec(&prtd->number_of_cmds_in_progress);
		wake_up_interruptible(&dbmdx_wq);
	}

	return 0;
}

void wait_for_pcm_commands(struct snd_dbmdx_runtime_data *prtd)
{
	int ret;

	while (1) {
		wait_event_interruptible(dbmdx_wq,
			!(atomic_read(&prtd->command_in_progress)));

		ret = pcm_command_in_progress(prtd, 1);
		if (!ret)
			break;
	}
}

u32 stream_get_position(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	/* pr_debug("%s\n", __func__); */

	if (runtime == NULL) {
		pr_err("%s: NULL ptr runtime\n", __func__);
		return 0;
	}

	return *(u32 *)&(runtime->dma_area[MAX_BUFFER_SIZE]);
}

void stream_set_position(struct snd_pcm_substream *substream,
				u32 position)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	/* pr_debug("%s\n", __func__); */

	if (runtime == NULL) {
		pr_err("%s: NULL ptr runtime\n", __func__);
		return;
	}

	*(u32 *)&(runtime->dma_area[MAX_BUFFER_SIZE]) = position;
}

#ifdef TIMER_LIST_PTR
static void dbmdx_pcm_timer(struct timer_list *t)
#else
static void dbmdx_pcm_timer(unsigned long _substream)
#endif /* TIMER_LIST_PTR */
{
#ifdef TIMER_LIST_PTR
	struct snd_dbmdx_runtime_data *prtd = from_timer(prtd, t, timer);
	struct snd_pcm_substream *substream = prtd->substream;
#else
	struct snd_pcm_substream *substream =
				(struct snd_pcm_substream *)_substream;
#endif /* TIMER_LIST_PTR */
	struct snd_pcm_runtime *runtime = substream->runtime;
#ifdef TIMER_LIST_PTR
	struct timer_list *timer = &(prtd->timer);
#else
	struct snd_dbmdx_runtime_data *prtd = runtime->private_data;
	struct timer_list *timer = prtd->timer;
#endif /* TIMER_LIST_PTR */
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
#ifdef SND_SOC_COMPONENT
#ifdef USE_KERNEL_ABOVE_5_10
	struct snd_soc_component *component = asoc_rtd_to_codec(rtd, 0)->component;
#else
	struct snd_soc_component *component = rtd->codec_dai->component;
#endif
#else
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
#endif /* SND_SOC_COMPONENT */

	unsigned int size = snd_pcm_lib_buffer_bytes(substream);
	u32 pos;
	unsigned long msecs;
	unsigned long to_copy;

	msecs = (runtime->period_size * 1000) / runtime->rate;
	mod_timer(timer, jiffies + msecs_to_jiffies(msecs));
	/* pr_debug("%s\n", __func__); */


	pos = stream_get_position(substream);
	to_copy = frames_to_bytes(runtime, runtime->period_size);

#ifdef SND_SOC_COMPONENT
	if (dbmdx_get_samples(component, runtime->dma_area + pos,
#else
	if (dbmdx_get_samples(codec, runtime->dma_area + pos,
#endif /* SND_SOC_COMPONENT */
		runtime->channels * runtime->period_size)) {
		memset(runtime->dma_area + pos, 0, to_copy);
		pr_debug("%s Inserting %d bytes of silence\n",
			__func__, (int)to_copy);
	}

	pos += to_copy;
	if (pos >= size)
		pos = 0;

	stream_set_position(substream, pos);

	snd_pcm_period_elapsed(substream);

}

#ifdef USE_KERNEL_ABOVE_5_10
static int dbmdx_pcm_hw_params(struct snd_soc_component *component,
				struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
#else
static int dbmdx_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
#endif

{
	struct snd_pcm_runtime *runtime = substream->runtime;


	pr_debug("%s\n", __func__);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	runtime->channels    = params_channels(hw_params);
	runtime->dma_bytes   = params_buffer_bytes(hw_params);
	runtime->buffer_size = params_buffer_size(hw_params);
	runtime->rate = params_rate(hw_params);

	return 0;
}

#ifdef USE_KERNEL_ABOVE_5_10
static int dbmdx_pcm_prepare(struct snd_soc_component *component, struct snd_pcm_substream *substream)
#else
static int dbmdx_pcm_prepare(struct snd_pcm_substream *substream)
#endif
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	size_t	buf_bytes;
	size_t	period_bytes;

	pr_debug("%s\n", __func__);

	memset(runtime->dma_area, 0, REAL_BUFFER_SIZE);

	buf_bytes = snd_pcm_lib_buffer_bytes(substream);
	period_bytes = snd_pcm_lib_period_bytes(substream);

	pr_debug("%s - buffer size =%d period size = %d\n",
		       __func__, (int)buf_bytes, (int)period_bytes);

	/* We only support buffers that are multiples of the period */
	if (buf_bytes % period_bytes) {
		pr_err("%s - buffer=%d not multiple of period=%d\n",
		       __func__, (int)buf_bytes, (int)period_bytes);
		return -EINVAL;
	}

	return 0;
}

static int dbmdx_start_period_timer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dbmdx_runtime_data *prtd = runtime->private_data;
#ifdef TIMER_LIST_PTR
	struct timer_list *timer = &(prtd->timer);
#else
	struct timer_list *timer = prtd->timer;
#endif /* TIMER_LIST_PTR */
	unsigned long msecs;

	pr_debug("%s\n", __func__);
	prtd->timer_is_active = true;

	*(u32 *)&(runtime->dma_area[MAX_BUFFER_SIZE]) = 0;
	msecs = (runtime->period_size * 500) / runtime->rate;
	mod_timer(timer, jiffies + msecs_to_jiffies(msecs));

	return 0;
}

static int dbmdx_stop_period_timer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dbmdx_runtime_data *prtd = runtime->private_data;
#ifdef TIMER_LIST_PTR
	struct timer_list *timer = &(prtd->timer);
#else
	struct timer_list *timer = prtd->timer;
#endif /* TIMER_LIST_PTR */

	pr_debug("%s\n", __func__);


	del_timer_sync(timer);

	prtd->timer_is_active = false;

	return 0;
}


int dbmdx_set_pcm_timer_mode(struct snd_pcm_substream *substream,
				bool enable_timer)
{
	int ret;
	struct snd_pcm_runtime *runtime;
	struct snd_dbmdx_runtime_data *prtd;

	if (!substream) {
		pr_debug("%s:Substream is NULL\n", __func__);
		return -EINVAL;
	}

	runtime = substream->runtime;

	if (!runtime) {
		pr_debug("%s:Runtime is NULL\n", __func__);
		return -EINVAL;
	}

	prtd = runtime->private_data;

	if (!prtd) {
		pr_debug("%s:Runtime Pr. Data is NULL\n", __func__);
		return -EINVAL;
	}

	if (enable_timer) {
		if (!(prtd->capture_in_progress)) {
			pr_debug("%s:Capture is not in progress\n", __func__);
			return -EINVAL;
		}

		if (prtd->timer_is_active) {
			pr_debug("%s:Timer is active\n", __func__);
			return 0;
		}

		ret = dbmdx_start_period_timer(substream);
		if (ret < 0) {
			pr_err("%s: failed to start capture device\n",
				__func__);
			return -EIO;
		}
	} else {
		if (!(prtd->timer_is_active)) {
			pr_debug("%s:Timer is not active\n", __func__);
			return 0;
		}

		ret = dbmdx_stop_period_timer(substream);
		if (ret < 0) {
			pr_err("%s: failed to stop capture device\n", __func__);
			return -EIO;
		}

	}

	return 0;
}

unsigned int dbmdx_pcm_get_capture_status(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct snd_dbmdx_runtime_data *prtd;
	unsigned int capture_in_progress = 0;

	if (substream != NULL) {
		runtime = substream->runtime;
		if (runtime != NULL) {
			prtd = runtime->private_data;
			if (prtd != NULL)
				capture_in_progress = prtd->capture_in_progress;
		}
	}
	pr_debug("%s: capture_in_progress = %d\n",
		__func__, capture_in_progress);
	return capture_in_progress;
}

static void  dbmdx_pcm_start_capture_work(struct work_struct *work)
{
	int ret;
	struct snd_dbmdx_runtime_data *prtd = container_of(
			work, struct snd_dbmdx_runtime_data,
			pcm_start_capture_work.work);
	struct snd_pcm_substream *substream = prtd->substream;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
#ifdef SND_SOC_COMPONENT
#ifdef USE_KERNEL_ABOVE_5_10
	struct snd_soc_component *component = asoc_rtd_to_codec(rtd, 0)->component;
#else
	struct snd_soc_component *component = rtd->codec_dai->component;
#endif
#else
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
#endif /* SND_SOC_COMPONENT */


	pr_debug("%s:\n", __func__);

	wait_for_pcm_commands(prtd);

	if (prtd->capture_in_progress) {
		pr_debug("%s:Capture is already in progress\n", __func__);
		goto out;
	}

	prtd->capture_in_progress = 1;

#ifdef SND_SOC_COMPONENT
	ret = dbmdx_start_pcm_streaming(component, substream);
#else
	ret = dbmdx_start_pcm_streaming(codec, substream);
#endif /* SND_SOC_COMPONENT */
	if (ret < 0) {
		prtd->capture_in_progress = 0;
		pr_err("%s: failed to start capture device\n", __func__);
		goto out;
	}

	msleep(DBMDX_MSLEEP_PCM_STREAMING_WORK);
out:
	pcm_command_in_progress(prtd, 0);
}

static void dbmdx_pcm_stop_capture_work(struct work_struct *work)
{
	int ret;
	struct snd_dbmdx_runtime_data *prtd = container_of(
			work, struct snd_dbmdx_runtime_data,
			pcm_stop_capture_work.work);
	struct snd_pcm_substream *substream = prtd->substream;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
#ifdef SND_SOC_COMPONENT
#ifdef USE_KERNEL_ABOVE_5_10
	struct snd_soc_component *component = asoc_rtd_to_codec(rtd, 0)->component;
#else
	struct snd_soc_component *component = rtd->codec_dai->component;
#endif
#else
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
#endif /* SND_SOC_COMPONENT */

	pr_debug("%s:\n", __func__);

	wait_for_pcm_commands(prtd);

	if (!(prtd->capture_in_progress)) {
		pr_debug("%s:Capture is not in progress\n", __func__);
		goto out;
	}

#ifdef SND_SOC_COMPONENT
	ret = dbmdx_stop_pcm_streaming(component);
#else
	ret = dbmdx_stop_pcm_streaming(codec);
#endif /* SND_SOC_COMPONENT */
	if (ret < 0)
		pr_err("%s: failed to stop pcm streaming\n", __func__);

	if (prtd->timer_is_active) {

		ret = dbmdx_stop_period_timer(substream);
		if (ret < 0)
			pr_err("%s: failed to stop timer\n", __func__);
	}

	prtd->capture_in_progress = 0;
out:
	pcm_command_in_progress(prtd, 0);
}

#ifdef USE_KERNEL_ABOVE_5_10
static int dbmdx_pcm_open(struct snd_soc_component *component, struct snd_pcm_substream *substream)
#else
static int dbmdx_pcm_open(struct snd_pcm_substream *substream)
#endif
{
#ifdef CONFIG_AMZN_MINERVA_METRICS_LOG
	char minerva_buf[DBMDX_METRICS_STR_LEN];
#endif
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dbmdx_runtime_data *prtd;
#ifndef USE_KERNEL_ABOVE_5_10
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component = rtd->codec_dai->component;
#else
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
#endif /* SND_SOC_COMPONENT */
#endif
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#else
	struct dbmdx_private *p = dbmdx_data;
#endif /* !USE_KERNEL_ABOVE_5_10 */

	struct timer_list *timer;
	int ret;

	pr_debug("%s\n", __func__);

	if (!p->device_ready) {
		pr_err("%s: device not ready\n", __func__);
		return -EFAULT;
	}

#ifdef SND_SOC_COMPONENT
	if (dbmdx_component_lock(component)) {
#else
	if (dbmdx_codec_lock(codec)) {
#endif /* SND_SOC_COMPONENT */
		ret = -EBUSY;
		goto out;
	}

	prtd = kzalloc(sizeof(struct snd_dbmdx_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto out_unlock;
	}

#ifdef TIMER_LIST_PTR
	timer = &(prtd->timer);
	timer_setup(timer, dbmdx_pcm_timer, 0);
#else
	timer = kzalloc(sizeof(*timer), GFP_KERNEL);
	if (!timer) {
		ret = -ENOMEM;
		goto out_free_prtd;
	}

	p->usr_op_lock(p);

	init_timer(timer);
	timer->function = dbmdx_pcm_timer;
	timer->data = (unsigned long)substream;

	prtd->timer = timer;
#endif /* TIMER_LIST_PTR */
	prtd->substream = substream;
	atomic_set(&prtd->command_in_progress, 0);
	atomic_set(&prtd->number_of_cmds_in_progress, 0);

	INIT_DELAYED_WORK(&prtd->pcm_start_capture_work,
		dbmdx_pcm_start_capture_work);
	INIT_DELAYED_WORK(&prtd->pcm_stop_capture_work,
		dbmdx_pcm_stop_capture_work);
	prtd->dbmdx_pcm_workq = create_workqueue("dbmdx-pcm-wq");
	if (!prtd->dbmdx_pcm_workq) {
		pr_err("%s: Could not create pcm workqueue\n", __func__);
		ret = -EIO;
		p->usr_op_unlock(p);
#ifdef TIMER_LIST_PTR
		goto out_free_prtd;
#else
		goto out_free_timer;
#endif /* TIMER_LIST_PTR */
	}

	runtime->private_data = prtd;

	snd_soc_set_runtime_hwparams(substream, &dbmdx_pcm_hardware);

	ret = snd_pcm_hw_constraint_integer(runtime,
					SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		pr_debug("%s Error setting pcm constraint int\n", __func__);

	p->usr_op_unlock(p);

	p->fgMetricLogPrint = false;
	p->StreamOpenTime = ktime_get();
	p->dspStreamDuration = 0;
#ifdef CONFIG_AMZN_METRICS_LOG
	log_counter_to_vitals(ANDROID_LOG_INFO, "Kernel", "Kernel",
			"DBMD4_DSP_metrics_count", "DSP_DATA_PROCESS_BEGIN", 1, "count", NULL, VITALS_NORMAL);
#endif

#ifdef CONFIG_AMZN_MINERVA_METRICS_LOG
	minerva_metrics_log(minerva_buf, DBMDX_METRICS_STR_LEN,
			"%s:%s:100:%s,%s,%s,DSP_IRQ=false;BO,DSP_RESET=false;BO,"
			"DSP_WDT=false;BO,DSP_DATA_PROCESS_BEGIN=true;BO:us-east-1",
			METRICS_DSP_GROUP_ID, METRICS_DSP_VOICE_SCHEMA_ID,
			PREDEFINED_ESSENTIAL_KEY, PREDEFINED_DEVICE_ID_KEY, PREDEFINED_DEVICE_LANGUAGE_KEY);
	minerva_counter_to_vitals(ANDROID_LOG_INFO,
			VITALS_DSP_GROUP_ID, VITALS_DSP_COUNTER_SCHEMA_ID,
			"Kernel", "Kernel", "DBMD4_DSP_metrics_count",
			"DSP_DATA_PROCESS_BEGIN", 1, "count",
			NULL, VITALS_NORMAL, NULL, NULL);
#endif

	return 0;

#ifndef TIMER_LIST_PTR
out_free_timer:
	kfree(timer);
#endif /* !TIMER_LIST_PTR */
out_free_prtd:
	kfree(prtd);
out_unlock:
#ifdef SND_SOC_COMPONENT
	dbmdx_component_unlock(component);
#else
	dbmdx_codec_unlock(codec);
#endif /* SND_SOC_COMPONENT */
out:
	return ret;
}

#ifdef USE_KERNEL_ABOVE_5_10
static int dbmdx_pcm_trigger(struct snd_soc_component *component, struct snd_pcm_substream *substream, int cmd)
#else
static int dbmdx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
#endif
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dbmdx_runtime_data *prtd;
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#else
	struct dbmdx_private *p = dbmdx_data;
#endif
	int ret = 0;
	int num_of_active_cmds;

	pr_debug("%s: cmd=%d\n", __func__, cmd);

	if (!p->device_ready) {
		pr_err("%s: device not ready\n", __func__);
		return -EFAULT;
	}

	if (runtime == NULL) {
		pr_err("%s: runtime NULL ptr\n", __func__);
		return -EFAULT;
	}

	prtd = runtime->private_data;

	if (prtd == NULL) {
		pr_err("%s: prtd NULL ptr\n", __func__);
		return -EFAULT;
	}

	num_of_active_cmds = atomic_read(&prtd->number_of_cmds_in_progress);
	pr_debug("%s: Number of active commands=%d\n", __func__,
		num_of_active_cmds);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		atomic_inc(&prtd->number_of_cmds_in_progress);
		ret = queue_delayed_work(prtd->dbmdx_pcm_workq,
			&prtd->pcm_start_capture_work,
			msecs_to_jiffies(num_of_active_cmds*100));
		if (!ret) {
			pr_debug("%s: Start command is already pending\n",
				__func__);
			atomic_dec(&prtd->number_of_cmds_in_progress);
		} else
			pr_debug("%s: Start has been scheduled\n", __func__);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		atomic_inc(&prtd->number_of_cmds_in_progress);
		ret = queue_delayed_work(prtd->dbmdx_pcm_workq,
			&prtd->pcm_stop_capture_work,
			msecs_to_jiffies(num_of_active_cmds*100));
		if (!ret) {
			pr_debug("%s: Stop command is already pending\n",
				__func__);
			atomic_dec(&prtd->number_of_cmds_in_progress);
		} else
			pr_debug("%s: Stop has been scheduled\n", __func__);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return 0;
	}

	return ret;
}

#ifdef USE_KERNEL_ABOVE_5_10
static int dbmdx_pcm_close(struct snd_soc_component *component, struct snd_pcm_substream *substream)
#else
static int dbmdx_pcm_close(struct snd_pcm_substream *substream)
#endif
{
#ifdef CONFIG_AMZN_MINERVA_METRICS_LOG
	char minerva_buf[DBMDX_METRICS_STR_LEN];
#endif
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dbmdx_runtime_data *prtd = runtime->private_data;
#ifndef USE_KERNEL_ABOVE_5_10
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component = rtd->codec_dai->component;
	struct dbmdx_private *p = dev_get_drvdata(component->dev);
#else
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct dbmdx_private *p = dev_get_drvdata(codec->dev);
#endif /* SND_SOC_COMPONENT */
#endif /* !USE_KERNEL_ABOVE_5_10 */

#ifndef TIMER_LIST_PTR
	struct timer_list *timer = prtd->timer;
#endif /* !TIMER_LIST_PTR */

#if defined(CONFIG_AMZN_METRICS_LOG) || defined(CONFIG_AMZN_MINERVA_METRICS_LOG)
	ktime_t Current;
#endif

	pr_debug("%s\n", __func__);

	flush_delayed_work(&prtd->pcm_start_capture_work);
	flush_delayed_work(&prtd->pcm_stop_capture_work);
	queue_delayed_work(prtd->dbmdx_pcm_workq,
		&prtd->pcm_stop_capture_work,
		msecs_to_jiffies(0));
	flush_delayed_work(&prtd->pcm_stop_capture_work);
	flush_workqueue(prtd->dbmdx_pcm_workq);
	usleep_range(10000, 11000);
	destroy_workqueue(prtd->dbmdx_pcm_workq);
#ifndef TIMER_LIST_PTR
	kfree(timer);
	timer = NULL;
#endif /* !TIMER_LIST_PTR */
	kfree(prtd);
	prtd = NULL;

	if (p->fgMetricLogPrint == false) {
		p->fgMetricLogPrint = true;
#ifdef CONFIG_AMZN_METRICS_LOG
		Current = ktime_get();
		log_timer_to_vitals(ANDROID_LOG_INFO, "Kernel", "Kernel",
			"DBMD4_DSP_metrics_time", "DSP_DATA_CANCEL",
			ktime_to_ms(Current)  - ktime_to_ms(p->StreamOpenTime),
			"ms", VITALS_NORMAL);
#endif

#ifdef CONFIG_AMZN_MINERVA_METRICS_LOG
		Current = ktime_get();
		minerva_metrics_log(minerva_buf, DBMDX_METRICS_STR_LEN,
			"%s:%s:100:%s,%s,%s,DSP_DATA_CATCH_UP_FINISH=%d;IN,"
			"DSP_DATA_PROCESS_FINISH=%d;IN,DSP_DATA_CANCEL=%d;IN:us-east-1",
			METRICS_DSP_GROUP_ID, METRICS_DSP_CATCH_SCHEMA_ID,
			PREDEFINED_ESSENTIAL_KEY, PREDEFINED_DEVICE_ID_KEY, PREDEFINED_DEVICE_LANGUAGE_KEY,
			-1, -1, ktime_to_ms(Current) - ktime_to_ms(p->StreamOpenTime));
		minerva_timer_to_vitals(ANDROID_LOG_INFO,
			VITALS_DSP_GROUP_ID, VITALS_DSP_TIMER_SCHEMA_ID,
			"Kernel", "Kernel",
			"DBMD4_DSP_metrics_time", "DSP_DATA_CANCEL",
			ktime_to_ms(Current)  - ktime_to_ms(p->StreamOpenTime),
			"ms", VITALS_NORMAL, NULL, NULL);
#endif
}

#ifdef SND_SOC_COMPONENT
	dbmdx_component_unlock(component);
#else
	dbmdx_codec_unlock(codec);
#endif /* SND_SOC_COMPONENT */

	return 0;
}

#ifdef USE_KERNEL_ABOVE_5_10
int dbmdx_pcm_ioctl(struct snd_soc_component *component, struct snd_pcm_substream *substream,
		      unsigned int cmd, void *arg)
{
	return snd_pcm_lib_ioctl(substream, cmd, arg);
}
#endif

#ifdef USE_KERNEL_ABOVE_5_10
static snd_pcm_uframes_t dbmdx_pcm_pointer(struct snd_soc_component *component, struct snd_pcm_substream *substream)
#else
static snd_pcm_uframes_t dbmdx_pcm_pointer(struct snd_pcm_substream *substream)
#endif
{
	u32 pos;

	/* pr_debug("%s\n", __func__); */


	pos = stream_get_position(substream);
	return bytes_to_frames(substream->runtime, pos);
}

#ifndef USE_KERNEL_ABOVE_5_10
static struct snd_pcm_ops dbmdx_pcm_ops = {
	.open		= dbmdx_pcm_open,
	.close		= dbmdx_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= dbmdx_pcm_hw_params,
	.prepare	= dbmdx_pcm_prepare,
	.trigger	= dbmdx_pcm_trigger,
	.pointer	= dbmdx_pcm_pointer,
};
#endif

static int dbmdx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = MAX_BUFFER_SIZE;

	pr_debug("%s\n", __func__);


	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev,
				       REAL_BUFFER_SIZE,
				       &buf->addr,
				       GFP_KERNEL);
	if (!buf->area) {
		pr_err("%s: Failed to allocate dma memory.\n", __func__);
		pr_err("%s: Please increase uncached DMA memory region\n",
			__func__);
		return -ENOMEM;
	}
	buf->bytes = size;

	return 0;
}

#ifdef SND_SOC_COMPONENT
static int dbmdx_pcm_probe(struct snd_soc_component *c)
#else
static int dbmdx_pcm_probe(struct snd_soc_platform *pt)
#endif /* SND_SOC_COMPONENT */
{
	struct snd_dbmdx *dbmdx;

	pr_debug("%s\n", __func__);


	dbmdx = kzalloc(sizeof(*dbmdx), GFP_KERNEL);
	if (!dbmdx)
		return -ENOMEM;
#ifdef SND_SOC_COMPONENT
	dbmdx->card = c->card;
#else
#if USE_ALSA_API_3_10_XX
	dbmdx->card = pt->card;
#else
	dbmdx->card = pt->component.card;
#endif
#endif /* SND_SOC_COMPONENT */
	dbmdx->pcm_hw = dbmdx_pcm_hardware;
#ifdef SND_SOC_COMPONENT
	snd_soc_component_set_drvdata(c, dbmdx);
#else
	snd_soc_platform_set_drvdata(pt, dbmdx);
#endif /* SND_SOC_COMPONENT */

	return 0;
}

#ifdef SND_SOC_COMPONENT
static void dbmdx_pcm_remove(struct snd_soc_component *c)
#else
static int dbmdx_pcm_remove(struct snd_soc_platform *pt)
#endif /* SND_SOC_COMPONENT */
{
	struct snd_dbmdx *dbmdx;

	pr_debug("%s\n", __func__);


#ifdef SND_SOC_COMPONENT
	dbmdx = snd_soc_component_get_drvdata(c);
#else
	dbmdx = snd_soc_platform_get_drvdata(pt);
#endif /* SND_SOC_COMPONENT */
	kfree(dbmdx);
#ifndef SND_SOC_COMPONENT
	return 0;
#endif
}

#ifdef USE_KERNEL_ABOVE_5_10
static int dbmdx_pcm_construct(struct snd_soc_component *component, struct snd_soc_pcm_runtime *runtime)
#else
static int dbmdx_pcm_new(struct snd_soc_pcm_runtime *runtime)
#endif
{
	struct snd_pcm *pcm;
	int ret = 0;

	pr_debug("%s\n", __func__);


	pcm = runtime->pcm;
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = dbmdx_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = dbmdx_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
out:
	return ret;
}

#ifdef USE_KERNEL_ABOVE_5_10
static void dbmdx_pcm_destruct(struct snd_soc_component *component, struct snd_pcm *pcm)
#else
static void dbmdx_pcm_free(struct snd_pcm *pcm)
#endif
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	pr_debug("%s\n", __func__);


	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		dma_free_coherent(pcm->card->dev,
				  REAL_BUFFER_SIZE,
				  (void *)buf->area,
				  buf->addr);
		buf->area = NULL;
	}
}

#ifdef SND_SOC_COMPONENT
static struct snd_soc_component_driver dbmdx_soc_component_drv = {
#else
static struct snd_soc_platform_driver dbmdx_soc_platform = {
#endif /* SND_SOC_COMPONENT */
	.probe		= &dbmdx_pcm_probe,
	.remove		= &dbmdx_pcm_remove,
#ifdef USE_KERNEL_ABOVE_5_10
	.open		= dbmdx_pcm_open,
	.close		= dbmdx_pcm_close,
	.ioctl			= dbmdx_pcm_ioctl,
	.hw_params	= dbmdx_pcm_hw_params,
	.prepare	= dbmdx_pcm_prepare,
	.trigger	= dbmdx_pcm_trigger,
	.pointer	= dbmdx_pcm_pointer,
	.pcm_construct	= dbmdx_pcm_construct,
	.pcm_destruct		= dbmdx_pcm_destruct,
#else
	.ops		= &dbmdx_pcm_ops,
	.pcm_new	= dbmdx_pcm_new,
	.pcm_free	= dbmdx_pcm_free,
#endif
};

static int dbmdx_pcm_platform_probe(struct platform_device *pdev)
{
	int err;

	pr_debug("%s\n", __func__);

#ifdef SND_SOC_COMPONENT
	err = snd_soc_register_component(&pdev->dev, &dbmdx_soc_component_drv, NULL, 0);
#else
	err = snd_soc_register_platform(&pdev->dev, &dbmdx_soc_platform);
#endif /* SND_SOC_COMPONENT */
	if (err)
		dev_err(&pdev->dev, "%s: snd_soc_register_platform() failed",
			__func__);

	return err;
}

static int dbmdx_pcm_platform_remove(struct platform_device *pdev)
{
#ifdef SND_SOC_COMPONENT
	snd_soc_unregister_component(&pdev->dev);
#else
	snd_soc_unregister_platform(&pdev->dev);
#endif /* SND_SOC_COMPONENT */

	pr_debug("%s\n", __func__);


	return 0;
}

static const struct of_device_id snd_soc_platform_of_ids[] = {
	{ .compatible = "dspg,dbmdx-snd-soc-platform" },
	{ },
};

static struct platform_driver dbmdx_pcm_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = snd_soc_platform_of_ids,
	},
	.probe = dbmdx_pcm_platform_probe,
	.remove = dbmdx_pcm_platform_remove,
};

#ifdef CONFIG_SND_SOC_DBMDX
static int __init snd_dbmdx_pcm_init(void)
{
	return platform_driver_register(&dbmdx_pcm_driver);
}
late_initcall(snd_dbmdx_pcm_init);

static void __exit snd_dbmdx_pcm_exit(void)
{
	platform_driver_unregister(&dbmdx_pcm_driver);
}
module_exit(snd_dbmdx_pcm_exit);
#else
int snd_dbmdx_pcm_init(void)
{
	return platform_driver_register(&dbmdx_pcm_driver);
}

void snd_dbmdx_pcm_exit(void)
{
	platform_driver_unregister(&dbmdx_pcm_driver);
}
#endif

MODULE_DESCRIPTION("DBMDX ASoC platform driver");
MODULE_AUTHOR("DSP Group");
MODULE_LICENSE("GPL");
