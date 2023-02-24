/*
 * mt8168-quartz.c  --  MT8168 machine driver
 *
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Jia Zeng <jia.zeng@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "mt8168-afe-common.h"
#ifdef CONFIG_MTK_HIFIXDSP_SUPPORT
#include "mt8168-adsp-utils.h"
#endif

#define PREFIX	"mediatek,"

#define TEST_BACKEND_WITH_ENDPOINT

enum SPK_PA_SKU_ID {
	EXT_AMP_ALC105 = 0,
	EXT_AMP_TPA2011,
	EXT_AMP_AD51562
};

enum PINCTRL_PIN_STATE {
	PIN_STATE_MOSI_ON = 0,
	PIN_STATE_MOSI_OFF,
	PIN_STATE_MISO_ON,
	PIN_STATE_MISO_OFF,
	PIN_STATE_DEFAULT,
	PIN_STATE_DMIC,
	PIN_STATE_PCM,
	PIN_STATE_I2S0,
	PIN_STATE_I2S1,
	PIN_STATE_I2S2,
	PIN_STATE_I2S3,
	PIN_STATE_TDM_OUT,
	PIN_STATE_TDM_IN,
	PIN_STATE_MAX
};

static const char * const mt8168_quartz_pin_str[PIN_STATE_MAX] = {
	"aud_mosi_on",
	"aud_mosi_off",
	"aud_miso_on",
	"aud_miso_off",
	"default",
	"aud_dmic",
	"aud_pcm",
	"aud_i2s0",
	"aud_i2s1",
	"aud_i2s2",
	"aud_i2s3",
	"aud_tdm_out",
	"aud_tdm_in",
};

struct mt8168_quartz_dmic_ctrl_data {
	unsigned int fix_rate;
	unsigned int fix_channels;
	unsigned int fix_bit_width;
};

struct mt8168_quartz_tdm_ctrl_data {
	unsigned int tdmin_fix_rate;
	unsigned int tdmin_fix_bit_width;
	unsigned int tdmin_fix_channels;
};

struct mt8168_quartz_priv {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_states[PIN_STATE_MAX];
	int tab_spk_en_gpio;
	int tab_spk_en2_gpio;
	int spk_pa_sku_id_gpio;
	int spk_pa_sku_id2_gpio;
	int spk_pa_5v_en_gpio;
	int apa_sdn_gpio;
	int spk_pa_gain1_gpio;
	int spk_pa_gain2_gpio;
	enum SPK_PA_SKU_ID spk_pa_id;
	unsigned int ext_spk_amp_warmup_time_us;
	unsigned int ext_spk_amp_shutdown_time_us;
	unsigned int ext_spk_amp_vdd_on_time_us;
	struct mt8168_quartz_dmic_ctrl_data dmic_data;
	struct mt8168_quartz_tdm_ctrl_data tdm_data;
};

struct mt8168_dai_link_prop {
	char *name;
	unsigned int link_id;
};

enum {
	/* FE */
	DAI_LINK_DL1_PLAYBACK = 0,
	DAI_LINK_DL2_PLAYBACK,
	DAI_LINK_AWB_CAPTURE,
	DAI_LINK_VUL_CAPTURE,
	DAI_LINK_VUL2_CAPTURE,
	DAI_LINK_VUL3_CAPTURE,
	DAI_LINK_TDM_OUT,
	DAI_LINK_TDM_IN,
#ifdef CONFIG_SND_SOC_MTK_BTCVSD
	DAI_LINK_BTCVSD,
#endif
#ifdef CONFIG_MTK_HIFIXDSP_SUPPORT
	DAI_LINK_FE_HOSTLESS_VA,
	DAI_LINK_FE_VA,
	DAI_LINK_FE_MIC_RECORD,
#endif
	/* BE */
	DAI_LINK_I2S_INTF,
	DAI_LINK_2ND_I2S_INTF,
	DAI_LINK_PCM1_INTF,
	DAI_LINK_VIRTUAL_DL_SOURCE,
	DAI_LINK_VIRTUAL_TDM_OUT_SOURCE,
	DAI_LINK_DMIC,
	DAI_LINK_INT_ADDA,
	DAI_LINK_TDM_OUT_IO,
	DAI_LINK_TDM_IN_IO,
	DAI_LINK_GASRC1,
	DAI_LINK_GASRC2,
	DAI_LINK_TDM_ASRC,
#ifdef CONFIG_MTK_HIFIXDSP_SUPPORT
	DAI_LINK_BE_VUL2_IN,
	DAI_LINK_BE_TDM_IN,
	DAI_LINK_BE_VUL_IN,
#endif
#ifdef CONFIG_SND_SOC_RT551X
	DAI_LINK_FE_SPI32765,
#endif
	DAI_LINK_NUM
};

#ifdef CONFIG_MTK_HIFIXDSP_SUPPORT
static const unsigned int adsp_dai_links[] = {
	DAI_LINK_FE_HOSTLESS_VA,
	DAI_LINK_FE_VA,
	DAI_LINK_FE_MIC_RECORD,
	DAI_LINK_BE_VUL2_IN,
	DAI_LINK_BE_TDM_IN,
	DAI_LINK_BE_VUL_IN
};

static bool is_adsp_dai_link(unsigned int dai)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(adsp_dai_links); i++)
		if (dai == adsp_dai_links[i])
			return true;

	return false;
}
#endif

static unsigned int spkr_gain;

const char * const EXT_SPK_PGA_GAIN[] = {"5dB", "8dB", "13dB", "19dB"};

static const struct soc_enum mt8168_quartz_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(EXT_SPK_PGA_GAIN), EXT_SPK_PGA_GAIN),
};

static int mt8168_qua_ext_speaker_pga_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = spkr_gain;
	return 0;
}

static int mt8168_qua_ext_speaker_pga_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct mt8168_quartz_priv *card_data =
		snd_soc_card_get_drvdata(card);

	if (card_data->spk_pa_id == EXT_AMP_TPA2011 ||
		card_data->spk_pa_id == EXT_AMP_AD51562 ||
		!gpio_is_valid(card_data->spk_pa_gain1_gpio) ||
		!gpio_is_valid(card_data->spk_pa_gain2_gpio)) {
		dev_dbg(card->dev, "%s, speaker amp determined by hw\n", __func__);
		return 0;
	}

	if (ucontrol->value.enumerated.item[0] >= ARRAY_SIZE(EXT_SPK_PGA_GAIN)) {
		dev_err(card->dev, "%s value set error\n", __func__);
		return -EINVAL;
	}

	spkr_gain = ucontrol->value.integer.value[0];

	dev_dbg(card->dev, "%s spkr_gain %d\n", __func__, spkr_gain);

	switch (spkr_gain) {
	case 0:
		if (gpio_is_valid(card_data->spk_pa_gain1_gpio))
			gpio_set_value(card_data->spk_pa_gain1_gpio, 0);
		if (gpio_is_valid(card_data->spk_pa_gain2_gpio))
			gpio_set_value(card_data->spk_pa_gain2_gpio, 0);
		break;
	case 1:
		if (gpio_is_valid(card_data->spk_pa_gain1_gpio))
			gpio_set_value(card_data->spk_pa_gain1_gpio, 1);
		if (gpio_is_valid(card_data->spk_pa_gain2_gpio))
			gpio_set_value(card_data->spk_pa_gain2_gpio, 0);
		break;
	case 2:
		if (gpio_is_valid(card_data->spk_pa_gain1_gpio))
			gpio_set_value(card_data->spk_pa_gain1_gpio, 0);
		if (gpio_is_valid(card_data->spk_pa_gain2_gpio))
			gpio_set_value(card_data->spk_pa_gain2_gpio, 1);
		break;
	case 3:
		if (gpio_is_valid(card_data->spk_pa_gain1_gpio))
			gpio_set_value(card_data->spk_pa_gain1_gpio, 1);
		if (gpio_is_valid(card_data->spk_pa_gain2_gpio))
			gpio_set_value(card_data->spk_pa_gain2_gpio, 1);
		break;
	default:
		dev_err(card->dev, "%s: unsupport speaker pga gain value\n", __func__);
		return -EINVAL;

	}
	return 0;
}

static const struct snd_kcontrol_new mt8168_quartz_controls[] = {
	SOC_ENUM_EXT("Ext_Speaker_PGA_Gain",
		mt8168_quartz_enum[0],
		mt8168_qua_ext_speaker_pga_get,
		mt8168_qua_ext_speaker_pga_put),
};

#ifdef CONFIG_SND_SOC_RT551X
#include "mtk-auddrv-def.h"
#include "../../codecs/rt551x.h"

static int rt551x_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, RT551X_SCLK_S_MCLK,
				params_rate(params) * 256, SND_SOC_CLOCK_IN);

	if (ret)
		return ret;
	return 0;
}

static struct snd_soc_ops rt551x_ops = {
	.hw_params = rt551x_hw_params,
};

static struct snd_soc_dai_link_component rt551x_codecs[] = {
	{
		.dai_name = "rt551x-aif1",
	},
};
#endif

static void mt8168_qua_ext_amp_spk_turn_on(struct snd_soc_card *card)
{
	struct mt8168_quartz_priv *card_data =
		snd_soc_card_get_drvdata(card);

	/* enable spk amp */
	if (gpio_is_valid(card_data->spk_pa_5v_en_gpio)) {
		gpio_set_value(card_data->spk_pa_5v_en_gpio, 1);

		if (card_data->ext_spk_amp_vdd_on_time_us > 0)
			usleep_range(card_data->ext_spk_amp_vdd_on_time_us,
				card_data->ext_spk_amp_vdd_on_time_us + 1);

		if (card_data->spk_pa_id == EXT_AMP_TPA2011 ||
			card_data->spk_pa_id == EXT_AMP_AD51562) {
			if (gpio_is_valid(card_data->tab_spk_en2_gpio))
				gpio_set_value(card_data->tab_spk_en2_gpio, 1);
			else {
				dev_err(card->dev, "%s, tab_spk_en2_gpio invalid\n", __func__);
				goto exit;
			}
		} else if (card_data->spk_pa_id == EXT_AMP_ALC105) {
			if (gpio_is_valid(card_data->tab_spk_en_gpio))
				gpio_set_value(card_data->tab_spk_en_gpio, 1);
			else {
				dev_err(card->dev, "%s, tab_spk_en_gpio invalid\n", __func__);
				goto exit;
			}

			if (card_data->ext_spk_amp_warmup_time_us > 0)
				usleep_range(card_data->ext_spk_amp_warmup_time_us,
					card_data->ext_spk_amp_warmup_time_us + 1);
		} else {
			dev_err(card->dev, "%s, unknown spk_pa_id: %d\n", __func__, card_data->spk_pa_id);
			goto exit;
		}
	} else
		dev_err(card->dev, "%s, spk_pa_5v_en_gpio invalid\n", __func__);

	return;
exit:
	gpio_set_value(card_data->spk_pa_5v_en_gpio, 0);

	if (card_data->ext_spk_amp_shutdown_time_us > 0)
		usleep_range(card_data->ext_spk_amp_shutdown_time_us,
			card_data->ext_spk_amp_shutdown_time_us + 1);
}

static void mt8168_qua_ext_amp_spk_turn_off(struct snd_soc_card *card)
{
	struct mt8168_quartz_priv *card_data =
		snd_soc_card_get_drvdata(card);

	/* disable spk amp */
	if (gpio_is_valid(card_data->spk_pa_5v_en_gpio)) {
		if (card_data->spk_pa_id == EXT_AMP_TPA2011 ||
			card_data->spk_pa_id == EXT_AMP_AD51562) {
			if (gpio_is_valid(card_data->tab_spk_en2_gpio))
				gpio_set_value(card_data->tab_spk_en2_gpio, 0);
			else
				dev_err(card->dev, "%s, tab_spk_en2_gpio invalid\n", __func__);
		} else if (card_data->spk_pa_id == EXT_AMP_ALC105) {
			if (gpio_is_valid(card_data->tab_spk_en_gpio))
				gpio_set_value(card_data->tab_spk_en_gpio, 0);
			else
				dev_err(card->dev, "%s, tab_spk_en_gpio invalid\n", __func__);
		} else {
			dev_err(card->dev, "%s, unknown spk_pa_id: %d\n", __func__, card_data->spk_pa_id);
			return;
		}

	} else {
		dev_err(card->dev, "%s, spk_pa_5v_en_gpio invalid\n", __func__);
		return;
	}

	gpio_set_value(card_data->spk_pa_5v_en_gpio, 0);

	if (card_data->ext_spk_amp_shutdown_time_us > 0)
		usleep_range(card_data->ext_spk_amp_shutdown_time_us,
			card_data->ext_spk_amp_shutdown_time_us + 1);
}

/* HP Spk Amp */
static int mt8168_qua_spk_amp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;

	dev_dbg(card->dev, "%s, event %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mt8168_qua_ext_amp_spk_turn_on(card);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mt8168_qua_ext_amp_spk_turn_off(card);
		break;
	default:
		break;
	}

	return 0;
}

static void mt8168_quartz_ext_hp_amp_turn_on(struct snd_soc_card *card)
{
	struct mt8168_quartz_priv *card_data =
		snd_soc_card_get_drvdata(card);

	/* enable spk amp1 */
	if (gpio_is_valid(card_data->apa_sdn_gpio))
		gpio_set_value(card_data->apa_sdn_gpio, 1);
}

static void mt8168_quartz_ext_hp_amp_turn_off(struct snd_soc_card *card)
{
	struct mt8168_quartz_priv *card_data =
		snd_soc_card_get_drvdata(card);

	/* disable spk amp1 */
	if (gpio_is_valid(card_data->apa_sdn_gpio))
		gpio_set_value(card_data->apa_sdn_gpio, 0);
}

/* HP Amp */
static int mt8168_qua_hp_amp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;

	dev_dbg(card->dev, "%s, event %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mt8168_quartz_ext_hp_amp_turn_on(card);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mt8168_quartz_ext_hp_amp_turn_off(card);
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_kcontrol_new mt8168_quartz_hpl_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8168_quartz_hpr_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8168_quartz_lol_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8168_quartz_hpl_output_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8168_quartz_hpr_output_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8168_quartz_hpl_spk_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8168_quartz_hpr_spk_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);

static const struct snd_soc_dapm_widget mt8168_quartz_widgets[] = {
	SND_SOC_DAPM_SWITCH("AU_HPL",
		SND_SOC_NOPM, 0, 0, &mt8168_quartz_hpl_switch_ctrl),
	SND_SOC_DAPM_SWITCH("AU_HPR",
		SND_SOC_NOPM, 0, 0, &mt8168_quartz_hpr_switch_ctrl),
	SND_SOC_DAPM_SWITCH("AU_LOL",
		SND_SOC_NOPM, 0, 0, &mt8168_quartz_lol_switch_ctrl),
	SND_SOC_DAPM_SWITCH("HP_L Output",
		SND_SOC_NOPM, 0, 0, &mt8168_quartz_hpl_output_switch_ctrl),
	SND_SOC_DAPM_SWITCH("HP_R Output",
		SND_SOC_NOPM, 0, 0, &mt8168_quartz_hpr_output_switch_ctrl),
	SND_SOC_DAPM_SWITCH("HP_L Spk Output",
		SND_SOC_NOPM, 0, 0,
		&mt8168_quartz_hpl_spk_switch_ctrl),
	SND_SOC_DAPM_SWITCH("HP_R Spk Output",
		SND_SOC_NOPM, 0, 0,
		&mt8168_quartz_hpr_spk_switch_ctrl),

	SND_SOC_DAPM_SPK("HP Spk Amp", NULL),
	SND_SOC_DAPM_SPK("LOL Spk Amp", mt8168_qua_spk_amp_event),
	SND_SOC_DAPM_MIC("PMIC MIC", NULL),
	SND_SOC_DAPM_HP("Headphone", mt8168_qua_hp_amp_event),
#ifdef TEST_BACKEND_WITH_ENDPOINT
	SND_SOC_DAPM_OUTPUT("TDM_OUT Out"),
#endif
};

static const struct snd_soc_dapm_route mt8168_quartz_routes[] = {
	{"AU_HPL", "Switch", "MT6357 Playback"},
	{"AU_HPR", "Switch", "MT6357 Playback"},
	{"AU_LOL", "Switch", "MT6357 Playback"},

	{"HP_L Output", "Switch", "AU_HPL"},
	{"HP_R Output", "Switch", "AU_HPR"},
	{"HP_L Spk Output", "Switch", "AU_HPL"},
	{"HP_R Spk Output", "Switch", "AU_HPR"},

	{"Headphone", NULL, "HP_L Output"},
	{"Headphone", NULL, "HP_R Output"},

	{"HP Spk Amp", NULL, "HP_R Spk Output"},
	{"HP Spk Amp", NULL, "HP_L Spk Output"},
	{"LOL Spk Amp", NULL, "AU_LOL"},

	{"MT6357 Capture", NULL, "PMIC MIC"},

#ifdef TEST_BACKEND_WITH_ENDPOINT
	{"TDM_OUT Out", NULL, "TDM_OUT Playback"},
#endif
};

static int mt8168_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
	struct snd_pcm_hw_params *params)
{
	struct mt8168_quartz_priv *priv =
		snd_soc_card_get_drvdata(rtd->card);
	int id = rtd->dai_link->id;
	struct mt8168_quartz_tdm_ctrl_data *tdm;
	struct mt8168_quartz_dmic_ctrl_data *dmic;
	unsigned int fix_rate = 0;
	unsigned int fix_bit_width = 0;
	unsigned int fix_channels = 0;

	if (id == DAI_LINK_DMIC) {
		dmic = &priv->dmic_data;
		fix_rate = dmic->fix_rate;
		fix_bit_width = dmic->fix_bit_width;
		fix_channels = dmic->fix_channels;
	} else if (id == DAI_LINK_TDM_IN_IO) {
		tdm = &priv->tdm_data;
		fix_rate = tdm->tdmin_fix_rate;
		fix_bit_width = tdm->tdmin_fix_bit_width;
		fix_channels = tdm->tdmin_fix_channels;
	}

	if (fix_rate > 0) {
		struct snd_interval *rate =
			hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);

		rate->max = rate->min = fix_rate;
	}

	if (fix_bit_width > 0) {
		struct snd_mask *mask =
			hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

		if (fix_bit_width == 32) {
			snd_mask_none(mask);
			snd_mask_set(mask, SNDRV_PCM_FORMAT_S32_LE);
		} else if (fix_bit_width == 16) {
			snd_mask_none(mask);
			snd_mask_set(mask, SNDRV_PCM_FORMAT_S16_LE);
		}
	}

	if (fix_channels > 0) {
		struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

		channels->min = channels->max = fix_channels;
	}

	return 0;
}
#ifdef CONFIG_MTK_HIFIXDSP_SUPPORT
static int mt8168_adsp_hostless_va_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dapm_context *dapm;
	struct snd_soc_card *card;
	struct snd_soc_pcm_runtime *afe_rtd;

	/* set ignore suspend for hostless path */
	dapm = snd_soc_component_get_dapm(rtd->cpu_dai->component);
	snd_soc_dapm_ignore_suspend(dapm, "FE_HOSTLESS_VA");
	snd_soc_dapm_ignore_suspend(dapm, "VA VUL2 In");
	snd_soc_dapm_ignore_suspend(dapm, "VA TDM_IN In");
	snd_soc_dapm_ignore_suspend(dapm, "VA VUL In");

	card = rtd->card;

	list_for_each_entry(afe_rtd, &card->rtd_list, list) {
		if (!strcmp(afe_rtd->cpu_dai->name, "DMIC"))
			break;
	}
	if (afe_rtd != NULL) {
		dapm =
			snd_soc_component_get_dapm(afe_rtd->cpu_dai->component);
		snd_soc_dapm_ignore_suspend(dapm, "DMIC In");
		snd_soc_dapm_ignore_suspend(dapm, "TDM_IN In");
	}

	snd_soc_dapm_ignore_suspend(&rtd->card->dapm, "PMIC MIC");

	return 0;
}

static int mt8168_adsp_hostless_va_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_dpcm *dpcm;
	struct snd_soc_pcm_runtime *fe = substream->private_data;
	struct snd_soc_pcm_runtime *be;

	/* should set all the connected be to ignore suspend */
	/* all there will be a substream->ops action in suspend_all */

	list_for_each_entry(dpcm,
	    &fe->dpcm[SNDRV_PCM_STREAM_CAPTURE].be_clients, list_be) {
		be = dpcm->be;
		be->dai_link->ignore_suspend = 1;
	}
	return 0;
}

static void mt8168_adsp_hostless_va_shutdown(
	struct snd_pcm_substream *substream)
{
	struct snd_soc_dpcm *dpcm;
	struct snd_soc_pcm_runtime *fe = substream->private_data;
	struct snd_soc_pcm_runtime *be;

	/* should resume all the connected be */
	list_for_each_entry(dpcm,
	    &fe->dpcm[SNDRV_PCM_STREAM_CAPTURE].be_clients, list_be) {
		be = dpcm->be;
		be->dai_link->ignore_suspend = 0;
	}
}

static struct snd_soc_ops adsp_hostless_va_ops = {
	.startup = mt8168_adsp_hostless_va_startup,
	.shutdown = mt8168_adsp_hostless_va_shutdown,
};
#endif

static int mt8168_qua_int_adda_startup(
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mt8168_quartz_priv *priv =
		snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (IS_ERR(priv->pin_states[PIN_STATE_MOSI_ON]))
			return ret;

		ret = pinctrl_select_state(priv->pinctrl,
					priv->pin_states[PIN_STATE_MOSI_ON]);
		if (ret)
			dev_err(rtd->card->dev, "%s failed to select state %d\n",
				__func__, ret);
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (IS_ERR(priv->pin_states[PIN_STATE_MISO_ON]))
			return ret;

		ret = pinctrl_select_state(priv->pinctrl,
					priv->pin_states[PIN_STATE_MISO_ON]);
		if (ret)
			dev_err(rtd->card->dev, "%s failed to select state %d\n",
				__func__, ret);
	}

	return 0;
}

static void mt8168_qua_int_adda_shutdown(
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mt8168_quartz_priv *priv =
		snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (IS_ERR(priv->pin_states[PIN_STATE_MOSI_OFF]))
			return;

		ret = pinctrl_select_state(priv->pinctrl,
					priv->pin_states[PIN_STATE_MOSI_OFF]);
		if (ret)
			dev_err(rtd->card->dev, "%s failed to select state %d\n",
				__func__, ret);
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (IS_ERR(priv->pin_states[PIN_STATE_MISO_OFF]))
			return;

		ret = pinctrl_select_state(priv->pinctrl,
					priv->pin_states[PIN_STATE_MISO_OFF]);
		if (ret)
			dev_err(rtd->card->dev, "%s failed to select state %d\n",
				__func__, ret);
	}

}

static struct snd_soc_ops mt8168_quartz_int_adda_ops = {
	.startup = mt8168_qua_int_adda_startup,
	.shutdown = mt8168_qua_int_adda_shutdown,
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link mt8168_quartz_dais[] = {
	/* Front End DAI links */
	[DAI_LINK_DL1_PLAYBACK] = {
		.name = "DL1_FE",
		.stream_name = "MultiMedia1_PLayback",
		.cpu_dai_name = "DL1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_DL1_PLAYBACK,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	[DAI_LINK_DL2_PLAYBACK] = {
		.name = "DL2_FE",
		.stream_name = "MultiMedia2_PLayback",
		.cpu_dai_name = "DL2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_DL2_PLAYBACK,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	[DAI_LINK_AWB_CAPTURE] = {
		.name = "AWB_FE",
		.stream_name = "DL1_AWB_Record",
		.cpu_dai_name = "AWB",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_AWB_CAPTURE,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	[DAI_LINK_VUL_CAPTURE] = {
		.name = "VUL_FE",
		.stream_name = "MultiMedia1_Capture",
		.cpu_dai_name = "VUL",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_VUL_CAPTURE,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	[DAI_LINK_VUL2_CAPTURE] = {
		.name = "VUL2_FE",
		.stream_name = "VUL2_Capture",
		.cpu_dai_name = "VUL2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_VUL2_CAPTURE,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
#ifdef CONFIG_SND_SOC_RT551X
	[DAI_LINK_VUL3_CAPTURE] = {
		.name = "VUL3_FE",
		.stream_name = "VUL3_Capture",
		.cpu_dai_name = "VUL3",
		.id = DAI_LINK_VUL3_CAPTURE,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		.codec_name = NULL,
		.codec_dai_name = NULL,
		.codecs = rt551x_codecs,
		.num_codecs = 1,
		.ops = &rt551x_ops,
	},
#else
	[DAI_LINK_VUL3_CAPTURE] = {
		.name = "VUL3_FE",
		.stream_name = "VUL3_Capture",
		.cpu_dai_name = "VUL3",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_VUL3_CAPTURE,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
#endif
	[DAI_LINK_TDM_OUT] = {
		.name = "TDM_OUT_FE",
		.stream_name = "TDM_Playback",
		.cpu_dai_name = "TDM_OUT",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_TDM_OUT,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	[DAI_LINK_TDM_IN] = {
		.name = "TDM_IN_FE",
		.stream_name = "TDM_Capture",
		.cpu_dai_name = "TDM_IN",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_TDM_IN,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
#ifdef CONFIG_SND_SOC_MTK_BTCVSD
	[DAI_LINK_BTCVSD] = {
		.name = "BTCVSD",
		.stream_name = "BTCVSD",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "18050000.mtk-btcvsd-snd",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
	},
#endif
#ifdef CONFIG_MTK_HIFIXDSP_SUPPORT
	[DAI_LINK_FE_HOSTLESS_VA] = {
		.name = "ADSP HOSTLESS_VA",
		.stream_name = "HOSTLESS_VA",
		.cpu_dai_name = "FE_HOSTLESS_VA",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.init = mt8168_adsp_hostless_va_init,
		.ops = &adsp_hostless_va_ops,
	},
	[DAI_LINK_FE_VA] = {
		.name = "ADSP VA_FE",
		.stream_name = "VA_Capture",
		.cpu_dai_name = "FE_VA",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	[DAI_LINK_FE_MIC_RECORD] = {
		.name = "ADSP MIC_RECORD",
		.stream_name = "MIC_RECORD",
		.cpu_dai_name = "FE_MICR",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
#endif
	/* Back End DAI links */
	[DAI_LINK_I2S_INTF] = {
		.name = "I2S BE",
		.cpu_dai_name = "I2S",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_I2S_INTF,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	[DAI_LINK_2ND_I2S_INTF] = {
		.name = "2ND_I2S BE",
		.cpu_dai_name = "2ND I2S",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_2ND_I2S_INTF,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	[DAI_LINK_PCM1_INTF] = {
		.name = "PCM1 BE",
		.cpu_dai_name = "PCM1",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_PCM1_INTF,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	[DAI_LINK_VIRTUAL_DL_SOURCE] = {
		.name = "VIRTUAL_DL_SRC BE",
		.cpu_dai_name = "VIRTUAL_DL_SRC",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_VIRTUAL_DL_SOURCE,
		.dpcm_capture = 1,
	},
	[DAI_LINK_VIRTUAL_TDM_OUT_SOURCE] = {
		.name = "VIRTUAL_TDM_OUT_SRC BE",
		.cpu_dai_name = "VIRTUAL_TDM_OUT_SRC",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_VIRTUAL_TDM_OUT_SOURCE,
		.dpcm_capture = 1,
	},
	[DAI_LINK_DMIC] = {
		.name = "DMIC BE",
		.cpu_dai_name = "DMIC",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_DMIC,
		.dpcm_capture = 1,
	},
	[DAI_LINK_INT_ADDA] = {
		.name = "MTK Codec",
		.cpu_dai_name = "INT ADDA",
		.no_pcm = 1,
		.codec_name = "mt-soc-codec",
		.codec_dai_name = "mt6357-codec-dai",
		.id = DAI_LINK_INT_ADDA,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &mt8168_quartz_int_adda_ops,
	},
	[DAI_LINK_TDM_OUT_IO] = {
		.name = "TDM_OUT BE",
		.cpu_dai_name = "TDM_OUT_IO",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_TDM_OUT_IO,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
	},
	[DAI_LINK_TDM_IN_IO] = {
		.name = "TDM_IN BE",
		.cpu_dai_name = "TDM_IN_IO",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_TDM_IN_IO,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_IB_IF |
				SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_capture = 1,
	},
	[DAI_LINK_GASRC1] = {
		.name = "GASRC1 BE",
		.cpu_dai_name = "GASRC1",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_GASRC1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	[DAI_LINK_GASRC2] = {
		.name = "GASRC2 BE",
		.cpu_dai_name = "GASRC2",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_GASRC2,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	[DAI_LINK_TDM_ASRC] = {
		.name = "TDM_ASRC BE",
		.cpu_dai_name = "TDM_ASRC",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.id = DAI_LINK_TDM_ASRC,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
#ifdef CONFIG_MTK_HIFIXDSP_SUPPORT
	[DAI_LINK_BE_VUL2_IN] = {
		.name = "ADSP_VUL2_IN BE",
		.cpu_dai_name = "BE_VUL2_IN",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_playback = 0,
		.dpcm_capture = 1,
	},
	[DAI_LINK_BE_TDM_IN] = {
		.name = "ADSP_TDM_IN BE",
		.cpu_dai_name = "BE_TDM_IN",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_playback = 0,
		.dpcm_capture = 1,
	},
	[DAI_LINK_BE_VUL_IN] = {
		.name = "ADSP_VUL_IN BE",
		.cpu_dai_name = "BE_VUL_IN",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_playback = 0,
		.dpcm_capture = 1,
	},
#endif
#ifdef CONFIG_SND_SOC_RT551X
	[DAI_LINK_FE_SPI32765] = {
		.name = "RT551X DSP",
		.stream_name = "DSP_Capture",
		.cpu_dai_name = "spi0.0",
		.platform_name = "spi0.0",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
#endif
};

static int mt8168_qua_gpio_probe(struct snd_soc_card *card)
{
	struct mt8168_quartz_priv *priv = snd_soc_card_get_drvdata(card);
	struct device_node *np = card->dev->of_node;
	int ret = 0;
	int i;

	priv->pinctrl = devm_pinctrl_get(card->dev);
	if (IS_ERR(priv->pinctrl)) {
		ret = PTR_ERR(priv->pinctrl);
		dev_err(card->dev, "%s devm_pinctrl_get failed %d\n",
			__func__, ret);
		return ret;
	}

	for (i = 0 ; i < PIN_STATE_MAX ; i++) {
		priv->pin_states[i] = pinctrl_lookup_state(priv->pinctrl,
			mt8168_quartz_pin_str[i]);
		if (IS_ERR(priv->pin_states[i])) {
			ret = PTR_ERR(priv->pin_states[i]);
			dev_info(card->dev, "%s Can't find pin state %s %d\n",
				 __func__, mt8168_quartz_pin_str[i], ret);
		}
	}

	priv->tab_spk_en_gpio = of_get_named_gpio(np, "tab-spk-en-gpio", 0);
	if (!gpio_is_valid(priv->tab_spk_en_gpio))
		dev_warn(card->dev, "%s get invalid tab-spk-en-gpio %d\n",
				__func__, priv->tab_spk_en_gpio);

	priv->tab_spk_en2_gpio = of_get_named_gpio(np, "tab-spk-en2-gpio", 0);
	if (!gpio_is_valid(priv->tab_spk_en2_gpio))
		dev_warn(card->dev, "%s get invalid tab-spk-en2-gpio %d\n",
				__func__, priv->tab_spk_en2_gpio);

	priv->spk_pa_sku_id_gpio = of_get_named_gpio(np, "spk-pa-sku-id-gpio", 0);
	if (!gpio_is_valid(priv->spk_pa_sku_id_gpio))
		dev_warn(card->dev, "%s get invalid spk-pa-sku-id-gpio %d\n",
				__func__, priv->spk_pa_sku_id_gpio);

	priv->spk_pa_sku_id2_gpio = of_get_named_gpio(np, "spk-pa-sku-id2-gpio", 0);
	if (!gpio_is_valid(priv->spk_pa_sku_id2_gpio))
		dev_warn(card->dev, "%s get invalid spk-pa-sku-id2-gpio %d\n",
				__func__, priv->spk_pa_sku_id2_gpio);

	priv->spk_pa_5v_en_gpio = of_get_named_gpio(np, "spk-pa-5v-en-gpio", 0);
	if (!gpio_is_valid(priv->spk_pa_5v_en_gpio))
		dev_warn(card->dev, "%s get invalid spkr-sel-gpio %d\n",
				__func__, priv->spk_pa_5v_en_gpio);

	priv->apa_sdn_gpio = of_get_named_gpio(np, "apa-sdn-gpio", 0);
	if (!gpio_is_valid(priv->apa_sdn_gpio))
		dev_warn(card->dev, "%s get invalid apa-sdn-gpio %d\n",
				__func__, priv->apa_sdn_gpio);

	priv->spk_pa_gain1_gpio = of_get_named_gpio(np, "spk-pa-gain1-gpio", 0);
	if (!gpio_is_valid(priv->spk_pa_gain1_gpio))
		dev_warn(card->dev, "%s get invalid spk_pa_gain1_gpio %d\n",
				__func__, priv->spk_pa_gain1_gpio);

	priv->spk_pa_gain2_gpio = of_get_named_gpio(np, "spk-pa-gain2-gpio", 0);
	if (!gpio_is_valid(priv->spk_pa_gain2_gpio))
		dev_warn(card->dev, "%s get invalid spk_pa_gain2_gpio %d\n",
				__func__, priv->spk_pa_gain2_gpio);

	priv->spk_pa_id = EXT_AMP_ALC105;

	if (gpio_is_valid(priv->tab_spk_en_gpio)) {
		ret = devm_gpio_request_one(card->dev, priv->tab_spk_en_gpio,
			GPIOF_OUT_INIT_LOW, "tab-spk-en");
		if (ret < 0)
			dev_err(card->dev, "%s failed to init tab spk enable gpio %d\n",
				__func__, ret);
	}

	if (gpio_is_valid(priv->tab_spk_en2_gpio)) {
		ret = devm_gpio_request_one(card->dev, priv->tab_spk_en2_gpio,
			GPIOF_OUT_INIT_LOW, "tab-spk-en2");
		if (ret < 0)
			dev_err(card->dev, "%s failed to init tab spk enable2 gpio %d\n",
				__func__, ret);
	}

	if (gpio_is_valid(priv->spk_pa_sku_id_gpio)) {
		ret = devm_gpio_request_one(card->dev, priv->spk_pa_sku_id_gpio,
			GPIOF_IN, "spk-pa-sku-id");
		if (ret < 0)
			dev_err(card->dev, "%s failed to init tab spk pa sku id gpio %d\n",
				__func__, ret);
		else
			priv->spk_pa_id = gpio_get_value(priv->spk_pa_sku_id_gpio);
	}

	if (gpio_is_valid(priv->spk_pa_sku_id2_gpio)) {
		ret = devm_gpio_request_one(card->dev, priv->spk_pa_sku_id2_gpio,
			GPIOF_IN, "spk-pa-sku-id2");
		if (ret < 0)
			dev_err(card->dev, "%s failed to init tab spk pa sku id2 gpio %d\n",
				__func__, ret);
		else
			priv->spk_pa_id |= gpio_get_value(priv->spk_pa_sku_id2_gpio) << 1;
	}

	dev_info(card->dev, "%s get spk_pa_id %d\n", __func__, priv->spk_pa_id);

	if (gpio_is_valid(priv->spk_pa_5v_en_gpio)) {
		ret = devm_gpio_request_one(card->dev, priv->spk_pa_5v_en_gpio,
			GPIOF_OUT_INIT_LOW, "spk power en");
		if (ret < 0)
			dev_err(card->dev, "%s failed to init spk power en gpio %d\n",
				__func__, ret);
	}

	if (gpio_is_valid(priv->apa_sdn_gpio)) {
		ret = devm_gpio_request_one(card->dev, priv->apa_sdn_gpio,
			GPIOF_OUT_INIT_LOW, "apa sdn gpio");
		if (ret < 0)
			dev_err(card->dev, "%s failed to init apa sdn gpio %d\n",
				__func__, ret);
	}

	if (gpio_is_valid(priv->spk_pa_gain1_gpio)) {
		ret = devm_gpio_request_one(card->dev, priv->spk_pa_gain1_gpio,
			GPIOF_OUT_INIT_LOW, "spk pa gain1");
		if (ret < 0)
			dev_err(card->dev, "%s failed to init spk pa gain1 gpio %d\n",
				__func__, ret);
	}

	if (gpio_is_valid(priv->spk_pa_gain2_gpio)) {
		ret = devm_gpio_request_one(card->dev, priv->spk_pa_gain2_gpio,
			GPIOF_OUT_INIT_LOW, "spk pa gain2");
		if (ret < 0)
			dev_err(card->dev, "%s failed to init spk pa gain2 gpio %d\n",
				__func__, ret);
	}

	for (i = PIN_STATE_DEFAULT ; i < PIN_STATE_MAX ; i++) {
		if (IS_ERR(priv->pin_states[i])) {
			dev_info(card->dev, "%s check pin %s state err\n",
				 __func__, mt8168_quartz_pin_str[i]);
			continue;
		}

		/* default state */
		ret = pinctrl_select_state(priv->pinctrl,
				priv->pin_states[i]);
		if (ret)
			dev_info(card->dev, "%s failed to select state %d\n",
				__func__, ret);
	}

	/* turn off mosi pin if exist */
	if (!IS_ERR(priv->pin_states[PIN_STATE_MOSI_OFF])) {
		ret = pinctrl_select_state(priv->pinctrl,
				priv->pin_states[PIN_STATE_MOSI_OFF]);
		if (ret)
			dev_info(card->dev,
				"%s failed to select state %d\n",
				__func__, ret);
	}

	/* turn off miso pin if exist */
	if (!IS_ERR(priv->pin_states[PIN_STATE_MISO_OFF])) {
		ret = pinctrl_select_state(priv->pinctrl,
				priv->pin_states[PIN_STATE_MISO_OFF]);
		if (ret)
			dev_info(card->dev,
				"%s failed to select state %d\n",
				__func__, ret);
	}


	return ret;
}

static int link_to_dai(int link_id)
{
	switch (link_id) {
	case DAI_LINK_TDM_IN_IO:
		return MT8168_AFE_IO_TDM_IN;
	case DAI_LINK_DMIC:
		return MT8168_AFE_IO_DMIC;
	default:
		break;
	}
	return -1;
}

static void mt8168_quartz_parse_of(struct snd_soc_card *card,
	struct device_node *np)
{
	struct mt8168_quartz_priv *priv = snd_soc_card_get_drvdata(card);
	size_t i;
	int ret;
	char prop[128];
	unsigned int val;

	static const struct mt8168_dai_link_prop of_dai_links_io[] = {
		{ "dmic",	DAI_LINK_DMIC },
		{ "tdmin",	DAI_LINK_TDM_IN_IO },
	};

	for (i = 0; i < ARRAY_SIZE(of_dai_links_io); i++) {
		unsigned int link_id = of_dai_links_io[i].link_id;
		struct snd_soc_dai_link *dai_link =
			&mt8168_quartz_dais[link_id];
		struct mt8168_quartz_dmic_ctrl_data *dmic;
		struct mt8168_quartz_tdm_ctrl_data *tdm;

		/* parse fix rate */
		snprintf(prop, sizeof(prop), PREFIX"%s-fix-rate",
			 of_dai_links_io[i].name);
		ret = of_property_read_u32(np, prop, &val);

		if (ret == 0 && mt8168_afe_rate_supported(val,
			link_to_dai(link_id))) {
			switch (link_id) {
			case DAI_LINK_DMIC:
				dmic = &priv->dmic_data;
				dmic->fix_rate = val;
				break;
			case DAI_LINK_TDM_IN_IO:
				tdm = &priv->tdm_data;
				tdm->tdmin_fix_rate = val;
				break;
			default:
				break;
			}

			dai_link->be_hw_params_fixup =
				mt8168_be_hw_params_fixup;
		}

		/* parse fix bit width */
		snprintf(prop, sizeof(prop), PREFIX"%s-fix-bit-width",
			 of_dai_links_io[i].name);
		ret = of_property_read_u32(np, prop, &val);
		if (ret == 0 && (val == 32 || val == 16)) {
			switch (link_id) {
			case DAI_LINK_DMIC:
				dmic = &priv->dmic_data;
				dmic->fix_bit_width = val;
				break;
			case DAI_LINK_TDM_IN_IO:
				tdm = &priv->tdm_data;
				tdm->tdmin_fix_bit_width = val;
				break;
			default:
				break;
			}

			dai_link->be_hw_params_fixup =
				mt8168_be_hw_params_fixup;
		}

		/* parse fix channels */
		snprintf(prop, sizeof(prop), PREFIX"%s-fix-channels",
			 of_dai_links_io[i].name);
		ret = of_property_read_u32(np, prop, &val);

		if (ret == 0 && mt8168_afe_channel_supported(val,
			link_to_dai(link_id))) {
			switch (link_id) {
			case DAI_LINK_DMIC:
				dmic = &priv->dmic_data;
				dmic->fix_channels = val;
				break;
			case DAI_LINK_TDM_IN_IO:
				tdm = &priv->tdm_data;
				tdm->tdmin_fix_channels = val;
				break;
			default:
				break;
			}

			dai_link->be_hw_params_fixup =
				mt8168_be_hw_params_fixup;
		}
	}

	of_property_read_u32(np,
		"mediatek,ext-spk-amp-warmup-time-us",
		&priv->ext_spk_amp_warmup_time_us);

	of_property_read_u32(np,
		"mediatek,ext-spk-amp-shutdown-time-us",
		&priv->ext_spk_amp_shutdown_time_us);

	of_property_read_u32(np,
		"mediatek,ext-spk-amp-vdd-on-time-us",
		&priv->ext_spk_amp_vdd_on_time_us);
}

static struct snd_soc_card mt8168_quartz_card = {
	.name = "mt-snd-card",
	.owner = THIS_MODULE,
	.dai_link = mt8168_quartz_dais,
	.num_links = ARRAY_SIZE(mt8168_quartz_dais),
	.controls = mt8168_quartz_controls,
	.num_controls = ARRAY_SIZE(mt8168_quartz_controls),
	.dapm_widgets = mt8168_quartz_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt8168_quartz_widgets),
	.dapm_routes = mt8168_quartz_routes,
	.num_dapm_routes = ARRAY_SIZE(mt8168_quartz_routes),
};

static int mt8168_qua_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt8168_quartz_card;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *platform_node;
	struct mt8168_quartz_priv *priv;
	int i, ret;

	platform_node = of_parse_phandle(dev->of_node, "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(dev, "Property 'platform' missing or invalid\n");
		return -EINVAL;
	}

	for (i = 0; i < card->num_links; i++) {
#ifdef CONFIG_MTK_HIFIXDSP_SUPPORT
		if (is_adsp_dai_link(i))
			continue;
#endif
		if (mt8168_quartz_dais[i].platform_name)
			continue;
		mt8168_quartz_dais[i].platform_of_node = platform_node;
	}
#ifdef CONFIG_SND_SOC_RT551X
	if (platform_node)
		rt551x_codecs[0].of_node = of_parse_phandle(platform_node, "sound-dai", 0);
	if (!rt551x_codecs[0].of_node) {
		dev_err(dev, "Property sound-dai missing or invalid\n");
		return -EINVAL;
	}
#endif
#ifdef CONFIG_MTK_HIFIXDSP_SUPPORT
	platform_node = of_parse_phandle(dev->of_node,
					 "mediatek,adsp-platform", 0);
	if (!platform_node) {
		dev_err(dev, "Property 'adsp-platform' missing or invalid\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(adsp_dai_links); i++) {
		unsigned int dai = adsp_dai_links[i];

		if (mt8168_quartz_dais[dai].platform_name)
			continue;
		mt8168_quartz_dais[dai].platform_of_node = platform_node;
	}
#endif

	card->dev = dev;

	priv = devm_kzalloc(dev, sizeof(struct mt8168_quartz_priv),
			    GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		dev_err(dev, "%s allocate card private data fail %d\n",
			__func__, ret);
		return ret;
	}

	snd_soc_card_set_drvdata(card, priv);

	mt8168_qua_gpio_probe(card);

	mt8168_quartz_parse_of(card, np);

	ret = devm_snd_soc_register_card(dev, card);
	if (ret)
		dev_err(dev, "%s snd_soc_register_card fail %d\n",
			__func__, ret);

	return ret;
}

static const struct of_device_id mt8168_quartz_dt_match[] = {
	{ .compatible = "mediatek,mt8168-quartz", },
	{ }
};
MODULE_DEVICE_TABLE(of, mt8168_quartz_dt_match);

static struct platform_driver mt8168_qua_driver = {
	.driver = {
		   .name = "mt8168-qua",
		   .of_match_table = mt8168_quartz_dt_match,
#ifdef CONFIG_PM
		   .pm = &snd_soc_pm_ops,
#endif
	},
	.probe = mt8168_qua_dev_probe,
};

module_platform_driver(mt8168_qua_driver);

/* Module information */
MODULE_DESCRIPTION("MT8168 Quartz SoC machine driver");
MODULE_AUTHOR("Jia Zeng <jia.zeng@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mt8168-quartz");
