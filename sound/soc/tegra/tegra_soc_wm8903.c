/*
 * tegra_soc_wm8903.c  --  SoC audio for tegra
 *
 * (c) 2010-2011 Nvidia Graphics Pvt. Ltd.
 *  http://www.nvidia.com
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include "tegra_soc.h"
#include <linux/gpio.h>
#include <sound/soc-dapm.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include "../codecs/wm8903.h"
#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA) || defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
#include "tegra_wired_jack.h"
extern int getAudioTable(void);
extern void setAudioTable(int table_value);
#endif

#ifdef CONFIG_MACH_ACER_VANGOGH
#include <mach/gpio.h>
#define GPIO_CDC_IRQ 220
#endif

static struct platform_device *tegra_snd_device;

static struct regulator *reg_vmic = NULL;
extern int en_dmic;

extern struct snd_soc_dai tegra_i2s_dai[];
extern struct snd_soc_dai tegra_spdif_dai;
extern struct snd_soc_dai tegra_generic_codec_dai[];
extern struct snd_soc_platform tegra_soc_platform;
extern struct wired_jack_conf tegra_wired_jack_conf;

#ifdef CONFIG_MACH_ACER_VANGOGH
struct wm8903_cdc_irq
{
	struct snd_soc_codec* codec;
	struct work_struct work;
	int gpio;
};

static struct wm8903_cdc_irq* wm8903_gpio_intr = NULL;
#endif

/* codec register values */
#define B00_IN_VOL		0
#define B00_INR_ENA		0
#define B01_INL_ENA		1
#define B01_MICDET_ENA		1
#define B00_MICBIAS_ENA		0
#define B15_DRC_ENA		15
#define B01_ADCL_ENA		1
#define B00_ADCR_ENA		0
#define B06_IN_CM_ENA		6
#define B04_IP_SEL_N		4
#define B02_IP_SEL_P		2
#define B00_MODE 		0
#define B06_AIF_ADCL		7
#define B06_AIF_ADCR		6
#define B04_ADC_HPF_ENA		4
#define R20_SIDETONE_CTRL	32
#define R29_DRC_1		41

#define B08_GPx_FN		8
#define B07_GPx_DIR		7

#define DMIC_CLK_OUT		(0x6 << B08_GPx_FN)
#define DMIC_DAT_DATA_IN	(0x6 << B08_GPx_FN)
#define GPIO_DIR_OUT		(0x0 << B07_GPx_DIR)
#define GPIO_DIR_IN			(0x1 << B07_GPx_DIR)

#define ADC_DIGITAL_VOL_9DB		0x1D8
#define ADC_DIGITAL_VOL_12DB	0x1E0
#define ADC_ANALOG_VOLUME		0x1C
#define DRC_MAX_36DB			0x03

#define SET_REG_VAL(r,m,l,v) (((r)&(~((m)<<(l))))|(((v)&(m))<<(l)))

#ifdef MACH_ACER_AUDIO
static void hp_enable(struct snd_soc_codec *codec, int enable);
#endif

static ssize_t digital_mic_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", en_dmic);
}

static ssize_t digital_mic_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	if (count > 3) {
		pr_err("%s: buffer size %d too big\n", __func__, count);
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &en_dmic) != 1) {
		pr_err("%s: invalid input string [%s]\n", __func__, buf);
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(enable_digital_mic, 0644, digital_mic_show, digital_mic_store);

#ifndef MACH_ACER_AUDIO
static void configure_dmic(struct snd_soc_codec *codec)
{
	u16 test4, reg;

	if (en_dmic) {
		/* Set GP1_FN as DMIC_LR */
		snd_soc_write(codec, WM8903_GPIO_CONTROL_1,
					DMIC_CLK_OUT | GPIO_DIR_OUT);

		/* Set GP2_FN as DMIC_DAT */
		snd_soc_write(codec, WM8903_GPIO_CONTROL_2,
					DMIC_DAT_DATA_IN | GPIO_DIR_IN);

		/* Enable ADC Digital volumes */
		snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_LEFT,
					ADC_DIGITAL_VOL_9DB);
		snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT,
					ADC_DIGITAL_VOL_9DB);

		/* Enable DIG_MIC */
		test4 = WM8903_ADC_DIG_MIC;
	} else {
		/* Disable DIG_MIC */
		test4 = snd_soc_read(codec, WM8903_CLOCK_RATE_TEST_4);
		test4 &= ~WM8903_ADC_DIG_MIC;
	}

	reg = snd_soc_read(codec, WM8903_CONTROL_INTERFACE_TEST_1);
	snd_soc_write(codec, WM8903_CONTROL_INTERFACE_TEST_1,
			 reg | WM8903_TEST_KEY);
	snd_soc_write(codec, WM8903_CLOCK_RATE_TEST_4, test4);
	snd_soc_write(codec, WM8903_CONTROL_INTERFACE_TEST_1, reg);

}
#endif

static int tegra_hifi_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tegra_audio_data* audio_data = rtd->socdev->codec_data;
	enum dac_dap_data_format data_fmt;
	int dai_flag = 0, sys_clk;
	int err;

	if (tegra_das_is_port_master(tegra_audio_codec_type_hifi))
		dai_flag |= SND_SOC_DAIFMT_CBM_CFM;
	else
		dai_flag |= SND_SOC_DAIFMT_CBS_CFS;

	data_fmt = tegra_das_get_codec_data_fmt(tegra_audio_codec_type_hifi);

	/* We are supporting DSP and I2s format for now */
	if (data_fmt & dac_dap_data_format_i2s)
		dai_flag |= SND_SOC_DAIFMT_I2S;
	else
		dai_flag |= SND_SOC_DAIFMT_DSP_A;

	err = snd_soc_dai_set_fmt(codec_dai, dai_flag);
	if (err < 0) {
		pr_err("codec_dai fmt not set \n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, dai_flag);
	if (err < 0) {
		pr_err("cpu_dai fmt not set \n");
		return err;
	}

	sys_clk = clk_get_rate(audio_data->dap_mclk);
	err = snd_soc_dai_set_sysclk(codec_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("codec_dai clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(cpu_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai clock not set\n");
		return err;
	}

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		int CtrlReg = 0;
		pr_err("SNDRV_PCM_STREAM_CAPTURE \n");
#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
		if (!audio_data->is_speech_mode)
			reroute_table();
#endif
		/* Mic Bias enable */
		CtrlReg = (0x1<<B00_MICBIAS_ENA) | (0x1<<B01_MICDET_ENA);
		snd_soc_write(codec, WM8903_MIC_BIAS_CONTROL_0, CtrlReg);
		/* Enable DRC */
		CtrlReg = snd_soc_read(codec, WM8903_DRC_0);
		CtrlReg |= (1<<B15_DRC_ENA);
		snd_soc_write(codec, WM8903_DRC_0, CtrlReg);
		CtrlReg = snd_soc_read(codec, R29_DRC_1);
		CtrlReg |= 0x3; /*mic volume 18 db */
		snd_soc_write(codec, R29_DRC_1, CtrlReg);
		/* Single Ended Mic */
#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
		CtrlReg = (0x0<<B06_IN_CM_ENA) |
			(0x0<<B00_MODE) | (0x0<<B04_IP_SEL_N)
					| (0x1<<B02_IP_SEL_P);
#elif defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA)
		if (wired_jack_state() == HEADSET_MIC) {
			CtrlReg = (0x0<<B06_IN_CM_ENA) |
				(0x0<<B00_MODE) | (0x1<<B04_IP_SEL_N)
				| (0x1<<B02_IP_SEL_P);
		} else {
			CtrlReg = (0x0<<B06_IN_CM_ENA) |
				(0x0<<B00_MODE) | (0x0<<B04_IP_SEL_N)
				| (0x1<<B02_IP_SEL_P);
		}
#else
		CtrlReg = (0x0<<B06_IN_CM_ENA) |
			(0x0<<B00_MODE) | (0x0<<B04_IP_SEL_N)
					| (0x1<<B02_IP_SEL_P);
#endif
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_1, CtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_1, CtrlReg);
		/* voulme for single ended mic */
#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA)
		if (!audio_data->is_video_call_mode && !audio_data->is_speech_mode) {
			CtrlReg = (0x1C << B00_IN_VOL);
		}
#elif defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
		if (!audio_data->is_video_call_mode && !audio_data->is_speech_mode) {
			CtrlReg = (0x1C << B00_IN_VOL);
		} else {
			CtrlReg = (0x10 << B00_IN_VOL);
		}
#else
		CtrlReg = (0x1C << B00_IN_VOL);
#endif
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0, CtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0, CtrlReg);
		/* Left ADC data on both channels */
		CtrlReg = snd_soc_read(codec, WM8903_AUDIO_INTERFACE_0);
		CtrlReg  = SET_REG_VAL(CtrlReg, 0x1, B06_AIF_ADCR, 0x0);
		CtrlReg  = SET_REG_VAL(CtrlReg, 0x1, B06_AIF_ADCL, 0x0);
		snd_soc_write(codec, WM8903_AUDIO_INTERFACE_0, CtrlReg);
		/* ADC Settings */
		CtrlReg = snd_soc_read(codec, WM8903_ADC_DIGITAL_0);
		CtrlReg |= (0x1<<B04_ADC_HPF_ENA);
		snd_soc_write(codec, WM8903_ADC_DIGITAL_0, CtrlReg);
		/* Disable sidetone */
		CtrlReg = 0;
		snd_soc_write(codec, R20_SIDETONE_CTRL, CtrlReg);
		/* Enable analog inputs */
#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA) || defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
		if (audio_data->isMicMuted)
			CtrlReg = (0x0<<B01_INL_ENA);
		else
			CtrlReg = (0x1<<B01_INL_ENA);
#else
		CtrlReg = (0x1<<B01_INL_ENA);
#endif
		snd_soc_write(codec, WM8903_POWER_MANAGEMENT_0, CtrlReg);
		/* Enable ADC */
		CtrlReg = snd_soc_read(codec, WM8903_POWER_MANAGEMENT_6);
		CtrlReg |= (0x1<<B01_ADCL_ENA);
		snd_soc_write(codec, WM8903_POWER_MANAGEMENT_6, CtrlReg);

#ifndef MACH_ACER_AUDIO
		configure_dmic(codec);
#endif

	}

#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA)
	snd_soc_write(codec, WM8903_ANALOGUE_OUT2_LEFT, 0xB7);
	snd_soc_write(codec, WM8903_ANALOGUE_OUT2_RIGHT, 0xB7);
#elif defined(CONFIG_MACH_ACER_VANGOGH)
	snd_soc_write(codec, WM8903_ANALOGUE_OUT2_LEFT, 0xB3);
	snd_soc_write(codec, WM8903_ANALOGUE_OUT2_RIGHT, 0xB3);
#else
	snd_soc_write(codec, WM8903_ANALOGUE_OUT2_LEFT, 0xB7);
	snd_soc_write(codec, WM8903_ANALOGUE_OUT2_RIGHT, 0xB7);
#endif
	return 0;
}

static int tegra_voice_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct tegra_audio_data* audio_data = rtd->socdev->codec_data;
	enum dac_dap_data_format data_fmt;
	int dai_flag = 0, sys_clk;
	int err;

	if (tegra_das_is_port_master(tegra_audio_codec_type_bluetooth))
		dai_flag |= SND_SOC_DAIFMT_CBM_CFM;
	else
		dai_flag |= SND_SOC_DAIFMT_CBS_CFS;

	data_fmt = tegra_das_get_codec_data_fmt(tegra_audio_codec_type_bluetooth);

	/* We are supporting DSP and I2s format for now */
	if (data_fmt & dac_dap_data_format_dsp)
		dai_flag |= SND_SOC_DAIFMT_DSP_A;
	else
		dai_flag |= SND_SOC_DAIFMT_I2S;

	err = snd_soc_dai_set_fmt(codec_dai, dai_flag);
	if (err < 0) {
		pr_err("codec_dai fmt not set \n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, dai_flag);
	if (err < 0) {
		pr_err("cpu_dai fmt not set \n");
		return err;
	}

	sys_clk = clk_get_rate(audio_data->dap_mclk);
	err = snd_soc_dai_set_sysclk(codec_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(cpu_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai clock not set\n");
		return err;
	}

	return 0;
}

static int tegra_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	return 0;
}

int tegra_codec_startup(struct snd_pcm_substream *substream)
{
	tegra_das_power_mode(true);

	if ((SNDRV_PCM_STREAM_CAPTURE == substream->stream) && en_dmic) {
		/* enable d-mic */
		if (reg_vmic) {
			regulator_enable(reg_vmic);
		}
	}

	return 0;
}

void tegra_codec_shutdown(struct snd_pcm_substream *substream)
{
	tegra_das_power_mode(false);

	if ((SNDRV_PCM_STREAM_CAPTURE == substream->stream) && en_dmic) {
		/* disable d-mic */
		if (reg_vmic) {
			regulator_disable(reg_vmic);
		}
	}
}

int tegra_soc_suspend_pre(struct platform_device *pdev, pm_message_t state)
{
	tegra_jack_suspend();

#if defined(CONFIG_MACH_ACER_VANGOGH)
	gpio_set_value_cansleep(tegra_wired_jack_conf.en_spkr, 0);
#endif

	return 0;
}

int tegra_soc_suspend_post(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct tegra_audio_data* audio_data = socdev->codec_data;

	clk_disable(audio_data->dap_mclk);

	return 0;
}

int tegra_soc_resume_pre(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct tegra_audio_data* audio_data = socdev->codec_data;

	clk_enable(audio_data->dap_mclk);

	return 0;
}

int tegra_soc_resume_post(struct platform_device *pdev)
{
	tegra_jack_resume();

#if defined(CONFIG_MACH_ACER_VANGOGH)
	gpio_set_value_cansleep(tegra_wired_jack_conf.en_spkr, 1);
#endif

	return 0;
}

static struct snd_soc_ops tegra_hifi_ops = {
	.hw_params = tegra_hifi_hw_params,
	.startup = tegra_codec_startup,
	.shutdown = tegra_codec_shutdown,
};

static struct snd_soc_ops tegra_voice_ops = {
	.hw_params = tegra_voice_hw_params,
	.startup = tegra_codec_startup,
	.shutdown = tegra_codec_shutdown,
};

static struct snd_soc_ops tegra_spdif_ops = {
	.hw_params = tegra_spdif_hw_params,
};

#ifdef MACH_ACER_AUDIO
static void hp_enable(struct snd_soc_codec *codec, int enable)
{
	if (enable) {
#if defined(CONFIG_MACH_ACER_PICASSO_E)
		snd_soc_write(codec, WM8903_ANALOGUE_OUT1_LEFT, 0xAD);
		snd_soc_write(codec, WM8903_ANALOGUE_OUT1_RIGHT, 0xAD);
#else
		snd_soc_write(codec, WM8903_ANALOGUE_OUT1_LEFT, 0xB8);
		snd_soc_write(codec, WM8903_ANALOGUE_OUT1_RIGHT, 0xB8);
#endif
		pr_info("[Audio] Headphone Unmute");
	} else {
		snd_soc_write(codec, WM8903_ANALOGUE_OUT1_LEFT, 0x1B8);
		snd_soc_write(codec, WM8903_ANALOGUE_OUT1_RIGHT, 0x1B8);
		pr_info("[Audio] Headphone Mute");
	}
}


static void speaker_louder(struct snd_soc_codec *codec, bool enable)
{
	int CtrlReg = 0;
	CtrlReg = snd_soc_read(codec, WM8903_ANALOGUE_LEFT_INPUT_0);
	CtrlReg &= ~0x1F;
	CtrlReg |= 0x10;
	snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0, CtrlReg);
	snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0, CtrlReg);
	snd_soc_write(codec, WM8903_ANALOGUE_OUT2_LEFT, 0xBF);
	snd_soc_write(codec, WM8903_ANALOGUE_OUT2_RIGHT, 0xBF);
	pr_info("[Audio] Video Call: speaker louder mode!\n");
}

static void voip_hp(struct snd_soc_codec *codec, bool enable)
{
	int CtrlReg = 0;
		CtrlReg = snd_soc_read(codec, WM8903_ANALOGUE_LEFT_INPUT_0);
		CtrlReg &= ~0x1F;
#if defined(CONFIG_MACH_ACER_PICASSO_E)
		snd_soc_write(codec, WM8903_POWER_MANAGEMENT_3, 0x03);
		CtrlReg |= 0x1C;
#else
		CtrlReg |= 0x10;
#endif
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0, CtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0, CtrlReg);
		pr_info("[Audio] Video Call: headset mode!\n");
}

#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
void reroute_table(void)
{
	if (wired_jack_state() == HEADSET_MIC) {
		if ((getAudioTable() != ACOUSTIC_HEADSET_MIC_MUSIC_RECOGNITION_TABLE) &&
		   (getAudioTable() != ACOUSTIC_CAMCORDER_TABLE))
			setAudioTable(ACOUSTIC_HEADSET_MIC_RECORDING_TABLE);
	} else if (wired_jack_state() == HEADPHONE_MIC) {
		if (getAudioTable() == ACOUSTIC_HEADSET_MIC_MUSIC_RECOGNITION_TABLE) {
			setAudioTable(ACOUSTIC_DEVICE_MIC_MUSIC_RECOGNITION_TABLE);
		}
	} else {
		if ((getAudioTable() != ACOUSTIC_DEVICE_MIC_MUSIC_RECOGNITION_TABLE) &&
		   (getAudioTable() != ACOUSTIC_CAMCORDER_TABLE))
			setAudioTable(ACOUSTIC_DEVICE_MIC_RECORDING_TABLE);
	}
	pr_err("rerout table %d \n", getAudioTable());
}
#endif

int switch_audio_table(struct snd_soc_codec *codec, int new_con)
{
	struct tegra_audio_data* audio_data = codec->socdev->codec_data;

	if (new_con & TEGRA_VOIP_CALL) {
#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
		if (wired_jack_state() == HEADSET_MIC)
			audio_data->mode = ACOUSTIC_HEADSET_MIC_VOIP_TABLE;
		else
			audio_data->mode = ACOUSTIC_DEVICE_MIC_VOIP_TABLE;
#else
		audio_data->mode = ACOUSTIC_DEVICE_MIC_VOIP_TABLE;
#endif
	} else if (new_con & TEGRA_SPEECH_MODE) {
		if (wired_jack_state() == DEVICE_MIC || wired_jack_state() == HEADPHONE_MIC)
			audio_data->mode = ACOUSTIC_SPEECH_RECOGNITION_TABLE;
	} else {
#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
		if (wired_jack_state() == HEADSET_MIC)
			audio_data->mode = ACOUSTIC_HEADSET_MIC_RECORDING_TABLE;
		else
			audio_data->mode = ACOUSTIC_DEVICE_MIC_RECORDING_TABLE;
#else
			audio_data->mode = ACOUSTIC_DEVICE_MIC_RECORDING_TABLE;
#endif
	}

	if (audio_data->mode != getAudioTable()) {
#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
		if ((getAudioTable() == ACOUSTIC_DEVICE_MIC_MUSIC_RECOGNITION_TABLE) ||
			(getAudioTable() == ACOUSTIC_HEADSET_MIC_MUSIC_RECOGNITION_TABLE) ||
			(getAudioTable() == ACOUSTIC_CAMCORDER_TABLE)) {
			if (!audio_data->is_speech_mode)
				return 0;
		}
#else
		if ((getAudioTable() == ACOUSTIC_DEVICE_MIC_MUSIC_RECOGNITION_TABLE) ||
			(getAudioTable() == ACOUSTIC_REAR_CAMCORDER_TABLE)) {
			return 0;
		}
#endif
		pr_info("[Audio] previous mode %d, current mode %d !\n", getAudioTable(), audio_data->mode);
		setAudioTable(audio_data->mode);
	}

	return 1;
}
#endif

void tegra_ext_control(struct snd_soc_codec *codec, int new_con)
{
	struct tegra_audio_data* audio_data = codec->socdev->codec_data;
#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA) || defined(CONFIG_MACH_ACER_VANGOGH)
	int CtrlReg = 0;
#endif

	/* Disconnect old codec routes and connect new routes*/
#ifdef MACH_ACER_AUDIO
	if (new_con & (TEGRA_HEADPHONE | TEGRA_VOIP_RINGTONE))
#else
	if (new_con & TEGRA_HEADPHONE)
#endif
		snd_soc_dapm_enable_pin(codec, "Headphone");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone");

#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA) || defined(CONFIG_MACH_ACER_VANGOGH)
	if (new_con & (TEGRA_LINEOUT | TEGRA_EAR_SPK | TEGRA_SPK))
		snd_soc_dapm_enable_pin(codec, "Lineout");
	else
		snd_soc_dapm_disable_pin(codec, "Lineout");
#else
	if (new_con & (TEGRA_LINEOUT | TEGRA_EAR_SPK))
		snd_soc_dapm_enable_pin(codec, "Lineout");
	else
		snd_soc_dapm_disable_pin(codec, "Lineout");

	if (new_con & TEGRA_SPK)
		snd_soc_dapm_enable_pin(codec, "Int Spk");
	else
		snd_soc_dapm_disable_pin(codec, "Int Spk");
#endif

	if (new_con & TEGRA_INT_MIC)
		snd_soc_dapm_enable_pin(codec, "Int Mic");
	else
		snd_soc_dapm_disable_pin(codec, "Int Mic");

	if (new_con & TEGRA_EXT_MIC)
		snd_soc_dapm_enable_pin(codec, "Ext Mic");
	else
		snd_soc_dapm_disable_pin(codec, "Ext Mic");

	if (new_con & TEGRA_LINEIN)
		snd_soc_dapm_enable_pin(codec, "Linein");
	else
		snd_soc_dapm_disable_pin(codec, "Linein");

	if (new_con & TEGRA_HEADSET_OUT)
		snd_soc_dapm_enable_pin(codec, "Headset Out");
	else
		snd_soc_dapm_disable_pin(codec, "Headset Out");

	if (new_con & TEGRA_HEADSET_IN)
		snd_soc_dapm_enable_pin(codec, "Headset In");
	else
		snd_soc_dapm_disable_pin(codec, "Headset In");

#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA) || defined(CONFIG_MACH_ACER_VANGOGH)
	if (new_con & TEGRA_MIC_MUTE) {
		CtrlReg = snd_soc_read(codec, WM8903_ANALOGUE_LEFT_INPUT_0);
		CtrlReg |= WM8903_LINMUTE;
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0, CtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0, CtrlReg);
	} else {
		CtrlReg = snd_soc_read(codec, WM8903_ANALOGUE_LEFT_INPUT_0);
		CtrlReg &= ~WM8903_LINMUTE;
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0, CtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0, CtrlReg);
	}

	if (new_con & (TEGRA_HEADPHONE | TEGRA_HEADSET_OUT |
			TEGRA_HEADSET_IN | TEGRA_VOIP_RINGTONE))
		hp_enable(codec, 1);
	else
		hp_enable(codec, 0);

	if (new_con & TEGRA_VOIP_CALL) {
		CtrlReg = snd_soc_read(codec, WM8903_ANALOGUE_LEFT_INPUT_0);
		CtrlReg &= ~0x1F;
		CtrlReg |= 0x10;
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0, CtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0, CtrlReg);

		if (new_con & (TEGRA_HEADSET_OUT | TEGRA_HEADSET_IN))
			voip_hp(codec, true);
		else if (new_con & (TEGRA_LINEOUT | TEGRA_EAR_SPK | TEGRA_SPK))
			speaker_louder(codec, true);
	}
	switch_audio_table(codec, new_con);
#endif

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);
	audio_data->codec_con = new_con;
}

static int tegra_dapm_event_int_spk(struct snd_soc_dapm_widget* w,
				    struct snd_kcontrol* k, int event)
{
	if (tegra_wired_jack_conf.en_spkr != -1) {
#ifdef MACH_ACER_AUDIO
		if (SND_SOC_DAPM_EVENT_ON(event)) {
#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA)
			gpio_set_value_cansleep(tegra_wired_jack_conf.en_spkr, 1);
#elif defined(CONFIG_MACH_ACER_VANGOGH)
			gpio_set_value_cansleep(tegra_wired_jack_conf.en_spkr_mute, 0);
#endif
			pr_info("Audio: speaker on!!\n");
		} else {
#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA)
			gpio_set_value_cansleep(tegra_wired_jack_conf.en_spkr, 0);
#elif defined(CONFIG_MACH_ACER_VANGOGH)
			gpio_set_value_cansleep(tegra_wired_jack_conf.en_spkr_mute, 1);
#endif
			pr_info("Audio: speaker off!!\n");
		}
#else
		if (tegra_wired_jack_conf.amp_reg) {
			if (SND_SOC_DAPM_EVENT_ON(event) &&
				!tegra_wired_jack_conf.amp_reg_enabled) {
				regulator_enable(tegra_wired_jack_conf.amp_reg);
				tegra_wired_jack_conf.amp_reg_enabled = 1;
			}
			else if (!SND_SOC_DAPM_EVENT_ON(event) &&
				tegra_wired_jack_conf.amp_reg_enabled) {
				regulator_disable(tegra_wired_jack_conf.amp_reg);
				tegra_wired_jack_conf.amp_reg_enabled = 0;
			}
		}

		gpio_set_value_cansleep(tegra_wired_jack_conf.en_spkr,
			SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0);
#endif
	}

	return 0;
}

static int tegra_dapm_event_int_mic(struct snd_soc_dapm_widget* w,
				    struct snd_kcontrol* k, int event)
{
#ifdef MACH_ACER_AUDIO
	int CtrlReg = 0;
	struct wm8903_priv *wm8903 = (struct wm8903_priv *)snd_soc_codec_get_drvdata(tegra_wired_jack_conf.codec);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_fm2018, 1);

		CtrlReg = (0x1 << B00_MICBIAS_ENA) | (0x1 << B01_MICDET_ENA);
		snd_soc_write(tegra_wired_jack_conf.codec,
			WM8903_MIC_BIAS_CONTROL_0, CtrlReg);

#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
		// Enable analog inputs
		CtrlReg = (0x1<<B01_INL_ENA);
		snd_soc_write(tegra_wired_jack_conf.codec, WM8903_POWER_MANAGEMENT_0, CtrlReg);

		// Enable ADC
		CtrlReg = snd_soc_read(tegra_wired_jack_conf.codec, WM8903_POWER_MANAGEMENT_6);
		CtrlReg |= (0x1<<B01_ADCL_ENA);
		snd_soc_write(tegra_wired_jack_conf.codec, WM8903_POWER_MANAGEMENT_6, CtrlReg);
#endif

		pr_info("Audio: int mic enable! \n");
	} else {
		if (wm8903->capture_active > 0) {
			CtrlReg = (0x1 << B00_MICBIAS_ENA) | (0x1 << B01_MICDET_ENA);
			snd_soc_write(tegra_wired_jack_conf.codec,
			WM8903_MIC_BIAS_CONTROL_0, CtrlReg);
			pr_info("Audio: enable mic bias! \n");
		} else {
			gpio_set_value_cansleep(tegra_wired_jack_conf.en_fm2018, 0);

			snd_soc_write(tegra_wired_jack_conf.codec,
				WM8903_MIC_BIAS_CONTROL_0, CtrlReg);
			pr_info("Audio: int mic disable! \n");
		}
	}
#else
	if (tegra_wired_jack_conf.en_mic_int != -1)
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_mic_int,
			SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0);

	if (tegra_wired_jack_conf.en_mic_ext != -1)
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_mic_ext,
			SND_SOC_DAPM_EVENT_ON(event) ? 0 : 1);
#endif

	return 0;
}

static int tegra_dapm_event_ext_mic(struct snd_soc_dapm_widget* w,
				    struct snd_kcontrol* k, int event)
{
#ifdef MACH_ACER_AUDIO
	int CtrlReg = 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_fm2018, 1);
#endif
		CtrlReg = (0x1 << B00_MICBIAS_ENA) | (0x1 << B01_MICDET_ENA);
		snd_soc_write(tegra_wired_jack_conf.codec,
			WM8903_MIC_BIAS_CONTROL_0, CtrlReg);

		pr_info("Audio: ext mic enable! \n");
	} else {
#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_fm2018, 0);
#endif
		snd_soc_write(tegra_wired_jack_conf.codec,
			WM8903_MIC_BIAS_CONTROL_0, CtrlReg);

		pr_info("Audio: ext mic disable! \n");
	}
#else
	if (tegra_wired_jack_conf.en_mic_ext != -1)
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_mic_ext,
			SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0);

	if (tegra_wired_jack_conf.en_mic_int != -1)
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_mic_int,
			SND_SOC_DAPM_EVENT_ON(event) ? 0 : 1);
#endif

	return 0;
}

#ifdef CONFIG_MACH_ACER_VANGOGH
static void wm8903_gpio_intr_work(struct work_struct *work)
{
	if (gpio_get_value(GPIO_CDC_IRQ))
	{
		snd_soc_write(wm8903_gpio_intr->codec, WM8903_GPIO_CONTROL_3, 0x10);
		snd_soc_write(wm8903_gpio_intr->codec, WM8903_GPIO_CONTROL_1, 0x0);
	}
	else
	{
		snd_soc_write(wm8903_gpio_intr->codec, WM8903_GPIO_CONTROL_3, 0x00);
		snd_soc_write(wm8903_gpio_intr->codec, WM8903_GPIO_CONTROL_1, 0x10);
	}
}

static irqreturn_t wm8903_cdc_irq(int irq, void *data)
{
	schedule_work(&wm8903_gpio_intr->work);
	return IRQ_HANDLED;
}

static int wm8903_cdc_irq_init(struct snd_soc_codec *codec)
{
	int ret;

	if (!wm8903_gpio_intr)
	{
		wm8903_gpio_intr = kzalloc(sizeof(*wm8903_gpio_intr), GFP_KERNEL);
		if (!wm8903_gpio_intr) {
			pr_err("failed to allocate wm8903 cdc irq interrupt \n");
			return -ENOMEM;
		}

		wm8903_gpio_intr->gpio = GPIO_CDC_IRQ;
		wm8903_gpio_intr->codec = codec;
		INIT_WORK(&wm8903_gpio_intr->work, wm8903_gpio_intr_work);

		ret = gpio_request(wm8903_gpio_intr->gpio, "cdc-irq-gpio");
		if (ret) {
			pr_err("failed to request cdc irq gpio \n");
			goto failed;
		}

		ret = gpio_direction_input(wm8903_gpio_intr->gpio);
		if (ret) {
			pr_err("failed to config cdc irq gpio \n");
			goto gpio_failed;
		}

		tegra_gpio_enable(wm8903_gpio_intr->gpio);
		ret = request_irq(gpio_to_irq(wm8903_gpio_intr->gpio),wm8903_cdc_irq,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "wm8903_gpio_intr", wm8903_gpio_intr);

		if (ret) {
			pr_err("failed to request cdc irq \n");
			goto gpio_failed;
		}
	}
	return ret;

gpio_failed:
	gpio_free(wm8903_gpio_intr->gpio);
failed:
	kfree(wm8903_gpio_intr);
	wm8903_gpio_intr = NULL;
	return ret;
}
#endif

/*tegra machine dapm widgets */
static const struct snd_soc_dapm_widget tegra_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_HP("Headset Out", NULL),
#if defined(MACH_ACER_AUDIO)
	SND_SOC_DAPM_MIC("Headset In", tegra_dapm_event_ext_mic),
	SND_SOC_DAPM_SPK("Lineout", tegra_dapm_event_int_spk),
#else
	SND_SOC_DAPM_MIC("Headset In", NULL),
	SND_SOC_DAPM_SPK("Lineout", NULL),
#endif
	SND_SOC_DAPM_SPK("Int Spk", tegra_dapm_event_int_spk),
	SND_SOC_DAPM_MIC("Ext Mic", tegra_dapm_event_ext_mic),
	SND_SOC_DAPM_MIC("Int Mic", tegra_dapm_event_int_mic),
	SND_SOC_DAPM_LINE("Linein", NULL),
};

/* Tegra machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map[] = {

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone", NULL, "HPOUTR"},
	{"Headphone", NULL, "HPOUTL"},

	/* headset Jack  - in = micin, out = HPOUT*/
	{"Headset Out", NULL, "HPOUTR"},
	{"Headset Out", NULL, "HPOUTL"},
#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
	{"IN1L", NULL, "Headset In"},
#elif defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA)
	{"IN2L", NULL, "Headset In"},
#else
	{"IN1L", NULL, "Headset In"},
#endif

	/* lineout connected to LINEOUTR and LINEOUTL */
	{"Lineout", NULL, "LINEOUTR"},
	{"Lineout", NULL, "LINEOUTL"},

	/* build-in speaker connected to LON/P RON/P */
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "LON"},
	{"Int Spk", NULL, "LOP"},

	/* internal mic is mono */
	{"IN1R", NULL, "Int Mic"},

	/* external mic is stereo */
#if defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_PICASSO_E)
	{"IN1L", NULL, "Ext Mic"},
	{"IN1R", NULL, "Ext Mic"},
#elif defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_MAYA)
	{"IN2L", NULL, "Ext Mic"},
	{"IN2R", NULL, "Ext Mic"},
#else
	{"IN1L", NULL, "Ext Mic"},
	{"IN1R", NULL, "Ext Mic"},
#endif

	/* Line In */
	{"IN3L", NULL, "Linein"},
	{"IN3R", NULL, "Linein"},
};


static int tegra_codec_init(struct snd_soc_codec *codec)
{
	struct tegra_audio_data* audio_data = codec->socdev->codec_data;
	int err = 0;
#ifdef CONFIG_MACH_ACER_VANGOGH
	int ret = 0;
#endif

	if (!audio_data->init_done) {
		audio_data->dap_mclk = tegra_das_get_dap_mclk();
		if (!audio_data->dap_mclk) {
			pr_err("Failed to get dap mclk \n");
			err = -ENODEV;
			return err;
		}

		/* Add tegra specific widgets */
		snd_soc_dapm_new_controls(codec, tegra_dapm_widgets,
					ARRAY_SIZE(tegra_dapm_widgets));

		/* Set up tegra specific audio path audio_map */
		snd_soc_dapm_add_routes(codec, audio_map,
					ARRAY_SIZE(audio_map));

		/* Add jack detection */
		err = tegra_jack_init(codec);
		if (err < 0) {
			pr_err("Failed in jack init \n");
			return err;
		}

		/* Default to OFF */
		tegra_ext_control(codec, TEGRA_AUDIO_OFF);

		err = tegra_controls_init(codec);
		if (err < 0) {
			pr_err("Failed in controls init \n");
			return err;
		}

#ifdef CONFIG_MACH_ACER_VANGOGH
		ret = wm8903_cdc_irq_init(codec);
		if (ret < 0) {
			pr_err("Failed in cdc irq init \n");
		}
#endif

#ifdef MACH_ACER_AUDIO
		snd_soc_write(codec, WM8903_MIC_BIAS_CONTROL_0, 0);
#endif

		audio_data->codec = codec;
		audio_data->init_done = 1;
	}

	return err;
}

static struct snd_soc_dai_link tegra_soc_dai[] = {
	{
		.name = "WM8903",
		.stream_name = "WM8903 HiFi",
		.cpu_dai = &tegra_i2s_dai[0],
		.codec_dai = &wm8903_dai,
		.init = tegra_codec_init,
		.ops = &tegra_hifi_ops,
	},
	{
		.name = "Tegra-generic",
		.stream_name = "Tegra Generic Voice",
		.cpu_dai = &tegra_i2s_dai[1],
		.codec_dai = &tegra_generic_codec_dai[0],
		.init = tegra_codec_init,
		.ops = &tegra_voice_ops,
	},
	{
		.name = "Tegra-spdif",
		.stream_name = "Tegra Spdif",
		.cpu_dai = &tegra_spdif_dai,
		.codec_dai = &tegra_generic_codec_dai[1],
		.init = tegra_codec_init,
		.ops = &tegra_spdif_ops,
	},
};

static struct tegra_audio_data audio_data = {
	.init_done = 0,
	.play_device = TEGRA_AUDIO_DEVICE_NONE,
	.capture_device = TEGRA_AUDIO_DEVICE_NONE,
	.is_call_mode = false,
	.codec_con = TEGRA_AUDIO_OFF,
};

static struct snd_soc_card tegra_snd_soc = {
	.name = "tegra",
	.platform = &tegra_soc_platform,
	.dai_link = tegra_soc_dai,
	.num_links = ARRAY_SIZE(tegra_soc_dai),
	.suspend_pre = tegra_soc_suspend_pre,
	.suspend_post = tegra_soc_suspend_post,
	.resume_pre = tegra_soc_resume_pre,
	.resume_post = tegra_soc_resume_post,
};

static struct snd_soc_device tegra_snd_devdata = {
	.card = &tegra_snd_soc,
	.codec_dev = &soc_codec_dev_wm8903,
	.codec_data = &audio_data,
};

static int __init tegra_init(void)
{
	int ret = 0;

	tegra_snd_device = platform_device_alloc("soc-audio", -1);
	if (!tegra_snd_device) {
		pr_err("failed to allocate soc-audio \n");
		return -ENOMEM;
	}

	platform_set_drvdata(tegra_snd_device, &tegra_snd_devdata);
	tegra_snd_devdata.dev = &tegra_snd_device->dev;

	ret = platform_device_add(tegra_snd_device);
	if (ret) {
		pr_err("audio device could not be added \n");
		goto fail;
	}

	ret = device_create_file(&tegra_snd_device->dev,
							&dev_attr_enable_digital_mic);
	if (ret < 0) {
		dev_err(&tegra_snd_device->dev,
				"%s: could not create sysfs entry %s: %d\n",
				__func__, dev_attr_enable_digital_mic.attr.name, ret);
		goto fail;
	}

	reg_vmic = regulator_get(&tegra_snd_device->dev, "vmic");
	if (IS_ERR_OR_NULL(reg_vmic)) {
		pr_err("Couldn't get vmic regulator\n");
		reg_vmic = NULL;
	}

	return 0;

fail:
	if (tegra_snd_device) {
		platform_device_put(tegra_snd_device);
		tegra_snd_device = 0;
	}

	return ret;
}

static void __exit tegra_exit(void)
{
	tegra_jack_exit();
	if (reg_vmic) {
		regulator_put(reg_vmic);
		reg_vmic = NULL;
	}
	platform_device_unregister(tegra_snd_device);
}

module_init(tegra_init);
module_exit(tegra_exit);

/* Module information */
MODULE_DESCRIPTION("Tegra ALSA SoC");
MODULE_LICENSE("GPL");
