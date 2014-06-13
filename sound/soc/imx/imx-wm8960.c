/*
* mx6_wm8960.c  --  WM8960 ALSA SoC Audio driver
*
* 
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/
//63529411 i2s
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/kthread.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/jack.h>
#include <mach/dma.h>
#include <mach/clock.h>
#include <mach/audmux.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>


//#include <asm/hardware.h>
//#include <asm/arch/dam.h>

#include "../codecs/wm8960.h"
#include "imx-pcm.h"
#include "imx-ssi.h"

/* BCLK clock dividers */
#define WM8960_BCLK_DIV_1 (0)
#define WM8960_BCLK_DIV_2 (1 << 1)
#define WM8960_BCLK_DIV_4 (1 << 2)
#define WM8960_BCLK_DIV_8 (0x7)
#define WM8960_BCLK_DIV_16 (0xa)

/*
* DAI Left/Right Clocks.
*
* Specifies whether the DAI can support different samples for similtanious
* playback and capture. This usually requires a seperate physical frame
* clock for playback and capture.
*/
#define SND_SOC_DAIFMT_SYNC (0 << 5) /* Tx FRM = Rx FRM */
#define SND_SOC_DAIFMT_ASYNC (1 << 5) /* Tx FRM ~ Rx FRM */


struct imx_priv {
    int sysclk;         /*mclk from the outside*/
    int codec_sysclk;
    int dai_hifi;
    int hp_irq;
    int hp_status;
    int amic_irq;
    int amic_status;
    struct platform_device *pdev;
    struct snd_pcm_substream *first_stream;
    struct snd_pcm_substream *second_stream;
};
unsigned int sample_format = SNDRV_PCM_FMTBIT_S16_LE;
static struct imx_priv card_priv;
static struct snd_soc_card snd_soc_card_imx;

struct wm8960_setup_data {
    unsigned short i2c_address;
};
int wm8960_capture;



static int imx6_hw_params(struct snd_pcm_substream *substream,
                          struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *codec_dai = rtd->codec_dai;
    struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    int    ret = 0;
    unsigned int pll_out = 0, bclk = 0;
    unsigned int sysclk_div = 0, dacdiv = 0, adcdiv = 0;
    unsigned int channels = params_channels(params);
    int pll_id = 0;
    switch (params_rate(params)) {
    case 8000:
    case 16000:
        bclk = WM8960_BCLK_DIV_8;
        pll_out = 11289600;
        sysclk_div = WM8960_SYSCLK_DIV_1;
        dacdiv = 0x28;
        adcdiv = 0x140;
        pll_id = 0;
        break;
    case 48000:
        sysclk_div = WM8960_SYSCLK_DIV_1;
        pll_out = 12288000;
        if (channels == 1) {
            /* Mono mode */
            dacdiv = WM8960_DAC_DIV_2; /* 24.0 KHz */
            bclk = WM8960_BCLK_DIV_8;
        }
        else {
            /* Stereo Mode */
            dacdiv = WM8960_DAC_DIV_1; /* 48.0 KHz */
            bclk = WM8960_BCLK_DIV_4;
        }
        pll_id = 1;
        break;
     case 32000:
        bclk = WM8960_BCLK_DIV_8;
        pll_out = 11289600;
        sysclk_div = WM8960_SYSCLK_DIV_1;
        dacdiv = WM8960_DAC_DIV_1;
        break;
      case 11025:
        bclk = WM8960_BCLK_DIV_8;
        pll_out = 11289600;
        sysclk_div =  WM8960_SYSCLK_DIV_1;
        dacdiv = WM8960_DAC_DIV_4;
        break;
       case 22050:
        if (channels == 1) {
            /* Mono Mode */
            bclk = WM8960_BCLK_DIV_16;
            dacdiv = WM8960_DAC_DIV_4; /* 11.025 KHz */
        }
        else {
            /* Stereo Mode */
            bclk = WM8960_BCLK_DIV_8;
            dacdiv = WM8960_DAC_DIV_2; /* 22.05 KHz */
        }
        pll_out = 11289600;
        sysclk_div = WM8960_SYSCLK_DIV_1;
        break;
       case 44100:
        if (channels == 1) {
            /* Mono mode */
            dacdiv = WM8960_DAC_DIV_2; /* 22.05 KHz */
            bclk = WM8960_BCLK_DIV_16;
        }
        else {
            /* Stereo Mode */
            dacdiv =WM8960_DAC_DIV_1; /* 44.1 KHz */
            bclk = WM8960_BCLK_DIV_8;
        }
        sysclk_div = WM8960_SYSCLK_DIV_1;
        pll_out = 11289600;
        break;
      default:
        printk("do not support this sample frequency");
        return -EINVAL;
    }
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        wm8960_capture = 0;
    else
        wm8960_capture = 1;
    printk("sysclk_div=%d,dacdiv=%d,bclk=%d,pll_out=%d\n",sysclk_div,dacdiv,bclk,pll_out);
    //i2s mode select
    ret = snd_soc_dai_set_fmt( codec_dai, SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF|SND_SOC_DAIFMT_CBM_CFM|SND_SOC_DAIFMT_SYNC);
    if( ret < 0 ){
        printk( "%s: Codec DAI configuration error, %d\n", __func__, ret );
        return ret;
    }
    ret = snd_soc_dai_set_fmt( cpu_dai, SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF|SND_SOC_DAIFMT_CBM_CFM|SND_SOC_DAIFMT_SYNC);
    if( ret < 0 ){
        printk( "%s: AP DAI configuration error, %d\n", __func__, ret );
        return ret;
    }
    snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

    //codec and pll
    ret = snd_soc_dai_set_clkdiv( codec_dai, WM8960_SYSCLKDIV, sysclk_div );// /1
    if( ret < 0 ){
        printk( "%s: Codec SYSCLKDIV setting error, %d\n", __func__, ret );
        return ret;
    }

    ret = snd_soc_dai_set_clkdiv( codec_dai, WM8960_DACDIV, dacdiv);//lracclk= sysclk/1*256
    if( ret < 0 ){
        printk( "%s: Codec DACDIV setting error, %d\n", __func__, ret );
        return ret;
    }

    ret = snd_soc_dai_set_clkdiv( codec_dai, WM8960_BCLKDIV, bclk);//bitclk=sysclk/8 
    if( ret < 0 ){
        printk( "%s: Codec WM8960_BCLKDIV setting error, %d\n", __func__, ret );
        return ret;
    }
    ret=snd_soc_dai_set_pll(codec_dai,1,0,12000000,pll_out);
    if( ret < 0 ){
        printk( "%s: AP codec pll error, %d\n", __func__, ret );
        return ret;
    }
#if 0
    //cpu clk
    ret = snd_soc_dai_set_clkdiv( cpu_dai, IMX_SSI_TX_DIV_2, 0 );//3
    if( ret < 0 ){
        printk( "%s: AP prescalar setting error, %d\n", __func__, ret );
        return ret;
    }

    ret = snd_soc_dai_set_clkdiv( cpu_dai,IMX_SSI_TX_DIV_PSR, 1 );//2
    if( ret < 0 ){
        printk( "%s: AP RFS setting error, %d\n", __func__, ret );
        return ret;
    }

    ret = snd_soc_dai_set_clkdiv( cpu_dai, IMX_SSI_TX_DIV_PM, 4 );
    if( ret < 0 ){
        printk( "%s: AP BFS setting error, %d\n", __func__, ret );//1
        return ret;
    }
#endif

    return 0;
}
static int imx6_hw_free(struct snd_pcm_substream *substream)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *codec_dai = rtd->codec_dai;

    /* disable the PLL */
    return snd_soc_dai_set_pll(codec_dai, 0,0, 0, 0);
}

static struct snd_soc_ops imx6_wm8960_ops = {
    .hw_params = imx6_hw_params,
    .hw_free = imx6_hw_free,
};

static const struct snd_soc_dapm_widget imx6_dapm_capture_widgets[] = {
    SND_SOC_DAPM_MIC(    "Mic Jack",            NULL ),
    SND_SOC_DAPM_LINE(    "Line Input 3 (FM)",NULL ),
};

static const struct snd_soc_dapm_widget imx6_dapm_playback_widgets[] = {
    SND_SOC_DAPM_HP(    "Headphone Jack",    NULL ),
    SND_SOC_DAPM_SPK(    "Speaker_L",        NULL ),
    SND_SOC_DAPM_SPK(    "Speaker_R",        NULL ),
};

static const struct snd_soc_dapm_route imx6_audio_map[] = {
    { "Headphone Jack",    NULL,    "HP_L"        },
    { "Headphone Jack",    NULL,     "HP_R"        },
    { "Speaker_L",        NULL,     "SPK_LP"    }, 
    { "Speaker_L",        NULL,     "SPK_LN"     }, 
    { "Speaker_R",        NULL,     "SPK_RP"     }, 
    { "Speaker_R",        NULL,     "SPK_RN"     }, 
    { "LINPUT1",        NULL,     "MICB"        },
    { "MICB",            NULL,     "Mic Jack"    },
};

static int imx6_wm8960_init(struct snd_soc_pcm_runtime *rtd)
{
    struct snd_soc_codec *codec = rtd->codec;
    struct snd_soc_dapm_context *dapm = &codec->dapm;

    snd_soc_dapm_nc_pin(dapm, "RINPUT1");
    snd_soc_dapm_nc_pin(dapm, "LINPUT2");
    snd_soc_dapm_nc_pin(dapm, "RINPUT2");
    snd_soc_dapm_nc_pin(dapm, "OUT3");
    
    snd_soc_dapm_new_controls( dapm, imx6_dapm_capture_widgets, ARRAY_SIZE( imx6_dapm_capture_widgets ) );
    snd_soc_dapm_new_controls( dapm, imx6_dapm_playback_widgets, ARRAY_SIZE( imx6_dapm_playback_widgets ) );

    snd_soc_dapm_add_routes( dapm, imx6_audio_map, ARRAY_SIZE( imx6_audio_map ) );

    snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
    snd_soc_dapm_enable_pin(dapm, "Mic Jack");
    snd_soc_dapm_enable_pin(dapm, "Speaker_L");
    snd_soc_dapm_enable_pin(dapm, "Speaker_R");
    
    snd_soc_dapm_disable_pin(dapm, "Line Input 3 (FM)");

    snd_soc_dapm_sync( dapm );

    return 0;
}

static struct snd_soc_dai_link imx_dai = {
    .name = "WM8960",
    .stream_name = "WM8960",
    .cpu_dai_name = "imx-ssi.1",
    .codec_dai_name = "wm8960-hifi",
    .platform_name = "imx-pcm-audio.1",
    .codec_name = "wm8960-codec.1-001a",
    .init = imx6_wm8960_init,
    .ops = &imx6_wm8960_ops,
};

static struct snd_soc_card snd_soc_card_imx = {
    .name = "wm8960-audio",
    .dai_link = &imx_dai,
    .num_links = 1,
};

static int imx_audmux_config(int slave, int master)
{
    unsigned int ptcr, pdcr;
    slave = slave - 1;
    master = master - 1;

    ptcr = MXC_AUDMUX_V2_PTCR_SYN |
           MXC_AUDMUX_V2_PTCR_TFSDIR |
           MXC_AUDMUX_V2_PTCR_TFSEL(master) |
           MXC_AUDMUX_V2_PTCR_TCLKDIR |
           MXC_AUDMUX_V2_PTCR_TCSEL(master);
    pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
    mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

    ptcr = MXC_AUDMUX_V2_PTCR_SYN;
    pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
    mxc_audmux_v2_configure_port(master, ptcr, pdcr);

    return 0;
}

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int __devinit imx_wm8960_probe(struct platform_device *pdev)
{

    struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
    struct imx_priv *priv = &card_priv;
    int ret = 0;
    priv->pdev = pdev;
    imx_audmux_config(plat->src_port, plat->ext_port);
    //imx_audmux_config(plat->ext_port,plat->src_port);
    if (plat->init && plat->init()) {
        ret = -EINVAL;
        return ret;
    }
    priv->sysclk = plat->sysclk;
    priv->first_stream = NULL;
    priv->second_stream = NULL;
    printk("imx_wm8960_probe suceesfull\n");
    return ret;
}

static int __devexit imx_wm8960_remove(struct platform_device *pdev)
{
    struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
    struct imx_priv *priv = &card_priv;

    if (plat->finit)
        plat->finit();

    if (priv->hp_irq)
        free_irq(priv->hp_irq, priv);
    if (priv->amic_irq)
        free_irq(priv->amic_irq, priv);

    return 0;
}

static struct platform_driver imx_wm8960_driver = {
    .probe = imx_wm8960_probe,
    .remove = imx_wm8960_remove,
    .driver = {
        .name = "imx-wm8960",
        .owner = THIS_MODULE,
    },
};

static struct platform_device *imx6_snd_device;

static int __init imx6_audio_init(void)
{
    int ret;
    ret = platform_driver_register(&imx_wm8960_driver);
    if (ret < 0)
    {   printk("imx_wm8960_driver register error\n");
        goto exit;
    }

    imx6_snd_device = platform_device_alloc("soc-audio", -1);
    if (!imx6_snd_device ){
        return -ENOMEM;
    }
    printk("wm8960 machine device add\n");
    platform_set_drvdata( imx6_snd_device, &snd_soc_card_imx );
    ret = platform_device_add( imx6_snd_device );
    if(ret){
	platform_device_put(imx6_snd_device);
        printk("wm8960 machine device add error\n");
    }
    exit:
    return ret;
}

static void __exit imx6_audio_exit(void)
{   
    platform_device_unregister( imx6_snd_device );
}


module_init( imx6_audio_init );
module_exit( imx6_audio_exit );

MODULE_AUTHOR("marui, marui@emsym.com");
MODULE_DESCRIPTION("ALSA SoC IMX6+WM8960");
MODULE_LICENSE("GPL");

