/*
 * arch/arm/mach-tegra/board-harmony-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include "gpio-names.h"
#include "board.h"

#define VENTANA_WLAN_PWR	TEGRA_GPIO_PK5
#define VENTANA_WLAN_RST	TEGRA_GPIO_PK6
#define VENTANA_WLAN_IRQ	TEGRA_GPIO_PS0
#if defined(CONFIG_MACH_ACER_PICASSO_E)
#define VENTANA_BT_RST		TEGRA_GPIO_PU0
#define VENTANA_BCM_VDD	TEGRA_GPIO_PD1
#endif

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int ventana_wifi_status_register(void (*callback)(int , void *), void *);
static struct clk *wifi_32k_clk;

static int ventana_wifi_reset(int on);
static int ventana_wifi_power(int on);
static int ventana_wifi_set_carddetect(int val);

#if defined(CONFIG_BCM4329_HW_OOB) || defined(CONFIG_BCM4329_OOB_INTR_ONLY)
static struct resource ventana_wifi_wakeup_resources[] = {
       {
               .name   = "bcm4329_wlan_irq",
               .start  = TEGRA_GPIO_TO_IRQ(VENTANA_WLAN_IRQ),
               .end    = TEGRA_GPIO_TO_IRQ(VENTANA_WLAN_IRQ),
               .flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
       },
};
#endif //CONFIG_BCM4329_HW_OOB || CONFIG_BCM4329_OOB_INTR_ONLY

static struct wifi_platform_data ventana_wifi_control = {
	.set_power      = ventana_wifi_power,
	.set_reset      = ventana_wifi_reset,
	.set_carddetect = ventana_wifi_set_carddetect,
};

static struct platform_device ventana_wifi_device = {
	.name           = "bcm4329_wlan",
	.id             = 1,
#if defined(CONFIG_BCM4329_HW_OOB) || defined(CONFIG_BCM4329_OOB_INTR_ONLY)
	.num_resources  = ARRAY_SIZE(ventana_wifi_wakeup_resources),
	.resource       = ventana_wifi_wakeup_resources,
#endif //CONFIG_BCM4329_HW_OOB || CONFIG_BCM4329_OOB_INTR_ONLY
	.dev            = {
		.platform_data = &ventana_wifi_control,
	},
};

static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};


static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.clk_id = NULL,
	.force_hs = 0,
	.register_status_notify	= ventana_wifi_status_register,
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4329,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
};

#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_MAYA) || defined(CONFIG_MACH_ACER_PICASSO_E)
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.clk_id = NULL,
	.force_hs = 1,
	.cd_gpio = TEGRA_GPIO_PI5,
	.cd_gpio_polarity = 0,
	.wp_gpio = -1,
	.power_gpio = TEGRA_GPIO_PI6,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.clk_id = NULL,
	.force_hs = 0,
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
};
#else
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.clk_id = NULL,
	.force_hs = 1,
	.cd_gpio = TEGRA_GPIO_PI5,
	.wp_gpio = TEGRA_GPIO_PH1,
	.power_gpio = TEGRA_GPIO_PT3,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.clk_id = NULL,
	.force_hs = 0,
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = TEGRA_GPIO_PI6,
};
#endif

static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int ventana_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int ventana_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

#if defined(CONFIG_MACH_ACER_PICASSO_E)
static int wifi_sdio_gpio[] = {
	TEGRA_GPIO_PZ0,
	TEGRA_GPIO_PZ1,
	TEGRA_GPIO_PY7,
	TEGRA_GPIO_PY6,
	TEGRA_GPIO_PY5,
	TEGRA_GPIO_PY4,
};

static int enable_wifi_sdio_func(void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(wifi_sdio_gpio); i++) {
		tegra_gpio_disable(wifi_sdio_gpio[i]);
		gpio_free(wifi_sdio_gpio[i]);
	}
	return 0;
}

static int disable_wifi_sdio_func(void)
{
	unsigned int rc = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(wifi_sdio_gpio); i++) {
		rc = gpio_request(wifi_sdio_gpio[i], NULL);
		if (rc) {
			printk(KERN_INFO "%s, request gpio %d failed !!!\n", __func__, wifi_sdio_gpio[i]);
			return rc;
		}

		tegra_gpio_enable(wifi_sdio_gpio[i]);

		rc = gpio_direction_output(wifi_sdio_gpio[i], 0);
		if (rc) {
			printk(KERN_INFO "%s, direction gpio %d failed !!!\n", __func__, wifi_sdio_gpio[i]);
			return rc;
		}
	}
	return 0;
}
#endif

static int ventana_wifi_power(int on)
{
	pr_debug("%s: %d\n", __func__, on);

	if (on)
		gpio_direction_input(VENTANA_WLAN_IRQ);
	else
		gpio_direction_output(VENTANA_WLAN_IRQ, 0);

#if defined(CONFIG_MACH_ACER_PICASSO_E)
	/* Set VDD high at first before turning on*/
	if (on) {
		enable_wifi_sdio_func();
		if (!gpio_get_value(VENTANA_BCM_VDD)) {
			gpio_set_value(VENTANA_BCM_VDD, 1);
			pr_err("%s: VDD=1\n", __func__);
		}
	}
#endif

	mdelay(50);
	gpio_set_value(VENTANA_WLAN_RST, on);
	mdelay(80);

	if (on)
		clk_enable(wifi_32k_clk);
	else
		clk_disable(wifi_32k_clk);

#if defined(CONFIG_MACH_ACER_PICASSO_E)
	/*
	 * When BT and Wi-Fi turn off at the same time, the last one must do the VDD off action.
	 * So BT/WI-FI must check the other's status in order to set VDD low at last.
	 */
	if (!on) {
		if (!gpio_get_value(VENTANA_BT_RST)) {
			gpio_set_value(VENTANA_BCM_VDD, 0);
			pr_err("%s: VDD=0\n", __func__);
		}
		disable_wifi_sdio_func();
	}
#endif
	return 0;
}

static int ventana_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static int __init ventana_wifi_init(void)
{
	wifi_32k_clk = clk_get_sys(NULL, "blink");
	if (IS_ERR(wifi_32k_clk)) {
		pr_err("%s: unable to get blink clock\n", __func__);
		return PTR_ERR(wifi_32k_clk);
	}

#if defined(CONFIG_BCM4329_HW_OOB) || defined(CONFIG_BCM4329_OOB_INTR_ONLY)
       gpio_request(VENTANA_WLAN_IRQ,"oob irq");
       tegra_gpio_enable(VENTANA_WLAN_IRQ);
       gpio_direction_input(VENTANA_WLAN_IRQ);
#endif //CONFIG_BCM4329_HW_OOB || CONFIG_BCM4329_OOB_INTR_ONLY


	gpio_request(VENTANA_WLAN_PWR, "wlan_power");
	gpio_request(VENTANA_WLAN_RST, "wlan_rst");

	tegra_gpio_enable(VENTANA_WLAN_PWR);
	tegra_gpio_enable(VENTANA_WLAN_RST);

	gpio_direction_output(VENTANA_WLAN_PWR, 0);
	gpio_direction_output(VENTANA_WLAN_RST, 0);

#if defined(CONFIG_MACH_ACER_PICASSO_E)
	gpio_request(VENTANA_BCM_VDD, "bcm_vdd");
	tegra_gpio_enable(VENTANA_BCM_VDD);
	gpio_direction_output(VENTANA_BCM_VDD, 0);
#endif
	platform_device_register(&ventana_wifi_device);

	device_init_wakeup(&ventana_wifi_device.dev, 1);
	device_set_wakeup_enable(&ventana_wifi_device.dev, 0);

#if defined(CONFIG_MACH_ACER_PICASSO_E)
	disable_wifi_sdio_func();
#endif
	return 0;
}
int __init ventana_sdhci_init(void)
{
#if defined(CONFIG_MACH_ACER_PICASSO) || defined(CONFIG_MACH_ACER_VANGOGH) || defined(CONFIG_MACH_ACER_MAYA) || defined(CONFIG_MACH_ACER_PICASSO_E)
	gpio_request(tegra_sdhci_platform_data2.power_gpio, "sdhci2_power");
	gpio_request(tegra_sdhci_platform_data2.cd_gpio, "sdhci2_cd");

	tegra_gpio_enable(tegra_sdhci_platform_data2.power_gpio);
	tegra_gpio_enable(tegra_sdhci_platform_data2.cd_gpio);

	gpio_direction_output(tegra_sdhci_platform_data2.power_gpio, 1);
#else
	gpio_request(tegra_sdhci_platform_data2.power_gpio, "sdhci2_power");
	gpio_request(tegra_sdhci_platform_data2.cd_gpio, "sdhci2_cd");
	gpio_request(tegra_sdhci_platform_data2.wp_gpio, "sdhci2_wp");
	gpio_request(tegra_sdhci_platform_data3.power_gpio, "sdhci3_power");

	tegra_gpio_enable(tegra_sdhci_platform_data2.power_gpio);
	tegra_gpio_enable(tegra_sdhci_platform_data2.cd_gpio);
	tegra_gpio_enable(tegra_sdhci_platform_data2.wp_gpio);
	tegra_gpio_enable(tegra_sdhci_platform_data3.power_gpio);

	gpio_direction_output(tegra_sdhci_platform_data2.power_gpio, 1);
	gpio_direction_output(tegra_sdhci_platform_data3.power_gpio, 1);
	gpio_set_value(tegra_sdhci_platform_data3.power_gpio, 1);
#endif
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);
	platform_device_register(&tegra_sdhci_device0);

	ventana_wifi_init();
	return 0;
}
