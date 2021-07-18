/********************************************************************
 *
 * Kathrein UFS-910 14W frontpanel LED driver
 *
 * - 20.Jun 2008 - captaintrip
 *
 * ToDo:
 * + suspend & resume
 * + .... 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif

#include "ufs910_14w_led.h"

static struct stpio_pin *ledgreen;
static struct stpio_pin *ledyellow;
static struct stpio_pin *ledred;

static void ufs910led_green_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	dprintk("[LED] %s > \n", __func__);
	
	if (ledgreen == NULL)
	{
		dprintk("[LED] ledgreen is NULL, requesting pin\n");
		ledgreen = stpio_request_pin(0, 0, "LED", STPIO_OUT);
	}
	if (value)
	{
		dprintk("[LED] %s ON\n", __func__);
		stpio_set_pin(ledgreen, 0);
	}
	else
	{
		dprintk("[LED] %s OFF\n", __func__);
		stpio_set_pin(ledgreen, 1);
	}
}

static void ufs910led_yellow_set(struct led_classdev *led_cdev,	enum led_brightness value)
{
	dprintk("[LED] %s > \n", __func__);
	
	if (ledyellow == NULL)
	{
		dprintk("[LED] ledyellow is NULL, requesting pin\n");
		ledyellow = stpio_request_pin(0, 1, "LED", STPIO_OUT);
	}
	if (value)
	{
		dprintk("[LED] %s ON\n", __func__);
		stpio_set_pin(ledyellow, 0);
	}
	else
	{
		dprintk("[LED] %s OFF\n", __func__);
		stpio_set_pin(ledyellow, 1);
	}
}

static void ufs910led_red_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	dprintk("[LED] %s > \n", __func__);
	
	if (ledred == NULL)
	{
		dprintk("[LED] ledred is NULL, requesting pin\n");
		ledred = stpio_request_pin(0, 2, "LED", STPIO_OUT);
	}
	if (value)
	{
		dprintk("[LED] %s ON\n", __func__);
		stpio_set_pin(ledred, 0);
	}
	else
	{
		dprintk("[LED] %s OFF\n", __func__);
		stpio_set_pin(ledred, 1);
	}
}


static struct led_classdev ufs910_green_led =
{
	.name            = "ufs910:green",
	.default_trigger = "",
	.brightness_set  = ufs910led_green_set
};

static struct led_classdev ufs910_yellow_led =
{
	.name            = "ufs910:yellow",
	.default_trigger = "",
	.brightness_set  = ufs910led_yellow_set
};

static struct led_classdev ufs910_red_led =
{
	.name            = "ufs910:red",
	.default_trigger = "",
	.brightness_set  = ufs910led_red_set
};

#define ufs910led_suspend NULL
#define ufs910led_resume NULL

static int ufs910led_probe(struct platform_device *pdev)
{
	int ret;

	ret = led_classdev_register(&pdev->dev, &ufs910_green_led);
	if (ret < 0)
	{
		dprintk("[LED] ufs910led_probe - till green - result = %i\n", ret);
		return ret;
	}

	ret = led_classdev_register(&pdev->dev, &ufs910_yellow_led);
	if (ret < 0)
	{
		led_classdev_unregister(&ufs910_green_led);
		dprintk("[LED] ufs910led_probe - till yellow - result = %i\n", ret);
		return ret;
	}

	ret = led_classdev_register(&pdev->dev, &ufs910_red_led);
	if (ret < 0)
	{
		led_classdev_unregister(&ufs910_green_led);
		led_classdev_unregister(&ufs910_yellow_led);
	}
	dprintk("[LED] ufs910led_probe - till red - result = %i\n", ret);

	return ret;
}

static int ufs910led_remove(struct platform_device *pdev)
{
	dprintk("[LED] ufs910led_remove\n");
	led_classdev_unregister(&ufs910_green_led);
	led_classdev_unregister(&ufs910_yellow_led);
	led_classdev_unregister(&ufs910_red_led);
	
	if (ledgreen != NULL)
	{
		stpio_free_pin(ledgreen);
	}
	if (ledyellow != NULL)
	{
		stpio_free_pin(ledyellow);
	}
	if (ledred != NULL)
	{
		stpio_free_pin(ledred);
	}
	return 0;
}

static struct platform_driver ufs910led_driver =
{
	.probe    = ufs910led_probe,
	.remove   = ufs910led_remove,
	.suspend  = ufs910led_suspend,
	.resume   = ufs910led_resume,
	.driver   =
	{
		.name = "ufs910-led",
	},
};

static int __init ufs910led_init(void)
{
	dprintk("[LED] initializing...\n");
	return platform_driver_register(&ufs910led_driver);
}

static void __exit ufs910led_exit(void)
{
	dprintk("[LED] unloading...\n");
 	platform_driver_unregister(&ufs910led_driver);
}

module_init(ufs910led_init);
module_exit(ufs910led_exit);

MODULE_AUTHOR("Open Vision developers");
MODULE_DESCRIPTION("Kathrein UFS-910 14W frontpanel LED driver");
MODULE_LICENSE("GPL");
// vim:ts=4
