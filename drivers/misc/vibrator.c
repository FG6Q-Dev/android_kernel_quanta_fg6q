/* drivers/misc/drv8601-vib.c
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2008 Google, Inc.
 * Author: Larry Li <l-li@ti.com>
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
 * Derived from: two6040-vib.c
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c/twl.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/vibrator.h>
#include "../staging/android/vibration.h"

struct vibrator_data {
	struct vibration_dev dev; 
	struct work_struct vibrator_work;
	spinlock_t lock;
};

static int (*enable)(int);
static int (*time)(int);

/* Set vibrator on or off */
static void vibrator_set(int on)
{
	if(enable)
		enable(on);
}

/* Worker thread for timer ISR */
static void vibrator_update(struct work_struct *work)
{
}


/* Enable vibrator output for a given number of microseconds */
static void vibrator_enable(struct vibration_dev *dev, unsigned long value)
{
	//printk(KERN_ERR "vibration: %s\n", __func__);
	
	if (value < 0) {
		pr_err("%s: Invalid vibrator timer value\n", __func__);
		return;
	}

	vibrator_set(value);	
}

/* Below two API are required only for power management is enbled */
#if CONFIG_PM
static int vibrator_suspend(struct device *dev)
{	
	struct omap_vibrator *vib = dev->platform_data;

	vibrator_set(0);

	if(vib && vib->power_off)
		vib->power_off();
	
	return 0;
}

static int vibrator_resume(struct device *dev)
{	
	struct omap_vibrator *vib = dev->platform_data;

	if(vib && vib->power_on)
		vib->power_on();
	
	return 0;
}
#else
#define vibrator_suspend NULL
#define vibrator_resume  NULL
#endif

/* vibrator PM structure */
static const struct dev_pm_ops vibrator_pm_ops = {
	.suspend = vibrator_suspend,
	.resume = vibrator_resume,
};

/* initialize default pattern to support SEMCO LRA */
static void vibrator_init_pattern(struct vibration_dev *dev)
{

//	int i;

	/* Start phase */

	/* Sustain phase */

	/* Stop phase */

	/* PWM frequency */
}


/* vibrator probe function */
static int vibrator_probe(struct platform_device *pdev)
{
	struct vibrator_data *data;
	int ret = 0;

	printk(KERN_ERR "vibration: %s\n", __func__);

        /* Allocate omap_vib_data */
	data = kzalloc(sizeof(struct vibrator_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err0;
	}
	((struct omap_vibrator*)pdev->dev.platform_data)->init(); 
	
	enable = ((struct omap_vibrator*)pdev->dev.platform_data)->enable;	
    time = ((struct omap_vibrator*)pdev->dev.platform_data)->time; 

        /* Initialize pwm pattern */
    vibrator_init_pattern(&data->dev);

        /* Setup top-half processing */
	INIT_WORK(&data->vibrator_work, vibrator_update);

        /* Initialize the spin lock for use later */
	spin_lock_init(&data->lock);

        /* Setup and register timed_pwm device */ 
	data->dev.name = "vibrator";
	data->dev.enable = vibrator_enable;
	ret = vibration_dev_register(&data->dev);
	if (ret < 0)
		goto err1;

	/* misc_data now points to vib data instance */

	platform_set_drvdata(pdev, data);
		
	return 0;

err1:
	kfree(data);
err0:
	return ret;
}

/* Unregister vibrator as a timed_pwm device */ 
static int vibrator_remove(struct platform_device *pdev)
{
	struct vibrator_data *data = platform_get_drvdata(pdev);

        /* Disable OMAP timer */
	vibrator_set(0);

	vibration_dev_unregister(&data->dev);
	kfree(data);

	return 0;
}

static struct platform_driver vibrator_driver = {
	.probe = vibrator_probe,
	.remove = vibrator_remove,
	.driver = {
		   .name = "vibrator",      
		   .owner = THIS_MODULE,
		   .pm = &vibrator_pm_ops,  
	},
};

static int __init vibrator_init(void)
{
	printk(KERN_ERR "vibration: %s\n", __func__);

	return platform_driver_register(&vibrator_driver);
}

static void __exit vibrator_exit(void)
{
	platform_driver_unregister(&vibrator_driver);
}

module_init(vibrator_init);
module_exit(vibrator_exit);

MODULE_AUTHOR("Brox Chen <chun-ting.chen@quantatw.com>");
MODULE_DESCRIPTION("PWM Vibrator Driver");
MODULE_LICENSE("GPL v2");
