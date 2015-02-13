/* drivers/misc/timed_pwm.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Larry Li <l-li@ti.com>
 * Based on timed_output implementation 
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include "vibration.h"

static struct class *vibration_class;
static atomic_t device_count;

/***********  enable access methods ****************/
static ssize_t enable_show (struct device *dev, struct device_attribute *attr,
		char *buf)
{
//	printk(KERN_ERR "vibration: %s\n", __func__);

	return sprintf(buf, "%d\n", 0);
}

static ssize_t enable_store (
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct vibration_dev *vdev = dev_get_drvdata(dev);
	unsigned long value;

//	printk(KERN_ERR "vibration: %s\n", __func__);
	
	if (sscanf(buf, "%lu", &value) != 1)
		return -EINVAL;

	vdev->enable(vdev, value);

	return size;
}
static DEVICE_ATTR(enable, S_IRUSR|S_IRGRP | S_IWUSR|S_IWGRP, enable_show, enable_store);

static int create_vibration_class(void)
{
//	printk(KERN_ERR "vibration: %s\n", __func__);

	if (!vibration_class) {
		vibration_class = class_create(THIS_MODULE, "vibration");
		if (IS_ERR(vibration_class))
			return PTR_ERR(vibration_class);
		atomic_set(&device_count, 0);
	}

	return 0;
}

int vibration_dev_register(struct vibration_dev *vdev)
{
	int ret;

//	printk(KERN_ERR "vibration: %s\n", __func__);

	if (!vdev || !vdev->name || !vdev->enable)
	{
		printk(KERN_ERR "vibration: Failed to register driver %s\n",
			vdev->name);
		return -EINVAL;
	}
	ret = create_vibration_class();
	if (ret < 0)
		return ret;

	vdev->index = atomic_inc_return(&device_count);
	vdev->dev = device_create(vibration_class, NULL,
		MKDEV(0, vdev->index), NULL, vdev->name);
	if (IS_ERR(vdev->dev))
		return PTR_ERR(vdev->dev);

	ret = device_create_file(vdev->dev, &dev_attr_enable);
	if (ret < 0)
		goto err_create_file;
	dev_set_drvdata(vdev->dev, vdev);
	vdev->state = 0;
	return 0;

err_create_file:
	device_destroy(vibration_class, MKDEV(0, vdev->index));
	printk(KERN_ERR "vibration: Failed to register driver %s\n",
			vdev->name);

	return ret;
}
EXPORT_SYMBOL_GPL(vibration_dev_register);

void vibration_dev_unregister(struct vibration_dev *vdev)
{
	device_remove_file(vdev->dev, &dev_attr_enable);
	device_destroy(vibration_class, MKDEV(0, vdev->index));
	dev_set_drvdata(vdev->dev, NULL);
}
EXPORT_SYMBOL_GPL(vibration_dev_unregister);

static int __init vibration_init(void)
{
//	printk(KERN_ERR "vibration: %s\n", __func__);

	return create_vibration_class();
}

static void __exit vibration_exit(void)
{
	class_destroy(vibration_class);
}

module_init(vibration_init);
module_exit(vibration_exit);

MODULE_AUTHOR("Brox Chen <chun-ting.chen@quantatw.com>");
MODULE_DESCRIPTION("vibration class driver");
MODULE_LICENSE("GPL");
