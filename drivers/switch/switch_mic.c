/*
 * drivers/switch/switch_mic.c
 *
 * Copyright (C) 2011 Quanta Computer, Inc.
 * Author: Trek Lee <Trek.Lee@quantatw.com>
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
#include <linux/kernel.h>
#include <linux/init.h>
//#include <linux/interrupt.h>
#include <linux/slab.h>
//#include <linux/workqueue.h>
#include <linux/gpio.h>
//#include <linux/switch.h>
#include <linux/switch_mic.h>
#include <linux/platform_device.h>
//#include <linux/device.h>

struct mic_switch_data {
	struct mic_switch_platform_data		*pdata; 
	struct device					*dev; 
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
};

//struct switch_dev *mic;
int mic_layout=1;

/*
EP5N design  
layout|DMIC_SW1| DMIC_SW2|   Left    |   Right   |
      |        |         |(DMIC_LR=L)|(DMIC_LR=H)|
    1 |    0   |    0	 |   DMIC1   |	  DMIC2  |
    2 |    0   |    1	 |   DMIC1   |	  DMIC3  |
    3 |    1   |    0	 |   DMIC2   |	         | 
    4 |    1   |    1	 |   DMIC2   |	  DMIC3  | default setting

	     DMIC3 
 ___________________
/__________________/| 
|	    DMIC2      ||
|		           ||
|		           ||
|		           ||
|		           ||
|      DMIC1       |/
-------------------

    
*/

static ssize_t mic_switch_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t size)
{
	int layout;
	struct mic_switch_platform_data *pdata = dev->platform_data;
	if (!sscanf(buf, "%d\n", &layout))
		return -EINVAL;
	
	printk(KERN_INFO, "==== mic state: %d\n",layout);

	if(layout>0){		
		switch(layout)
		{
			case DOWN_TOP:
				gpio_direction_output(pdata->dmic_sw1, 0);
				gpio_direction_output(pdata->dmic_sw2, 0);
				gpio_direction_output(pdata->dmic_lr,  1);			
				break;
			case DOWN_BACK:
				gpio_direction_output(pdata->dmic_sw1, 0);
				gpio_direction_output(pdata->dmic_sw2, 1);
				gpio_direction_output(pdata->dmic_lr,  0);;//DMIC2 don't care L or R, so setting to L channel 			
				break;
			case TOP_ONLY:
				gpio_direction_output(pdata->dmic_sw1, 1);
				gpio_direction_output(pdata->dmic_sw2, 0);
				gpio_direction_output(pdata->dmic_lr,  0); 
				break;
			case TOP_BACK:
				gpio_direction_output(pdata->dmic_sw1, 1);
				gpio_direction_output(pdata->dmic_sw2, 1);
				gpio_direction_output(pdata->dmic_lr,  0);		
				break;
		}
	}
	mic_layout = layout;
	return size;
}
static ssize_t mic_switch_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "%u\n", mic_layout); 
}

static DEVICE_ATTR(mic_status, 0664, mic_switch_show, mic_switch_store); 

static int mic_switch_probe(struct platform_device *pdev)
{
	struct mic_switch_platform_data *pdata = pdev->dev.platform_data;
	struct mic_switch_data *switch_data;

	int ret=0;
	
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev,"mic_switch_probe\n");
	switch_data = kzalloc(sizeof(struct mic_switch_data), GFP_KERNEL);
	if ( NULL == switch_data) { 
			dev_err(&pdev->dev, "Can't allocate mic_switch_data struct\n");
			ret=-ENOMEM; 
			goto err_kzalloc; 
	} 
	
	ret = device_create_file(&pdev->dev, &dev_attr_mic_status);
	if(ret < 0)
	{
		dev_err(&pdev->dev,"driver_create_file fail(%d)\n",ret);
		goto err_kzalloc;
	}
	//default is TOP and BACK DMIC
	if(pdata->dmic_sw1!=-1){
		ret = gpio_request(pdata->dmic_sw1, "MIC_SW1");	 
		if (ret < 0) { 
				dev_err(&pdev->dev, "error requesting MIC_SW1 gpio\n"); 
				goto err_gpio; 
		} 
		gpio_direction_output(pdata->dmic_sw1, 1);
	}
	if(pdata->dmic_sw2!=-1){
		ret = gpio_request(pdata->dmic_sw2, "MIC_SW2");	 
		if (ret < 0) { 
				dev_err(&pdev->dev, "error requesting MIC_SW2 gpio\n"); 
				goto err_gpio; 
		} 
		gpio_direction_output(pdata->dmic_sw2, 1);
	}
	if(pdata->dmic_lr!=-1){
		ret = gpio_request(pdata->dmic_lr,  "MIC_LR");	 
		if (ret < 0) { 
				dev_err(&pdev->dev, "error requesting MIC_LR gpio\n"); 
				goto err_gpio; 
		} 
		gpio_direction_output(pdata->dmic_lr,  0);			
	}
	mic_layout=4;
	return 0;
	
err_gpio: 
	kfree(pdata); 
err_kzalloc:	
	return ret;
}
static int __devexit mic_switch_remove(struct platform_device *pdev)
{
	struct mic_switch_data *switch_data = platform_get_drvdata(pdev);

	if(switch_data->pdata->dmic_sw1!=-1) 
		gpio_free(switch_data->pdata->dmic_sw1);

	if(switch_data->pdata->dmic_sw2!=-1)
		gpio_free(switch_data->pdata->dmic_sw2);

	if(switch_data->pdata->dmic_lr!=-1)
		gpio_free(switch_data->pdata->dmic_lr);

	kfree(switch_data);

	return 0;
}

static struct platform_driver mic_switch_driver = {
	.probe		= mic_switch_probe,
	.remove		= __devexit_p(mic_switch_remove),
	.driver		= {
		.name	= "switch-mic",
		.owner	= THIS_MODULE,
	},
};

static int __init mic_switch_init(void)
{
	return platform_driver_register(&mic_switch_driver);
}

static void __exit mic_switch_exit(void)
{
	platform_driver_unregister(&mic_switch_driver);
}
module_init(mic_switch_init);
module_exit(mic_switch_exit);

MODULE_AUTHOR("Ruby Hsu <ruby-hsu@quantatw.com>");
MODULE_DESCRIPTION("Microphone Switch driver");
MODULE_LICENSE("GPL");
