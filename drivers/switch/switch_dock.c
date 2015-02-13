/*
 * drivers/switch/switch_dock.c
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
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/switch_dock.h>
#include <linux/delay.h>

#define GPIO_HDMI_DEMUX                 99  /* HDMI Demux pin for SW control */
/*
 Gen2 Docking Design.
 It's compatible with all EPXX series.
  
 AUDIO SOURCE
  	+ SPL     --> Speaker
	+ HPL/HPR + --> Headphone
		  | ^(Docking GPIO_51)  
		  + --> AUD_Dock_L/R
		  		+ --> Speaker
				|      ^(SHDN#)
				+ --> Headphone
 */
struct gen2_dock_data *dock;

int dock_switch_reg(void)
{
	int ret;

	dock = kzalloc(sizeof(struct gen2_dock_data), GFP_KERNEL);
	dock->sdev.name = "dock";
	dock->debounce = 10;

	ret = switch_dev_register(&dock->sdev);
	if(ret < 0)
		goto err_switch_dev_register;
	return 0;

err_switch_dev_register:
	kfree(dock);

	return ret;
}

void dock_switch_work(int state)
{
	//printk(KERN_ERR "==== dock state: %d\n",state);

	if (dock == NULL)
		dock_switch_reg();

	msleep(dock->debounce);
	/* [Trek] HDMI DEMUX function implementation */
	if(state){
		switch_set_state(&dock->sdev, UNDOCKED);
		//gpio_direction_output(GPIO_HDMI_DEMUX, 1);
	}else{
		switch_set_state(&dock->sdev, LE_DESK_DOCK);
		//gpio_direction_output(GPIO_HDMI_DEMUX, 0);
	}
}

static int __init dock_switch_init(void)
{
	int ret=0;

	if (dock == NULL)
		ret=dock_switch_reg();

	return ret;
}

static void __exit dock_switch_exit(void)
{
	switch_dev_unregister(&dock->sdev);
	kfree(dock);
}

module_init(dock_switch_init);
module_exit(dock_switch_exit);

MODULE_AUTHOR("Trek Lee <Trek.Lee@quantatw.com>, Ryan Chang <Ryan.Chang@quantatw.com>");
MODULE_DESCRIPTION("DOCK Switch driver");
MODULE_LICENSE("GPL");
