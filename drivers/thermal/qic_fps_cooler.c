/*
 *  qic_fps_cooler.c - qic frame rate cooler.
 *
 *  Copyright (C) 2013 Quanta
 *  Copyright (C) 2013 Derrick Liu <Derrick.Liu@quantatw.com>
 *
 *  Deisgn concept:
 *			1.Suppress frame rate of hw composer to 39fps when device overheat
 *			2.Currently, only 39fps and 59fps are supported on Tegra4
 *			3.Default tipped temperature is 75C
 *			4.Bind "nct_ext" thermal zone
 *			5.Debug file node /d/qic_coolers/fps_apply or fps_cdev_state
 */
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/thermal.h>
#include <linux/fb.h>
#include <linux/debugfs.h>
#include "qic_coolers.h"

#define DBG_FPSCDEV 0
#define MAX_TRIPS 3

static int fps_trips_table[MAX_TRIPS] = { 75, };

struct tegra_cooling_device {
        char *cdev_type;
        int *trip_temperatures;
        int trip_temperatures_num;
};

static int g_start_fps_cooling = 0;
static int g_fps_apply = 0;
struct dentry *g_qic_coolers_dir;

static struct tegra_cooling_device qic_fps_cdev = {
	.cdev_type = "fps_cooler",
	.trip_temperatures_num = 1,
};

int get_g_fps_apply(void)
{
	return g_fps_apply;
}

int get_g_fps_cooling_status(void)
{
	return g_start_fps_cooling;
}
struct dentry *get_g_qic_coolers_dir(void)
{
	return g_qic_coolers_dir;
}
struct tegra_cooling_device *qic_get_fps_cdev(void)
{
	qic_fps_cdev.trip_temperatures = fps_trips_table;
	return &qic_fps_cdev;
}
int fps_cooler_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *max_state)
{
	*max_state = 1 ;
	return 0;
}
int fps_cooler_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *cur_state)
{
	*cur_state = g_start_fps_cooling;
	return 0;
}

int fps_cooler_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long cur_state)
{
	int ret = 0;

	if (!g_fps_apply)//If fps cooler is not applied, return directly.
		return 0;

	if(DBG_FPSCDEV) pr_err("[**]%s %ld\n", __func__, cur_state);
	if (cur_state == 1)
	{
		/*Set FPS to 39*/
		ret = set_frame_rate(39);
		if(ret) pr_err("[**]Set frame rate 39 failed\n");
		/*Set flag to true, to prevent high 59 FPS request*/
		g_start_fps_cooling = 1;
	}else if((cur_state == 0)){
		/*Set flag to false*/
		g_start_fps_cooling = 0;

	}
	return ret;
}

static struct thermal_cooling_device_ops qic_fps_cooling_ops = {
	.get_max_state = fps_cooler_get_max_state,
	.get_cur_state = fps_cooler_get_cur_state,
	.set_cur_state = fps_cooler_set_cur_state,
};

static int fps_cdev_state_show(struct seq_file *s, void *data)
{
	seq_printf(s, "fps cooler state: %s\n",
		   g_start_fps_cooling ? "True" : "False");
	return 0;
}
static int fps_cdev_state_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, fps_cdev_state_show, inode->i_private);
}
static int fps_cooler_debugfs_state_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[3] = "Q";

	if (copy_from_user(buf, userbuf, count))
		pr_err("FAILED: copy string error!");

	if(DBG_FPSCDEV) pr_err("[**]%s %s\n", __func__, (char*)buf);

	if (!g_fps_apply)//If fps cooler is not applied, return directly.
	{
		pr_err("Please apply fps cooler first \n (echo Y > d/qic_coolers/fps_apply)\n");
		return -EINVAL;
	}

	if (!strncmp(buf, "Y", 1))
		g_start_fps_cooling = 1;
	else if (!strncmp(buf, "N", 1))
		g_start_fps_cooling = 0;
	else
		pr_err("FAILED: wrong parameter %s ", (char*)buf);

	if (g_start_fps_cooling)
		set_frame_rate(39);

	if(DBG_FPSCDEV) pr_err("[**]%s g_start_fps_cooling %d\n", __func__, g_start_fps_cooling);
	return count;

}
static const struct file_operations cooler_state_debugfs_fops = {
	.open		= fps_cdev_state_debugfs_open,
	.read		= seq_read,
	.write		= fps_cooler_debugfs_state_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int fps_apply_show(struct seq_file *s, void *data)
{
	seq_printf(s, "fps cooler apply: %s\n",
		   g_fps_apply ? "On" : "Off");
	return 0;
}
static int fps_apply_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, fps_apply_show, inode->i_private);
}
static int fps_apply_debugfs_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[3] = "Q";

	if (copy_from_user(buf, userbuf, count))
		pr_err("FAILED: copy string error!");

	if(DBG_FPSCDEV) pr_err("[**]%s %s\n", __func__, (char*)buf);

	if (!strncmp(buf, "Y", 1))
		g_fps_apply = 1;
	else if (!strncmp(buf, "N", 1))
		g_fps_apply = 0;
	else
		pr_err("FAILED: wrong parameter %s ", (char*)buf);

		g_start_fps_cooling = 0;//When turn on/off fps cooler, make sure g_start_fps_cooling is false.

	if(DBG_FPSCDEV) pr_err("[**]%s fps cooler apply (%d) state (%d)\n", __func__, g_fps_apply, g_start_fps_cooling);
	return count;

}
static const struct file_operations apply_debugfs_fops = {
	.open		= fps_apply_debugfs_open,
	.read		= seq_read,
	.write		= fps_apply_debugfs_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init fps_cooling_init(void)
{
	struct dentry *qic_coolers_dir, *d_fps_cdev_state, *d_fps_apply;

	if(DBG_FPSCDEV) pr_err("[**]%s IN\n", __func__);

	//regist fps_cooler
	thermal_cooling_device_register(
				"fps_cooler",
				NULL,
				&qic_fps_cooling_ops);

	qic_coolers_dir = debugfs_create_dir("qic_coolers", NULL);/*/d/qic_coolers/*/
	if (!qic_coolers_dir)
		goto qic_coolers_dir_err;

	g_qic_coolers_dir = qic_coolers_dir;

	//fps cooler state flag
	d_fps_cdev_state = debugfs_create_file("fps_cdev_state", S_IRUGO | S_IWUSR, qic_coolers_dir, NULL,
				&cooler_state_debugfs_fops);
	if (!d_fps_cdev_state)
		goto _err;

	//fps cooler apply
	d_fps_apply = debugfs_create_file("fps_apply", S_IRUGO | S_IWUSR, qic_coolers_dir, NULL,
				&apply_debugfs_fops);
	if (!d_fps_apply)
		goto _err;

_err:
	debugfs_remove(qic_coolers_dir);
qic_coolers_dir_err:
	return -ENOMEM;

	return 0;
}
module_init(fps_cooling_init);
/*Derrick.Liu add new coolers fps_cooler*/