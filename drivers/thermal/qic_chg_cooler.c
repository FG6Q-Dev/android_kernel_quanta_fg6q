/*
 *  qic_chg_cooler.c - qic charging cooler.
 *
 *  Copyright (C) 2013 Quanta
 *  Copyright (C) 2013 Derrick Liu <Derrick.Liu@quantatw.com>
 *
 *  Deisgn concept:
 *			1.Stop device charging when device overheat
 *			2.Default tipped temperature is 80C
 *			4.Bind "nct_ext" thermal zone
 *			5.Debug file node /d/qic_coolers/chg_apply or chg_cdev_state
 */
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/thermal.h>
#include <linux//i2c/bq24160_charger.h>
#include <linux/power_supply.h>
#include <linux/debugfs.h>
#include "qic_coolers.h"

#define DBG_CHGCDEV 0
#define MAX_TRIPS 3

static int chg_trips_table[MAX_TRIPS] = { 80, };
struct tegra_cooling_device {
        char *cdev_type;
        int *trip_temperatures;
        int trip_temperatures_num;
};

static int g_start_chg_cooling = 0;
static int g_chg_apply = 0;

static struct tegra_cooling_device qic_chg_cdev = {
	.cdev_type = "chg_cooler",
	.trip_temperatures_num = 1,
};

int get_g_chg_apply(void)
{
	return g_chg_apply;
}

int get_g_chg_cooling_status(void)
{
	return g_start_chg_cooling;
}
struct tegra_cooling_device *qic_get_chg_cdev(void)
{
	qic_chg_cdev.trip_temperatures = chg_trips_table;
	return &qic_chg_cdev;
}
int chg_cooler_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *max_state)
{
	*max_state = 1 ;
	return 0;
}
int chg_cooler_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *cur_state)
{
	*cur_state = g_start_chg_cooling;
	return 0;
}
int chg_cooler_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long cur_state)
{
	int ret = 0;


	if (!g_chg_apply)//If chg cooler is not applied, return directly.
		return 0;
	if(DBG_CHGCDEV) pr_err("[**]%s cur_state %ld\n", __func__, cur_state);
	if (cur_state == 1)
		g_start_chg_cooling = 1;
	else if((cur_state == 0))
		g_start_chg_cooling = 0;

	update_charger_bq24160_status();
	
	return ret;
}

static struct thermal_cooling_device_ops qic_chg_cooling_ops = {
	.get_max_state = chg_cooler_get_max_state,
	.get_cur_state = chg_cooler_get_cur_state,
	.set_cur_state = chg_cooler_set_cur_state,
};

static int chg_cdev_state_show(struct seq_file *s, void *data)
{
	seq_printf(s, "chg cooler state: %s\n",
		   g_start_chg_cooling ? "True" : "False");
	return 0;
}
static int chg_cdev_state_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, chg_cdev_state_show, inode->i_private);
}
static int chg_cooler_debugfs_state_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[3] = "Q";

	
	if (copy_from_user(buf, userbuf, count))
		pr_err("FAILED: copy string error!");

	if(DBG_CHGCDEV) pr_err("[**]%s %s\n", __func__, (char*)buf);

	if (!g_chg_apply)//If chg cooler is not applied, return directly.
	{
		pr_err("Please apply chg cooler first \n (echo Y > d/qic_coolers/chg_apply)\n");
		return -EINVAL;
	}

	if (!strncmp(buf, "Y", 1))
		g_start_chg_cooling = 1;
	else if (!strncmp(buf, "N", 1))
		g_start_chg_cooling = 0;
	else
		pr_err("FAILED: wrong parameter %s ", (char*)buf);

	update_charger_bq24160_status();

	if(DBG_CHGCDEV) pr_err("[**]%s g_start_chg_cooling %d\n", __func__, g_start_chg_cooling);
	return count;

}
static const struct file_operations cooler_state_debugfs_fops = {
	.open		= chg_cdev_state_debugfs_open,
	.read		= seq_read,
	.write		= chg_cooler_debugfs_state_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int chg_apply_show(struct seq_file *s, void *data)
{
	seq_printf(s, "chg cooler apply: %s\n",
		   g_chg_apply ? "On" : "Off");
	return 0;
}
static int chg_apply_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, chg_apply_show, inode->i_private);
}
static int chg_apply_debugfs_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[3] = "Q";

	if (copy_from_user(buf, userbuf, count))
		pr_err("FAILED: copy string error!");

	if(DBG_CHGCDEV) pr_err("[**]%s %s\n", __func__, (char*)buf);

	if (!strncmp(buf, "Y", 1))
		g_chg_apply = 1;
	else if (!strncmp(buf, "N", 1))
		g_chg_apply = 0;
	else
		pr_err("FAILED: wrong parameter %s ", (char*)buf);

	g_start_chg_cooling = 0;//When turn on/off chg cooler, make sure g_start_chg_cooling is false.
	update_charger_bq24160_status();

	if(DBG_CHGCDEV) pr_err("[**]%s chg cooler apply (%d) state (%d)\n", __func__, g_chg_apply, g_start_chg_cooling);
	return count;

}
static const struct file_operations apply_debugfs_fops = {
	.open		= chg_apply_debugfs_open,
	.read		= seq_read,
	.write		= chg_apply_debugfs_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init chg_cooling_init(void)
{
	struct dentry *qic_coolers_dir, *d_chg_cdev_state, *d_chg_apply;

	if(DBG_CHGCDEV) pr_err("[**]%s IN\n", __func__);

	thermal_cooling_device_register(
				"chg_cooler",
				NULL,
				&qic_chg_cooling_ops);
			
	qic_coolers_dir = get_g_qic_coolers_dir();
	if (!qic_coolers_dir)
	{
		pr_err("[**]No qic_coolers_dir\n");
		goto qic_coolers_dir_err;
	}

	d_chg_cdev_state = debugfs_create_file("chg_cdev_state", S_IRUGO | S_IWUSR, qic_coolers_dir, NULL,
				&cooler_state_debugfs_fops);
	if (!d_chg_cdev_state)
		goto _err;

	//chg cooler apply
	d_chg_apply = debugfs_create_file("chg_apply", S_IRUGO | S_IWUSR, qic_coolers_dir, NULL,
				&apply_debugfs_fops);
	if (!d_chg_apply)
		goto _err;

_err:
	debugfs_remove(qic_coolers_dir);
qic_coolers_dir_err:
	return -ENOMEM;

	return 0;
}
module_init(chg_cooling_init);
/*Derrick.Liu add new coolers chg_cool*/