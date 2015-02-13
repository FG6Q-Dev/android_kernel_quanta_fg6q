/* Quanta Projects Version Control
 *
 * Copyright (C) 2011 Quanta Computer Inc.
 * Author: Wayne Lin <wayne.lin@quantatw.com>
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

/*-----------------------------------------------------------------------------
 * Global Include files
 *---------------------------------------------------------------------------*/
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <linux/vermagic.h>
#include <linux/string.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include "hw_version.h"

//#define MAX_USB_SERIAL_NUM		17
//#ifdef CONFIG_ARCH_OMAP4
//#define DIE_ID_REG_BASE         (L4_44XX_PHYS + 0x2000)
//#define DIE_ID_REG_OFFSET               0x200
//#else
//#define DIE_ID_REG_BASE         (L4_WK_34XX_PHYS + 0xA000)
//#define DIE_ID_REG_OFFSET               0x218
//#endif /* CONFIG_ARCH_OMAP4 */

/*-----------------------------------------------------------------------------
 * Local Include files
 *---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 * Constants
 *---------------------------------------------------------------------------*/
#define KERNEL_VERSION UTS_RELEASE
#define PROJECTS_PROC_NAME	"board_id"
#define RAMCODES_PROC_NAME	"ramcode_id"

/*-----------------------------------------------------------------------------
 * Marcos
 *---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 * Global variables
 *---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 * Local Functions
 *---------------------------------------------------------------------------*/

static int version_proc_show(struct seq_file *m, void *v)
{
	int	ret = -1;
	char	*hwver_ptr = NULL;
	ret = qci_mainboard_version();
//	unsigned int val[4] = { 0 };
//	unsigned int reg;
//	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;
//	char device_serial[MAX_USB_SERIAL_NUM];
//	u8 *type;
//	val[0] = omap_readl(reg);
//	val[1] = omap_readl(reg + 0x8);
//	val[2] = omap_readl(reg + 0xC);
//	val[3] = omap_readl(reg + 0x10);
//	snprintf(device_serial, MAX_USB_SERIAL_NUM, "%08X%08X%08X%08X",
//					val[3], val[2], val[1], val[0]);

	if ( ret == HW_REV_A )
		hwver_ptr = "00";
	else if ( ret == HW_REV_B )
		hwver_ptr = "01";
	else if ( ret == HW_REV_C )
		hwver_ptr = "10";
	else if ( ret == HW_REV_D )
		hwver_ptr = "11";
	else
		hwver_ptr = "Reserve";

	//seq_printf(m, "Kernel Version	: %s\n", KERNEL_VERSION);
	//seq_printf(m, "Device ID 	: %s\n", device_serial);
	seq_printf(m, "%s\n", hwver_ptr);
	//seq_printf(m, "CPU Information	: OMAP%04x type(%s)\n",omap_rev() >> 16, type);
	return 0;
}

static int ramcodes_proc_show(struct seq_file *m, void *v)
{

/*
 * Elpida   : R241(del),R367(del),R221(add),R152(add)   ID :00 (RAM_CODE1,RAM_CODE0)
 * Micron  	: R241(add),R367(del),R221(del),R152(add)   ID :01 (RAM_CODE1,RAM_CODE0)
 * Samsung 	: R241(del),R367(add),R221(add),R152(del)   ID :10 (RAM_CODE1,RAM_CODE0)
 * Hynix 	: R241(add),R367(add),R221(del),R152(del)   ID :11 (RAM_CODE1,RAM_CODE0)
*/
	int	ret = -1;
	char	*hwver_ptr = NULL;
	ret = qci_ramcodes_version();
	switch (ret){
		case HW_RAMCODES_ELPIDA:
			hwver_ptr = "00";
			break;
		case HW_RAMCODES_MICRON:
			hwver_ptr = "01";
			break;
		case HW_RAMCODES_SAMSUNG:
			hwver_ptr = "10";
			break;
		case HW_RAMCODES_HYNIX:
			hwver_ptr = "11";
			break;
		default:
			hwver_ptr = "N/A";
	}
	seq_printf(m, "%s\n", hwver_ptr);
	return 0;
}


static int version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, version_proc_show, NULL);
}

static int ramcodes_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ramcodes_proc_show, NULL);
}


static const struct file_operations version_proc_fops = {
	.open		= version_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations ramcodes_proc_fops = {
	.open		= ramcodes_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int __init proc_version_init(void)
{
	proc_create(PROJECTS_PROC_NAME, 0, NULL, &version_proc_fops);
	proc_create(RAMCODES_PROC_NAME, 0, NULL, &ramcodes_proc_fops);

	return 0;
}

static void __exit proc_version_exit(void)
{
	remove_proc_entry(PROJECTS_PROC_NAME, NULL);
	remove_proc_entry(RAMCODES_PROC_NAME, NULL);
}

module_init(proc_version_init);
module_exit(proc_version_exit);

//MODULE_AUTHOR("Quanta Computer Inc.");
//MODULE_DESCRIPTION("Quanta Projects Version Control");
//MODULE_VERSION("0.1.2");
//MODULE_LICENSE("GPL v2");

