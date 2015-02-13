#include <linux/kernel.h>
#include <linux/module.h>

#define DUMPMEMIOC		1
#define DUMPMEM_READ _IOWR(DUMPMEMIOC, 1, struct mem_access)
#define DUMPMEM_WRITE _IOW(DUMPMEMIOC, 2, struct mem_access)
#define DUMPMEM_READ_VIRTUAL _IOWR(DUMPMEMIOC, 3, struct mem_access)
#define DUMPMEM_WRITE_VIRTUAL _IOW(DUMPMEMIOC, 4, struct mem_access)

struct mem_access
{
	volatile u32 *addr;
	u32 val;
};

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
static long reg_access_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct mem_access mem_access;
	volatile u32 *addr;
	int ret = 0;
	
	if (copy_from_user(&mem_access, arg, sizeof(mem_access)))
	{
		ret = -EFAULT;
		goto end;
	}
	
	addr = ioremap((u32)mem_access.addr, 4);

	if(!addr)
	{
		printk("ioremap fail\n");
		ret = -EFAULT;
		goto end;
	}

	switch(cmd)
	{
	case DUMPMEM_READ:
		mem_access.val = *addr;
		if (copy_to_user(arg, &mem_access, sizeof(mem_access)))
			ret = -EFAULT;
		break;
	case DUMPMEM_WRITE:
		*addr = mem_access.val;
		break;
	case DUMPMEM_READ_VIRTUAL:
		mem_access.val = *mem_access.addr;
		if (copy_to_user(arg, &mem_access, sizeof(mem_access)))
			ret = -EFAULT;
		break;
	case DUMPMEM_WRITE_VIRTUAL:
		*mem_access.addr = mem_access.val;
		break;
	}

	iounmap(addr);	
end:
	return ret;	
}

static int reg_access_open(struct inode *inode, struct file *file)
{
	return 0;
}


static int reg_access_release(struct inode *ignored, struct file *file)
{
	return 0;
}


static const struct file_operations reg_access_fops = {
	.owner = THIS_MODULE,
	.open = reg_access_open,
	.release = reg_access_release,
	.unlocked_ioctl = reg_access_ioctl,
	.compat_ioctl = reg_access_ioctl,
};

static struct miscdevice reg_access_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "reg_access",
	.fops = &reg_access_fops,
};

static int __init reg_access_init(void)
{
	int ret;

	ret = misc_register(&reg_access_misc);

	if(ret)
		printk("reg_access register misc device fail ret = %d\n", ret);

	return ret;	
}

module_init(reg_access_init);