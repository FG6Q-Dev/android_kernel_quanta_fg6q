/* drivers/input/touchscreen/fts.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h> /* regulator & voltage */
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/ft5x06_ts.h>
#include <linux/ft5x06_ex_fun.h>


/* FTS chip declare */
#define FT5606  //Sakia note: FT5816
//#define FT5x16
//#define FT5606
//#define FT6x06

/* kernel multi-touch report protocol define */
//#define TOUCH_REPORT_TYPE_A
#define TOUCH_REPORT_TYPE_B

//#define DEBUG_MSG
//#define VIRTUAL_KEY
#define FTS_EXTENSION
#define HW_PWR_OFF

#ifdef TOUCH_REPORT_TYPE_B
#include <linux/input/mt.h>
#endif

#ifdef DEBUG_MSG
#define fts_dbg(format, arg...) dev_err(&ts->client->dev, "%s: " format , __func__ , ## arg)
#else
#define fts_dbg(format, arg...) do {} while (0)
#endif

#define fts_msg(format, arg...) dev_err(&ts->client->dev, "%s: " format "\n" , \
	__func__ , ## arg)

#define FILL_I2C_MSG(msg, address, flag, length, buffer)	{\
	msg.addr = address;	\
	msg.flags = flag;	\
	msg.len = length;	\
	msg.buf = (u8 *)buffer;	\
}

/* Panel parameter define */
#ifdef CONFIG_PROJECT_PP3N
#define MAX_X	1366
#define MAX_Y	768
#endif

#ifdef CONFIG_PROJECT_PP3N_FHD
#define MAX_X	1920
#define MAX_Y	1080
#endif

#ifdef CONFIG_PROJECT_EP5N
#define MAX_X	2560
#define MAX_Y	1600
#endif

#ifdef CONFIG_PROJECT_EP5N
#define FTS_MAX_TOUCH	10
#else
#define FTS_MAX_TOUCH	5
#endif

#define FTS_MAX_TOUCH_PRESS	255

#define F_DOWN		0x00
#define F_UP			0x01
#define F_CONTACT	0x02

#define FT5x06_ID		0x55
#define FT5x16_ID		0x0A
#define FT5606_ID		0x08
#define FT6X06_ID		0x06

#ifdef FT5x06
#define FTS_DEV_ID		FT5x06_ID
#elif defined(FT5x16)
#define FTS_DEV_ID		FT5x16_ID
#elif defined(FT5606)
#define FTS_DEV_ID		FT5606_ID
#elif defined(FT6x06)
#define FTS_DEV_ID		FT6X06_ID
#else
#error Sorry, no define FTS chip
#endif

#define REG_DEV_MODE		0x00
#define REG_RAWDATA_ROW	0x01
#define REG_RAWDATA		0x10
#define REG_PWR_MODE		0xA5
#define REG_FW_VER			0xA6

typedef	enum{		
	P_ACTIVE,
	P_MONITOR,
	P_STANDBY,
	P_HIBERNATE,
}PWR_MODE;

#define FTS_MODE_NORMAL	0x00
#define FTS_MODE_SYS		0x10
#define FTS_MODE_TEST		0x40

#define DRIVER_VERSION		"v0.1"
#define DRIVER_DESC			"FTS TouchScreen driver"
#define DRIVER_NAME			"fts"
#define DRIVER_DEV_NAME	"fts_ts"

#define POINTER_TOUCH	BIT(0)
#define KEY_TOUCH		BIT(1)

struct fts_info{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct mutex lock;
};

struct fts_touch_info{
	u8 x_msb:4, rev:2, event:2;
	u8 x_lsb;
	u8 y_msb:4, id:4;
	u8 y_lsb;
	u8 weight;
	u8 speed:2, direction:2, area:4;
};

struct fts_packet_info{
	u8 gesture;
	u8 fingers:4, frame:4;
	struct fts_touch_info touch[FTS_MAX_TOUCH];
};

int ft5x06_touch_power(int on);
void ft5x06_gpio_reset(void);
static void fts_input_release(struct input_dev *in_dev);

static struct i2c_client *this_client;
static unsigned long start_jiffies;

void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
		for (j = 0; j < 1000; j++)
		{
			 udelay(1);
		}
	}
}

static int fts_readsb(struct fts_info *ts, u8 *addr, u8 *buf, u16 len)
{
	int ret;
	struct i2c_msg msg[2];

	if(addr){
		FILL_I2C_MSG(msg[0], ts->client->addr, 0, 1, addr);
		FILL_I2C_MSG(msg[1], ts->client->addr, I2C_M_RD, len, buf);
		ret = i2c_transfer(ts->client->adapter, msg, 2);
	}
	else{
		FILL_I2C_MSG(msg[0], ts->client->addr, I2C_M_RD, len, buf);
		ret = i2c_transfer(ts->client->adapter, &msg[0], 1);
	}

	if (ret < 0){
		fts_msg("i2c_transfer failed !\n");
		return ret;
	}

	return 0;
}

static int fts_writesb(struct fts_info *ts, u8 *buf, u16 len)
{
	int ret;
	struct i2c_msg msg;

	FILL_I2C_MSG(msg, ts->client->addr, 0, len, buf);

	ret = i2c_transfer(ts->client->adapter, &msg, 1);
	if (ret < 0){
		fts_msg("i2c_transfer failed !\n");
		return ret;
	}

	return 0;
}

static int fts_readb(struct fts_info *ts, u8 addr, u8 *value)
{
	u8 reg = addr;
	int ret;
	struct i2c_msg msg[2];

	FILL_I2C_MSG(msg[0], ts->client->addr, 0, 1, &reg);
	FILL_I2C_MSG(msg[1], ts->client->addr, I2C_M_RD, 1, value);

	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0){
		fts_msg("i2c_transfer failed !\n");
		return ret;
	}

	fts_dbg("Read %02X = 0x%02X\n", addr, *value);

	return 0;
}

static int fts_writeb(struct fts_info *ts, u8 addr, u8 value)
{
	struct i2c_msg msg;
	int ret;
	u8 buf[2];

	buf[0] = addr;
	buf[1] = value;

	FILL_I2C_MSG(msg, ts->client->addr, 0, 2, buf);

	ret = i2c_transfer(ts->client->adapter, &msg, 1);
	if (ret < 0){
		fts_msg("i2c_transfer failed !\n");
		return ret;
	}

	return 0;
}

/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata

Input	:	*rxdata
                     *length

Output	:	ret

function	:

***********************************************************************************************/
int ft5x0x_i2c_Read(char * writebuf, int writelen, char *readbuf, int readlen)
{
	int ret;

	if(writelen > 0)
	{
		struct i2c_msg msgs[] = {
			{
				.addr	= this_client->addr,
				.flags	= 0,
				.len	= writelen,
				.buf	= writebuf,
			},
			{
				.addr	= this_client->addr,
				.flags	= I2C_M_RD,
				.len	= readlen,
				.buf	= readbuf,
			},
		};
		ret = i2c_transfer(this_client->adapter, msgs, 2);
		if (ret < 0)
			DbgPrintk("msg %s i2c read error: %d\n", __func__, ret);
	}
	else
	{
		struct i2c_msg msgs[] = {
			{
				.addr	= this_client->addr,
				.flags	= I2C_M_RD,
				.len	= readlen,
				.buf	= readbuf,
			},
		};
		ret = i2c_transfer(this_client->adapter, msgs, 1);
		if (ret < 0)
			DbgPrintk("msg %s i2c read error: %d\n", __func__, ret);
	}
	return ret;
}EXPORT_SYMBOL(ft5x0x_i2c_Read);
/***********************************************************************************************
Name	:	 ft5x0x_i2c_Write

Input	:


Output	:0-write success
		other-error code
function	:	write data by i2c

***********************************************************************************************/
int ft5x0x_i2c_Write(char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= writelen,
			.buf	= writebuf,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		DbgPrintk("%s i2c write error: %d\n", __func__, ret);

	return ret;
}EXPORT_SYMBOL(ft5x0x_i2c_Write);

#if 0  //Sakia remove, not used
int ft5x0x_Download_i2c_Read(char * writebuf, int writelen, char *readbuf, int readlen)
{
	int ret;

	if(writelen > 0)
	{
		struct i2c_msg msgs[] = {
			{
				.addr	= I2C_Download_Addr,
				.flags	= 0,
				.len	= writelen,
				.buf	= writebuf,
			},
			{
				.addr	= I2C_Download_Addr,
				.flags	= I2C_M_RD,
				.len	= readlen,
				.buf	= readbuf,
			},
		};
		ret = i2c_transfer(this_client->adapter, msgs, 2);
		if (ret < 0)
			DbgPrintk("msg %s i2c read error: %d\n", __func__, ret);
	}
	else
	{
		struct i2c_msg msgs[] = {
			{
				.addr	= I2C_Download_Addr,
				.flags	= I2C_M_RD,
				.len	= readlen,
				.buf	= readbuf,
			},
		};
		ret = i2c_transfer(this_client->adapter, msgs, 1);
		if (ret < 0)
			DbgPrintk("msg %s i2c read error: %d\n", __func__, ret);
	}
	return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_i2c_Write

Input	:


Output	:0-write success
		other-error code
function	:	write data by i2c

***********************************************************************************************/
int ft5x0x_Download_i2c_Write(char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= I2C_Download_Addr,
			.flags	= 0,
			.len	= writelen,
			.buf	= writebuf,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		DbgPrintk("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
#endif

static int fts_gpio_reset(struct fts_info *ts)
{
	fts_enable = 0;

	/* Disable Interrupt */
	disable_irq(ts->client->irq);
	
#ifdef HW_PWR_OFF
	/* hardware reset */
	ft5x06_gpio_reset();
#endif

	/* Enable Interrutp */
	enable_irq(ts->client->irq);

	fts_enable = 1;

	return 0;
}

static int fts_config_gpio(void)
{



	return 0;
}

static int fts_fw_version(struct fts_info *ts, u8 *version)
{
	int ret;

	ret = fts_readb(ts, REG_FW_VER, version);
	
	if(ret){
		fts_msg("Can't read firmware version !\n");
		return -ENODEV;
	}

	return 0;
}

#ifdef FTS_EXTENSION
#include "fts_extension.c"
#endif

static int fts_verify_dev(struct fts_info *ts)
{
	u8 id;

	if(fts_readb(ts, 0xA3, &id))
		return -ENODEV;

#ifdef DEBUG_MSG
	switch (id){
		case FT5x06_ID:
			fts_dbg("FTS FT5x06 Chip\n");
			break;		
		case FT5x16_ID:
			fts_dbg("FTS FT5x16 Chip\n");
			break;		
		case FT5606_ID:
			fts_dbg("FTS FT5606 Chip\n");
			break;
	}
#endif

	fts_msg("FTS Chip ID = 0x%02X !\n", id);

	return 0;
}

static void fts_input_release(struct input_dev *in_dev)
{
	int i;

	for(i = 0; i < FTS_MAX_TOUCH; i++)
	{
		input_mt_slot(in_dev, i);
		input_mt_report_slot_state(in_dev, MT_TOOL_FINGER, false);
	}

	input_sync(in_dev);
}

#ifdef TOUCH_REPORT_TYPE_B
int current_finger = 0;
#endif
static void fts_input_report(struct fts_info *ts, struct fts_packet_info *buf)
{
	int i, j, x, y;
	int key, touch_type;
	struct fts_touch_info *touch;

	touch = &buf->touch[0];
	key = touch_type = 0;

	if(!start_jiffies || jiffies_to_msecs(jiffies - start_jiffies) < 1000)
		return;

	for(i = 0; (touch->id < 0x0F) && (i < FTS_MAX_TOUCH); i++, touch++){

		x = (u16)(touch->x_msb << 8) | (u16)touch->x_lsb;
#ifdef CONFIG_PROJECT_EP5N
		y = ( (u16)(touch->y_msb << 8) | (u16)touch->y_lsb);
#else
		y = MAX_Y-( (u16)(touch->y_msb << 8) | (u16)touch->y_lsb);
#endif

		fts_dbg("ID = %d, Event = %d, X = %d, Y = %d\n", touch->id, touch->event, x, y);

#ifdef TOUCH_REPORT_TYPE_A
		if ((touch->event == F_DOWN) || (touch->event == F_CONTACT)){
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, FTS_MAX_TOUCH_PRESS);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, FTS_MAX_TOUCH_PRESS);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
			input_mt_sync(ts->input_dev);
			touch_type |= POINTER_TOUCH;
		}
#else
		input_mt_slot(ts->input_dev, touch->id);

		if ((touch->event == F_DOWN) || (touch->event == F_CONTACT)){
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			//input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, FTS_MAX_TOUCH_PRESS);
			//input_report_abs(ts->input_dev, ABS_MT_PRESSURE, FTS_MAX_TOUCH_PRESS);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);

			current_finger |= BIT(touch->id);
		}
		else{
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);

			current_finger &= ~BIT(touch->id);
		}

		fts_dbg("ID %d Event %d, X = %d, Y = %d\n", touch->id, touch->event, x, y);

		touch_type |= POINTER_TOUCH;
#endif
	}

	if(touch_type){
#ifdef TOUCH_REPORT_TYPE_A
		if(touch_type & POINTER_TOUCH){
			input_report_abs(ts->input_dev, ABS_X, x);
			input_report_abs(ts->input_dev, ABS_Y, y);
			input_report_key(ts->input_dev, BTN_TOUCH, touch_type);
			input_report_key(ts->input_dev, BTN_TOOL_FINGER, touch_type);
		}
#else
		if(touch_type & POINTER_TOUCH)
			input_mt_report_pointer_emulation(ts->input_dev, false);
#endif

		input_sync(ts->input_dev);
	}
#ifdef TOUCH_REPORT_TYPE_A
	else{
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, 0);
		input_mt_sync(ts->input_dev);
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
		input_sync(ts->input_dev);
	}
#endif

#ifdef TOUCH_REPORT_TYPE_B
	if (buf->fingers == 0 && current_finger) {
		for(i = 0; i < FTS_MAX_TOUCH; i++){
			if(BIT(i) & current_finger){
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			}
		}
		current_finger = 0;
	}
#endif
}

static irqreturn_t fts_irq(int irq, void *dev_id)
{
	struct fts_info *ts = dev_id;
	struct fts_packet_info buf;
	u8 addr = 0x01;

	if(fts_enable) {
		if(fts_readsb(ts, &addr, (u8 *)&buf, sizeof(struct fts_packet_info)))
			goto exit;

		fts_input_report(ts, &buf);
	}

exit:
	return IRQ_HANDLED;
}

static void fts_suspend(struct fts_info *ts)
{
	int err;

	fts_dbg("called\n");

	fts_enable = 0;
	disable_irq(ts->client->irq);

	/* Touch Release */
	fts_input_release(ts->input_dev);

#ifdef HW_PWR_OFF
	/* Put reset pin in low and delay 2ms */
	/* hardware power off */
	ft5x06_touch_power(0);
#else
	if((err = fts_writeb(ts, REG_PWR_MODE, P_HIBERNATE)))
		fts_msg("touchpanel early suspend fail %d", err);
#endif
}

static void fts_resume(struct fts_info *ts)
{
	fts_dbg("called\n");

#ifdef HW_PWR_OFF
	/* hardware power on */
	ft5x06_touch_power(1);
#endif

	enable_irq(ts->client->irq);
	fts_enable = 1;

	/* Touch Release */
	fts_input_release(ts->input_dev);
	
	fts_gpio_reset(ts);
}

static int fts_init_dev(struct fts_info *ts)
{
	/*
	if(fts_config_gpio())
		return -ENODEV;

	fts_gpio_reset(ts);
	*/
	ft5x06_touch_power(1);

	return fts_verify_dev(ts);
}

static int fts_input_enable(struct input_dev *in_dev)
{
	struct i2c_client *client = to_i2c_client(in_dev->dev.parent);
	struct fts_info *ts = i2c_get_clientdata(client);
	int error = 0;

	start_jiffies = jiffies;

	fts_resume(ts);

	return error;
}

static int fts_input_disable(struct input_dev *in_dev)
{
	struct i2c_client *client = to_i2c_client(in_dev->dev.parent);
	struct fts_info *ts = i2c_get_clientdata(client);
	int error = 0;

	fts_suspend(ts);

	return error;
}

static int fts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct input_dev *input_dev;
	struct fts_info *ts;
	int retry = 0, err = -ENOMEM;

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		fts_msg("FTS: need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	ts = kzalloc (sizeof(struct fts_info), GFP_KERNEL);
	if (!ts)
		return err;

	/* register driver_data */
	ts->client = client;
	i2c_set_clientdata(client, ts);

	fts_dbg("%s install %02X\n", DRIVER_DESC, client->addr);

	if((err = fts_init_dev(ts)) < 0)
		goto out_free;

	input_dev = input_allocate_device();
	if (!input_dev)
		goto out_free;

	input_dev->name = DRIVER_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->enable = fts_input_enable;
	input_dev->disable= fts_input_disable;

	ts->input_dev = input_dev;

#ifdef TOUCH_REPORT_TYPE_B
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	/* These parameters are not necessary for multi-touch protocol */
	/*
	input_set_abs_params(input_dev, ABS_X, 0, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, FTS_MAX_TOUCH_PRESS, 0, 0);
	*/

	input_mt_init_slots(input_dev, FTS_MAX_TOUCH);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, FTS_MAX_TOUCH_PRESS, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, FTS_MAX_TOUCH_PRESS, 0, 0);
#else
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, FTS_MAX_TOUCH_PRESS, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, FTS_MAX_TOUCH_PRESS, 0, 0);
#endif

	input_set_drvdata(input_dev, ts);

	err = input_register_device(ts->input_dev);
	if (err) {
		fts_msg("input_register_device failed, err: %d", err);
		goto out_free_mem;
	}

	if(client->irq){
		fts_dbg("Using IRQ %d\n", client->irq);

		err = request_threaded_irq(client->irq, NULL, fts_irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->dev.driver->name, ts);

		if(err){
			fts_msg("request_irq failed");
			goto out_unregister_device;
		}
		disable_irq(client->irq);
	}
	else{
		fts_msg("No IRQ resource\n");
		goto out_unregister_device;
	}
	
	if(fts_init_sysfs(ts))
		goto out_unregister_device;

	fts_gpio_reset(ts);

	return 0;

out_releas_sysfs:
	fts_release_sysfs(ts);

out_unregister_device:
	input_unregister_device(input_dev);

out_free_mem:
	input_free_device(input_dev);

out_free:
	kfree(ts);

	return err;
}

static int fts_remove(struct i2c_client *client)
{
	struct fts_info *ts = i2c_get_clientdata(client);

	fts_dbg("called\n");

	free_irq(client->irq, ts);

	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);

	fts_release_sysfs(ts);

	kfree(ts);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int fts_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_info *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		fts_suspend(ts);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int fts_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_info *ts = i2c_get_clientdata(client);
	struct input_dev *input_dev = ts->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		fts_resume(ts);

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(fts_pm_ops, fts_i2c_suspend, fts_i2c_resume);

static const struct i2c_device_id i2c_id[] = {
	{ DRIVER_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, i2c_id);

static struct i2c_driver fts_i2c_driver = {
	.probe =		fts_probe,
	.remove =	fts_remove,
	.id_table =	i2c_id,
	.driver = {
		.name = DRIVER_NAME,
		.pm	= &fts_pm_ops,
	},
};

static int fts_init(void)
{
	return i2c_add_driver(&fts_i2c_driver);
}

static void fts_exit(void)
{
	i2c_del_driver(&fts_i2c_driver);
}

module_init(fts_init);
module_exit(fts_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

