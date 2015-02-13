#include <linux/module.h>

#include <linux/init.h> 
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <mach/gpio-tegra.h>
#include "qic_modem.h"

#define DRIVER_NAME    "qic_modem"

#define QIC_MODEM_POWER_DOWN	0
#define QIC_MODEM_POWER_UP		1

/* 	Option GTM601 low active.
	LM52x A2 hardware is low active.
	A3 is high active.
	Ericsson is high active but with clock 32K rework.
	Huawei ME906X is high active.
	*/
#define QIC_MODEM_HIGH_ACTIVE
//#define QIC_MODEM_LM52X
#define QIC_MODEM_LM52X_POWER_ON_TIME 350
#define QIC_MODEM_LM52X_POWER_OFF_TIME 350
#define QIC_MODEM_ME906_POWER_ON_TIME 350


//default vin, on_key is also set as low on modem side as default value.
static int qic_modem_power_status = QIC_MODEM_POWER_UP;
static int qic_modem_gpio_onkey = 0;
static int qic_modem_gpio_reset = 0;

#ifdef QIC_MODEM_SIM_DETECTION_ENHANCEMENT
static int sim_detection_status = -1;

static ssize_t sim_detection_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sim_detection_status);
}

#ifdef QIC_MODEM_RW_SIM_DETECTION
static ssize_t sim_detection_status_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int input_value = 0;	
	sscanf(buf, "%d", &input_value);
	(input_value>0)? 1:0;
	sim_detection_status = input_value;
	return sprintf(buf, "sim_detection_status = %d\n", sim_detection_status);
}
#endif

#ifdef QIC_MODEM_RW_SIM_DETECTION
static DEVICE_ATTR(sim_detection_status, 0664, sim_detection_status_show, sim_detection_status_store);
#else
static DEVICE_ATTR(sim_detection_status, 0664, sim_detection_status_show, NULL);
#endif

static struct attribute *qic_modem_attributes[] = {
	&dev_attr_sim_detection_status.attr,
	NULL
};

static const struct attribute_group qic_modem_attr_group = {
	.attrs = qic_modem_attributes,
};

static void qic_modem_sim_detection_check_status(void *data)
{
    struct qic_modem_platform_data *pdata = (struct qic_modem_platform_data *)data;
	int val = gpio_get_value(pdata->gpio_sim_detection);
	
    if ( 0 == val)
    {
    	/* sim removed */
		sim_detection_status = 0;
		printk("%s:, modem sim detection triggerred. gpio value:%d\n",__func__, val);     
    }
	else
	{
		/* sim inserted */
		sim_detection_status = 1;
		printk("%s:, modem sim detection triggerred. gpio value:%d\n",__func__, val);     
	}
};
#endif

void qic_modem_power_down_by_onkey()
{
#ifdef QIC_MODEM_HIGH_ACTIVE
	pr_info("%s: original modem status is %s.\n",__func__, ((QIC_MODEM_POWER_UP == qic_modem_power_status) ? "on": "off") );     
	if ((QIC_MODEM_POWER_UP == qic_modem_power_status) && (qic_modem_gpio_onkey))
	{
#ifdef QIC_MODEM_LM52X
		gpio_direction_output(qic_modem_gpio_onkey, 1);
		msleep(QIC_MODEM_LM52X_POWER_OFF_TIME);
		gpio_direction_output(qic_modem_gpio_onkey, 0);
#else
		gpio_direction_output(qic_modem_gpio_onkey, 0); // Ericsson C5621 at least for 100 ns
#endif
		qic_modem_power_status = QIC_MODEM_POWER_DOWN;
	}
#else
	pr_info("%s: original modem status is %s.\n",__func__, ((QIC_MODEM_POWER_UP == qic_modem_power_status) ? "on": "off") );     
	if ((QIC_MODEM_POWER_UP == qic_modem_power_status) && (qic_modem_gpio_onkey))
	{
		gpio_request(qic_modem_gpio_onkey, "mdm_onkey");
		gpio_direction_output(qic_modem_gpio_onkey, 1);
		mdelay(5);
		gpio_direction_output(qic_modem_gpio_onkey, 0);
		mdelay(300);
		gpio_direction_output(qic_modem_gpio_onkey, 1);
		qic_modem_power_status = QIC_MODEM_POWER_DOWN;
	}
#endif
}

static int qic_modem_notifier_call(struct notifier_block *this,
					unsigned long code, void *cmd)
{
	if ((code == SYS_DOWN)||(code == SYS_HALT)||(code == SYS_POWER_OFF)) {
		qic_modem_power_down_by_onkey();
	}

	return NOTIFY_DONE;
}

static struct notifier_block qic_modem_tablet_reboot_notifier = {
	.notifier_call = qic_modem_notifier_call,
};

static irqreturn_t qic_modem_fatal_isr(int irq, void *data)
{
    struct qic_modem_platform_data *pdata = (struct qic_modem_platform_data *)data;
	int val = gpio_get_value(pdata->gpio_fatal);
	
    if ( 1 == val)
    {
		printk("%s:, modem fatal error triggerred. gpio value:%d, irq num:%d\n",__func__, val, irq);     
    }   

    return IRQ_HANDLED;
};

static irqreturn_t qic_modem_wakeup_host_isr(int irq, void *data)
{
    struct qic_modem_platform_data *pdata = (struct qic_modem_platform_data *)data;
	int val = gpio_get_value(pdata->gpio_modem_wakeup_host);
	
    if ( 0 == val)
    {
		printk("%s:, modem wake up host triggerred. gpio value:%d, irq num:%d\n",__func__, val, irq);     
    }   

    return IRQ_HANDLED;
};

static irqreturn_t qic_modem_sim_detection_isr(int irq, void *data)
{
#ifdef QIC_MODEM_SIM_DETECTION_ENHANCEMENT
	qic_modem_sim_detection_check_status(data);
#endif
    return IRQ_HANDLED;
};

static int __devinit qic_modem_probe(struct platform_device *pdev)
{
	struct qic_modem_platform_data *pdata;
	int irq;

	pr_info("%s.\n", __func__);
	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	register_reboot_notifier(&qic_modem_tablet_reboot_notifier);

	if (QIC_MODEM_GPIO_NULL != pdata->gpio_power_enable)
	{
		tegra_gpio_enable(pdata->gpio_power_enable);
		gpio_request(pdata->gpio_power_enable, "mdm_pwr");
		gpio_direction_output(pdata->gpio_power_enable, 1);
	}

	if (QIC_MODEM_GPIO_NULL != pdata->gpio_host_wakeup_wwan)
	{
		tegra_gpio_enable(pdata->gpio_host_wakeup_wwan);
		gpio_request(pdata->gpio_host_wakeup_wwan, "mdm_wake_wwan");
		gpio_direction_output(pdata->gpio_host_wakeup_wwan, 1);
	}

#if 0
	/* Set redundant radio off to floating */
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_radio_off_dummy)
	{
		tegra_gpio_enable(pdata->gpio_radio_off);
		gpio_request(pdata->gpio_radio_off, "mdm_radio_off_dummy");
		gpio_direction_input(pdata->gpio_radio_off);
	}
#endif
	
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_radio_off)
	{
		tegra_gpio_enable(pdata->gpio_radio_off);
		gpio_request(pdata->gpio_radio_off, "mdm_radio_off");
		gpio_direction_output(pdata->gpio_radio_off, 1);
	}

	if (QIC_MODEM_GPIO_NULL != pdata->gpio_reset)
	{
		tegra_gpio_enable(pdata->gpio_reset);
		gpio_request(pdata->gpio_reset, "mdm_reset");
		gpio_direction_output(pdata->gpio_reset, 1);
		qic_modem_gpio_reset = pdata->gpio_reset;
	}

	if (QIC_MODEM_GPIO_NULL != pdata->gpio_fatal)
	{
		tegra_gpio_enable(pdata->gpio_fatal);
		irq = gpio_to_irq(pdata->gpio_fatal);
		request_irq(irq, qic_modem_fatal_isr, IRQF_TRIGGER_HIGH, DRIVER_NAME, pdata);	
	}

#if 0 /* Disabled to avoid endless wakeup irq. */
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_modem_wakeup_host)
	{
		tegra_gpio_enable(pdata->gpio_modem_wakeup_host);
		irq = gpio_to_irq(pdata->gpio_modem_wakeup_host);
		request_irq(irq, qic_modem_wakeup_host_isr, IRQF_TRIGGER_LOW, DRIVER_NAME, pdata);	
	}
#endif

#if 0
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_sim_detection)
	{
		tegra_gpio_enable(pdata->gpio_sim_detection);
		irq = gpio_to_irq(pdata->gpio_sim_detection);
		gpio_request(pdata->gpio_sim_detection, "mdm_sim_det");
		request_irq(irq, qic_modem_sim_detection_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, DRIVER_NAME, pdata);
	}
#else /* Huawei SIM detection pin needs pull up. */
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_sim_detection)
	{
		tegra_gpio_enable(pdata->gpio_sim_detection);
		gpio_request(pdata->gpio_sim_detection, "mdm_sim_det");
		//gpio_direction_output(pdata->gpio_sim_detection, 1);
		gpio_direction_input(pdata->gpio_sim_detection);
	}
#endif

#ifdef QIC_MODEM_SIM_DETECTION_ENHANCEMENT
	sysfs_create_group(&pdev->dev.kobj, &qic_modem_attr_group);
#endif

#ifdef QIC_MODEM_HIGH_ACTIVE
#ifdef QIC_MODEM_LM52X
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_onkey)
	{
		tegra_gpio_enable(pdata->gpio_onkey);
		gpio_request(pdata->gpio_onkey, "mdm_onkey");
		gpio_direction_output(pdata->gpio_onkey, 1); //Option GTM601 low active ; LM52x A2 hardware is low active; A3 is high active
			msleep(QIC_MODEM_LM52X_POWER_ON_TIME);
		gpio_direction_output(pdata->gpio_onkey, 0); //Option GTM601 low active ; LM52x A2 hardware is low active; A3 is high active
		qic_modem_power_status = QIC_MODEM_POWER_UP;
		qic_modem_gpio_onkey = pdata->gpio_onkey;
	}
#else
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_onkey)
	{
		tegra_gpio_enable(pdata->gpio_onkey);
		gpio_request(pdata->gpio_onkey, "mdm_onkey");
		gpio_direction_output(pdata->gpio_onkey, 0); //Option GTM601 low active ; LM52x A2 hardware is low active; A3 is high active
			msleep(QIC_MODEM_ME906_POWER_ON_TIME);
		gpio_direction_output(pdata->gpio_onkey, 1); //Option GTM601 low active ; LM52x A2 hardware is low active; A3 is high active
		qic_modem_power_status = QIC_MODEM_POWER_UP;
		qic_modem_gpio_onkey = pdata->gpio_onkey;
	}
#endif
#else
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_onkey)
	{
		tegra_gpio_enable(pdata->gpio_onkey);
		gpio_request(pdata->gpio_onkey, "mdm_onkey");
		gpio_direction_output(pdata->gpio_onkey, 0); /* Option GTM601 low active ; LM52x A2 hardware is low active; A3 is high active */
		qic_modem_power_status = QIC_MODEM_POWER_UP;
		qic_modem_gpio_onkey = pdata->gpio_onkey;
	}
#endif

//	pr_info("%s: onkey=%d, radio_off=%d, reset=%d, wake_wwan=%d", __func__, gpio_get_value(pdata->gpio_onkey), gpio_get_value(pdata->gpio_radio_off), gpio_get_value(pdata->gpio_reset), gpio_get_value(pdata->gpio_host_wakeup_wwan));
	pr_info("%s: onkey=%d, radio_off=%d, reset=%d\n", __func__, gpio_get_value(pdata->gpio_onkey), gpio_get_value(pdata->gpio_radio_off), gpio_get_value(pdata->gpio_reset));
	return 0;
}

static int __devexit qic_modem_remove(struct platform_device *pdev)
{
	struct qic_modem_platform_data *pdata = pdev->dev.platform_data;

	if (QIC_MODEM_GPIO_NULL != pdata->gpio_power_enable)
	{
		gpio_free(pdata->gpio_power_enable);
	}
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_host_wakeup_wwan)
	{
		gpio_free(pdata->gpio_host_wakeup_wwan);
	}
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_radio_off)
	{
		gpio_free(pdata->gpio_radio_off);
	}
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_reset)
	{
		gpio_free(pdata->gpio_reset);
		qic_modem_gpio_reset=0;
	}
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_onkey)
	{
		qic_modem_power_down_by_onkey();
		gpio_free(pdata->gpio_onkey);
		qic_modem_gpio_onkey = 0;
	}
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_fatal)
	{
		free_irq(gpio_to_irq(pdata->gpio_fatal), pdata);	
		gpio_free(pdata->gpio_fatal);
	}
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_modem_wakeup_host)
	{
		free_irq(gpio_to_irq(pdata->gpio_modem_wakeup_host), pdata);	
		gpio_free(pdata->gpio_modem_wakeup_host);
	}
	if (QIC_MODEM_GPIO_NULL != pdata->gpio_sim_detection)
	{
		free_irq(gpio_to_irq(pdata->gpio_sim_detection), pdata);	
		gpio_free(pdata->gpio_sim_detection);
	}

	sysfs_remove_group(&pdev->dev.kobj, &qic_modem_attr_group);
	
	return 0;
}
#ifdef CONFIG_PM
static int qic_modem_suspend(struct device *dev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct qic_modem_platform_data *pdata = pdev->dev.platform_data;	 

	printk("%s modem_suspend.\n", __func__);

    return 0;
}

static int qic_modem_resume(struct device *dev)
{
  
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct qic_modem_platform_data *pdata = pdev->dev.platform_data;
	
	printk("%s modem resume.\n", __func__);
	
    return 0;
}

static const struct dev_pm_ops qic_modem_pm_ops = {
	.suspend	= qic_modem_suspend,
	.resume	= qic_modem_resume,
};
#endif

static struct platform_driver qic_modem_driver = {
	.probe = qic_modem_probe,
	.remove = __devexit_p(qic_modem_remove),
	.driver		= {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &qic_modem_pm_ops,
#endif
	}	
};

static int __init qic_modem_init(void)
{
    pr_info("%s.\n", __func__);
    return platform_driver_register(&qic_modem_driver);
}

static void __exit qic_modem_exit(void)
{
	platform_driver_unregister(&qic_modem_driver);
}

module_init(qic_modem_init);
module_exit(qic_modem_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("QIC modem driver");
MODULE_AUTHOR("QIC");
