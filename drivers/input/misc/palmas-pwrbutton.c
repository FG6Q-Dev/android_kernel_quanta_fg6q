/**
 * palmas-pwrbutton.c - PALMAS Power Button Input Driver
 *
 * Copyright (C) 2013 Quanta Computer, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/palmas.h>
#include <linux/delay.h>


struct button_data {
	struct input_dev *input;
	struct palmas		*palmas;
	struct work_struct work;
	struct completion completion;
	struct timer_list timer;
	unsigned int timer_debounce;	/* in msecs */
	unsigned int irq;
	spinlock_t lock;
	bool key_pressed;
};


static irqreturn_t powerbutton_irq(int irq, void *_pwr)
{
	struct input_dev *pwr = _pwr;

	dev_info(&pwr->dev, "PWR BTN interrupt %d\n", irq);

	input_report_key(pwr, KEY_POWER, 1);
	input_report_key(pwr, KEY_POWER, 0);
	input_sync(pwr);

	return IRQ_HANDLED;
}

static void keys_work_func(struct work_struct *work)
{
	struct button_data *bdata =
		container_of(work, struct button_data, work);
	struct input_dev *input = bdata->input;
	unsigned long flags;
	unsigned int status;
	int err;
	int count = 0;

	while(1) {
		mdelay(100);
		err = palmas_read(bdata->palmas, PALMAS_INTERRUPT_BASE,
							PALMAS_INT1_LINE_STATE, &status);
		if (err < 0) {
			dev_info(&input->dev,
					"NT1_LINE_STATE read failed: %d\n", err);
		}

		if ((status & PALMAS_INT1_LINE_STATE_PWRON) == 0x00) {
			count++;
		} else {
			break;
		}

		if (count >= 15) {
			bdata->key_pressed = true;
			count = 0;
			break;
		}
	}

	spin_lock_irqsave(&bdata->lock, flags);
	if (!bdata->key_pressed) {
		dev_info(&input->dev, "keys_irq_timer: input_event ==> 0\n");
		input_event(input, EV_KEY, KEY_POWER, 0);		
		bdata->key_pressed = false;
	}
	input_sync(input);
	
	spin_unlock_irqrestore(&bdata->lock, flags);

}

static void keys_irq_timer(unsigned long _data)
{
	struct button_data *bdata = (struct button_data *)_data;
	//struct input_dev *input = bdata->input;

	schedule_work(&bdata->work);

#if 0
	spin_lock_irqsave(&bdata->lock, flags);
	if (bdata->key_pressed) {
		input_event(input, EV_KEY, KEY_POWER, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}
	spin_unlock_irqrestore(&bdata->lock, flags);
#endif

}

static irqreturn_t keys_irq_isr(int irq, void *dev_id)
{
	struct button_data *bdata = dev_id;
	struct input_dev *input = bdata->input;
	unsigned long flags;
	
	dev_info(&input->dev, "PWR BTN interrupt %d\n", irq);

	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);

	if (bdata->key_pressed) {
		input_event(input, EV_KEY, KEY_POWER, 0);
		input_sync(input);

		bdata->key_pressed = false;
	}

	if (!bdata->key_pressed) {
		input_event(input, EV_KEY, KEY_POWER, 1);
		//input_sync(input);
	}

	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));

	spin_unlock_irqrestore(&bdata->lock, flags);

	return IRQ_HANDLED;
}


static int __devinit palmas_pwrbutton_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	int irq = platform_get_irq(pdev, 0);
	int err;
	struct button_data *bdata;
	irq_handler_t isr;
	unsigned long irqflags;

	bdata = devm_kzalloc(&pdev->dev, sizeof(*bdata), GFP_KERNEL);
	if (!bdata) {
		dev_err(&pdev->dev, "Memory allocation failed for bdata\n");
		return -ENOMEM;
	}
	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		return -ENOMEM;
	}

	pwr->evbit[0] = BIT_MASK(EV_KEY);
	pwr->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	pwr->name = "palmas_pwrbutton";
	pwr->phys = "palmas_pwrbutton/input0";
	pwr->dev.parent = &pdev->dev;

	/*err = request_threaded_irq(irq, NULL, powerbutton_irq,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"palmas_pwrbutton", pwr);
		if (err < 0) {
			dev_dbg(&pdev->dev, "Can't get IRQ for pwrbutton: %d\n", err);
			goto free_input_dev;
	}*/


	bdata->irq = irq;
	INIT_WORK(&bdata->work, keys_work_func);
	bdata->timer_debounce = 300;
	bdata->palmas = dev_get_drvdata(pdev->dev.parent);
	dev_set_drvdata(&pdev->dev, bdata);

	bdata->input = pwr;
	setup_timer(&bdata->timer,
			keys_irq_timer, (unsigned long)bdata);

	isr = keys_irq_isr;
	irqflags = 0;

	err = request_any_context_irq(bdata->irq, isr, irqflags, "palmas_pwrbutton", bdata);
	if (err < 0) {
		dev_err(&pdev->dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, err);
		goto free_input_dev;
	}

	err = input_register_device(pwr);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power button: %d\n", err);
		goto free_irq;
	}

	platform_set_drvdata(pdev, pwr);
	
	device_set_wakeup_capable(&pdev->dev, 1);

	return 0;

free_irq:
	free_irq(irq, pwr);
free_input_dev:
	input_free_device(pwr);
	return err;
}

static int __devexit palmas_pwrbutton_remove(struct platform_device *pdev)
{
	struct input_dev *pwr = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);

	free_irq(irq, pwr);
	input_unregister_device(pwr);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int palmas_pwrbutton_suspend(struct device *dev)
{
	struct button_data *bdata = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(bdata->irq);
	return 0;
}

static int palmas_pwrbutton_resume(struct device *dev)
{
	struct button_data *bdata = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(bdata->irq);
	return 0;
};
#endif

static const struct dev_pm_ops palmas_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(palmas_pwrbutton_suspend,
				palmas_pwrbutton_resume)
};

static struct platform_driver palmas_pwrbutton_driver = {
        .probe          = palmas_pwrbutton_probe,
	.remove		= __exit_p(palmas_pwrbutton_remove),
	.driver		= {
		.name	= "palmas-pwrbutton",
		.owner	= THIS_MODULE,
		.pm = &palmas_pm_ops,
	},
};

static int __init palmas_pwrbutton_init(void)
{
	return platform_driver_register(&palmas_pwrbutton_driver);
}
module_init(palmas_pwrbutton_init);

static void __exit palmas_pwrbutton_exit(void)
{
	platform_driver_unregister(&palmas_pwrbutton_driver);
}
module_exit(palmas_pwrbutton_exit);

MODULE_ALIAS("platform:palmas-pwrbutton");
MODULE_DESCRIPTION("Palmas Power Button");
MODULE_LICENSE("GPL");

