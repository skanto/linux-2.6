/*
 * Watchdog driver for GPIO driven watchdog.
 *
 * Copyright (C) 2010 Telemerkki Oy <www.telemerkki.fi>
 * Copyright (C) 2009 Embedtronics Oy
 * Copyright (C) 2007 Renaud CERRATO r.cerrato@til-technologies.fr
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>

#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/at91_pio.h>
#include <mach/gpio.h>
#include <mach/at91_wdt.h>

#define	WDT_TIMER_INTERVAL	100UL	/* timer callback interval (ms) */
#define	WDT_TIMEOUT		16	/* watchdog timeout (s) */

static volatile int wdt_cntr = WDT_TIMEOUT * 1000 / WDT_TIMER_INTERVAL;
static struct hrtimer wdt_timer;
static int wdt_timeout = WDT_TIMEOUT;
static volatile unsigned long at91sam9_gpio_wdt_busy;
static unsigned at91sam9_gpio_pin;

/* ......................................................................... */

/*
 * Reload the watchdog timer.  (ie, pat the watchdog)
 */
static inline void at91sam9_gpio_wdt_reload(void)
{
	at91_set_gpio_value(at91sam9_gpio_pin, 1);
	at91_sys_write(AT91_WDT_CR, AT91_WDT_KEY | AT91_WDT_WDRSTT);
	at91_set_gpio_value(at91sam9_gpio_pin, 0);
}

static enum hrtimer_restart at91sam9_gpio_wdt_timer_cb(struct hrtimer *tmr)
{
	int cntr;
	if (!at91sam9_gpio_wdt_busy) {
		wdt_cntr = wdt_timeout;
		at91sam9_gpio_wdt_reload();
	} else if ((cntr = wdt_cntr) > 0) {
		wdt_cntr = cntr - 1;
		at91sam9_gpio_wdt_reload();
	}
	hrtimer_add_expires_ns(tmr, WDT_TIMER_INTERVAL * 1000000UL);
	return HRTIMER_RESTART;
}


/* ......................................................................... */

/*
 * Watchdog device is opened, and watchdog starts running.
 */
static int at91sam9_gpio_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &at91sam9_gpio_wdt_busy))
		return -EBUSY;

	return nonseekable_open(inode, file);
}

/*
 * Close the watchdog device.
 */
static int at91sam9_gpio_wdt_close(struct inode *inode, struct file *file)
{
	clear_bit(0, &at91sam9_gpio_wdt_busy);
	return 0;
}

/*
 * Change the watchdog time interval.
 */
#define	WDT_MAX_TIME	16
static int at91sam9_gpio_wdt_settimeout(int new_time)
{
	unsigned int reg, mr;
	/*
	 * All counting occurs at SLOW_CLOCK / 128 = 256 Hz
	 *
	 * Since WDV is a 12-bit counter, the maximum period is
	 * 4096 / 256 = 16 seconds.
	 */
	if ((new_time <= 0) || (new_time > WDT_MAX_TIME))
		return -EINVAL;

	wdt_cntr = new_time * 1000 / WDT_TIMER_INTERVAL;
	wdt_timeout = new_time;

	/* Program the Watchdog */
	reg = AT91_WDT_WDRSTEN					/* causes watchdog reset */
		| AT91_WDT_WDRPROC				/* causes processor reset */
		| AT91_WDT_WDDBGHLT				/* disabled in debug mode */
		| AT91_WDT_WDD					/* restart at any time */
		| AT91_WDT_WDV/*(((wdt_timeout * 256) - 1) & AT91_WDT_WDV)*/;	/* timer value */
	at91_sys_write(AT91_WDT_MR, reg);

	/* Check if watchdog could be programmed */
	mr = at91_sys_read(AT91_WDT_MR);
	if (mr != reg) {
		printk(KERN_ERR "at91sam9_gpio_wdt: Watchdog register already programmed.\n");
		return -EIO;
	}

	at91sam9_gpio_wdt_reload();

	printk(KERN_INFO "AT91SAM9 Watchdog enabled (%d seconds, nowayout)\n", wdt_timeout);
	return 0;
}

static struct watchdog_info at91sam9_gpio_wdt_info = {
	.identity	= "at91sam9 watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
};


/*
 * Handle commands from user-space.
 */
static long at91sam9_gpio_wdt_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value, err;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &at91sam9_gpio_wdt_info,
				sizeof(at91sam9_gpio_wdt_info)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_KEEPALIVE:
		wdt_cntr = wdt_timeout * 1000 / WDT_TIMER_INTERVAL;
		at91sam9_gpio_wdt_reload();	/* pat the watchdog */
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;

		err = at91sam9_gpio_wdt_settimeout(new_value);
		if (err)
			return err;

		return put_user(wdt_timeout, p);	/* return current value */
	case WDIOC_GETTIMEOUT:
		return put_user(wdt_timeout, p);
	default:
		return -ENOTTY;

	}
}

/*
 * Pat the watchdog whenever device is written to.
 */
static ssize_t at91sam9_gpio_wdt_write(struct file *file, const char *data,
						size_t len, loff_t *ppos)
{
	wdt_cntr = wdt_timeout * 1000 / WDT_TIMER_INTERVAL;
	at91sam9_gpio_wdt_reload();		/* pat the watchdog */
	return len;
}

/* ......................................................................... */

static const struct file_operations at91sam9_gpio_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= at91sam9_gpio_wdt_ioctl,
	.open		= at91sam9_gpio_wdt_open,
	.release	= at91sam9_gpio_wdt_close,
	.write		= at91sam9_gpio_wdt_write,
};

static struct miscdevice at91sam9_gpio_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &at91sam9_gpio_wdt_fops,
};

static int __init at91sam9_gpio_wdt_probe(struct platform_device *pdev)
{
	int res;

	if (at91sam9_gpio_wdt_miscdev.parent)
		return -EBUSY;

	at91sam9_gpio_wdt_miscdev.parent = &pdev->dev;

	res = misc_register(&at91sam9_gpio_wdt_miscdev);
	if (res)
		return res;

	/* Set watchdog */
	if (at91sam9_gpio_wdt_settimeout(wdt_timeout) == -EINVAL) {
		pr_info("at91sam9_gpio_wdt: invalid timeout (must be between 1 and %d)\n", WDT_MAX_TIME);
		return 0;
	}

	return 0;
}

static int __exit at91sam9_gpio_wdt_remove(struct platform_device *pdev)
{
	int res;

	res = misc_deregister(&at91sam9_gpio_wdt_miscdev);
	if (!res)
		at91sam9_gpio_wdt_miscdev.parent = NULL;

	return res;
}

#ifdef CONFIG_PM

static int at91sam9_gpio_wdt_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int at91sam9_gpio_wdt_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define at91sam9_gpio_wdt_suspend	NULL
#define at91sam9_gpio_wdt_resume	NULL
#endif

static struct platform_driver at91sam9_gpio_wdt_driver = {
	.remove		= __exit_p(at91sam9_gpio_wdt_remove),
	.suspend	= at91sam9_gpio_wdt_suspend,
	.resume		= at91sam9_gpio_wdt_resume,
	.driver		= {
		.name	= "at91sam9_gpio_wdt",
		.owner	= THIS_MODULE,
	},
};

static int __init at91sam9_gpio_wdt_init(void)
{
	at91sam9_gpio_wdt_reload();
	hrtimer_init(&wdt_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	wdt_timer.function = at91sam9_gpio_wdt_timer_cb;
	hrtimer_start(&wdt_timer, ktime_set(0, WDT_TIMER_INTERVAL * 1000000UL), HRTIMER_MODE_REL);
	return platform_driver_probe(&at91sam9_gpio_wdt_driver, at91sam9_gpio_wdt_probe);
}

static void __exit at91sam9_gpio_wdt_exit(void)
{
	platform_driver_unregister(&at91sam9_gpio_wdt_driver);
}

static struct platform_device at91sam9_gpio_wdt_device = {
	.name		= "at91sam9_gpio_wdt",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
			.platform_data	= &at91sam9_gpio_pin,
	}
};

int __init at91_add_gpio_wdt(unsigned pin)
{
	if (!pin)
		return -EINVAL;

	at91_set_gpio_output(pin, 0);

	at91sam9_gpio_pin = pin;
	at91sam9_gpio_wdt_reload();

	platform_device_register(&at91sam9_gpio_wdt_device);

	return 0;
}

module_init(at91sam9_gpio_wdt_init);
module_exit(at91sam9_gpio_wdt_exit);

MODULE_AUTHOR("Sami Kantoluoto");
MODULE_DESCRIPTION("GPIO driven watchdog driver for AT91SAM9-family");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:at91sam9_gpio_wdt");
