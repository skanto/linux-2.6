/*
 * GPIO driver for TM-EFDC Digital and Analog I/O
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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include <linux/freezer.h>
#include <linux/poll.h>

#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/at91_pio.h>
#include <mach/gpio.h>

#include <linux/tm_efdc-gpio.h>

#define	DRIVER_NAME		"efdc-gpio"
#define	POLL_INTERVAL		1000000UL	// timer callback interval (ns)
#define	ADC_SLIDE_WIN		16		// length of ADC sliding window
#define	LOAD_DET_INTERVAL	5000		// open-load detection interval (in ms)

struct efdcgpio_dev {
	struct hrtimer		timer;		// timer for constant polling
	struct task_struct	*kthread;	// kthread for polling
	struct fasync_struct	*async_queue;	// queue for the misc device
	wait_queue_head_t	misc_wait;	// wait queue for the misc device
	unsigned long		misc_opened;	//

	tm_efdc_gpio_t		*gpio;		// values

	u_int32_t		di_last;	// last state of digital inputs
	int			di_delay[TM_EFDC_DI_COUNT];	// current delay value for inputs

	unsigned		pwm_bit;	// PWM-bit number
	u_int32_t		pwm_value[TM_EFDC_DO_COUNT];	// current pwm value

  	u_int32_t		adc_slide[TM_EFDC_AIN_COUNT];	// sliding window..
} efdcgpio_dev = {
};

/* ......................................................................... */

static enum hrtimer_restart tm_efdc_gpio_timer_cb(struct hrtimer *tmr)
{
	if (efdcgpio_dev.kthread) {
		wake_up_process(efdcgpio_dev.kthread);
	}
	hrtimer_add_expires_ns(tmr, POLL_INTERVAL);
	return HRTIMER_RESTART;
}

/* ......................................................................... */

static __inline void poll_dio(struct efdcgpio_dev *dev)
{
  int c;
  u_int32_t do_raw, di_raw, do_bits, do_pwm_ena, di_old, di_new;
  int val[TM_EFDC_DO_COUNT];

  // first build word to transfer, take current PWM bit for all channels

  for (c = 0, do_raw = 0; c < TM_EFDC_DO_COUNT; c++) {	// process all outputs
    do_raw <<= 1;
    do_raw |= (dev->pwm_value[c] >> dev->pwm_bit) & 1;
  }

  if (++dev->pwm_bit >= TM_EFDC_DO_PWM_NBITS) {		// wrap PWM bit number
    dev->pwm_bit = 0;
    for (c = 0; c < TM_EFDC_DO_COUNT; c++) {		// copy new PWM values
      dev->pwm_value[c] = dev->gpio->DO_PWM[c];
    }
  }

  do_bits = dev->gpio->DO;				// get output states
  do_pwm_ena = dev->gpio->DO_PWM_Ena;			// get mask of PWM channels
  do_raw = (do_raw & do_pwm_ena & do_bits) | (do_bits & ~do_pwm_ena);

  // transfer to GPIO..
  for (c = 0; c < TM_EFDC_DO_COUNT; c++) {		// interface is a bit stupid..
    val[c] = do_raw & (1U << c) ? 1 : 0;
  }
  gpio_set_values(TM_EFDC_DOUT_BASE, TM_EFDC_DO_COUNT, &val[0]);

  // read GPIO:
  gpio_get_values(TM_EFDC_DIN_BASE, TM_EFDC_DI_COUNT, &val[0]);

  for (di_raw = 0, c = 0; c < TM_EFDC_DI_COUNT; c++) {
    di_raw <<= 1;
    if (val[c]) {
      di_raw |= 1;
    }
  }

  di_raw ^= dev->gpio->DI_Invert;			// apply invert

  dev->gpio->DI_Raw = di_raw;
  dev->gpio->DO_Raw = do_raw;				// show to user-space too

  // filter inputs:
  di_old = dev->di_last;				// get last state of inputs
  di_new = di_old;					// prepare new state of inputs

  for (c = 0; c < TM_EFDC_DI_COUNT; c++) {
    if (di_raw & (1U << c)) {				// new sample was 1:
      if (++dev->di_delay[c] >= 0) {			  // time to change filtered state
	if (!(di_old & (1U << c))) {			    // old filtered state was 0:
	  dev->gpio->DI_Pulses[c]++;			      // increment pulse count
	}
	dev->di_delay[c] = dev->gpio->DI_OffDelay[c];       // reset delay
	di_new |= (1U << c);				    // update state
      }
    } else {						// new sample was 0:
      if (--dev->di_delay[c] <= 0) {			  // time to change filtered state
	dev->di_delay[c] = -dev->gpio->DI_OnDelay[c];       // reset delay
	di_new &= ~(1U << c);				    // update state
      }
    }
  }

  dev->gpio->DI = dev->di_last = di_new;		// update state
}

static __inline void poll_aio(struct efdcgpio_dev *dev)
{
  int c;
  int val[TM_EFDC_AOUT_COUNT < TM_EFDC_AIN_COUNT ? TM_EFDC_AIN_COUNT : TM_EFDC_AOUT_COUNT];

  // process analog outputs:
  for (c = 0; c < TM_EFDC_AOUT_COUNT; c++) {
    val[c] = dev->gpio->AO[c];
  }

  // write analog outputs:
  gpio_set_values(TM_EFDC_AOUT_BASE, TM_EFDC_AOUT_COUNT, &val[0]);

  // read analog inputs:
  gpio_get_values(TM_EFDC_AIN_BASE, TM_EFDC_AIN_COUNT, &val[0]);

  // sliding window filtering for analog inputs:
  for (c = 0; c < TM_EFDC_AIN_COUNT; c++) {
    u_int32_t slide = dev->adc_slide[c];
    slide -= (slide + ADC_SLIDE_WIN / 2) / ADC_SLIDE_WIN;
    slide += (u_int16_t)val[c];
    dev->adc_slide[c] = slide;
    dev->gpio->AI[c] = slide / ADC_SLIDE_WIN;
  }
}

static int tm_efdc_gpio_kthread(void *data)
{
  struct efdcgpio_dev *dev = data;

  set_current_state(TASK_UNINTERRUPTIBLE);

  while (!kthread_should_stop()) {
    // waiting for signal..
    try_to_freeze();
    schedule();

    // poll digital I/O!!
    poll_dio(dev);

    // poll analog I/O
    poll_aio(dev);
  }

  set_current_state(TASK_RUNNING);

  return 0;
}

/* ......................................................................... */

/*
 * GPIO device is opened.
 */
static int tm_efdc_gpio_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &efdcgpio_dev.misc_opened))
		return -EBUSY;

	return nonseekable_open(inode, file);
}

/*
 * Close the GPIO device.
 */
static int tm_efdc_gpio_close(struct inode *inode, struct file *file)
{
	//kthread_stop(efdcgpio_dev.kthread);
	clear_bit(0, &efdcgpio_dev.misc_opened);
	return 0;
}


/*
 * Handle commands from user-space.
 */
static long tm_efdc_gpio_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	//void __user *argp = (void __user *)arg;
	//int __user *p = argp;
	//int new_value, err;

	switch (cmd) {
	default:
		return -ENOTTY;
	}
}


/*
 * Read an event:
 */
static ssize_t tm_efdc_gpio_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	return 0;
}



/*
 * Pat the watchdog whenever device is written to.
 */
static ssize_t tm_efdc_gpio_write(struct file *file, const char *data,
						size_t len, loff_t *ppos)
{
	return len;
}

/* ......................................................................... */

static int tm_efdc_gpio_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long page, pos;

	if (offset + size > PAGE_ALIGN(sizeof(*efdcgpio_dev.gpio))) {
		return -EINVAL;
	}

	pos = (unsigned long)efdcgpio_dev.gpio;

	while (size > 0) {
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED)) {
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	vma->vm_flags |= VM_RESERVED;   /* avoid to swap out this VMA */

	return 0;
}

/* ......................................................................... */

static unsigned int tm_efdc_gpio_poll(struct file *file, poll_table *wait)
{
	return 0;
}

/* ......................................................................... */

static const struct file_operations tm_efdc_gpio_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.open		= tm_efdc_gpio_open,
	.release	= tm_efdc_gpio_close,
  	.read		= tm_efdc_gpio_read,
	.write		= tm_efdc_gpio_write,
	.poll		= tm_efdc_gpio_poll,
	.unlocked_ioctl	= tm_efdc_gpio_ioctl,
	.mmap		= tm_efdc_gpio_mmap,
};

static struct miscdevice tm_efdc_gpio_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "efdcgpio",
	.fops		= &tm_efdc_gpio_fops,
};

static int __init tm_efdc_gpio_probe(struct platform_device *pdev)
{
	int res;

	if (tm_efdc_gpio_miscdev.parent)
		return -EBUSY;

	tm_efdc_gpio_miscdev.parent = &pdev->dev;

	res = misc_register(&tm_efdc_gpio_miscdev);
	if (res)
		return res;

	return 0;
}

static int __exit tm_efdc_gpio_remove(struct platform_device *pdev)
{
	int res;

	res = misc_deregister(&tm_efdc_gpio_miscdev);
	if (!res)
		tm_efdc_gpio_miscdev.parent = NULL;

	return res;
}

#ifdef CONFIG_PM

static int tm_efdc_gpio_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int tm_efdc_gpio_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define tm_efdc_gpio_suspend	NULL
#define tm_efdc_gpio_resume	NULL
#endif

static struct platform_driver tm_efdc_gpio_driver = {
	.remove		= __exit_p(tm_efdc_gpio_remove),
	.suspend	= tm_efdc_gpio_suspend,
	.resume		= tm_efdc_gpio_resume,
	.driver		= {
		.name	= "tm_efdc_gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init tm_efdc_gpio_init(void)
{
	unsigned long size, adr;

	// prepare hrtimer:
	hrtimer_init(&efdcgpio_dev.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	efdcgpio_dev.timer.function = tm_efdc_gpio_timer_cb;

	// allocate memory for shared memory:
	size = PAGE_ALIGN(sizeof(*efdcgpio_dev.gpio));
	efdcgpio_dev.gpio = vmalloc_32(size);
	if (!efdcgpio_dev.gpio) {
		return -ENOMEM;
	}

	// clear memory:
	memset((void*)efdcgpio_dev.gpio, 0, size);

	adr = (unsigned long)efdcgpio_dev.gpio;
	while (size > 0) {
		SetPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	// create kthread:
	efdcgpio_dev.kthread = kthread_run(tm_efdc_gpio_kthread, &efdcgpio_dev, "efdcgpiod");
	if (IS_ERR(efdcgpio_dev.kthread)) {
		int ret = PTR_ERR(efdcgpio_dev.kthread);
		clear_bit(0, &efdcgpio_dev.misc_opened);
		return ret;
	}

	hrtimer_start(&efdcgpio_dev.timer, ktime_set(0, POLL_INTERVAL), HRTIMER_MODE_REL);

	return platform_driver_probe(&tm_efdc_gpio_driver, tm_efdc_gpio_probe);
}

static void __exit tm_efdc_gpio_exit(void)
{
	platform_driver_unregister(&tm_efdc_gpio_driver);
}

module_init(tm_efdc_gpio_init);
module_exit(tm_efdc_gpio_exit);

MODULE_AUTHOR("Sami Kantoluoto");
MODULE_DESCRIPTION("GPIO driver for TM-EFDC board");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:tm_efdc_gpio");
