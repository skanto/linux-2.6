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

	tm_efdc_gpio_t		*gpio;		// values

	unsigned		configured:1;	// set if device has been configured

	u32			di_last;	// last state of digital inputs
	u32			do_raw;		// last transmitted do_raw
	int			di_delay[TM_EFDC_DIN_COUNT];	// current delay value for inputs

	u32			pwr_last;	// last state of power bits
	int			pwr_delay[2];	// delay value for power bits

	u32			id_last;	// last state of ID switches:
	int			id_delay;	// delay value for ID switches

	unsigned		pwm_bit;	// PWM-bit number
	u32			pwm_value[TM_EFDC_DOUT_COUNT];	// current pwm value

  	u32			adc_slide[TM_EFDC_AIN_COUNT];	// sliding window..
} efdcgpio_dev = {
};

static const volatile u16 pwr_on_delay[2] = { 500, 500 };
static const volatile u16 pwr_off_delay[2] = { 500, 500 };

#if TM_EFDC_DOUT_PWM_NBITS == 32
static const u32 pwm_32_table[33] = {
  0x00000000, 0x00000001, 0x00010001, 0x00200801, 0x01010101, 0x04082041,
  0x08210821, 0x08844221, 0x11111111, 0x12244891, 0x24492449, 0x24929249,
  0x29292929, 0x4A5294A5, 0x4AA54AA5, 0x54AAAA55, 0x55555555, 0x55AAAB55,
  0x5AB55AB5, 0x6B5AB5AD, 0x6D6D6D6D, 0x6DB6DB6D, 0xB6DBB6DB, 0xB76EEDDB,
  0xBBBBBBBB, 0xBDDEF77B, 0xDEF7DEF7, 0xDFBEFBF7, 0xEFEFEFEF, 0xF7FEFFDF,
  0xFEFFFEFF, 0xFFFEFFFF, 0xFFFFFFFF
};
#define	pwm_table	pwm_32_table
#endif

#if TM_EFDC_DOUT_PWM_NBITS == 20
static const u32 pwm_20_table[21] = {
  0x00000, 0x00001, 0x00401, 0x02081, 0x08421, 0x11111, 0x22489, 0x24A49,
  0x4A529, 0x52A95, 0x55555, 0x5AAB5, 0x6B5AD, 0x6DB6D, 0xB6EDB, 0xBBBBB,
  0xDEF7B, 0xDFBF7, 0xF7FDF, 0xFFBFF, 0xFFFFF
};
#define	pwm_table	pwm_20_table
#endif

#if TM_EFDC_DOUT_PWM_NBITS == 10
static const u32 pwm_10_table[11] = {
  0x000, 0x001, 0x021, 0x089, 0x129, 0x155, 0x1AD, 0x2DB, 0x37B, 0x3DF, 0x3FF
};
#define	pwm_table	pwm_10_table
#endif

#ifndef	pwm_table
#error	Check TM_EFDC_DOUT_PWM_NBITS!
#endif

/* ......................................................................... */
int tm_efdc_dio_transfer(u32 dout, void (*complete)(void *, u32, u32), void *context);

static u32 tm_efdc_gpio_in_filter(struct efdcgpio_dev *dev, int count, u32 raw, u32 state,
				  int *delay, volatile const u16 *ond, volatile const u16 *offd)
{
  u32 new_state = state;				// prepare new state
  int c;

  for (c = 0; c < count; c++) {
    if (raw & (1U << c)) {				// new sample was 1:
      if (++delay[c] >= 0) {				  // time to change filtered state
	if (!(state & (1U << c))) {			    // old filtered state was 0:
	  dev->gpio->DI_Pulses[c]++;			      // increment pulse count
	}
	new_state |= (1U << c);				    // update state
	delay[c] = offd[c];				    // reset delay
      }
    } else {						// new sample was 0:
      if (--delay[c] <= 0) {				  // time to change filtered state
	new_state &= ~(1U << c);			    // update state
	delay[c] = -ond[c];				    // reset delay
      }
    }
  }

  return new_state;
}

static void tm_efdc_gpio_dio_polled(void *context, u32 di_raw, u32 do_stat)
{
  struct efdcgpio_dev *dev = context;
  u32 di_new;

  // combine...
  di_raw ^= dev->gpio->DI_Invert;			// apply invert

  dev->gpio->DO_Raw = dev->do_raw;			// show to user-space too
  dev->gpio->DI_Raw = di_raw;

  // filter inputs:
  di_new = tm_efdc_gpio_in_filter(dev, TM_EFDC_DIN_COUNT, di_raw, dev->di_last,
				  dev->di_delay, dev->gpio->DI_OnDelay, dev->gpio->DI_OffDelay);
  dev->gpio->DI = dev->di_last = di_new;
}

static __inline void poll_dio(struct efdcgpio_dev *dev)
{
  int c;
  u32 do_raw, do_bits, do_pwm_ena;

  // first build word to transfer, take current PWM bit for all channels
  for (c = 0, do_raw = 0; c < TM_EFDC_DOUT_COUNT; c++) {	// process all outputs
    do_raw <<= 1;
    do_raw |= (dev->pwm_value[c] >> dev->pwm_bit) & 1;
  }

  if (++dev->pwm_bit >= TM_EFDC_DOUT_PWM_NBITS) {	// wrap PWM bit number
    dev->pwm_bit = 0;
    for (c = 0; c < TM_EFDC_DOUT_COUNT; c++) {		// copy new PWM values
      int p = dev->gpio->DO_PWM_Set[c], mp = dev->gpio->DO_PWM_Min[c];
      if (p < mp) {
	p = mp;
      }
      dev->gpio->DO_PWM[c] = p;
      dev->pwm_value[c] = pwm_table[(((unsigned)p * TM_EFDC_DOUT_PWM_NBITS + 50) / 100)];
    }
  }

  do_bits = dev->gpio->DO;				// get output states
  do_pwm_ena = dev->gpio->DO_PWM_Ena;			// get mask of PWM channels
  do_raw = (do_raw & do_pwm_ena & do_bits) | (do_bits & ~do_pwm_ena);

  dev->do_raw = do_raw;

  // transfer data:
  tm_efdc_dio_transfer(do_raw, tm_efdc_gpio_dio_polled, dev);
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
    u32 slide = dev->adc_slide[c];
    slide -= (slide + ADC_SLIDE_WIN / 2) / ADC_SLIDE_WIN;
    slide += (u16)val[c];
    dev->adc_slide[c] = slide;
    dev->gpio->AI[c] = slide / ADC_SLIDE_WIN;
  }
}

static int tm_efdc_gpio_kthread(void *data)
{
  struct efdcgpio_dev *dev = data;

  while (!kthread_should_stop()) {
    // waiting for signal..
    set_current_state(TASK_UNINTERRUPTIBLE);
    schedule();

    // poll analog I/O
    poll_aio(dev);

    // increment counter
    dev->gpio->PollCount++;
  }

  return 0;
}

#define	ID_SWITCH_DELAY	200

/* ......................................................................... */
static enum hrtimer_restart tm_efdc_gpio_timer_cb(struct hrtimer *tmr)
{
	struct efdcgpio_dev *dev = &efdcgpio_dev;

	// get switches and power status:
	u32 portb = at91_sys_read(AT91_PIOB + PIO_PDSR);	// read port B inputs
	u32 portc = at91_sys_read(AT91_PIOC + PIO_PDSR);	// read port C inputs

	hrtimer_add_expires_ns(tmr, POLL_INTERVAL);

	// poll digital I/O
	if (dev->gpio->Configured || dev->configured) {
		dev->configured = 1;
		poll_dio(dev);
	}

	// filter power bits:
	dev->pwr_last = tm_efdc_gpio_in_filter(dev, 2, (portc >> 13), dev->pwr_last,
					       dev->pwr_delay, pwr_on_delay, pwr_off_delay);
	dev->gpio->Power = dev->pwr_last;

	// filter switch inputs:
	portb = ((~portb) >> 24) & 0xff;

	if (portb == dev->id_last) {
		if (--dev->id_delay <= 0) {
			dev->gpio->ID = portb;
			dev->id_delay = ID_SWITCH_DELAY;
		}
	} else {
		if (++dev->id_delay > ID_SWITCH_DELAY) {
			dev->id_delay = ID_SWITCH_DELAY;
		}
	}

	dev->id_last = portb;

	// wake up thread..
	if (dev->configured && efdcgpio_dev.kthread) {
		wake_up_process(efdcgpio_dev.kthread);
	}

	return HRTIMER_RESTART;
}


/* ......................................................................... */

/*
 * GPIO device is opened.
 */
static int tm_efdc_gpio_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

/*
 * Close the GPIO device.
 */
static int tm_efdc_gpio_close(struct inode *inode, struct file *file)
{
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

static const struct {
	unsigned	base, count;
	int		output;
	const char *	label;
} gpio_table[] = {
#if 0
	{
		.base = TM_EFDC_DIN_BASE,
		.count = TM_EFDC_DIN_COUNT,
		.label = "DIN"
	},
	{
		.base = TM_EFDC_DOUT_S_BASE,
		.count = TM_EFDC_DOUT_COUNT,
		.label = "DOUT_S"
	},
	{
		.base = TM_EFDC_DOUT_BASE,
		.count = TM_EFDC_DOUT_COUNT,
		.output = 1,
		.label = "DOUT"
	},
#endif
	{
		.base = TM_EFDC_AIN_BASE,
		.count = TM_EFDC_AIN_COUNT,
		.label = "AIN"
	},
	{
		.base = TM_EFDC_AOUT_BASE,
		.count = TM_EFDC_AOUT_COUNT,
		.output = 1,
		.label = "AOUT"
	},
	{
		.label = NULL
	}
};

static void free_gpios(int index, int gpio)
{
	int i;

	for (;;) {
		while (gpio > gpio_table[index].base) {
			gpio_free(gpio--);
		}
		if (--i < 0)
			break;
		gpio = gpio_table[i].base + gpio_table[i].count - 1;
	}
}

static int request_gpios(struct device *dev)
{
	int i, gpio, gpioe, res = 0;
	char buf[32];

	for (i = 0; gpio_table[i].label && res == 0; i++) {
		for (gpio = gpio_table[i].base,
		     gpioe = gpio + gpio_table[i].count; gpio < gpioe; gpio++) {
			snprintf(buf, sizeof(buf), "%.8s%d", gpio_table[i].label, gpio - gpio_table[i].base + 1);

			// request gpio:
			if ((res = gpio_request(gpio, buf)) != 0) {
				dev_err(dev, "couldn't request GPIO '%s' (#%u): %d\n",
					buf, gpio, res);
				// free requested GPIOs:
				free_gpios(i, gpio);
				// and get out:
				break;
			}

			// set gpio direction:
			if (gpio_table[i].output) {
				res = gpio_direction_output(gpio, -1);
			} else {
				res = gpio_direction_input(gpio);
			}

			// bail out on failure
			if (res) {
				dev_err(dev, "couldn't set GPIO '%s' (#%u) to %s: %d\n",
					buf, gpio, gpio_table[i].output ? "output" : "input", res);
				break;
			}
		}
	}

	return res;
}

static int __devinit tm_efdc_gpio_probe(struct platform_device *pdev)
{
	unsigned long size, adr;
	int res, c;

	if (tm_efdc_gpio_miscdev.parent) {
		printk(KERN_ERR DRIVER_NAME ": already probed!\n");
		return -EBUSY;
	}

	tm_efdc_gpio_miscdev.parent = &pdev->dev;

	if (!efdcgpio_dev.gpio) {
		// allocate memory for shared memory:
		size = PAGE_ALIGN(sizeof(*efdcgpio_dev.gpio));
		efdcgpio_dev.gpio = vmalloc_32(size);
		if (!efdcgpio_dev.gpio) {
			printk(KERN_ERR DRIVER_NAME ": memory allocation failed\n");
			return -ENOMEM;
		}

		// clear memory:
		memset((void*)efdcgpio_dev.gpio, 0, size);

		// initialize parameters:
		for (c = 0; c < TM_EFDC_DIN_COUNT; c++) {
			efdcgpio_dev.gpio->DI_OnDelay[c] = 20;
			efdcgpio_dev.gpio->DI_OffDelay[c] = 20;
		}

		adr = (unsigned long)efdcgpio_dev.gpio;
		while (size > 0) {
			SetPageReserved(vmalloc_to_page((void *)adr));
			adr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
	}

	// request I/O:
	if ((res = request_gpios(&pdev->dev)) != 0) {
		printk(KERN_ERR DRIVER_NAME ": failed to request GPIOs: %d\n", res);
		return res;
	}

	// register device:
	res = misc_register(&tm_efdc_gpio_miscdev);
	if (res) {
		printk(KERN_ERR DRIVER_NAME ": misc_register() failed: %d\n", res);
		return res;
	}

	// prepare hrtimer:
	hrtimer_init(&efdcgpio_dev.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	efdcgpio_dev.timer.function = tm_efdc_gpio_timer_cb;

	// create kthread:
	efdcgpio_dev.kthread = kthread_create(tm_efdc_gpio_kthread, &efdcgpio_dev, "efdcgpiod");
	if (IS_ERR(efdcgpio_dev.kthread)) {
		int ret = PTR_ERR(efdcgpio_dev.kthread);
		printk(KERN_ERR DRIVER_NAME ": kthread_create() failed: %d\n", ret);
		return ret;
	}
	efdcgpio_dev.kthread->prio = 1;	// @@@ we want priority!

	// start ticking
	hrtimer_start(&efdcgpio_dev.timer, ktime_set(0, POLL_INTERVAL), HRTIMER_MODE_REL);

	return 0;
}

static int __devexit tm_efdc_gpio_remove(struct platform_device *pdev)
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
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.remove		= __devexit_p(tm_efdc_gpio_remove),
	.suspend	= tm_efdc_gpio_suspend,
	.resume		= tm_efdc_gpio_resume,
	.probe		= tm_efdc_gpio_probe,
};

static int __init tm_efdc_gpio_init(void)
{
	int ret;

	ret = platform_driver_register(&tm_efdc_gpio_driver);
	if (ret)
		printk(KERN_ERR DRIVER_NAME ": register failed: %d\n", ret);

	return ret;
}

static void __exit tm_efdc_gpio_exit(void)
{
	platform_driver_unregister(&tm_efdc_gpio_driver);
}

//module_init(tm_efdc_gpio_init);

late_initcall(tm_efdc_gpio_init);
module_exit(tm_efdc_gpio_exit);

MODULE_AUTHOR("Sami Kantoluoto");
MODULE_DESCRIPTION("GPIO driver for TM-EFDC board");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:efdc-gpio");
