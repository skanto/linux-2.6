/*
 * GPIO driver for TM-EFDC Digital and Analog I/O
 *
 * Copyright (C) 2010-2011 Telemerkki Oy <www.telemerkki.fi>
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
#include <linux/freezer.h>
#include <linux/poll.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/at91_pio.h>
#include <mach/gpio.h>

#include <linux/tm_efdc-gpio.h>

#ifdef	XENOMAI
#include <xenomai/rtdm/rtdm_driver.h>
#endif

#define	DRIVER_NAME		"efdc-gpio"
#define	DIO_POLL_HZ		(2500UL)	// poll interval (Hz)
#define	DIO_POLL_INT		(1000000000UL/DIO_POLL_HZ)	// timer callback interval (ns)
#define	LOAD_DET_INTERVAL	(5*DIO_POLL_HZ)	// open-load detection interval (in ticks)
#define	MS2DIOTCK(_ms)		(((_ms)*DIO_POLL_HZ+999)/1000)

#define	AIO_POLL_INT		(1000000UL)	// analog I/O poll interval (ns)
#define	ADC_SLIDE_WIN		16		// length of ADC sliding window

#define	ID_SWITCH_DELAY		MS2DIOTCK(200)	// ID switch delay

#define	EFDC_PIN_LOAD_DET	AT91_PIN_PC0
#define	EFDC_PIN_DO_FLAG	AT91_PIN_PC4

typedef struct efdcgpio_file_private {
	int			din_evn_num;	// DIN event number
	int			dout_evn_num;	// DOUT event number
	int			dout_status_evn_num;	// DOUT status event number
	int			aout_evn_num;	// AOUT event number
	int			power_evn_num;	// power status event number
} efdcgpio_file_private_t;

static struct efdcgpio_dev {
	struct task_struct	*aio_kthread;	// analog I/O thread
#ifndef	XENOMAI
	struct hrtimer		dio_timer;	// digital I/O timer
#endif
	struct hrtimer		aio_timer;	// analog I/O timer
	volatile int		aio_poll_count;	// analog I/O poll count
	wait_queue_head_t	aio_wqh;	// analog I/O wait queue..
#ifdef	XENOMAI
	rtdm_task_t		dio_task;	// digital I/O task
#endif

	tm_efdc_gpio_t		gpio;		// values

	atomic_t		din_evn_num;	// DIN event number
	atomic_t		dout_evn_num;	// DOUT event number
	atomic_t		dout_status_evn_num;	// DOUT status event number
	atomic_t		aout_evn_num;	// AOUT event number
	atomic_t		power_evn_num;	// power event number

	int			load_det_tick;	// load float detection

	u32			di_last;	// last state of digital inputs
	u32			do_raw;		// last transmitted do_raw
	int			di_delay[TM_EFDC_DIN_COUNT];	// current delay value for inputs
	int			di_off_delay[TM_EFDC_DIN_COUNT];
	int			di_on_delay[TM_EFDC_DIN_COUNT];

	u64			do_float;	// mask of floating outputs

	u32			pwr_last;	// last state of power bits
	int			pwr_delay[2];	// delay value for power bits

	u32			id_last;	// last state of ID switches:
	int			id_delay;	// delay value for ID switches

	unsigned		pwm_bit;	// PWM-bit number
	u32			pwm_value[TM_EFDC_DOUT_COUNT];	// current pwm value

  	u32			adc_slide[TM_EFDC_AIN_COUNT];	// sliding window..
} efdcgpio_dev;

static DECLARE_WAIT_QUEUE_HEAD(tm_efdc_gpio_event_queue);
static const int pwr_on_delay[2] = { MS2DIOTCK(500), MS2DIOTCK(500) };
static const int pwr_off_delay[2] = { MS2DIOTCK(500), MS2DIOTCK(500) };

static DEFINE_MUTEX(efdc_gpio_mutex);

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

static __inline u32 tm_efdc_gpio_in_filter(struct efdcgpio_dev *dev, int count, u32 raw, u32 state,
				  int *delay, const int *ond, const int *offd,
				  volatile u32 *pulses)
{
  u32 new_state = state;				// prepare new state
  int c;

  for (c = 0; c < count; c++) {
    if (raw & (1U << c)) {				// new sample was 1:
      if (++(delay[c]) >= 0) {				  // time to change filtered state
	if (pulses && !(state & (1U << c))) {		    // old filtered state was 0:
	  (pulses[c])++;			      	      // increment pulse count
	}
	new_state |= (1U << c);				    // update state
	delay[c] = offd[c];				    // reset delay
      }
    } else {						// new sample was 0:
      if (--(delay[c]) <= 0) {				  // time to change filtered state
	new_state &= ~(1U << c);			    // update state
	delay[c] = -(int)ond[c];			    // reset delay
      }
    }
  }

  return new_state;
}

/* ......................................................................... */
static void tm_efdc_gpio_dio_polled(void *context, u32 di_raw, u32 do_stat)
{
  struct efdcgpio_dev *dev = context;
  u32 di_new, di_old;
  u64 do_status;
  int ldet;
  int evn = 0;

  // combine...
  di_raw ^= dev->gpio.DI_Invert;			// apply invert

  dev->gpio.DO_Raw = dev->do_raw;			// show to user-space too
  dev->gpio.DI_Raw = di_raw;

  // filter inputs:
  di_old = dev->di_last;

  di_new = tm_efdc_gpio_in_filter(dev, TM_EFDC_DIN_COUNT, di_raw, di_old,
				  dev->di_delay, dev->di_on_delay, dev->di_off_delay,
				  dev->gpio.DI_Pulses);
  if (di_new != di_old) {
    dev->di_last = di_new;
    evn |= 1;
  }

  dev->gpio.DI = di_new;

  // was load state detection active:
  ldet = at91_get_gpio_value(EFDC_PIN_LOAD_DET);

  if (!ldet) {						// load detection was active:
    dev->do_float = 0;					// so nothing floats by default..
  }

  // process digital output errors:
  if (!at91_get_gpio_value(EFDC_PIN_DO_FLAG)) {		// some kind of error:
    int c;
    u64 err, err1;
    u32 do_fail = do_stat ^ dev->do_raw;		// mask of failed outputs

    err = dev->do_float;				// current errors

    err1 = ldet ? 0x01 : 0x02;				// error 1 (0x01 = shortcut, 0x02 = floating)

    c = 0; do {
      if (do_fail & (1U << c)) {			// this output has failure:
	err = ((err & ~(3ULL << (c * 2)))		// merge error bits:
	       | ((do_stat & (1U << c) ? err1 : 0x01LLU) << (c * 2)));
      }
    } while (++c < TM_EFDC_DOUT_COUNT);

    do_status = err;					// final status

    if (!ldet) {					// load detection was active:
      dev->do_float = err & 0xAAAAAAAAAAAAAAAALLU;	// save floating outputs
    }
  } else {						// no errors:
    do_status = dev->do_float;				// remember floating outputs
  }

  if (dev->gpio.DO_Status != do_status) {		// DO status changed:
    dev->gpio.DO_Status = do_status;			// show status to applications
    evn |= 2;						// remember that status changed
  }

  // is it time to do load floating detection:
  if (--dev->load_det_tick <= 0) {
    dev->load_det_tick = LOAD_DET_INTERVAL;
    at91_set_gpio_value(EFDC_PIN_LOAD_DET, 0);
  } else {
    at91_set_gpio_value(EFDC_PIN_LOAD_DET, 1);
  }

  // send an event if something important changes:
  if (evn) {						// send event!
    if (evn & 1) {
      atomic_inc(&efdcgpio_dev.din_evn_num);		// increment event counter
    }
    if (evn & 2) {
      atomic_inc(&efdcgpio_dev.dout_status_evn_num);	// increment event counter
    }
    wake_up_interruptible(&tm_efdc_gpio_event_queue);
  }
}

/* ......................................................................... */
static __inline void poll_dio(struct efdcgpio_dev *dev)
{
  int c, res;
  u32 do_raw, do_bits, do_pwm_ena;

  // first build word to transfer, take current PWM bit for all channels
  for (c = TM_EFDC_DOUT_COUNT - 1, do_raw = 0; c >= 0; c--) {	// process all outputs
    do_raw <<= 1;
    do_raw |= (dev->pwm_value[c] >> dev->pwm_bit) & 1;
  }

  if (++dev->pwm_bit >= TM_EFDC_DOUT_PWM_NBITS) {	// wrap PWM bit number
    dev->pwm_bit = 0;
    for (c = 0; c < TM_EFDC_DOUT_COUNT; c++) {		// copy new PWM values
      int p = dev->gpio.DO_PWM_Set[c], mp = dev->gpio.DO_PWM_Min[c];
      if (p < mp) {
	p = mp;
      }
      dev->gpio.DO_PWM[c] = p;
      dev->pwm_value[c] = pwm_table[(((unsigned)p * TM_EFDC_DOUT_PWM_NBITS + 50) / 100)];
    }
  }

  do_bits = dev->gpio.DO;				// get output states
  do_pwm_ena = dev->gpio.DO_PWM_Ena;			// get mask of PWM channels
  do_raw = (do_raw & do_pwm_ena & do_bits) | (do_bits & ~do_pwm_ena);

  dev->do_raw = do_raw;

  // transfer data:
  if ((res = tm_efdc_dio_transfer(do_raw, tm_efdc_gpio_dio_polled, dev)) != 0) {
    //printk("dio: %d\n", res);
  }
}

/* ......................................................................... */
static __inline void poll_aio(struct efdcgpio_dev *dev)
{
  int c;
  int val[TM_EFDC_AOUT_COUNT < TM_EFDC_AIN_COUNT ? TM_EFDC_AIN_COUNT : TM_EFDC_AOUT_COUNT];

  // process analog outputs:
  for (c = 0; c < TM_EFDC_AOUT_COUNT; c++) {
    val[c] = dev->gpio.AO[c];
  }

  // write analog outputs:
  gpio_set_values(TM_EFDC_AOUT_BASE, TM_EFDC_AOUT_COUNT, &val[0]);

  // read analog inputs:
  gpio_get_values(TM_EFDC_AIN_BASE, TM_EFDC_AIN_COUNT, &val[0]);

  // sliding window filtering for analog inputs:
  for (c = 0; c < TM_EFDC_AIN_COUNT; c++) {
    u32 slide = dev->adc_slide[c];
    slide -= (slide + ADC_SLIDE_WIN / 2) / ADC_SLIDE_WIN;
    slide += val[c];
    dev->adc_slide[c] = slide;
    dev->gpio.AI[c] = slide / ADC_SLIDE_WIN;
  }
}

/* ......................................................................... */
static int tm_efdc_gpio_aio_kthread(void *cookie)
{
  struct efdcgpio_dev *dev = cookie;
  int poll_count = dev->aio_poll_count;

  while (!kthread_should_stop()) {
    wait_event(dev->aio_wqh, poll_count != dev->aio_poll_count);
    poll_count = dev->aio_poll_count;
    poll_aio(dev);
  }

  return 0;
}

/* ......................................................................... */
static enum hrtimer_restart tm_efdc_gpio_aio_timer_cb(struct hrtimer *tmr)
{
	struct efdcgpio_dev *dev = &efdcgpio_dev;
	if (dev->gpio.Configured) {
	  dev->aio_poll_count++;
	  wake_up(&dev->aio_wqh);
	}
	hrtimer_add_expires_ns(tmr, AIO_POLL_INT);
	return HRTIMER_RESTART;
}

/* ......................................................................... */
static void tm_efdc_gpio_dio_poll(struct efdcgpio_dev *dev)
{
	u32 portb, portc, pwr_last;

	// get switches and power status:
	portb = at91_sys_read(AT91_PIOB + PIO_PDSR);	// read port B inputs
	portc = at91_sys_read(AT91_PIOC + PIO_PDSR);	// read port C inputs

	// poll digital I/O
	if (dev->gpio.Configured) {
		dev->gpio.PollCount++;
		poll_dio(dev);
	}

	// filter power bits:
	pwr_last = dev->pwr_last;
	dev->pwr_last = tm_efdc_gpio_in_filter(dev, 2, ~(portc >> 13), pwr_last,
					       dev->pwr_delay, pwr_on_delay, pwr_off_delay,
					       NULL);
	dev->gpio.Power = dev->pwr_last;

	if (dev->pwr_last != pwr_last) {
		atomic_inc(&efdcgpio_dev.power_evn_num);	// increment event counter
		//wake_up_interruptible(&tm_efdc_gpio_event_queue);
	}

	// filter switch inputs:
	portb = ((~portb) >> 24) & 0xff;

	if (portb == dev->id_last) {
		if (--dev->id_delay <= 0) {
			dev->gpio.ID = portb;
			dev->id_delay = ID_SWITCH_DELAY;
		}
	} else {
		if (++dev->id_delay > ID_SWITCH_DELAY) {
			dev->id_delay = ID_SWITCH_DELAY;
		}
	}

	dev->id_last = portb;
}

#ifndef	XENOMAI
/* ......................................................................... */
static enum hrtimer_restart tm_efdc_gpio_dio_timer_cb(struct hrtimer *tmr)
{
	struct efdcgpio_dev *dev = &efdcgpio_dev;
	tm_efdc_gpio_dio_poll(dev);
	hrtimer_add_expires_ns(tmr, DIO_POLL_INT);
	return HRTIMER_RESTART;
}
#endif

/* ......................................................................... */
#ifdef	XENOMAI
static void tm_efdc_gpio_dio_task(void *cookie)
{
	struct efdcgpio_dev *dev = cookie;

	for (;;) {
		tm_efdc_gpio_dio_poll(dev);
		rtdm_task_wait_period();
	}
}
#endif


/* ......................................................................... */

/*
 * GPIO device is opened.
 */
static int tm_efdc_gpio_open(struct inode *inode, struct file *file)
{
	int ret;
	efdcgpio_file_private_t *priv = kmalloc(sizeof(*priv), GFP_KERNEL);

	if (priv) {
		ret = nonseekable_open(inode, file);

		if (ret == 0) {
			struct efdcgpio_dev *dev = &efdcgpio_dev;

			priv->din_evn_num = atomic_read(&dev->din_evn_num);
			priv->dout_status_evn_num = atomic_read(&dev->dout_status_evn_num);
			priv->dout_evn_num = atomic_read(&dev->dout_evn_num);
			priv->aout_evn_num = atomic_read(&dev->aout_evn_num);
			priv->power_evn_num = atomic_read(&dev->power_evn_num);

			file->private_data = priv;
		} else {
			kfree(priv);
		}
	} else {
		ret = -ENOMEM;
	}

	return ret;
}

/*
 * Close the GPIO device.
 */
static int tm_efdc_gpio_close(struct inode *inode, struct file *file)
{
	if (file->private_data) {
		kfree(file->private_data);
		file->private_data = NULL;
	}
	return 0;
}


/*
 * Handle commands from user-space.
 */
static __inline long _tm_efdc_gpio_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	struct efdcgpio_dev *dev = &efdcgpio_dev;
	tm_efdc_gpio_t *gpio = &dev->gpio;
	void __user *argp = (void __user *)arg;
	tm_efdc_gpio_set_t set;
	tm_efdc_gpio_set_2_t set2;
	__u8 u8[32];
	__u32 u32[32];
	__u16 u16[32];
	__u64 u64;
	int i;
	//int __user *p = argp;
	//int new_value, err;

	switch (cmd) {
	case EFDCIOC_GET_DI:
		u32[0] = gpio->DI;
		return copy_to_user(argp, &u32, 4) ? -EFAULT : 0;

	case EFDCIOC_GET_DI_RAW:
		u32[0] = gpio->DI_Raw;
		return copy_to_user(argp, &u32, 4) ? -EFAULT : 0;

	case EFDCIOC_GET_DI_INV:
		u32[0] = gpio->DI_Invert;
		return copy_to_user(argp, &u32, 4) ? -EFAULT : 0;

	case EFDCIOC_SET_DI_INV:
#if 1
		if (copy_from_user(&u32, argp, 4))
			return -EFAULT;
		gpio->DI_Invert = u32[0];
		return 0;
#else
		if (copy_from_user(&set, argp, sizeof(set)))
			return -EFAULT;
		if (set.num < 0 || set.num >= TM_EFDC_DIN_COUNT || set.value < 0 || set.value > 1)
			return -EINVAL;
		if (set.value)
			gpio->DI_Invert |= (1U << set.num);
		else
			gpio->DI_Invert &= ~(1U << set.num);
		return 0;
#endif

	case EFDCIOC_GET_DI_PULSES:
		for (i = 0; i < TM_EFDC_DIN_COUNT; i++) {
			u32[i] = gpio->DI_Pulses[i];
		}
		return copy_to_user(argp, &u32, 4 * i) ? -EFAULT : 0;

	case EFDCIOC_GET_DI_ONDELAY:
		for (i = 0; i < TM_EFDC_DIN_COUNT; i++) {
			u16[i] = gpio->DI_OnDelay[i];
		}
		return copy_to_user(argp, &u16, 2 * i) ? -EFAULT : 0;

	case EFDCIOC_GET_DI_OFFDELAY:
		for (i = 0; i < TM_EFDC_DIN_COUNT; i++) {
			u16[i] = gpio->DI_OffDelay[i];
		}
		return copy_to_user(argp, &u16, 2 * i) ? -EFAULT : 0;

	case EFDCIOC_SET_DI_ONDELAY:
	case EFDCIOC_SET_DI_OFFDELAY:
		if (copy_from_user(&set, argp, sizeof(set)))
			return -EFAULT;
		if (set.num < 0 || set.num >= TM_EFDC_DIN_COUNT || set.value < 0 || set.value > 65535)
			return -EINVAL;
		if (cmd == EFDCIOC_SET_DI_ONDELAY) {
			gpio->DI_OnDelay[set.num] = set.value;
			dev->di_on_delay[set.num] = MS2DIOTCK(set.value);
		} else {
			gpio->DI_OffDelay[set.num] = set.value;
			dev->di_off_delay[set.num] = MS2DIOTCK(set.value);
		}
		return 0;

	case EFDCIOC_GET_ID:
		u16[0] = gpio->ID;
		return copy_to_user(argp, &u16, 2) ? -EFAULT : 0;

	case EFDCIOC_GET_POWER:
		u16[0] = gpio->Power;
		return copy_to_user(argp, &u16, 2) ? -EFAULT : 0;

	case EFDCIOC_GET_DO:
		u32[0] = gpio->DO;
		return copy_to_user(argp, &u32, 4) ? -EFAULT : 0;

	case EFDCIOC_GET_DO_STATUS:
		u64 = gpio->DO_Status;
		return copy_to_user(argp, &u64, 8) ? -EFAULT : 0;

	case EFDCIOC_SET_DO:
		if (copy_from_user(&set, argp, sizeof(set)))
			return -EFAULT;
		if (set.num < 0 || set.num >= TM_EFDC_DOUT_COUNT || set.value < 0 || set.value > 1)
			return -EINVAL;
		u32[0] = u32[1] = gpio->DO;

		if (set.value)
			u32[1] |= (1U << set.num);
		else
			u32[1] &= ~(1U << set.num);

		if (u32[1] != u32[0]) {
			gpio->DO = u32[1];
			atomic_inc(&efdcgpio_dev.dout_evn_num);			// increment event counter
			wake_up_interruptible(&tm_efdc_gpio_event_queue);
		}
		return 0;

	case EFDCIOC_SET_DO_2:
		if (copy_from_user(&set2, argp, sizeof(set2)))
			return -EFAULT;
		u32[0] = gpio->DO;
		u32[1] = (u32[0] & ~set2.mask) | (set2.states & set2.mask);
		if (u32[1] != u32[0]) {
			gpio->DO = u32[1];
			atomic_inc(&efdcgpio_dev.dout_evn_num);			// increment event counter
			wake_up_interruptible(&tm_efdc_gpio_event_queue);
		}
		return 0;

	case EFDCIOC_GET_DO_RAW:
		u32[0] = gpio->DO_Raw;
		return copy_to_user(argp, &u32, 4) ? -EFAULT : 0;

	case EFDCIOC_GET_DO_PWM_ENA:
		u32[0] = gpio->DO_PWM_Ena;
		return copy_to_user(argp, &u32, 4) ? -EFAULT : 0;

	case EFDCIOC_SET_DO_PWM_ENA:
		if (copy_from_user(&set, argp, sizeof(set)))
			return -EFAULT;
		if (set.num < 0 || set.num >= TM_EFDC_DIN_COUNT || set.value < 0 || set.value > 1)
			return -EINVAL;
		if (set.value)
			gpio->DO_PWM_Ena |= (1U << set.num);
		else
			gpio->DO_PWM_Ena &= ~(1U << set.num);
		return 0;

	case EFDCIOC_SET_DO_PWM_ENA_2:
		if (copy_from_user(&set2, argp, sizeof(set2)))
			return -EFAULT;
		gpio->DO_PWM_Ena = (gpio->DO_PWM_Ena & ~set2.mask) | (set2.states & set2.mask);
		return 0;

	case EFDCIOC_GET_DO_PWM:
		if (copy_from_user(&set, argp, sizeof(set)))
			return -EFAULT;
		if (set.num < 0 || set.num >= TM_EFDC_DOUT_COUNT)
			return -EINVAL;
		set.value = gpio->DO_PWM[set.num];
		return copy_to_user(argp, &set, sizeof(set)) ? -EFAULT : 0;

	case EFDCIOC_GET_DO_PWM_MIN:
		for (i = 0; i < TM_EFDC_DOUT_COUNT; i++) {
			u8[i] = gpio->DO_PWM_Min[i];
		}
		return copy_to_user(argp, &u8, i) ? -EFAULT : 0;

	case EFDCIOC_SET_DO_PWM:
	case EFDCIOC_SET_DO_PWM_MIN:
		if (copy_from_user(&set, argp, sizeof(set)))
			return -EFAULT;
		if (set.num < 0 || set.num >= TM_EFDC_DOUT_COUNT || set.value < 0 || set.value > 100)
			return -EINVAL;
		if (cmd == EFDCIOC_SET_DO_PWM) {
			gpio->DO_PWM_Set[set.num] = set.value;
			gpio->DO_PWM[set.num] = set.value;
		} else
			gpio->DO_PWM_Min[set.num] = set.value;
		return 0;

	case EFDCIOC_GET_AO:
		if (copy_from_user(&set, argp, sizeof(set)))
			return -EFAULT;
		if (set.num < 0 || set.num >= TM_EFDC_AOUT_COUNT)
			return -EINVAL;
		set.value = gpio->AO[set.num];
		return copy_to_user(argp, &set, sizeof(set)) ? -EFAULT : 0;

	case EFDCIOC_SET_AO:
		if (copy_from_user(&set, argp, sizeof(set)))
			return -EFAULT;
		if (set.num < 0 || set.num >= TM_EFDC_AOUT_COUNT || set.value < 0 || set.value > 65535)
			return -EINVAL;
		if (gpio->AO[set.num] != set.value) {
			gpio->AO[set.num] = set.value;
			atomic_inc(&efdcgpio_dev.aout_evn_num);			// increment event counter
			wake_up_interruptible(&tm_efdc_gpio_event_queue);
		}
		return 0;

	case EFDCIOC_GET_AI:
		if (copy_from_user(&set, argp, sizeof(set)))
			return -EFAULT;
		if (set.num < 0 || set.num >= TM_EFDC_AIN_COUNT)
			return -EINVAL;
		set.value = gpio->AI[set.num];
		return copy_to_user(argp, &set, sizeof(set)) ? -EFAULT : 0;

	case EFDCIOC_GET_CONFIGURED:
		i = gpio->Configured;
		return copy_to_user(argp, &i, sizeof(i)) ? -EFAULT : 0;

	case EFDCIOC_SET_CONFIGURED:
		if (copy_from_user(&i, argp, sizeof(i)))
			return -EFAULT;
		gpio->Configured = i != 0;
		return 0;

	default:
		return -ENOTTY;
	}
}

static long tm_efdc_gpio_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	long ret;

	mutex_lock(&efdc_gpio_mutex);

	ret = _tm_efdc_gpio_ioctl(file, cmd, arg);

	mutex_unlock(&efdc_gpio_mutex);

	return ret;
}


/*
 * Read an event:
 */
static ssize_t tm_efdc_gpio_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	struct efdcgpio_dev *dev = &efdcgpio_dev;
	efdcgpio_file_private_t *priv = file->private_data;
	int ret = 0;
	u8 evn;

	if (count >= sizeof(evn)) {
		int evn_num;
		evn = 0;

		if (priv->din_evn_num != (evn_num = atomic_read(&dev->din_evn_num))) {
			priv->din_evn_num = evn_num;
			evn |= EFDC_EVN_DIN;
		}
		if (priv->dout_evn_num != (evn_num = atomic_read(&dev->dout_evn_num))) {
			priv->dout_evn_num = evn_num;
			evn |= EFDC_EVN_DOUT;
		}
		if (priv->dout_status_evn_num != (evn_num = atomic_read(&dev->dout_status_evn_num))) {
			priv->dout_status_evn_num = evn_num;
			evn |= EFDC_EVN_DOUT_S;
		}
		if (priv->aout_evn_num != (evn_num = atomic_read(&dev->aout_evn_num))) {
			priv->aout_evn_num = evn_num;
			evn |= EFDC_EVN_AOUT;
		}

		if (priv->power_evn_num != (evn_num = atomic_read(&dev->power_evn_num))) {
			priv->power_evn_num = evn_num;
			evn |= EFDC_EVN_POWER;
		}

		if (evn) {
			ret = copy_to_user(buf, &evn, sizeof(evn)) ? -EFAULT : sizeof(evn);
		}
	}

	return ret;
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
#if 0
static int tm_efdc_gpio_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret;

	if ((offset + size) > PAGE_ALIGN(sizeof(*efdcgpio_dev.gpio))) {
		return -EINVAL;
	}

	ret = remap_pfn_range(vma,
			      vma->vm_start,
			      virt_to_phys((void*)efdcgpio_dev.gpio) >> PAGE_SHIFT,
			      size,
			      vma->vm_page_prot);

	if (ret < 0)
		return ret;

	vma->vm_flags |= VM_RESERVED;   /* avoid to swap out this VMA */

	return 0;
}
#endif

/* ......................................................................... */

static unsigned int tm_efdc_gpio_poll(struct file *file, poll_table *wait)
{
	struct efdcgpio_dev *dev = &efdcgpio_dev;
	efdcgpio_file_private_t *priv = file->private_data;

	poll_wait(file, &tm_efdc_gpio_event_queue, wait);

	if (priv->din_evn_num != atomic_read(&dev->din_evn_num)
	    || priv->dout_status_evn_num != atomic_read(&dev->dout_status_evn_num)
	    || priv->dout_evn_num != atomic_read(&dev->dout_evn_num)
	    || priv->aout_evn_num != atomic_read(&dev->aout_evn_num)) {
		return POLLIN | POLLRDNORM;
	}

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
//	.mmap		= tm_efdc_gpio_mmap,
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
	for (;;) {
		while (gpio > gpio_table[index].base) {
			gpio_free(gpio--);
		}
		if (--index < 0)
			break;
		gpio = gpio_table[index].base + gpio_table[index].count - 1;
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
	struct efdcgpio_dev *dev = &efdcgpio_dev;
	int res;

	if (tm_efdc_gpio_miscdev.parent) {
		printk(KERN_ERR DRIVER_NAME ": already probed!\n");
		return -EBUSY;
	}

	tm_efdc_gpio_miscdev.parent = &pdev->dev;

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

	// configure I/O-pins
	at91_set_gpio_output(EFDC_PIN_LOAD_DET, 1);
	at91_set_gpio_input(EFDC_PIN_DO_FLAG, 1);

#ifndef	XENOMAI
	// prepare dio hrtimer
	hrtimer_init(&dev->dio_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	dev->dio_timer.function = tm_efdc_gpio_dio_timer_cb;
#endif

	// prepare aio hrtimer
	hrtimer_init(&dev->aio_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	dev->aio_timer.function = tm_efdc_gpio_aio_timer_cb;

	// create kthread for polling analog I/O:
	init_waitqueue_head(&dev->aio_wqh);

	dev->aio_kthread = kthread_create(tm_efdc_gpio_aio_kthread, &efdcgpio_dev, "efdcgpiod");
	if (IS_ERR(dev->aio_kthread)) {
		int ret = PTR_ERR(dev->aio_kthread);
		printk(KERN_ERR DRIVER_NAME ": kthread_create() failed: %d\n", ret);
		return ret;
	}
	dev->aio_kthread->prio = 5; //MAX_RT_PRIO - 5;
  
	// start ticking
#ifdef	XENOMAI
	rtdm_task_init(&dev->dio_task, "efdcgpio-dio", tm_efdc_gpio_dio_task, &efdcgpio_dev, 90, DIO_POLL_INT);
#else
	hrtimer_start(&dev->dio_timer, ktime_set(0, DIO_POLL_INT), HRTIMER_MODE_REL);
#endif
	hrtimer_start(&dev->aio_timer, ktime_set(0, AIO_POLL_INT), HRTIMER_MODE_REL);

	wake_up_process(dev->aio_kthread);

	printk(KERN_INFO DRIVER_NAME ": started succesfully\n");

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
