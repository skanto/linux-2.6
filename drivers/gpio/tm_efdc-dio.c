/**
 * drivers/gpio/tm_efdc-dio.c
 *
 * Copyright (C) 2006 Juergen Beisert, Pengutronix
 * Copyright (C) 2008 Guennadi Liakhovetski, Pengutronix
 * Copyright (C) 2010 Sami Kantoluoto, Embedtronics Oy
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The Maxim's MAX4896 device is an SPI driven GPIO expander. There are
 * 8 GPIOs and chips can be daisy-chained. See datasheet for more
 * details
 * Note:
 * - DIN must be stable at the rising edge of clock.
 * - at DIN: 8 (* amount of chips) don't care followed by D7 first, D0 last (for each chip)
 * - at DOUT: input pins first followed by output status
 *
 * The driver exports a standard gpiochip interface
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>

#define DRIVER_NAME "tm_efdc-dio"


/*
 * Some registers must be read back to modify.
 * To save time we cache them here in memory
 */
struct tm_efdc_dio {
	spinlock_t	lock;

	struct spi_device *spi;
	struct spi_message sm;
	struct spi_transfer st;
	volatile u8	transfer_busy:1;/* set if transmission is busy */
	volatile u32	in_levels;
	volatile u32	out_status;
	volatile u32	dummy;
	volatile u32	out_levels;
	void		(*callback)(void *context, u32 inputs, u32 out_status);
	void		*context;
} *tm_efdc_dio;


static void tm_efdc_dio_spi_complete(void *context)
{
	struct tm_efdc_dio *ts = context;

	u32 inp = be32_to_cpu(ts->in_levels);
	u32 out_stat = be32_to_cpu(ts->out_status);

	out_stat = (out_stat >> 1) | (inp << 31U);
	inp = (inp >> 1) | (ts->sm.first_bit << 31U);

	if (ts->callback)
		(*ts->callback)(ts->context, ~inp, out_stat);

	ts->transfer_busy = 0;
}


/**
 * tm_efdc_dio_transfer - Transmit data/to from chip(s)
 * @dout: digital outputs
 * @complete: completion callback
 * @context: completion callback context pointer
 *
 * A write to the MAX4896 means one message with one transfer
 *
 * Returns 0 if successful or a negative value on error
 */
int tm_efdc_dio_transfer(u32 dout, void (*complete)(void *, u32, u32), void *context)
{
	struct tm_efdc_dio *ts = tm_efdc_dio;
	unsigned long flags;

	if (!ts)
		return -ENXIO;

	spin_lock_irqsave(&ts->lock, flags);

	if (ts->transfer_busy) {
		spin_unlock_irqrestore(&ts->lock, flags);
		return -EBUSY;
	}

	ts->transfer_busy = 1;

	spin_unlock_irqrestore(&ts->lock, flags);

	ts->callback = complete;
	ts->context = context;

	ts->out_levels	= cpu_to_be32(dout);

	memset(&ts->st, 0, sizeof(ts->st));
	ts->st.tx_buf	= (u32*)&ts->out_levels - 1;		/* first 32-bits are ignored */
	ts->st.rx_buf	= (u32*)&ts->in_levels;			/* first ts->num_chips are inputs and next are status */
	ts->st.len	= 8;					/* number of bytes to transfer */

	spi_message_init(&ts->sm);
	ts->sm.complete	= tm_efdc_dio_spi_complete;
	ts->sm.context	= ts;
	spi_message_add_tail(&ts->st, &ts->sm);

	return spi_async(ts->spi, &ts->sm);
}

static int __devinit tm_efdc_dio_probe(struct spi_device *spi)
{
	struct tm_efdc_dio *ts;
	int ret;

	if (tm_efdc_dio) {
		return -EBUSY;
	}

	/*
	 * bits_per_word cannot be configured in platform data
	 */
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	ts = kzalloc(sizeof(struct tm_efdc_dio), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	spin_lock_init(&ts->lock);

	dev_set_drvdata(&spi->dev, ts);

	ts->spi = spi;

	tm_efdc_dio = ts;
	dev_info(&spi->dev, "initialized!\n");

	return 0;
}

static int tm_efdc_dio_remove(struct spi_device *spi)
{
	struct tm_efdc_dio *ts;

	ts = dev_get_drvdata(&spi->dev);
	if (ts == NULL)
		return -ENODEV;

	if (ts == tm_efdc_dio)
		tm_efdc_dio = NULL;

	dev_set_drvdata(&spi->dev, NULL);

	kfree(ts);

	return 0;
}

static struct spi_driver tm_efdc_dio_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= tm_efdc_dio_probe,
	.remove		= __devexit_p(tm_efdc_dio_remove),
};

static int __init tm_efdc_dio_init(void)
{
	return spi_register_driver(&tm_efdc_dio_driver);
}

/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(tm_efdc_dio_init);

static void __exit tm_efdc_dio_exit(void)
{
	spi_unregister_driver(&tm_efdc_dio_driver);
}
module_exit(tm_efdc_dio_exit);

MODULE_AUTHOR("Sami Kantoluoto");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TM-EFDC SPI based Digital I/O expander");
