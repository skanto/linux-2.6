/**
 * drivers/gpio/ltc186x.c
 *
 * Copyright (C) 2010 Sami Kantoluoto, Embedtronics Oy
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The Linear Techonology's LTC1863/1867 is an 8-channel
 * 12-/16-bit SPI ADC. 
 * See datasheet for more detals.
 *
 * The driver exports a standard gpiochip interface
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/ltc186x.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#define DRIVER_NAME "ltc186x"


/*
 * Some registers must be read back to modify.
 * To save time we cache them here in memory
 */
struct ltc186x {
	struct mutex	lock;
	struct gpio_chip chip;
	struct spi_device *spi;
	struct spi_message sm;
	struct spi_transfer st;
	u16		*tx_buf;
	u16		*rx_buf;
	unsigned	channels;	/* number of channels */
	u16		value[];	/* cached values */
};

#define	LTC186X_XFER_SIZE	2
#define	LTC186X_XFER_SIZE	2


/**
 * ltc186x_transfer - Transmit data from chip(s)
 * @ts: Self pointer
 *
 * A write to the ltc186x means one message with one transfer
 *
 * Returns 0 if successful or a negative value on error
 */

static __inline int ltc186x_send_cmd(struct ltc186x *ts, u16 cmd, unsigned delay_usecs, u16 *dst)
{
	int ret;

	ts->st.delay_usecs = delay_usecs;		/* copy delay to transfer */
	ts->tx_buf[0] = cmd;
	ret = spi_sync(ts->spi, &ts->sm);

	if (ret >= 0 && dst) {
		*dst = ts->rx_buf[0];
	}

	return ret;
}

static __inline int ltc186x_transfer(struct ltc186x *ts)
{
	static const u16 cmd[LTC186X_NUM_CHANNELS + 1] = {
	  0x8400, 0xC400, 0x9400, 0xD400, 0xA400, 0xE400, 0xB400, 0xF400, 0x8400
	};
	int i;
	u16 *dst = NULL;

	for (i = 0; i <= ts->channels; i++) {
	  ltc186x_send_cmd(ts, cmd[i], i < ts->channels ? 2 : 0, dst);

	  if (i < ts->channels) {		/* still something to convert: */
	    udelay(4);				/* conversion time */
	    dst = &ts->value[i];		/* next transfers returns word for this channel */
	  }
	}

	return 0;
}

static int ltc186x_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static int ltc186x_get(struct gpio_chip *chip, unsigned offset)
{
	struct ltc186x *ts = container_of(chip, struct ltc186x, chip);
	int level = -EINVAL;

	mutex_lock(&ts->lock);

	ltc186x_transfer(ts);
	level = ts->value[offset];

	mutex_unlock(&ts->lock);

	return level;
}

static int ltc186x_get_multiple(struct gpio_chip *chip, unsigned offset, unsigned count, int *values)
{
	struct ltc186x *ts = container_of(chip, struct ltc186x, chip);

	mutex_lock(&ts->lock);

	ltc186x_transfer(ts);

	while (count-- > 0) {
		*values++ = ts->value[offset++];
	}

	mutex_unlock(&ts->lock);

	return 0;
}

static int __devinit ltc186x_setup_transfer(struct ltc186x *ts)
{
	int ret = -ENOMEM;

	ts->tx_buf = kmalloc(LTC186X_XFER_SIZE, GFP_KERNEL);
	if (ts->tx_buf) {
		ts->rx_buf = kmalloc(LTC186X_XFER_SIZE, GFP_KERNEL);
		if (ts->rx_buf)
			ret = 0;
		else {
			kfree(ts->tx_buf);
		}
	}

	if (ret == 0) {
		ts->st.tx_buf = ts->tx_buf;
		ts->st.rx_buf = ts->rx_buf;
		ts->st.len = LTC186X_XFER_SIZE;

		spi_message_init(&ts->sm);
		spi_message_add_tail(&ts->st, &ts->sm);
	}

	return ret;
}

static int __devinit ltc186x_probe(struct spi_device *spi)
{
	struct ltc186x *ts;
	struct ltc186x_platform_data *pdata;
	int ret;

	pdata = spi->dev.platform_data;
	if (!pdata || !pdata->base || pdata->channels < 1 || pdata->channels > LTC186X_NUM_CHANNELS) {
		dev_dbg(&spi->dev, "incorrect or missing platform data\n");
		return -EINVAL;
	}

	/*
	 * bits_per_word cannot be configured in platform data
	 */
	spi->bits_per_word = 16;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	ts = kzalloc(sizeof(struct ltc186x) + sizeof(u16) * pdata->channels, GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ret = ltc186x_setup_transfer(ts);
	if (ret < 0) {
		kfree(ts);
		return ret;
	}

	mutex_init(&ts->lock);

	ts->spi = spi;
	spi_set_drvdata(spi, ts);

	ts->channels = pdata->channels;

	ts->chip.label = DRIVER_NAME;

	ts->chip.direction_input = ltc186x_direction_input;
	ts->chip.get = ltc186x_get;
	ts->chip.get_multiple = ltc186x_get_multiple;

	ts->chip.base = pdata->base;
	ts->chip.ngpio = pdata->channels;
	ts->chip.can_sleep = 1;
	ts->chip.dev = &spi->dev;
	ts->chip.owner = THIS_MODULE;

	ret = gpiochip_add(&ts->chip);
	if (ret)
		goto exit_destroy;

	return ret;

exit_destroy:
	spi_set_drvdata(spi, NULL);
	mutex_destroy(&ts->lock);
	kfree(ts->tx_buf);
	kfree(ts->rx_buf);
	kfree(ts);
	return ret;
}

static int ltc186x_remove(struct spi_device *spi)
{
	struct ltc186x *ts;
	int ret;

	ts = dev_get_drvdata(&spi->dev);
	if (ts == NULL)
		return -ENODEV;

	dev_set_drvdata(&spi->dev, NULL);

	ret = gpiochip_remove(&ts->chip);
	if (!ret) {
		mutex_destroy(&ts->lock);
		kfree(ts);
	} else
		dev_err(&spi->dev, "Failed to remove the GPIO controller: %d\n",
			ret);

	return ret;
}

static struct spi_driver ltc186x_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= ltc186x_probe,
	.remove		= __devexit_p(ltc186x_remove),
};

static int __init ltc186x_init(void)
{
	return spi_register_driver(&ltc186x_driver);
}
/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(ltc186x_init);

static void __exit ltc186x_exit(void)
{
	spi_unregister_driver(&ltc186x_driver);
}
module_exit(ltc186x_exit);

MODULE_AUTHOR("Sami Kantoluoto");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("LTC186x SPI based A/D-converter");
