/**
 * drivers/gpio/max4896.c
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
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/max4896.h>
#include <linux/gpio.h>

#define DRIVER_NAME "max4896"


/*
 * Some registers must be read back to modify.
 * To save time we cache them here in memory
 */
struct max4896 {
	struct mutex	lock;
	u8		num_chips;	/* number of chips */
	struct gpio_chip chip;
	struct spi_device *spi;
	u8		*out_status;	/* cached output status */
	u8		*out_level;	/* cached output level */
	u8		in_level[];	/* cached input levels */
};

/**
 * max4896_transfer - Transmit data/to from chip(s)
 * @ts: Self pointer
 *
 * A write to the MAX4896 means one message with one transfer
 *
 * Returns 0 if successful or a negative value on error
 */
static int max4896_transfer(struct max4896 *ts)
{
	struct spi_transfer	t = {
		.tx_buf		= ts->out_level - ts->num_chips,	/* first ts->num_chips bytes are ignored */
		.len		= ts->num_chips * 2,	/* number of bytes to transfer */
		.rx_buf		= ts->in_level,		/* first ts->num_chips are inputs and next are status */
	};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(ts->spi, &m);
}

static int max4896_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct max4896 *ts = container_of(chip, struct max4896, chip);
	int ret = -EINVAL;

	/* first 8 bits are for inputs and next 8 bits are for output statuses: */
	if (offset < ts->num_chips * 8 * 2) {
		ret = 0;
	}

	return ret;
}

static int max4896_direction_output(struct gpio_chip *chip, unsigned offset,
				    int value)
{
	struct max4896 *ts = container_of(chip, struct max4896, chip);
	int ret = -EINVAL;

	if (offset >= ts->num_chips * 8 * 2) {
		ret = 0;
	}

	return ret;
}

static int max4896_get(struct gpio_chip *chip, unsigned offset)
{
	struct max4896 *ts = container_of(chip, struct max4896, chip);
	int level = -EINVAL;

	mutex_lock(&ts->lock);

	if (offset < ts->num_chips * 8) {
		/* inputs */
		max4896_transfer(ts);
		level = (ts->in_level[offset / 8] & (1U << (offset % 8))) ? INT_MAX : 0;
	} else if (offset < ts->num_chips * 8 * 2) {
		/* output status */
		offset -= ts->num_chips * 8;
		level = (ts->out_status[offset / 8] & (1U << (offset % 8))) ? INT_MAX : 0;
	} else {
	  	/* outputs */
		offset -= ts->num_chips * 8 * 2;
		level = (ts->out_level[offset / 8] & (1U << (offset % 8))) ? INT_MAX : 0;
	}

	mutex_unlock(&ts->lock);

	return level;
}

static int max4896_get_multiple(struct gpio_chip *chip, unsigned offset, unsigned count, int *values)
{
	struct max4896 *ts = container_of(chip, struct max4896, chip);

	mutex_lock(&ts->lock);

	if (offset < ts->num_chips * 8 * 2) {
		/* inputs or output status, do transfer */
		max4896_transfer(ts);
	}

	/* inputs: */
	while (count > 0 && offset < ts->num_chips * 8) {
		*values++ = (ts->in_level[offset / 8] & (1U << (offset % 8))) ? INT_MAX : 0;
		offset++; count--;
	}

	/* output status: */
	offset -= ts->num_chips * 8;
	while (count > 0 && offset < ts->num_chips * 8) {
		*values++ = (ts->out_status[offset / 8] & (1U << (offset % 8))) ? INT_MAX : 0;
		offset++; count--;
	}

	/* outputs: */
	offset -= ts->num_chips * 8;
	while (count > 0 && offset < ts->num_chips * 8) {
		*values++ = (ts->out_level[offset / 8] & (1U << (offset % 8))) ? INT_MAX : 0;
		offset++; count--;
	}

	mutex_unlock(&ts->lock);

	return 0;
}

static void max4896_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct max4896 *ts = container_of(chip, struct max4896, chip);

	if (offset >= ts->num_chips * 8 * 2) {
		offset -= ts->num_chips * 8 * 2;

		mutex_lock(&ts->lock);

		if (value) {
		  ts->out_level[offset / 8] |= 1U << (offset % 8);
		} else {
		  ts->out_level[offset / 8] &= ~(1U << (offset % 8));
		}

		max4896_transfer(ts);

		mutex_unlock(&ts->lock);
	}
}

static void max4896_set_multiple(struct gpio_chip *chip, unsigned offset, unsigned count, const int *value)
{
	struct max4896 *ts = container_of(chip, struct max4896, chip);

	if (offset >= ts->num_chips * 8 * 2) {
		offset -= ts->num_chips * 8 * 2;

		if ((offset + count) > ts->num_chips * 8) {
			BUG();
		}

		mutex_lock(&ts->lock);

		while (count-- > 0) {
			if (*value++)
				ts->out_level[offset / 8] |= 1U << (offset % 8);
			else
				ts->out_level[offset / 8] &= ~(1U << (offset % 8));
		}

		/*max4896_transfer(ts);*/	// @@@ kludge, assume get_multiple gets called for inputs @@@

		mutex_unlock(&ts->lock);
	}
}

static int __devinit max4896_probe(struct spi_device *spi)
{
	struct max4896 *ts;
	struct max4896_platform_data *pdata;
	int ret;

	pdata = spi->dev.platform_data;
	if (!pdata || !pdata->base || !pdata->chips) {
		dev_dbg(&spi->dev, "incorrect or missing platform data\n");
		return -EINVAL;
	}

	/*
	 * bits_per_word cannot be configured in platform data
	 */
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	ts = kzalloc(sizeof(struct max4896) + sizeof(u8) * pdata->chips * 3, GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	mutex_init(&ts->lock);
	ts->out_status = &ts->in_level[pdata->chips * 1];
	ts->out_level = &ts->out_status[pdata->chips * 2];

	dev_set_drvdata(&spi->dev, ts);

	ts->spi = spi;
	ts->num_chips = pdata->chips;

	ts->chip.label = DRIVER_NAME,

	ts->chip.direction_input = max4896_direction_input;
	ts->chip.get = max4896_get;
	ts->chip.get_multiple = max4896_get_multiple;
	ts->chip.direction_output = max4896_direction_output;
	ts->chip.set = max4896_set;
	ts->chip.set_multiple = max4896_set_multiple;

	ts->chip.base = pdata->base;
	ts->chip.ngpio = pdata->chips * 8 * 3;
	ts->chip.can_sleep = 1;
	ts->chip.dev = &spi->dev;
	ts->chip.owner = THIS_MODULE;

	ret = gpiochip_add(&ts->chip);
	if (ret)
		goto exit_destroy;

	return ret;

exit_destroy:
	dev_set_drvdata(&spi->dev, NULL);
	mutex_destroy(&ts->lock);
	kfree(ts);
	return ret;
}

static int max4896_remove(struct spi_device *spi)
{
	struct max4896 *ts;
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

static struct spi_driver max4896_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= max4896_probe,
	.remove		= __devexit_p(max4896_remove),
};

static int __init max4896_init(void)
{
	return spi_register_driver(&max4896_driver);
}
/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(max4896_init);

static void __exit max4896_exit(void)
{
	spi_unregister_driver(&max4896_driver);
}
module_exit(max4896_exit);

MODULE_AUTHOR("Sami Kantoluoto");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX4896 SPI based GPIO-Expander");
