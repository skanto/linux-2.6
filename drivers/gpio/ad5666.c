/**
 * drivers/gpio/ad5666.c
 *
 * Copyright (C) 2010 Sami Kantoluoto, Embedtronics Oy
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The Analog Device's AD5666 is quad 16-bit DAC. See datasheet for more details.
 *
 * The driver exports a standard gpiochip interface.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/ad5666.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#define DRIVER_NAME "ad5666"


/*
 * Some registers must be read back to modify.
 * To save time we cache them here in memory
 */
struct ad5666 {
	struct mutex	lock;
	struct gpio_chip chip;
	struct spi_device *spi;
	u32		pwr_mask:4;	/* channels to power on */
	u32		ref:1;		/* 1 = use internal reference */
	u16		value[4];	/* cached output values */
	struct timespec	cfg_ts;		/* when it's time to reconfigure */
};

/**
 * ad5666_transfer - Transmit data/to from chip(s)
 * @ts: Self pointer
 *
 * A write to the AD5666 means one message with one transfer
 *
 * Returns 0 if successful or a negative value on error
 */
static int ad5666_transfer(struct ad5666 *ts, u32 word)
{
	u32 w = cpu_to_be32(word);
	return spi_write(ts->spi, (const u8 *)&w, 4);
}

static __inline int ad5666_wr_chn(struct ad5666 *ts, unsigned chn, u16 value, unsigned upd)
{
	return ad5666_transfer(ts, (upd ? 0x2000000 : 0) | (chn << 20) | (value << 4));
}

static void ad5666_configure(struct ad5666 *ts)
{
	ad5666_transfer(ts, 0x08000000 | ts->ref);
	ad5666_transfer(ts, 0x04000000 | ts->pwr_mask);
}

static int ad5666_direction_output(struct gpio_chip *chip, unsigned offset,
				    int value)
{
	return 0;
}

static int ad5666_get(struct gpio_chip *chip, unsigned offset)
{
	struct ad5666 *ts = container_of(chip, struct ad5666, chip);
	int level = -EINVAL;

	mutex_lock(&ts->lock);

	level = ts->value[offset];

	mutex_unlock(&ts->lock);

	return level;
}

static int ad5666_get_multiple(struct gpio_chip *chip, unsigned offset, unsigned count, int *values)
{
	struct ad5666 *ts = container_of(chip, struct ad5666, chip);

	mutex_lock(&ts->lock);

	while (count-- > 0) {
		*values++ = ts->value[offset++];
	}

	mutex_unlock(&ts->lock);

	return 0;
}

static __inline void ad5666_maybe_configure(struct ad5666 *ts)
{
	struct timespec cts;

	ktime_get_ts(&cts);

	if (timespec_compare(&cts, &ts->cfg_ts) >= 0) {
		timespec_add_ns(&cts, 10LLU * 1000000000LLU);
		ts->cfg_ts = cts;
		ad5666_configure(ts);
	}
}

static void ad5666_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct ad5666 *ts = container_of(chip, struct ad5666, chip);

	mutex_lock(&ts->lock);

	ad5666_maybe_configure(ts);

	ts->value[offset] = value;
	ad5666_wr_chn(ts, offset, ts->value[offset], 1);

	mutex_unlock(&ts->lock);
}

static void ad5666_set_multiple(struct gpio_chip *chip, unsigned offset, unsigned count, const int *values)
{
	struct ad5666 *ts = container_of(chip, struct ad5666, chip);

	mutex_lock(&ts->lock);

	ad5666_maybe_configure(ts);

	while (count-- > 0) {
		ts->value[offset] = *values++;
		ad5666_wr_chn(ts, offset, ts->value[offset], count == 0);
		offset++;
	}

	mutex_unlock(&ts->lock);
}

static int __devinit ad5666_probe(struct spi_device *spi)
{
	struct ad5666 *ts;
	struct ad5666_platform_data *pdata;
	int ret, c;

	pdata = spi->dev.platform_data;
	if (!pdata || !pdata->base || !(pdata->pwr_mask & 15) || (pdata->pwr_mask & ~15)) {
		dev_dbg(&spi->dev, "incorrect or missing platform data\n");
		return -EINVAL;
	}

	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	ts = kzalloc(sizeof(struct ad5666), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	mutex_init(&ts->lock);

	dev_set_drvdata(&spi->dev, ts);

	ts->spi = spi;
	ts->ref = pdata->ref;
	ts->pwr_mask = pdata->pwr_mask;
	ktime_get_ts(&ts->cfg_ts);
	timespec_add_ns(&ts->cfg_ts, 10LLU * 1000000000LLU);

	ts->chip.label = DRIVER_NAME;

	ts->chip.get = ad5666_get;
	ts->chip.get_multiple = ad5666_get_multiple;
	ts->chip.direction_output = ad5666_direction_output;
	ts->chip.set = ad5666_set;
	ts->chip.set_multiple = ad5666_set_multiple;

	ts->chip.base = pdata->base;
	ts->chip.ngpio = AD5666_NUM_CHANNELS;
	ts->chip.can_sleep = 1;
	ts->chip.dev = &spi->dev;
	ts->chip.owner = THIS_MODULE;

	ret = gpiochip_add(&ts->chip);
	if (ret)
		goto exit_destroy;

	mutex_lock(&ts->lock);

	/* reset the dac */
	ad5666_transfer(ts, 0x07000000);		/* reset DAC */
	ad5666_transfer(ts, 0x07000000);		/* reset DAC again, just in case */

	msleep(10);					/* small delay after reset */

	for (c = 0; c < 4; c++) {			/* configure four times, just in case */
		ad5666_configure(ts);
		msleep(1);
	}

	mutex_unlock(&ts->lock);


	return ret;

exit_destroy:
	dev_set_drvdata(&spi->dev, NULL);
	mutex_destroy(&ts->lock);
	kfree(ts);
	return ret;
}

static int ad5666_remove(struct spi_device *spi)
{
	struct ad5666 *ts;
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

static struct spi_driver ad5666_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= ad5666_probe,
	.remove		= __devexit_p(ad5666_remove),
};

static int __init ad5666_init(void)
{
	return spi_register_driver(&ad5666_driver);
}
/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(ad5666_init);

static void __exit ad5666_exit(void)
{
	spi_unregister_driver(&ad5666_driver);
}
module_exit(ad5666_exit);

MODULE_AUTHOR("Sami Kantoluoto");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("AD5666 SPI based GPIO-Expander");
