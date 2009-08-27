/*
 * mc74x595.c - SPI gpio expander driver
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

#include <linux/spi/spi.h>
#include <linux/spi/mc74x595.h>

#include <asm/gpio.h>


/* Registers are all 8 bits wide. Multiple drivers can be daisy
 * chained.
 *
 */

/* A given spi_device can represent up to 8 mc74x595 chips
 * sharing the same chipselect but using different addresses
 * (e.g. chips #0 and #3 might be populated, but not #1 or $2).
 * Driver data holds all the per-chip data.
 */
struct mc74x595 {
	struct spi_device	*spi;

	struct mutex		lock;

	struct gpio_chip	chip;

	struct work_struct	work;

	unsigned		nchips;
	u8			cache[MC74X595_MAX_CHIPS];
	u8			inversion[MC74X595_MAX_CHIPS];
};


static int
mc74x595_write(struct mc74x595 *mcp, const u8 *vals, unsigned n)
{
	if (n > sizeof mcp->cache)
		return -EINVAL;
	return spi_write(mcp->spi, vals, n);
}

#if 0
static int
mc74x595_read(struct mc74x595 *mcp, u8 *vals, unsigned n)
{
	if (n > sizeof mcp->cache)
		return -EINVAL;
	return spi_read(mcp->spi, vals, n);
}
#endif

/*----------------------------------------------------------------------*/

static int mc74x595_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return -EINVAL;
}

static int mc74x595_get(struct gpio_chip *chip, unsigned offset)
{
	struct mc74x595	*mcp = container_of(chip, struct mc74x595, chip);
	int status;

	mutex_lock(&mcp->lock);

	status = !!((mcp->cache[offset / 8] ^ mcp->inversion[offset / 8]) & (1 << (offset % 8)));

	mutex_unlock(&mcp->lock);

	return status;
}

static void mc74x595_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mc74x595	*mcp = container_of(chip, struct mc74x595, chip);
	u8 mask = 1 << (offset % 8);

	mutex_lock(&mcp->lock);
	if ((value != 0) ^ ((mcp->inversion[offset / 8] & mask) != 0))
		mcp->cache[offset / 8] |= mask;
	else
		mcp->cache[offset / 8] &= ~mask;

	mc74x595_write(mcp, mcp->cache, mcp->nchips);
	mutex_unlock(&mcp->lock);
}

static int
mc74x595_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	return 0;
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_DEBUG_FS

#include <linux/seq_file.h>

/*
 * This shows more info than the generic gpio dump code:
 * pullups, deglitching, open drain drive.
 */
static void mc74x595_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct mc74x595	*mcp;
	int		t;
	unsigned	mask;

	mcp = container_of(chip, struct mc74x595, chip);

	mutex_lock(&mcp->lock);

  	for (t = 0; t < mcp->nchips * 8; t++)
		seq_printf(s, " %s%d %s out\n",
			   chip->name, t, (mcp->cache[t / 8] & (1 << (t % 8))) ? "hi" : "lo");

	mutex_unlock(&mcp->lock);
}

#else
#define mc74x595_dbg_show	NULL
#endif

/*----------------------------------------------------------------------*/

#if 0
static int mc74x595_probe_one(struct spi_device *spi, unsigned addr,
		unsigned base, unsigned pullups)
{
	struct mc74x595_driver_data	*data = spi_get_drvdata(spi);
	struct mc74x595			*mcp = data->mcp[addr];
	int				status;
	int				do_update = 0;

	mutex_init(&mcp->lock);

	mcp->spi = spi;
	mcp->addr = 0x40 | (addr << 1);

	mcp->chip.label = "mc74x595",

	mcp->chip.direction_input = mc74x595_direction_input;
	mcp->chip.get = mc74x595_get;
	mcp->chip.direction_output = mc74x595_direction_output;
	mcp->chip.set = mc74x595_set;
	mcp->chip.dbg_show = mc74x595_dbg_show;

	mcp->chip.base = base;
	mcp->chip.ngpio = 8;
	mcp->chip.can_sleep = 1;
	mcp->chip.dev = &spi->dev;
	mcp->chip.owner = THIS_MODULE;

	/* verify MCP_IOCON.SEQOP = 0, so sequential reads work,
	 * and MCP_IOCON.HAEN = 1, so we work with all chips.
	 */
	status = mc74x595_read(mcp, MCP_IOCON);
	if (status < 0)
		goto fail;
	if ((status & IOCON_SEQOP) || !(status & IOCON_HAEN)) {
		status &= ~IOCON_SEQOP;
		status |= IOCON_HAEN;
		status = mc74x595_write(mcp, MCP_IOCON, (u8) status);
		if (status < 0)
			goto fail;
	}

	/* configure ~100K pullups */
	status = mc74x595_write(mcp, MCP_GPPU, pullups);
	if (status < 0)
		goto fail;

	status = mc74x595_read_regs(mcp, 0, mcp->cache, sizeof mcp->cache);
	if (status < 0)
		goto fail;

	/* disable inverter on input */
	if (mcp->cache[MCP_IPOL] != 0) {
		mcp->cache[MCP_IPOL] = 0;
		do_update = 1;
	}

	/* disable irqs */
	if (mcp->cache[MCP_GPINTEN] != 0) {
		mcp->cache[MCP_GPINTEN] = 0;
		do_update = 1;
	}

	if (do_update) {
		u8 tx[4];

		tx[0] = mcp->addr;
		tx[1] = MCP_IPOL;
		memcpy(&tx[2], &mcp->cache[MCP_IPOL], sizeof(tx) - 2);
		status = spi_write_then_read(mcp->spi, tx, sizeof tx, NULL, 0);
		if (status < 0)
			goto fail;
	}

	status = gpiochip_add(&mcp->chip);
fail:
	if (status < 0)
		dev_dbg(&spi->dev, "can't setup chip %d, --> %d\n",
				addr, status);
	return status;
}
#endif

static int mc74x595_probe(struct spi_device *spi)
{
	struct mc74x595_platform_data	*pdata;
	struct mc74x595			*mcp;
	int				status;
	unsigned			base;

	pdata = spi->dev.platform_data;
	if (!pdata || !gpio_is_valid(pdata->base))
		return -ENODEV;
	if (pdata->nchips < 1 || pdata->nchips > MC74X595_MAX_CHIPS)
		return -EINVAL;

	mcp = kzalloc(sizeof *mcp, GFP_KERNEL);
	if (!mcp)
		return -ENOMEM;

	spi_set_drvdata(spi, mcp);

	base = pdata->base;
	mutex_init(&mcp->lock);

	mcp->spi = spi;
	mcp->nchips = pdata->nchips;
	memcpy(mcp->cache, pdata->inversion, sizeof mcp->cache);
	memcpy(mcp->inversion, pdata->inversion, sizeof mcp->inversion);

	mcp->chip.label = "mc74x595";

	mcp->chip.direction_input = mc74x595_direction_input;
	mcp->chip.get = mc74x595_get;
	mcp->chip.direction_output = mc74x595_direction_output;
	mcp->chip.set = mc74x595_set;
	mcp->chip.dbg_show = mc74x595_dbg_show;

	mcp->chip.base = base;
	mcp->chip.ngpio = pdata->nchips * 8;
	mcp->chip.can_sleep = 1;
	mcp->chip.dev = &spi->dev;
	mcp->chip.owner = THIS_MODULE;

	status = gpiochip_add(&mcp->chip);

	if (status == 0) {
		mutex_lock(&mcp->lock);
		mc74x595_write(mcp, mcp->cache, mcp->nchips);
		mutex_unlock(&mcp->lock);
	} else
		kfree(mcp);

	return status;
}

static int mc74x595_remove(struct spi_device *spi)
{
	struct mc74x595			*mcp = spi_get_drvdata(spi);
	int				status;

	status = gpiochip_remove(&mcp->chip);

	if (status < 0) {
		dev_err(&spi->dev, "%s --> %d\n", "remove", status);
	}

	if (status == 0)
		kfree(mcp);

	return status;
}

static struct spi_driver mc74x595_driver = {
	.probe		= mc74x595_probe,
	.remove		= mc74x595_remove,
	.driver = {
		.name	= "mc74x595",
		.owner	= THIS_MODULE,
	},
};

/*----------------------------------------------------------------------*/

static int __init mc74x595_init(void)
{
	return spi_register_driver(&mc74x595_driver);
}
module_init(mc74x595_init);

static void __exit mc74x595_exit(void)
{
	spi_unregister_driver(&mc74x595_driver);
}
module_exit(mc74x595_exit);

MODULE_LICENSE("GPL");
