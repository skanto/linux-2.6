#ifndef LINUX_SPI_AD5666_H
#define LINUX_SPI_AD5666_H

#define	AD5666_NUM_CHANNELS	4	/* number of channels */

struct ad5666_platform_data {
	unsigned	base;		/* number assigned to first GPIO */
	unsigned	ref:1;		/* 1 = use internal reference */
	unsigned	pwr_mask;	/* mask of channels to power on */
};

#endif
