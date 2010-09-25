#ifndef LINUX_SPI_LTC186X_H
#define LINUX_SPI_LTC186X_H

#define	LTC186X_NUM_CHANNELS	8	/* amount of bipolar channels */

struct ltc186x_platform_data {
	unsigned	base;		/* number assigned to the first GPIO */
	unsigned	channels;	/* number of channels [1-8] */
};

#endif
