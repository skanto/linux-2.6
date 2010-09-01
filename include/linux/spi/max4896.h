#ifndef LINUX_SPI_MAX4896_H
#define LINUX_SPI_MAX4896_H

struct max4896_platform_data {
	/* number assigned to the first GPIO */
	unsigned	base;
	/* amount of daisy-chained chips: */
	unsigned	chips;
};

#endif
