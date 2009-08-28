
#define	MC74X595_MAX_CHIPS	8

struct mc74x595_platform_data {
	u8	nchips;			/* number of chips (1 - 8)	*/
	u8	inversion[MC74X595_MAX_CHIPS];	/* output inversion bits*/
	u8	initial[MC74X595_MAX_CHIPS];	/* initial state (logical) */

	/* "base" is the number of the first GPIO.  Dynamic assignment is
	 * not currently supported, and even if there are gaps in chip
	 * addressing the GPIO numbers are sequential .. so for example
	 * if only slaves 0 and 3 are present, their GPIOs range from
	 * base to base+15.
	 */
	unsigned	base;
};
