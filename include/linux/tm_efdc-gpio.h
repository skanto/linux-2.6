#ifndef	_tm_efdc_gpio_h_
#define	_tm_efdc_gpio_h_

#include <linux/ioctl.h>

// structure for TM-EFDC memory mapped I/O:

#define	TM_EFDC_DIN_COUNT	32		// amount of digital inputs
#define	TM_EFDC_SW_COUNT	10		// state of switches
#define	TM_EFDC_DOUT_COUNT	32		// amount of digital outputs
#define	TM_EFDC_DOUT_PWM_NBITS	32		// amount of bits in PWM value
#define	TM_EFDC_AIN_COUNT	8		// amount of analog inputs
#define	TM_EFDC_AOUT_COUNT	8		// amount of analog outputs

typedef volatile struct tm_efdc_gpio {
  __u32		DI;				// state of digital inputs (R/)
  __u32		DI_Invert;			// bitmask of input pins to invert (R/W)
  __u32		DI_Pulses[TM_EFDC_DIN_COUNT];	// number of pulses detected in inputs (R/)
  __u16		DI_OnDelay[TM_EFDC_DIN_COUNT];	// input active delay in ms (R/W)
  __u16		DI_OffDelay[TM_EFDC_DIN_COUNT];	// input passive delay in ms (R/W)
  __u16		ID;				// state of ID switches (R/)
  __u16		Power;				// power flags
  __u32		DI_Raw;				// raw (non-filtered) state of digital inputs (R/)
  __u32		DO;				// state of digital outputs (R/W)
						//  bit == 0: corresponding channel is always zero
						//  bit == 1: corresponding channel is one or
						//	      PWM bits are driven to channel if
						//            corresponding DO_PWM_Ena bit is set
  __u32		DO_Raw;				// raw state of digital outputs (R/)
  __u32		DO_PWM_Ena;			// Bit for each mask of digital outputs with PWM enabled (R/W)
  __u64		DO_Status;			// status of outputs
  __u8		DO_PWM[TM_EFDC_DOUT_COUNT];	// PWM-values (percent) (R)
  __u8		DO_PWM_Set[TM_EFDC_DOUT_COUNT];	// PWM value to set (percent) (W)
  __u8		DO_PWM_Min[TM_EFDC_DOUT_COUNT];	// minimum PWM-values (percent) (R/W)
  __u16		AO[TM_EFDC_AOUT_COUNT];		// Analog Output values (raw)
  __s16		AI[TM_EFDC_AIN_COUNT];		// Analog Input values (raw values but filtered)

  __u32		PollCount;			// Counter gets incremented every poll
  __u8		Configured;			// Application must set this when configured
} tm_efdc_gpio_t;

// I/O control interface:
#define	EFDC_IOCTL_BASE	'E'

typedef struct tm_efdc_gpio_set {
	int	num;		// input/output number
	long	value;		// value
} tm_efdc_gpio_set_t;

typedef struct tm_efdc_gpio_set_2 {
	__u32	mask;		// mask
	__u32	states;		// values
} tm_efdc_gpio_set_2_t;

#define	EFDCIOC_GET_DI		_IOR(EFDC_IOCTL_BASE, 0, __u32)
#define	EFDCIOC_GET_DI_RAW	_IOR(EFDC_IOCTL_BASE, 1, __u32)
#define	EFDCIOC_GET_DI_INV	_IOR(EFDC_IOCTL_BASE, 2, __u32)
#define	EFDCIOC_SET_DI_INV	_IOW(EFDC_IOCTL_BASE, 3, __u32)
#define	EFDCIOC_GET_DI_PULSES	_IOR(EFDC_IOCTL_BASE, 4, __u32[TM_EFDC_DIN_COUNT])
#define	EFDCIOC_GET_DI_ONDELAY	_IOR(EFDC_IOCTL_BASE, 5, __u16[TM_EFDC_DIN_COUNT])
#define	EFDCIOC_SET_DI_ONDELAY	_IOW(EFDC_IOCTL_BASE, 6, struct tm_efdc_gpio_set)
#define	EFDCIOC_GET_DI_OFFDELAY	_IOR(EFDC_IOCTL_BASE, 7, __u16[TM_EFDC_DIN_COUNT])
#define	EFDCIOC_SET_DI_OFFDELAY	_IOW(EFDC_IOCTL_BASE, 8, struct tm_efdc_gpio_set)
#define	EFDCIOC_GET_ID		_IOR(EFDC_IOCTL_BASE, 9, __u16)
#define	EFDCIOC_GET_POWER	_IOR(EFDC_IOCTL_BASE, 10, __u16)

#define	EFDCIOC_GET_DO		_IOR(EFDC_IOCTL_BASE, 11, __u32)
#define	EFDCIOC_SET_DO		_IOW(EFDC_IOCTL_BASE, 12, struct tm_efdc_gpio_set)
#define	EFDCIOC_SET_DO_2	_IOW(EFDC_IOCTL_BASE, 13, struct tm_efdc_gpio_set_2)
#define	EFDCIOC_GET_DO_RAW	_IOR(EFDC_IOCTL_BASE, 14, __u32)
#define	EFDCIOC_GET_DO_PWM_ENA	_IOR(EFDC_IOCTL_BASE, 15, __u32)
#define	EFDCIOC_SET_DO_PWM_ENA	_IOW(EFDC_IOCTL_BASE, 16, struct tm_efdc_gpio_set)
#define	EFDCIOC_SET_DO_PWM_ENA_2 _IOW(EFDC_IOCTL_BASE, 17, struct tm_efdc_gpio_set_2)
#define	EFDCIOC_GET_DO_PWM	_IOWR(EFDC_IOCTL_BASE, 18, struct tm_efdc_gpio_set)
#define	EFDCIOC_SET_DO_PWM	_IOW(EFDC_IOCTL_BASE, 19, struct tm_efdc_gpio_set)
#define	EFDCIOC_GET_DO_PWM_MIN	_IOR(EFDC_IOCTL_BASE, 20, __u8[TM_EFDC_DOUT_COUNT])
#define	EFDCIOC_SET_DO_PWM_MIN	_IOW(EFDC_IOCTL_BASE, 21, struct tm_efdc_gpio_set)
#define	EFDCIOC_GET_DO_STATUS	_IOR(EFDC_IOCTL_BASE, 22, __u64)

#define	EFDCIOC_GET_AO		_IOWR(EFDC_IOCTL_BASE, 23, struct tm_efdc_gpio_set)
#define	EFDCIOC_SET_AO		_IOW(EFDC_IOCTL_BASE, 24, struct tm_efdc_gpio_set)
#define	EFDCIOC_GET_AI		_IOWR(EFDC_IOCTL_BASE, 25, struct tm_efdc_gpio_set)

#define	EFDCIOC_GET_CONFIGURED	_IOR(EFDC_IOCTL_BASE, 98, int)
#define	EFDCIOC_SET_CONFIGURED	_IOW(EFDC_IOCTL_BASE, 99, int)

#endif	// !_tm_efdc_gpio_h_
