#ifndef	_tm_efdc_gpio_h_
#define	_tm_efdc_gpio_h_

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
  __u8		DO_PWM[TM_EFDC_DOUT_COUNT];	// PWM-values (percent) (R)
  __u8		DO_PWM_Set[TM_EFDC_DOUT_COUNT];	// PWM value to set (percent) (W)
  __u8		DO_PWM_Min[TM_EFDC_DOUT_COUNT];	// minimum PWM-values (percent) (R/W)
  __u16		AO[TM_EFDC_AOUT_COUNT];		// Analog Output values (raw)
  __s16		AI[TM_EFDC_AIN_COUNT];		// Analog Input values (raw values but filtered)

  __u32		PollCount;			// Counter gets incremented every poll
  __u32		Configured:1;			// Application must set this when configured
} tm_efdc_gpio_t;

#endif	// !_tm_efdc_gpio_h_
