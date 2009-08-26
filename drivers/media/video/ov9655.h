/*
 * drivers/media/video/ov9655.h
 *
 * Register definitions for the OmniVision OV9655 CameraChip.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OV9655_H
#define OV9655_H

#define OV9655_I2C_ADDR		0x30

/* define register offsets for the OV9655 sensor chip */
#define OV9655_GAIN		0x00
#define OV9655_BLUE		0x01
#define OV9655_RED		0x02
#define OV9655_VREF		0x03
#define OV9655_COM1		0x04
#define OV9655_BAVE		0x05
#define OV9655_GbAVE		0x06
#define OV9655_GrAVE		0x07
#define OV9655_RAVE		0x08
#define OV9655_COM2		0x09
#define OV9655_PID		0x0A
#define OV9655_VER		0x0B
#define OV9655_COM3		0x0C
#define OV9655_COM4		0x0D
#define OV9655_COM5		0x0E
#define OV9655_COM6		0x0F
#define OV9655_AEC		0x10
#define OV9655_CLKRC		0x11
#define OV9655_COM7		0x12
#define OV9655_COM8		0x13
#define OV9655_COM9		0x14
#define OV9655_COM10		0x15
#define OV9655_REG16		0x16
#define OV9655_HSTRT		0x17
#define OV9655_HSTOP		0x18
#define OV9655_VSTRT		0x19
#define OV9655_VSTOP		0x1A
#define OV9655_PSHFT		0x1B
#define OV9655_MIDH		0x1C
#define OV9655_MIDL		0x1D
#define OV9655_MVFP		0x1E
#define OV9655_LAEC		0x1F
#define OV9655_BOS		0x20
#define OV9655_GBOS		0x21
#define OV9655_GROS		0x22
#define OV9655_ROS		0x23
#define OV9655_AEW		0x24
#define OV9655_AEB		0x25
#define OV9655_VPT		0x26
#define OV9655_BBIAS		0x27
#define OV9655_GBBIAS		0x28
#define OV9655_PREGAIN		0x29
#define OV9655_EXHCH		0x2A
#define OV9655_EXHCL		0x2B
#define OV9655_RBIAS		0x2C
#define OV9655_ADVFL		0x2D
#define OV9655_ADVFH		0x2E
#define OV9655_YAVE		0x2F
#define OV9655_HSYST		0x30
#define OV9655_HSYEN		0x31
#define OV9655_HREF		0x32
#define OV9655_CHLF		0x33
#define OV9655_AREF1		0x34
#define OV9655_AREF2		0x35
#define OV9655_AREF3		0x36
#define OV9655_ADC1		0x37
#define OV9655_ADC2		0x38
#define OV9655_AREF4		0x39
#define OV9655_TSLB		0x3A
#define OV9655_COM11		0x3B
#define OV9655_COM12		0x3C
#define OV9655_COM13		0x3D
#define OV9655_COM14		0x3E
#define OV9655_EDGE		0x3F
#define OV9655_COM15		0x40
#define OV9655_COM16		0x41
#define OV9655_COM17		0x42
#define OV9655_MTX1		0x4F
#define OV9655_MTX2		0x50
#define OV9655_MTX3		0x51
#define OV9655_MTX4		0x52
#define OV9655_MTX5		0x53
#define OV9655_MTX6		0x54
#define OV9655_BRTN		0x55
#define OV9655_CNST1		0x56
#define OV9655_CNST2		0x57
#define OV9655_MTXS		0x58
#define OV9655_AWBOP1		0x59
#define OV9655_AWBOP2		0x5A
#define OV9655_AWBOP3		0x5B
#define OV9655_AWBOP4		0x5C
#define OV9655_AWBOP5		0x5D
#define OV9655_AWBOP6		0x5E
#define OV9655_BLMT		0x5F
#define OV9655_RLMT		0x60
#define OV9655_GLMT		0x61
#define OV9655_LCC1		0x62
#define OV9655_LCC2		0x63
#define OV9655_LCC3		0x64
#define OV9655_LCC4		0x65
#define OV9655_LCC5		0x66
#define OV9655_MANU		0x67
#define OV9655_MANV		0x68
#define OV9655_BD50MAX		0x6A
#define OV9655_DBLV		0x6B
#define OV9655_DNSTH		0x70
#define OV9655_POIDX		0x72
#define OV9655_PCKDV		0x73
#define OV9655_XINDX		0x74
#define OV9655_YINDX		0x75
#define OV9655_SLOP		0x7A
#define OV9655_GAM1		0x7B
#define OV9655_GAM2		0x7C
#define OV9655_GAM3		0x7D
#define OV9655_GAM4		0x7E
#define OV9655_GAM5		0x7F
#define OV9655_GAM6		0x80
#define OV9655_GAM7		0x81
#define OV9655_GAM8		0x82
#define OV9655_GAM9		0x83
#define OV9655_GAM10		0x84
#define OV9655_GAM11		0x85
#define OV9655_GAM12		0x86
#define OV9655_GAM13		0x87
#define OV9655_GAM14		0x88
#define OV9655_GAM15		0x89
#define OV9655_COM18		0x8B
#define OV9655_COM19		0x8C
#define OV9655_GAM20		0x8D
#define OV9655_DMLNL		0x92
#define OV9655_DMLNH		0x93
#define OV9655_LCC6		0x9D
#define OV9655_LCC7		0x9E
#define OV9655_AECH		0xA1
#define OV9655_BD50		0xA2
#define OV9655_BD60		0xA3
#define OV9655_COM21		0xA4
#define OV9655_GREEN		0xA6
#define OV9655_VZST		0xA7
#define OV9655_REFA8		0xA8
#define OV9655_REFA9		0xA9
#define OV9655_BLC1		0xAC
#define OV9655_BLC2		0xAD
#define OV9655_BLC3		0xAE
#define OV9655_BLC4		0xAF
#define OV9655_BLC5		0xB0
#define OV9655_BLC6		0xB1
#define OV9655_BLC7		0xB2
#define OV9655_BLC8		0xB3
#define OV9655_CTRLB4		0xB4
#define OV9655_FRSTL		0xB7
#define OV9655_FRSTH		0xB8
#define OV9655_ADBOFF		0xBC
#define OV9655_ADROFF		0xBD
#define OV9655_ADGbOFF		0xBE
#define OV9655_ADGrOFF		0xBF
#define OV9655_COM23		0xC4
#define OV9655_BD60MAX		0xC5
#define OV9655_COM24		0xC7

#define OV9655_NUM_REGS		(OV9655_GST15 + 1)

#define OV9655_PID_MAGIC	0x96	/* high byte of product ID number */
#define OV9655_VER_REV2		0x48	/* low byte of product ID number */
#define OV9655_VER_REV3		0x49	/* low byte of product ID number */
#define OV9655_MIDH_MAGIC	0x7F	/* high byte of mfg ID */
#define OV9655_MIDL_MAGIC	0xA2	/* low byte of mfg ID */

#define OV9655_REG_TERM 0xFF	/* terminating list entry for reg */
#define OV9655_VAL_TERM 0xFF	/* terminating list entry for val */

/*
 * The nominal xclk input frequency of the OV9655 is 24MHz, maximum
 * frequency is 48MHz, and minimum frequency is 10MHz.
 */
#define OV9655_XCLK_MIN 10000000
#define OV9655_XCLK_MAX 48000000
#define OV9655_XCLK_NOM 24000000

/* define a structure for ov9655 register initialization values */
struct ov9655_reg {
	unsigned char reg;
	unsigned char val;
};

enum image_size { QQCIF, QQVGA, QCIF, QVGA, CIF, VGA, SXGA };
enum pixel_format { YUV, RGB565, RGB555 };

#define NUM_IMAGE_SIZES 7
#define NUM_PIXEL_FORMATS 3

struct capture_size {
	unsigned long width;
	unsigned long height;
};

struct ov9655_platform_data {
	/* Set power state, zero is off, non-zero is on. */
	int (*power_set)(int power);
	/* Default registers written after power-on or reset. */
	const struct ov9655_reg *default_regs;
	int (*ifparm)(struct v4l2_ifparm *p);
};

/*
 * Array of image sizes supported by OV9655.  These must be ordered from
 * smallest image size to largest.
 */
const static struct capture_size ov9655_sizes[] = {
	{   88,  72 },	/* QQCIF */
	{  160, 120 },	/* QQVGA */
	{  176, 144 },	/* QCIF */
	{  320, 240 },	/* QVGA */
	{  352, 288 },	/* CIF */
	{  640, 480 },	/* VGA */
	{ 1280, 960 },	/* SXGA */
};

#endif /* ifndef OV9655_H */
