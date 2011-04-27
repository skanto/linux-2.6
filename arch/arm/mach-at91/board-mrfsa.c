/*	$Id: board-mrfsa.c,v 1.4 2009/06/19 19:24:29 skanto Exp $	*/

/*
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
 *  Copyright (C) 2009 Embedtronics OY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/i2c-gpio.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>
#include <asm/gpio.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/serial_at91.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <linux/input.h>
#include <mach/at91sam9_smc.h>

#include <linux/phy.h>
#include <linux/ethtool.h>
#include <linux/phy_fixed.h>

#include "sam9_smc.h"
#include "generic.h"

#include <linux/serial_core.h>

#define	USART_DBGU	0
#define	USART_SER1	AT91SAM9260_ID_US0
#define	USART_SER2	AT91SAM9260_ID_US1
#define	USART_SER3	AT91SAM9260_ID_US5
#define	USART_GSM	AT91SAM9260_ID_US2
#define	USART_GSM_DBG	AT91SAM9260_ID_US3
#define	USART_GPS	AT91SAM9260_ID_US4

#define	TTYS_DBGU	0
#define	TTYS_SER1	1
#define	TTYS_SER2	2
#define	TTYS_SER3	3
#define	TTYS_GSM	4
#define	TTYS_GPS	5
#define	TTYS_GSM_DBG	6

#define	MRFSA_PIN_GSM_PWRMON	AT91_PIN_PC6

#include <linux/delay.h>

static int mrfsa_uart_open(struct uart_port *port)
{
	switch (port->line) {
	case TTYS_GSM:
	case TTYS_GSM_DBG:
		if (!at91_get_gpio_value(MRFSA_PIN_GSM_PWRMON)) {
			/* need to power on gsm */
			gpio_set_value(MRFSA_PIN_GSM_ONOFF, 1);
			msleep(1100);
			gpio_set_value(MRFSA_PIN_GSM_ONOFF, 0);
			/* @@@ configure pin directions @@@ */
			if (!at91_get_gpio_value(MRFSA_PIN_GSM_PWRMON)) {
				/* gsm did not power on? */
			}
		}
		break;
	case TTYS_GPS:
		gpio_set_value(MRFSA_PIN_GPS_ON, 0);
		gpio_set_value(MRFSA_PIN_GPS_RESET, 1);
		break;
	}
	return 0;
}

static void mrfsa_uart_close(struct uart_port *port)
{
	switch (port->line) {
	case TTYS_GSM:
	case TTYS_GSM_DBG:
		/* @@@ power off if other port is not in use @@@ */
		break;
	case TTYS_GPS:
		gpio_set_value(MRFSA_PIN_GPS_ON, 1);
		gpio_set_value(MRFSA_PIN_GPS_RESET, 1);
		break;
	}
}

static struct atmel_port_fns atmel_uart_port_fns = {
	.open		= mrfsa_uart_open,
	.close		= mrfsa_uart_close,
};

static void __init mrfsa_map_io(void)
{
	/* Initialize processor: 20.000 MHz crystal */
	at91sam9260_initialize(20000000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(USART_DBGU, TTYS_DBGU, 0);

	/* register handlers.. */
	atmel_register_uart_fns(&atmel_uart_port_fns);

	/* USART0 on ttyS1. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI) */
	at91_register_uart(USART_SER1, TTYS_SER1, ATMEL_UART_CTS | ATMEL_UART_RTS
			   | ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD
			   | ATMEL_UART_RI);

	/* USART1 on ttyS2. (Rx, Tx, RTS, CTS) */
	at91_register_uart(USART_SER2, TTYS_SER2, ATMEL_UART_CTS | ATMEL_UART_RTS);

	/* USART5 on ttyS3. (Rx, Tx) */
	at91_register_uart(USART_SER3, TTYS_SER3, 0);

	/* USART2 on ttyS4. (Rx, Tx, RTS, CTS) */
	at91_register_uart(USART_GSM, TTYS_GSM, ATMEL_UART_CTS | ATMEL_UART_RTS);

	/* USART4 on ttyS5. (Rx, Tx) */
	at91_register_uart(USART_GPS, TTYS_GPS, 0);

	/* USART3 on ttyS6. (Rx, Tx) */
	at91_register_uart(USART_GSM_DBG, TTYS_GSM_DBG, 0);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(TTYS_DBGU);
}

static void __init mrfsa_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}


/*
 * USB Host port
 */
static struct at91_usbh_data __initdata mrfsa_usbh_data = {
	.ports		= 2,
	.vbus_pin	= {0, 0},
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata mrfsa_udc_data = {
	.vbus_pin	= AT91_PIN_PC5,
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};


/*
 * SPI devices.
 */
#include <linux/spi/mc74x595.h>

static const struct mc74x595_platform_data mrfsa_mc74x595_pdata = {
	.nchips	= 2,
	.inversion = {0xff},
	.initial = {0x00, 0x30},
	.base = MRFSA_PIN_BASE,
};
static struct spi_board_info mrfsa_spi_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PA3,
	},
	{	/* DataFlash card */
		.modalias	= "mtd_dataflash",
		.chip_select	= 4,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PC11,
	},
	{	/* KS8995MA switch */
		.modalias	= "spi-ks8995",
		.chip_select	= 1,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PB30,
	},
	{	/* ZB */
		.modalias	= "spi-cc2420",
		.chip_select	= 2,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PB31,
	},
	{	/* GPIO */
		.modalias	= "mc74x595",
		.chip_select	= 3,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PB17,
		.platform_data	= (const void *)&mrfsa_mc74x595_pdata,
	},
	{	/* extension GPIO */
		.modalias	= "spi-gpio",
		.chip_select	= 0,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 1,
		.controller_data= (void*)AT91_PIN_PB3,
	},
};


/*
 * I2C device
 */

static struct i2c_gpio_platform_data i2c_pdata = {
	.sda_pin		= AT91_PIN_PC2,
	.sda_is_open_drain	= 1,
	.scl_pin		= AT91_PIN_PC3,
	.scl_is_open_drain	= 1,
	.udelay			= 2,		/* ~100 kHz */
};

static struct platform_device mrfsa_i2c_device = {
	.name			= "i2c-gpio",
	.id			= -1,
	.dev.platform_data	= &i2c_pdata,
};

static struct i2c_board_info mrfsa_i2c_devices[] = {
	{
		I2C_BOARD_INFO("24c02", 0x50)
	},
	{
		I2C_BOARD_INFO("ds1672", 0x68)
	},
};

/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata mrfsa_macb_data = {
	.phy_mask	= 0,
	.is_rmii	= 0,
};


/*
 * MCI (SD/MMC)
 * det_pin, wp_pin and vcc_pin are not connected
 */
static struct mci_platform_data __initdata mrfsa_mci_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PB25,
		.wp_pin		= AT91_PIN_PB21,
	}
};


/*
 * LEDs
 */
static struct gpio_led mrfsa_leds[] = {
	{	/* heartbeat led, green, userled1 to be defined */
		.name			= "STATUS",
		.gpio			= MRFSA_PIN_STAT_LED,
		.active_low		= 0,
		.default_trigger	= "heartbeat",
	},
	{	/* GSM led */
		.name			= "GSM",
		.gpio			= MRFSA_PIN_GSM_LED,
		.active_low		= 0,
	},
	{	/* ZigBee led */
		.name			= "ZIGBEE",
		.gpio			= MRFSA_PIN_ZB_LED,
		.active_low		= 0,
	},
	{	/* SER1 led */
		.name			= "SER1",
		.gpio			= MRFSA_PIN_SLED1,
		.active_low		= 0,
	},
	{	/* SER2 led */
		.name			= "SER2",
		.gpio			= MRFSA_PIN_SLED2,
		.active_low		= 0,
	},
	{	/* SER3 led */
		.name			= "SER3",
		.gpio			= MRFSA_PIN_SLED3,
		.active_low		= 0,
	},
	{	/* Input 1 led */
		.name			= "IN1",
		.gpio			= MRFSA_PIN_ILED1,
		.active_low		= 0,
	},
	{	/* Input 1 led */
		.name			= "IN2",
		.gpio			= MRFSA_PIN_ILED2,
		.active_low		= 0,
	},
};

/*
 * GPIO Buttons
 */
static void __init mrfsa_add_device_buttons(void) {}

static void __init mrfsa_board_init(void)
{
	struct fixed_phy_status fixed_phy_status = {
		.link			= 1,
		.speed			= SPEED_100,
		.duplex			= DUPLEX_FULL,
	};

	/* Serial */
	at91_add_device_serial();
	/* Watchdog */
	at91_add_gpio_wdt(AT91_PIN_PC4);
	/* USB Host */
	at91_add_device_usbh(&mrfsa_usbh_data);
	/* USB Device */
	at91_add_device_udc(&mrfsa_udc_data);
	/* SPI */
	at91_add_device_spi(mrfsa_spi_devices, ARRAY_SIZE(mrfsa_spi_devices));
	/* I2C */
	at91_set_GPIO_periph(AT91_PIN_PC3, 1);		/* SCL */
	at91_set_multi_drive(AT91_PIN_PC3, 1);
	at91_set_GPIO_periph(AT91_PIN_PC2, 1);		/* SDA */
	at91_set_multi_drive(AT91_PIN_PC2, 1);
	i2c_register_board_info(0, mrfsa_i2c_devices, ARRAY_SIZE(mrfsa_i2c_devices));
	platform_device_register(&mrfsa_i2c_device);
	/* Ethernet */
	if (fixed_phy_add(PHY_POLL, 1, &fixed_phy_status) != 0) {
		printk(KERN_ERR "fixed_phy_add() failed!\n");
	}
	at91_add_device_eth(&mrfsa_macb_data);
	/* MMC */
	at91_add_device_mci(0, &mrfsa_mci_data);
	/* SSC */
	at91_add_device_ssc(AT91SAM9260_ID_SSC, ATMEL_SSC_TX);
	/* LEDs */
	at91_gpio_leds(mrfsa_leds, ARRAY_SIZE(mrfsa_leds));
	/* Push Buttons */
	mrfsa_add_device_buttons();
}

MACHINE_START(MRFSA, "Embedtronics MRFSA")
	/* Maintainer: Embedtronics Oy */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= mrfsa_map_io,
	.init_irq	= mrfsa_init_irq,
	.init_machine	= mrfsa_board_init,
MACHINE_END
