/*
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
 *  Copyright (C) 2009 Embedtronics OY
 *  Copyright (C) 2010 Telemerkki OY
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

#include <linux/spi/ltc186x.h>
#include <linux/spi/ad5666.h>

#include "sam9_smc.h"
#include "generic.h"

#include <linux/serial_core.h>

#define	USART_DBGU	0
#define	USART_SER1	AT91SAM9260_ID_US0
#define	USART_SER2	AT91SAM9260_ID_US1
#define	USART_SER3	AT91SAM9260_ID_US2
#define	USART_SER4	AT91SAM9260_ID_US3
#define	USART_SER5	AT91SAM9260_ID_US4
#define	USART_SER6	AT91SAM9260_ID_US5

#define	TTYS_DBGU	0
#define	TTYS_SER1	1
#define	TTYS_SER2	2
#define	TTYS_SER3	3
#define	TTYS_SER4	4
#define	TTYS_SER5	5
#define	TTYS_SER6	6

#include <linux/delay.h>

static int efdc_uart_open(struct uart_port *port)
{
	return 0;
}

static void efdc_uart_close(struct uart_port *port)
{
}

static struct atmel_port_fns atmel_uart_port_fns = {
	.open		= efdc_uart_open,
	.close		= efdc_uart_close,
};

static void __init efdc_map_io(void)
{
	/* Initialize processor: 20.000 MHz crystal */
	at91sam9260_initialize(20000000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(USART_DBGU, TTYS_DBGU, 0);

	/* register handlers.. */
	//atmel_register_uart_fns(&atmel_uart_port_fns);

	/* USART0 on ttyS1. (Rx, Tx) */
	at91_register_uart(USART_SER1, TTYS_SER1, 0);

	/* USART1 on ttyS2. (Rx, Tx) */
	at91_register_uart(USART_SER2, TTYS_SER2, 0);

	/* USART2 on ttyS3. (Rx, Tx) */
	at91_register_uart(USART_SER3, TTYS_SER3, 0);

	/* USART3 on ttyS4. (Rx, Tx) */
	at91_register_uart(USART_SER4, TTYS_SER4, 0);

	/* USART4 on ttyS5. (Rx, Tx) */
	at91_register_uart(USART_SER5, TTYS_SER5, 0);

	/* USART5 on ttyS6. (Rx, Tx) */
	at91_register_uart(USART_SER6, TTYS_SER6, 0);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(TTYS_DBGU);
}

static void __init efdc_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}


/*
 * SPI devices.
 */
static struct ltc186x_platform_data ltc186x_1_platform_data = {
	.base		= TM_EFDC_AIN_BASE,
	.channels	= 4,
};

static struct ltc186x_platform_data ltc186x_2_platform_data = {
	.base		= TM_EFDC_AIN_BASE + 4,
	.channels	= 4,
};

static struct ad5666_platform_data ad5666_1_platform_data = {
	.base		= TM_EFDC_AOUT_BASE,
	.ref	 	= 1,
	.pwr_mask	= 0xF,
};

static struct ad5666_platform_data ad5666_2_platform_data = {
	.base		= TM_EFDC_AOUT_BASE + 4,
	.ref	 	= 1,
	.pwr_mask	= 0xF,
};

static struct spi_board_info efdc_spi_devices[] = {
#if 0
	{	/* SPI-flash chip */
		.modalias	= "m25p80",
		.irq		= -1,
		.chip_select	= 0,
		.max_speed_hz	= 20 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PA3,
	},
#endif
	{	/* or DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PA3,
	},
	{	/* KS8995MA switch */
		.modalias	= "spi-ks8995",
		.chip_select	= 1,
		.max_speed_hz	= 4 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PA4,
	},
	{	/* CAN */
		.modalias	= "spi-mc251x",
		.chip_select	= 1+4,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PA5,
	},
	{	/* ADC1 */
		.modalias	= "ltc186x",
		.chip_select	= 2,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PC3,
		.platform_data	= &ltc186x_1_platform_data,
	},
	{	/* ADC2 */
		.modalias	= "ltc186x",
		.chip_select	= 2+4,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.controller_data= (void*)AT91_PIN_PC6,
		.platform_data	= &ltc186x_2_platform_data,
	},
	{	/* DAC1 */
		.modalias	= "ad5666",
		.chip_select	= 3,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.mode		= SPI_MODE_2,
		.controller_data= (void*)AT91_PIN_PC8,
		.platform_data	= &ad5666_1_platform_data,
	},
	{	/* DAC2 */
		.modalias	= "ad5666",
		.chip_select	= 3+4,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.mode		= SPI_MODE_2,
		.controller_data= (void*)AT91_PIN_PC9,
		.platform_data	= &ad5666_2_platform_data,
	},
	{	/* digital I/O */
		.modalias	= "tm_efdc-dio",
		.chip_select	= 0+4,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 1,
		.controller_data= (void*)AT91_PIN_PB3,
	},
};


/*
 * I2C device
 */

static struct i2c_gpio_platform_data i2c_pdata = {
	.sda_pin		= AT91_PIN_PB17,
	.sda_is_open_drain	= 1,
	.scl_pin		= AT91_PIN_PB16,
	.scl_is_open_drain	= 1,
	.udelay			= 2,		/* ~100 kHz */
};

static struct platform_device efdc_i2c_device = {
	.name			= "i2c-gpio",
	.id			= -1,
	.dev.platform_data	= &i2c_pdata,
};

static struct i2c_board_info efdc_i2c_devices[] = {
	{
		I2C_BOARD_INFO("24c02", 0x50)
	},
};

static struct platform_device efdc_gpio_device = {
	.name			= "efdc-gpio",
};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata efdc_macb_data = {
	.phy_mask	= 0,
	.is_rmii	= 0,
};


/*
 * MCI (SD/MMC), VCC pin is not connected
 */
static struct mci_platform_data __initdata efdc_mci_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PB23,
		.wp_pin		= AT91_PIN_PB22,
	}
};


/*
 * LEDs
 */
static struct gpio_led efdc_leds[] = {
	{	/* heartbeat led, green, userled1 to be defined */
		.name			= "STATUS",
		.gpio			= AT91_PIN_PC11,
		.active_low		= 0,
		.default_trigger	= "heartbeat",
	},
	{	/* action led, green */
		.name			= "ACT",
		.gpio			= AT91_PIN_PC12,
		.active_low		= 0,
	},
};

static void __init efdc_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* Watchdog */
	at91_add_gpio_wdt(AT91_PIN_PC2);
	/* SPI */
	at91_add_device_spi(efdc_spi_devices, ARRAY_SIZE(efdc_spi_devices));
	/* I2C */
	at91_set_GPIO_periph(AT91_PIN_PB16, 1);		/* SCL */
	at91_set_multi_drive(AT91_PIN_PB16, 1);
	at91_set_GPIO_periph(AT91_PIN_PB17, 1);		/* SDA */
	at91_set_multi_drive(AT91_PIN_PB17, 1);
	i2c_register_board_info(0, efdc_i2c_devices, ARRAY_SIZE(efdc_i2c_devices));
	platform_device_register(&efdc_i2c_device);
	/* Ethernet */
	at91_add_device_eth(&efdc_macb_data);
	/* MMC */
	at91_add_device_mci(0, &efdc_mci_data);
	/* LEDs */
	at91_gpio_leds(efdc_leds, ARRAY_SIZE(efdc_leds));
	/* GPIO */
	platform_device_register(&efdc_gpio_device);
}

MACHINE_START(TM_EFDC, "Telemerkki TM-EFDC")
	/* Maintainer: Embedtronics Oy */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= efdc_map_io,
	.init_irq	= efdc_init_irq,
	.init_machine	= efdc_board_init,
MACHINE_END
