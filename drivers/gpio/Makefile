# gpio support: dedicated expander chips, etc

ccflags-$(CONFIG_DEBUG_GPIO)	+= -DDEBUG
EXTRA_CFLAGS += -D__IN_XENOMAI__ -Iinclude/xenomai

obj-$(CONFIG_GPIOLIB)		+= gpiolib.o

obj-$(CONFIG_GPIO_MAX4896)	+= max4896.o
obj-$(CONFIG_GPIO_MAX7301)	+= max7301.o
obj-$(CONFIG_GPIO_MAX732X)	+= max732x.o
obj-$(CONFIG_GPIO_MCP23S08)	+= mcp23s08.o
obj-$(CONFIG_GPIO_MC74X595)	+= mc74x595.o
obj-$(CONFIG_GPIO_PCA953X)	+= pca953x.o
obj-$(CONFIG_GPIO_PCF857X)	+= pcf857x.o
obj-$(CONFIG_GPIO_TWL4030)	+= twl4030-gpio.o
obj-$(CONFIG_GPIO_XILINX)	+= xilinx_gpio.o
obj-$(CONFIG_GPIO_BT8XX)	+= bt8xxgpio.o
obj-$(CONFIG_GPIO_TM_EFDC_DIO)	+= tm_efdc-dio.o
obj-$(CONFIG_GPIO_TM_EFDC)	+= tm_efdc-gpio.o
obj-$(CONFIG_GPIO_LTC186X)	+= ltc186x.o
obj-$(CONFIG_GPIO_AD5666)	+= ad5666.o
