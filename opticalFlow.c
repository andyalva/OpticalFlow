/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Modified by Fernando Cortes <fermando.corcam@gmail.com>
 * modified by Guillermo Rivera <memogrg@gmail.com>
 * Modified by Andres Alvarado <aalvaradov126@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <ADNS3080.h>


#define LBLUE GPIOE, GPIO8
#define LRED GPIOE, GPIO9
#define LORANGE GPIOE, GPIO10
#define LGREEN GPIOE, GPIO11
#define LBLUE2 GPIOE, GPIO12
#define LRED2 GPIOE, GPIO13
#define LORANGE2 GPIOE, GPIO14
#define LGREEN2 GPIOE, GPIO15

#define LD4 GPIOE, GPIO8
#define LD3 GPIOE, GPIO9
#define LD5 GPIOE, GPIO10
#define LD7 GPIOE, GPIO11
#define LD9 GPIOE, GPIO12
#define LD10 GPIOE, GPIO13
#define LD8 GPIOE, GPIO14
#define LD6 GPIOE, GPIO15

static void spi_setup(void)
{
	rcc_periph_clock_enable(RCC_SPI2);
	/* For spi signal pins */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOE);


	/* Setup GPIO pins for OpticalFlow for SPI2 signals. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO14 | GPIO15 | GPIO13);

	gpio_set_af(GPIOB, GPIO_AF5, GPIO14 | GPIO15 | GPIO13);



	/* Setup GPIOE3 pin for spi mode l3gd20 select. */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
	/* Start with spi communication disabled */
	gpio_set(GPIOE, GPIO3);

	//spi initialization;
	spi_set_master_mode(SPI2);
	spi_set_baudrate_prescaler(SPI2, SPI_CR1_BR_FPCLK_DIV_64);
	spi_set_clock_polarity_0(SPI2);
	spi_set_clock_phase_0(SPI2);
	spi_set_full_duplex_mode(SPI2);
	spi_set_unidirectional_mode(SPI2); /* bidirectional but in 3-wire */
	spi_set_data_size(SPI2, SPI_CR2_DS_8BIT);
	spi_enable_software_slave_management(SPI2);
	spi_send_msb_first(SPI2);
	spi_set_nss_high(SPI2);
	//spi_set_nss_low(SPI2);
	spi_enable_ss_output(SPI2);
	spi_fifo_reception_threshold_8bit(SPI2);
	SPI_I2SCFGR(SPI2) &= ~SPI_I2SCFGR_I2SMOD;
	spi_enable(SPI2);
}

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2| GPIO3);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}



static void my_usart_print_int(uint32_t usart, int32_t value)
{
	int8_t i;
	int8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send_blocking(usart, '-');
		value = value * -1;
	}

	if (value == 0) {
		usart_send_blocking(usart, '0');
	}

	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}

	for (i = nr_digits-1; i >= 0; i--) {
		usart_send_blocking(usart, buffer[i]);
	}

	usart_send_blocking(usart, '\r');
	usart_send_blocking(usart, '\n');
}

static void clock_setup(void)
{
	rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);
}

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOE);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
		GPIO14 | GPIO15);
}

int main(void)
{
	unsigned char motion;	
	signed char dx;
	signed char dy;
	unsigned char squal;
	unsigned char shutter_upper;
	unsigned char shutter_lower;
	unsigned char max_pixel;

	clock_setup();
	gpio_setup();
	usart_setup();
	spi_setup();


	

	//config
	gpio_clear(GPIOE, GPIO3);
	spi_send8(SPI2, ADNS3080_CONFIGURATION_BITS);
	spi_read8(SPI2);
	spi_send8(SPI2, ADNS3080_PIXELS_X | ADNS3080_PIXELS_Y);
	spi_read8(SPI2);
	gpio_set(GPIOE, GPIO3);

	while (1) {
		int i;


		gpio_clear(GPIOE, GPIO3);
		spi_send8(SPI2, ADNS3080_MOTION_BURST);
		spi_read8(SPI2);

		spi_send8(SPI2, 0); 
		motion = spi_read8(SPI2);

		spi_send8(SPI2, 0);
		dx = spi_read8(SPI2);

		spi_send8(SPI2, 0);
		dy = spi_read8(SPI2);

		spi_send8(SPI2, 0);
		squal = spi_read8(SPI2);

		spi_send8(SPI2, 0);
		shutter_upper = spi_read8(SPI2);

		spi_send8(SPI2, 0);
		shutter_lower = spi_read8(SPI2);

		spi_send8(SPI2, 0);
		max_pixel = spi_read8(SPI2);
		gpio_set(GPIOE, GPIO3);

		my_usart_print_int(USART2, (dx));

		if(dx >0){
			gpio_clear(GPIOE, GPIO15);
			gpio_set(GPIOE, GPIO11);}
		if(dx <0){
			gpio_clear(GPIOE, GPIO11);
			gpio_set(GPIOE, GPIO15);}
		if(dx == 0){
			gpio_clear(GPIOE, GPIO11);
			gpio_clear(GPIOE, GPIO15);}

		if(dy >0){
			gpio_clear(GPIOE, GPIO13);
			gpio_set(GPIOE, GPIO9);}
		if(dy <0){
			gpio_clear(GPIOE, GPIO9);
			gpio_set(GPIOE, GPIO13);}
		if(dy == 0){
			gpio_clear(GPIOE, GPIO9);
			gpio_clear(GPIOE, GPIO13);}
		my_usart_print_int(USART2, (dy));


		gpio_toggle(GPIOE, GPIO12);	/* LED on/off */
		for (i = 0; i < 80000; i++)
			__asm__("nop");
	}

	return 0;
}

