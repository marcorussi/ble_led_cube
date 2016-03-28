/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2015] [Marco Russi]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/




/* ------------------- Inclusions ------------------- */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_trace.h"
#include "app_util_platform.h"

#include "config.h"
#ifdef UART_DEBUG
#include "uart.h"
#endif
#include "mpu6050.h"
#include "application.h"




/* ------------------- Local defines ------------------- */

/* Number of motion axis */
#define MPU6050_NUM_OF_MOTION_AXIS			3

/* Burst read APP timer defines */
#define APP_TIMER_TICK_PERIOD_MS			CFG_MPU6050_BURST_READ_UPDATE_MS
#define APP_TIMER_TICK_COUNT				((uint32_t)(((uint64_t)APP_TIMER_TICK_PERIOD_MS * 1000000)/30517))

/* LSB sensitivity */
#define LSB_SENS_RANGE_2G					16384
#define LSB_SENS_RANGE_4G					8192
#define LSB_SENS_RANGE_8G					4096
#define LSB_SENS_RANGE_16G					2048

/* 1G threshold */
#define ACC_LSB_THRESHOLD_1G				LSB_SENS_RANGE_4G

/* LSB tolerance */
#define ACC_LSB_THRESHOLD_TOL				2000

/* G thresholds */
#define ACC_LSB_THRESHOLD_1G_LOW			(ACC_LSB_THRESHOLD_1G - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_1G_HIGH			(ACC_LSB_THRESHOLD_1G + ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_2G_LOW			((ACC_LSB_THRESHOLD_1G*2) - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_2G_HIGH			((ACC_LSB_THRESHOLD_1G*2) + ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_3G_LOW			((ACC_LSB_THRESHOLD_1G*3) - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_3G_HIGH			((ACC_LSB_THRESHOLD_1G*3) + ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_4G_LOW			((ACC_LSB_THRESHOLD_1G*4) - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_4G_HIGH			((ACC_LSB_THRESHOLD_1G*4) + ACC_LSB_THRESHOLD_TOL)




/* ------------------- Local typedefs ------------------- */

/* Structure containing x,y,z and temperature values */
typedef struct
{
	int16_t acc_xyz[MPU6050_NUM_OF_MOTION_AXIS];
	int16_t temp;
} motion_temp_st;

/* Structure to store motion and temperature values */
static motion_temp_st motion_temp_values;




/* ------------------- Local macros ------------------- */

/* Define timer for MPU6050 burst read trigger */
APP_TIMER_DEF(read_trigger);




/* ------------------- Local functions prototypes ------------------- */

static void mpu6050_burst_read_callback	(int16_t *, uint8_t);
static void read_timeout_handler		(void *);
static void timer_config				(void);




/* ------------------- Local functions prototypes ------------------- */

/* Burst read callback function */
void mpu6050_burst_read_callback( int16_t *p_data, uint8_t data_length)
{
	uint8_t i;

	/* convert x,y,z in G */
	for(i=0; i<MPU6050_NUM_OF_MOTION_AXIS; i++)
	{
		/* calculate value in G */

		/* if higher than ACC_LSB_THRESHOLD_4G_HIGH */
		if(p_data[i] > ACC_LSB_THRESHOLD_4G_HIGH)
		{
			/* saturate at 4G */
			motion_temp_values.acc_xyz[i] = 4;
		}
		/* around 4G */
		if((p_data[i] < ACC_LSB_THRESHOLD_4G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_4G_LOW))
		{
			motion_temp_values.acc_xyz[i] = 4;
		}
		/* around 3G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_3G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_3G_LOW))
		{
			motion_temp_values.acc_xyz[i] = 3;
		}
		/* around 2G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_2G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_2G_LOW))
		{
			motion_temp_values.acc_xyz[i] = 2;
		}
		/* around 1G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_1G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_1G_LOW))
		{
			motion_temp_values.acc_xyz[i] = 1;
		}
		/* else lower than ACC_LSB_THRESHOLD_1G_LOW */
		else
		{
			/* consider 0 G */
			motion_temp_values.acc_xyz[i] = 0;
		}
	}
#ifdef LED_DEBUG
	nrf_gpio_pin_toggle(21);
#endif
	/* store temperature */
	motion_temp_values.temp = p_data[3];

	//application_run();

#ifdef UART_DEBUG
	uint8_t uart_string[20];
	sprintf((char *)uart_string, "_X: %d - %d", p_data[0], motion_temp_values.acc_xyz[0]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
	sprintf((char *)uart_string, "_Y: %d - %d", p_data[1], motion_temp_values.acc_xyz[1]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
	sprintf((char *)uart_string, "_Z: %d - %d", p_data[2], motion_temp_values.acc_xyz[2]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif
}


/* Timer timeout handler for triggering a new conversion */
static void read_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);

	/* start burst read of MPU6050 */
	mpu6050_start_burst_read();
}


/* Function to init the read timer */
static void timer_config(void)
{
    uint32_t err_code;

	/* init ADC trigger timer */
	err_code = app_timer_create(&read_trigger, APP_TIMER_MODE_REPEATED, read_timeout_handler);
    APP_ERROR_CHECK(err_code);

	/* start ADC trigger timer */
	err_code = app_timer_start(read_trigger, APP_TIMER_TICK_COUNT, NULL);
    APP_ERROR_CHECK(err_code);
}




/* --------------- Exported functions ----------------- */

/* Function to init the application */
void application_init( void )
{
	mpu6050_init_st init_data;

	/* set burst read callback function */
	init_data.burst_read_handler = (burst_read_handler_st)mpu6050_burst_read_callback;

	/* if mpu6050 initialised successfully */
	if(true == mpu6050_init(&init_data))
	{
		/* start burst read timer */
		timer_config();
	}
	else
	{
		/* mpu6050 init failed. Do nothing */
	}
}


/* Main loop function of the application */
void application_run( void )
{
#ifdef LED_DEBUG
	if(motion_temp_values.acc_xyz[0] > 0)
	{
		nrf_gpio_pin_write(22, 0);
	}
	else
	{
		nrf_gpio_pin_write(22, 1);
	}

	if(motion_temp_values.acc_xyz[1] > 0)
	{
		nrf_gpio_pin_write(23, 0);
	}
	else
	{
		nrf_gpio_pin_write(23, 1);
	}

	if(motion_temp_values.acc_xyz[2] > 0)
	{
		nrf_gpio_pin_write(24, 0);
	}
	else
	{
		nrf_gpio_pin_write(24, 1);
	}
#endif
}




/* End of file */




