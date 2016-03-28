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




/* ---------------- Inclusions ------------------- */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "nordic_common.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_twi.h"

#include "config.h"
#ifdef UART_DEBUG
#include "uart.h"
#endif
#include "mpu6050.h"




/* TODO: allow range other configurations through the init structure */
/* TODO: convert temperature value */




/* ---------------- Local defines ------------------- */

/* TWI pin numbers */
#define TWI_SCL_PIN_NUMBER					12
#define TWI_SDA_PIN_NUMBER					13

/* Max number of pending TWI transactions */
#define MAX_PENDING_TRANSACTIONS    		5

/* Burst read buffer data length in bytes */
#define BUFFER_SIZE  						8

/* Burst read defines */
#define MPU6050_MOTION_VALUES				3
#define MPU6050_TEMPERATURE_VALUES			1
#define MPU6050_BURST_NUM_OF_VALUES			(MPU6050_MOTION_VALUES + MPU6050_TEMPERATURE_VALUES)
#define MPU6050_VALUES_BYTES_LENGTH     	2
#define MPU6050_BURST_READ_BYTES     		(MPU6050_BURST_NUM_OF_VALUES * MPU6050_VALUES_BYTES_LENGTH)

/* MPU6050 device TWI address */
#define MPU6050_ADDR     					0x68   

/* Time to wait before configuring the MPU6050 at start up */
#define MPU6050_POWER_UP_DELAY_MS			3000

/* Transfers count */
#define MPU6050_WHO_AM_I_TRANSFER_COUNT 	2
#define MPU6050_INIT_TRANSFER_COUNT 		5

/* Registers addresses */
#define CONFIG_REG_ADDR						0x1A
#define ACCEL_CONFIG_REG_ADDR				0x1C
#define SIGNAL_PATH_RESET_REG_ADDR			0x68
#define POWER_MAN1_REG_ADDR					0x6B
#define POWER_MAN2_REG_ADDR					0x6C

/* Registers init values */
#define CONFIG_REG_INIT_VALUE				0x05
#define ACCEL_CONFIG_REG_INIT_VALUE			0x08
#define SIGNAL_PATH_RESET_REG_INIT_VALUE	0x07
#define POWER_MAN1_REG_INIT_VALUE			0x20
#define POWER_MAN2_REG_INIT_VALUE			0xC0

/* Expected WHO_AM_I register value */
#define WHO_AM_I_REG_VALUE					0x68




/* ---------------- Local const variables ------------------- */

/* Addresses const variables */
static uint8_t const xout_high_reg_addr = 0x3B;	
static uint8_t const who_am_i_reg_addr  = 0x75;	




/* ---------------- Local macros ------------------- */

/* Macro defining the list of transfers for reading bytes */
#define MPU6050_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(MPU6050_ADDR, p_reg_addr, 1,        APP_TWI_NO_STOP), \
    APP_TWI_READ (MPU6050_ADDR, p_buffer,   byte_cnt, 0)

/* Macro defining x,y,z and temperature burst read */
#define MPU6050_READ_XYZ_TEMP(p_buffer) \
    MPU6050_READ(&xout_high_reg_addr, p_buffer, MPU6050_BURST_READ_BYTES)




/* ---------------- Local variables ------------------- */

/* APP TWI instance */
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);

/* Application callback function set at init */
static burst_read_handler_st app_burst_read_handler;

/* Final converted values. ATTENTION: This array must be static because passed 
   through the application callback as parameter */
static int16_t final_values[MPU6050_BURST_NUM_OF_VALUES];

/* Buffer for data read from sensors */
static uint8_t m_buffer[BUFFER_SIZE];

/* Data lists for initial transfers */
static uint8_t const transf_config_data[] = { CONFIG_REG_ADDR, CONFIG_REG_INIT_VALUE };
static uint8_t const transf_acc_config_data[] = { ACCEL_CONFIG_REG_ADDR, ACCEL_CONFIG_REG_INIT_VALUE };
static uint8_t const transf_signal_path_reset_data[] = { SIGNAL_PATH_RESET_REG_ADDR, SIGNAL_PATH_RESET_REG_INIT_VALUE };
static uint8_t const transf_power_mng1_data[] = { POWER_MAN1_REG_ADDR, POWER_MAN1_REG_INIT_VALUE };
static uint8_t const transf_power_mng2_data[] = { POWER_MAN2_REG_ADDR, POWER_MAN2_REG_INIT_VALUE };

/* WHO_AM_I transfer */
static app_twi_transfer_t const mpu6050_whoami_transfers[MPU6050_WHO_AM_I_TRANSFER_COUNT] = 
{
	APP_TWI_WRITE(MPU6050_ADDR, &who_am_i_reg_addr, 1, APP_TWI_NO_STOP),
	APP_TWI_READ (MPU6050_ADDR, m_buffer, 1, 0)
};

/* Initial transfer */
static app_twi_transfer_t const mpu6050_init_transfers[MPU6050_INIT_TRANSFER_COUNT] =
{
	APP_TWI_WRITE(MPU6050_ADDR, transf_signal_path_reset_data,  sizeof(transf_signal_path_reset_data), 	0),
	APP_TWI_WRITE(MPU6050_ADDR, transf_power_mng1_data,  		sizeof(transf_power_mng1_data), 		0),
	APP_TWI_WRITE(MPU6050_ADDR, transf_power_mng2_data,  		sizeof(transf_power_mng2_data), 		0),
	APP_TWI_WRITE(MPU6050_ADDR, transf_config_data, 			sizeof(transf_config_data), 			0),
	APP_TWI_WRITE(MPU6050_ADDR, transf_acc_config_data,  		sizeof(transf_acc_config_data), 		0)
};




/* ---------------- Local variables prototypes ------------------- */

static void bust_read_cb	(ret_code_t, void *);
static void twi_config		(void);




/* ---------------- Local functions ------------------- */

/* Burst read local callback function */
static void bust_read_cb(ret_code_t result, void * p_user_data)
{
	uint8_t i;
	uint16_t raw_values[MPU6050_BURST_NUM_OF_VALUES];
	
#ifdef UART_DEBUG
	uint8_t uart_string[10];

	//nrf_gpio_pin_toggle(21);
#endif

	/* if success */
    if (result == NRF_SUCCESS)
    {
		/* get obtained motion raw values and calculate final values */
		for(i=0; i<MPU6050_MOTION_VALUES; i++)
		{
			/* get 16-bit raw value */
			raw_values[i] = (((uint16_t)m_buffer[(i*2)+0] << 8) & 0xFF00);
			raw_values[i] |= ((uint16_t)m_buffer[(i*2)+1] & 0x00FF);
	
			/* convert from 2's complement to signed int */
			if((raw_values[i] & (1 << 15)) != 0)
			{
				final_values[i] = -((~raw_values[i]) + 1);
			}
			else
			{
		  		final_values[i] = (int16_t)raw_values[i];
			}
#ifdef UART_DEBUG
			/* pepare UART string for debugging */
			sprintf((char *)uart_string, "%x - %d", raw_values[i], final_values[i]);
			uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif
		}

		/* get obtained temperature raw value and calculate final value */
		/* get 16-bit raw value */
		raw_values[3] = (((uint16_t)m_buffer[6] << 8) & 0xFF00);
		raw_values[3] |= ((uint16_t)m_buffer[7] & 0x00FF);

		/* convert into temperature. TODO */
		final_values[3] = raw_values[3];
#ifdef UART_DEBUG
		/* pepare UART string for debugging */
		sprintf((char *)uart_string, "%x - %d", raw_values[3], final_values[3]);
		uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif

		/* send final values to application */
		if(app_burst_read_handler != NULL)
		{
			app_burst_read_handler(final_values, MPU6050_BURST_NUM_OF_VALUES);
		}
		else
		{
			/* it should never arrive here */
		}
	}
	else
	{
		/* do nothing */
#ifdef UART_DEBUG
		/* send ERROR string */
		uart_send_string((uint8_t *)"MPU6050 burst read ERROR", 24);
#endif
	}
}


/* TWI (with transaction manager) initialization */
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = TWI_SCL_PIN_NUMBER,
       .sda                = TWI_SDA_PIN_NUMBER,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
}




/* ---------------- Exported functions ------------------- */

/* Start a burst read operation */
bool mpu6050_start_burst_read(void)
{
	/* ATTENTION: always return success now */
	bool success = true;
#ifdef UART_DEBUG
    /* Signal on LED that something is going on */
    //nrf_gpio_pin_toggle(22);
#endif

    /* [these structures have to be "static" - they cannot be placed on stack
       since the transaction is scheduled and these structures most likely
       will be referred after this function returns] */
	static app_twi_transfer_t const transfers[] =
    {
        MPU6050_READ_XYZ_TEMP(&m_buffer[0])
    };

    static app_twi_transaction_t const transaction =
    {
        .callback            = bust_read_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));

	return success;
}


/* Init MPU6050 */
bool mpu6050_init( mpu6050_init_st *p_init )
{
	bool success = true;
#ifdef UART_DEBUG
	uint8_t uart_string[8];

	/* send initial UART string */
	uart_send_string((uint8_t *)"MPU6050 INIT", 12);
#endif

	/* wait enough time for MPU6050 to boot up */
	nrf_delay_ms(MPU6050_POWER_UP_DELAY_MS);

	/* innit TWI module */
    twi_config();

	/* perfomr WHO_AM_I register value check. This function is blocking */
	APP_ERROR_CHECK(app_twi_perform(&m_app_twi, mpu6050_whoami_transfers, MPU6050_WHO_AM_I_TRANSFER_COUNT, NULL));

	/* if the obtained value is equal to the expected one */
	if(m_buffer[0] == WHO_AM_I_REG_VALUE)
	{
		/* store burst reada handler function */
		app_burst_read_handler = p_init->burst_read_handler;

		/* perform initial tranfers for configuring the MPU6050 */
		APP_ERROR_CHECK(app_twi_perform(&m_app_twi, mpu6050_init_transfers, MPU6050_INIT_TRANSFER_COUNT, NULL));

#ifdef UART_DEBUG
		sprintf((char *)uart_string, "OK - %x", m_buffer[0]);
		uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif
#ifdef LED_DEBUG
		nrf_gpio_pin_write(21, 0);
#endif
	}
	else
	{
		/* error: do nothing */
		success = false;

#ifdef UART_DEBUG
		uart_send_string((uint8_t *)"FAIL", 4);
#endif
	}

	return success;
}




/* End of file */





