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
#include "app_uart.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_twi.h"




/* ---------------- Local defines ------------------- */

#define APP_TIMER_TICK_PERIOD_MS			3000
#define APP_TIMER_TICK_COUNT				((uint32_t)(((uint64_t)APP_TIMER_TICK_PERIOD_MS * 1000000)/30517))

#define MAX_PENDING_TRANSACTIONS    		5

#define UART_TX_BUF_SIZE            		256
#define UART_RX_BUF_SIZE            		1

#define APP_TIMER_PRESCALER         		0
#define APP_TIMER_OP_QUEUE_SIZE     		2

/* Buffer data length in bytes */
#define BUFFER_SIZE  						8

#define MPU6050_ADDR     					0x68   

#define MPU6050_WHO_AM_I_TRANSFER_COUNT 	2
#define MPU6050_INIT_TRANSFER_COUNT 		5

#define CONFIG_REG_ADDR						0x1A
#define SIGNAL_PATH_RESET_REG_ADDR			0x68

#define CONFIG_REG_INIT_VALUE				0x05
#define SIGNAL_PATH_RESET_REG_INIT_VALUE	0x07

#define WHO_AM_I_REG_VALUE					0x68


static uint8_t const xout_high_reg_addr = 0x3B;	
static uint8_t const who_am_i_reg_addr  = 0x75;	




/* ---------------- Local macros ------------------- */

#define MPU6050_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(MPU6050_ADDR, p_reg_addr, 1,        APP_TWI_NO_STOP), \
    APP_TWI_READ (MPU6050_ADDR, p_buffer,   byte_cnt, 0)

#define MPU6050_READ_XYZ(p_buffer) \
    MPU6050_READ(&xout_high_reg_addr, p_buffer, 8)




/* ---------------- Local variables ------------------- */

/* Define timer for ADC trigger */
APP_TIMER_DEF(adc_tim_trigger);

/* APP TWI instance */
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);

/* Buffer for data read from sensors */
static uint8_t m_buffer[BUFFER_SIZE];

static uint8_t const transf_config_data[] = { CONFIG_REG_ADDR, CONFIG_REG_INIT_VALUE };
static uint8_t const transf_signal_path_reset_data[] = { SIGNAL_PATH_RESET_REG_ADDR, SIGNAL_PATH_RESET_REG_INIT_VALUE };
static uint8_t const transf_acc_config_data[] = { 0x1C, 0x08 };
static uint8_t const transf_power_mng1_data[] = { 0x6B, 0x20 };
static uint8_t const transf_power_mng2_data[] = { 0x6C, 0x40 };

static app_twi_transfer_t const mpu6050_whoami_transfers[MPU6050_WHO_AM_I_TRANSFER_COUNT] = 
{
	APP_TWI_WRITE(MPU6050_ADDR, &who_am_i_reg_addr, 1, APP_TWI_NO_STOP),
	APP_TWI_READ (MPU6050_ADDR, m_buffer, 1, 0)
};

static app_twi_transfer_t const mpu6050_init_transfers[MPU6050_INIT_TRANSFER_COUNT] =
{
    APP_TWI_WRITE(MPU6050_ADDR, transf_config_data, 			sizeof(transf_config_data), 			0),
	APP_TWI_WRITE(MPU6050_ADDR, transf_signal_path_reset_data,  sizeof(transf_signal_path_reset_data), 	0),
	APP_TWI_WRITE(MPU6050_ADDR, transf_acc_config_data,  		sizeof(transf_acc_config_data), 		0),
	APP_TWI_WRITE(MPU6050_ADDR, transf_power_mng1_data,  		sizeof(transf_power_mng1_data), 		0),
	APP_TWI_WRITE(MPU6050_ADDR, transf_power_mng2_data,  		sizeof(transf_power_mng2_data), 		0)
};




/* ---------------- Local functions ------------------- */

/* Function for send a string */
static void uart_send_string(uint8_t *data_string, uint8_t data_length)
{
	uint8_t index;

	for(index = 0; index < data_length; index++)
	{
		app_uart_put(data_string[index]);
		nrf_delay_ms(10);
	}

	app_uart_put('\r');
	//app_uart_flush();
}


void read_all_cb(ret_code_t result, void * p_user_data)
{
	nrf_gpio_pin_toggle(21);

    if (result == NRF_SUCCESS)
    {
		uint8_t stringa[6];

		uint16_t x_out = (((uint16_t)m_buffer[0] << 8) & 0xFF00);
		x_out |= ((uint16_t)m_buffer[1] & 0x00FF);

		uint16_t y_out = (((uint16_t)m_buffer[2] << 8) & 0xFF00);
		y_out |= ((uint16_t)m_buffer[3] & 0x00FF);

		uint16_t z_out = (((uint16_t)m_buffer[4] << 8) & 0xFF00);
		z_out |= ((uint16_t)m_buffer[5] & 0x00FF);

		uint16_t temp = (((uint16_t)m_buffer[6] << 8) & 0xFF00);
		temp |= ((uint16_t)m_buffer[7] & 0x00FF);

		sprintf((char *)stringa, "%x", x_out);
		uart_send_string((uint8_t *)stringa, strlen((const char *)stringa));

		sprintf((char *)stringa, "%x", y_out);
		uart_send_string((uint8_t *)stringa, strlen((const char *)stringa));

		sprintf((char *)stringa, "%x", z_out);
		uart_send_string((uint8_t *)stringa, strlen((const char *)stringa));

		sprintf((char *)stringa, "%x", temp);
		uart_send_string((uint8_t *)stringa, strlen((const char *)stringa));
	}
	else
	{
		/* do nothing at the moment */
		uart_send_string((uint8_t *)"ERROR", 5);
	}
}


static void read_all(void)
{
    /* Signal on LED that something is going on */
    nrf_gpio_pin_toggle(22);

    /* [these structures have to be "static" - they cannot be placed on stack
       since the transaction is scheduled and these structures most likely
       will be referred after this function returns] */
	static app_twi_transfer_t const transfers[] =
    {
        MPU6050_READ_XYZ(&m_buffer[0])
    };

    static app_twi_transaction_t const transaction =
    {
        .callback            = read_all_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}


/* UART handling */
static void uart_event_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
		/* manage data from UART */
        case APP_UART_DATA_READY:
		{
			uint8_t rx_byte;
			UNUSED_VARIABLE(app_uart_get(&rx_byte));
			if(rx_byte == 'f')
			{
				nrf_gpio_pin_toggle(21);
				uart_send_string((uint8_t *)"CULO", 4);
			}
			break;
		}
        case APP_UART_COMMUNICATION_ERROR:
		{
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
		}
        case APP_UART_FIFO_ERROR:
		{
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;
		}
        default:
            break;
    }
}


static void uart_config(void)
{
    uint32_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        1,	// RX
        2,	// TX
        3,	// RTS
        4,	// CTS
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


/* TWI (with transaction manager) initialization */
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = 10,
       .sda                = 11,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
}


/* Timer timeout handler for triggering a new conversion */
static void temp_meas_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);

	read_all();
}


static void timer_config(void)
{
    uint32_t err_code;

	/* init ADC trigger timer */
	err_code = app_timer_create(&adc_tim_trigger, APP_TIMER_MODE_REPEATED, temp_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

	/* start ADC trigger timer */
	err_code = app_timer_start(adc_tim_trigger, APP_TIMER_TICK_COUNT, NULL);
    APP_ERROR_CHECK(err_code);
}




/* ---------------- Exported functions ------------------- */

bool mpu6050_init( void )
{
	bool success = true;

	nrf_gpio_pin_dir_set(21, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(22, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_write(21, 0);
	nrf_gpio_pin_write(22, 1);

	//lfclk_config();

    uart_config();
	
	nrf_delay_ms(200);

	uart_send_string((uint8_t *)"FANCULO", 7);

    twi_config();

	APP_ERROR_CHECK(app_twi_perform(&m_app_twi, mpu6050_whoami_transfers, MPU6050_WHO_AM_I_TRANSFER_COUNT, NULL));

	if(m_buffer[0] == WHO_AM_I_REG_VALUE)
	{
		nrf_gpio_pin_write(22, 0);

		uint8_t stringa[6];
		sprintf((char *)stringa, "%x", m_buffer[0]);
		uart_send_string((uint8_t *)stringa, strlen((const char *)stringa));

		APP_ERROR_CHECK(app_twi_perform(&m_app_twi, mpu6050_init_transfers, MPU6050_INIT_TRANSFER_COUNT, NULL));

    	timer_config();
	}
	else
	{
		/* error: do nothing */
		success = false;
	}

	return success;
}




/* End of file */





