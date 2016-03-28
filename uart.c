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




/* --------------- Inclusions ----------------- */

#include "config.h"
#ifdef UART_DEBUG
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

#include "uart.h"




/* --------------- Local defines ----------------- */

/* UART TX and RX buffers size */
#define UART_TX_BUF_SIZE            		256
#define UART_RX_BUF_SIZE            		1

/* UART interface pins numbers */
#define RX_PIN_NUMBER						7
#define TX_PIN_NUMBER						30
#define RTS_PIN_NUMBER						3
#define CTS_PIN_NUMBER 						4




/* --------------- Local functions prototypes ----------------- */

static void uart_event_handler	(app_uart_evt_t *);




/* --------------- Local functions ----------------- */

/* Function to handle an UART event */
static void uart_event_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
		/* manage data from UART */
        case APP_UART_DATA_READY:
		{
			/* The followig code is just an ACK upon 'f' reception */
			uint8_t rx_byte;
			UNUSED_VARIABLE(app_uart_get(&rx_byte));
			if(rx_byte == 'f')
			{
				//nrf_gpio_pin_toggle(21);
				uart_send_string((uint8_t *)"GOT IT", 6);
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




/* ------------------- Exported functions ------------------- */

/* Function for sending a string */
void uart_send_string(uint8_t *data_string, uint8_t data_length)
{
	uint8_t index;

	for(index = 0; index < data_length; index++)
	{
		app_uart_put(data_string[index]);
		//nrf_delay_ms(10);
	}

	app_uart_put('\r');
}


/* Function to configure UART */
void uart_init(void)
{
    uint32_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
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



#endif
/* End of file */





