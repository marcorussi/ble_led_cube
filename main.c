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
#include "ble_manager.h"
#include "application.h"




/* ------------------- Local defines ------------------- */

/* Value used as error code on stack dump, can be used to identify stack location on stack unwind */
#define DEAD_BEEF                       	0xDEADBEEF                        

#define APP_TIMER_PRESCALER             	0                           	/* Value of the RTC1 PRESCALER register */
#define APP_TIMER_MAX_TIMERS            	(2+BSP_APP_TIMERS_NUMBER)     	/* Maximum number of simultaneously created timers */
#define APP_TIMER_OP_QUEUE_SIZE         	4                             	/* Size of timer operation queues */




/* ------------------- Local functions ------------------- */

/* Callback function for asserts in the SoftDevice.
   This function will be called in case of an assert in the SoftDevice.
   This handler is an example only and does not fit a final product. 
   You need to analyze how your product is supposed to react in case of Assert.
   On assert from the SoftDevice, the system can only recover on reset.
   param: line_num   Line number of the failing ASSERT call.
   param: file_name  File name of the failing ASSERT call.
*/
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/* Function for doing power management */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/* Function for application main entry */
int main(void)
{
    /* Initialize */
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

#ifdef LED_DEBUG
	/* init debug pins */
	nrf_gpio_pin_dir_set(21, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(22, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_write(21, 0);
	nrf_gpio_pin_write(22, 0);
	nrf_gpio_pin_write(23, 0);
	nrf_gpio_pin_write(24, 0);
#endif

#ifdef UART_DEBUG
	/* init UART */
    uart_init();
#endif

	/* init application */
	app_init();

    /* Enter main loop */
    for (;;)
    {
		/* application main loop function */
		app_run();

		//nrf_delay_ms(200);

        power_manage();
    }
}




/* End of file */




