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


/* ------------------- Local defines ------------------- */

/* Comment this define to use flash defined values */
//#define USE_UICR_FOR_MAJ_MIN_VALUES

/* Low frequency clock source to be used by the SoftDevice */
#define NRF_CLOCK_LFCLKSRC      			{.source        = NRF_CLOCK_LF_SRC_RC,				\
                                 		 	 .rc_ctiv       = 15,                              	\
                                 		 	 .rc_temp_ctiv  = 15,								\
                                 		 	 .xtal_accuracy = 0}

/* Number of central links used by the application. When changing this number remember to adjust the RAM settings */
#define CENTRAL_LINK_COUNT              	0     

/* Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings */
#define PERIPHERAL_LINK_COUNT            	1  

/* Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device */
#define IS_SRVC_CHANGED_CHARACT_PRESENT 	0                                 

/* Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout */
#define APP_CFG_NON_CONN_ADV_TIMEOUT    	0          
                
/* The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s) */       
#define NON_CONNECTABLE_ADV_INTERVAL    	MSEC_TO_UNITS(100, UNIT_0_625_MS) 

/* Timer period in milli seconds */
#define TRIGGER_TIMER_PERIOD_MS				5000		/* 5 s */
/* Timer counter value */
#define TRIGGER_TIMER_COUNT					((uint32_t)(((uint64_t)TRIGGER_TIMER_PERIOD_MS * 1000000)/30517))

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
/* Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location */
#define UICR_ADDRESS              			0x10001080                  	
#endif

/* Adv fixed fields values */
#define ADV_FLAGS_TYPE						BLE_GAP_AD_TYPE_FLAGS
#define BR_EDR_NOT_SUPPORTED				BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED
#define MANUF_DATA_TYPE						BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA
#define MANUFACTURER_ID						0x0059
#define MANUF_DATA_LENGTH					19
#define MANUF_PRODUCT_ID					0x0202	
#define MANUF_SERVICE_ID					0x0110	
#define TX_POWER_MEASURED_RSSI				0xc2

#define FIXED_DEVICE_ID						0x00010203

#define MAX_ADV_LENGTH						31




/* ---------- Local typedefs ---------- */

/* Adv packet format to scan */
typedef enum
{
	FIRST_LENGTH_POS,					/* first length */
	ADV_TYPE_FLAGS_POS,					/* adv flags type */
	BR_EDR_NOT_SUPPORTED_POS,			/* BR/EDR not supported */
	SECOND_LENGTH_POS,					/* second length */
	MANUF_DATA_TYPE_POS,				/* manufacturer data type */
	MANUF_ID_BYTE_0_POS,				/* manufacturer ID lower byte */
	MANUF_ID_BYTE_1_POS,				/* manufacturer ID higher byte */
	MANUF_DATA_LENGTH_POS,				/* data length */
	PRODUCT_ID_BYTE_0_POS,				/* product type ID lower byte */
	PRODUCT_ID_BYTE_1_POS,				/* product type ID higher byte */
	SERVICE_UUID_BYTE_0_POS,			/* service ID lower byte */
	SERVICE_UUID_BYTE_1_POS,			/* service ID higher byte */
	DEVICE_ID_BYTE_0_POS,				/* device ID byte 0 */
	DEVICE_ID_BYTE_1_POS,				/* device ID byte 1 */
	DEVICE_ID_BYTE_2_POS,				/* device ID byte 2 */
	DEVICE_ID_BYTE_3_POS,				/* device ID byte 3 */
	DATA_BYTE_0_POS,					/* data byte 0 */
	DATA_BYTE_1_POS,					/* data byte 1 */
	DATA_BYTE_2_POS,					/* data byte 2 */
	DATA_BYTE_3_POS,					/* data byte 3 */
	DATA_BYTE_4_POS,					/* data byte 4 */
	DATA_BYTE_5_POS,					/* data byte 5 */
	DATA_BYTE_6_POS,					/* data byte 6 */
	DATA_BYTE_7_POS,					/* data byte 7 */
	RESERVED_0_POS,						/* reserved 0 */
	RESERVED_1_POS,						/* reserved 1 */
	CALIB_RSSI_POS,						/* calibrated RSSI */
	ADV_PACKET_LENGTH					/* Adv packet length. This is not included. It is for fw purpose only */
} adv_packet_form_e;




/* ------------------ Local macros ----------------------- */

/* Advertisement timer for test update */
APP_TIMER_DEF(adv_timer);




/* ---------- Local const variables ---------- */

/* Preamble of the Adv packet. This string represent a fixed part of the adv packet */
static const uint8_t preamble_adv[ADV_PACKET_LENGTH] = 
{
	0x02,								/* first length */
	ADV_FLAGS_TYPE,						/* adv flags type */
	BR_EDR_NOT_SUPPORTED,				/* BR/EDR not supported */
	(uint8_t)(MANUF_DATA_LENGTH + 4),	/* second length */
	MANUF_DATA_TYPE,					/* manufacturer data type */
	(uint8_t)MANUFACTURER_ID,			/* manufacturer ID lower byte */
	(uint8_t)(MANUFACTURER_ID >> 8),	/* manufacturer ID higher byte */
	MANUF_DATA_LENGTH,					/* manufacturer specific data length */
	(uint8_t)MANUF_PRODUCT_ID,			/* product type ID lower byte */
	(uint8_t)(MANUF_PRODUCT_ID >> 8),	/* product type ID higher byte */
	(uint8_t)MANUF_SERVICE_ID,			/* service UUID lower byte */
	(uint8_t)(MANUF_SERVICE_ID >> 8),	/* service UUID higher byte */
	(uint8_t)FIXED_DEVICE_ID,			/* Device ID byte 0 */
	(uint8_t)(FIXED_DEVICE_ID >> 8),	/* Device ID byte 1 */
	(uint8_t)(FIXED_DEVICE_ID >> 16),	/* Device ID byte 2 */
	(uint8_t)(FIXED_DEVICE_ID >> 24),	/* Device ID byte 3 */
	0x00,								/* Data 0 */
	0x00,								/* Data 1 */
	0x00,								/* Data 2 */
	0x00,								/* Data 3 */
	0x00,								/* Data 4 */
	0x00,								/* Data 5 */
	0x00,								/* Data 6 */
	0x00,								/* Data 7 */
	0x00,								/* reserved 0 */
	0x00,								/* reserved 1 */
	TX_POWER_MEASURED_RSSI				/* Calibrated TX RSSI */
};




/* ------------------- Local variables ------------------- */

/* Parameters to be passed to the stack when starting advertising */
static ble_gap_adv_params_t m_adv_params;    

/* Parameters to be passed to the stack when starting advertising */
static uint8_t device_name[] = "ble_cube_proto"; 

/* Array containing advertisement packet data */
uint8_t adv_data[ADV_PACKET_LENGTH];

/* Array containing scan response packet data */
uint8_t scan_resp_data[MAX_ADV_LENGTH];




/* ------------------- Local functions prototypes ------------------- */

static void advertising_init	(void);        
static void advertising_start	(void);  
static void ble_stack_init		(void);
static void timer_handler		(void *);                




/* ------------------- Local functions ------------------- */

/* Function for initializing the Advertising functionality.
   Encodes the required advertising data and passes it to the stack.
   Also builds a structure to be passed to the stack when starting advertising.
*/
static void advertising_init(void)
{
	uint32_t ret_code;
    ble_gap_conn_sec_mode_t gap_conn_sec_mode;

	/* set adv data */
	memcpy((void *)&adv_data[0], (const void *)preamble_adv, ADV_PACKET_LENGTH);

    /* Overwrite device ID with UICR values if needed */
#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    uint32_t device_id;
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    device_id = *(uint32_t *)UICR_ADDRESS;
	/* overwrite device ID value */
    memcpy((void *)&adv_data[DEVICE_ID_BYTE_0_POS], (const void *)&device_id, 4);
#endif

    /* set scan response data with shortened device name */
    scan_resp_data[0] = 1 + strlen((const char*)device_name);
    scan_resp_data[1] = BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME;
    memcpy(&scan_resp_data[2], device_name, (scan_resp_data[0] - 1));

    /* set advertising data */
    ret_code = sd_ble_gap_adv_data_set((uint8_t const *)adv_data, ADV_PACKET_LENGTH, (uint8_t const *)scan_resp_data, (scan_resp_data[0] + 1));
	APP_ERROR_CHECK(ret_code);

    /* Set device name to BLE gap layer */
    gap_conn_sec_mode.sm = 0;
    gap_conn_sec_mode.lv = 0;
    ret_code = sd_ble_gap_device_name_set((ble_gap_conn_sec_mode_t const *)&gap_conn_sec_mode, device_name, (scan_resp_data[0] - 1));
	APP_ERROR_CHECK(ret_code);

    /* Initialize advertising parameters (used when starting advertising) */
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_SCAN_IND;
    m_adv_params.p_peer_addr = NULL;	/* Undirected advertisement */
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/* brief Function for starting advertising */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/* Function for initializing the BLE stack.
   Initializes the SoftDevice and the BLE event interrupt */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    /* Initialize the SoftDevice handler module. */
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    /* Check the ram settings against the used number of links */
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
 
    /* Enable BLE stack. */
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}


/* Timer timeout function */
static void timer_handler(void * p_context)
{
	uint32_t err_code;

	nrf_gpio_pin_toggle(24);

	err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

	if(adv_data[DATA_BYTE_0_POS] == 2)
	{
		adv_data[DATA_BYTE_0_POS] = 1;
	}
	else
	{
		adv_data[DATA_BYTE_0_POS] = 2;
	} 

	err_code = sd_ble_gap_adv_data_set((uint8_t const *)adv_data, ADV_PACKET_LENGTH, (uint8_t const *)scan_resp_data, (scan_resp_data[0] + 1));
	APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/* Function for application main entry */
void broadcaster_init(void)
{
    uint32_t err_code;

	nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_write(24, 1);

    ble_stack_init();

    advertising_init();

	/* init button timer */
	err_code = app_timer_create(&adv_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* Start execution */
    advertising_start();

	/* start timer */
	err_code = app_timer_start(adv_timer, TRIGGER_TIMER_COUNT, NULL);
	APP_ERROR_CHECK(err_code);

	/* stop debounce timer */
//	err_code = app_timer_stop(button_timer);
//	APP_ERROR_CHECK(err_code);
}




/* End of file */




