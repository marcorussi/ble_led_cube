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
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_dis.h"
#include "ble_bas.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "ble_conn_params.h"
#include "app_timer.h"
#include "app_trace.h"
#include "app_util_platform.h"
#include "dfu_init.h"

#include "config.h"
#include "cfg_service.h"
#include "ble_manager.h"




/* ------------------- Local defines ------------------- */

#ifdef FACE_INDEX_TEST
/* Timer period in milli seconds */
#define TRIGGER_TIMER_PERIOD_MS				5000		/* 5 s */
/* Timer counter value */
#define TRIGGER_TIMER_COUNT					((uint32_t)(((uint64_t)TRIGGER_TIMER_PERIOD_MS * 1000000)/30517))
#endif

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

/* The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms) */
#define APP_ADV_INTERVAL                 	100 

/* The advertising timeout (in units of seconds). 
   ATTENTION: do not exceed the maximum value in case of LIMITED DISCOVERABILITY MODE */
#define APP_ADV_TIMEOUT_IN_SECONDS      	60  

/* Value of the RTC1 PRESCALER register */
#define APP_TIMER_PRESCALER              	0    

/* Minimum acceptable connection interval (0.1 seconds) */
#define MIN_CONN_INTERVAL                	MSEC_TO_UNITS(100, UNIT_1_25_MS)           

/* Maximum acceptable connection interval (0.2 second) */
#define MAX_CONN_INTERVAL                	MSEC_TO_UNITS(200, UNIT_1_25_MS)           

/* Slave latency */
#define SLAVE_LATENCY                    	0                                          

/* Connection supervisory timeout (4 seconds) */
#define CONN_SUP_TIMEOUT                 	MSEC_TO_UNITS(4000, UNIT_10_MS)    

/* Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds) */
#define FIRST_CONN_PARAMS_UPDATE_DELAY   	APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) 

/* Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds) */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    	APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)

/* Number of attempts before giving up the connection parameter negotiation */
#define MAX_CONN_PARAMS_UPDATE_COUNT     	3                        

/* Min face index value */
#define MIN_FACE_INDEX_VALUE				1

/* Max face index value */
#define MAX_FACE_INDEX_VALUE				6

/* Initial face index value */
#define INITIAL_FACE_INDEX_VALUE			MIN_FACE_INDEX_VALUE

/* Max adv length */
#define MAX_ADV_LENGTH						31

#define ADV_FLAGS_TYPE						BLE_GAP_AD_TYPE_FLAGS
#define BR_EDR_NOT_SUPPORTED				BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED
#define MANUF_DATA_TYPE						BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA
#define MANUFACTURER_ID						TEMP_COMPANY_ID
#define MANUF_DATA_LENGTH					11




/* ---------- Local typedefs ---------- */

/* Device serial number integer type */
typedef uint32_t serial_num_int;

/* Device serial number string type */
typedef char serial_num_string[12];

/* Manufacturer data packet format to scan */
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
	SERVICE_ID_BYTE_0_POS,				/* service ID lower byte */
	SERVICE_ID_BYTE_1_POS,				/* service ID higher byte */
	DATA_BYTE_0_POS,					/* data byte 0 */
	DATA_BYTE_1_POS,					/* data byte 1 */
	DATA_BYTE_2_POS,					/* data byte 2 */
	DATA_BYTE_3_POS,					/* data byte 3 */
	DATA_BYTE_4_POS,					/* data byte 4 */
	DATA_BYTE_5_POS,					/* data byte 5 */
	DATA_BYTE_6_POS,					/* data byte 6 */
	DATA_BYTE_7_POS,					/* data byte 7 */
	CALIB_RSSI_POS,						/* calibrated RSSI */
	ADV_DATA_PACKET_LENGTH				/* Manufacturer data adv packet length. This is not included. It is for fw purpose only */
} manuf_data_packet_e;




/* ------------------ Local macros ----------------------- */

#ifdef FACE_INDEX_TEST
/* Advertisement timer for test update */
APP_TIMER_DEF(adv_timer);
#endif

/* Define a pointer type to the device serial number stored in the UICR */
#define UICR_DEVICE_SERIAL_NUM			(*((serial_num_int *)(NRF_UICR_BASE + UICR_CUSTOMER_RESERVED_OFFSET)))




/* ---------- Local const variables ---------- */

/* Default manufaturer data packet. This string represent the  fixed part of the adv packet */
static const uint8_t adv_data_packet[ADV_DATA_PACKET_LENGTH] = 
{
	0x02,								/* first length */
	ADV_FLAGS_TYPE,						/* adv flags type */
	BR_EDR_NOT_SUPPORTED,				/* BR/EDR not supported */
	(uint8_t)(MANUF_DATA_LENGTH + 4),	/* second length */
	MANUF_DATA_TYPE,					/* manufacturer data type */
	(uint8_t)MANUFACTURER_ID,			/* manufacturer ID lower byte */
	(uint8_t)(MANUFACTURER_ID >> 8),	/* manufacturer ID higher byte */
	MANUF_DATA_LENGTH,
	(uint8_t)MANUF_SERVICE_ID,			/* service ID lower byte */
	(uint8_t)(MANUF_SERVICE_ID >> 8),	/* service ID higher byte */
	INITIAL_FACE_INDEX_VALUE,			/* Data 0 */
	0x00,								/* Data 1 */
	0x00,								/* Data 2 */
	0x15,								/* Data 3 */
	0x00,								/* Data 4 */
	0x00,								/* Data 5 */
	0x00,								/* Data 6 */
	0x00,								/* Data 7 */
	TX_POWER_MEASURED_RSSI
};




/* ------------------- Local variables ------------------- */

/* Structure used to identify the Battery Service */
static ble_bas_t m_bas;    

/* Parameters to be passed to the stack when starting advertising */
static ble_gap_adv_params_t m_adv_params;  

/* Structure to identify the CUBE_CFG Service */
static ble_cube_cfg_st m_cube_cfg;                                                                             

/* Handle of the current connection. */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;     

/* BLE UUID fields */
/* ATTENTION: custom CUBE_CFG UUID only */
static ble_uuid_t adv_uuids[] =
{
    {BLE_UUID_CUBE_CFG_SERVICE, CUBE_CFG_SERVICE_UUID_TYPE},
};

#if 0
/* Manufacturer data array */
static uint8_t data_array[MANUF_DATA_PACKET_LENGTH];

/* Array containing advertisement packet data */
static ble_advdata_t ble_adv_data;

/* Array containing scan response packet data */
static ble_advdata_t ble_scan_resp;
#endif

/* Array containing advertisement packet data */
uint8_t adv_data[ADV_DATA_PACKET_LENGTH];

/* Array containing scan response packet data */
uint8_t scan_resp_data[MAX_ADV_LENGTH];

/* Parameters to be passed to the stack when starting advertising */
static uint8_t device_name[] = DEVICE_NAME; 

/* Flag to indicate new RGBW values */
static uint8_t toggle_valid_values = 0x00;




/* ------------------- Local functions prototypes ------------------- */

static void cube_cfg_data_handler		(ble_cube_cfg_st *);
static void on_ble_evt					(ble_evt_t *);
static void ble_evt_dispatch			(ble_evt_t *);
static void services_init				(void);
static void on_conn_params_evt			(ble_conn_params_evt_t *);
static void conn_params_error_handler	(uint32_t);
static void conn_params_init			(void);
static void gap_params_init				(void);
static void advertising_init			(void);        
static void ble_stack_init				(void);
#ifdef FACE_INDEX_TEST
static void timer_handler				(void *);                
#endif




/* ------------------- Local functions ------------------- */

/* dimmer data handler function */
static void cube_cfg_data_handler(ble_cube_cfg_st * p_cube_cfg)
{
	UNUSED_PARAMETER(p_cube_cfg);

	/* do nothing */
}


/* Function for handling the Application's BLE Stack events.
   Parameters:
   - p_ble_evt: Bluetooth stack event. */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
	uint32_t err_code;
	const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;	

    switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_TIMEOUT:
		{
			/* if advertising timeout */
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
				/* ATTENTION: to be implemented. Temporary infinite advertising */
				err_code = sd_ble_gap_adv_start(&m_adv_params);
				APP_ERROR_CHECK(err_code);
			}
			else
			{
				/* do nothing */
			}
		}
        case BLE_GAP_EVT_CONNECTED:
		{
			/* store connection handle */
            m_conn_handle = p_gap_evt->conn_handle;
            break;
		}
        case BLE_GAP_EVT_DISCONNECTED:
		{
			/* reset connection handle */
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

			/* start advertising again */
			err_code = sd_ble_gap_adv_start(&m_adv_params);
    		APP_ERROR_CHECK(err_code);
            break;
		}
		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		{
            /* Pairing not supported */
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
		}
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		{
            /* No system attributes have been stored */
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
		}
		case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
		{
            /* it should not pass here */
            break;
		}
		case BLE_GATTC_EVT_TIMEOUT:	/* this case should not be managed */
        case BLE_GATTS_EVT_TIMEOUT:
		{
            /* Disconnect on GATT Server and Client timeout events. */
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
		}
        default:
		{
            /* No implementation needed. */
            break;
		}
    }
}


/* Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
   This function is called from the BLE Stack event interrupt handler after a BLE stack
   event has been received.
   Parameter:
   - p_ble_evt:  Bluetooth stack event. */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
	ble_cube_cfg_on_ble_evt(&m_cube_cfg, p_ble_evt);  
	on_ble_evt(p_ble_evt);
}


/* Function for initializing services that will be used by the application */
static void services_init(void)
{
	uint32_t err_code;
	ble_bas_init_t    bas_init_obj;
	ble_dis_init_t    dis_init_obj;
	ble_cube_cfg_init_st cube_cfg_init;


	/* init Battery Service */
    memset(&bas_init_obj, 0, sizeof(bas_init_obj));
    /* Here the sec level for the Battery Service can be changed/increased. */
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_report_read_perm);
    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
	/* the battery level will be updated later on application init */
    bas_init_obj.initial_batt_level   = 0;	
	/* init Battery service */
    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);


	/* init Device Information Service */
    memset(&dis_init_obj, 0, sizeof(dis_init_obj));
	/* set device information fields */
	serial_num_string serial_num_ascii;
	ble_srv_ascii_to_utf8(&dis_init_obj.hw_rev_str, HW_REVISION);
	ble_srv_ascii_to_utf8(&dis_init_obj.fw_rev_str, FW_REVISION);
    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
	/* set serial number from the UICR */
	sprintf(serial_num_ascii, "%d", (unsigned int)UICR_DEVICE_SERIAL_NUM);
	ble_srv_ascii_to_utf8(&dis_init_obj.serial_num_str, serial_num_ascii);
/*
	ble_srv_utf8_str_t 	manufact_name_str
	ble_srv_utf8_str_t 	model_num_str
	ble_srv_utf8_str_t 	serial_num_str
	ble_srv_utf8_str_t 	hw_rev_str
	ble_srv_utf8_str_t 	fw_rev_str
	ble_srv_utf8_str_t 	sw_rev_str
	ble_dis_sys_id_t * 	p_sys_id
	ble_dis_reg_cert_data_list_t * 	p_reg_cert_data_list
	ble_dis_pnp_id_t * 	p_pnp_id
	ble_srv_security_mode_t 	dis_attr_md
*/
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);
	/* init Device Info service */
    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);


    /* init CUBE_CFG service */
    memset(&cube_cfg_init, 0, sizeof(cube_cfg_init));
	/* set CUBE_CFG data handler */
    cube_cfg_init.data_handler = cube_cfg_data_handler;
    /* init CUBE_CFG service */
    err_code = ble_cube_cfg_init(&m_cube_cfg, &cube_cfg_init);
    APP_ERROR_CHECK(err_code);
}


/* Function for handling the Connection Parameters Module.
   This function will be called for all events in the Connection Parameters Module which
   are passed to the application.
   All this function does is to disconnect. This could have been done by simply
   setting the disconnect_on_fail config parameter, but instead we use the event
   handler mechanism to demonstrate its use.
   Parameters:
   - p_evt: Event received from the Connection Parameters Module. */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/* Function for handling a Connection Parameters error.
   Parameters:
   - nrf_error: Error code containing information about what went wrong. */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/* Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/* Function for the GAP initialization.
   This function sets up all the necessary GAP (Generic Access Profile) parameters of the
   device including the device name, appearance, and the preferred connection parameters. */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* TODO: Use an appearance value matching the application's use case. */
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);
    APP_ERROR_CHECK(err_code); 

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/* Function for initializing the Advertising functionality.
   Encodes the required advertising data and passes it to the stack.
   Also builds a structure to be passed to the stack when starting advertising.
*/
static void advertising_init(void)
{
	uint32_t err_code;
	//ble_advdata_manuf_data_t manuf_data;
	//int8_t tx_power_level = TX_POWER_MEASURED_RSSI;

	/* set adv data */
	memcpy((void *)&adv_data[0], (const void *)adv_data_packet, ADV_DATA_PACKET_LENGTH);

	/* set scan response data with shortened device name */
    scan_resp_data[0] = 1 + strlen((const char*)device_name);
    scan_resp_data[1] = BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME;
    memcpy(&scan_resp_data[2], device_name, (scan_resp_data[0] - 1));

    /* set advertising data */
    err_code = sd_ble_gap_adv_data_set((uint8_t const *)adv_data, ADV_DATA_PACKET_LENGTH, (uint8_t const *)scan_resp_data, (scan_resp_data[0] + 1));
	APP_ERROR_CHECK(err_code);

    /* Initialize advertising parameters (used when starting advertising) */
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;	/* Undirected advertisement */
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.p_whitelist = NULL;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;


#if 0
	/* clear and set advertising data */
    memset(&ble_adv_data, 0, sizeof(ble_adv_data));
	/* set name type, appearance, flags and TX power */
    ble_adv_data.name_type = BLE_ADVDATA_NO_NAME;
    ble_adv_data.include_appearance = true;
    ble_adv_data.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    ble_adv_data.p_tx_power_level = &tx_power_level;

	/* Manufacturer specific data */
	memcpy(&data_array[0], (const uint8_t *)&def_manuf_data_packet, MANUF_DATA_PACKET_LENGTH);
	/* set manufacturer specific data */
	manuf_data.data.size = MANUF_DATA_PACKET_LENGTH;
	manuf_data.data.p_data = data_array;
	/* set company identifier */
	manuf_data.company_identifier = TEMP_COMPANY_ID;
	/* set manufacturer specific data structure */
	ble_adv_data.p_manuf_specific_data = &manuf_data;

    /* clear and set scan response data */
	memset(&ble_scan_resp, 0, sizeof(ble_scan_resp));
	/* insert full name in scan resp */
	ble_scan_resp.name_type = BLE_ADVDATA_FULL_NAME;
	/* set UUIDs fields */
    ble_scan_resp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    ble_scan_resp.uuids_complete.p_uuids  = adv_uuids;

	/* set advertising data. Maximum available size is BLE_GAP_ADV_MAX_SIZE */
	err_code = ble_advdata_set(&ble_adv_data, &ble_scan_resp);
	APP_ERROR_CHECK(err_code);

	/* Initialize advertising parameters with defaults values */
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
	/* indirect advertising */
    m_adv_params.type = BLE_GAP_ADV_TYPE_ADV_IND;
	/* No peer address */
    m_adv_params.p_peer_addr = NULL;
	/* Allow scan requests and connect requests from any device */
    m_adv_params.fp = BLE_GAP_ADV_FP_ANY;
	/* No whitelist */
    m_adv_params.p_whitelist = NULL;
	/* Set advertising interval */
	m_adv_params.interval = APP_ADV_INTERVAL;
	/* set advertising timeout */
    m_adv_params.timeout = APP_ADV_TIMEOUT_IN_SECONDS;
#endif

	/* advertising is started in a separated function */
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
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);
 
    /* Enable BLE stack. */
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

	/* Register with the SoftDevice handler module for BLE events. */
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


#ifdef FACE_INDEX_TEST
static uint8_t fake_face_index = 0;
/* Timer timeout function */
static void timer_handler(void * p_context)
{
	uint8_t pwm_values_index;

	/* increment fake face index */
	fake_face_index++;
	if(fake_face_index > 5)
	{
		fake_face_index = 0;
	}

	pwm_values_index = (uint8_t)(BLE_CUBE_CFG_PRESET1_CHAR_POS + (BLE_PRESET_NUM_OF_BYTES * fake_face_index));
	/* update BLE adv packet with RGBW values */
	ble_man_adv_update((uint8_t *)&char_values[pwm_values_index], BLE_PRESET_NUM_OF_BYTES);
}
#endif




/* ------------------- Exported functions ------------------- */

/* Function to update the battery level in the battery service */
void ble_mng_update_batt_level( uint8_t batt_percentage )
{
	/* call the battery service API function to update the battery level */
	uint32_t err_code = ble_bas_battery_level_update(&m_bas, batt_percentage);
	/* if an error occurred */
    if (err_code != NRF_SUCCESS) 
    {
		/* do nothing at the moment */
		/* consider to manage this error in the future */
    }
	else
	{
		/* success: do nothing */
	}
}


/* Function for updating the adv packet */
void ble_man_adv_update(uint8_t *pwm_values, uint8_t length)
{
	uint32_t err_code;
	//ble_advdata_manuf_data_t manuf_data;

	/* TODO: consider to implement a kind of values check */
	if((length > 0) && (length <= BLE_PRESET_NUM_OF_BYTES)
	&& (pwm_values != NULL))
	{
		/* stop advertising */
		err_code = sd_ble_gap_adv_stop();
		APP_ERROR_CHECK(err_code);

		/* toggle the new data flag and copy it into its adv field */
		toggle_valid_values = (uint8_t)(0xFF - toggle_valid_values);
		adv_data[DATA_BYTE_2_POS] = toggle_valid_values;
		/* set new RGBW pwm values */
		adv_data[DATA_BYTE_3_POS] = pwm_values[0];
		adv_data[DATA_BYTE_4_POS] = pwm_values[1];
		adv_data[DATA_BYTE_5_POS] = pwm_values[2];
		adv_data[DATA_BYTE_6_POS] = pwm_values[3];
		/* set new adv data packet */
		err_code = sd_ble_gap_adv_data_set((uint8_t const *)adv_data, ADV_DATA_PACKET_LENGTH, (uint8_t const *)scan_resp_data, (scan_resp_data[0] + 1));
		APP_ERROR_CHECK(err_code);

#if 0
		/* toggle the new data flag and copy it into its adv field */
		toggle_valid_values = (uint8_t)(0xFF - toggle_valid_values);
		data_array[DATA_BYTE_2_POS] = toggle_valid_values;
		/* set new RGBW pwm values */
		data_array[DATA_BYTE_3_POS] = pwm_values[0];
		data_array[DATA_BYTE_4_POS] = pwm_values[1];
		data_array[DATA_BYTE_5_POS] = pwm_values[2];
		data_array[DATA_BYTE_6_POS] = pwm_values[3];

		/* re-set manufacturer data */
		manuf_data.data.size = MANUF_DATA_PACKET_LENGTH;
		manuf_data.data.p_data = data_array;
		/* set company identifier */
		manuf_data.company_identifier = TEMP_COMPANY_ID;
		/* set manufacturer specific data structure */
		ble_adv_data.p_manuf_specific_data = &manuf_data;
		
		/* set new adv data packet */
		err_code = ble_advdata_set(&ble_adv_data, &ble_scan_resp);
		APP_ERROR_CHECK(err_code);
#endif

		/* start advertising again */
		err_code = sd_ble_gap_adv_start(&m_adv_params);
		APP_ERROR_CHECK(err_code);
	}
	else
	{
		/* do nothing */
	}
}


/* Function for starting advertising */
void ble_man_adv_start(void)
{
    uint32_t err_code;

	/* init advertising */
    advertising_init();

	/* start advertising */
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/* Function for application main entry */
void ble_man_init(void)
{
	/* stack BLE stack */
    ble_stack_init();
	gap_params_init();
    services_init();
    conn_params_init();

	nrf_gpio_pin_write(24, 1);

#ifdef FACE_INDEX_TEST
	uint32_t err_code;

	/* init button timer */
	err_code = app_timer_create(&adv_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    APP_ERROR_CHECK(err_code);

	/* start timer */
	err_code = app_timer_start(adv_timer, TRIGGER_TIMER_COUNT, NULL);
	APP_ERROR_CHECK(err_code);
#endif
}


/*
	uint32_t err_code;

	err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
	APP_ERROR_CHECK(err_code);
*/




/* End of file */




