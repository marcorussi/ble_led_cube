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


/* 
	ATTENTION:
	LIGHT characteristic values are stored and have default values but are not used.
   	They are only read upon reception of new values. Then they are stored again just for logging. 
*/


/*
	TODO: consider to check min and max values. Defines are already defined.
*/


/* ------------- Inclusions --------------- */

#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "ble_srv_common.h"
#include "memory.h"
#include "application.h"

#include "cfg_service.h"




/* ------------- Local definitions --------------- */

/* The UUID of the PRESET1 Characteristic */
#define BLE_UUID_CUBE_CFG_PRESET1_CHAR				0x0005 

/* The UUID of the PRESET2 Characteristic */
#define BLE_UUID_CUBE_CFG_PRESET2_CHAR				0x000A   

/* The UUID of the SPECIAL_OP Characteristic */
#define BLE_UUID_CUBE_CFG_SPECIAL_OP_CHAR			0x000F   

/* User vendor specific UUID */
#define CUBE_CFG_BASE_UUID                  		{{0x8A, 0xAF, 0xA6, 0xC2, 0x3A, 0x32, 0x8F, 0x84, 0x75, 0x4F, 0xF3, 0x02, 0x01, 0x50, 0x65, 0x20}} 

/* Minimum and maximum range values for LIGHT timeout */
#define MIN_PWM_VALUE_PERCENT						0
#define MAX_PWM_VALUE_PERCENT						100




/* ------------- Exported variables --------------- */

uint8_t char_values[BLE_CUBE_CFG_SERVICE_CHARS_LENGTH];




/* ------------- Local functions prototypes --------------- */

static void 	on_connect(ble_cube_cfg_st *, ble_evt_t *);
static void 	on_disconnect(ble_cube_cfg_st *, ble_evt_t *);
static void 	on_write(ble_cube_cfg_st *, ble_evt_t *);
static uint32_t	char_add(ble_cube_cfg_st *, ble_gatts_char_handles_t *, bool, uint8_t, uint16_t, uint8_t *);




/* ------------- Local functions --------------- */

/* Function for handling the BLE_GAP_EVT_CONNECTED event from the SoftDevice */
static void on_connect(ble_cube_cfg_st * p_cube_cfg, ble_evt_t * p_ble_evt)
{
	/* store connection handle */
    p_cube_cfg->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/* Function for handling the BLE_GAP_EVT_DISCONNECTED event from the SoftDevice */
static void on_disconnect(ble_cube_cfg_st * p_cube_cfg, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
	/* reset connection handle */
    p_cube_cfg->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/* Function for handling the BLE_GATTS_EVT_WRITE event from the SoftDevice */
static void on_write(ble_cube_cfg_st * p_cube_cfg, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	/* if data pointer is valid and length is greater than 0 */
	if((p_evt_write->data != NULL)
    && (p_evt_write->len > 0))
	{
		if(p_evt_write->handle == p_cube_cfg->preset1_charac_handles.value_handle)
		{
			/* TODO: consider to send memory result to upper layers */
			/* update field in the persistent memory, just for log */
			memory_update_field(BLE_CUBE_CFG_PRESET1_CHAR_POS, p_evt_write->data, p_evt_write->len);

			/* call previously registered callback function for CMD */
			p_cube_cfg->data_handler(p_cube_cfg);
		}
		else if(p_evt_write->handle == p_cube_cfg->preset2_charac_handles.value_handle)
		{
			/* TODO: consider to send memory result to upper layers */
			/* update field in the persistent memory */
			memory_update_field(BLE_CUBE_CFG_PRESET2_CHAR_POS, p_evt_write->data, p_evt_write->len);
		}
		else if(p_evt_write->handle == p_cube_cfg->special_op_charac_handles.value_handle)
		{
			/* if length is 1 */
			if(p_evt_write->len == 1)
			{
				/* manage the received value for SPECIAL_OP in application module */
				app_special_op(((uint8_t)(*p_evt_write->data)));
			}
			else
			{
				/* discard the received data */
			}
		}
		else
		{
			/* handle not found: do nothing */
		}
	}
	else
	{
		/* discard it */
	}
}


/* Function for adding same type characteristic */
static uint32_t char_add(	ble_cube_cfg_st * p_cube_cfg, 
							ble_gatts_char_handles_t * p_char_handle, 
							bool read_enabled, 
							uint8_t char_value_length, 
							uint16_t char_uuid,
							uint8_t *p_value)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

	/* if read feature is requested */
	if(true == read_enabled)
	{
		/* set read flag */
		char_md.char_props.read = 1;
	}
	else
	{
		/* do not set read flag */
	}
    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_cube_cfg->uuid_type;
    ble_uuid.uuid = char_uuid;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_USER;	/* ATTENTION: user must provide a valid global variable */
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = char_value_length;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = char_value_length;
	attr_char_value.p_value   = p_value;

    return sd_ble_gatts_characteristic_add(p_cube_cfg->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           p_char_handle);
}




/* ------------- Exported functions --------------- */

/* Function for handling received events for CUBE_CFG service */
void ble_cube_cfg_on_ble_evt(ble_cube_cfg_st * p_cube_cfg, ble_evt_t * p_ble_evt)
{
    if ((p_cube_cfg == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cube_cfg, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cube_cfg, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_cube_cfg, p_ble_evt);
            break;

        default:
            /* No implementation needed. */
            break;
    }
}


/* Function to init CUBE_CFG service */
uint32_t ble_cube_cfg_init(ble_cube_cfg_st * p_cube_cfg, const ble_cube_cfg_init_st * p_cube_cfg_init)
{
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t cube_cfg_base_uuid = CUBE_CFG_BASE_UUID;

    if ((p_cube_cfg == NULL) || (p_cube_cfg_init == NULL))
    {
        return NRF_ERROR_NULL;
    }

    /* Initialize the service structure */
    p_cube_cfg->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_cube_cfg->data_handler            = p_cube_cfg_init->data_handler;

    /* Adding proprietary Service to SoftDevice] */
    err_code = sd_ble_uuid_vs_add(&cube_cfg_base_uuid, &p_cube_cfg->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ble_uuid.type = p_cube_cfg->uuid_type;
    ble_uuid.uuid = BLE_UUID_CUBE_CFG_SERVICE;

    /* Adding proprietary Service to SoftDevice */
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_cube_cfg->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /* Add the PRESET1 Characteristic - Read/Write */
    err_code = char_add(p_cube_cfg, 
						&p_cube_cfg->preset1_charac_handles, 
						true, 
						BLE_CUBE_CFG_PRESET1_CHAR_LENGTH, 
						BLE_UUID_CUBE_CFG_PRESET1_CHAR, 
						(uint8_t *)&char_values[BLE_CUBE_CFG_PRESET1_CHAR_POS]);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	/* Add the PRESET2 Characteristic - Read/Write */
    err_code = char_add(p_cube_cfg, 
						&p_cube_cfg->preset2_charac_handles, 
						true, 
						BLE_CUBE_CFG_PRESET2_CHAR_LENGTH, 
						BLE_UUID_CUBE_CFG_PRESET2_CHAR, 
						(uint8_t *)&char_values[BLE_CUBE_CFG_PRESET2_CHAR_POS]);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	/* Add the SPECIAL_OP Characteristic - Write */
	/* ATTENTION: consider to use a stack location for this characteristic.
       The same user buffer of preset chars is used at the moment. */
    err_code = char_add(p_cube_cfg, 
						&p_cube_cfg->special_op_charac_handles, 
						false, 
						BLE_CUBE_CFG_SPECIAL_OP_CHAR_LENGTH, 
						BLE_UUID_CUBE_CFG_SPECIAL_OP_CHAR, 
						(uint8_t *)&char_values[BLE_CUBE_CFG_SPECIAL_OP_CHAR_POS]);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	return NRF_SUCCESS;
}




/* End of file */


