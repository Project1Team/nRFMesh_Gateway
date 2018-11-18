/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "sdk_config.h"
#include "mesh_adv.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"
#include "mesh_softdevice_init.h"

/* Models */
#include "generic_byte_client.h"
#include "generic_byte_client.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Project specific includes */
#include "app_config.h"
#include "nrf_mesh_config_thesis.h"
#include "gateway_model_common.h"
#include "thesis_common.h"
#include "nrf_gpio.h"

/* TWI specific includes */
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_twi_mngr.h"
#include "lm75b.h"
#include "compiler_abstraction.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define TWI_INSTANCE_ID             0
#define MAX_PENDING_TRANSACTIONS    5
// Buffer for data read from sensors.
#define BUFFER_SIZE  11
// Data structures needed for averaging of data read from sensors.
// [max 32, otherwise "int16_t" won't be sufficient to hold the sum
//  of temperature samples]
#define NUMBER_OF_SAMPLES               16

#define APP_UNACK_MSG_REPEAT_COUNT      2

#define DEVICE_NAME                     "Mesh Client Node"
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(150,  UNIT_1_25_MS)           /**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(250,  UNIT_1_25_MS)           /**< Maximum acceptable connection interval. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(100)                        /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called. */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(2000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first call. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define LED_PIN                         (2)
#define CLIENT_LED_0                    (BSP_LED_0)
#define CLIENT_LED_1                    (BSP_LED_1)
#define CLIENT_LED_2                    (BSP_LED_2)
#define CLIENT_LED_3                    (BSP_LED_3)

/* Generic message opcode for clients' data */
#define MSG_OPCODE_FIRE                 (0x46 << 8)                                 /**< ASCII "F"(0x46) for "Fire" */
#define MSG_OPCODE_GAS                  (0x47 << 8)                                 /**< ASCII "G"(0x47) for "Gas" */
#define MSG_OPCODE_TEMP                 (0x54 << 8)                                 /**< ASCII "T"(0x54) for "Temperature" */
#define MSG_OPCODE_SWITCH_ON            (0x50)                                      /**< ASCII "0"(0x4F) + (1) for "ON" */
#define MSG_OPCODE_SWITCH_OFF           (0x4F)                                      /**< ASCII "0"(0x4F) + (0) for "OFF" */



/*------------------------------------TWI-----------------------------------*/
static uint8_t m_buffer[BUFFER_SIZE];
NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);
APP_TIMER_DEF(m_timer);

typedef struct
{
    int16_t temp;
} sum_t;

static sum_t m_sum = { 0 };

typedef struct
{
    // [use bit fields to fit whole structure into one 32-bit word]
    int16_t temp : 11;
} sample_t;

static sample_t m_samples[NUMBER_OF_SAMPLES] = { { 0 } };

static uint8_t m_sample_idx = 0;

#if defined( __GNUC__ ) && (__LINT__ == 0)
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    __ASM(".global _printf_float");
#endif



/*------------------------------------MESH-----------------------------------*/
static void gap_params_init(void);
static void conn_params_init(void);

static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    (void) nrf_mesh_on_sd_evt(sd_evt);
}

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);

/* 4 clients:
/*              1. TEMPERATURE SENSOR
/*              2. FIRE SENSOR
/*              3. GAS SENSOR
/*              4. 4-RELAYS MODULE
*/
static generic_byte_client_t m_clients[CLIENT_NODE_COUNT];

static bool                   m_device_provisioned;

/* Forward declaration */
static void app_gen_byte_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_byte_client_status_cb(const generic_byte_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_byte_status_params_t * p_in);
static void app_gen_byte_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);

const generic_byte_client_callbacks_t client_cbs =
{
    .byte_status_cb = app_generic_byte_client_status_cb,
    .ack_transaction_status_cb = app_gen_byte_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_byte_client_publish_interval_cb
};

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    /* Restores the application parameters after switching from the Provisioning service to the Proxy  */
    gap_params_init();
    conn_params_init();

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_byte_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_gen_byte_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

/* Generic OnOff client model interface: Process the received status message in this callback */
static void app_generic_byte_client_status_cb(const generic_byte_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_byte_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Server: 0x%04x, Message: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_byte, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Server: 0x%04x, Message: %d\n",
              p_meta->src.value, p_in->present_byte);

        /* TODO FIRE and GAS sensor ----> THONG will do this */
        /* Just for testing receive on onff msg from server*/
        if (p_in->present_byte == MSG_OPCODE_SWITCH_ON)
        {
            nrf_gpio_cfg_output(LED_PIN);
            nrf_gpio_pin_set(LED_PIN);
        }
        if (p_in->present_byte == MSG_OPCODE_SWITCH_OFF)
        {
            nrf_gpio_cfg_output(LED_PIN);
            nrf_gpio_pin_clear(LED_PIN);
        }
    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}
static void input_event_handler(uint32_t input_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Input %u received\n", input_number);
    switch(input_number)
    {
        case 0:
            // updatre received data
             hal_led_pin_set(CLIENT_LED_0, !hal_led_pin_get(CLIENT_LED_0));
            break;
        case 1:
            // update received data
            hal_led_pin_set(CLIENT_LED_1, !hal_led_pin_get(CLIENT_LED_1));
            break;
    }
}
static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    uint32_t status = NRF_SUCCESS;
    generic_byte_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    switch(button_number)
    {
        case 0:
            set_params.byte = ((uint16_t)(MSG_OPCODE_TEMP)) | ((uint16_t)(123));
            break;
    }

    set_params.tid = tid++;
    transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
    transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: %d\n", set_params.byte);

    switch (button_number)
    {
        case 0:
            (void)access_model_reliable_cancel(m_clients[1].model_handle);
            status = generic_byte_client_set(&m_clients[1], &set_params, &transition_params);
            hal_led_pin_set(CLIENT_LED_0, !hal_led_pin_get(CLIENT_LED_0));
            break;
      }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", button_number);
            hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void rtt_input_handler(int key)
{
    if (key >= '0' && key <= '3')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    
    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_CONFIG_MIC_SIZE;

        ERROR_CHECK(generic_byte_client_init(&m_clients[i], i + 1));
    }
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
    else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully updated connection parameters\n");
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
    uint8_t node_uuid_prefix[NODE_UUID_PREFIX_LEN] = CLIENT_NODE_UUID_PREFIX;

    ERROR_CHECK(mesh_app_uuid_gen(dev_uuid, node_uuid_prefix, NODE_UUID_PREFIX_LEN));
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = dev_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t  gap_conn_params;

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = &gap_conn_params;
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

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Client -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

    //ERROR_CHECK(hal_buttons_init(button_event_handler));
    ERROR_CHECK(hal_inputs_init(input_event_handler));
    uint32_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
#if defined S140 // todo remove that after S140 priority fixing
    softdevice_irq_priority_checker();
#endif

    uint32_t ram_start = 0;
    /* Set the default configuration (as defined through sdk_config.h). */
    err_code = nrf_sdh_ble_default_cfg_set(MESH_SOFTDEVICE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    gap_params_init();
    conn_params_init();

    mesh_init();
}

static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .p_device_uri = NULL
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

/*------------------------------------TWI-----------------------------------*/
void read_all_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        //NRF_LOG_WARNING("read_all_cb - error: %d", (int)result);
        return;
    }

    uint32_t status = NRF_SUCCESS;
    generic_byte_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    sample_t * p_sample = &m_samples[m_sample_idx];
    m_sum.temp -= p_sample->temp;


    uint8_t temp_hi = m_buffer[0];
    uint8_t temp_lo = m_buffer[1];

    p_sample->temp = LM75B_GET_TEMPERATURE_VALUE(temp_hi, temp_lo);

    m_sum.temp += p_sample->temp;


    ++m_sample_idx;
    if (m_sample_idx >= NUMBER_OF_SAMPLES)
    {
        m_sample_idx = 0;
    }

    // Show current average values every time sample index rolls over (for RTC
    // ticking at 32 Hz and 16 samples it will be every 500 ms) or when tilt
    // status changes.
    if (m_sample_idx == 0)
    {
        //NRF_LOG_RAW_INFO("\nTemp: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT((float)((m_sum.temp * 0.125) / NUMBER_OF_SAMPLES)));
        //NRF_LOG_INFO("Temp: %d\n", (int)((m_sum.temp * 0.125) / NUMBER_OF_SAMPLES));
        //set_params.byte = (uint8_t)((m_sum.temp * 0.125) / NUMBER_OF_SAMPLES);

        /* Sending TWO BYTES: TEMPERATURE OPCODE|VALUE */
        set_params.byte = ((uint16_t)(MSG_OPCODE_TEMP)) | ((uint16_t)((uint8_t)((m_sum.temp * 0.125) / NUMBER_OF_SAMPLES)));
        set_params.tid = tid++;
        transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
        transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;

        status = generic_byte_client_set(&m_clients[0], &set_params, &transition_params);

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending Temperature: %d Celsius degrees.\n", set_params.byte);

        switch (status)
        {
            case NRF_SUCCESS:
                break;

            case NRF_ERROR_NO_MEM:
            case NRF_ERROR_BUSY:
            case NRF_ERROR_INVALID_STATE:
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client cannot send temperature\n");
                hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
                hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
                break;

            case NRF_ERROR_INVALID_PARAM:
                /* Publication not enabled for this client. One (or more) of the following is wrong:
                 * - An application key is missing, or there is no application key bound to the model
                 * - The client does not have its publication state set
                 *
                 * It is the provisioner that adds an application key, binds it to the model and sets
                 * the model's publication state.
                 */
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for temperature client\n");
                break;

            default:
                ERROR_CHECK(status);
                break;
        }
    }
}


static void read_all(void)
{
    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static nrf_twi_mngr_transfer_t const transfers[] =
    {
        LM75B_READ_TEMP(&m_buffer[0])
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = read_all_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));
}


static void bsp_config(void)
{
    uint32_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

// TWI (with transaction manager) initialization.
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
    APP_ERROR_CHECK(err_code);
}

static void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

void timer_handler(void * p_context)
{
    read_all();
}

void read_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    APP_ERROR_CHECK(err_code);
    
    //about half a minute calculated
    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(300), NULL);
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    lfclk_config();
    bsp_config();
    twi_config();
    initialize();
    execution_start(start);
    read_init();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}