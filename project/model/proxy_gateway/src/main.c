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

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_thesis.h"
#include "gateway_model_common.h"
#include "thesis_common.h"

/* UART specific includes */
#include "app_uart.h"
#include "nrf_uart.h"
#include "nrf_delay.h"

#define CLIENT_LED_VALUE                (0)

#define APP_UNACK_MSG_REPEAT_COUNT      (2)

#define DEVICE_NAME                     "Gateway"
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(150,  UNIT_1_25_MS)         /**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(250,  UNIT_1_25_MS)         /**< Maximum acceptable connection interval. */
#define SLAVE_LATENCY                   0                                         /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)           /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(100)                      /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called. */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(2000)                     /**< Time between each call to sd_ble_gap_conn_param_update after the first call. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                         /**< Number of attempts before giving up the connection parameter negotiation. */

/* Generic message opcode for SWITCH_NODE_1 (0x0) */
#define MSG_OPCODE_SWITCH_ON_1          (0x0301)                                   /**< ASCII "0"(0x30) + "1" for "ON-1"    */
#define MSG_OPCODE_SWITCH_ON_2          (0x0302)                                   /**< ASCII "0"(0x30) + "2" for "ON-2"    */
#define MSG_OPCODE_SWITCH_ON_3          (0x0303)                                   /**< ASCII "0"(0x30) + "3" for "ON-3"    */
#define MSG_OPCODE_SWITCH_ON_4          (0x0304)                                   /**< ASCII "0"(0x30) + "4" for "ON-4"    */

#define MSG_OPCODE_SWITCH_OFF_1         (0x0311)                                   /**< ASCII "1"(0x31) + "1" for "OFF-1"    */
#define MSG_OPCODE_SWITCH_OFF_2         (0x0312)                                   /**< ASCII "1"(0x31) + "2" for "OFF-2"    */
#define MSG_OPCODE_SWITCH_OFF_3         (0x0313)                                   /**< ASCII "1"(0x31) + "3" for "OFF-3"    */
#define MSG_OPCODE_SWITCH_OFF_4         (0x0314)                                   /**< ASCII "1"(0x31) + "4" for "OFF-4"    */

/* Generic message opcode for SWITCH_NODE_2 (0x1) */
#define MSG_OPCODE_SWITCH_ON_5          (0x1301)                                   /**< ASCII "0"(0x30) + "1" for "ON-1"    */
#define MSG_OPCODE_SWITCH_ON_6          (0x1302)                                   /**< ASCII "0"(0x30) + "2" for "ON-2"    */
#define MSG_OPCODE_SWITCH_ON_7          (0x1303)                                   /**< ASCII "0"(0x30) + "3" for "ON-3"    */
#define MSG_OPCODE_SWITCH_ON_8          (0x1304)                                   /**< ASCII "0"(0x30) + "4" for "ON-4"    */

#define MSG_OPCODE_SWITCH_OFF_5         (0x1311)                                   /**< ASCII "1"(0x31) + "1" for "OFF-1"    */
#define MSG_OPCODE_SWITCH_OFF_6         (0x1312)                                   /**< ASCII "1"(0x31) + "2" for "OFF-2"    */
#define MSG_OPCODE_SWITCH_OFF_7         (0x1313)                                   /**< ASCII "1"(0x31) + "3" for "OFF-3"    */
#define MSG_OPCODE_SWITCH_OFF_8         (0x1314)                                   /**< ASCII "1"(0x31) + "4" for "OFF-4"    */

#define MSG_OPCODE_CLEAR_FIRE           (0x4600)                                   /** Clear Fire State */
#define MSG_OPCODE_CLEAR_GAS            (0x4700)                                   /** Clear Gas State */


/* UART specific includes */
#define UART_TX_BUF_SIZE 256                                                      /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                                                      /**< UART RX buffer size. */


static uint16_t received_data = 0x0000;

static int count_value = 0;

static void gap_params_init(void);
static void conn_params_init(void);

static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    (void) nrf_mesh_on_sd_evt(sd_evt);
}

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);

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

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t cr;
    uint32_t err_code;
    uint32_t status = NRF_SUCCESS;
    generic_byte_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            err_code = app_uart_get(&cr);
            
            do
            {
                //err_code = app_uart_put(cr);
                if(count_value > 0)
                {
                   received_data = (received_data << 8) | (uint16_t)(cr);
                   count_value ++;
                }

                if(count_value == 3)
                {
                   // sent data to device
                    set_params.byte = received_data;
                    set_params.tid = tid++;
                    transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
                    transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: %d\n", set_params.byte);

                    // Fire clear state
                    if (received_data == MSG_OPCODE_CLEAR_FIRE) 
                    {
                        (void)access_model_reliable_cancel(m_clients[0].model_handle);
                        status = generic_byte_client_set_unack(&m_clients[0], &set_params, &transition_params, 1);
                    }

                    // Gas clear state
                    if (received_data == MSG_OPCODE_CLEAR_GAS)
                    {
                        (void)access_model_reliable_cancel(m_clients[1].model_handle);
                        status = generic_byte_client_set_unack(&m_clients[1], &set_params, &transition_params, 1);
                    }

                    // Switch 1 Controller
                    if ((received_data == MSG_OPCODE_SWITCH_ON_1)  || (received_data == MSG_OPCODE_SWITCH_ON_2)  ||
                        (received_data == MSG_OPCODE_SWITCH_ON_3)  || (received_data == MSG_OPCODE_SWITCH_ON_4)  ||
                        (received_data == MSG_OPCODE_SWITCH_OFF_1) || (received_data == MSG_OPCODE_SWITCH_OFF_2) ||
                        (received_data == MSG_OPCODE_SWITCH_OFF_3) || (received_data == MSG_OPCODE_SWITCH_OFF_4)) 
                    {
                        (void)access_model_reliable_cancel(m_clients[2].model_handle);
                        status = generic_byte_client_set_unack(&m_clients[2], &set_params, &transition_params, 1);
                    }
                    
                    // Switch 2 Controller
                    if ((received_data == MSG_OPCODE_SWITCH_ON_5)  || (received_data == MSG_OPCODE_SWITCH_ON_6)  ||
                        (received_data == MSG_OPCODE_SWITCH_ON_7)  || (received_data == MSG_OPCODE_SWITCH_ON_8)  ||
                        (received_data == MSG_OPCODE_SWITCH_OFF_5) || (received_data == MSG_OPCODE_SWITCH_OFF_6) ||
                        (received_data == MSG_OPCODE_SWITCH_OFF_7) || (received_data == MSG_OPCODE_SWITCH_OFF_8)) 
                    {
                        (void)access_model_reliable_cancel(m_clients[3].model_handle);
                        status = generic_byte_client_set_unack(&m_clients[3], &set_params, &transition_params, 1);
                    }

                    count_value = 0; 
                }
                if(cr == 0xFE)
                {
                    count_value ++;
                }
                hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
                hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
                set_params.byte = cr;
                //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending message to client... \n");
            } while (err_code == NRF_ERROR_BUSY);
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Gateway cannot send\n");
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for Gateway\n");
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}


/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = NRF_UART_BAUDRATE_115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
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
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Device: 0x%04x, Message: %d\n",
              p_meta->src.value, p_in->present_byte);
    int i;
    uint16_t data = p_in->present_byte;
    uint8_t start_byte = 0xFE;

    app_uart_put(start_byte);
    for(i = 0; i < 2; i++)
    {
        app_uart_put((uint8_t)(data & 0x00FF));
        data = data >> 8;
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
            set_params.byte = 65535;
        break;
    }

    set_params.tid = tid++;
    transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
    transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: ONOFF SET %d\n", set_params.byte);

    switch (button_number)
    {
        case 0:
            (void)access_model_reliable_cancel(m_clients[0].model_handle);
            status = generic_byte_client_set(&m_clients[0], &set_params, &transition_params);
            hal_led_pin_set(BSP_LED_0, CLIENT_LED_VALUE);
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
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
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
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Gateway-----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif
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

int main(void)
{
    nrf_delay_ms(2000);
    uart_init();
    initialize();
    execution_start(start);

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
