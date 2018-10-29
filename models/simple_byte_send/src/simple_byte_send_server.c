/**************************************************************************
  * @file       : simple_byte_send_server.c
  * @brief      : create a new server model for interact with the client's message
  * @author     : William Nguyen(thnam.nguyen27)
  * @version    : 0.1
  * @history    : created on @20181029
***************************************************************************/

#include "simple_byte_send_server.h"
#include "simple_byte_send_common.h"

#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "nrf_mesh_assert.h"

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void reply_status(const simple_byte_send_server_t *p_server, const access_message_rx_t *p_message, uint8_t msg)
{
    simple_byte_send_msg_status_t status;
    status.present_msg = msg;
    access_message_tx_t reply;
    reply.opcode.opcode = SIMPLE_BYTE_SEND_OPCODE_STATUS;
    reply.opcode.company_id = SIMPLE_BYTE_SEND_COMPANY_ID;
    reply.p_buffer = (const uint8_t *)&status;
    reply.length = sizeof(status);
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    (void)access_model_reply(p_server->model_handle, p_message, &reply);
}

/*****************************************************************************
 * Opcode handler callbacks
 *****************************************************************************/

static void handle_set_cb(access_model_handle_t handle, const access_message_rx_t *p_message, void *p_args)
{
    simple_byte_send_server_t *p_server = p_args;
    NRF_MESH_ASSERT(p_server->set_cb != NULL);

    uint8_t value = ((simple_byte_send_msg_set_t *)p_message->p_data)->msg;
    value = p_server->set_cb(p_server, value);
    reply_status(p_server, p_message, value);
    (void)simple_byte_send_server_status_publish(p_server, value);
}

static void handle_get_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_byte_send_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->get_cb != NULL);
    reply_status(p_server, p_message, p_server->get_cb(p_server));
}

static void handle_set_unreliable_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_byte_send_server_t * p_server = p_args;
    uint8_t value = ((simple_byte_send_msg_set_unreliable_t*)p_message->p_data)->msg;
    value = p_server->set_cb(p_server, value);
    (void)simple_byte_send_server_status_publish(p_server, value);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_VENDOR(SIMPLE_BYTE_SEND_OPCODE_SET,              SIMPLE_BYTE_SEND_COMPANY_ID), handle_set_cb},
    {ACCESS_OPCODE_VENDOR(SIMPLE_BYTE_SEND_OPCODE_GET,              SIMPLE_BYTE_SEND_COMPANY_ID), handle_get_cb},
    {ACCESS_OPCODE_VENDOR(SIMPLE_BYTE_SEND_OPCODE_SET_UNRELIABLE,   SIMPLE_BYTE_SEND_COMPANY_ID), handle_set_unreliable_cb}
};

/*****************************************************************************
 * Public API
 *****************************************************************************/
uint32_t simple_byte_send_server_init(simple_byte_send_server_t * p_server, uint16_t element_index)
{
    if (p_server == NULL ||
        p_server->get_cb == NULL ||
        p_server->set_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params;
    init_params.element_index = element_index;
    init_params.model_id.model_id = SIMPLE_BYTE_SEND_SERVER_MODLE_ID;
    init_params.model_id.company_id = SIMPLE_BYTE_SEND_COMPANY_ID;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_server;
    init_params.publish_timeout_cb = NULL;
    return access_model_add(&init_params, &p_server->model_handle);
}

uint32_t simple_byte_send_server_status_publish(simple_byte_send_server_t *p_server, uint8_t value)
{
    simple_byte_send_msg_status_t status;
    status.present_msg = value;
    access_message_tx_t msg;
    msg.opcode.opcode = SIMPLE_BYTE_SEND_OPCODE_STATUS;
    msg.opcode.company_id = SIMPLE_BYTE_SEND_COMPANY_ID;
    msg.p_buffer = (const uint8_t *)&status;
    msg.length = sizeof(status);
    msg.force_segmented = false;
    msg.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    return access_model_publish(p_server->model_handle, &msg);
}