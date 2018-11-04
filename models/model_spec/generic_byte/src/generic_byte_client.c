/**************************************************************************
  * @file       : generic_byte_client.c
  * @brief      : create a new model client for sending a byte (unsigned char) from client
  * @author     : William Nguyen(thnam.nguyen27) 
  * @version    : 0.2
  * @history    : created on @20181029
***************************************************************************/


#include "generic_byte_client.h"
#include "model_common.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"


/** Opcode Handlers */

static void status_handle(access_model_handle_t handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_byte_client_t * p_client = (generic_byte_client_t *) p_args;
    generic_byte_status_params_t in_data = {0};

    if (p_rx_msg->length == GENERIC_BYTE_STATUS_MINLEN || p_rx_msg->length == GENERIC_BYTE_STATUS_MAXLEN)
    {
        generic_byte_status_msg_pkt_t * p_msg_params_packed = (generic_byte_status_msg_pkt_t *) p_rx_msg->p_data;

        if (p_rx_msg->length == GENERIC_BYTE_STATUS_MINLEN)
        {
            in_data.present_byte = p_msg_params_packed->present_byte;
            in_data.target_byte = p_msg_params_packed->present_byte;
            in_data.remaining_time_ms = 0;
        }
        else
        {
            in_data.present_byte = p_msg_params_packed->present_byte;
            in_data.target_byte = p_msg_params_packed->target_byte;
            in_data.remaining_time_ms = model_transition_time_decode(p_msg_params_packed->remaining_time);
        }

        p_client->settings.p_callbacks->byte_status_cb(p_client, &p_rx_msg->meta_data, &in_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    //{ACCESS_OPCODE_SIG(GENERIC_BYTE_OPCODE_STATUS), status_handle},
      {{GENERIC_BYTE_OPCODE_STATUS, ACCESS_COMPANY_ID_NORDIC}, status_handle}
};

static uint8_t message_set_packet_create(generic_byte_set_msg_pkt_t *p_set, const generic_byte_set_params_t * p_params,
                                      const model_transition_t * p_transition)
{
        p_set->byte = p_params->byte;
        p_set->tid = p_params->tid;

        if (p_transition != NULL)
        {
            p_set->transition_time = model_transition_time_encode(p_transition->transition_time_ms);
            p_set->delay = model_delay_encode(p_transition->delay_ms);
            return GENERIC_BYTE_SET_MAXLEN;
        }
        else
        {
            return GENERIC_BYTE_SET_MINLEN;
        }
}

static void message_create(generic_byte_client_t * p_client, uint16_t tx_opcode, const uint8_t * p_buffer,
                           uint16_t length, access_message_tx_t *p_message)
{
    p_message->opcode.opcode = tx_opcode;
    p_message->opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    p_message->p_buffer = p_buffer;
    p_message->length = length;
    p_message->force_segmented = p_client->settings.force_segmented;
    p_message->transmic_size = p_client->settings.transmic_size;
    p_message->access_token = nrf_mesh_unique_token_get();
}

static void reliable_context_create(generic_byte_client_t * p_client, uint16_t reply_opcode,
                                    access_reliable_t * p_reliable)
{
    p_reliable->model_handle = p_client->model_handle;
    p_reliable->reply_opcode.opcode = reply_opcode;
    p_reliable->reply_opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    p_reliable->timeout = p_client->settings.timeout;
    p_reliable->status_cb = p_client->settings.p_callbacks->ack_transaction_status_cb;
}

/** Interface functions */

uint32_t generic_byte_client_init(generic_byte_client_t * p_client, uint8_t element_index)
{
    if (p_client == NULL ||
        p_client->settings.p_callbacks == NULL ||
        p_client->settings.p_callbacks->byte_status_cb == NULL ||
        p_client->settings.p_callbacks->periodic_publish_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_client->settings.timeout == 0)
    {
        p_client->settings.timeout= MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT;
    }

    access_model_add_params_t add_params =
    {
        .model_id = GENERIC_BYTE_CLIENT_MODEL_ID,        
        .element_index = element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_client,
        .publish_timeout_cb = p_client->settings.p_callbacks->periodic_publish_cb
    };

    uint32_t status = access_model_add(&add_params, &p_client->model_handle);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_client->model_handle);
    }

    return status;
}

uint32_t generic_byte_client_set(generic_byte_client_t * p_client, const generic_byte_set_params_t * p_params,
                                  const model_transition_t * p_transition)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_transition != NULL &&
        (p_transition->transition_time_ms > TRANSITION_TIME_MAX_MS ||
         p_transition->delay_ms > DELAY_TIME_MAX_MS))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length = message_set_packet_create(&p_client->msg_pkt.set, p_params, p_transition);

        message_create(p_client, GENERIC_BYTE_OPCODE_SET, (const uint8_t *) &p_client->msg_pkt.set,
                       server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, GENERIC_BYTE_OPCODE_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t generic_byte_client_set_unack(generic_byte_client_t * p_client, const generic_byte_set_params_t * p_params,
                                        const model_transition_t * p_transition, uint8_t repeats)
{
    uint32_t status = NRF_ERROR_INTERNAL;

    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_transition != NULL &&
        (p_transition->transition_time_ms > TRANSITION_TIME_MAX_MS ||
         p_transition->delay_ms > DELAY_TIME_MAX_MS))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    uint8_t server_msg_length = message_set_packet_create(&p_client->msg_pkt.set, p_params, p_transition);

    message_create(p_client, GENERIC_BYTE_OPCODE_SET_UNACKNOWLEDGED,
                   (const uint8_t *) &p_client->msg_pkt.set, server_msg_length,
                   &p_client->access_message.message);

    for (uint32_t i = 0; i <= repeats; ++i)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
        if (status != NRF_SUCCESS)
        {
            break;
        }
    }
    return status;
}

uint32_t generic_byte_client_get(generic_byte_client_t * p_client)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        message_create(p_client, GENERIC_BYTE_OPCODE_GET, NULL, 0, &p_client->access_message.message);
        reliable_context_create(p_client, GENERIC_BYTE_OPCODE_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

