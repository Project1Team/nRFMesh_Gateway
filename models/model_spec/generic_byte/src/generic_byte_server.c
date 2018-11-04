/**************************************************************************
  * @file       : generic_byte_server.c
  * @brief      : create a new server model for interact with the client's message
  * @author     : William Nguyen(thnam.nguyen27)
  * @version    : 0.2
  * @history    : created on @20181029
***************************************************************************/


#include "generic_byte_server.h"
#include "generic_byte_common.h"
#include "generic_byte_messages.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"


static uint32_t status_send(generic_byte_server_t * p_server,
                            const access_message_rx_t * p_message,
                            const generic_byte_status_params_t * p_params)
{
    generic_byte_status_msg_pkt_t msg_pkt;

    if (p_params->present_byte > GENERIC_BYTE_MAX ||
        p_params->target_byte  > GENERIC_BYTE_MAX ||
        p_params->remaining_time_ms > TRANSITION_TIME_STEP_10M_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    msg_pkt.present_byte = p_params->present_byte;
    if (p_params->remaining_time_ms > 0)
    {
        msg_pkt.target_byte = p_params->target_byte;
        msg_pkt.remaining_time = model_transition_time_encode(p_params->remaining_time_ms);
    }

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(GENERIC_BYTE_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = p_params->remaining_time_ms > 0 ? GENERIC_BYTE_STATUS_MAXLEN : GENERIC_BYTE_STATUS_MINLEN,
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    if (p_message == NULL)
    {
        return access_model_publish(p_server->model_handle, &reply);
    }
    else
    {
        return access_model_reply(p_server->model_handle, p_message, &reply);
    }
}

static void periodic_publish_cb(access_model_handle_t handle, void * p_args)
{
    generic_byte_server_t * p_server = (generic_byte_server_t *)p_args;
    generic_byte_status_params_t out_data = {0};

    p_server->settings.p_callbacks->byte_cbs.get_cb(p_server, NULL, &out_data);
    (void) status_send(p_server, NULL, &out_data);
}

/** Opcode Handlers */

static inline bool set_params_validate(const access_message_rx_t * p_rx_msg, const generic_byte_set_msg_pkt_t * p_params)
{
    return (
            (p_rx_msg->length == GENERIC_BYTE_SET_MINLEN || p_rx_msg->length == GENERIC_BYTE_SET_MAXLEN) &&
            (p_params->byte <= GENERIC_BYTE_MAX)
           );
}

static void handle_set(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_byte_server_t * p_server = (generic_byte_server_t *) p_args;
    generic_byte_set_params_t in_data = {0};
    model_transition_t in_data_tr = {0};
    generic_byte_status_params_t out_data = {0};
    generic_byte_set_msg_pkt_t * p_msg_params_packed = (generic_byte_set_msg_pkt_t *) p_rx_msg->p_data;

    if (set_params_validate(p_rx_msg, p_msg_params_packed))
    {
        in_data.byte = p_msg_params_packed->byte;
        in_data.tid = p_msg_params_packed->tid;

        if (model_tid_validate(&p_server->tid_tracker, &p_rx_msg->meta_data, GENERIC_BYTE_OPCODE_SET, in_data.tid))
        {
            if (p_rx_msg->length == GENERIC_BYTE_SET_MAXLEN)
            {
                if (!model_transition_time_is_valid(p_msg_params_packed->transition_time))
                {
                    return;
                }

                in_data_tr.transition_time_ms = model_transition_time_decode(p_msg_params_packed->transition_time);
                in_data_tr.delay_ms = model_delay_decode(p_msg_params_packed->delay);
            }

            p_server->settings.p_callbacks->byte_cbs.set_cb(p_server,
                                                            &p_rx_msg->meta_data,
                                                            &in_data,
                                                            (p_rx_msg->length == GENERIC_BYTE_SET_MINLEN) ? NULL : &in_data_tr,
                                                            (p_rx_msg->opcode.opcode == GENERIC_BYTE_OPCODE_SET) ? &out_data : NULL);

            if (p_rx_msg->opcode.opcode == GENERIC_BYTE_OPCODE_SET)
            {
                (void) status_send(p_server, p_rx_msg, &out_data);
            }
        }
    }
}

static inline bool get_params_validate(const access_message_rx_t * p_rx_msg)
{
    return (p_rx_msg->length == 0);
}

static void handle_get(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_byte_server_t * p_server = (generic_byte_server_t *) p_args;
    generic_byte_status_params_t out_data = {0};

    if (get_params_validate(p_rx_msg))
    {
        p_server->settings.p_callbacks->byte_cbs.get_cb(p_server, &p_rx_msg->meta_data, &out_data);
        (void) status_send(p_server, p_rx_msg, &out_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
//    {ACCESS_OPCODE_SIG(GENERIC_BYTE_OPCODE_SET), handle_set},
//    {ACCESS_OPCODE_SIG(GENERIC_BYTE_OPCODE_SET_UNACKNOWLEDGED), handle_set},
//    {ACCESS_OPCODE_SIG(GENERIC_BYTE_OPCODE_GET), handle_get},
      {{GENERIC_BYTE_OPCODE_SET, GENERIC_BYTE_COMPANY_ID}, handle_set},
      {{GENERIC_BYTE_OPCODE_SET_UNACKNOWLEDGED, GENERIC_BYTE_COMPANY_ID}, handle_set},
      {{GENERIC_BYTE_OPCODE_GET, GENERIC_BYTE_COMPANY_ID}, handle_set}
};


/** Interface functions */
uint32_t generic_byte_server_init(generic_byte_server_t * p_server, uint8_t element_index)
{
    if (p_server == NULL ||
        p_server->settings.p_callbacks == NULL ||
        p_server->settings.p_callbacks->byte_cbs.set_cb == NULL ||
        p_server->settings.p_callbacks->byte_cbs.get_cb == NULL )
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params =
    {
        //.model_id = ACCESS_MODEL_SIG(GENERIC_BYTE_SERVER_MODEL_ID),
        .model_id = GENERIC_BYTE_SERVER_MODEL_ID,
        .element_index =  element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_server,
        .publish_timeout_cb = periodic_publish_cb
    };

    uint32_t status = access_model_add(&init_params, &p_server->model_handle);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_server->model_handle);
    }

    return status;
}

uint32_t generic_byte_server_status_publish(generic_byte_server_t * p_server, const generic_byte_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_send(p_server, NULL, p_params);
}

