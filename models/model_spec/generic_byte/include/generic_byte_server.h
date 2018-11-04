/**************************************************************************
  * @file       : generic_byte_server.h
  * @brief      : create a new server model for interact with the client's message
  * @author     : William Nguyen(thnam.nguyen27)
  * @version    : 0.2
  * @history    : created on @20181029
***************************************************************************/

#ifndef GENERIC_BYTE_SERVER_H__
#define GENERIC_BYTE_SERVER_H__

#include <stdint.h>
#include "access.h"
#include "generic_byte_common.h"
#include "model_common.h"

/**
 * @defgroup GENERIC_BYTE_SERVER Generic Byte server model interface
 * @ingroup GENERIC_BYTE_MODEL
 * @{
 */

/** Server model ID */
#define GENERIC_BYTE_SERVER_MODEL_ID 0x1000

/* Forward declaration */
typedef struct __generic_byte_server_t generic_byte_server_t;

/**
 * Callback type for Generic Byte Set/Set Unacknowledged message
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input parameters for the user application
 * @param[in]     p_in_transition          Pointer to transition parameters, if present in the incoming message,
 *                                         otherwise set to null.
 * @param[out]    p_out                    Pointer to store the output parameters from the user application.
 *                                         If null, indicates that it is UNACKNOWLEDGED message and no
 *                                         output params are required.
 */
typedef void (*generic_byte_state_set_cb_t)(const generic_byte_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             const generic_byte_set_params_t * p_in,
                                             const model_transition_t * p_in_transition,
                                             generic_byte_status_params_t * p_out);

/**
 * Callback type for Generic Byte Get message
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[out]    p_out                    Pointer to store the output parameters from the user application
 */
typedef void (*generic_byte_state_get_cb_t)(const generic_byte_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             generic_byte_status_params_t * p_out);

/**
 * Transaction callbacks for the Byte state
 */
typedef struct
{
    generic_byte_state_set_cb_t    set_cb;
    generic_byte_state_get_cb_t    get_cb;
} generic_byte_server_state_cbs_t;

/**
 * Byte server callback list
 */
typedef struct
{
    /** Callbacks for the Byte state */
    generic_byte_server_state_cbs_t byte_cbs;
} generic_byte_server_callbacks_t;

/**
 * User provided settings and callbacks for the model instance
 */
typedef struct
{
    /** If server should force outgoing messages as segmented messages */
    bool force_segmented;
    /** TransMIC size used by the outgoing server messages. See @ref nrf_mesh_transmic_size_t */
    nrf_mesh_transmic_size_t transmic_size;

    /** Callback list */
    const generic_byte_server_callbacks_t * p_callbacks;
} generic_byte_server_settings_t;

/**  */
struct __generic_byte_server_t
{
    /** Model handle assigned to this instance */
    access_model_handle_t model_handle;
    /** Tid tracker structure */
    tid_tracker_t tid_tracker;

    /** Model settings and callbacks for this instance */
    generic_byte_server_settings_t settings;
};

/**
 * Initializes Generic Byte server.
 *
 * @note This function should only be called _once_.
 * @note The server handles the model allocation and adding.
 *
 * @param[in]     p_server             Generic Byte server context pointer.
 * @param[in]     element_index            Element index to add the model
 *
 */
uint32_t generic_byte_server_init(generic_byte_server_t * p_server, uint8_t element_index);

/**
 * Publishes unsolicited Status message
 *
 * This API can be used to send unsolicited messages to report updated state value as a result of local action.
 *
 * @param[in]     p_server                 Status server context pointer.
 * @param[in]     p_params                 Message parameters
 *
 */
uint32_t generic_byte_server_status_publish(generic_byte_server_t * p_server, const generic_byte_status_params_t * p_params);

/**@} end of GENERIC_BYTE_SERVER */
#endif /* GENERIC_BYTE_SERVER_H__ */
