/**************************************************************************
  * @file       : app_byte.h
  * @brief      : control server message
  * @author     : William Nguyen(thnam.nguyen27) 
  * @version    : 0.1
  * @history    : created on @20181103
***************************************************************************/

#ifndef APP_BYTE_H__
#define APP_BYTE_H__

#include <stdint.h>

#include "generic_byte_server.h"
#include "app_timer.h"


/**
 * @defgroup APP_BYTE Generic Byte server behaviour
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application level Byte server behavioral structures, functions, and callbacks.
 *
 * This module implements the behavioral requirements of the Generic Byte server model.
 *
 * The application should use the set callback provided by this module to set the hardware state.
 * The hardware state could be changed by reflecting the value provided by the set callback on the GPIO
 * or by sending this value to the connected lighting peripheral using some other interface (e.g.
 * serial interface). Similarly, the application should use the get callback provided by this
 * module to read the hardware state.
 *
 * This module triggers the set callback only when it determins that it is time to inform the user
 * application. It is possible that the client can send multiple overlapping set commands.
 * In such case any transition in progress will be abandoned and fresh transition will be started if
 * required.
 * <br>
 * @warning To comply with the Mesh Model Specification test cases, the application must adhere to the
 * requirements defined in the following sections:
 * - Section 3.1.1 (Generic Byte) and Section 3.3.1.2 (Generic Byte state behaviour) of Mesh
 * Model Specification v1.0.
 * - Section 3.7.6.1 (Publish) of Mesh Profile Specification v1.0.
 *
 * These requirements are documented at appropriate places in the module source code.
 *
 * @{
 */

/**
 * Macro to create application level app_byte_server_t context.
 *
 * Individual timer instances are created for each model instance.
 *
 * @param[in] _name                 Name of the app_byte_server_t instance
 * @param[in] _force_segmented      If the Generic Byte server shall use force segmentation of messages
 * @param[in] _mic_size             MIC size to be used by Generic Byte server
 * @param[in] _set_cb               Callback for setting the application state to given value.
 * @param[in] _get_cb               Callback for reading the state from the application.
*/

#define APP_BYTE_SERVER_DEF(_name, _force_segmented, _mic_size, _set_cb, _get_cb)  \
    APP_TIMER_DEF(_name ## _timer); \
    static app_byte_server_t _name =  \
    {  \
        .server.settings.force_segmented = _force_segmented,  \
        .server.settings.transmic_size = _mic_size,  \
        .p_timer_id = &_name ## _timer,  \
        .byte_set_cb = _set_cb,  \
        .byte_get_cb = _get_cb  \
    };


/** Internal structure to hold state and timing information. */
typedef struct
{
    /** Present value of the Byte state */
    uint8_t present_byte;
    /** Target value of the Byte state, as received from the model interface. */
    uint8_t target_byte;
    /** Remaining time to reach `target_byte`. */
    uint32_t remaining_time_ms;
    /** Time to delay the processing of received SET message. */
    uint32_t delay_ms;
} app_byte_state_t;

/* Forward declaration */
typedef struct __app_byte_server_t app_byte_server_t;

/** Application state set callback prototype.
 *
 * This callback is called by the this module whenever application is required to
 * be informed to reflect the desired Byte value, as a result of the received SET message. Depending
 * on the received Target Byte value and timing parameters, this callback may be triggered after the
 * delay+transition time is over or instantly after the delay if the Target Byte value is `1`, as
 * required by the Mesh Model Specification v1.0.
 *
 * Note: Since the behavioral module encapsulates functionality required for the compliance with timing
 * behaviour, it is not possible to infer number of Generic Byte Set messages received by the
 * node by counting the number of times this callback is triggered.
 *
 * @param[in]   p_server        Pointer to @ref __app_byte_server_t [app_byte_server_t] context
 * @param[in]   byte           New byte value to be used by the application
 */
typedef void (*app_byte_set_cb_t)(const app_byte_server_t * p_server, uint8_t byte);

/** Application state read callback prototype.
 * This callback is called by the app_model_behaviour.c whenever application byte state is required
 * to be read.
 *
 * @param[in]  p_server          Pointer to @ref __app_byte_server_t [app_byte_server_t] context
 * @param[out] p_present_byte   User application fills this value with the value retrived from
 *                               the hardware interface.
 */
typedef void (*app_byte_get_cb_t)(const app_byte_server_t * p_server, uint8_t * p_present_byte);

/** Application level structure holding the Byte server model context and Byte state representation */
struct __app_byte_server_t
{
    /** Byte server model interface context structure */
    generic_byte_server_t server;
    /** APP timer instance pointer */
    app_timer_id_t const * p_timer_id;
    /** Callaback to be called for informing the user application to update the value*/
    app_byte_set_cb_t  byte_set_cb;
    /** Callback to be called for requesting current value from the user application */
    app_byte_get_cb_t byte_get_cb;

    /** Internal variable. Representation of the Byte state related data and transition parameters
     *  required for behavioral implementation, and for communicating with the application */
    app_byte_state_t state;
    /** Internal variable. It is used for acquiring RTC counter value. */
    uint32_t last_rtc_counter;
    /** Internal variable. To flag if the received message has been processed to update the present
     * Byte value */
    uint8_t value_updated;
};

/** Initiates value fetch from the user application by calling a get callback, updates internal state,
 * and publishes the Generic Byte Status message.
 *
 * This API must always be called by an application when user initiated action (e.g. button press) results
 * in the local Byte state change. Mesh Profile Specification v1.0 mandates that, every local state
 * change must be published if model publication state is configured. If model publication is not
 * configured this API call will not generate any error condition.
 *
 * @param[in] p_server              Pointer to @ref __app_byte_server_t [app_byte_server_t] context
 */
void app_byte_status_publish(app_byte_server_t * p_server);

/** Initializes the behavioral module for the generic Byte model
 *
 * @param[in] p_server               Pointer to the application Byte server struture array.
 * @param[in] element_index          Element index on which this server will be instantiated.
 *
 * @retval  NRF_ERROR_NULL           NULL pointer is supplied to the function or to the required
 *                                   member variable pointers.
 * @retval  NRF_ERROR_INVALID_PARAM  If value of the `server_count` is zero, or other parameters
 *                                   required by lower level APIs are not correct.
 * @returns Other return values returned by the lower layer APIs.
 *
*/
uint32_t app_byte_init(app_byte_server_t * p_server, uint8_t element_index);

/** @} end of APP_BYTE */
#endif /* APP_BYTE_H__ */
