/**************************************************************************
  * @file       : generic_byte_common.h
  * @brief      : define some info for generic_byte model
  * @author     : William Nguyen(thnam.nguyen27)
  * @version    : 0.2
  * @history    : created on @20181029
***************************************************************************/

#ifndef GENERIC_BYTE_COMMON_H__
#define GENERIC_BYTE_COMMON_H__

#include <stdint.h>
#include "model_common.h"

/**
 * @defgroup GENERIC_BYTE_MODEL Generic Byte model
 * @ingroup MESH_API_GROUP_GENERIC_MODELS
 * This model implements the message based interface required to set the Byte value on the server.
 * Server model itself is a stateless model. The state information resides in the user application.
 * This interface API takes care of validating the packet formats and field values.
 * These APIs should be used in combination with necessary behavioral implementation to create
 * a qualifiable model implementation.
 * @{
 */

/** Model Company ID */
#define GENERIC_BYTE_COMPANY_ID (ACCESS_COMPANY_ID_NONE)

/** Maximum value of the Byte state, as defined in the Mesh Model Specification v1.0 */
#define GENERIC_BYTE_MAX        (0xFFFF)

/**
 * Unpacked message structure typedefs are used for API interfaces and for implementing model code. This helps to minimize code
 * footprint.
 */

/** Structure containing value of the Byte state */
typedef struct
{
    uint16_t byte;                                            /**< State to set */
    uint8_t tid;                                            /**< Transaction ID */
} generic_byte_state_t;

/** Mandatory parameters for the Generic Byte Set message. */
typedef struct
{
    uint16_t byte;                                            /**< State to set */
    uint8_t tid;                                            /**< Transaction ID */
} generic_byte_set_params_t;

/** Parameters for the Generic Byte Status message. */
typedef struct
{
    uint16_t present_byte;                                 /**< The present value of the Generic Byte state */
    uint32_t remaining_time_ms;                             /**< Remaining time value in milliseconds */
} generic_byte_status_params_t;

/**@} end of GENERIC_BYTE_MODEL */
#endif /* GENERIC_BYTE_COMMON_H__ */
