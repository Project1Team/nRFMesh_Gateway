/**************************************************************************
  * @file       : generic_byte_message.h
  * @brief      : Control message package
  * @author     : William Nguyen(thnam.nguyen27) 
  * @version    : 0.2
  * @history    : created on @20181029
***************************************************************************/


#ifndef GENERIC_BYTE_MESSAGES_H__
#define GENERIC_BYTE_MESSAGES_H__

#include <stdint.h>

/**
 * @internal
 * @defgroup GENERIC_BYTE_MESSAGES Internal header
 * @ingroup GENERIC_BYTE_MODEL
 * This internal header contains packed structures required for message parsing.
 * @{
 */

/** Shortest allowed length for the Set message. */
#define GENERIC_BYTE_SET_MINLEN 2
/** Longest allowed length for the Set message. */
#define GENERIC_BYTE_SET_MAXLEN 4

/** Shortest allowed length for the Status message. */
#define GENERIC_BYTE_STATUS_MINLEN 1
/** Longest allowed length for the Status message. */
#define GENERIC_BYTE_STATUS_MAXLEN 3

/** Generic Byte model message opcodes. */
typedef enum
{
    GENERIC_BYTE_OPCODE_SET = 0x8202,
    GENERIC_BYTE_OPCODE_SET_UNACKNOWLEDGED = 0x8203,
    GENERIC_BYTE_OPCODE_GET = 0x8201,
    GENERIC_BYTE_OPCODE_STATUS = 0x8204
} generic_byte_opcode_t;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/** Message format for the generic_byte Set message. */
typedef struct __attribute((packed))
{
    uint16_t byte;                                         /**< State to set */
    uint8_t tid;                                            /**< Transaction number for application */
    uint8_t transition_time;                                /**< Encoded transition time value */
    uint8_t delay;                                          /**< Encoded message execution delay in 5 millisecond steps */
} generic_byte_set_msg_pkt_t;

/** Message format for the generic_byte Status message. */
typedef struct __attribute((packed))
{
    uint16_t present_byte;                                 /**< The present value of the Generic Byte state */
    uint16_t target_byte;                                  /**< The target value of the Generic Byte state (optional) */
    uint8_t remaining_time;                                 /**< Encoded remaining time */
} generic_byte_status_msg_pkt_t;

/**@} end of GENERIC_BYTE_MODEL_INTENRAL */
#endif /* GENERIC_BYTE_MESSAGES_H__ */
