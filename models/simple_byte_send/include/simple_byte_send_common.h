/**************************************************************************
  * @file       : simple_byte_send_common.h
  * @brief      : define some info for simple_byte_send model
  * @author     : William Nguyen(thnam.nguyen27)
  * @version    : 0.1
  * @history    : created on @20181029
***************************************************************************/

#ifndef SIMPLE_BYTE_SEND_COMMON_H__
#define SIMPLE_BYTE_SEND_COMMON_H__

#include <stdint.h>
#include "access.h"

/** Vendor specific company ID for Simple ByteSend model */
#define SIMPLE_BYTE_SEND_COMPANY_ID (ACCESS_COMPANY_ID_NORDIC)

/** Simple Byte Send opcodes. */
typedef enum
{
    SIMPLE_BYTE_SEND_OPCODE_SET = 0xD1,             /**< Simple ByteSend Acknowledged Set. */
    SIMPLE_BYTE_SEND_OPCODE_GET = 0xD2,             /**< Simple ByteSend Get. */
    SIMPLE_BYTE_SEND_OPCODE_SET_UNRELIABLE = 0xD3,  /**< Simple ByteSend Set Unreliable. */
    SIMPLE_BYTE_SEND_OPCODE_STATUS = 0xD4           /**< Simple ByteSend Status. */
} simple_byte_send_opcode_t;

/** Message format for the Simple ByteSend Set message. */
typedef struct __attribute((packed))
{
    uint8_t msg; /**< State to set. */
    uint8_t tid; /**< Transaction number. */
} simple_byte_send_msg_set_t;

/** Message format for th Simple ByteSend Set Unreliable message. */
typedef struct __attribute((packed))
{
    uint8_t msg; /**< State to set. */
    uint8_t tid; /**< Transaction number. */
} simple_byte_send_msg_set_unreliable_t;

/** Message format for the Simple ByteSend Status message. */
typedef struct __attribute((packed))
{
    uint8_t present_msg; /**< Current state. */
} simple_byte_send_msg_status_t;

#endif //SIMPLE_BYTE_SEND_COMMON_H__