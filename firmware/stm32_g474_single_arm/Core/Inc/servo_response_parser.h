#ifndef SERVO_RESPONSE_PARSER_H
#define SERVO_RESPONSE_PARSER_H

#include <stdint.h>

#define SERVO_RESPONSE_MAX_BODY_LENGTH UINT8_C(18)

typedef enum
{
    SERVO_RESPONSE_REJECT_NONE = 0,
    SERVO_RESPONSE_REJECT_HEADER = 1,
    SERVO_RESPONSE_REJECT_ID = 2,
    SERVO_RESPONSE_REJECT_LENGTH = 3,
    SERVO_RESPONSE_REJECT_CHECKSUM = 4
} ServoResponseRejectReason;

typedef enum
{
    SERVO_RESPONSE_NEED_MORE = 0,
    SERVO_RESPONSE_FRAME_READY = 1,
    SERVO_RESPONSE_FRAME_REJECTED = 2,
    SERVO_RESPONSE_STATUS_ERROR = 3
} ServoResponseParseResult;

typedef struct
{
    uint8_t expected_id;
    uint8_t expected_data_length;
    uint8_t sync_count;
    uint8_t frame_id;
    uint8_t frame_length;
    uint8_t body_index;
    uint8_t body[SERVO_RESPONSE_MAX_BODY_LENGTH];
    uint8_t servo_status;
    ServoResponseRejectReason last_reject;
    uint16_t discarded_bytes;
} ServoResponseParser;

void ServoResponseParser_Init(
    ServoResponseParser *parser,
    uint8_t expected_id,
    uint8_t expected_data_length
);

ServoResponseParseResult ServoResponseParser_Push(
    ServoResponseParser *parser,
    uint8_t byte
);

const uint8_t *ServoResponseParser_Data(
    const ServoResponseParser *parser
);

#endif /* SERVO_RESPONSE_PARSER_H */
