#include "servo_response_parser.h"

#include <stddef.h>
#include <string.h>

enum
{
    SERVO_RESPONSE_READ_ID = 2,
    SERVO_RESPONSE_READ_LENGTH = 3,
    SERVO_RESPONSE_READ_BODY = 4
};

static void ServoResponseParser_ResetFrame(
    ServoResponseParser *parser
)
{
    parser->sync_count = 0U;
    parser->frame_id = 0U;
    parser->frame_length = 0U;
    parser->body_index = 0U;
}

static ServoResponseParseResult ServoResponseParser_Reject(
    ServoResponseParser *parser,
    ServoResponseRejectReason reason,
    uint16_t frame_bytes
)
{
    parser->last_reject = reason;
    if ((uint16_t)(UINT16_MAX - parser->discarded_bytes) < frame_bytes)
    {
        parser->discarded_bytes = UINT16_MAX;
    }
    else
    {
        parser->discarded_bytes =
            (uint16_t)(parser->discarded_bytes + frame_bytes);
    }
    ServoResponseParser_ResetFrame(parser);
    return SERVO_RESPONSE_FRAME_REJECTED;
}

void ServoResponseParser_Init(
    ServoResponseParser *parser,
    uint8_t expected_id,
    uint8_t expected_data_length
)
{
    if (parser == NULL)
    {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->expected_id = expected_id;
    parser->expected_data_length = expected_data_length;
}

ServoResponseParseResult ServoResponseParser_Push(
    ServoResponseParser *parser,
    uint8_t byte
)
{
    if (parser == NULL)
    {
        return SERVO_RESPONSE_FRAME_REJECTED;
    }

    if (parser->sync_count < 2U)
    {
        if (byte == 0xFFU)
        {
            parser->sync_count++;
        }
        else
        {
            parser->sync_count = 0U;
            if (parser->discarded_bytes < UINT16_MAX)
            {
                parser->discarded_bytes++;
            }
            parser->last_reject = SERVO_RESPONSE_REJECT_HEADER;
        }
        return SERVO_RESPONSE_NEED_MORE;
    }

    if (parser->sync_count == SERVO_RESPONSE_READ_ID)
    {
        if (byte == 0xFFU)
        {
            /*
             * Keep the last two bytes as a possible header. This handles an
             * arbitrary run of 0xFF bytes and the overlap between a stale
             * trailing sync byte and the next valid frame's FF FF header.
             * A unicast servo status response can never use ID 0xFF.
             */
            return SERVO_RESPONSE_NEED_MORE;
        }
        parser->frame_id = byte;
        parser->sync_count = SERVO_RESPONSE_READ_LENGTH;
        return SERVO_RESPONSE_NEED_MORE;
    }

    if (parser->sync_count == SERVO_RESPONSE_READ_LENGTH)
    {
        parser->frame_length = byte;
        parser->body_index = 0U;
        if ((byte < 2U) || (byte > SERVO_RESPONSE_MAX_BODY_LENGTH))
        {
            return ServoResponseParser_Reject(
                parser,
                SERVO_RESPONSE_REJECT_LENGTH,
                4U
            );
        }
        parser->sync_count = SERVO_RESPONSE_READ_BODY;
        return SERVO_RESPONSE_NEED_MORE;
    }

    parser->body[parser->body_index++] = byte;
    if (parser->body_index < parser->frame_length)
    {
        return SERVO_RESPONSE_NEED_MORE;
    }

    uint8_t sum = (uint8_t)(
        parser->frame_id + parser->frame_length
    );
    for (uint8_t index = 0U;
         index < (uint8_t)(parser->frame_length - 1U);
         index++)
    {
        sum = (uint8_t)(sum + parser->body[index]);
    }

    uint16_t frame_bytes = (uint16_t)parser->frame_length + 4U;
    if (parser->body[parser->frame_length - 1U] != (uint8_t)(~sum))
    {
        return ServoResponseParser_Reject(
            parser,
            SERVO_RESPONSE_REJECT_CHECKSUM,
            frame_bytes
        );
    }
    if (parser->frame_id != parser->expected_id)
    {
        return ServoResponseParser_Reject(
            parser,
            SERVO_RESPONSE_REJECT_ID,
            frame_bytes
        );
    }
    if (parser->frame_length !=
        (uint8_t)(parser->expected_data_length + 2U))
    {
        return ServoResponseParser_Reject(
            parser,
            SERVO_RESPONSE_REJECT_LENGTH,
            frame_bytes
        );
    }

    parser->servo_status = parser->body[0];
    if (parser->servo_status != 0U)
    {
        return SERVO_RESPONSE_STATUS_ERROR;
    }
    return SERVO_RESPONSE_FRAME_READY;
}

const uint8_t *ServoResponseParser_Data(
    const ServoResponseParser *parser
)
{
    if (parser == NULL)
    {
        return NULL;
    }
    return &parser->body[1];
}
