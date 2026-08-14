#include "actuator_core/protocol.h"

#include <string.h>

#include "actuator_core/cobs.h"
#include "actuator_core/crc32c.h"
#if defined(ACTUATOR_ENABLE_STREAM_V2_MESSAGES) && \
    ACTUATOR_ENABLE_STREAM_V2_MESSAGES
#include "actuator_core/stream_contract_v2.h"
#endif

static void write_u16_le(uint8_t *destination, uint16_t value) {
    destination[0] = (uint8_t)(value & 0xFFu);
    destination[1] = (uint8_t)((value >> 8u) & 0xFFu);
}

static void write_u32_le(uint8_t *destination, uint32_t value) {
    destination[0] = (uint8_t)(value & 0xFFu);
    destination[1] = (uint8_t)((value >> 8u) & 0xFFu);
    destination[2] = (uint8_t)((value >> 16u) & 0xFFu);
    destination[3] = (uint8_t)((value >> 24u) & 0xFFu);
}

static uint16_t read_u16_le(const uint8_t *source) {
    return (uint16_t)((uint16_t)source[0] | ((uint16_t)source[1] << 8u));
}

static uint32_t read_u32_le(const uint8_t *source) {
    return (uint32_t)source[0] |
           ((uint32_t)source[1] << 8u) |
           ((uint32_t)source[2] << 16u) |
           ((uint32_t)source[3] << 24u);
}

bool actuator_protocol_message_is_known(uint8_t message_type) {
    switch (message_type) {
        case ACTUATOR_MSG_HELLO_REQUEST:
        case ACTUATOR_MSG_HELLO_RESPONSE:
        case ACTUATOR_MSG_HEARTBEAT:
        case ACTUATOR_MSG_TIME_SYNC_REQUEST:
        case ACTUATOR_MSG_TIME_SYNC_RESPONSE:
        case ACTUATOR_MSG_ARM_REQUEST:
        case ACTUATOR_MSG_ARM_RESPONSE:
        case ACTUATOR_MSG_ENABLE:
        case ACTUATOR_MSG_HOLD:
        case ACTUATOR_MSG_SAFE_STOP:
        case ACTUATOR_MSG_DISABLE:
        case ACTUATOR_MSG_CLEAR_FAULT:
        case ACTUATOR_MSG_SETPOINT_BATCH:
        case ACTUATOR_MSG_SETPOINT_STATUS:
        case ACTUATOR_MSG_RIGHT_ARM_JOG_ONCE_REQUEST:
        case ACTUATOR_MSG_RIGHT_ARM_JOG_ONCE_RESPONSE:
        case ACTUATOR_MSG_RIGHT_ARM_TORQUE_ENABLE_ONCE_REQUEST:
        case ACTUATOR_MSG_RIGHT_ARM_TORQUE_ENABLE_ONCE_RESPONSE:
        case ACTUATOR_MSG_RIGHT_ARM_CONFIGURE_ONCE_REQUEST:
        case ACTUATOR_MSG_RIGHT_ARM_CONFIGURE_ONCE_RESPONSE:
        case ACTUATOR_MSG_GET_STATE:
        case ACTUATOR_MSG_STATE_FEEDBACK:
        case ACTUATOR_MSG_FAULT_REPORT:
        case ACTUATOR_MSG_DIAGNOSTICS:
        case ACTUATOR_MSG_RIGHT_ARM_DISCOVERY_REQUEST:
        case ACTUATOR_MSG_RIGHT_ARM_DISCOVERY_RESPONSE:
        case ACTUATOR_MSG_RIGHT_ARM_CONFIGURATION_REQUEST:
        case ACTUATOR_MSG_RIGHT_ARM_CONFIGURATION_RESPONSE:
        case ACTUATOR_MSG_RIGHT_ARM_DISABLE_REQUEST:
        case ACTUATOR_MSG_RIGHT_ARM_DISABLE_RESPONSE:
#if defined(ACTUATOR_ENABLE_STREAM_V2_MESSAGES) && \
    ACTUATOR_ENABLE_STREAM_V2_MESSAGES
        case ACTUATOR_V2_MSG_STREAM_OPEN:
        case ACTUATOR_V2_MSG_STREAM_STATUS:
        case ACTUATOR_V2_MSG_SPLICE:
#endif
#if defined(ACTUATOR_ENABLE_STREAM_V2_EXECUTOR_MESSAGES) && \
    ACTUATOR_ENABLE_STREAM_V2_EXECUTOR_MESSAGES
        case ACTUATOR_V2_MSG_GET_EXECUTOR_DIAGNOSTICS:
        case ACTUATOR_V2_MSG_EXECUTOR_DIAGNOSTICS:
#endif
#if defined(ACTUATOR_ENABLE_STREAM_V2_SHADOW_MESSAGES) && \
    ACTUATOR_ENABLE_STREAM_V2_SHADOW_MESSAGES
        case ACTUATOR_V2_MSG_PREPARE_SHADOW:
        case ACTUATOR_V2_MSG_SHADOW_SNAPSHOT:
#endif
#if defined(ACTUATOR_ENABLE_BIMANUAL_DISPATCH_MESSAGES) && \
    ACTUATOR_ENABLE_BIMANUAL_DISPATCH_MESSAGES
        case ACTUATOR_V2_MSG_GET_DISPATCH_DIAGNOSTICS:
        case ACTUATOR_V2_MSG_DISPATCH_DIAGNOSTICS:
#endif
#if defined(ACTUATOR_ENABLE_BIMANUAL_TRACKING_MESSAGES) && \
    ACTUATOR_ENABLE_BIMANUAL_TRACKING_MESSAGES
        case ACTUATOR_V2_MSG_GET_TRACKING_DIAGNOSTICS:
        case ACTUATOR_V2_MSG_TRACKING_DIAGNOSTICS:
#endif
#if defined(ACTUATOR_ENABLE_BIMANUAL_FEEDBACK_SNAPSHOT_MESSAGES) && \
    ACTUATOR_ENABLE_BIMANUAL_FEEDBACK_SNAPSHOT_MESSAGES
        case ACTUATOR_V2_MSG_GET_FEEDBACK_SNAPSHOT:
        case ACTUATOR_V2_MSG_FEEDBACK_SNAPSHOT:
#endif
            return true;
        default:
            return false;
    }
}

actuator_protocol_result_t actuator_frame_encode(
    const actuator_frame_t *frame,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length) {
    uint8_t decoded[ACTUATOR_PROTOCOL_MAX_DECODED_SIZE];
    size_t decoded_length;
    size_t encoded_length;
    uint32_t crc;

    if (frame == NULL || output == NULL || output_length == NULL) {
        return ACTUATOR_PROTOCOL_NULL_ARGUMENT;
    }
    if (!actuator_protocol_message_is_known(frame->message_type)) {
        return ACTUATOR_PROTOCOL_UNKNOWN_MESSAGE;
    }
    if (frame->payload_length > ACTUATOR_PROTOCOL_MAX_PAYLOAD) {
        return ACTUATOR_PROTOCOL_BAD_LENGTH;
    }
    if (output_capacity < 2u) {
        return ACTUATOR_PROTOCOL_OUTPUT_TOO_SMALL;
    }

    write_u16_le(&decoded[0], ACTUATOR_PROTOCOL_MAGIC);
    decoded[2] = (uint8_t)ACTUATOR_PROTOCOL_VERSION;
    decoded[3] = frame->message_type;
    write_u16_le(&decoded[4], frame->flags);
    write_u16_le(&decoded[6], frame->payload_length);
    write_u32_le(&decoded[8], frame->sequence);
    write_u32_le(&decoded[12], frame->sender_time_ms);
    if (frame->payload_length > 0u) {
        memcpy(&decoded[ACTUATOR_PROTOCOL_HEADER_SIZE], frame->payload, frame->payload_length);
    }
    decoded_length = ACTUATOR_PROTOCOL_HEADER_SIZE + frame->payload_length;
    crc = actuator_crc32c(decoded, decoded_length);
    write_u32_le(&decoded[decoded_length], crc);
    decoded_length += ACTUATOR_PROTOCOL_CRC_SIZE;

    if (!actuator_cobs_encode(decoded, decoded_length, output, output_capacity - 1u, &encoded_length)) {
        return ACTUATOR_PROTOCOL_OUTPUT_TOO_SMALL;
    }
    output[encoded_length] = 0u;
    *output_length = encoded_length + 1u;
    return ACTUATOR_PROTOCOL_OK;
}

actuator_protocol_result_t actuator_frame_decode(
    const uint8_t *encoded,
    size_t encoded_length,
    actuator_frame_t *frame) {
    uint8_t decoded[ACTUATOR_PROTOCOL_MAX_DECODED_SIZE];
    size_t decoded_length;
    uint16_t payload_length;
    size_t expected_length;
    uint32_t expected_crc;
    uint32_t actual_crc;

    if (encoded == NULL || frame == NULL) {
        return ACTUATOR_PROTOCOL_NULL_ARGUMENT;
    }
    if (!actuator_cobs_decode(encoded, encoded_length, decoded, sizeof(decoded), &decoded_length)) {
        return ACTUATOR_PROTOCOL_COBS_ERROR;
    }
    if (decoded_length < ACTUATOR_PROTOCOL_HEADER_SIZE + ACTUATOR_PROTOCOL_CRC_SIZE) {
        return ACTUATOR_PROTOCOL_FRAME_TOO_SHORT;
    }
    if (read_u16_le(&decoded[0]) != ACTUATOR_PROTOCOL_MAGIC) {
        return ACTUATOR_PROTOCOL_BAD_MAGIC;
    }
    if (decoded[2] != (uint8_t)ACTUATOR_PROTOCOL_VERSION) {
        return ACTUATOR_PROTOCOL_BAD_VERSION;
    }
    if (!actuator_protocol_message_is_known(decoded[3])) {
        return ACTUATOR_PROTOCOL_UNKNOWN_MESSAGE;
    }

    payload_length = read_u16_le(&decoded[6]);
    if (payload_length > ACTUATOR_PROTOCOL_MAX_PAYLOAD) {
        return ACTUATOR_PROTOCOL_BAD_LENGTH;
    }
    expected_length = ACTUATOR_PROTOCOL_HEADER_SIZE + payload_length + ACTUATOR_PROTOCOL_CRC_SIZE;
    if (decoded_length != expected_length) {
        return ACTUATOR_PROTOCOL_BAD_LENGTH;
    }

    expected_crc = read_u32_le(&decoded[ACTUATOR_PROTOCOL_HEADER_SIZE + payload_length]);
    actual_crc = actuator_crc32c(decoded, ACTUATOR_PROTOCOL_HEADER_SIZE + payload_length);
    if (actual_crc != expected_crc) {
        return ACTUATOR_PROTOCOL_BAD_CRC;
    }

    frame->message_type = decoded[3];
    frame->flags = read_u16_le(&decoded[4]);
    frame->payload_length = payload_length;
    frame->sequence = read_u32_le(&decoded[8]);
    frame->sender_time_ms = read_u32_le(&decoded[12]);
    if (payload_length > 0u) {
        memcpy(frame->payload, &decoded[ACTUATOR_PROTOCOL_HEADER_SIZE], payload_length);
    }
    return ACTUATOR_PROTOCOL_OK;
}

void actuator_stream_parser_init(actuator_stream_parser_t *parser) {
    if (parser != NULL) {
        parser->length = 0u;
        parser->dropping_overflowed_frame = false;
    }
}

actuator_protocol_result_t actuator_stream_parser_push(
    actuator_stream_parser_t *parser,
    uint8_t byte,
    actuator_frame_t *frame) {
    actuator_protocol_result_t result;

    if (parser == NULL || frame == NULL) {
        return ACTUATOR_PROTOCOL_NULL_ARGUMENT;
    }
    if (byte != 0u) {
        if (parser->dropping_overflowed_frame) {
            return ACTUATOR_PROTOCOL_NO_FRAME;
        }
        if (parser->length >= sizeof(parser->encoded)) {
            parser->length = 0u;
            parser->dropping_overflowed_frame = true;
            return ACTUATOR_PROTOCOL_OVERFLOW;
        }
        parser->encoded[parser->length++] = byte;
        return ACTUATOR_PROTOCOL_NO_FRAME;
    }

    if (parser->dropping_overflowed_frame) {
        parser->length = 0u;
        parser->dropping_overflowed_frame = false;
        return ACTUATOR_PROTOCOL_OVERFLOW;
    }
    if (parser->length == 0u) {
        return ACTUATOR_PROTOCOL_NO_FRAME;
    }

    result = actuator_frame_decode(parser->encoded, parser->length, frame);
    parser->length = 0u;
    return result;
}
