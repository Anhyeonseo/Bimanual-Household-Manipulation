#include "servo_response_parser.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static size_t build_reply(
    uint8_t *packet,
    uint8_t servo_id,
    uint8_t status,
    const uint8_t *data,
    uint8_t data_length
)
{
    packet[0] = 0xFFU;
    packet[1] = 0xFFU;
    packet[2] = servo_id;
    packet[3] = (uint8_t)(data_length + 2U);
    packet[4] = status;
    uint8_t sum = (uint8_t)(servo_id + packet[3] + status);
    for (uint8_t index = 0U; index < data_length; index++)
    {
        packet[5U + index] = data[index];
        sum = (uint8_t)(sum + data[index]);
    }
    packet[5U + data_length] = (uint8_t)(~sum);
    return (size_t)data_length + 6U;
}

static ServoResponseParseResult feed(
    ServoResponseParser *parser,
    const uint8_t *bytes,
    size_t length
)
{
    ServoResponseParseResult result = SERVO_RESPONSE_NEED_MORE;
    for (size_t index = 0U; index < length; index++)
    {
        result = ServoResponseParser_Push(parser, bytes[index]);
        if ((result == SERVO_RESPONSE_FRAME_READY) ||
            (result == SERVO_RESPONSE_STATUS_ERROR))
        {
            return result;
        }
    }
    return result;
}

static void test_valid_frame(void)
{
    ServoResponseParser parser;
    uint8_t packet[8] = {0U};
    const uint8_t data[2] = {0x34U, 0x12U};
    size_t length = build_reply(packet, 1U, 0U, data, 2U);

    ServoResponseParser_Init(&parser, 1U, 2U);
    assert(feed(&parser, packet, length) == SERVO_RESPONSE_FRAME_READY);
    assert(memcmp(ServoResponseParser_Data(&parser), data, 2U) == 0);
}

static void test_stale_prefix_resynchronizes(void)
{
    ServoResponseParser parser;
    uint8_t stream[16] = {0x12U, 0x34U, 0xFFU, 0x01U};
    const uint8_t data[2] = {0x78U, 0x56U};
    size_t reply_length = build_reply(&stream[4], 1U, 0U, data, 2U);

    ServoResponseParser_Init(&parser, 1U, 2U);
    assert(feed(&parser, stream, 4U + reply_length) ==
        SERVO_RESPONSE_FRAME_READY);
    assert(parser.discarded_bytes >= 3U);
    assert(memcmp(ServoResponseParser_Data(&parser), data, 2U) == 0);
}

static void test_repeated_sync_bytes_resynchronize(void)
{
    ServoResponseParser parser;
    uint8_t stream[16] = {0xFFU};
    const uint8_t data[2] = {0xABU, 0xCDU};
    size_t reply_length = build_reply(&stream[1], 1U, 0U, data, 2U);

    ServoResponseParser_Init(&parser, 1U, 2U);
    assert(feed(&parser, stream, 1U + reply_length) ==
        SERVO_RESPONSE_FRAME_READY);
    assert(memcmp(ServoResponseParser_Data(&parser), data, 2U) == 0);
}

static void test_late_wrong_id_then_expected_frame(void)
{
    ServoResponseParser parser;
    uint8_t stream[20] = {0U};
    const uint8_t stale[2] = {0xAAU, 0xBBU};
    const uint8_t expected[2] = {0xCCU, 0xDDU};
    size_t first = build_reply(stream, 6U, 0U, stale, 2U);
    size_t second = build_reply(&stream[first], 1U, 0U, expected, 2U);

    ServoResponseParser_Init(&parser, 1U, 2U);
    assert(feed(&parser, stream, first + second) ==
        SERVO_RESPONSE_FRAME_READY);
    assert(parser.last_reject == SERVO_RESPONSE_REJECT_ID);
    assert(memcmp(ServoResponseParser_Data(&parser), expected, 2U) == 0);
}

static void test_bad_checksum_then_expected_frame(void)
{
    ServoResponseParser parser;
    uint8_t stream[20] = {0U};
    const uint8_t data[2] = {0x11U, 0x22U};
    size_t first = build_reply(stream, 1U, 0U, data, 2U);
    stream[first - 1U] ^= 0x01U;
    size_t second = build_reply(&stream[first], 1U, 0U, data, 2U);

    ServoResponseParser_Init(&parser, 1U, 2U);
    assert(feed(&parser, stream, first + second) ==
        SERVO_RESPONSE_FRAME_READY);
    assert(parser.last_reject == SERVO_RESPONSE_REJECT_CHECKSUM);
}

static void test_invalid_length_then_expected_frame(void)
{
    ServoResponseParser parser;
    uint8_t stream[16] = {0xFFU, 0xFFU, 1U, 1U};
    const uint8_t data[2] = {0x44U, 0x33U};
    size_t reply_length = build_reply(&stream[4], 1U, 0U, data, 2U);

    ServoResponseParser_Init(&parser, 1U, 2U);
    assert(feed(&parser, stream, 4U + reply_length) ==
        SERVO_RESPONSE_FRAME_READY);
    assert(parser.last_reject == SERVO_RESPONSE_REJECT_LENGTH);
}

static void test_split_bursts_preserve_parser_state(void)
{
    ServoResponseParser parser;
    uint8_t packet[8] = {0U};
    const uint8_t data[2] = {0xCDU, 0xABU};
    size_t length = build_reply(packet, 1U, 0U, data, 2U);

    ServoResponseParser_Init(&parser, 1U, 2U);
    assert(feed(&parser, packet, 3U) == SERVO_RESPONSE_NEED_MORE);
    assert(feed(&parser, &packet[3], length - 3U) ==
        SERVO_RESPONSE_FRAME_READY);
    assert(memcmp(ServoResponseParser_Data(&parser), data, 2U) == 0);
}

static void test_split_stale_and_corrupt_frames_resynchronize(void)
{
    ServoResponseParser parser;
    uint8_t stream[24] = {0x12U, 0xFFU, 0x01U};
    const uint8_t data[2] = {0x34U, 0x12U};
    size_t corrupt_length = build_reply(&stream[3], 1U, 0U, data, 2U);
    stream[3U + corrupt_length - 1U] ^= 0x01U;
    size_t valid_offset = 3U + corrupt_length;
    size_t valid_length = build_reply(
        &stream[valid_offset],
        1U,
        0U,
        data,
        2U
    );

    ServoResponseParser_Init(&parser, 1U, 2U);
    assert(feed(&parser, stream, 5U) == SERVO_RESPONSE_NEED_MORE);
    assert(feed(&parser, &stream[5], valid_offset - 5U + 2U) ==
        SERVO_RESPONSE_NEED_MORE);
    assert(feed(
        &parser,
        &stream[valid_offset + 2U],
        valid_length - 2U
    ) == SERVO_RESPONSE_FRAME_READY);
    assert(parser.last_reject == SERVO_RESPONSE_REJECT_CHECKSUM);
    assert(memcmp(ServoResponseParser_Data(&parser), data, 2U) == 0);
}

static void test_target_status_error_is_terminal(void)
{
    ServoResponseParser parser;
    uint8_t packet[8] = {0U};
    const uint8_t data[2] = {0U, 0U};
    size_t length = build_reply(packet, 1U, 4U, data, 2U);

    ServoResponseParser_Init(&parser, 1U, 2U);
    assert(feed(&parser, packet, length) ==
        SERVO_RESPONSE_STATUS_ERROR);
    assert(parser.servo_status == 4U);
}

int main(void)
{
    test_valid_frame();
    test_stale_prefix_resynchronizes();
    test_repeated_sync_bytes_resynchronize();
    test_late_wrong_id_then_expected_frame();
    test_bad_checksum_then_expected_frame();
    test_invalid_length_then_expected_frame();
    test_split_bursts_preserve_parser_state();
    test_split_stale_and_corrupt_frames_resynchronize();
    test_target_status_error_is_terminal();
    return 0;
}
