#include "servo_rx_window.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define RING_CAPACITY UINT16_C(256)

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

static uint32_t ring_write(
    uint8_t ring[RING_CAPACITY],
    uint32_t producer,
    const uint8_t *bytes,
    size_t length
)
{
    for (size_t index = 0U; index < length; index++)
    {
        ring[producer % RING_CAPACITY] = bytes[index];
        producer++;
    }
    return producer;
}

static void test_cold_start_immediate_response_all_lengths(void)
{
    static const uint8_t lengths[] = {1U, 2U, 3U, 5U, 10U, 11U, 15U, 16U};
    for (size_t case_index = 0U;
         case_index < sizeof(lengths);
         case_index++)
    {
        uint8_t ring[RING_CAPACITY] = {0U};
        uint8_t packet[24] = {0U};
        uint8_t input[16] = {0U};
        uint8_t output[16] = {0U};
        uint8_t length = lengths[case_index];
        for (uint8_t index = 0U; index < length; index++)
        {
            input[index] = (uint8_t)(index + 0x20U);
        }

        ServoRxWindow window;
        ServoRxWindow_Init(&window, 1U, length, 0U);
        size_t packet_length = build_reply(packet, 1U, 0U, input, length);
        uint32_t producer = ring_write(ring, 0U, packet, packet_length);
        assert(ServoRxWindow_Consume(
            &window, ring, RING_CAPACITY, producer, output, sizeof(output)
        ) == SERVO_RX_WINDOW_FRAME_READY);
        assert(memcmp(input, output, length) == 0);
    }
}

static void test_partial_arrival_preserves_state(void)
{
    uint8_t ring[RING_CAPACITY] = {0U};
    uint8_t packet[16] = {0U};
    uint8_t output[2] = {0U};
    const uint8_t input[2] = {0x34U, 0x12U};
    size_t length = build_reply(packet, 1U, 0U, input, 2U);
    ServoRxWindow window;
    ServoRxWindow_Init(&window, 1U, 2U, 0U);

    uint32_t producer = ring_write(ring, 0U, packet, 3U);
    assert(ServoRxWindow_Consume(
        &window, ring, RING_CAPACITY, producer, output, sizeof(output)
    ) == SERVO_RX_WINDOW_NEED_MORE);
    producer = ring_write(ring, producer, &packet[3], length - 3U);
    assert(ServoRxWindow_Consume(
        &window, ring, RING_CAPACITY, producer, output, sizeof(output)
    ) == SERVO_RX_WINDOW_FRAME_READY);
    assert(memcmp(input, output, 2U) == 0);
}

static void test_ring_wrap_and_stale_corrupt_resynchronize(void)
{
    uint8_t ring[RING_CAPACITY] = {0U};
    uint8_t stream[40] = {0x12U, 0x34U};
    uint8_t output[2] = {0U};
    const uint8_t input[2] = {0x78U, 0x56U};
    size_t corrupt = build_reply(&stream[2], 6U, 0U, input, 2U);
    stream[2U + corrupt - 1U] ^= 0x01U;
    size_t valid_offset = 2U + corrupt;
    size_t valid = build_reply(&stream[valid_offset], 1U, 0U, input, 2U);
    uint32_t start = 250U;
    ServoRxWindow window;
    ServoRxWindow_Init(&window, 1U, 2U, start);
    uint32_t producer = ring_write(
        ring, start, stream, valid_offset + valid
    );
    assert(ServoRxWindow_Consume(
        &window, ring, RING_CAPACITY, producer, output, sizeof(output)
    ) == SERVO_RX_WINDOW_FRAME_READY);
    assert(memcmp(input, output, 2U) == 0);
    assert(window.parser.discarded_bytes > 0U);
}

static void test_power_edge_noise_before_transaction_is_quarantined(void)
{
    uint8_t ring[RING_CAPACITY] = {0U};
    uint8_t packet[16] = {0U};
    uint8_t output[2] = {0U};
    const uint8_t power_edge_noise[] = {0x00U, 0x00U, 0x7FU, 0xFFU};
    const uint8_t input[2] = {0xE1U, 0x07U};
    uint32_t producer = ring_write(
        ring, 0U, power_edge_noise, sizeof(power_edge_noise)
    );
    uint32_t transaction_cursor = producer;
    size_t length = build_reply(packet, 1U, 0U, input, 2U);
    producer = ring_write(ring, producer, packet, length);

    ServoRxWindow window;
    ServoRxWindow_Init(&window, 1U, 2U, transaction_cursor);
    assert(ServoRxWindow_Consume(
        &window, ring, RING_CAPACITY, producer, output, sizeof(output)
    ) == SERVO_RX_WINDOW_FRAME_READY);
    assert(memcmp(input, output, 2U) == 0);
    assert(window.parser.discarded_bytes == 0U);
}

static void test_framing_noise_after_request_requires_complete_checksum(void)
{
    uint8_t ring[RING_CAPACITY] = {0U};
    uint8_t stream[32] = {0x00U, 0xFEU, 0xFFU, 0x12U, 0xFFU};
    uint8_t output[2] = {0U};
    const uint8_t input[2] = {0x22U, 0x09U};
    size_t valid = build_reply(&stream[5], 1U, 0U, input, 2U);
    ServoRxWindow window;
    ServoRxWindow_Init(&window, 1U, 2U, 0U);
    uint32_t producer = ring_write(ring, 0U, stream, 5U + valid);

    assert(ServoRxWindow_Consume(
        &window, ring, RING_CAPACITY, producer, output, sizeof(output)
    ) == SERVO_RX_WINDOW_FRAME_READY);
    assert(memcmp(input, output, 2U) == 0);
    assert(window.parser.discarded_bytes == 3U);
}

static void test_status_error_is_terminal(void)
{
    uint8_t ring[RING_CAPACITY] = {0U};
    uint8_t packet[16] = {0U};
    uint8_t output[2] = {0U};
    const uint8_t input[2] = {0U, 0U};
    size_t length = build_reply(packet, 1U, 4U, input, 2U);
    ServoRxWindow window;
    ServoRxWindow_Init(&window, 1U, 2U, 0U);
    uint32_t producer = ring_write(ring, 0U, packet, length);
    assert(ServoRxWindow_Consume(
        &window, ring, RING_CAPACITY, producer, output, sizeof(output)
    ) == SERVO_RX_WINDOW_STATUS_ERROR);
}

static void test_window_and_ring_overflow_are_rejected(void)
{
    uint8_t ring[RING_CAPACITY] = {0U};
    uint8_t output[2] = {0U};
    ServoRxWindow window;
    ServoRxWindow_Init(&window, 1U, 2U, 0U);
    assert(ServoRxWindow_Consume(
        &window, ring, RING_CAPACITY, 65U, output, sizeof(output)
    ) == SERVO_RX_WINDOW_OVERFLOW);

    ServoRxWindow_Init(&window, 1U, 2U, 0U);
    assert(ServoRxWindow_Consume(
        &window, ring, RING_CAPACITY, 257U, output, sizeof(output)
    ) == SERVO_RX_WINDOW_OVERFLOW);
}

static void test_deadline_is_uint32_wrap_safe(void)
{
    assert(ServoRxWindow_DeadlineExpired(
        UINT32_C(0xFFFFFFF0), UINT32_C(0x00000021), 50U
    ) == 0U);
    assert(ServoRxWindow_DeadlineExpired(
        UINT32_C(0xFFFFFFF0), UINT32_C(0x00000022), 50U
    ) == 1U);
}

static void test_power_domain_lifecycle_policy(void)
{
    assert(ServoRxWindow_ArmPermitted(0U, 1U, 10U, 2U, 0U) == 0U);
    assert(ServoRxWindow_ArmPermitted(1U, 1U, 1U, 2U, 0U) == 0U);
    assert(ServoRxWindow_ArmPermitted(1U, 1U, 2U, 2U, 0U) == 1U);
    assert(ServoRxWindow_ArmPermitted(1U, 1U, 10U, 2U, 1U) == 0U);

    assert(ServoRxWindow_HardResyncRequired(1U, 0U, 0U, 0U) == 1U);
    assert(ServoRxWindow_HardResyncRequired(0U, 1U, 0U, 0U) == 1U);
    assert(ServoRxWindow_HardResyncRequired(0U, 0U, 1U, 0U) == 1U);
    assert(ServoRxWindow_HardResyncRequired(0U, 0U, 0U, 1U) == 1U);
    assert(ServoRxWindow_HardResyncRequired(0U, 0U, 0U, 0U) == 0U);
}

static void test_failure_snapshot_preserves_recent_wraparound_bytes(void)
{
    uint8_t ring[RING_CAPACITY] = {0U};
    uint8_t stream[24] = {0U};
    uint8_t snapshot[16] = {0U};
    for (uint8_t index = 0U; index < sizeof(stream); index++)
    {
        stream[index] = (uint8_t)(0x80U + index);
    }

    uint32_t transaction_start = UINT32_C(0xFFFFFFF8);
    uint32_t producer = ring_write(
        ring, transaction_start, stream, sizeof(stream)
    );
    uint8_t captured = ServoRxWindow_CaptureRecent(
        ring,
        RING_CAPACITY,
        transaction_start,
        producer,
        snapshot,
        sizeof(snapshot)
    );
    assert(captured == sizeof(snapshot));
    assert(memcmp(snapshot, &stream[8], sizeof(snapshot)) == 0);
}

int main(void)
{
    test_cold_start_immediate_response_all_lengths();
    test_partial_arrival_preserves_state();
    test_ring_wrap_and_stale_corrupt_resynchronize();
    test_power_edge_noise_before_transaction_is_quarantined();
    test_framing_noise_after_request_requires_complete_checksum();
    test_status_error_is_terminal();
    test_window_and_ring_overflow_are_rejected();
    test_deadline_is_uint32_wrap_safe();
    test_power_domain_lifecycle_policy();
    test_failure_snapshot_preserves_recent_wraparound_bytes();
    return 0;
}
