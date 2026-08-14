#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "actuator_core/sts3215_packet.h"

static int failures = 0;

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    ++failures; return; } } while (0)

static void test_six_servo_position_packet(void) {
    const uint8_t ids[6] = {1u, 2u, 3u, 4u, 5u, 6u};
    const uint16_t positions[6] = {
        0x0800u, 0x0801u, 0x0000u, 0x0fffu, 0x1234u, 0x00ffu};
    const uint8_t expected[26] = {
        0xffu, 0xffu, 0xfeu, 0x16u, 0x83u, 0x2au, 0x02u,
        0x01u, 0x00u, 0x08u,
        0x02u, 0x01u, 0x08u,
        0x03u, 0x00u, 0x00u,
        0x04u, 0xffu, 0x0fu,
        0x05u, 0x34u, 0x12u,
        0x06u, 0xffu, 0x00u,
        0xc3u};
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
    size_t length = 0u;

    CHECK(actuator_sts3215_build_sync_write_positions(
              ids, positions, 6u, packet, &length) ==
          ACTUATOR_STS3215_PACKET_OK);
    CHECK(length == sizeof(expected));
    CHECK(memcmp(packet, expected, sizeof(expected)) == 0);
}

static void test_invalid_inputs_are_rejected(void) {
    const uint8_t duplicate_ids[2] = {1u, 1u};
    const uint8_t broadcast_id[1] = {0xfeu};
    const uint16_t positions[2] = {2048u, 2048u};
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
    size_t length = 99u;

    CHECK(actuator_sts3215_build_sync_write_positions(
              duplicate_ids, positions, 2u, packet, &length) ==
          ACTUATOR_STS3215_PACKET_DUPLICATE_SERVO_ID);
    CHECK(actuator_sts3215_build_sync_write_positions(
              broadcast_id, positions, 1u, packet, &length) ==
          ACTUATOR_STS3215_PACKET_INVALID_SERVO_ID);
    CHECK(actuator_sts3215_build_sync_write_positions(
              duplicate_ids, positions, 0u, packet, &length) ==
          ACTUATOR_STS3215_PACKET_INVALID_COUNT);
    CHECK(actuator_sts3215_build_sync_write_positions(
              NULL, positions, 1u, packet, &length) ==
          ACTUATOR_STS3215_PACKET_NULL_ARGUMENT);
}

int main(void) {
    test_six_servo_position_packet();
    test_invalid_inputs_are_rejected();
    if (failures != 0) {
        fprintf(stderr, "%d test(s) failed\n", failures);
        return 1;
    }
    puts("sts3215 packet tests passed");
    return 0;
}
