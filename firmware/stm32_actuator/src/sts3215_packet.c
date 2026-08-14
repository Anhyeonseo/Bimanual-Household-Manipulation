#include "actuator_core/sts3215_packet.h"

#include <stdbool.h>
#include <string.h>

static uint8_t checksum(const uint8_t *packet, size_t last_index) {
    uint8_t sum = 0u;
    size_t index;

    for (index = 2u; index <= last_index; ++index) {
        sum = (uint8_t)(sum + packet[index]);
    }
    return (uint8_t)(~sum);
}

actuator_sts3215_packet_result_t
actuator_sts3215_build_sync_write_positions(
    const uint8_t *servo_ids,
    const uint16_t *positions,
    uint8_t servo_count,
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE],
    size_t *packet_length) {
    bool seen[254] = {false};
    size_t packet_index;
    uint8_t servo;

    if (servo_ids == NULL || positions == NULL || packet == NULL ||
        packet_length == NULL) {
        return ACTUATOR_STS3215_PACKET_NULL_ARGUMENT;
    }
    if (servo_count == 0u ||
        servo_count > ACTUATOR_STS3215_MAX_SYNC_WRITE_SERVOS) {
        return ACTUATOR_STS3215_PACKET_INVALID_COUNT;
    }
    for (servo = 0u; servo < servo_count; ++servo) {
        const uint8_t servo_id = servo_ids[servo];
        if (servo_id == 0u || servo_id >= 0xfeu) {
            return ACTUATOR_STS3215_PACKET_INVALID_SERVO_ID;
        }
        if (seen[servo_id]) {
            return ACTUATOR_STS3215_PACKET_DUPLICATE_SERVO_ID;
        }
        seen[servo_id] = true;
    }

    memset(packet, 0, ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE);
    packet[0] = 0xffu;
    packet[1] = 0xffu;
    packet[2] = 0xfeu;
    packet[3] = (uint8_t)(4u + (uint8_t)(servo_count * 3u));
    packet[4] = 0x83u;
    packet[5] = 42u;
    packet[6] = 2u;
    packet_index = 7u;

    for (servo = 0u; servo < servo_count; ++servo) {
        packet[packet_index++] = servo_ids[servo];
        packet[packet_index++] = (uint8_t)(positions[servo] & 0xffu);
        packet[packet_index++] = (uint8_t)(positions[servo] >> 8u);
    }
    packet[packet_index] = checksum(packet, packet_index - 1u);
    ++packet_index;
    *packet_length = packet_index;
    return ACTUATOR_STS3215_PACKET_OK;
}
