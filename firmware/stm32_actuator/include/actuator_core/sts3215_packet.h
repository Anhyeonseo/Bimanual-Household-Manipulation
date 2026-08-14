#ifndef ACTUATOR_CORE_STS3215_PACKET_H
#define ACTUATOR_CORE_STS3215_PACKET_H

#include <stddef.h>
#include <stdint.h>

#define ACTUATOR_STS3215_MAX_SYNC_WRITE_SERVOS UINT8_C(6)
#define ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE UINT8_C(26)

typedef enum {
    ACTUATOR_STS3215_PACKET_OK = 0,
    ACTUATOR_STS3215_PACKET_NULL_ARGUMENT,
    ACTUATOR_STS3215_PACKET_INVALID_COUNT,
    ACTUATOR_STS3215_PACKET_INVALID_SERVO_ID,
    ACTUATOR_STS3215_PACKET_DUPLICATE_SERVO_ID
} actuator_sts3215_packet_result_t;

/*
 * Build one protocol-0 broadcast SYNC WRITE for Goal Position (address 42).
 * This module is HAL-free so the exact bytes used by both arm buses can be
 * verified on the host before either UART is allowed to transmit them.
 */
actuator_sts3215_packet_result_t
actuator_sts3215_build_sync_write_positions(
    const uint8_t *servo_ids,
    const uint16_t *positions,
    uint8_t servo_count,
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE],
    size_t *packet_length);

#endif
