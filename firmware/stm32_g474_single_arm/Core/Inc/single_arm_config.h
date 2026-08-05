#ifndef SINGLE_ARM_CONFIG_H
#define SINGLE_ARM_CONFIG_H

#include <stdint.h>

#define SINGLE_ARM_JOINT_COUNT 6U

#define ENABLE_SERVO_CENTERING_COMMAND 0U
#define ENABLE_BOOT_ID1_AUTOCONFIG 0U

#define HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00022A00)
#define HOST_BINARY_CAPABILITIES UINT32_C(0x00000FFF)
#define HOST_BUFFERED_VALIDATION_CAPABILITY UINT32_C(0x00000400)
#define HOST_BUFFERED_EXECUTION_CAPABILITY UINT32_C(0x00000800)
#define HOST_BINARY_HEARTBEAT_TIMEOUT_MS UINT32_C(500)
#define HOST_BINARY_RX_BURST_MAX_BYTES UINT8_C(64)

/*
 * LPUART1 host link. Mirrors hlpuart1.Init.BaudRate in main.c, which
 * tests/test_stm32_status_frame_transmit_budget.py keeps in agreement.
 *
 * This is not documentation. Host_SendBinaryFrame transmits by a blocking
 * call on the same cooperative loop that steps the buffered executor, so the
 * encoded length of a response divided by this rate is charged directly to
 * apply lateness. HOST_BINARY_FRAME_TRANSMIT_MS below turns that into a
 * build-time bound, enforced by an #error in binary_control.c.
 */
#define HOST_BINARY_UART_BAUD UINT32_C(115200)

/* 8N1: one start bit and one stop bit per octet. */
#define HOST_BINARY_UART_BITS_PER_BYTE UINT32_C(10)

/*
 * COBS frames a 16 byte header, the payload and a 4 byte CRC, adding one code
 * byte for runs shorter than 254 and one trailing delimiter. Zero bytes cost
 * nothing extra, so for every status payload the encoded length is exact.
 */
#define HOST_BINARY_FRAME_WIRE_BYTES(payload_bytes) \
    ((payload_bytes) + 16U + 4U + 1U + 1U)

/* Ceiling, because lateness is counted in whole HAL_GetTick milliseconds. */
#define HOST_BINARY_FRAME_TRANSMIT_MS(payload_bytes)               \
    (((HOST_BINARY_FRAME_WIRE_BYTES(payload_bytes) *               \
       HOST_BINARY_UART_BITS_PER_BYTE * 1000U) +                   \
      (HOST_BINARY_UART_BAUD - 1U)) /                              \
     HOST_BINARY_UART_BAUD)

/* Terminal buffered status frames carry the apply-lateness distribution. */
#define HOST_BUFFERED_STATUS_TERMINAL UINT8_C(6)

/*
 * Motion-4 exposes only the no-motion validation route. These bounds retain
 * the existing single-point wire envelope; they are not operational queue
 * tuning values and must not authorize buffered servo output.
 */
#define HOST_BUFFERED_VALIDATION_MINIMUM_LEAD_MS UINT32_C(20)
#define HOST_BUFFERED_VALIDATION_MAXIMUM_LEAD_MS UINT32_C(2000)
#define HOST_BUFFERED_VALIDATION_MINIMUM_START_SAMPLES UINT8_C(2)
#define HOST_BUFFERED_VALIDATION_MAXIMUM_APPLY_LATENESS_MS UINT32_C(0)

/*
 * Physical buffered execution uses only the separately measured and reviewed
 * Pi-VCP timing policy.  The first wire sample is the validated t=0 pose at
 * 100 ms lead; the interpolation anchor is the same pose 20 ms earlier.
 */
#define HOST_BUFFERED_EXECUTION_SAMPLE_PERIOD_MS UINT32_C(20)
#define HOST_BUFFERED_EXECUTION_MINIMUM_LEAD_MS UINT32_C(60)
#define HOST_BUFFERED_EXECUTION_MAXIMUM_LEAD_MS UINT32_C(400)
#define HOST_BUFFERED_EXECUTION_MINIMUM_START_SAMPLES UINT8_C(16)
#define HOST_BUFFERED_EXECUTION_ANCHOR_OFFSET_MS UINT32_C(20)
#define HOST_BUFFERED_EXECUTION_OUTPUT_PERIOD_MS UINT32_C(5)
#define HOST_BUFFERED_EXECUTION_MAXIMUM_APPLY_LATENESS_MS UINT32_C(5)

#if (HOST_BUFFERED_EXECUTION_SAMPLE_PERIOD_MS % \
     HOST_BUFFERED_EXECUTION_OUTPUT_PERIOD_MS) != 0U
#error "Buffered output period must divide the 20 ms sample period"
#endif

#if HOST_BUFFERED_EXECUTION_MINIMUM_START_SAMPLES != 16U
#error "Buffered execution must retain the reviewed startup prime depth"
#endif

#if HOST_BUFFERED_EXECUTION_MAXIMUM_APPLY_LATENESS_MS != \
    HOST_BUFFERED_EXECUTION_OUTPUT_PERIOD_MS
#error "Buffered apply lateness must not exceed one output period"
#endif

/*
 * A background GET_STATE position sweep already retries one servo read three
 * times. Treat one failed sweep as observable degraded feedback, but require
 * failures in three consecutive host feedback periods before latching. The
 * motion start/final verification paths remain independently fail-closed on
 * their first exhausted sweep.
 */
#define HOST_POSITION_READ_FAILURE_LIMIT UINT8_C(3)

#if HOST_POSITION_READ_FAILURE_LIMIT < 2U
#error "Background position feedback must distinguish transient read failure"
#endif

/*
 * STS3215 동작 중 보호 기준.
 * load 1000은 최대 출력 100%, current 1은 약 6.5mA다.
 * 한 번의 main-loop 호출에서 한 축만 읽고 16ms 슬롯으로 순환한다.
 * 6축 전체는 약 96ms마다 갱신되며 축별 2회 연속 초과 시 중단한다.
 */
#define SERVO_MOTION_SAFETY_SLOT_MS UINT32_C(16)
#define SERVO_MOTION_SAFETY_SWEEP_MS UINT32_C(96)
#define SERVO_MOTION_LOAD_LIMIT_RAW UINT16_C(800)
#define SERVO_MOTION_CURRENT_LIMIT_RAW UINT16_C(320)
#define SERVO_MOTION_LIMIT_CONSECUTIVE UINT8_C(2)

/*
 * A trajectory endpoint is not judged from one early sample. Keep the
 * existing load/current watchdog active while collecting stable position
 * samples, then report the latest error after the bounded settling window.
 */
#define SERVO_FINAL_SETTLE_SAMPLE_MS UINT32_C(100)
#define SERVO_FINAL_SETTLE_MAX_MS UINT32_C(1000)
#define SERVO_FINAL_SETTLE_CONSECUTIVE UINT8_C(2)
#define SERVO_FINAL_ERROR_TOLERANCE_RAW UINT16_C(30)

#if SERVO_FINAL_SETTLE_MAX_MS < SERVO_FINAL_SETTLE_SAMPLE_MS
#error "Final settling window must contain at least one sample"
#endif

#if SERVO_FINAL_SETTLE_CONSECUTIVE < 1U
#error "Final settling requires at least one in-tolerance sample"
#endif

/*
 * Joint torque caps stay below the independent sustained-load stop threshold.
 * The Shoulder/Elbow caps account for the installed camera payload while the
 * load/current watchdog remains unchanged.
 */
#define SERVO_SHOULDER_TORQUE_LIMIT_RAW UINT16_C(780)
#define SERVO_ELBOW_TORQUE_LIMIT_RAW UINT16_C(650)

#if SERVO_SHOULDER_TORQUE_LIMIT_RAW >= SERVO_MOTION_LOAD_LIMIT_RAW
#error "Shoulder torque cap must remain below the load safety threshold"
#endif

#if SERVO_ELBOW_TORQUE_LIMIT_RAW >= SERVO_MOTION_LOAD_LIMIT_RAW
#error "Elbow torque cap must remain below the load safety threshold"
#endif

#endif /* SINGLE_ARM_CONFIG_H */
