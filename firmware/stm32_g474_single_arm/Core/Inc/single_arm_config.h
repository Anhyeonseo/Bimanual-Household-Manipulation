#ifndef SINGLE_ARM_CONFIG_H
#define SINGLE_ARM_CONFIG_H

#include <stdint.h>

#define SINGLE_ARM_JOINT_COUNT 6U

#define ENABLE_SERVO_CENTERING_COMMAND 0U
#define ENABLE_BOOT_ID1_AUTOCONFIG 0U

#ifndef HOST_BINARY_FIRMWARE_VERSION
#define HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023B00)
#endif
#ifndef HOST_BINARY_CAPABILITIES
#define HOST_BINARY_CAPABILITIES UINT32_C(0x007FFFFF)
#endif
#define HOST_F0_METRICS_CAPABILITY UINT32_C(0x00001000)
#define HOST_F2_ASYNC_HOST_TX_CAPABILITY UINT32_C(0x00002000)
#define HOST_F1_HEARTBEAT_RX_TIMESTAMP_CAPABILITY UINT32_C(0x00004000)
#define HOST_H2_IN_MOTION_TELEMETRY_CAPABILITY UINT32_C(0x00008000)
#define HOST_F3_CONTROL_TICK_METRICS_CAPABILITY UINT32_C(0x00010000)
#define HOST_RIGHT_ARM_READ_ONLY_DISCOVERY_CAPABILITY UINT32_C(0x00020000)
#define HOST_RIGHT_ARM_JOG_ONCE_CAPABILITY UINT32_C(0x00040000)
#define HOST_RIGHT_ARM_TORQUE_ENABLE_ONCE_CAPABILITY UINT32_C(0x00080000)
#define HOST_RIGHT_ARM_CONFIGURATION_SNAPSHOT_CAPABILITY UINT32_C(0x00100000)
#define HOST_RIGHT_ARM_CONFIGURE_ONCE_CAPABILITY UINT32_C(0x00200000)
#define HOST_RIGHT_ARM_VERIFIED_DISABLE_CAPABILITY UINT32_C(0x00400000)
#define HOST_F25_HIGH_BAUD_VALIDATION_CAPABILITY UINT32_C(0x00800000)
#define HOST_PROTOCOL_V2_VALIDATION_CAPABILITY UINT32_C(0x01000000)
#define HOST_PROTOCOL_V2_EXECUTOR_CAPABILITY UINT32_C(0x02000000)
#define HOST_PROTOCOL_V2_SHADOW_CAPABILITY UINT32_C(0x04000000)
#define HOST_PROTOCOL_V2_UNWRAP_SHADOW_CAPABILITY UINT32_C(0x08000000)
#define HOST_RIGHT_ARM_J2_BASE_LIMIT_CAPABILITY UINT32_C(0x10000000)
#define HOST_BIMANUAL_OPERATIONAL_LIMITS_CAPABILITY UINT32_C(0x20000000)
#define HOST_BIMANUAL_DISPATCH_REFACTOR_CAPABILITY UINT32_C(0x40000000)
#define HOST_BIMANUAL_DMA_DISPATCH_CAPABILITY UINT32_C(0x80000000)
#define HOST_SHADOW_FEEDBACK_LIMIT_MARGIN_RAW UINT16_C(30)
#define HOST_BUFFERED_VALIDATION_CAPABILITY UINT32_C(0x00000400)
#define HOST_BUFFERED_EXECUTION_CAPABILITY UINT32_C(0x00000800)
#ifndef HOST_F25_VALIDATION_ONLY_BUILD
#define HOST_F25_VALIDATION_ONLY_BUILD 0U
#endif
#ifndef HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
#define HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD 0U
#endif
#ifndef HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
#define HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD 0U
#endif
#ifndef HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD
#define HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD 0U
#endif
#ifndef HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
#define HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD 0U
#endif
#ifndef HOST_PROTOCOL_V2_J1_LIMITS_VALIDATION_BUILD
#define HOST_PROTOCOL_V2_J1_LIMITS_VALIDATION_BUILD 0U
#endif
#ifndef HOST_BIMANUAL_OPERATIONAL_LIMITS_BUILD
#define HOST_BIMANUAL_OPERATIONAL_LIMITS_BUILD 0U
#endif
#ifndef HOST_BIMANUAL_DISPATCH_REFACTOR_BUILD
#define HOST_BIMANUAL_DISPATCH_REFACTOR_BUILD 0U
#endif
#ifndef HOST_BIMANUAL_DMA_DISPATCH_BUILD
#define HOST_BIMANUAL_DMA_DISPATCH_BUILD 0U
#endif
#ifndef HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD
#define HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD 0U
#endif
#ifndef HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
#define HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD 0U
#endif
#ifndef HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD
#define HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD 0U
#endif
#ifndef HOST_BIMANUAL_RESIDENT_FINITE_BUILD
#define HOST_BIMANUAL_RESIDENT_FINITE_BUILD 0U
#endif
#ifndef HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD
#define HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD 0U
#endif
#ifndef HOST_BIMANUAL_TERMINAL_SETTLE_BUILD
#define HOST_BIMANUAL_TERMINAL_SETTLE_BUILD 0U
#endif
#ifndef HOST_BIMANUAL_GRIPPER_TERMINAL_SETTLE_BUILD
#define HOST_BIMANUAL_GRIPPER_TERMINAL_SETTLE_BUILD 0U
#endif
#ifndef HOST_BINARY_JOINT_COUNT
#define HOST_BINARY_JOINT_COUNT SINGLE_ARM_JOINT_COUNT
#endif
#define HOST_BINARY_HEARTBEAT_TIMEOUT_MS UINT32_C(500)
#define HOST_BINARY_RX_BURST_MAX_BYTES UINT8_C(64)

/*
 * LPUART1 host link. Mirrors hlpuart1.Init.BaudRate in main.c, which
 * tests/test_stm32_status_frame_transmit_budget.py keeps in agreement.
 *
 * F2 sends encoded responses through a bounded DMA queue. Wire duration still
 * limits host response latency, but it is no longer charged to the cooperative
 * control loop's apply lateness.
 */
#ifndef HOST_BINARY_UART_BAUD
#define HOST_BINARY_UART_BAUD UINT32_C(115200)
#endif

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
#define SERVO_MOTION_LOAD_LIMIT_RAW UINT16_C(950)
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
 * A finite 12-axis stream is complete only after both buses have produced
 * twelve consecutive in-tolerance feedback pairs (six joints, twice). Pair
 * counting deliberately avoids coupling completion to an asynchronous joint
 * index boundary. Arm joints retain the legacy terminal envelope
 * (30 raw ~= 0.04602 rad). F8.7 permits a held-object residual on each
 * gripper up to the unchanged route-time tracking limit (0.09 rad). The host
 * repeats the same per-axis check before exposing READY.
 */
#define HOST_BIMANUAL_TERMINAL_SETTLE_ARM_TOLERANCE_URAD INT32_C(46020)
#if HOST_BIMANUAL_GRIPPER_TERMINAL_SETTLE_BUILD
#define HOST_BIMANUAL_TERMINAL_SETTLE_GRIPPER_TOLERANCE_URAD INT32_C(90000)
#else
#define HOST_BIMANUAL_TERMINAL_SETTLE_GRIPPER_TOLERANCE_URAD \
    HOST_BIMANUAL_TERMINAL_SETTLE_ARM_TOLERANCE_URAD
#endif
#define HOST_BIMANUAL_TERMINAL_SETTLE_CONSECUTIVE_PAIRS UINT8_C(12)
#define HOST_BIMANUAL_TERMINAL_SETTLE_MAX_MS UINT32_C(1000)

/*
 * One exhausted in-motion position pair is degraded telemetry, not proof that
 * either arm is unsafe. A later successful pair clears the streak. Preserve
 * immediate stops for measured tracking-limit violations and DMA/dispatch
 * faults, but require three consecutive read-pair failures before latching.
 */
#define HOST_BIMANUAL_TRACKING_READ_FAILURE_LIMIT UINT8_C(3)

#if HOST_BIMANUAL_TRACKING_READ_FAILURE_LIMIT < 2U
#error "Bimanual tracking must tolerate one transient read-pair failure"
#endif

#if HOST_BIMANUAL_TERMINAL_SETTLE_CONSECUTIVE_PAIRS < 1U
#error "Bimanual terminal settling requires at least one feedback pair"
#endif

/*
 * Joint torque caps stay below the independent sustained-load stop threshold.
 * The Shoulder/Elbow caps account for the installed camera payload while the
 * load/current watchdog remains unchanged.
 *
 * Raised 2026-08-07 to chase a higher CONSERVATIVE_TRACKING_RATE_RAW_S: at
 * the old caps (780/650) both already sat just under the old load watchdog
 * (800), so Present_Load -- which the servo internally clamps to
 * Torque_Limit -- could never approach that watchdog for either joint. The
 * independent current watchdog (SERVO_MOTION_CURRENT_LIMIT_RAW) is untouched
 * by this change and remains the backstop against a genuine jam/collision.
 */
#define SERVO_SHOULDER_TORQUE_LIMIT_RAW UINT16_C(900)
#define SERVO_ELBOW_TORQUE_LIMIT_RAW UINT16_C(800)

/*
 * STS3215 Goal_Speed (address 46-47). Feetech convention: 0 = unlimited,
 * 1..4095 = a persistent speed cap applied to subsequent Goal_Position
 * writes. `Servo_ConfigureForTrajectory` wrote an unlabeled 65 here for
 * every joint, which is a severe cap out of the 4095 range.
 *
 * 2026-08-07 speed-ramp evidence: a 250 raw/s leg left SHOULDER 713 raw
 * short at trajectory end, then closed that gap at a strikingly constant
 * ~65 raw/s -- matching this register's value almost exactly. Buffered
 * streaming re-writes Goal_Position every 20 ms in small steps and never
 * seemed to hit this cap, but the one large catch-up move to the final
 * held position clearly did. Raised to keep some governor rather than
 * removing it outright (0).
 */
#define SERVO_GOAL_SPEED_RAW UINT16_C(800)

#if SERVO_SHOULDER_TORQUE_LIMIT_RAW >= SERVO_MOTION_LOAD_LIMIT_RAW
#error "Shoulder torque cap must remain below the load safety threshold"
#endif

#if SERVO_ELBOW_TORQUE_LIMIT_RAW >= SERVO_MOTION_LOAD_LIMIT_RAW
#error "Elbow torque cap must remain below the load safety threshold"
#endif

#endif /* SINGLE_ARM_CONFIG_H */
