#include "bimanual_tracking_feedback.h"

#include "right_servo_bus.h"
#include "servo_bus.h"
#include "single_arm_config.h"

#include <stddef.h>
#include <string.h>

typedef struct
{
    BimanualTrackingFeedbackSnapshot snapshot;
    uint8_t joint_index;
    uint32_t started_at_ms;
    uint16_t left_commanded_raw[BIMANUAL_TRACKING_ARM_JOINT_COUNT];
    uint16_t right_commanded_raw[BIMANUAL_TRACKING_ARM_JOINT_COUNT];
    int32_t left_commanded_urad;
    int32_t right_commanded_urad;
} BimanualTrackingFeedbackState;

static BimanualTrackingFeedbackState tracking = {0};

static BimanualTrackingFeedbackResult RecordPairFailure(void)
{
    tracking.snapshot.failed_pairs++;
    tracking.snapshot.pending = 0U;
    if (tracking.snapshot.consecutive_failed_pairs < UINT8_MAX)
    {
        tracking.snapshot.consecutive_failed_pairs++;
    }
    if (tracking.snapshot.consecutive_failed_pairs >=
        HOST_BIMANUAL_TRACKING_READ_FAILURE_LIMIT)
    {
        return BIMANUAL_TRACKING_FAULT;
    }
    return BIMANUAL_TRACKING_TRANSIENT_FAILURE;
}

HAL_StatusTypeDef BimanualTrackingFeedback_Begin(void)
{
    if (tracking.snapshot.active != 0U)
    {
        return HAL_BUSY;
    }
    memset(&tracking, 0, sizeof(tracking));
    Servo_InMotionTelemetryBegin();
    if (RightServoBus_InMotionTelemetryBegin() != HAL_OK)
    {
        Servo_InMotionTelemetryEnd();
        return HAL_ERROR;
    }
    tracking.snapshot.active = 1U;
    return HAL_OK;
}

void BimanualTrackingFeedback_End(void)
{
    Servo_InMotionTelemetryEnd();
    RightServoBus_InMotionTelemetryEnd();
    tracking.snapshot.active = 0U;
    tracking.snapshot.pending = 0U;
}

uint8_t BimanualTrackingFeedback_Active(void)
{
    return tracking.snapshot.active;
}

uint8_t BimanualTrackingFeedback_Pending(void)
{
    return tracking.snapshot.pending;
}

HAL_StatusTypeDef BimanualTrackingFeedback_Start(
    uint8_t joint_index,
    uint32_t started_at_ms,
    const uint16_t left_commanded_raw[BIMANUAL_TRACKING_ARM_JOINT_COUNT],
    const uint16_t right_commanded_raw[BIMANUAL_TRACKING_ARM_JOINT_COUNT],
    int32_t left_commanded_urad,
    int32_t right_commanded_urad)
{
    if ((tracking.snapshot.active == 0U) ||
        (tracking.snapshot.pending != 0U) ||
        (joint_index >= BIMANUAL_TRACKING_ARM_JOINT_COUNT) ||
        (left_commanded_raw == NULL) || (right_commanded_raw == NULL))
    {
        return HAL_BUSY;
    }
    tracking.joint_index = joint_index;
    tracking.started_at_ms = started_at_ms;
    memcpy(tracking.left_commanded_raw, left_commanded_raw,
           sizeof(tracking.left_commanded_raw));
    memcpy(tracking.right_commanded_raw, right_commanded_raw,
           sizeof(tracking.right_commanded_raw));
    tracking.left_commanded_urad = left_commanded_urad;
    tracking.right_commanded_urad = right_commanded_urad;
    if (Servo_InMotionTelemetryStart(joint_index, started_at_ms) != HAL_OK)
    {
        tracking.snapshot.failed_pairs++;
        return HAL_ERROR;
    }
    if (RightServoBus_InMotionTelemetryStart(
            joint_index, started_at_ms) != HAL_OK)
    {
        tracking.snapshot.failed_pairs++;
        BimanualTrackingFeedback_End();
        return HAL_ERROR;
    }
    tracking.snapshot.requested_pairs++;
    tracking.snapshot.pending = 1U;
    return HAL_OK;
}

BimanualTrackingFeedbackResult BimanualTrackingFeedback_Poll(
    uint32_t now_ms,
    BimanualTrackingFeedbackSample *sample)
{
    HAL_StatusTypeDef left_status;
    HAL_StatusTypeDef right_status;
    const ServoInMotionTelemetrySnapshot *left;
    const RightServoInMotionTelemetrySnapshot *right;

    if ((tracking.snapshot.active == 0U) ||
        (tracking.snapshot.pending == 0U))
    {
        return BIMANUAL_TRACKING_IDLE;
    }
    left_status = Servo_InMotionTelemetryPoll(
        now_ms, tracking.left_commanded_raw);
    right_status = RightServoBus_InMotionTelemetryPoll(
        now_ms, tracking.right_commanded_raw);
    if ((left_status == HAL_ERROR) || (left_status == HAL_TIMEOUT) ||
        (right_status == HAL_ERROR) || (right_status == HAL_TIMEOUT))
    {
        return RecordPairFailure();
    }
    if ((Servo_InMotionTelemetryPending() != 0U) ||
        (RightServoBus_InMotionTelemetryPending() != 0U))
    {
        return BIMANUAL_TRACKING_PENDING;
    }
    left = Servo_InMotionTelemetryGetSnapshot();
    right = RightServoBus_InMotionTelemetryGetSnapshot();
    if ((left == NULL) || (right == NULL) ||
        (left->last_joint_index != tracking.joint_index) ||
        (right->last_joint_index != tracking.joint_index) ||
        (sample == NULL))
    {
        return RecordPairFailure();
    }
    sample->joint_index = tracking.joint_index;
    sample->left_position_raw = left->last_position_raw;
    sample->right_position_raw = right->last_position_raw;
    sample->left_commanded_urad = tracking.left_commanded_urad;
    sample->right_commanded_urad = tracking.right_commanded_urad;
    tracking.snapshot.completed_pairs++;
    tracking.snapshot.consecutive_failed_pairs = 0U;
    tracking.snapshot.pending = 0U;
    {
        const uint32_t latency = now_ms - tracking.started_at_ms;
        if (latency > tracking.snapshot.maximum_reply_latency_ms)
        {
            tracking.snapshot.maximum_reply_latency_ms = latency;
        }
    }
    return BIMANUAL_TRACKING_SAMPLE_READY;
}

const BimanualTrackingFeedbackSnapshot *
BimanualTrackingFeedback_GetSnapshot(void)
{
    return &tracking.snapshot;
}
