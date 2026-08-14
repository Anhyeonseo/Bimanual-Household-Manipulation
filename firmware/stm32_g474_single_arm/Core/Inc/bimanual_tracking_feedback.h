#ifndef BIMANUAL_TRACKING_FEEDBACK_H
#define BIMANUAL_TRACKING_FEEDBACK_H

#include "stm32g4xx_hal.h"

#include <stdint.h>

#define BIMANUAL_TRACKING_ARM_JOINT_COUNT UINT8_C(6)

typedef enum
{
    BIMANUAL_TRACKING_IDLE = 0,
    BIMANUAL_TRACKING_PENDING = 1,
    BIMANUAL_TRACKING_SAMPLE_READY = 2,
    BIMANUAL_TRACKING_TRANSIENT_FAILURE = 3,
    BIMANUAL_TRACKING_FAULT = 4
} BimanualTrackingFeedbackResult;

typedef struct
{
    uint8_t joint_index;
    uint16_t left_position_raw;
    uint16_t right_position_raw;
    int32_t left_commanded_urad;
    int32_t right_commanded_urad;
} BimanualTrackingFeedbackSample;

typedef struct
{
    uint32_t requested_pairs;
    uint32_t completed_pairs;
    uint32_t failed_pairs;
    uint32_t maximum_reply_latency_ms;
    uint8_t consecutive_failed_pairs;
    uint8_t active;
    uint8_t pending;
} BimanualTrackingFeedbackSnapshot;

HAL_StatusTypeDef BimanualTrackingFeedback_Begin(void);
void BimanualTrackingFeedback_End(void);
uint8_t BimanualTrackingFeedback_Active(void);
uint8_t BimanualTrackingFeedback_Pending(void);
HAL_StatusTypeDef BimanualTrackingFeedback_Start(
    uint8_t joint_index,
    uint32_t started_at_ms,
    const uint16_t left_commanded_raw[BIMANUAL_TRACKING_ARM_JOINT_COUNT],
    const uint16_t right_commanded_raw[BIMANUAL_TRACKING_ARM_JOINT_COUNT],
    int32_t left_commanded_urad,
    int32_t right_commanded_urad
);
BimanualTrackingFeedbackResult BimanualTrackingFeedback_Poll(
    uint32_t now_ms,
    BimanualTrackingFeedbackSample *sample
);
const BimanualTrackingFeedbackSnapshot *
BimanualTrackingFeedback_GetSnapshot(void);

#endif
