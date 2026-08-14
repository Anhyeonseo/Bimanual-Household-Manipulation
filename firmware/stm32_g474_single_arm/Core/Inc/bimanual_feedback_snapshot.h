#ifndef BIMANUAL_FEEDBACK_SNAPSHOT_H
#define BIMANUAL_FEEDBACK_SNAPSHOT_H

#include <stdint.h>

#define BIMANUAL_FEEDBACK_JOINT_COUNT UINT8_C(12)
#define BIMANUAL_FEEDBACK_ARM_JOINT_COUNT UINT8_C(6)
#define BIMANUAL_FEEDBACK_COMPLETE_MASK UINT16_C(0x0FFF)

typedef struct
{
    uint16_t present_mask;
    uint32_t firmware_tick_ms;
    uint32_t completed_pairs;
    int32_t positions_urad[BIMANUAL_FEEDBACK_JOINT_COUNT];
    uint32_t sample_age_ms[BIMANUAL_FEEDBACK_JOINT_COUNT];
} BimanualFeedbackSnapshot;

void BimanualFeedbackSnapshot_Reset(void);
void BimanualFeedbackSnapshot_Seed(
    const int32_t positions_urad[BIMANUAL_FEEDBACK_JOINT_COUNT],
    uint32_t sampled_at_ms
);
void BimanualFeedbackSnapshot_UpdatePair(
    uint8_t arm_joint,
    int32_t left_position_urad,
    int32_t right_position_urad,
    uint32_t sampled_at_ms
);
void BimanualFeedbackSnapshot_Copy(
    uint32_t now_ms,
    BimanualFeedbackSnapshot *snapshot
);

#endif
