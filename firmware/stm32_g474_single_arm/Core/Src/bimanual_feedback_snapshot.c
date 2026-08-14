#include "bimanual_feedback_snapshot.h"

#include <stddef.h>
#include <string.h>

typedef struct
{
    uint16_t present_mask;
    uint32_t completed_pairs;
    int32_t positions_urad[BIMANUAL_FEEDBACK_JOINT_COUNT];
    uint32_t sampled_at_ms[BIMANUAL_FEEDBACK_JOINT_COUNT];
} BimanualFeedbackSnapshotState;

static BimanualFeedbackSnapshotState feedback = {0};

void BimanualFeedbackSnapshot_Reset(void)
{
    memset(&feedback, 0, sizeof(feedback));
}

void BimanualFeedbackSnapshot_Seed(
    const int32_t positions_urad[BIMANUAL_FEEDBACK_JOINT_COUNT],
    uint32_t sampled_at_ms)
{
    BimanualFeedbackSnapshot_Reset();
    if (positions_urad == NULL)
    {
        return;
    }
    memcpy(feedback.positions_urad, positions_urad,
           sizeof(feedback.positions_urad));
    for (uint8_t joint = 0U;
         joint < BIMANUAL_FEEDBACK_JOINT_COUNT;
         joint++)
    {
        feedback.sampled_at_ms[joint] = sampled_at_ms;
    }
    feedback.present_mask = BIMANUAL_FEEDBACK_COMPLETE_MASK;
}

void BimanualFeedbackSnapshot_UpdatePair(
    uint8_t arm_joint,
    int32_t left_position_urad,
    int32_t right_position_urad,
    uint32_t sampled_at_ms)
{
    const uint8_t right_joint =
        (uint8_t)(arm_joint + BIMANUAL_FEEDBACK_ARM_JOINT_COUNT);
    if (arm_joint >= BIMANUAL_FEEDBACK_ARM_JOINT_COUNT)
    {
        return;
    }
    feedback.positions_urad[arm_joint] = left_position_urad;
    feedback.positions_urad[right_joint] = right_position_urad;
    feedback.sampled_at_ms[arm_joint] = sampled_at_ms;
    feedback.sampled_at_ms[right_joint] = sampled_at_ms;
    feedback.present_mask |= (uint16_t)(UINT16_C(1) << arm_joint);
    feedback.present_mask |= (uint16_t)(UINT16_C(1) << right_joint);
    feedback.completed_pairs++;
}

void BimanualFeedbackSnapshot_Copy(
    uint32_t now_ms,
    BimanualFeedbackSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->present_mask = feedback.present_mask;
    snapshot->firmware_tick_ms = now_ms;
    snapshot->completed_pairs = feedback.completed_pairs;
    memcpy(snapshot->positions_urad, feedback.positions_urad,
           sizeof(snapshot->positions_urad));
    for (uint8_t joint = 0U;
         joint < BIMANUAL_FEEDBACK_JOINT_COUNT;
         joint++)
    {
        snapshot->sample_age_ms[joint] =
            ((feedback.present_mask &
              (uint16_t)(UINT16_C(1) << joint)) == 0U) ?
            UINT32_MAX : (now_ms - feedback.sampled_at_ms[joint]);
    }
}
