#include <stdio.h>

#include "actuator_core/bimanual_dispatch.h"

static int failures;

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    ++failures; return; } } while (0)

static void test_atomic_pair_completion_and_metrics(void) {
    actuator_bimanual_dispatch_t dispatch;
    const actuator_bimanual_dispatch_snapshot_t *snapshot;

    actuator_bimanual_dispatch_init(&dispatch);
    CHECK(actuator_bimanual_dispatch_can_launch(&dispatch));
    CHECK(actuator_bimanual_dispatch_begin(
              &dispatch, 100u, 500000u, 500012u, 500017u) ==
          ACTUATOR_BIMANUAL_DISPATCH_OK);
    CHECK(!actuator_bimanual_dispatch_can_launch(&dispatch));
    CHECK(actuator_bimanual_dispatch_begin(
              &dispatch, 105u, 505000u, 505010u, 505014u) ==
          ACTUATOR_BIMANUAL_DISPATCH_BUSY);
    CHECK(actuator_bimanual_dispatch_complete_left(&dispatch) ==
          ACTUATOR_BIMANUAL_DISPATCH_OK);
    CHECK(!actuator_bimanual_dispatch_can_launch(&dispatch));
    CHECK(actuator_bimanual_dispatch_complete_right(&dispatch) ==
          ACTUATOR_BIMANUAL_DISPATCH_OK);
    CHECK(actuator_bimanual_dispatch_can_launch(&dispatch));

    snapshot = actuator_bimanual_dispatch_snapshot(&dispatch);
    CHECK(snapshot != NULL);
    CHECK(snapshot->launch_count == 1u);
    CHECK(snapshot->completed_count == 1u);
    CHECK(snapshot->maximum_start_skew_us == 5u);
    CHECK(snapshot->maximum_launch_lateness_us == 12u);
}

static void test_transport_failure_is_latched(void) {
    actuator_bimanual_dispatch_t dispatch;
    const actuator_bimanual_dispatch_snapshot_t *snapshot;

    actuator_bimanual_dispatch_init(&dispatch);
    CHECK(actuator_bimanual_dispatch_begin(
              &dispatch, 200u, 1000000u, 1000004u, 1000008u) ==
          ACTUATOR_BIMANUAL_DISPATCH_OK);
    actuator_bimanual_dispatch_fail(&dispatch);
    actuator_bimanual_dispatch_fail(&dispatch);
    CHECK(!actuator_bimanual_dispatch_can_launch(&dispatch));
    CHECK(actuator_bimanual_dispatch_complete_left(&dispatch) ==
          ACTUATOR_BIMANUAL_DISPATCH_BAD_STATE);
    snapshot = actuator_bimanual_dispatch_snapshot(&dispatch);
    CHECK(snapshot->failure_count == 1u);
    CHECK(snapshot->faulted);
}

static void test_invalid_start_order_is_rejected(void) {
    actuator_bimanual_dispatch_t dispatch;

    actuator_bimanual_dispatch_init(&dispatch);
    CHECK(actuator_bimanual_dispatch_begin(
              &dispatch, 1u, 10u, 20u, 19u) ==
          ACTUATOR_BIMANUAL_DISPATCH_BAD_STATE);
    CHECK(actuator_bimanual_dispatch_can_launch(&dispatch));
}

int main(void) {
    test_atomic_pair_completion_and_metrics();
    test_transport_failure_is_latched();
    test_invalid_start_order_is_rejected();
    if (failures != 0) {
        fprintf(stderr, "%d test(s) failed\n", failures);
        return 1;
    }
    puts("bimanual dispatch tests passed");
    return 0;
}
