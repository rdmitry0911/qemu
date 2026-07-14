/*
 * AppleVirgl frame-pump state-machine tests
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/display/apple-virgl-frame-pump.h"

static void test_signal_capture_submit_order(void)
{
    AppleVirglFramePump pump;
    AppleVirglFramePumpActions actions;
    uint64_t generation;

    apple_virgl_frame_pump_init(&pump);
    g_assert_true(apple_virgl_frame_pump_new_signal(&pump));
    g_assert_true(apple_virgl_frame_pump_begin_bh(&pump, &generation));
    actions = apple_virgl_frame_pump_finish_capture(&pump, generation, 1);
    g_assert_false(actions.schedule_bh);
    g_assert_true(actions.schedule_submit);
    g_assert_true(apple_virgl_frame_pump_begin_submit(&pump, &generation));
    actions = apple_virgl_frame_pump_finish_submit(&pump, generation, 1);
    g_assert_false(actions.schedule_bh);
    g_assert_false(actions.schedule_submit);
    g_assert_cmpuint(pump.submitted_inflight, ==, 1);
    actions = apple_virgl_frame_pump_frame_completed(&pump);
    g_assert_false(actions.schedule_bh);
    g_assert_false(actions.schedule_submit);
    g_assert_cmpuint(pump.submitted_inflight, ==, 0);
}

static void test_resignal_preserves_bh_edges_and_fifo_continuation(void)
{
    AppleVirglFramePump pump;
    AppleVirglFramePumpActions actions;
    uint64_t generation;

    apple_virgl_frame_pump_init(&pump);
    g_assert_true(apple_virgl_frame_pump_new_signal(&pump));
    g_assert_false(apple_virgl_frame_pump_new_signal(&pump));
    g_assert_cmpuint(pump.pending_signal_events, ==, 2);

    g_assert_true(apple_virgl_frame_pump_begin_bh(&pump, &generation));
    actions = apple_virgl_frame_pump_finish_capture(&pump, generation, 1);
    g_assert_true(actions.schedule_bh);
    g_assert_true(actions.schedule_submit);
    g_assert_true(apple_virgl_frame_pump_begin_submit(&pump, &generation));
    (void)apple_virgl_frame_pump_finish_submit(&pump, generation, 1);

    g_assert_true(apple_virgl_frame_pump_begin_bh(&pump, &generation));
    actions = apple_virgl_frame_pump_finish_capture(&pump, generation, 1);
    g_assert_false(actions.schedule_submit);
    g_assert_cmpuint(pump.captured_requests, ==, 1);

    actions = apple_virgl_frame_pump_frame_completed(&pump);
    g_assert_true(actions.schedule_submit);
    g_assert_true(apple_virgl_frame_pump_begin_submit(&pump, &generation));
    (void)apple_virgl_frame_pump_finish_submit(&pump, generation, 1);
    g_assert_cmpuint(pump.captured_requests, ==, 0);
}

static void test_reset_fences_stale_bh_and_post_reset_signal(void)
{
    AppleVirglFramePump pump;
    AppleVirglFramePumpActions actions;
    uint64_t old_generation;
    uint64_t new_generation;

    apple_virgl_frame_pump_init(&pump);
    g_assert_true(apple_virgl_frame_pump_new_signal(&pump));
    g_assert_true(apple_virgl_frame_pump_begin_bh(&pump, &old_generation));
    apple_virgl_frame_pump_begin_reset(&pump, false);
    actions = apple_virgl_frame_pump_finish_capture(&pump, old_generation, 1);
    g_assert_false(actions.schedule_bh);
    g_assert_false(actions.schedule_submit);
    g_assert_true(apple_virgl_frame_pump_is_idle(&pump));

    apple_virgl_frame_pump_resume(&pump);
    g_assert_true(apple_virgl_frame_pump_new_signal(&pump));
    g_assert_true(apple_virgl_frame_pump_begin_bh(&pump, &new_generation));
    g_assert_cmpuint(new_generation, >, old_generation);
    actions = apple_virgl_frame_pump_finish_capture(&pump, new_generation, 1);
    g_assert_true(actions.schedule_submit);
}

static void test_deferred_submit_waits_for_legitimate_new_event(void)
{
    AppleVirglFramePump pump;
    AppleVirglFramePumpActions actions;
    uint64_t generation;

    apple_virgl_frame_pump_init(&pump);
    g_assert_true(apple_virgl_frame_pump_new_signal(&pump));
    g_assert_true(apple_virgl_frame_pump_begin_bh(&pump, &generation));
    actions = apple_virgl_frame_pump_finish_capture(&pump, generation, 1);
    g_assert_true(actions.schedule_submit);
    g_assert_true(apple_virgl_frame_pump_begin_submit(&pump, &generation));
    actions = apple_virgl_frame_pump_finish_submit(&pump, generation, 0);
    g_assert_false(actions.schedule_bh);
    g_assert_false(actions.schedule_submit);
    g_assert_cmpuint(pump.captured_requests, ==, 1);

    g_assert_true(apple_virgl_frame_pump_new_signal(&pump));
    g_assert_true(apple_virgl_frame_pump_begin_bh(&pump, &generation));
    actions = apple_virgl_frame_pump_finish_capture(&pump, generation, 0);
    g_assert_true(actions.schedule_submit);
}

static void test_inline_completion_waits_for_submit_visibility(void)
{
    AppleVirglFramePump pump;
    AppleVirglFramePumpActions actions;
    uint64_t generation;

    apple_virgl_frame_pump_init(&pump);
    g_assert_true(apple_virgl_frame_pump_new_signal(&pump));
    g_assert_true(apple_virgl_frame_pump_begin_bh(&pump, &generation));
    actions = apple_virgl_frame_pump_finish_capture(&pump, generation, 1);
    g_assert_true(actions.schedule_submit);
    g_assert_true(apple_virgl_frame_pump_begin_submit(&pump, &generation));

    actions = apple_virgl_frame_pump_frame_completed(&pump);
    g_assert_false(actions.schedule_bh);
    g_assert_false(actions.schedule_submit);
    g_assert_cmpuint(pump.early_completions, ==, 1);

    actions = apple_virgl_frame_pump_finish_submit(&pump, generation, 1);
    g_assert_false(actions.schedule_bh);
    g_assert_false(actions.schedule_submit);
    g_assert_cmpuint(pump.captured_requests, ==, 0);
    g_assert_cmpuint(pump.submitted_inflight, ==, 0);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/apple-virgl/frame-pump/signal-capture-submit-order",
                    test_signal_capture_submit_order);
    g_test_add_func("/apple-virgl/frame-pump/resignal-fifo-continuation",
                    test_resignal_preserves_bh_edges_and_fifo_continuation);
    g_test_add_func("/apple-virgl/frame-pump/reset-fences-stale-work",
                    test_reset_fences_stale_bh_and_post_reset_signal);
    g_test_add_func("/apple-virgl/frame-pump/deferred-submit-event-only",
                    test_deferred_submit_waits_for_legitimate_new_event);
    g_test_add_func("/apple-virgl/frame-pump/inline-completion-ordering",
                    test_inline_completion_waits_for_submit_visibility);
    return g_test_run();
}
