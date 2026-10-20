/*
 * AppleVirgl QMetal frame-pump state machine
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "apple-virgl-frame-pump.h"

static AppleVirglFramePumpActions apple_virgl_frame_pump_no_actions(void)
{
    return (AppleVirglFramePumpActions) { 0 };
}

static bool apple_virgl_frame_pump_can_schedule_submit(
    const AppleVirglFramePump *pump)
{
    return !pump->resetting && !pump->shutdown &&
           pump->captured_requests != 0 &&
           pump->submitted_inflight == 0 &&
           !pump->submit_scheduled && !pump->submit_active;
}

static AppleVirglFramePumpActions apple_virgl_frame_pump_schedule_ready(
    AppleVirglFramePump *pump)
{
    AppleVirglFramePumpActions actions = apple_virgl_frame_pump_no_actions();

    if (!pump->resetting && !pump->shutdown &&
        pump->pending_signal_events != 0 && !pump->bh_scheduled) {
        pump->bh_scheduled = true;
        actions.schedule_bh = true;
    }
    if (apple_virgl_frame_pump_can_schedule_submit(pump)) {
        pump->submit_scheduled = true;
        actions.schedule_submit = true;
    }
    return actions;
}

void apple_virgl_frame_pump_init(AppleVirglFramePump *pump)
{
    if (!pump) {
        return;
    }
    *pump = (AppleVirglFramePump) {
        .generation = 1,
    };
}

bool apple_virgl_frame_pump_new_signal(AppleVirglFramePump *pump)
{
    if (!pump || pump->resetting || pump->shutdown) {
        return false;
    }

    pump->pending_signal_events++;
    if (pump->bh_scheduled) {
        return false;
    }
    pump->bh_scheduled = true;
    return true;
}

bool apple_virgl_frame_pump_begin_bh(AppleVirglFramePump *pump,
                                     uint64_t *out_generation)
{
    if (!pump) {
        return false;
    }

    pump->bh_scheduled = false;
    if (pump->resetting || pump->shutdown ||
        pump->pending_signal_events == 0 || pump->bh_active) {
        return false;
    }

    pump->pending_signal_events--;
    pump->bh_active = true;
    if (out_generation) {
        *out_generation = pump->generation;
    }
    return true;
}

AppleVirglFramePumpActions
apple_virgl_frame_pump_finish_capture(AppleVirglFramePump *pump,
                                      uint64_t generation,
                                      int capture_result)
{
    AppleVirglFramePumpActions actions = apple_virgl_frame_pump_no_actions();

    if (!pump) {
        return actions;
    }

    pump->bh_active = false;
    if (!pump->resetting && !pump->shutdown &&
        generation == pump->generation && capture_result > 0) {
        pump->captured_requests++;
    }
    return apple_virgl_frame_pump_schedule_ready(pump);
}

bool apple_virgl_frame_pump_begin_submit(AppleVirglFramePump *pump,
                                         uint64_t *out_generation)
{
    if (!pump) {
        return false;
    }

    pump->submit_scheduled = false;
    if (pump->resetting || pump->shutdown || pump->submit_active ||
        pump->captured_requests == 0 || pump->submitted_inflight != 0) {
        return false;
    }

    pump->submit_active = true;
    if (out_generation) {
        *out_generation = pump->generation;
    }
    return true;
}

AppleVirglFramePumpActions
apple_virgl_frame_pump_finish_submit(AppleVirglFramePump *pump,
                                     uint64_t generation,
                                     int submit_result)
{
    AppleVirglFramePumpActions actions = apple_virgl_frame_pump_no_actions();

    if (!pump) {
        return actions;
    }

    pump->submit_active = false;
    if (pump->resetting || pump->shutdown || generation != pump->generation) {
        return actions;
    }

    if (submit_result > 0) {
        if (pump->captured_requests != 0) {
            pump->captured_requests--;
        }
        pump->submitted_inflight++;

        /* A completion can legally arrive inline on an error/fallback path.
         * Consume it only after the matching submit has become visible. */
        if (pump->early_completions != 0) {
            pump->early_completions--;
            pump->submitted_inflight--;
        }
        return apple_virgl_frame_pump_schedule_ready(pump);
    }

    /* submit == 0 retains QMetal's FIFO head for a producer that has not
     * retired yet.  Do not poll or synthesize a frame; a later legitimate
     * source/completion event is the only permitted retry edge. */
    pump->early_completions = 0;
    return actions;
}

AppleVirglFramePumpActions
apple_virgl_frame_pump_frame_completed(AppleVirglFramePump *pump)
{
    AppleVirglFramePumpActions actions = apple_virgl_frame_pump_no_actions();

    if (!pump || pump->resetting || pump->shutdown) {
        return actions;
    }

    if (pump->submitted_inflight != 0) {
        pump->submitted_inflight--;
    } else if (pump->submit_active) {
        pump->early_completions++;
        return actions;
    } else {
        return actions;
    }

    return apple_virgl_frame_pump_schedule_ready(pump);
}

void apple_virgl_frame_pump_begin_reset(AppleVirglFramePump *pump,
                                        bool shutdown)
{
    if (!pump) {
        return;
    }

    pump->resetting = true;
    pump->shutdown = pump->shutdown || shutdown;
    pump->generation++;
    if (pump->generation == 0) {
        pump->generation = 1;
    }
    pump->pending_signal_events = 0;
    pump->captured_requests = 0;
    pump->submitted_inflight = 0;
    pump->early_completions = 0;
    pump->bh_scheduled = false;
}

void apple_virgl_frame_pump_resume(AppleVirglFramePump *pump)
{
    if (!pump || pump->shutdown) {
        return;
    }
    pump->resetting = false;
}

bool apple_virgl_frame_pump_is_idle(const AppleVirglFramePump *pump)
{
    return !pump || (!pump->bh_active && !pump->submit_scheduled &&
                     !pump->submit_active);
}
