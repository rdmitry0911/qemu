/*
 * AppleVirgl QMetal frame-pump state machine
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_APPLE_VIRGL_FRAME_PUMP_H
#define HW_DISPLAY_APPLE_VIRGL_FRAME_PUMP_H

#include <stdbool.h>
#include <stdint.h>

/*
 * This is deliberately a lock-free-in-the-small state machine: its caller
 * serializes every operation with the bridge frame-pump mutex, then performs
 * the returned BH/worker actions after dropping that mutex.  Keeping it free
 * of QMU and QEMU calls makes its reset and FIFO rules unit-testable.
 */
typedef struct AppleVirglFramePump {
    uint64_t generation;
    uint32_t pending_signal_events;
    uint32_t captured_requests;
    uint32_t submitted_inflight;
    uint32_t early_completions;
    bool resetting;
    bool shutdown;
    bool bh_scheduled;
    bool bh_active;
    bool submit_scheduled;
    bool submit_active;
} AppleVirglFramePump;

typedef struct AppleVirglFramePumpActions {
    bool schedule_bh;
    bool schedule_submit;
} AppleVirglFramePumpActions;

void apple_virgl_frame_pump_init(AppleVirglFramePump *pump);

/* Called only after new_frame_signal has acknowledged its QMetal signal. */
bool apple_virgl_frame_pump_new_signal(AppleVirglFramePump *pump);

bool apple_virgl_frame_pump_begin_bh(AppleVirglFramePump *pump,
                                     uint64_t *out_generation);
AppleVirglFramePumpActions
apple_virgl_frame_pump_finish_capture(AppleVirglFramePump *pump,
                                      uint64_t generation,
                                      int capture_result);

bool apple_virgl_frame_pump_begin_submit(AppleVirglFramePump *pump,
                                         uint64_t *out_generation);
AppleVirglFramePumpActions
apple_virgl_frame_pump_finish_submit(AppleVirglFramePump *pump,
                                     uint64_t generation,
                                     int submit_result);

/* A terminal QMetal frame_completed token can chain one already-captured FIFO
 * request, but it never captures or submits inline. */
AppleVirglFramePumpActions
apple_virgl_frame_pump_frame_completed(AppleVirglFramePump *pump);

/* The caller cancels the actual QEMU BH after this state transition. */
void apple_virgl_frame_pump_begin_reset(AppleVirglFramePump *pump,
                                        bool shutdown);
void apple_virgl_frame_pump_resume(AppleVirglFramePump *pump);
bool apple_virgl_frame_pump_is_idle(const AppleVirglFramePump *pump);

#endif /* HW_DISPLAY_APPLE_VIRGL_FRAME_PUMP_H */
