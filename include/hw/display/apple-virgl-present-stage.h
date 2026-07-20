/*
 * AppleVirgl owner-frame presentation staging
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_APPLE_VIRGL_PRESENT_STAGE_H
#define HW_DISPLAY_APPLE_VIRGL_PRESENT_STAGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hw/display/apple-virgl-p4-semantic-ledger.h"

/*
 * The stage is deliberately independent of QEMU display objects and QMU.
 * Its caller serializes all operations (normally with the bridge presenter
 * mutex), then hands the owned job to the main-loop presenter.  In
 * particular, it never retains the producer's pixels pointer.
 */
typedef enum AppleVirglPresentStageJobKind {
    APPLE_VIRGL_PRESENT_STAGE_FRAME_COMPLETION,
    APPLE_VIRGL_PRESENT_STAGE_MODE_CHANGE,
} AppleVirglPresentStageJobKind;

typedef struct AppleVirglPresentStageJob {
    struct AppleVirglPresentStageJob *next;
    AppleVirglPresentStageJobKind kind;
    bool frame_expected;
    bool payload_valid;
    uint8_t *pixels;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    /* Immutable QMetal P4 semantic-ledger id; zero keeps diagnostics inert. */
    uint64_t p4_ledger_id;
    AppleVirglP4OwnerBackingIdentity p4_owner_backing;
    uint32_t iosurface_pixel_format;
    uint64_t protection_requirements;
    size_t pixel_bytes;
} AppleVirglPresentStageJob;

typedef struct AppleVirglPresentStage {
    AppleVirglPresentStageJob *head;
    AppleVirglPresentStageJob *tail;
    size_t pending_jobs;
    bool resetting;
    bool shutdown;
} AppleVirglPresentStage;

void apple_virgl_present_stage_init(AppleVirglPresentStage *stage);

/*
 * Enqueue one terminal owner-completion token.  A false frame_expected value
 * produces a token without a pixel payload.  If frame_expected is true but
 * its payload is malformed, the token is still queued with payload_valid
 * false so the caller can retire the matching frame-pump work.  False is
 * returned only when the stage rejects new work during reset/shutdown, has
 * exhausted its queue counter, or is NULL.
 */
bool apple_virgl_present_stage_enqueue(AppleVirglPresentStage *stage,
                                       bool frame_expected,
                                       const void *pixels,
                                       uint32_t width,
                                       uint32_t height,
                                       uint32_t stride);

/* As above, while retaining the immutable P4 identity through the FIFO. */
bool apple_virgl_present_stage_enqueue_tagged(
    AppleVirglPresentStage *stage, bool frame_expected, const void *pixels,
    uint32_t width, uint32_t height, uint32_t stride, uint64_t p4_ledger_id,
    const AppleVirglP4OwnerBackingIdentity *p4_owner_backing);

/*
 * Enqueue one QMetal mode edge before its later frame publication.  The stage
 * owns only scalar callback data; the BQL consumer decides whether equal
 * geometry is a QEMU surface no-op.  Reject dimensions that cannot form the
 * signed four-byte-per-pixel QEMU stride and signed surface allocation size.
 */
bool apple_virgl_present_stage_enqueue_mode(
    AppleVirglPresentStage *stage, uint32_t width, uint32_t height,
    uint32_t iosurface_pixel_format, uint64_t protection_requirements);

/* Transfer the oldest queued mode event or terminal token to the caller. */
AppleVirglPresentStageJob *
apple_virgl_present_stage_take(AppleVirglPresentStage *stage);

/*
 * Detach the complete FIFO without freeing it.  The caller owns the returned
 * list and must free every job.  The bridge uses this at reset to emit an
 * explicit diagnostic receipt for tagged work that cannot reach scanout.
 */
AppleVirglPresentStageJob *
apple_virgl_present_stage_detach_all(AppleVirglPresentStage *stage);

void apple_virgl_present_stage_job_free(AppleVirglPresentStageJob *job);

/* Begin reset/teardown, reject new work, and free every queued token. */
void apple_virgl_present_stage_begin_reset(AppleVirglPresentStage *stage,
                                           bool shutdown);

/* Explicitly discard queued work; taken jobs remain caller-owned. */
void apple_virgl_present_stage_drain(AppleVirglPresentStage *stage);

/* Reopen a non-shutdown stage after a reset. */
void apple_virgl_present_stage_resume(AppleVirglPresentStage *stage);

bool apple_virgl_present_stage_is_idle(const AppleVirglPresentStage *stage);

#endif /* HW_DISPLAY_APPLE_VIRGL_PRESENT_STAGE_H */
