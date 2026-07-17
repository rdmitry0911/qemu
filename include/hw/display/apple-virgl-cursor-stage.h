/*
 * AppleVirgl cursor delivery staging
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_APPLE_VIRGL_CURSOR_STAGE_H
#define HW_DISPLAY_APPLE_VIRGL_CURSOR_STAGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum AppleVirglCursorStageKind {
    APPLE_VIRGL_CURSOR_STAGE_GLYPH,
    APPLE_VIRGL_CURSOR_STAGE_SHOW,
    APPLE_VIRGL_CURSOR_STAGE_MOVE,
} AppleVirglCursorStageKind;

/*
 * A glyph payload is row-packed in its producer byte order.  The QEMU cursor
 * channel conversion is deliberately deferred to the main-loop consumer,
 * where QEMUCursor ownership and UI publication are legal.
 */
typedef struct AppleVirglCursorStageJob {
    struct AppleVirglCursorStageJob *next;
    AppleVirglCursorStageKind kind;
    uint8_t *pixels;
    size_t pixel_bytes;
    uint32_t width;
    uint32_t height;
    uint32_t hot_x;
    uint32_t hot_y;
    uint32_t display_id;
    bool visible;
} AppleVirglCursorStageJob;

typedef struct AppleVirglCursorStage {
    AppleVirglCursorStageJob *head;
    AppleVirglCursorStageJob *tail;
    size_t pending_jobs;
    bool resetting;
    bool shutdown;
} AppleVirglCursorStage;

void apple_virgl_cursor_stage_init(AppleVirglCursorStage *stage);

/*
 * Make an owned, row-packed copy of one ephemeral QMetal glyph callback.
 * Returning false means that no event was queued (invalid geometry/allocation
 * failure, or a reset/shutdown stage), so callers must not retain pixels.
 */
bool apple_virgl_cursor_stage_enqueue_glyph(AppleVirglCursorStage *stage,
                                            const void *pixels,
                                            uint64_t mapped_length,
                                            uint64_t stride,
                                            uint32_t width,
                                            uint32_t height,
                                            uint32_t hot_x,
                                            uint32_t hot_y);

bool apple_virgl_cursor_stage_enqueue_show(AppleVirglCursorStage *stage,
                                           uint32_t display_id,
                                           bool visible);
bool apple_virgl_cursor_stage_enqueue_move(AppleVirglCursorStage *stage,
                                           uint32_t display_id);

AppleVirglCursorStageJob *
apple_virgl_cursor_stage_take(AppleVirglCursorStage *stage);
void apple_virgl_cursor_stage_job_free(AppleVirglCursorStageJob *job);

/*
 * Convert a row-packed producer glyph to the QEMUCursor uint32_t layout used
 * by the reference host: R << 16 | G << 8 | B | A << 24.
 */
bool apple_virgl_cursor_stage_fill_qemu_cursor(
    const AppleVirglCursorStageJob *job, uint32_t *destination,
    size_t destination_pixels);

void apple_virgl_cursor_stage_begin_reset(AppleVirglCursorStage *stage,
                                          bool shutdown);
void apple_virgl_cursor_stage_drain(AppleVirglCursorStage *stage);
void apple_virgl_cursor_stage_resume(AppleVirglCursorStage *stage);
bool apple_virgl_cursor_stage_is_idle(const AppleVirglCursorStage *stage);

#endif /* HW_DISPLAY_APPLE_VIRGL_CURSOR_STAGE_H */
