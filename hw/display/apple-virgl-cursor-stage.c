/*
 * AppleVirgl cursor delivery staging
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "hw/display/apple-virgl-cursor-stage.h"

#include <limits.h>

/* QEMU's cursor_alloc() accepts at most 512 pixels in either dimension. */
#define APPLE_VIRGL_QEMU_CURSOR_MAX_DIMENSION 512u

static bool apple_virgl_cursor_stage_accepting(
    const AppleVirglCursorStage *stage)
{
    return stage && !stage->resetting && !stage->shutdown &&
        stage->pending_jobs != SIZE_MAX;
}

static void apple_virgl_cursor_stage_append(AppleVirglCursorStage *stage,
                                            AppleVirglCursorStageJob *job)
{
    if (stage->tail) {
        stage->tail->next = job;
    } else {
        stage->head = job;
    }
    stage->tail = job;
    stage->pending_jobs++;
}

static bool apple_virgl_cursor_stage_copy_glyph(
    AppleVirglCursorStageJob *job, const void *pixels, uint64_t mapped_length,
    uint64_t stride, uint32_t width, uint32_t height)
{
    const uint8_t *source = pixels;
    uint8_t *destination;
    uint64_t row_bytes;
    uint64_t required_bytes;
    uint64_t packed_bytes;
    uint32_t row;

    if (!job || !pixels || width == 0 || height == 0 ||
        width > APPLE_VIRGL_QEMU_CURSOR_MAX_DIMENSION ||
        height > APPLE_VIRGL_QEMU_CURSOR_MAX_DIMENSION) {
        return false;
    }
    row_bytes = (uint64_t)width * 4u;
    if (stride < row_bytes || stride > SIZE_MAX || mapped_length > SIZE_MAX) {
        return false;
    }
    if ((uint64_t)(height - 1u) > (UINT64_MAX - row_bytes) / stride) {
        return false;
    }
    required_bytes = (uint64_t)(height - 1u) * stride + row_bytes;
    if (required_bytes > mapped_length) {
        return false;
    }
    if ((uint64_t)height > UINT64_MAX / row_bytes) {
        return false;
    }
    packed_bytes = (uint64_t)height * row_bytes;
    if (packed_bytes > SIZE_MAX) {
        return false;
    }

    destination = g_try_malloc((size_t)packed_bytes);
    if (!destination) {
        return false;
    }
    for (row = 0; row < height; row++) {
        memcpy(destination + (size_t)row * (size_t)row_bytes,
               source + (size_t)row * (size_t)stride,
               (size_t)row_bytes);
    }
    job->pixels = destination;
    job->pixel_bytes = (size_t)packed_bytes;
    return true;
}

void apple_virgl_cursor_stage_init(AppleVirglCursorStage *stage)
{
    if (!stage) {
        return;
    }
    *stage = (AppleVirglCursorStage) { 0 };
}

bool apple_virgl_cursor_stage_enqueue_glyph(AppleVirglCursorStage *stage,
                                            const void *pixels,
                                            uint64_t mapped_length,
                                            uint64_t stride,
                                            uint32_t width,
                                            uint32_t height,
                                            uint32_t hot_x,
                                            uint32_t hot_y)
{
    AppleVirglCursorStageJob *job;

    if (!apple_virgl_cursor_stage_accepting(stage) || hot_x > INT_MAX ||
        hot_y > INT_MAX) {
        return false;
    }
    job = g_new0(AppleVirglCursorStageJob, 1);
    job->kind = APPLE_VIRGL_CURSOR_STAGE_GLYPH;
    job->width = width;
    job->height = height;
    job->hot_x = hot_x;
    job->hot_y = hot_y;
    if (!apple_virgl_cursor_stage_copy_glyph(job, pixels, mapped_length,
                                              stride, width, height)) {
        apple_virgl_cursor_stage_job_free(job);
        return false;
    }
    apple_virgl_cursor_stage_append(stage, job);
    return true;
}

bool apple_virgl_cursor_stage_enqueue_show(AppleVirglCursorStage *stage,
                                           uint32_t display_id,
                                           bool visible)
{
    AppleVirglCursorStageJob *job;

    if (!apple_virgl_cursor_stage_accepting(stage)) {
        return false;
    }
    job = g_new0(AppleVirglCursorStageJob, 1);
    job->kind = APPLE_VIRGL_CURSOR_STAGE_SHOW;
    job->display_id = display_id;
    job->visible = visible;
    apple_virgl_cursor_stage_append(stage, job);
    return true;
}

bool apple_virgl_cursor_stage_enqueue_move(AppleVirglCursorStage *stage,
                                           uint32_t display_id)
{
    AppleVirglCursorStageJob *job;

    if (!apple_virgl_cursor_stage_accepting(stage)) {
        return false;
    }
    job = g_new0(AppleVirglCursorStageJob, 1);
    job->kind = APPLE_VIRGL_CURSOR_STAGE_MOVE;
    job->display_id = display_id;
    apple_virgl_cursor_stage_append(stage, job);
    return true;
}

AppleVirglCursorStageJob *
apple_virgl_cursor_stage_take(AppleVirglCursorStage *stage)
{
    AppleVirglCursorStageJob *job;

    if (!stage || stage->resetting || stage->shutdown || !stage->head) {
        return NULL;
    }
    job = stage->head;
    stage->head = job->next;
    if (!stage->head) {
        stage->tail = NULL;
    }
    job->next = NULL;
    stage->pending_jobs--;
    return job;
}

void apple_virgl_cursor_stage_job_free(AppleVirglCursorStageJob *job)
{
    if (!job) {
        return;
    }
    g_free(job->pixels);
    g_free(job);
}

bool apple_virgl_cursor_stage_fill_qemu_cursor(
    const AppleVirglCursorStageJob *job, uint32_t *destination,
    size_t destination_pixels)
{
    size_t required_pixels;
    size_t index;

    if (!job || job->kind != APPLE_VIRGL_CURSOR_STAGE_GLYPH ||
        !job->pixels || job->width == 0 || job->height == 0 ||
        (size_t)job->width > SIZE_MAX / (size_t)job->height) {
        return false;
    }
    required_pixels = (size_t)job->width * (size_t)job->height;
    if (!destination || destination_pixels < required_pixels ||
        required_pixels > SIZE_MAX / 4u ||
        job->pixel_bytes != required_pixels * 4u) {
        return false;
    }
    for (index = 0; index < required_pixels; index++) {
        const uint8_t *source = job->pixels + index * 4u;

        destination[index] = ((uint32_t)source[0] << 16u) |
                             ((uint32_t)source[1] << 8u) |
                             ((uint32_t)source[2] << 0u) |
                             ((uint32_t)source[3] << 24u);
    }
    return true;
}

void apple_virgl_cursor_stage_drain(AppleVirglCursorStage *stage)
{
    AppleVirglCursorStageJob *job;

    if (!stage) {
        return;
    }
    while ((job = stage->head)) {
        stage->head = job->next;
        apple_virgl_cursor_stage_job_free(job);
    }
    stage->tail = NULL;
    stage->pending_jobs = 0;
}

void apple_virgl_cursor_stage_begin_reset(AppleVirglCursorStage *stage,
                                          bool shutdown)
{
    if (!stage) {
        return;
    }
    stage->resetting = true;
    stage->shutdown = stage->shutdown || shutdown;
    apple_virgl_cursor_stage_drain(stage);
}

void apple_virgl_cursor_stage_resume(AppleVirglCursorStage *stage)
{
    if (!stage || stage->shutdown) {
        return;
    }
    stage->resetting = false;
}

bool apple_virgl_cursor_stage_is_idle(const AppleVirglCursorStage *stage)
{
    return !stage || stage->pending_jobs == 0;
}
