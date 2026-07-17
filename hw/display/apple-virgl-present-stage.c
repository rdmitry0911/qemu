/*
 * AppleVirgl owner-frame presentation staging
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "hw/display/apple-virgl-present-stage.h"

#include <limits.h>

static bool apple_virgl_present_stage_copy_payload(
    AppleVirglPresentStageJob *job, const void *pixels, uint32_t width,
    uint32_t height, uint32_t stride)
{
    const uint8_t *source = pixels;
    uint32_t packed_stride;
    uint64_t packed_bytes;
    uint64_t source_bytes;
    uint32_t row;

    if (!source || width == 0 || height == 0 ||
        umul32_overflow(width, 4, &packed_stride) ||
        width > INT_MAX || height > INT_MAX || stride > INT_MAX ||
        packed_stride > INT_MAX || stride < packed_stride ||
        umul64_overflow(packed_stride, height, &packed_bytes) ||
        umul64_overflow(stride, height, &source_bytes) ||
        packed_bytes > SIZE_MAX || source_bytes > SIZE_MAX) {
        return false;
    }

    job->pixels = g_try_malloc((size_t)packed_bytes);
    if (!job->pixels) {
        return false;
    }
    for (row = 0; row < height; row++) {
        memcpy(job->pixels + (size_t)row * packed_stride,
               source + (size_t)row * stride, packed_stride);
    }
    job->width = width;
    job->height = height;
    job->stride = packed_stride;
    job->pixel_bytes = (size_t)packed_bytes;
    job->payload_valid = true;
    return true;
}

static void apple_virgl_present_stage_append(AppleVirglPresentStage *stage,
                                             AppleVirglPresentStageJob *job)
{
    if (stage->tail) {
        stage->tail->next = job;
    } else {
        stage->head = job;
    }
    stage->tail = job;
    stage->pending_jobs++;
}

void apple_virgl_present_stage_init(AppleVirglPresentStage *stage)
{
    if (!stage) {
        return;
    }
    *stage = (AppleVirglPresentStage) { 0 };
}

bool apple_virgl_present_stage_enqueue(AppleVirglPresentStage *stage,
                                       bool frame_expected,
                                       const void *pixels,
                                       uint32_t width,
                                       uint32_t height,
                                       uint32_t stride)
{
    AppleVirglPresentStageJob *job;

    if (!stage || stage->resetting || stage->shutdown ||
        stage->pending_jobs == SIZE_MAX) {
        return false;
    }

    job = g_new0(AppleVirglPresentStageJob, 1);
    job->frame_expected = frame_expected;
    if (frame_expected) {
        (void)apple_virgl_present_stage_copy_payload(job, pixels, width,
                                                      height, stride);
    }
    apple_virgl_present_stage_append(stage, job);
    return true;
}

AppleVirglPresentStageJob *
apple_virgl_present_stage_take(AppleVirglPresentStage *stage)
{
    AppleVirglPresentStageJob *job;

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

void apple_virgl_present_stage_job_free(AppleVirglPresentStageJob *job)
{
    if (!job) {
        return;
    }
    g_free(job->pixels);
    g_free(job);
}

void apple_virgl_present_stage_drain(AppleVirglPresentStage *stage)
{
    AppleVirglPresentStageJob *job;

    if (!stage) {
        return;
    }

    while ((job = stage->head)) {
        stage->head = job->next;
        apple_virgl_present_stage_job_free(job);
    }
    stage->tail = NULL;
    stage->pending_jobs = 0;
}

void apple_virgl_present_stage_begin_reset(AppleVirglPresentStage *stage,
                                           bool shutdown)
{
    if (!stage) {
        return;
    }

    stage->resetting = true;
    stage->shutdown = stage->shutdown || shutdown;
    apple_virgl_present_stage_drain(stage);
}

void apple_virgl_present_stage_resume(AppleVirglPresentStage *stage)
{
    if (!stage || stage->shutdown) {
        return;
    }
    stage->resetting = false;
}

bool apple_virgl_present_stage_is_idle(const AppleVirglPresentStage *stage)
{
    return !stage || stage->pending_jobs == 0;
}
