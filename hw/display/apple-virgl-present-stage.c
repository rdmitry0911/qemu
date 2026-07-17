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

/*
 * qemu_create_displaysurface() passes width * 4 and then height * stride
 * through signed-int arithmetic before handing the result to Pixman.
 */
static bool apple_virgl_present_stage_surface_geometry(
    uint32_t width, uint32_t height, uint32_t *out_packed_stride,
    uint64_t *out_packed_bytes)
{
    uint32_t packed_stride;

    if (width == 0 || height == 0 ||
        width > (uint32_t)INT_MAX / 4u) {
        return false;
    }
    packed_stride = width * 4u;
    if (height > (uint32_t)INT_MAX / packed_stride) {
        return false;
    }
    if (out_packed_stride) {
        *out_packed_stride = packed_stride;
    }
    if (out_packed_bytes) {
        *out_packed_bytes = (uint64_t)packed_stride * height;
    }
    return true;
}

static bool apple_virgl_present_stage_copy_payload(
    AppleVirglPresentStageJob *job, const void *pixels, uint32_t width,
    uint32_t height, uint32_t stride)
{
    const uint8_t *source = pixels;
    uint32_t packed_stride;
    uint64_t packed_bytes;
    uint64_t source_bytes;
    uint32_t row;

    if (!source || !apple_virgl_present_stage_surface_geometry(
                       width, height, &packed_stride, &packed_bytes) ||
        stride > INT_MAX || stride < packed_stride ||
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
    job->kind = APPLE_VIRGL_PRESENT_STAGE_FRAME_COMPLETION;
    job->frame_expected = frame_expected;
    if (frame_expected) {
        (void)apple_virgl_present_stage_copy_payload(job, pixels, width,
                                                      height, stride);
    }
    apple_virgl_present_stage_append(stage, job);
    return true;
}

bool apple_virgl_present_stage_enqueue_mode(
    AppleVirglPresentStage *stage, uint32_t width, uint32_t height,
    uint32_t iosurface_pixel_format, uint64_t protection_requirements)
{
    AppleVirglPresentStageJob *job;

    if (!stage || stage->resetting || stage->shutdown ||
        stage->pending_jobs == SIZE_MAX ||
        !apple_virgl_present_stage_surface_geometry(width, height, NULL,
                                                    NULL)) {
        return false;
    }

    job = g_new0(AppleVirglPresentStageJob, 1);
    job->kind = APPLE_VIRGL_PRESENT_STAGE_MODE_CHANGE;
    job->width = width;
    job->height = height;
    job->iosurface_pixel_format = iosurface_pixel_format;
    job->protection_requirements = protection_requirements;
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
