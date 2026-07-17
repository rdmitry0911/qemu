/*
 * AppleVirgl owner-frame presentation staging tests
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/display/apple-virgl-present-stage.h"

static AppleVirglPresentStageJob *take_job(AppleVirglPresentStage *stage)
{
    AppleVirglPresentStageJob *job =
        apple_virgl_present_stage_take(stage);

    g_assert_nonnull(job);
    return job;
}

static void test_row_packs_bgra_without_padding(void)
{
    AppleVirglPresentStage stage;
    AppleVirglPresentStageJob *job;
    const uint8_t source[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0xaa, 0xaa, 0xaa, 0xaa,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0xbb, 0xbb, 0xbb, 0xbb,
    };
    const uint8_t expected[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    };

    apple_virgl_present_stage_init(&stage);
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, source,
                                                     2, 2, 12));
    job = take_job(&stage);
    g_assert_cmpint(job->kind, ==,
                    APPLE_VIRGL_PRESENT_STAGE_FRAME_COMPLETION);
    g_assert_true(job->frame_expected);
    g_assert_true(job->payload_valid);
    g_assert_cmpuint(job->width, ==, 2);
    g_assert_cmpuint(job->height, ==, 2);
    g_assert_cmpuint(job->stride, ==, 8);
    g_assert_cmpuint(job->pixel_bytes, ==, sizeof(expected));
    g_assert_cmpmem(job->pixels, job->pixel_bytes, expected, sizeof(expected));
    apple_virgl_present_stage_job_free(job);
    g_assert_true(apple_virgl_present_stage_is_idle(&stage));
}

static void test_mode_events_keep_fifo_order_and_metadata(void)
{
    AppleVirglPresentStage stage;
    AppleVirglPresentStageJob *job;
    const uint8_t pixels[] = { 0x10, 0x11, 0x12, 0x13 };

    apple_virgl_present_stage_init(&stage);
    g_assert_true(apple_virgl_present_stage_enqueue_mode(
        &stage, 1920, 1080, 0x42475241u, UINT64_C(0x1122334455667788)));
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, pixels,
                                                     1, 1, 4));

    job = take_job(&stage);
    g_assert_cmpint(job->kind, ==, APPLE_VIRGL_PRESENT_STAGE_MODE_CHANGE);
    g_assert_false(job->frame_expected);
    g_assert_false(job->payload_valid);
    g_assert_cmpuint(job->width, ==, 1920);
    g_assert_cmpuint(job->height, ==, 1080);
    g_assert_cmpuint(job->iosurface_pixel_format, ==, 0x42475241u);
    g_assert_cmpuint(job->protection_requirements, ==,
                     UINT64_C(0x1122334455667788));
    apple_virgl_present_stage_job_free(job);

    job = take_job(&stage);
    g_assert_cmpint(job->kind, ==,
                    APPLE_VIRGL_PRESENT_STAGE_FRAME_COMPLETION);
    g_assert_true(job->payload_valid);
    apple_virgl_present_stage_job_free(job);
    g_assert_true(apple_virgl_present_stage_is_idle(&stage));
}

static void test_invalid_mode_dimensions_are_rejected(void)
{
    AppleVirglPresentStage stage;

    apple_virgl_present_stage_init(&stage);
    g_assert_false(apple_virgl_present_stage_enqueue_mode(&stage, 0, 1,
                                                           0, 0));
    g_assert_false(apple_virgl_present_stage_enqueue_mode(&stage, 1, 0,
                                                           0, 0));
    g_assert_false(apple_virgl_present_stage_enqueue_mode(&stage,
                                                           UINT32_MAX, 1,
                                                           0, 0));
    g_assert_false(apple_virgl_present_stage_enqueue_mode(
        &stage, (uint32_t)INT_MAX / 4u + 1u, 1, 0, 0));
    g_assert_false(apple_virgl_present_stage_enqueue_mode(
        &stage, 1, (uint32_t)INT_MAX / 4u + 1u, 0, 0));
    g_assert_false(apple_virgl_present_stage_enqueue_mode(&stage, 1,
                                                           UINT32_MAX, 0, 0));
    g_assert_true(apple_virgl_present_stage_is_idle(&stage));
}

static void test_enqueue_owns_pixel_copy(void)
{
    AppleVirglPresentStage stage;
    AppleVirglPresentStageJob *job;
    uint8_t source[] = {
        0x00, 0x01, 0x02, 0x03, 0xaa, 0xaa, 0xaa, 0xaa,
        0x10, 0x11, 0x12, 0x13, 0xbb, 0xbb, 0xbb, 0xbb,
    };
    const uint8_t expected[] = {
        0x00, 0x01, 0x02, 0x03,
        0x10, 0x11, 0x12, 0x13,
    };

    apple_virgl_present_stage_init(&stage);
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, source,
                                                     1, 2, 8));
    memset(source, 0xff, sizeof(source));

    job = take_job(&stage);
    g_assert_true(job->payload_valid);
    g_assert_cmpuint(job->stride, ==, 4);
    g_assert_cmpmem(job->pixels, job->pixel_bytes, expected, sizeof(expected));
    apple_virgl_present_stage_job_free(job);
}

static void test_fifo_keeps_terminal_tokens(void)
{
    AppleVirglPresentStage stage;
    AppleVirglPresentStageJob *job;
    const uint8_t pixels[] = { 0x10, 0x11, 0x12, 0x13 };

    apple_virgl_present_stage_init(&stage);
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, pixels,
                                                     1, 1, 4));
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, false, NULL,
                                                     0, 0, 0));
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, NULL,
                                                     1, 1, 4));
    g_assert_cmpuint(stage.pending_jobs, ==, 3);

    job = take_job(&stage);
    g_assert_true(job->frame_expected);
    g_assert_true(job->payload_valid);
    g_assert_cmpmem(job->pixels, job->pixel_bytes, pixels, sizeof(pixels));
    apple_virgl_present_stage_job_free(job);

    job = take_job(&stage);
    g_assert_false(job->frame_expected);
    g_assert_false(job->payload_valid);
    g_assert_null(job->pixels);
    apple_virgl_present_stage_job_free(job);

    job = take_job(&stage);
    g_assert_true(job->frame_expected);
    g_assert_false(job->payload_valid);
    g_assert_null(job->pixels);
    apple_virgl_present_stage_job_free(job);
    g_assert_true(apple_virgl_present_stage_is_idle(&stage));
}

static void test_malformed_dimensions_make_terminal_jobs(void)
{
    AppleVirglPresentStage stage;
    AppleVirglPresentStageJob *job;
    const uint8_t pixels[] = { 0x10, 0x11, 0x12, 0x13 };
    unsigned int index;

    apple_virgl_present_stage_init(&stage);
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, NULL,
                                                     1, 1, 4));
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, pixels,
                                                     0, 1, 4));
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, pixels,
                                                     1, 1, 3));
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, pixels,
                                                     UINT32_MAX, 1,
                                                     UINT32_MAX));
    g_assert_true(apple_virgl_present_stage_enqueue(
        &stage, true, pixels, 1, (uint32_t)INT_MAX / 4u + 1u, 4));

    for (index = 0; index < 5; index++) {
        job = take_job(&stage);
        g_assert_true(job->frame_expected);
        g_assert_false(job->payload_valid);
        g_assert_null(job->pixels);
        g_assert_cmpuint(job->pixel_bytes, ==, 0);
        apple_virgl_present_stage_job_free(job);
    }
}

static void test_reset_shutdown_reject_drain_and_resume(void)
{
    AppleVirglPresentStage stage;
    AppleVirglPresentStageJob *job;
    const uint8_t pixels[] = { 0x10, 0x11, 0x12, 0x13 };

    apple_virgl_present_stage_init(&stage);
    g_assert_true(apple_virgl_present_stage_enqueue_mode(&stage, 2, 2,
                                                          0x42475241u, 0));
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, pixels,
                                                     1, 1, 4));
    apple_virgl_present_stage_begin_reset(&stage, false);
    g_assert_true(stage.resetting);
    g_assert_false(stage.shutdown);
    g_assert_true(apple_virgl_present_stage_is_idle(&stage));
    g_assert_null(apple_virgl_present_stage_take(&stage));
    g_assert_false(apple_virgl_present_stage_enqueue(&stage, true, pixels,
                                                      1, 1, 4));

    apple_virgl_present_stage_resume(&stage);
    g_assert_false(stage.resetting);
    g_assert_true(apple_virgl_present_stage_enqueue_mode(&stage, 2, 2,
                                                          0x42475241u, 0));
    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, pixels,
                                                     1, 1, 4));
    job = take_job(&stage);
    g_assert_cmpint(job->kind, ==, APPLE_VIRGL_PRESENT_STAGE_MODE_CHANGE);
    apple_virgl_present_stage_job_free(job);
    job = take_job(&stage);
    apple_virgl_present_stage_job_free(job);

    g_assert_true(apple_virgl_present_stage_enqueue(&stage, true, pixels,
                                                     1, 1, 4));
    apple_virgl_present_stage_begin_reset(&stage, true);
    g_assert_true(stage.shutdown);
    g_assert_true(apple_virgl_present_stage_is_idle(&stage));
    apple_virgl_present_stage_resume(&stage);
    g_assert_true(stage.resetting);
    g_assert_false(apple_virgl_present_stage_enqueue(&stage, true, pixels,
                                                      1, 1, 4));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/apple-virgl/present-stage/row-pack-bgra",
                    test_row_packs_bgra_without_padding);
    g_test_add_func("/apple-virgl/present-stage/owns-pixel-copy",
                    test_enqueue_owns_pixel_copy);
    g_test_add_func("/apple-virgl/present-stage/mode-fifo-metadata",
                    test_mode_events_keep_fifo_order_and_metadata);
    g_test_add_func("/apple-virgl/present-stage/invalid-mode-dimensions",
                    test_invalid_mode_dimensions_are_rejected);
    g_test_add_func("/apple-virgl/present-stage/fifo-terminal-tokens",
                    test_fifo_keeps_terminal_tokens);
    g_test_add_func("/apple-virgl/present-stage/malformed-terminal-tokens",
                    test_malformed_dimensions_make_terminal_jobs);
    g_test_add_func("/apple-virgl/present-stage/reset-shutdown-lifecycle",
                    test_reset_shutdown_reject_drain_and_resume);
    return g_test_run();
}
