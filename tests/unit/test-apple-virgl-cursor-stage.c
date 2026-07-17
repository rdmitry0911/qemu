/*
 * AppleVirgl cursor delivery staging tests
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/display/apple-virgl-cursor-stage.h"

static AppleVirglCursorStageJob *take_job(AppleVirglCursorStage *stage)
{
    AppleVirglCursorStageJob *job = apple_virgl_cursor_stage_take(stage);

    g_assert_nonnull(job);
    return job;
}

static void test_glyph_row_packs_and_converts(void)
{
    AppleVirglCursorStage stage;
    AppleVirglCursorStageJob *job;
    const uint8_t source[] = {
        0x10, 0x20, 0x30, 0x40, 0x11, 0x21, 0x31, 0x41,
        0xaa, 0xaa, 0xaa, 0xaa,
        0x12, 0x22, 0x32, 0x42, 0x13, 0x23, 0x33, 0x43,
        0xbb, 0xbb, 0xbb, 0xbb,
    };
    const uint8_t packed[] = {
        0x10, 0x20, 0x30, 0x40, 0x11, 0x21, 0x31, 0x41,
        0x12, 0x22, 0x32, 0x42, 0x13, 0x23, 0x33, 0x43,
    };
    uint32_t qemu_pixels[4] = { 0 };

    apple_virgl_cursor_stage_init(&stage);
    g_assert_true(apple_virgl_cursor_stage_enqueue_glyph(
        &stage, source, sizeof(source), 12, 2, 2, 1, 0));
    job = take_job(&stage);
    g_assert_cmpint(job->kind, ==, APPLE_VIRGL_CURSOR_STAGE_GLYPH);
    g_assert_cmpuint(job->pixel_bytes, ==, sizeof(packed));
    g_assert_cmpmem(job->pixels, job->pixel_bytes, packed, sizeof(packed));
    g_assert_cmpuint(job->hot_x, ==, 1);
    g_assert_cmpuint(job->hot_y, ==, 0);
    g_assert_true(apple_virgl_cursor_stage_fill_qemu_cursor(
        job, qemu_pixels, G_N_ELEMENTS(qemu_pixels)));
    g_assert_cmphex(qemu_pixels[0], ==, 0x40102030u);
    g_assert_cmphex(qemu_pixels[1], ==, 0x41112131u);
    g_assert_cmphex(qemu_pixels[2], ==, 0x42122232u);
    g_assert_cmphex(qemu_pixels[3], ==, 0x43132333u);
    apple_virgl_cursor_stage_job_free(job);
    g_assert_true(apple_virgl_cursor_stage_is_idle(&stage));
}

static void test_glyph_owns_callback_pixels(void)
{
    AppleVirglCursorStage stage;
    AppleVirglCursorStageJob *job;
    uint8_t source[] = {
        0x10, 0x20, 0x30, 0x40, 0xaa, 0xaa, 0xaa, 0xaa,
        0x11, 0x21, 0x31, 0x41, 0xbb, 0xbb, 0xbb, 0xbb,
    };
    const uint8_t expected[] = {
        0x10, 0x20, 0x30, 0x40,
        0x11, 0x21, 0x31, 0x41,
    };

    apple_virgl_cursor_stage_init(&stage);
    g_assert_true(apple_virgl_cursor_stage_enqueue_glyph(
        &stage, source, sizeof(source), 8, 1, 2, 0, 1));
    memset(source, 0xff, sizeof(source));
    job = take_job(&stage);
    g_assert_cmpmem(job->pixels, job->pixel_bytes, expected, sizeof(expected));
    apple_virgl_cursor_stage_job_free(job);
}

static void test_mixed_events_keep_fifo_order(void)
{
    AppleVirglCursorStage stage;
    AppleVirglCursorStageJob *job;
    const uint8_t pixels[] = { 0x10, 0x20, 0x30, 0x40 };

    apple_virgl_cursor_stage_init(&stage);
    g_assert_true(apple_virgl_cursor_stage_enqueue_show(&stage, 0, false));
    g_assert_true(apple_virgl_cursor_stage_enqueue_glyph(
        &stage, pixels, sizeof(pixels), 4, 1, 1, 0, 0));
    g_assert_true(apple_virgl_cursor_stage_enqueue_move(&stage, 0));
    g_assert_cmpuint(stage.pending_jobs, ==, 3);

    job = take_job(&stage);
    g_assert_cmpint(job->kind, ==, APPLE_VIRGL_CURSOR_STAGE_SHOW);
    g_assert_cmpuint(job->display_id, ==, 0);
    g_assert_false(job->visible);
    apple_virgl_cursor_stage_job_free(job);

    job = take_job(&stage);
    g_assert_cmpint(job->kind, ==, APPLE_VIRGL_CURSOR_STAGE_GLYPH);
    g_assert_cmpmem(job->pixels, job->pixel_bytes, pixels, sizeof(pixels));
    apple_virgl_cursor_stage_job_free(job);

    job = take_job(&stage);
    g_assert_cmpint(job->kind, ==, APPLE_VIRGL_CURSOR_STAGE_MOVE);
    g_assert_cmpuint(job->display_id, ==, 0);
    apple_virgl_cursor_stage_job_free(job);
    g_assert_true(apple_virgl_cursor_stage_is_idle(&stage));
}

static void test_malformed_glyph_rejects_without_event(void)
{
    AppleVirglCursorStage stage;
    const uint8_t pixels[] = { 0x10, 0x20, 0x30, 0x40 };

    apple_virgl_cursor_stage_init(&stage);
    g_assert_false(apple_virgl_cursor_stage_enqueue_glyph(
        &stage, NULL, 4, 4, 1, 1, 0, 0));
    g_assert_false(apple_virgl_cursor_stage_enqueue_glyph(
        &stage, pixels, sizeof(pixels), 3, 1, 1, 0, 0));
    g_assert_false(apple_virgl_cursor_stage_enqueue_glyph(
        &stage, pixels, sizeof(pixels), 4, 2, 1, 0, 0));
    g_assert_false(apple_virgl_cursor_stage_enqueue_glyph(
        &stage, pixels, sizeof(pixels), 4, 513, 1, 0, 0));
    g_assert_false(apple_virgl_cursor_stage_enqueue_glyph(
        &stage, pixels, sizeof(pixels), 4, 1, 1, UINT32_MAX, 0));
    g_assert_cmpuint(stage.pending_jobs, ==, 0);
    g_assert_true(apple_virgl_cursor_stage_is_idle(&stage));
}

static void test_reset_shutdown_reject_drain_and_resume(void)
{
    AppleVirglCursorStage stage;
    AppleVirglCursorStageJob *job;
    const uint8_t pixels[] = { 0x10, 0x20, 0x30, 0x40 };

    apple_virgl_cursor_stage_init(&stage);
    g_assert_true(apple_virgl_cursor_stage_enqueue_glyph(
        &stage, pixels, sizeof(pixels), 4, 1, 1, 0, 0));
    apple_virgl_cursor_stage_begin_reset(&stage, false);
    g_assert_true(stage.resetting);
    g_assert_false(stage.shutdown);
    g_assert_true(apple_virgl_cursor_stage_is_idle(&stage));
    g_assert_false(apple_virgl_cursor_stage_enqueue_move(&stage, 0));

    apple_virgl_cursor_stage_resume(&stage);
    g_assert_true(apple_virgl_cursor_stage_enqueue_show(&stage, 0, true));
    job = take_job(&stage);
    apple_virgl_cursor_stage_job_free(job);

    g_assert_true(apple_virgl_cursor_stage_enqueue_glyph(
        &stage, pixels, sizeof(pixels), 4, 1, 1, 0, 0));
    apple_virgl_cursor_stage_begin_reset(&stage, true);
    g_assert_true(stage.shutdown);
    apple_virgl_cursor_stage_resume(&stage);
    g_assert_true(stage.resetting);
    g_assert_false(apple_virgl_cursor_stage_enqueue_show(&stage, 0, false));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/apple-virgl/cursor-stage/glyph-row-pack-convert",
                    test_glyph_row_packs_and_converts);
    g_test_add_func("/apple-virgl/cursor-stage/owns-callback-pixels",
                    test_glyph_owns_callback_pixels);
    g_test_add_func("/apple-virgl/cursor-stage/fifo-mixed-events",
                    test_mixed_events_keep_fifo_order);
    g_test_add_func("/apple-virgl/cursor-stage/malformed-rejects",
                    test_malformed_glyph_rejects_without_event);
    g_test_add_func("/apple-virgl/cursor-stage/reset-shutdown-lifecycle",
                    test_reset_shutdown_reject_drain_and_resume);
    return g_test_run();
}
