/*
 * Render-start mode controls on actual QEMU AppleGfxML DeviceState objects.
 * The production kickoff and apply path use these same mode record/match
 * leaves. The controls exercise only CPU mode metadata, not a physical render
 * target, QMetal source selection, guest identity, GPU command or pixels.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "system/replay.h"
#include "system/system.h"
#include "hw/display/apple-gfx-ml.h"

int (*qemu_main)(void);

static AppleGfxMLState *fixture_new(uint32_t width, uint32_t height)
{
    AppleGfxMLState *s = APPLE_GFX_ML(object_new(TYPE_APPLE_GFX_ML));

    g_assert_false(DEVICE(s)->realized);
    g_assert_null(s->qmu_dev);
    s->fb_width = width;
    s->fb_height = height;
    return s;
}

static void assert_snapshot(const AppleGfxMLState *s,
                            uint32_t width, uint32_t height)
{
    g_assert_cmpuint(s->rendering_frame_width, ==, width);
    g_assert_cmpuint(s->rendering_frame_height, ==, height);
}

static void test_initial_start_and_unchanged_chain(void)
{
    AppleGfxMLState *s = fixture_new(1920, 1080);

    agfx_record_render_start_mode(s);
    assert_snapshot(s, 1920, 1080);
    g_assert_true(agfx_render_start_mode_is_current(s));
    for (unsigned i = 0; i < 64; ++i) {
        agfx_record_render_start_mode(s);
        assert_snapshot(s, 1920, 1080);
        g_assert_true(agfx_render_start_mode_is_current(s));
    }
    object_unref(OBJECT(s));
}

static void test_resize_rejects_old_but_next_chain_accepts(void)
{
    AppleGfxMLState *s = fixture_new(1920, 1080);

    agfx_record_render_start_mode(s);
    s->fb_width = 1280;
    s->fb_height = 720;
    assert_snapshot(s, 1920, 1080);
    g_assert_false(agfx_render_start_mode_is_current(s));

    /* The next actual kickoff uses the same leaf whether initiated by a new
     * frame BH or by its predecessor's completion-chain BH. */
    agfx_record_render_start_mode(s);
    assert_snapshot(s, 1280, 720);
    g_assert_true(agfx_render_start_mode_is_current(s));
    object_unref(OBJECT(s));
}

static void test_width_and_height_guards_remain_distinct(void)
{
    AppleGfxMLState *s = fixture_new(1920, 1080);

    agfx_record_render_start_mode(s);
    s->fb_width = 1440;
    g_assert_false(agfx_render_start_mode_is_current(s));
    agfx_record_render_start_mode(s);
    g_assert_true(agfx_render_start_mode_is_current(s));
    s->fb_height = 900;
    g_assert_false(agfx_render_start_mode_is_current(s));
    agfx_record_render_start_mode(s);
    assert_snapshot(s, 1440, 900);
    g_assert_true(agfx_render_start_mode_is_current(s));
    object_unref(OBJECT(s));
}

static void test_repeated_mode_changes_never_keep_predecessor(void)
{
    static const uint32_t modes[][2] = {
        {1920, 1080}, {1280, 720}, {1440, 900}, {2560, 1440},
    };
    AppleGfxMLState *s = fixture_new(modes[0][0], modes[0][1]);

    agfx_record_render_start_mode(s);
    for (unsigned i = 1; i <= 128; ++i) {
        const unsigned next = i % ARRAY_SIZE(modes);

        s->fb_width = modes[next][0];
        s->fb_height = modes[next][1];
        g_assert_false(agfx_render_start_mode_is_current(s));
        agfx_record_render_start_mode(s);
        assert_snapshot(s, modes[next][0], modes[next][1]);
        g_assert_true(agfx_render_start_mode_is_current(s));
    }
    object_unref(OBJECT(s));
}

int main(int argc, char **argv)
{
    char options[][16] = {"-machine", "none", "-accel", "tcg", "-nodefaults",
                         "-display", "none", "-S"};
    char *qemu_argv[] = {argv[0], options[0], options[1], options[2], options[3],
                        options[4], options[5], options[6], options[7], NULL};
    int status;

    g_test_init(&argc, &argv, NULL);
    qemu_init(ARRAY_SIZE(qemu_argv) - 1, qemu_argv);
    g_test_add_func("/agfx/render-mode/initial-and-unchanged-chain",
                    test_initial_start_and_unchanged_chain);
    g_test_add_func("/agfx/render-mode/resize-old-drop-next-chain-accept",
                    test_resize_rejects_old_but_next_chain_accepts);
    g_test_add_func("/agfx/render-mode/width-height-distinct",
                    test_width_and_height_guards_remain_distinct);
    g_test_add_func("/agfx/render-mode/repeated-mode-change-chain",
                    test_repeated_mode_changes_never_keep_predecessor);
    status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
