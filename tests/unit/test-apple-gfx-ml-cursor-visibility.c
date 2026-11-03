/*
 * Actual QEMU cursor callback -> AioContext BH -> console listener controls.
 * No QMetal factory/session, guest disk, physical GPU or pixel verdict.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/aio.h"
#include "qemu/main-loop.h"
#include "system/replay.h"
#include "system/system.h"
#include "ui/console.h"
#include "hw/display/apple-gfx-ml.h"

typedef struct Fixture {
    AppleGfxMLState *state;
    DisplayChangeListener listener;
    unsigned updates;
    bool observed[256];
    unsigned defines;
    int defined_width;
    int defined_height;
    int defined_hot_x;
    int defined_hot_y;
    uint32_t defined[16];
} Fixture;

int (*qemu_main)(void);

static void observe_cursor(DisplayChangeListener *listener,
                            int x, int y, bool visible)
{
    Fixture *f = container_of(listener, Fixture, listener);

    g_assert_cmpuint(f->updates, <, ARRAY_SIZE(f->observed));
    f->observed[f->updates++] = visible;
    g_assert_cmpint(x, ==, 11);
    g_assert_cmpint(y, ==, -7);
}

static void observe_cursor_define(DisplayChangeListener *listener,
                                  QEMUCursor *cursor)
{
    Fixture *f = container_of(listener, Fixture, listener);
    size_t pixels = (size_t)cursor->width * cursor->height;

    g_assert_cmpuint(pixels, <=, ARRAY_SIZE(f->defined));
    f->defines++;
    f->defined_width = cursor->width;
    f->defined_height = cursor->height;
    f->defined_hot_x = cursor->hot_x;
    f->defined_hot_y = cursor->hot_y;
    memcpy(f->defined, cursor->data, pixels * sizeof(uint32_t));
}

static const DisplayChangeListenerOps listener_ops = {
    .dpy_name = "test-agfx-cursor-visibility",
    .dpy_mouse_set = observe_cursor,
    .dpy_cursor_define = observe_cursor_define,
};

static const GraphicHwOps console_ops = { };

static Fixture *fixture_new(void)
{
    Fixture *f = g_new0(Fixture, 1);
    AppleGfxMLState *s = APPLE_GFX_ML(object_new(TYPE_APPLE_GFX_ML));

    f->state = s;
    g_assert_false(DEVICE(s)->realized);
    g_assert_null(s->qmu_dev);
    s->cursor_x = 11;
    s->cursor_y = -7;
    s->con = graphic_console_init(DEVICE(s), 0, &console_ops, s);
    /* Seed the real console position before registration reports it. */
    dpy_mouse_set(s->con, s->cursor_x, s->cursor_y,
                  qatomic_read(&s->cursor_show));
    f->listener.ops = &listener_ops;
    f->listener.con = s->con;
    register_displaychangelistener(&f->listener);
    /* A reused console replays its previous cursor on registration. */
    f->updates = 0;
    f->defines = 0;
    return f;
}

static void pump_bhs(void)
{
    /* Nonblocking actual main AioContext dispatch; no timing sleep/guess. */
    aio_poll(qemu_get_aio_context(), false);
}

static void fixture_free(Fixture *f)
{
    unregister_displaychangelistener(&f->listener);
    graphic_console_close(f->state->con);
    f->state->con = NULL;
    object_unref(OBJECT(f->state));
    g_free(f);
}

static void test_callback_publishes_before_ui_bh(void)
{
    Fixture *f = fixture_new();

    apple_gfx_ml_cursor_show(f->state, 0, 0);
    g_assert_false(qatomic_read(&f->state->cursor_show));
    g_assert_cmpuint(f->updates, ==, 0);
    pump_bhs();
    g_assert_cmpuint(f->updates, ==, 1);
    g_assert_false(f->observed[0]);
    apple_gfx_ml_cursor_show(f->state, 0, 1);
    g_assert_true(qatomic_read(&f->state->cursor_show));
    g_assert_cmpuint(f->updates, ==, 1);
    pump_bhs();
    g_assert_cmpuint(f->updates, ==, 2);
    g_assert_true(f->observed[1]);
    fixture_free(f);
}

static void test_pending_updates_sample_latest_visibility(void)
{
    Fixture *f = fixture_new();

    apple_gfx_ml_cursor_show(f->state, 0, 0);
    apple_gfx_ml_cursor_show(f->state, 0, 1);
    g_assert_true(qatomic_read(&f->state->cursor_show));
    g_assert_cmpuint(f->updates, ==, 0);
    pump_bhs();
    g_assert_cmpuint(f->updates, ==, 2);
    g_assert_true(f->observed[0]);
    g_assert_true(f->observed[1]);
    fixture_free(f);
}

static void test_earlier_move_observes_later_hide(void)
{
    Fixture *f = fixture_new();

    apple_gfx_ml_cursor_move(f->state, 0);
    apple_gfx_ml_cursor_show(f->state, 0, 0);
    g_assert_false(qatomic_read(&f->state->cursor_show));
    pump_bhs();
    g_assert_cmpuint(f->updates, ==, 2);
    g_assert_false(f->observed[0]);
    g_assert_false(f->observed[1]);
    fixture_free(f);
}

static void test_repeated_updates_and_null_callbacks(void)
{
    Fixture *f = fixture_new();

    apple_gfx_ml_cursor_show(NULL, 0, 0);
    apple_gfx_ml_cursor_move(NULL, 0);
    pump_bhs();
    g_assert_cmpuint(f->updates, ==, 0);
    for (unsigned i = 0; i < 128; ++i) {
        const bool visible = (i & 1u) != 0;

        apple_gfx_ml_cursor_show(f->state, 0, visible);
        g_assert_cmpint(qatomic_read(&f->state->cursor_show), ==, visible);
        pump_bhs();
        g_assert_cmpuint(f->updates, ==, i + 1);
        g_assert_cmpint(f->observed[i], ==, visible);
    }
    fixture_free(f);
}

/*
 * Production glyph path: host callback -> owned copy -> BH conversion ->
 * dpy_cursor_define.  Reference copy_GuestBGRA_ToHostBGRA_Task reads row y at
 * y * stride into a packed width * 4 row; QMetal has already validated
 * (height - 1) * stride + width * 4 <= mapped_length, with no stride lower
 * bound.  Each source byte is distinct, so every row origin is observable.
 */
static uint32_t expected_pixel(const uint8_t *src)
{
    return ((uint32_t)src[0] << 16) | ((uint32_t)src[1] << 8) |
           (uint32_t)src[2] | ((uint32_t)src[3] << 24);
}

static void check_glyph(uint64_t stride, uint32_t width, uint32_t height,
                        uint64_t mapped_length)
{
    Fixture *f = fixture_new();
    uint8_t src[64];

    g_assert_cmpuint(mapped_length, <=, sizeof(src));
    for (unsigned i = 0; i < sizeof(src); ++i) {
        src[i] = (uint8_t)(0x11 + i * 7);
    }
    apple_gfx_ml_cursor_glyph(f->state, src, mapped_length, stride, width,
                              height, 1, 2, 0);
    g_assert_cmpuint(f->defines, ==, 0);
    pump_bhs();
    g_assert_cmpuint(f->defines, ==, 1);
    g_assert_cmpint(f->defined_width, ==, width);
    g_assert_cmpint(f->defined_height, ==, height);
    g_assert_cmpint(f->defined_hot_x, ==, 1);
    g_assert_cmpint(f->defined_hot_y, ==, 2);
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const uint8_t *px = src + y * stride + x * 4u;

            g_assert_cmphex(f->defined[y * width + x], ==, expected_pixel(px));
        }
    }
    fixture_free(f);
}

static void test_glyph_packed_rows(void)
{
    check_glyph(8, 2, 2, 16);
}

static void test_glyph_padded_rows(void)
{
    check_glyph(12, 2, 2, 20);
}

static void test_glyph_overlapping_rows(void)
{
    /* stride < width * 4: (2 - 1) * 4 + 2 * 4 = 12 bytes. */
    check_glyph(4, 2, 2, 12);
}

static void test_glyph_zero_stride_rows(void)
{
    /* Every row is the same 8 source bytes. */
    check_glyph(0, 2, 3, 8);
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
    g_test_add_func("/agfx/cursor-visibility/callback-before-ui-bh",
                    test_callback_publishes_before_ui_bh);
    g_test_add_func("/agfx/cursor-visibility/pending-samples-current",
                    test_pending_updates_sample_latest_visibility);
    g_test_add_func("/agfx/cursor-visibility/earlier-move-later-hide",
                    test_earlier_move_observes_later_hide);
    g_test_add_func("/agfx/cursor-visibility/repeated-and-null",
                    test_repeated_updates_and_null_callbacks);
    g_test_add_func("/agfx/cursor-glyph/packed-rows", test_glyph_packed_rows);
    g_test_add_func("/agfx/cursor-glyph/padded-rows", test_glyph_padded_rows);
    g_test_add_func("/agfx/cursor-glyph/overlapping-rows",
                    test_glyph_overlapping_rows);
    g_test_add_func("/agfx/cursor-glyph/zero-stride-rows",
                    test_glyph_zero_stride_rows);
    status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
