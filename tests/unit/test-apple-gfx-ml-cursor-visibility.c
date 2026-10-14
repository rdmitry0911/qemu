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

static const DisplayChangeListenerOps listener_ops = {
    .dpy_name = "test-agfx-cursor-visibility",
    .dpy_mouse_set = observe_cursor,
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
    f->updates = 0;
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
    status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
