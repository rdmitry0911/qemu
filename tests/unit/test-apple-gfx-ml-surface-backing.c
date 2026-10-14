/*
 * Real Console/owned pixman backing controls, using the normal product leaf.
 * No graphics Device realize, QMetal factory, guest, GPU or pixel-parity claim.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/memfd.h"
#include "system/replay.h"
#include "system/system.h"
#include "ui/console.h"
#include "hw/display/apple-gfx-ml.h"

typedef struct Allocation {
    void *data;
    size_t size;
    int fd;
    unsigned frees;
} Allocation;

typedef struct Fixture {
    AppleGfxMLState *state;
    DisplayChangeListener listener;
    unsigned switches;
    bool check_switch;
    pixman_image_t *retained;
    uint8_t old_byte;
} Fixture;

static Allocation allocations[256];
static unsigned allocation_count;
int (*qemu_main)(void);

void agfx_surface_baseline_publish_display_mode(AppleGfxMLState *s,
    uint32_t width, uint32_t height, uint32_t pf, uint64_t protection);
void *__real_qemu_memfd_alloc(const char *name, size_t size, unsigned int seals,
                              int *fd, Error **errp);
void __real_qemu_memfd_free(void *ptr, size_t size, int fd);
void *__wrap_qemu_memfd_alloc(const char *name, size_t size, unsigned int seals,
                              int *fd, Error **errp);
void __wrap_qemu_memfd_free(void *ptr, size_t size, int fd);

void *__wrap_qemu_memfd_alloc(const char *name, size_t size, unsigned int seals,
                              int *fd, Error **errp)
{
    void *data = __real_qemu_memfd_alloc(name, size, seals, fd, errp);

    if (data) {
        g_assert_cmpuint(allocation_count, <, ARRAY_SIZE(allocations));
        allocations[allocation_count++] = (Allocation) {
            .data = data, .size = size, .fd = *fd,
        };
    }
    return data;
}

static Allocation *live_allocation(void *data)
{
    for (unsigned i = allocation_count; i > 0; --i) {
        Allocation *a = &allocations[i - 1];

        if (a->data == data && a->frees == 0) {
            return a;
        }
    }
    g_assert_not_reached();
}

void __wrap_qemu_memfd_free(void *ptr, size_t size, int fd)
{
    Allocation *a = live_allocation(ptr);

    g_assert_cmpuint(a->size, ==, size);
    g_assert_cmpint(a->fd, ==, fd);
    ++a->frees;
    __real_qemu_memfd_free(ptr, size, fd);
}

static void observe_switch(DisplayChangeListener *listener,
                           DisplaySurface *surface)
{
    Fixture *f = container_of(listener, Fixture, listener);
    AppleGfxMLState *s = f->state;

    ++f->switches;
    if (!f->check_switch) {
        return;
    }
    g_assert_true(qemu_console_surface(s->con) == surface);
    g_assert_true(surface_is_allocated(surface));
    g_assert_false(surface_is_placeholder(surface));
    g_assert_true(s->display_fb == surface_data(surface));
    g_assert_cmpuint(s->fb_width, ==, surface_width(surface));
    g_assert_cmpuint(s->fb_height, ==, surface_height(surface));
    g_assert_cmpuint(s->fb_stride, ==, surface_stride(surface));
    g_assert_cmpuint(s->display_fb_size, ==,
                     (size_t)s->fb_height * s->fb_stride);
    g_assert_cmpuint(s->fb_iosurface_pixel_format, ==, 0x42475241u);
    g_assert_cmpuint(s->fb_protection_requirements, ==, 17);
    if (f->retained) {
        uint8_t *old_data = (uint8_t *)pixman_image_get_data(f->retained);

        g_assert_true(old_data != s->display_fb);
        g_assert_cmpuint(old_data[0], ==, f->old_byte);
        g_assert_cmpuint(live_allocation(old_data)->frees, ==, 0);
    }
}

static const DisplayChangeListenerOps listener_ops = {
    .dpy_name = "test-agfx-surface-backing",
    .dpy_gfx_switch = observe_switch,
};
static const GraphicHwOps console_ops = { };

static Fixture *fixture_new(bool console)
{
    Fixture *f = g_new0(Fixture, 1);

    f->state = APPLE_GFX_ML(object_new(TYPE_APPLE_GFX_ML));
    g_assert_false(DEVICE(f->state)->realized);
    g_assert_null(f->state->qmu_dev);
    if (console) {
        f->state->con = graphic_console_init(DEVICE(f->state), 0,
                                             &console_ops, f->state);
        f->listener.ops = &listener_ops;
        f->listener.con = f->state->con;
        register_displaychangelistener(&f->listener);
        f->switches = 0;
    }
    return f;
}

static void fixture_free(Fixture *f)
{
    if (f->state->con) {
        f->check_switch = false;
        unregister_displaychangelistener(&f->listener);
        graphic_console_close(f->state->con);
        f->state->con = NULL;
    }
    agfx_clear_display_surface_borrow(f->state);
    object_unref(OBJECT(f->state));
    g_free(f);
}

static void publish(Fixture *f, uint32_t width, uint32_t height)
{
    f->check_switch = true;
    agfx_publish_display_mode(f->state, width, height, 0x42475241u, 17);
}

static void test_preconsole_and_initial_same_mode(void)
{
    Fixture *f = fixture_new(false);
    AppleGfxMLState *s = f->state;
    unsigned before = allocation_count;

    agfx_publish_display_mode(NULL, 640, 480, 0, 0);
    agfx_publish_display_mode(s, 0, 480, 0, 0);
    agfx_publish_display_mode(s, 640, 480, 0x42475241u, 17);
    g_assert_cmpuint(allocation_count, ==, before);
    g_assert_null(s->display_fb);
    g_assert_cmpuint(s->display_fb_size, ==, 0);
    g_assert_cmpuint(s->fb_width, ==, 640);
    g_assert_cmpuint(s->fb_height, ==, 480);
    g_assert_cmpuint(s->fb_stride, ==, 2560);
    g_assert_cmpuint(s->fb_iosurface_pixel_format, ==, 0x42475241u);
    g_assert_cmpuint(s->fb_protection_requirements, ==, 17);
    fixture_free(f);

    f = fixture_new(true);
    s = f->state;
    DisplaySurface *placeholder = qemu_console_surface(s->con);
    uint32_t width = surface_width(placeholder);
    uint32_t height = surface_height(placeholder);
    g_assert_true(surface_is_placeholder(placeholder));
    /* Equal initialized metadata cannot make the placeholder our frame. */
    s->fb_width = width;
    s->fb_height = height;
    s->fb_stride = width * 4;
    before = allocation_count;
    publish(f, width, height);
    g_assert_cmpuint(allocation_count, ==, before + 1);
    g_assert_cmpuint(f->switches, ==, 1);
    DisplaySurface *owned = qemu_console_surface(s->con);
    uint8_t *data = s->display_fb;
    for (unsigned i = 0; i < 128; ++i) {
        agfx_publish_display_mode(s, width, height, i, (uint64_t)i << 32);
        g_assert_true(qemu_console_surface(s->con) == owned);
        g_assert_true(s->display_fb == data);
        g_assert_cmpuint(s->fb_iosurface_pixel_format, ==, i);
        g_assert_cmpuint(s->fb_protection_requirements, ==, (uint64_t)i << 32);
    }
    g_assert_cmpuint(allocation_count, ==, before + 1);
    g_assert_cmpuint(f->switches, ==, 1);
    fixture_free(f);
}

static void test_changed_geometry_retains_backing(void)
{
    const uint32_t modes[][2] = {{64, 32}, {128, 64}, {32, 16}, {16, 32}};
    Fixture *f = fixture_new(true);

    publish(f, modes[0][0], modes[0][1]);
    memset(f->state->display_fb, 0x51, f->state->display_fb_size);
    for (unsigned i = 1; i < ARRAY_SIZE(modes); ++i) {
        DisplaySurface *old = qemu_console_surface(f->state->con);
        pixman_image_t *deferred = pixman_image_ref(old->image);
        Allocation *a = live_allocation(surface_data(old));
        size_t old_size = a->size;
        uint8_t *old_data = a->data;

        f->retained = pixman_image_ref(old->image);
        f->old_byte = 0x51;
        publish(f, modes[i][0], modes[i][1]);
        g_assert_true(f->state->display_fb != old_data);
        memset(f->state->display_fb, 0xa2, f->state->display_fb_size);
        for (size_t j = 0; j < old_size; ++j) {
            g_assert_cmpuint(old_data[j], ==, 0x51);
        }
        g_assert_cmpuint(a->frees, ==, 0);
        pixman_image_unref(f->retained);
        f->retained = NULL;
        g_assert_cmpuint(a->frees, ==, 0);
        pixman_image_unref(deferred);
        g_assert_cmpuint(a->frees, ==, 1);
        errno = 0;
        g_assert_cmpint(fcntl(a->fd, F_GETFD), ==, -1);
        g_assert_cmpint(errno, ==, EBADF);
        memset(f->state->display_fb, 0x51, f->state->display_fb_size);
    }
    g_assert_cmpuint(f->switches, ==, ARRAY_SIZE(modes));
    fixture_free(f);
}

static void test_borrow_clear_and_console_close(void)
{
    Fixture *f = fixture_new(true);

    publish(f, 64, 32);
    DisplaySurface *surface = qemu_console_surface(f->state->con);
    pixman_image_t *retained = pixman_image_ref(surface->image);
    Allocation *a = live_allocation(f->state->display_fb);
    uint8_t *data = a->data;
    memset(data, 0x37, a->size);
    agfx_clear_display_surface_borrow(f->state);
    g_assert_null(f->state->display_fb);
    g_assert_cmpuint(f->state->display_fb_size, ==, 0);
    g_assert_cmpuint(a->frees, ==, 0);
    fixture_free(f);
    g_assert_cmpuint(a->frees, ==, 0);
    for (size_t j = 0; j < a->size; ++j) {
        g_assert_cmpuint(data[j], ==, 0x37);
    }
    pixman_image_unref(retained);
    g_assert_cmpuint(a->frees, ==, 1);
}

static void test_exact_baseline_safe_shrink_alias(void)
{
    Fixture *f = fixture_new(true);
    AppleGfxMLState *s = f->state;

    agfx_surface_baseline_publish_display_mode(s, 64, 32, 0x42475241u, 17);
    DisplaySurface *old = qemu_console_surface(s->con);
    g_assert_false(surface_is_allocated(old));
    pixman_image_t *retained = pixman_image_ref(old->image);
    uint8_t *old_data = (uint8_t *)pixman_image_get_data(retained);
    memset(old_data, 0x51, s->display_fb_size);
    agfx_surface_baseline_publish_display_mode(s, 32, 16, 0x42475241u, 17);
    g_assert_true(s->display_fb == old_data);
    g_assert_true(qemu_console_surface(s->con)->image != retained);
    memset(s->display_fb, 0xa2, (size_t)s->fb_height * s->fb_stride);
    g_assert_cmpuint(old_data[0], ==, 0xa2);
    g_test_message("Exact main9a shrink witness: retained old image aliases successor backing and its bytes change; no freed pointer read");
    pixman_image_unref(retained);
    /* External baseline storage is freed only after Console and retained image
     * refs no longer access it. Never run baseline growth with a retained ref. */
    uint8_t *baseline_data = s->display_fb;
    fixture_free(f);
    g_free(baseline_data);
}

static void test_same_mode_cost(void)
{
    Fixture *candidate = fixture_new(true);
    Fixture *baseline = fixture_new(true);
    const unsigned iterations = 100000;
    unsigned before;
    uint64_t old_ns[15], new_ns[15];

    publish(candidate, 1920, 1080);
    agfx_surface_baseline_publish_display_mode(baseline->state, 1920, 1080,
                                               0x42475241u, 17);
    before = allocation_count;
    candidate->check_switch = false;
    for (unsigned pair = 0; pair < ARRAY_SIZE(old_ns); ++pair) {
        for (unsigned order = 0; order < 2; ++order) {
            bool run_new = ((pair + order) & 1u) != 0;
            int64_t start = g_get_monotonic_time();

            if (run_new) {
                for (unsigned i = 0; i < iterations; ++i) {
                    agfx_publish_display_mode(candidate->state, 1920, 1080,
                                              0x42475241u, 17);
                }
                new_ns[pair] = (g_get_monotonic_time() - start) * 1000;
            } else {
                for (unsigned i = 0; i < iterations; ++i) {
                    agfx_surface_baseline_publish_display_mode(
                        baseline->state, 1920, 1080, 0x42475241u, 17);
                }
                old_ns[pair] = (g_get_monotonic_time() - start) * 1000;
            }
        }
        g_test_message("same-mode-cost pair=%u iterations=%u old_total_ns=%" PRIu64
                       " new_total_ns=%" PRIu64,
                       pair, iterations, old_ns[pair], new_ns[pair]);
    }
    g_assert_cmpuint(allocation_count, ==, before);
    g_assert_cmpuint(candidate->switches, ==, 1);
    g_assert_cmpuint(baseline->switches, ==, 1);
    g_test_message("Cost scope: CPU same-mode metadata leaf only; actual Console, no allocation/switch/copy/wait/shader/GPU regression measured");
    uint8_t *baseline_data = baseline->state->display_fb;
    fixture_free(baseline);
    g_free(baseline_data);
    fixture_free(candidate);
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
    g_test_add_func("/agfx/surface/preconsole-initial-same-mode",
                     test_preconsole_and_initial_same_mode);
    g_test_add_func("/agfx/surface/grow-shrink-equal-byte-retain",
                     test_changed_geometry_retains_backing);
    g_test_add_func("/agfx/surface/borrow-clear-console-close",
                     test_borrow_clear_and_console_close);
    g_test_add_func("/agfx/surface/exact-baseline-safe-shrink-alias",
                     test_exact_baseline_safe_shrink_alias);
    g_test_add_func("/agfx/surface/same-mode-cost", test_same_mode_cost);
    status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
