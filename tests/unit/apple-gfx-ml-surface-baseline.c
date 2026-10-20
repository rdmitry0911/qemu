/*
 * Fixture-only committed-source baseline. The complete function body below is
 * from 9a87360560aa43bae4aba992e3b486887deb99c4:hw/display/apple-gfx-ml.c,
 * with only its symbol/linkage changed. The controller checks exact body bytes.
 * No baseline branch is compiled into the normal device.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "ui/console.h"
#include "hw/display/apple-gfx-ml.h"

void agfx_surface_baseline_publish_display_mode(AppleGfxMLState *s,
    uint32_t width, uint32_t height, uint32_t pf, uint64_t protection);

/* The un-realized unit Device has no async log worker. This test-only log sink
 * is outside the preserved body and does not change allocation or ownership. */
static G_GNUC_PRINTF(2, 3)
void agfx_log(AppleGfxMLState *s, const char *fmt, ...)
{
    va_list args;
    char *text;

    va_start(args, fmt);
    text = g_strdup_vprintf(fmt, args);
    va_end(args);
    qemu_log("%s", text);
    g_free(text);
}

void agfx_surface_baseline_publish_display_mode(AppleGfxMLState *s,
                                      uint32_t width,
                                      uint32_t height,
                                      uint32_t iosurface_pixel_format,
                                      uint64_t protection_requirements)
{
    const uint32_t stride = width * 4u;
    const size_t fb_size = (size_t)height * stride;

    if (!s || width == 0 || height == 0) {
        return;
    }

    if (fb_size > s->display_fb_size) {
        s->display_fb = g_realloc(s->display_fb, fb_size);
        s->display_fb_size = fb_size;
        agfx_log(s, "[apple-gfx-ml] display_fb reallocated: %zu bytes\n", fb_size);
    }

    if (s->con &&
        (width != s->fb_width || height != s->fb_height || stride != s->fb_stride)) {
        DisplaySurface *surface = qemu_create_displaysurface_from(
            width, height, PIXMAN_x8r8g8b8, stride, s->display_fb);
        dpy_gfx_replace_surface(s->con, surface);
    }

    s->fb_width = width;
    s->fb_height = height;
    s->fb_stride = stride;
    s->fb_iosurface_pixel_format = iosurface_pixel_format;
    s->fb_protection_requirements = protection_requirements;
}
