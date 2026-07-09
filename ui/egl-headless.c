#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "ui/console.h"
#include "ui/egl-helpers.h"
#include "ui/egl-context.h"
#include "ui/shader.h"

#include <inttypes.h>

typedef struct egl_dpy {
    DisplayChangeListener dcl;
    DisplaySurface *ds;
    QemuGLShader *gls;
    egl_fb guest_fb;
    egl_fb cursor_fb;
    egl_fb blit_fb;
    bool y_0_top;
    uint32_t pos_x;
    uint32_t pos_y;
} egl_dpy;

/* ------------------------------------------------------------------ */

static void apple_virgl_frame_dump(DisplaySurface *surface)
{
    static bool initialized;
    static bool enabled;
    static char dir[PATH_MAX];
    static uint64_t frame_index;
    static uint32_t last_hash;
    static FILE *manifest;
    uint8_t *data;
    uint32_t hash = 0x811c9dc5;
    int width;
    int height;
    int stride;
    int bpp;
    FILE *fp;

    if (!initialized) {
        const char *env = getenv("APPLE_VIRGL_FRAME_DIR");

        initialized = true;
        if (env && *env && snprintf(dir, sizeof(dir), "%s", env) < sizeof(dir) &&
            g_mkdir_with_parents(dir, 0755) == 0) {
            g_autofree char *manifest_path = g_build_filename(dir, "frames.tsv", NULL);

            enabled = true;
            manifest = fopen(manifest_path, "w");
            if (manifest) {
                fprintf(manifest,
                        "index\twidth\theight\tformat\tstride\tfnv1a\tpath\n");
                fflush(manifest);
            }
        }
    }

    if (!enabled || !surface) {
        return;
    }

    width = surface_width(surface);
    height = surface_height(surface);
    stride = surface_stride(surface);
    bpp = surface_bytes_per_pixel(surface);
    data = surface_data(surface);
    if (!data || width <= 0 || height <= 0 || bpp != 4) {
        return;
    }

    for (int y = 0; y < height; y += 4) {
        uint8_t *row = data + (size_t)y * stride;

        for (int x = 0; x < width; x += 4) {
            uint32_t px;

            memcpy(&px, row + (size_t)x * bpp, sizeof(px));
            hash ^= px;
            hash *= 0x01000193;
        }
    }
    if (hash == last_hash && frame_index > 0) {
        return;
    }
    last_hash = hash;
    frame_index++;

    g_autofree char *frame_name = g_strdup_printf("%06" PRIu64 ".bgra",
                                                  frame_index);
    g_autofree char *frame_path = g_build_filename(dir, frame_name, NULL);

    fp = fopen(frame_path, "wb");
    if (!fp) {
        return;
    }
    for (int y = 0; y < height; y++) {
        fwrite(data + (size_t)y * stride, 1, (size_t)width * bpp, fp);
    }
    fclose(fp);

    if (manifest) {
        fprintf(manifest, "%" PRIu64 "\t%d\t%d\t0x%x\t%d\t0x%08x\t%s\n",
                frame_index, width, height, surface_format(surface), stride,
                hash, frame_name);
        fflush(manifest);
    }
}

static void egl_refresh(DisplayChangeListener *dcl)
{
    graphic_hw_update(dcl->con);
}

static void egl_gfx_update(DisplayChangeListener *dcl,
                           int x, int y, int w, int h)
{
}

static void egl_gfx_switch(DisplayChangeListener *dcl,
                           struct DisplaySurface *new_surface)
{
    egl_dpy *edpy = container_of(dcl, egl_dpy, dcl);

    edpy->ds = new_surface;
}

static QEMUGLContext egl_create_context(DisplayGLCtx *dgc,
                                        QEMUGLParams *params)
{
    QEMUGLContext ctx;

    if (!eglMakeCurrent(qemu_egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE,
                        qemu_egl_rn_ctx)) {
        error_report("egl-headless: eglMakeCurrent rendernode failed: %s",
                     qemu_egl_get_error_string());
        return NULL;
    }

    ctx = qemu_egl_create_context(dgc, params);

    if (!eglMakeCurrent(qemu_egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE,
                        EGL_NO_CONTEXT)) {
        error_report("egl-headless: eglMakeCurrent no-context failed: %s",
                     qemu_egl_get_error_string());
    }

    return ctx;
}

static void egl_scanout_disable(DisplayChangeListener *dcl)
{
    egl_dpy *edpy = container_of(dcl, egl_dpy, dcl);

    egl_fb_destroy(&edpy->guest_fb);
    egl_fb_destroy(&edpy->blit_fb);
}

static void egl_scanout_texture(DisplayChangeListener *dcl,
                                uint32_t backing_id,
                                bool backing_y_0_top,
                                uint32_t backing_width,
                                uint32_t backing_height,
                                uint32_t x, uint32_t y,
                                uint32_t w, uint32_t h,
                                void *d3d_tex2d)
{
    egl_dpy *edpy = container_of(dcl, egl_dpy, dcl);

    edpy->y_0_top = backing_y_0_top;

    /* source framebuffer */
    egl_fb_setup_for_tex(&edpy->guest_fb,
                         backing_width, backing_height, backing_id, false);

    /* dest framebuffer */
    if (edpy->blit_fb.width  != backing_width ||
        edpy->blit_fb.height != backing_height) {
        egl_fb_destroy(&edpy->blit_fb);
        egl_fb_setup_new_tex(&edpy->blit_fb, backing_width, backing_height);
    }
}

#ifdef CONFIG_GBM

static void egl_scanout_dmabuf(DisplayChangeListener *dcl,
                               QemuDmaBuf *dmabuf)
{
    uint32_t width, height, texture;

    egl_dmabuf_import_texture(dmabuf);
    texture = qemu_dmabuf_get_texture(dmabuf);
    if (!texture) {
        return;
    }

    width = qemu_dmabuf_get_width(dmabuf);
    height = qemu_dmabuf_get_height(dmabuf);

    egl_scanout_texture(dcl, texture, false, width, height, 0, 0,
                        width, height, NULL);
}

static void egl_cursor_dmabuf(DisplayChangeListener *dcl,
                              QemuDmaBuf *dmabuf, bool have_hot,
                              uint32_t hot_x, uint32_t hot_y)
{
    uint32_t width, height, texture;
    egl_dpy *edpy = container_of(dcl, egl_dpy, dcl);

    if (dmabuf) {
        egl_dmabuf_import_texture(dmabuf);
        texture = qemu_dmabuf_get_texture(dmabuf);
        if (!texture) {
            return;
        }

        width = qemu_dmabuf_get_width(dmabuf);
        height = qemu_dmabuf_get_height(dmabuf);
        egl_fb_setup_for_tex(&edpy->cursor_fb, width, height, texture, false);
    } else {
        egl_fb_destroy(&edpy->cursor_fb);
    }
}

static void egl_release_dmabuf(DisplayChangeListener *dcl,
                               QemuDmaBuf *dmabuf)
{
    egl_dmabuf_release_texture(dmabuf);
}

#endif

static void egl_cursor_position(DisplayChangeListener *dcl,
                                uint32_t pos_x, uint32_t pos_y)
{
    egl_dpy *edpy = container_of(dcl, egl_dpy, dcl);

    edpy->pos_x = pos_x;
    edpy->pos_y = pos_y;
}

static void egl_scanout_flush(DisplayChangeListener *dcl,
                              uint32_t x, uint32_t y,
                              uint32_t w, uint32_t h)
{
    egl_dpy *edpy = container_of(dcl, egl_dpy, dcl);

    if (!edpy->guest_fb.texture || !edpy->ds) {
        return;
    }
    assert(surface_format(edpy->ds) == PIXMAN_x8r8g8b8);

    if (edpy->cursor_fb.texture) {
        /* have cursor -> render using textures */
        egl_texture_blit(edpy->gls, &edpy->blit_fb, &edpy->guest_fb,
                         !edpy->y_0_top);
        egl_texture_blend(edpy->gls, &edpy->blit_fb, &edpy->cursor_fb,
                          !edpy->y_0_top, edpy->pos_x, edpy->pos_y,
                          1.0, 1.0);
    } else {
        /* no cursor -> use simple framebuffer blit */
        egl_fb_blit(&edpy->blit_fb, &edpy->guest_fb, edpy->y_0_top);
    }

    egl_fb_read(edpy->ds, &edpy->blit_fb);
    apple_virgl_frame_dump(edpy->ds);
    dpy_gfx_update(edpy->dcl.con, x, y, w, h);
}

static const DisplayChangeListenerOps egl_ops = {
    .dpy_name                = "egl-headless",
    .dpy_refresh             = egl_refresh,
    .dpy_gfx_update          = egl_gfx_update,
    .dpy_gfx_switch          = egl_gfx_switch,

    .dpy_gl_scanout_disable  = egl_scanout_disable,
    .dpy_gl_scanout_texture  = egl_scanout_texture,
#ifdef CONFIG_GBM
    .dpy_gl_scanout_dmabuf   = egl_scanout_dmabuf,
    .dpy_gl_cursor_dmabuf    = egl_cursor_dmabuf,
    .dpy_gl_release_dmabuf   = egl_release_dmabuf,
#endif
    .dpy_gl_cursor_position  = egl_cursor_position,
    .dpy_gl_update           = egl_scanout_flush,
};

static bool
egl_is_compatible_dcl(DisplayGLCtx *dgc,
                      DisplayChangeListener *dcl)
{
    if (!dcl->ops->dpy_gl_update) {
        /*
         * egl-headless is compatible with all 2d listeners, as it blits the GL
         * updates on the 2d console surface.
         */
        return true;
    }

    return dcl->ops == &egl_ops;
}

static const DisplayGLCtxOps eglctx_ops = {
    .dpy_gl_ctx_is_compatible_dcl = egl_is_compatible_dcl,
    .dpy_gl_ctx_create       = egl_create_context,
    .dpy_gl_ctx_destroy      = qemu_egl_destroy_context,
    .dpy_gl_ctx_make_current = qemu_egl_make_context_current,
};

static void egl_headless_attach_console(QemuConsole *con)
{
    DisplayGLCtx *ctx;
    egl_dpy *edpy;

    if (!qemu_console_is_graphic(con) || console_has_gl(con)) {
        return;
    }

    if (!eglMakeCurrent(qemu_egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE,
                        qemu_egl_rn_ctx)) {
        error_report("egl-headless: eglMakeCurrent rendernode failed: %s",
                     qemu_egl_get_error_string());
        return;
    }

    edpy = g_new0(egl_dpy, 1);
    edpy->dcl.con = con;
    edpy->dcl.ops = &egl_ops;
    edpy->gls = qemu_gl_init_shader();

    if (!eglMakeCurrent(qemu_egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE,
                        EGL_NO_CONTEXT)) {
        error_report("egl-headless: eglMakeCurrent no-context failed: %s",
                     qemu_egl_get_error_string());
    }

    ctx = g_new0(DisplayGLCtx, 1);
    ctx->ops = &eglctx_ops;
    qemu_console_set_display_gl_ctx(con, ctx);
    register_displaychangelistener(&edpy->dcl);
}

static void early_egl_headless_init(DisplayOptions *opts)
{
    DisplayGLMode mode = DISPLAY_GL_MODE_ON;

    if (opts->has_gl) {
        mode = opts->gl;
    }

    egl_init(opts->u.egl_headless.rendernode, mode, &error_fatal);
}

static void egl_headless_init(DisplayState *ds, DisplayOptions *opts)
{
    QemuConsole *con;
    int idx;
    static bool hook_registered;

    for (idx = 0;; idx++) {
        con = qemu_console_lookup_by_index(idx);
        if (!con || !qemu_console_is_graphic(con)) {
            break;
        }

        egl_headless_attach_console(con);
    }

    if (!hook_registered) {
        graphic_console_add_init_hook(egl_headless_attach_console);
        hook_registered = true;
    }
}

static QemuDisplay qemu_display_egl = {
    .type       = DISPLAY_TYPE_EGL_HEADLESS,
    .early_init = early_egl_headless_init,
    .init       = egl_headless_init,
};

static void register_egl(void)
{
    qemu_display_register(&qemu_display_egl);
}

type_init(register_egl);

module_dep("ui-opengl");
