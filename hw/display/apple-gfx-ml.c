/*
 * Apple Paravirtualized Graphics - QEMU PCI Device (Minimal)
 *
 * Copyright (c) 2024
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This device is intentionally minimal - following Apple's pattern where
 * the QEMU device only handles PCI registration and proxies everything
 * to the library (qmetal unified API).
 *
 * Compare with Apple's apple-gfx-pci.m which is ~200 lines.
 *
 * Fixed critical display issue - present_frame now uses BH for
 * thread-safe display updates from qmetal's pthread.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/aio.h"       /* for aio_bh_schedule_oneshot */
#include "qemu/aio-wait.h"
#include "qemu/thread.h"    /* QemuEvent for DMA BH synchronization */
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msi.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/resettable.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "ui/console.h"
#include "trace.h"

#include "apple-gfx-ml.h"
#include "qmu/qmetal_unified.h"

/* Forward declarations from qmu_vulkan.h (C++ header, can't include directly) */
struct qmu_vulkan_ctx;
int qmu_vk_request_display_frame(struct qmu_vulkan_ctx *ctx);

/* Log throttling: show first N events, then every Mth */
#define AGFX_LOG_INITIAL_COUNT  10
#define AGFX_LOG_INTERVAL       60

static G_GNUC_PRINTF(7, 0)
void agfx_qmu_log_callback(void *ctx,
                           qmu_log_level level,
                           qmu_log_category category,
                           const char *file,
                           int line,
                           const char *func,
                           const char *fmt,
                           va_list args)
{
    char msg[2048];
    const char *base = file;

    (void)ctx;

    if (!fmt) {
        return;
    }

    if (file) {
        const char *slash = strrchr(file, '/');
        if (slash && slash[1]) {
            base = slash + 1;
        }
    }

    vsnprintf(msg, sizeof(msg), fmt, args);
    qemu_log("[apple-gfx-ml][qmetal][%s][%s] %s:%d %s: %s\n",
             qmu_log_level_name(level),
             qmu_log_category_name(category),
             base ? base : "?",
             line,
             func ? func : "?",
             msg);
}

/* ============================================================
 * Memory Access Callbacks (QEMU → qmetal)
 * These match Apple's PGDeviceDescriptor callback interface
 * ============================================================ */

static void *qemu_map_gpa(void *ctx, uint64_t gpa, size_t size, int writable)
{
    RCU_READ_LOCK_GUARD();   /* address_space_translate requires BQL or RCU */
    MemoryRegion *mr = NULL;
    hwaddr xlat = 0;
    hwaddr xlat_len = size;

    mr = address_space_translate(&address_space_memory, gpa,
                                  &xlat, &xlat_len, writable,
                                  MEMTXATTRS_UNSPECIFIED);
    if (!mr || xlat_len < size) {
        return NULL;
    }

    if (!memory_access_is_direct(mr, writable, MEMTXATTRS_UNSPECIFIED)) {
        return NULL;
    }

    void *ptr = memory_region_get_ram_ptr(mr);
    if (!ptr) {
        return NULL;
    }

    memory_region_ref(mr);
    trace_apple_gfx_ml_map_gpa(gpa, size, ptr + xlat, writable);
    return ptr + xlat;
}

static void qemu_unmap_gpa(void *ctx, void *hva, size_t size, int dirty)
{
    trace_apple_gfx_ml_unmap_gpa(hva, size);
    ram_addr_t offset;
    MemoryRegion *mr = memory_region_from_host(hva, &offset);
    if (mr) {
        if (dirty) {
            memory_region_set_dirty(mr, offset, size);
        }
        memory_region_unref(mr);
    }
}

/* ============================================================
 * DMA via BH+QemuEvent (reference: apple-gfx.m:2135-2168)
 * "Performing DMA requires BQL, so do it in a BH"
 * Called from qmetal pthread → schedules BH on main loop → waits.
 * ============================================================ */

typedef struct AgfxDMAJob {
    uint64_t gpa;
    void *buf;
    size_t size;
    bool is_write;
    bool dirty;
    MemTxResult result;
    QemuEvent event;
} AgfxDMAJob;

static void agfx_do_dma(void *opaque)
{
    AgfxDMAJob *job = opaque;
    if (job->is_write) {
        job->result = address_space_write(&address_space_memory, job->gpa,
                                           MEMTXATTRS_UNSPECIFIED,
                                           job->buf, job->size);
        if (job->result == MEMTX_OK && job->dirty) {
            MemoryRegion *mr = NULL;
            hwaddr xlat = 0, xlat_len = job->size;
            RCU_READ_LOCK_GUARD();
            mr = address_space_translate(&address_space_memory, job->gpa,
                                          &xlat, &xlat_len, true,
                                          MEMTXATTRS_UNSPECIFIED);
            if (mr) {
                memory_region_set_dirty(mr, xlat, job->size);
            }
        }
    } else {
        job->result = address_space_read(&address_space_memory, job->gpa,
                                          MEMTXATTRS_UNSPECIFIED,
                                          job->buf, job->size);
    }
    qemu_event_set(&job->event);
}

static int qemu_read_memory(void *ctx, uint64_t gpa, void *buf, size_t size)
{
    AgfxDMAJob job = { .gpa = gpa, .buf = buf, .size = size,
                       .is_write = false };
    trace_apple_gfx_ml_dma_read(gpa, size);
    qemu_event_init(&job.event, false);
    aio_bh_schedule_oneshot(qemu_get_aio_context(), agfx_do_dma, &job);
    qemu_event_wait(&job.event);
    qemu_event_destroy(&job.event);
    return (job.result == MEMTX_OK) ? 0 : -1;
}

static int qemu_write_memory(void *ctx, uint64_t gpa, const void *buf,
                              size_t size)
{
    AgfxDMAJob job = { .gpa = gpa, .buf = (void *)buf, .size = size,
                       .is_write = true, .dirty = true };
    trace_apple_gfx_ml_dma_write(gpa, size);
    qemu_event_init(&job.event, false);
    aio_bh_schedule_oneshot(qemu_get_aio_context(), agfx_do_dma, &job);
    qemu_event_wait(&job.event);
    qemu_event_destroy(&job.event);
    if (job.result != MEMTX_OK) {
        trace_apple_gfx_ml_dma_write_failed(gpa, size, job.result);
    }
    return (job.result == MEMTX_OK) ? 0 : -1;
}

/* ============================================================
 * Thread-safe Interrupt Delivery (Apple style)
 * IRQ is raised from qmetal pthread, must be delivered via BH
 * ============================================================ */

typedef struct AppleGfxMLInterruptJob {
    PCIDevice *device;
    uint32_t vector;
} AppleGfxMLInterruptJob;

static void apple_gfx_ml_raise_interrupt_bh(void *opaque)
{
    AppleGfxMLInterruptJob *job = opaque;
    AppleGfxMLState *s = APPLE_GFX_ML(job->device);

    s->irq_count++;

    if (s->irq_count <= AGFX_LOG_INITIAL_COUNT || (s->irq_count % AGFX_LOG_INTERVAL) == 0) {
        qemu_log("[apple-gfx-ml] raise_irq #%lu: msi_enabled=%d\n",
                 (unsigned long)s->irq_count, msi_enabled(job->device));
    }

    if (msi_enabled(job->device)) {
        msi_notify(job->device, job->vector);
    }

    g_free(job);
}

static void qemu_raise_irq(void *ctx, uint32_t vector)
{
    AppleGfxMLState *s = ctx;
    AppleGfxMLInterruptJob *job;
    
    /* Schedule interrupt in QEMU main loop (thread-safe, Apple style) */
    job = g_malloc0(sizeof(*job));
    job->device = &s->parent_obj;
    job->vector = vector;
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            apple_gfx_ml_raise_interrupt_bh, job);
}

/* ============================================================
 * Thread-safe Display Updates (Apple style)
 *
 * present_frame is called from qmetal's display_refresh_thread.
 * QEMU display operations MUST run in main loop.
 *
 * Solution: Double-buffered framebuffer + BH scheduling.
 * This matches Apple's approach with newFrameEventHandler + BH.
 * ============================================================ */

static void apple_gfx_ml_present_frame_bh(void *opaque)
{
    AppleGfxMLState *s = opaque;
    uint32_t width, height, stride;
    size_t size;

    /* Reference render_frame_completed_bh:
     * balance pending_frames from qemu_new_frame_signal(). */
    __atomic_sub_fetch(&s->pending_frames, 1, __ATOMIC_SEQ_CST);

    /* Get pending frame parameters under lock */
    qemu_mutex_lock(&s->frame_mutex);
    if (!s->frame_pending) {
        qemu_mutex_unlock(&s->frame_mutex);
        return;
    }

    width = s->pending_width;
    height = s->pending_height;
    stride = s->pending_stride;
    size = (size_t)height * stride;
    s->frame_pending = false;

    /* Reallocate display buffer if frame size increased */
    if (size > s->display_fb_size) {
        s->display_fb = g_realloc(s->display_fb, size);
        s->display_fb_size = size;
        qemu_log("[apple-gfx-ml] display_fb reallocated: %zu bytes\n", size);
    }

    /* Copy from staging to display buffer */
    if (s->staging_fb && s->display_fb) {
        memcpy(s->display_fb, s->staging_fb, size);
    }
    qemu_mutex_unlock(&s->frame_mutex);

    /* Update frame counter and log */
    s->frame_count++;
    if (s->frame_count <= AGFX_LOG_INITIAL_COUNT || (s->frame_count % AGFX_LOG_INTERVAL) == 0) {
        qemu_log("[apple-gfx-ml] present_frame_bh #%lu: %ux%u stride=%u\n",
                 (unsigned long)s->frame_count, width, height, stride);
    }

    /* Update QEMU display - now safe, we're in main loop! */
    if (s->con && s->display_fb) {
        /* Only replace surface when dimensions change (reference: set_mode) */
        if (width != s->fb_width || height != s->fb_height
            || stride != s->fb_stride) {
            DisplaySurface *surface = qemu_create_displaysurface_from(
                width, height, PIXMAN_x8r8g8b8, stride, s->display_fb);
            dpy_gfx_replace_surface(s->con, surface);
        }
    }

    /* Update display state after surface check (comparison uses old values) */
    s->fb_width = width;
    s->fb_height = height;
    s->fb_stride = stride;

    /* Coordinate with gfx_update via reference two-flag model */
    if (s->gfx_update_requested) {
        s->gfx_update_requested = false;
        dpy_gfx_update_full(s->con);
        graphic_hw_update_done(s->con);
        s->new_frame_ready = false;
    } else {
        s->new_frame_ready = true;
    }

    /* Reference render_frame_completed_bh (apple-gfx.m:2000-2016):
     * Chain to next frame if pending_frames > 0. */
    if (__atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST) > 0 && s->qmu_dev) {
        qmu_vk_request_display_frame(qmu_session_get_vulkan(s->qmu_dev));
    }
}

static void qemu_present_frame(void *ctx, const void *pixels,
                                uint32_t width, uint32_t height, uint32_t stride)
{
    AppleGfxMLState *s = ctx;
    size_t size = (size_t)height * stride;

    /* Log from pthread (before scheduling BH) */
    uint64_t pc = qatomic_fetch_inc(&s->present_count) + 1;
    if (pc <= AGFX_LOG_INITIAL_COUNT || (pc % AGFX_LOG_INTERVAL) == 0) {
        qemu_log("[apple-gfx-ml] present_frame #%lu: %ux%u stride=%u (from pthread)\n",
                 (unsigned long)pc, width, height, stride);
    }
    
    /* Reallocate staging buffer if needed */
    qemu_mutex_lock(&s->frame_mutex);
    if (!s->staging_fb || s->staging_fb_size < size) {
        g_free(s->staging_fb);
        s->staging_fb = g_malloc(size);
        s->staging_fb_size = size;
    }
    
    /* Copy pixels to staging buffer */
    memcpy(s->staging_fb, pixels, size);
    
    /* Store frame parameters */
    s->pending_width = width;
    s->pending_height = height;
    s->pending_stride = stride;
    s->frame_pending = true;
    qemu_mutex_unlock(&s->frame_mutex);
    
    /* Schedule BH to update display in QEMU main loop */
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            apple_gfx_ml_present_frame_bh, s);
}

typedef struct AppleGfxMLCursorGlyphJob {
    AppleGfxMLState *state;
    uint8_t *pixels;
    uint64_t mapped_length;
    uint64_t stride;
    uint32_t width;
    uint32_t height;
    uint32_t hot_x;
    uint32_t hot_y;
    uint32_t sum;
} AppleGfxMLCursorGlyphJob;

typedef struct AppleGfxMLCursorShowJob {
    AppleGfxMLState *state;
    uint32_t display_id;
    bool visible;
} AppleGfxMLCursorShowJob;

static void apple_gfx_ml_update_cursor(AppleGfxMLState *s)
{
    if (!s->con) {
        return;
    }
    dpy_mouse_set(s->con, 0, 0, s->cursor_show);
}

static void apple_gfx_ml_cursor_glyph_bh(void *opaque)
{
    AppleGfxMLCursorGlyphJob *job = opaque;
    AppleGfxMLState *s = job->state;
    const uint8_t *src = job->pixels;
    size_t row_padding = 0;

    if (job->stride >= (uint64_t)job->width * 4u) {
        row_padding = (size_t)(job->stride - (uint64_t)job->width * 4u);
    }

    if (s->cursor) {
        cursor_unref(s->cursor);
        s->cursor = NULL;
    }

    s->cursor = cursor_alloc(job->width, job->height);
    s->cursor->hot_x = job->hot_x;
    s->cursor->hot_y = job->hot_y;

    for (uint32_t y = 0; y < job->height; ++y) {
        for (uint32_t x = 0; x < job->width; ++x) {
            uint32_t *dst = &s->cursor->data[(size_t)y * job->width + x];

            /*
             * Match reference apple-gfx.m set_cursor_glyph conversion:
             * source bytes are kept in guest bitmap order and converted to
             * QEMUCursor channel layout when published to the UI.
             */
            *dst = ((uint32_t)src[0] << 16u) |
                   ((uint32_t)src[1] << 8u) |
                   ((uint32_t)src[2] << 0u) |
                   ((uint32_t)src[3] << 24u);
            src += 4;
        }
        src += row_padding;
    }

    qemu_log("[apple-gfx-ml] cursor_glyph: %ux%u stride=%" PRIu64 " hot=%u,%u sum=0x%08x\n",
             job->width, job->height, job->stride, job->hot_x, job->hot_y,
             job->sum);

    if (s->con) {
        dpy_cursor_define(s->con, s->cursor);
        apple_gfx_ml_update_cursor(s);
    }

    g_free(job->pixels);
    g_free(job);
}

static void apple_gfx_ml_cursor_show_bh(void *opaque)
{
    AppleGfxMLCursorShowJob *job = opaque;
    AppleGfxMLState *s = job->state;

    s->cursor_show = job->visible;
    qemu_log("[apple-gfx-ml] cursor_show: display=%u visible=%d\n",
             job->display_id, job->visible ? 1 : 0);
    apple_gfx_ml_update_cursor(s);
    g_free(job);
}

static void qemu_cursor_glyph(void *ctx,
                              const void *pixels,
                              uint64_t mapped_length,
                              uint64_t stride,
                              uint32_t width,
                              uint32_t height,
                              uint32_t hot_x,
                              uint32_t hot_y,
                              uint32_t sum)
{
    AppleGfxMLState *s = ctx;
    AppleGfxMLCursorGlyphJob *job;

    if (!s || !pixels || mapped_length == 0) {
        return;
    }

    job = g_new0(AppleGfxMLCursorGlyphJob, 1);
    job->state = s;
    job->pixels = g_memdup2(pixels, mapped_length);
    job->mapped_length = mapped_length;
    job->stride = stride;
    job->width = width;
    job->height = height;
    job->hot_x = hot_x;
    job->hot_y = hot_y;
    job->sum = sum;

    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            apple_gfx_ml_cursor_glyph_bh, job);
}

static void qemu_cursor_show(void *ctx, uint32_t display_id, int visible)
{
    AppleGfxMLState *s = ctx;
    AppleGfxMLCursorShowJob *job;

    if (!s) {
        return;
    }

    job = g_new0(AppleGfxMLCursorShowJob, 1);
    job->state = s;
    job->display_id = display_id;
    job->visible = visible != 0;

    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            apple_gfx_ml_cursor_show_bh, job);
}

/* Reference newFrameEventHandler (apple-gfx.m:2694) schedules
 * new_frame_handler_bh (2667) onto the AIO/mainloop. Keep the pending_frames
 * accounting on that BH path rather than calling into qmetal synchronously
 * from the signal source. */
static void qemu_new_frame_signal_bh(void *opaque)
{
    AppleGfxMLState *s = opaque;
    int pending;

    if (!s || !s->qmu_dev) {
        return;
    }

    /* Reference throttle: pending_frames >= 2 → drop (apple-gfx.m:2672) */
    pending = __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST);
    if (pending >= 2) {
        return;
    }
    pending = __atomic_add_fetch(&s->pending_frames, 1, __ATOMIC_SEQ_CST);

    /* Reference: if pending > 1, another frame will chain from completion (2678) */
    if (pending > 1) {
        return;
    }

    /* First frame — request encode (reference: apple_gfx_render_new_frame) */
    qmu_vk_request_display_frame(qmu_session_get_vulkan(s->qmu_dev));
}

static void qemu_new_frame_signal(void *ctx)
{
    AppleGfxMLState *s = ctx;
    if (!s) {
        return;
    }

    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            qemu_new_frame_signal_bh, s);
}

/* Display refresh is handled by qmetal library's internal thread.
 * No timer needed in QEMU driver - we just receive present_frame callbacks.
 */

/* ============================================================
 * Mainloop-safe Memory Access (for display completion BH)
 *
 * Direct address_space_read/write — safe when BQL is held (BH context).
 * The regular qemu_read/write_memory use nested BHs + QemuEvent wait,
 * which deadlocks when called from a BH (can't wait for yourself).
 * ============================================================ */

static int qemu_read_memory_mainloop(void *ctx, uint64_t gpa, void *buf,
                                      size_t size)
{
    return address_space_read(&address_space_memory, gpa,
                               MEMTXATTRS_UNSPECIFIED, buf, size) == MEMTX_OK ? 0 : -1;
}

static int qemu_write_memory_mainloop(void *ctx, uint64_t gpa, const void *buf,
                                       size_t size)
{
    return address_space_write(&address_space_memory, gpa,
                                MEMTXATTRS_UNSPECIFIED, buf, size) == MEMTX_OK ? 0 : -1;
}

/* ============================================================
 * Display Completion BH Trampoline
 *
 * Matches reference GCD dispatch_async for presentSurface completion.
 * Schedules qmetal's completion function to run in QEMU main event loop.
 * ============================================================ */

typedef struct AgfxCompletionJob {
    void (*fn)(void *);
    void *ctx;
} AgfxCompletionJob;

static void agfx_display_completion_bh(void *opaque)
{
    AgfxCompletionJob *job = opaque;
    job->fn(job->ctx);
    g_free(job);
}

static void qemu_schedule_display_completion(void *ctx,
                                              void (*fn)(void *),
                                              void *comp_ctx)
{
    AgfxCompletionJob *job = g_malloc(sizeof(*job));
    job->fn = fn;
    job->ctx = comp_ctx;
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                             agfx_display_completion_bh, job);
}

/* 1:1 reference apple_gfx_mmio_map_surface_memory (apple-gfx-mmio.m:145-158).
 * Maps guest physical memory for IOSurface backing with memory_region_ref pinning. */
static void *qemu_iosfc_map_memory(void *ctx, uint64_t gpa, uint64_t len, int read_only)
{
    MemoryRegion *mr = NULL;
    hwaddr xlat = 0;
    hwaddr xlat_len = len;

    RCU_READ_LOCK_GUARD();
    mr = address_space_translate(&address_space_memory, gpa,
                                  &xlat, &xlat_len, !read_only,
                                  MEMTXATTRS_UNSPECIFIED);
    if (!mr || xlat_len < len) {
        return NULL;
    }
    if (!memory_access_is_direct(mr, !read_only, MEMTXATTRS_UNSPECIFIED)) {
        return NULL;
    }
    void *ptr = memory_region_get_ram_ptr(mr);
    if (!ptr) {
        return NULL;
    }
    memory_region_ref(mr);
    return ptr + xlat;
}

/* 1:1 reference apple_gfx_mmio_unmap_surface_memory (apple-gfx-mmio.m:160-177) */
static int qemu_iosfc_unmap_memory(void *ctx, void *hva, uint64_t len)
{
    MemoryRegion *mr;
    ram_addr_t offset = 0;

    RCU_READ_LOCK_GUARD();
    mr = memory_region_from_host(hva, &offset);
    if (!mr) {
        qemu_log("[apple-gfx-ml] iosfc_unmap: memory at %p not found\n", hva);
        return -1;
    }
    memory_region_unref(mr);
    return 0;
}

static int qemu_read_vram(void *ctx, uint64_t vram_offset, void *buf, size_t size)
{
    AppleGfxMLState *s = ctx;
    
    /* VRAM is host_vram memory region */
    void *vram = memory_region_get_ram_ptr(&s->host_vram);
    if (!vram) {
        return -1;
    }
    
    uint64_t vram_size = memory_region_size(&s->host_vram);
    if (vram_offset + size > vram_size) {
        return -1;
    }
    
    memcpy(buf, (uint8_t *)vram + vram_offset, size);
    return 0;
}

/* ============================================================
 * Async MMIO Operations (1:1 with reference apple-gfx.m pattern)
 *
 * Reference uses dispatch_async_f + AIO_WAIT_WHILE to release
 * BQL during qmetal processing. On Linux we use a persistent
 * worker thread + semaphore instead of GCD.
 * ============================================================ */

typedef struct AppleGfxMLMMIOJob {
    AppleGfxMLState *state;
    uint64_t offset;
    uint64_t value;
    unsigned size;
    void (*func)(struct AppleGfxMLMMIOJob *);
    bool completed;
} AppleGfxMLMMIOJob;

static void *agfx_mmio_worker_thread(void *opaque)
{
    AppleGfxMLState *s = opaque;
    while (!qatomic_read(&s->mmio_worker_stop)) {
        qemu_sem_wait(&s->mmio_sem);
        AppleGfxMLMMIOJob *job = qatomic_read(&s->pending_job);
        if (job && job->func) {
            job->func(job);
        }
    }
    return NULL;
}

static void agfx_do_read(AppleGfxMLMMIOJob *job)
{
    job->value = qmu_mmio_read(job->state->qmu_dev,
                               (uint32_t)job->offset, job->size);
    qatomic_set(&job->completed, true);
    aio_wait_kick();
}

static void agfx_do_write(AppleGfxMLMMIOJob *job)
{
    qmu_mmio_write(job->state->qmu_dev,
                   (uint32_t)job->offset, job->value, job->size);
    qatomic_set(&job->completed, true);
    aio_wait_kick();
}

/*
 * MMIO handlers use stack-allocated jobs. This is safe because
 * AIO_WAIT_WHILE blocks until the worker completes the job,
 * keeping the stack frame alive for the job's entire lifetime.
 */
static uint64_t agfx_mmio_read(void *opaque, hwaddr offset, unsigned size)
{
    AppleGfxMLState *s = opaque;
    AppleGfxMLMMIOJob job = {
        .state = s, .offset = offset, .size = size,
        .func = agfx_do_read, .completed = false,
    };
    qemu_mutex_lock(&s->mmio_job_mutex);
    qatomic_set(&s->pending_job, &job);
    qemu_sem_post(&s->mmio_sem);
    AIO_WAIT_WHILE(NULL, !qatomic_read(&job.completed));
    qemu_mutex_unlock(&s->mmio_job_mutex);
    trace_apple_gfx_ml_mmio_read(offset, job.value, size);
    return job.value;
}

static void agfx_mmio_write(void *opaque, hwaddr offset,
                             uint64_t val, unsigned size)
{
    AppleGfxMLState *s = opaque;
    AppleGfxMLMMIOJob job = {
        .state = s, .offset = offset, .value = val, .size = size,
        .func = agfx_do_write, .completed = false,
    };
    qemu_mutex_lock(&s->mmio_job_mutex);
    qatomic_set(&s->pending_job, &job);
    qemu_sem_post(&s->mmio_sem);
    AIO_WAIT_WHILE(NULL, !qatomic_read(&job.completed));
    qemu_mutex_unlock(&s->mmio_job_mutex);
    trace_apple_gfx_ml_mmio_write(offset, val, size);
}

static const MemoryRegionOps agfx_mmio_ops = {
    .read = agfx_mmio_read,
    .write = agfx_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* ============================================================
 * Display Operations
 * ============================================================ */

static void agfx_gfx_update(void *opaque)
{
    AppleGfxMLState *s = opaque;
    if (s->new_frame_ready) {
        /* Path 1: Frame ready — push to display, signal done */
        dpy_gfx_update_full(s->con);
        s->new_frame_ready = false;
        graphic_hw_update_done(s->con);
    } else if (s->frame_pending) {
        /* Path 2: Frame in-flight — defer, completion BH will signal */
        s->gfx_update_requested = true;
    } else {
        /* Path 3: Idle — signal done to keep polling alive */
        graphic_hw_update_done(s->con);
    }
}

static const GraphicHwOps agfx_gfx_ops = {
    .gfx_update = agfx_gfx_update,
    .gfx_update_async = true,
};

/* ============================================================
 * Long Operation Hooks (release BQL during shader compilation)
 *
 * qmetal calls long_op_begin before slow operations (AIR->SPIR-V
 * translation, vkCreateGraphicsPipelines) and long_op_end after.
 * mmio_mutex is released by qmu_session.cpp wrappers before
 * these are called, allowing other channels' KICKs to proceed.
 * We kick AIO in begin to let the QEMU main loop process
 * pending BHs (interrupts, disk I/O) during the long op.
 * ============================================================ */

static void agfx_long_op_begin(void *ctx)
{
    (void)ctx;
    aio_wait_kick();
}

static void agfx_long_op_end(void *ctx)
{
    (void)ctx;
}

/* ============================================================
 * Device Lifecycle
 * ============================================================ */

static void agfx_realize(PCIDevice *pci_dev, Error **errp)
{
    AppleGfxMLState *s = APPLE_GFX_ML(pci_dev);
    uint64_t vram_size = (uint64_t)s->vram_size_mb << 20;
    size_t initial_fb_size;
    
    qemu_log("[apple-gfx-ml] Realizing device: %ux%u, VRAM=%uMB\n",
             s->display_width, s->display_height, s->vram_size_mb);
    
    /* OptionROM is handled via inherited 'romfile' property from PCIDevice.
     * Just like Apple does in apple-gfx-pci.m:
     *   pci->romfile = apple_gfx_pci_option_rom_path;
     * 
     * Usage: -device apple-gfx-ml,romfile=/path/to/AppleParavirtEFI.rom
     */
    if (pci_dev->romfile && pci_dev->romfile[0]) {
        qemu_log("[apple-gfx-ml] OptionROM configured: %s\n", pci_dev->romfile);
    }
    
    /* Setup MSI - Apple style: no INTERRUPT_PIN, just msi_init */
    int msi_ret = msi_init(pci_dev, APPLE_GFX_ML_MSI_CAP_AUTO, 1, true, false, errp);
    if (msi_ret == 0) {
        s->msi_used = true;
        qemu_log("[apple-gfx-ml] msi_init OK, 1 vector\n");
    } else {
        qemu_log("[apple-gfx-ml] msi_init FAILED: %d\n", msi_ret);
    }
    
    /* Setup MMIO BAR (BAR0) - Apple only uses this one BAR */
    memory_region_init_io(&s->mmio, OBJECT(s), &agfx_mmio_ops, s,
                          "apple-gfx-mmio", APPLE_GFX_ML_MMIO_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
    
    /* Setup host VRAM (not a PCI BAR - internal storage) */
    memory_region_init_ram(&s->host_vram, OBJECT(s), "apple-gfx-vram",
                           vram_size, errp);
    if (*errp) {
        return;
    }
    
    /* Initialize frame mutex for thread-safe display updates */
    qemu_mutex_init(&s->frame_mutex);
    
    /* Allocate double-buffered framebuffers */
    initial_fb_size = (size_t)s->display_height * s->display_width * 4;
    s->staging_fb = g_malloc0(initial_fb_size);
    s->staging_fb_size = initial_fb_size;
    s->display_fb = g_malloc0(initial_fb_size);
    s->display_fb_size = initial_fb_size;
    
    /* Create qmetal device with callbacks */
    qmu_extended_callbacks qmu_callbacks = {
        .user_ctx = s,
        .map_gpa = qemu_map_gpa,
        .unmap_gpa = qemu_unmap_gpa,
        .read_memory = qemu_read_memory,
        .write_memory = qemu_write_memory,
        .raise_irq = qemu_raise_irq,
        .present_frame = qemu_present_frame,
        .cursor_glyph = qemu_cursor_glyph,
        .cursor_show = qemu_cursor_show,
        .read_vram = qemu_read_vram,
        /* IOSurface mapper — 1:1 reference PGIOSurfaceHostDevice.
         * For PCI: iosfc_raise_irq wires to same qemu_raise_irq (one IRQ line,
         * matches apple-gfx-pci.m:107 — single raiseInterrupt with vector param). */
        .iosfc_map_memory = qemu_iosfc_map_memory,
        .iosfc_unmap_memory = qemu_iosfc_unmap_memory,
        .iosfc_raise_irq = qemu_raise_irq,
        /* Async display completion (reference GCD dispatch_async model) */
        .schedule_display_completion = qemu_schedule_display_completion,
        .read_memory_mainloop = qemu_read_memory_mainloop,
        .write_memory_mainloop = qemu_write_memory_mainloop,
        /* Reference: newFrameEventHandler via signalCurrentFrame (apple-gfx.m:2694) */
        .new_frame_signal = qemu_new_frame_signal,
    };

    qmu_extended_config qmu_config = {
        .struct_size = sizeof(qmu_extended_config),
        .ram_base = 0,
        .ram_size = 0,  /* No restriction */
        .max_protocol_version = 6,
        .iosfc_caps = 0x03,
        .display_port_count = 1,
        .vram_size = vram_size,
        .direct_scanout = s->direct_scanout,
        .vsync_enabled = s->vsync_enabled,
        .spirv_cache_dir = s->spirv_cache_dir,
        .long_op_unlock = agfx_long_op_begin,
        .long_op_lock = agfx_long_op_end,
        .long_op_ctx = s,
        .using_iosurface_mapper = 0,  /* PCI variant (apple-gfx-pci.m) does NOT use IOSurface mapper */
    };

    /* Reference apple-gfx.m:45-49: exactly 3 hardcoded modes.
     * Mode 0 is the primary display resolution. */
    qmu_config.mode_data[0] = (s->display_height << 16) | s->display_width;
    qmu_config.mode_data[1] = (1080u << 16) | 1440u;
    qmu_config.mode_data[2] = (1024u << 16) | 1280u;
    qmu_config.mode_count = 3;

    s->qmu_dev = qmu_create_extended(&qmu_config, &qmu_callbacks);
    if (!s->qmu_dev) {
        error_setg(errp, "Failed to create qmetal device");
        return;
    }

    qmu_log_set_callback(agfx_qmu_log_callback, NULL);
    qmu_set_debug_level(s->qmu_dev, s->debug_level);

    /* Start async MMIO worker thread (replaces GCD dispatch_async_f) */
    qemu_sem_init(&s->mmio_sem, 0);
    qemu_mutex_init(&s->mmio_job_mutex);
    s->mmio_worker_stop = false;
    qemu_thread_create(&s->mmio_worker, "agfx-io",
                        agfx_mmio_worker_thread, s, QEMU_THREAD_JOINABLE);

    /* Create QEMU console */
    s->con = graphic_console_init(DEVICE(s), 0, &agfx_gfx_ops, s);
    
    /* Initialize display parameters */
    s->fb_width = s->display_width;
    s->fb_height = s->display_height;
    s->fb_stride = s->display_width * 4;
    
    /* Create initial display surface using display_fb */
    DisplaySurface *surface = qemu_create_displaysurface_from(
        s->fb_width, s->fb_height, PIXMAN_x8r8g8b8,
        s->fb_stride, s->display_fb);
    dpy_gfx_replace_surface(s->con, surface);
    
    qemu_log("[apple-gfx-ml] Device realized successfully\n");
}

static void agfx_exit(PCIDevice *pci_dev)
{
    AppleGfxMLState *s = APPLE_GFX_ML(pci_dev);

    /* Stop async MMIO worker thread */
    qatomic_set(&s->mmio_worker_stop, true);
    qemu_sem_post(&s->mmio_sem);  /* Wake to exit */
    qemu_thread_join(&s->mmio_worker);
    qemu_sem_destroy(&s->mmio_sem);
    qemu_mutex_destroy(&s->mmio_job_mutex);

    /* Destroy qmetal device (stops display thread) */
    if (s->qmu_dev) {
        qmu_destroy(s->qmu_dev);
        s->qmu_dev = NULL;
    }
    qmu_log_set_callback(NULL, NULL);

    /* Cleanup framebuffers and mutex */
    qemu_mutex_destroy(&s->frame_mutex);
    g_free(s->staging_fb);
    s->staging_fb = NULL;
    g_free(s->display_fb);
    s->display_fb = NULL;
    if (s->cursor) {
        cursor_unref(s->cursor);
        s->cursor = NULL;
    }
}

static void agfx_reset(Object *obj, ResetType type)
{
    AppleGfxMLState *s = APPLE_GFX_ML(obj);
    
    s->frame_count = 0;
    qatomic_set(&s->present_count, 0);
    s->irq_count = 0;
    s->display_enabled = false;
    s->new_frame_ready = false;
    s->gfx_update_requested = false;
    s->pending_frames = 0;
    s->frame_pending = false;
    s->cursor_show = true;
    
    /* qmetal handles its own reset via MMIO writes from guest */
}

/* ============================================================
 * QOM Registration
 * ============================================================ */

static const Property agfx_properties[] = {
    APPLE_GFX_ML_PROPS,
};

static void agfx_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    
    /* Match Apple exactly - only vendor, device, class */
    k->vendor_id = APPLE_GFX_ML_VENDOR_ID;
    k->device_id = APPLE_GFX_ML_DEVICE_ID;
    k->class_id = APPLE_GFX_ML_CLASS_ID;
    k->realize = agfx_realize;
    k->exit = agfx_exit;
    
    rc->phases.hold = agfx_reset;
    
    dc->desc = "Apple Paravirtualized Graphics (PVG)";
    dc->hotpluggable = false;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    
    device_class_set_props(dc, agfx_properties);
}

static void agfx_instance_init(Object *obj)
{
    AppleGfxMLState *s = APPLE_GFX_ML(obj);
    
    s->vram_size_mb = APPLE_GFX_ML_DEFAULT_VRAM_MB;
    s->display_width = 1920;
    s->display_height = 1080;
    s->debug_level = 0;
    s->vsync_enabled = true;
    
    /* Initialize frame state */
    s->frame_pending = false;
    s->new_frame_ready = false;
    s->gfx_update_requested = false;
    s->pending_frames = 0;
    s->staging_fb = NULL;
    s->display_fb = NULL;
    s->cursor = NULL;
    s->cursor_show = true;
}

static const TypeInfo agfx_type_info = {
    .name = TYPE_APPLE_GFX_ML,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(AppleGfxMLState),
    .instance_init = agfx_instance_init,
    .class_init = agfx_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void agfx_register_types(void)
{
    type_register_static(&agfx_type_info);
}

type_init(agfx_register_types)
