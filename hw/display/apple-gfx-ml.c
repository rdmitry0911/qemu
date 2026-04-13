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
#include "qmu/pvg_regs.h"
#include "qmu/qmetal_unified.h"

/* Forward declarations from qmu_vulkan.h (C++ header, can't include directly) */
struct qmu_vulkan_ctx;
int qmu_vk_begin_display_frame(struct qmu_vulkan_ctx *ctx);
int qmu_vk_request_display_frame(struct qmu_vulkan_ctx *ctx);
void qmu_vk_consume_current_frame_signal(struct qmu_vulkan_ctx *ctx);
typedef struct AppleGfxMLSessionJob AppleGfxMLSessionJob;
typedef struct AgfxLogEntry AgfxLogEntry;
typedef struct AppleGfxMLFrameCompletionJob AppleGfxMLFrameCompletionJob;
typedef struct AppleGfxMLFramePayload AppleGfxMLFramePayload;
typedef struct AppleGfxMLDisplayCallbackJob AppleGfxMLDisplayCallbackJob;

/* Log throttling: show first N events, then every Mth */
#define AGFX_LOG_INITIAL_COUNT  10
#define AGFX_LOG_INTERVAL       60
#define AGFX_LOG_QUEUE_LIMIT    8192
#define AGFX_DISPLAY_FRAME_INTERVAL_MS  100  /* ~10Hz, matches reference */

struct AgfxLogEntry {
    AgfxLogEntry *next;
    char *text;
};

struct AppleGfxMLFrameCompletionJob {
    AppleGfxMLState *state;
    AppleGfxMLFrameCompletionJob *next;
    bool frame_expected;
    bool frame_staged;
    bool pending_accounted;
    bool chain_needed;
    uint8_t *frame_pixels;
    size_t frame_size;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_stride;
};

struct AppleGfxMLFramePayload {
    AppleGfxMLFramePayload *next;
    uint8_t *pixels;
    size_t size;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
};

typedef enum AgfxDisplayCallbackKind {
    AGFX_DISPLAY_CALLBACK_MODE_CHANGE,
    AGFX_DISPLAY_CALLBACK_NEW_FRAME,
    AGFX_DISPLAY_CALLBACK_CURSOR_GLYPH,
    AGFX_DISPLAY_CALLBACK_CURSOR_MOVE,
    AGFX_DISPLAY_CALLBACK_CURSOR_SHOW,
} AgfxDisplayCallbackKind;

struct AppleGfxMLDisplayCallbackJob {
    AppleGfxMLDisplayCallbackJob *next;
    AppleGfxMLState *state;
    AgfxDisplayCallbackKind kind;
    union {
        struct {
            uint32_t width;
            uint32_t height;
            uint32_t iosurface_pixel_format;
            uint64_t protection_requirements;
        } mode_change;
        struct {
            uint8_t *pixels;
            uint64_t mapped_length;
            uint64_t stride;
            uint32_t width;
            uint32_t height;
            uint32_t hot_x;
            uint32_t hot_y;
            uint32_t sum;
        } cursor_glyph;
        struct {
            uint32_t display_id;
            uint32_t x;
            uint32_t y;
        } cursor_move;
        struct {
            uint32_t display_id;
            bool visible;
        } cursor_show;
    } u;
};

static void agfx_free_frame_payload(AppleGfxMLFramePayload *payload)
{
    if (!payload) {
        return;
    }

    g_free(payload->pixels);
    g_free(payload);
}

static void agfx_free_frame_completion_job(AppleGfxMLFrameCompletionJob *job)
{
    if (!job) {
        return;
    }

    g_free(job->frame_pixels);
    g_free(job);
}

static AppleGfxMLFramePayload *agfx_create_frame_payload(const void *pixels,
                                                         size_t size,
                                                         uint32_t width,
                                                         uint32_t height,
                                                         uint32_t stride)
{
    AppleGfxMLFramePayload *payload;

    if (!pixels || size == 0) {
        return NULL;
    }

    payload = g_new0(AppleGfxMLFramePayload, 1);
    payload->pixels = g_memdup2(pixels, size);
    if (!payload->pixels) {
        g_free(payload);
        return NULL;
    }

    payload->size = size;
    payload->width = width;
    payload->height = height;
    payload->stride = stride;
    return payload;
}

static void agfx_take_frame_payload(AppleGfxMLFrameCompletionJob *job,
                                    AppleGfxMLFramePayload *payload)
{
    if (!job || !payload) {
        return;
    }

    g_free(job->frame_pixels);
    job->frame_pixels = payload->pixels;
    job->frame_size = payload->size;
    job->frame_width = payload->width;
    job->frame_height = payload->height;
    job->frame_stride = payload->stride;
    job->frame_staged = true;
    g_free(payload);
}

static void agfx_enqueue_frame_payload_locked(AppleGfxMLState *s,
                                              AppleGfxMLFramePayload *payload)
{
    if (!s || !payload) {
        return;
    }

    payload->next = NULL;
    if (s->frame_payload_tail) {
        s->frame_payload_tail->next = payload;
    } else {
        s->frame_payload_head = payload;
    }
    s->frame_payload_tail = payload;
    s->frame_payload_count++;
}

static AppleGfxMLFramePayload *agfx_dequeue_frame_payload_locked(AppleGfxMLState *s)
{
    AppleGfxMLFramePayload *payload;

    if (!s) {
        return NULL;
    }

    payload = s->frame_payload_head;
    if (!payload) {
        return NULL;
    }

    s->frame_payload_head = payload->next;
    if (!s->frame_payload_head) {
        s->frame_payload_tail = NULL;
    }
    payload->next = NULL;
    if (s->frame_payload_count > 0) {
        s->frame_payload_count--;
    }
    return payload;
}

static void agfx_free_queued_frame_payloads(AppleGfxMLState *s)
{
    AppleGfxMLFramePayload *payload;

    if (!s) {
        return;
    }

    payload = s->frame_payload_head;
    s->frame_payload_head = NULL;
    s->frame_payload_tail = NULL;
    s->frame_payload_count = 0;

    while (payload) {
        AppleGfxMLFramePayload *next = payload->next;
        agfx_free_frame_payload(payload);
        payload = next;
    }
}

static void agfx_free_display_callback_job(AppleGfxMLDisplayCallbackJob *job)
{
    if (!job) {
        return;
    }

    if (job->kind == AGFX_DISPLAY_CALLBACK_CURSOR_GLYPH) {
        g_free(job->u.cursor_glyph.pixels);
    }
    g_free(job);
}

static void agfx_free_queued_display_callback_jobs(AppleGfxMLState *s)
{
    AppleGfxMLDisplayCallbackJob *job;

    if (!s) {
        return;
    }

    qemu_mutex_lock(&s->display_callback_mutex);
    job = s->display_callback_head;
    s->display_callback_head = NULL;
    s->display_callback_tail = NULL;
    qemu_mutex_unlock(&s->display_callback_mutex);

    while (job) {
        AppleGfxMLDisplayCallbackJob *next = job->next;
        agfx_free_display_callback_job(job);
        job = next;
    }
}

static void agfx_log_write_direct(const char *text)
{
    if (text) {
        qemu_log("%s", text);
    }
}

static bool agfx_log_should_emit(uint64_t *counter)
{
    uint64_t value;

    if (!counter) {
        return true;
    }

    value = ++(*counter);
    return value <= AGFX_LOG_INITIAL_COUNT || (value % AGFX_LOG_INTERVAL) == 0;
}

static void agfx_free_waiting_frame_completion_jobs(AppleGfxMLState *s)
{
    AppleGfxMLFrameCompletionJob *job;

    if (!s) {
        return;
    }

    job = s->frame_completion_wait_head;
    s->frame_completion_wait_head = NULL;
    s->frame_completion_wait_tail = NULL;

    while (job) {
        AppleGfxMLFrameCompletionJob *next = job->next;
        agfx_free_frame_completion_job(job);
        job = next;
    }
}

static void agfx_enqueue_log_owned(AppleGfxMLState *s, char *text)
{
    AgfxLogEntry *entry;

    if (!text) {
        return;
    }

    if (!s || !s->log_writer_started) {
        agfx_log_write_direct(text);
        g_free(text);
        return;
    }

    qemu_mutex_lock(&s->log_mutex);
    if (s->log_depth >= AGFX_LOG_QUEUE_LIMIT) {
        s->log_dropped++;
        qemu_mutex_unlock(&s->log_mutex);
        g_free(text);
        return;
    }

    entry = g_new0(AgfxLogEntry, 1);
    entry->text = text;
    if (s->log_tail) {
        s->log_tail->next = entry;
    } else {
        s->log_head = entry;
    }
    s->log_tail = entry;
    s->log_depth++;
    qemu_mutex_unlock(&s->log_mutex);

    qemu_sem_post(&s->log_sem);
}

static G_GNUC_PRINTF(2, 3)
void agfx_log(AppleGfxMLState *s, const char *fmt, ...)
{
    va_list args;
    char *text;

    if (!fmt) {
        return;
    }

    va_start(args, fmt);
    text = g_strdup_vprintf(fmt, args);
    va_end(args);

    agfx_enqueue_log_owned(s, text);
}

static void *agfx_log_writer_thread(void *opaque)
{
    AppleGfxMLState *s = opaque;

    while (true) {
        AgfxLogEntry *entry;
        uint64_t dropped = 0;

        qemu_sem_wait(&s->log_sem);

        qemu_mutex_lock(&s->log_mutex);
        entry = s->log_head;
        if (entry) {
            s->log_head = entry->next;
            if (!s->log_head) {
                s->log_tail = NULL;
            }
            s->log_depth--;
        }
        if (!entry && s->log_writer_stop) {
            qemu_mutex_unlock(&s->log_mutex);
            break;
        }
        if (s->log_dropped) {
            dropped = s->log_dropped;
            s->log_dropped = 0;
        }
        qemu_mutex_unlock(&s->log_mutex);

        if (dropped) {
            qemu_log("[apple-gfx-ml][logger] dropped %" PRIu64 " log lines\n",
                     dropped);
        }
        if (!entry) {
            continue;
        }

        agfx_log_write_direct(entry->text);
        g_free(entry->text);
        g_free(entry);
    }

    return NULL;
}

static void agfx_log_init(AppleGfxMLState *s)
{
    qemu_mutex_init(&s->log_mutex);
    qemu_sem_init(&s->log_sem, 0);
    s->log_writer_stop = false;
    s->log_writer_started = true;
    s->log_head = NULL;
    s->log_tail = NULL;
    s->log_depth = 0;
    s->log_dropped = 0;
    qemu_thread_create(&s->log_writer, "agfx-log",
                       agfx_log_writer_thread, s, QEMU_THREAD_JOINABLE);
}

static void agfx_log_stop(AppleGfxMLState *s)
{
    AgfxLogEntry *entry;

    if (!s->log_writer_started) {
        return;
    }

    qemu_mutex_lock(&s->log_mutex);
    s->log_writer_stop = true;
    qemu_mutex_unlock(&s->log_mutex);
    qemu_sem_post(&s->log_sem);
    qemu_thread_join(&s->log_writer);
    qemu_sem_destroy(&s->log_sem);

    qemu_mutex_lock(&s->log_mutex);
    entry = s->log_head;
    s->log_head = NULL;
    s->log_tail = NULL;
    s->log_depth = 0;
    qemu_mutex_unlock(&s->log_mutex);
    while (entry) {
        AgfxLogEntry *next = entry->next;
        g_free(entry->text);
        g_free(entry);
        entry = next;
    }
    qemu_mutex_destroy(&s->log_mutex);
    s->log_writer_started = false;
}

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
    AppleGfxMLState *s = ctx;
    char *body;
    char *full;
    const char *base = file;

    if (!fmt) {
        return;
    }

    if (file) {
        const char *slash = strrchr(file, '/');
        if (slash && slash[1]) {
            base = slash + 1;
        }
    }

    body = g_strdup_vprintf(fmt, args);
    full = g_strdup_printf("[apple-gfx-ml][qmetal][%s][%s] %s:%d %s: %s\n",
                           qmu_log_level_name(level),
                           qmu_log_category_name(category),
                           base ? base : "?",
                           line,
                           func ? func : "?",
                           body ? body : "");
    g_free(body);
    agfx_enqueue_log_owned(s, full);
}

/* ============================================================
 * Memory Access Callbacks (QEMU → qmetal)
 * These match Apple's PGDeviceDescriptor callback interface
 * ============================================================ */

static void *qemu_map_gpa(void *ctx, uint64_t gpa, size_t size, int writable)
{
    AppleGfxMLState *s = ctx;
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
    if (s && s->debug_level >= 5) {
        trace_apple_gfx_ml_map_gpa(gpa, size, ptr + xlat, writable);
    }
    return ptr + xlat;
}

static void qemu_unmap_gpa(void *ctx, void *hva, size_t size, int dirty)
{
    AppleGfxMLState *s = ctx;
    if (s && s->debug_level >= 5) {
        trace_apple_gfx_ml_unmap_gpa(hva, size);
    }
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
    AppleGfxMLState *s = ctx;
    AgfxDMAJob job = { .gpa = gpa, .buf = buf, .size = size,
                       .is_write = false };
    if (s && s->debug_level >= 5) {
        trace_apple_gfx_ml_dma_read(gpa, size);
    }
    qemu_event_init(&job.event, false);
    aio_bh_schedule_oneshot(qemu_get_aio_context(), agfx_do_dma, &job);
    qemu_event_wait(&job.event);
    qemu_event_destroy(&job.event);
    return (job.result == MEMTX_OK) ? 0 : -1;
}

static int qemu_write_memory(void *ctx, uint64_t gpa, const void *buf,
                              size_t size)
{
    AppleGfxMLState *s = ctx;
    AgfxDMAJob job = { .gpa = gpa, .buf = (void *)buf, .size = size,
                       .is_write = true, .dirty = true };
    if (s && s->debug_level >= 5) {
        trace_apple_gfx_ml_dma_write(gpa, size);
    }
    qemu_event_init(&job.event, false);
    aio_bh_schedule_oneshot(qemu_get_aio_context(), agfx_do_dma, &job);
    qemu_event_wait(&job.event);
    qemu_event_destroy(&job.event);
    if (job.result != MEMTX_OK && s && s->debug_level >= 5) {
        trace_apple_gfx_ml_dma_write_failed(gpa, size, job.result);
    }
    return (job.result == MEMTX_OK) ? 0 : -1;
}

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
        agfx_log(s, "[apple-gfx-ml] raise_irq #%lu: msi_enabled=%d\n",
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

static void agfx_request_display_render(AppleGfxMLState *s);
static void agfx_merge_bootstrap_present_source_locked(AppleGfxMLState *s);
static bool agfx_take_bootstrap_present_source_locked(AppleGfxMLState *s);
static void apple_gfx_ml_frame_completed_bh(void *opaque);
static void apple_gfx_ml_mode_change_bh(void *opaque);
static void apple_gfx_ml_cursor_glyph_bh(void *opaque);
static void apple_gfx_ml_cursor_move_bh(void *opaque);
static void apple_gfx_ml_cursor_show_bh(void *opaque);
static void agfx_new_frame_handler_bh(void *opaque);
static void agfx_schedule_frame_presents(AppleGfxMLState *s);
static void agfx_schedule_frame_presents_locked(AppleGfxMLState *s, bool *started);
static void agfx_cancel_frame_presents(AppleGfxMLState *s);
static void agfx_cancel_frame_presents_locked(AppleGfxMLState *s);
static void agfx_enqueue_session_job(AppleGfxMLState *s,
                                     AppleGfxMLSessionJob *job);
static void agfx_enqueue_display_callback_job(AppleGfxMLState *s,
                                              AppleGfxMLDisplayCallbackJob *job);
static void qemu_frame_completed(void *ctx, int frame_expected);

static void agfx_kick_display_render(AppleGfxMLState *s, struct qmu_vulkan_ctx *vk)
{
    int rc;

    if (!s || !vk) {
        return;
    }

    rc = qmu_vk_begin_display_frame(vk);
    if (rc > 0) {
        if (agfx_log_should_emit(&s->render_worker_log_count)) {
            agfx_log(s,
                     "[apple-gfx-ml] kick_display_render: iosfc_async pending_frames=%d mmio_wait=%d\n",
                     __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST),
                     qatomic_read(&s->mmio_wait_active));
        }
        return;
    }
    if (rc == 0) {
        agfx_request_display_render(s);
        return;
    }

    if (agfx_log_should_emit(&s->render_worker_log_count)) {
        agfx_log(s,
                 "[apple-gfx-ml] kick_display_render: begin_display_frame failed pending_frames=%d\n",
                 __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST));
    }
    qemu_frame_completed(s, 0);
}

typedef enum AgfxSessionJobKind {
    AGFX_SESSION_JOB_MMIO_READ,
    AGFX_SESSION_JOB_MMIO_WRITE,
} AgfxSessionJobKind;

struct AppleGfxMLSessionJob {
    AppleGfxMLState *state;
    AppleGfxMLSessionJob *next;
    AgfxSessionJobKind kind;
    uint64_t offset;
    uint64_t value;
    unsigned size;
    bool completed;
    bool heap_owned;
};

typedef struct AgfxCompletionJob {
    void (*fn)(void *);
    void *ctx;
} AgfxCompletionJob;

static void agfx_display_completion_bh(void *opaque)
{
    AgfxCompletionJob *job = opaque;

    if (!job) {
        return;
    }

    /* Keep the completion BH lock-free on the main loop. qmu's Transaction3
     * completion path is designed to run asynchronously after releasing its
     * internal display lock so the MMIO worker can keep waiting on main-loop
     * BH/AIO progress without the completion callback blocking that loop. */
    job->fn(job->ctx);
    g_free(job);
}

static void agfx_publish_display_mode(AppleGfxMLState *s,
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

typedef struct AppleGfxMLModeChangeJob {
    AppleGfxMLState *state;
    uint32_t width;
    uint32_t height;
    uint32_t iosurface_pixel_format;
    uint64_t protection_requirements;
    bool wait_for_completion;
    QemuEvent completion;
} AppleGfxMLModeChangeJob;

static void apple_gfx_ml_mode_change_bh(void *opaque)
{
    AppleGfxMLModeChangeJob *job = opaque;
    AppleGfxMLState *s = job ? job->state : NULL;

    if (!job) {
        return;
    }
    if (!s) {
        if (job->wait_for_completion) {
            qemu_event_set(&job->completion);
        } else {
            g_free(job);
        }
        return;
    }

    agfx_log(s,
             "[apple-gfx-ml] mode_change: %ux%u iosurface_pf=0x%08x protection=0x%016" PRIx64 "\n",
             job->width,
             job->height,
             job->iosurface_pixel_format,
             job->protection_requirements);
    agfx_publish_display_mode(s,
                              job->width,
                              job->height,
                              job->iosurface_pixel_format,
                              job->protection_requirements);
    if (job->wait_for_completion) {
        qemu_event_set(&job->completion);
    } else {
        g_free(job);
    }
}

static bool apple_gfx_ml_apply_staged_frame(AppleGfxMLState *s,
                                            AppleGfxMLFrameCompletionJob *job)
{
    uint32_t width, height, stride;
    const uint8_t *pixels;
    size_t size;

    if (!s || !job || !job->frame_pixels || job->frame_size == 0) {
        return false;
    }

    width = job->frame_width;
    height = job->frame_height;
    stride = job->frame_stride;
    size = job->frame_size;
    pixels = job->frame_pixels;

    agfx_publish_display_mode(s, width, height, s->fb_iosurface_pixel_format,
                              s->fb_protection_requirements);

    /* Copy from the frame-owned payload to the display buffer. */
    if (pixels && s->display_fb) {
        memcpy(s->display_fb, pixels, size);
    }

    /* Update frame counter and log */
    s->frame_count++;
    if (s->frame_count <= AGFX_LOG_INITIAL_COUNT || (s->frame_count % AGFX_LOG_INTERVAL) == 0) {
        agfx_log(s, "[apple-gfx-ml] frame_completed_bh: present #%lu %ux%u stride=%u\n",
                 (unsigned long)s->frame_count, width, height, stride);
    }

    /* Coordinate with gfx_update via reference two-flag model */
    if (s->gfx_update_requested) {
        s->gfx_update_requested = false;
        dpy_gfx_update_full(s->con);
        graphic_hw_update_done(s->con);
        s->new_frame_ready = false;
    } else {
        s->new_frame_ready = true;
    }

    return true;
}

static void qemu_present_frame(void *ctx, const void *pixels,
                                uint32_t width, uint32_t height, uint32_t stride)
{
    AppleGfxMLState *s = ctx;
    size_t size = (size_t)height * stride;
    AppleGfxMLFramePayload *payload;
    AppleGfxMLFrameCompletionJob *wait_job = NULL;

    /* Log from pthread (before scheduling BH) */
    uint64_t pc = qatomic_fetch_inc(&s->present_count) + 1;
    if (pc <= AGFX_LOG_INITIAL_COUNT || (pc % AGFX_LOG_INTERVAL) == 0) {
        agfx_log(s, "[apple-gfx-ml] present_frame #%lu: %ux%u stride=%u (from pthread)\n",
                 (unsigned long)pc, width, height, stride);
    }
    
    payload = agfx_create_frame_payload(pixels, size, width, height, stride);
    if (!payload) {
        agfx_log(s,
                 "[apple-gfx-ml] present_frame: payload allocation failed %ux%u stride=%u\n",
                 width, height, stride);
        return;
    }

    qemu_mutex_lock(&s->frame_mutex);
    wait_job = s->frame_completion_wait_head;
    if (wait_job) {
        s->frame_completion_wait_head = wait_job->next;
        if (!s->frame_completion_wait_head) {
            s->frame_completion_wait_tail = NULL;
        }
        wait_job->next = NULL;
        agfx_take_frame_payload(wait_job, payload);
        payload = NULL;
    } else {
        agfx_enqueue_frame_payload_locked(s, payload);
        payload = NULL;
    }
    qemu_mutex_unlock(&s->frame_mutex);

    if (payload) {
        agfx_free_frame_payload(payload);
    }
    
    /* If completion arrived first, wake that same job once its frame-owned
     * payload has been attached. */
    if (wait_job) {
        aio_bh_schedule_oneshot(qemu_get_aio_context(),
                                apple_gfx_ml_frame_completed_bh, wait_job);
    }
}

static bool apple_gfx_ml_defer_until_staged_frame(AppleGfxMLState *s,
                                                  AppleGfxMLFrameCompletionJob *job)
{
    AppleGfxMLFramePayload *payload;

    if (!s || !job || job->frame_staged) {
        return false;
    }

    qemu_mutex_lock(&s->frame_mutex);
    payload = agfx_dequeue_frame_payload_locked(s);
    if (payload) {
        agfx_take_frame_payload(job, payload);
        qemu_mutex_unlock(&s->frame_mutex);
        return false;
    }

    job->next = NULL;
    job->frame_staged = false;
    if (s->frame_completion_wait_tail) {
        s->frame_completion_wait_tail->next = job;
    } else {
        s->frame_completion_wait_head = job;
    }
    s->frame_completion_wait_tail = job;
    qemu_mutex_unlock(&s->frame_mutex);

    return true;
}

static void apple_gfx_ml_frame_completed_bh(void *opaque)
{
    AppleGfxMLFrameCompletionJob *job = opaque;
    AppleGfxMLState *s = job ? job->state : NULL;
    bool frame_applied;

    if (!job || !s) {
        agfx_free_frame_completion_job(job);
        return;
    }

    if (job->frame_expected && apple_gfx_ml_defer_until_staged_frame(s, job)) {
        return;
    }

    if (!job->pending_accounted) {
        int pending = __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST);
        if (pending > 0) {
            pending = __atomic_sub_fetch(&s->pending_frames, 1, __ATOMIC_SEQ_CST);
        }
        job->chain_needed = pending > 0;
        job->pending_accounted = true;
    }

    frame_applied = job->frame_expected ? apple_gfx_ml_apply_staged_frame(s, job) : false;

    if (agfx_log_should_emit(&s->frame_completed_log_count)) {
        agfx_log(s,
                 "[apple-gfx-ml] frame_completed_bh: pending_frames=%d frame_applied=%d mmio_wait=%d\n",
                 __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST),
                 frame_applied ? 1 : 0,
                 qatomic_read(&s->mmio_wait_active));
    }

    if (job->chain_needed && s->qmu_dev) {
        struct qmu_vulkan_ctx *vk = qmu_session_get_vulkan(s->qmu_dev);
        if (!vk) {
            agfx_log(s, "[apple-gfx-ml] frame_completed_bh: vk=NULL\n");
        } else {
            agfx_kick_display_render(s, vk);
        }
    }

    agfx_free_frame_completion_job(job);
}

static void qemu_frame_completed(void *ctx, int frame_expected)
{
    AppleGfxMLState *s = ctx;
    AppleGfxMLFrameCompletionJob *job;

    if (!s) {
        return;
    }

    job = g_new0(AppleGfxMLFrameCompletionJob, 1);
    job->state = s;
    job->frame_expected = frame_expected != 0;
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            apple_gfx_ml_frame_completed_bh, job);
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

typedef struct AppleGfxMLCursorMoveJob {
    AppleGfxMLState *state;
    uint32_t display_id;
    uint32_t x;
    uint32_t y;
} AppleGfxMLCursorMoveJob;

static void agfx_enqueue_display_callback_job(AppleGfxMLState *s,
                                              AppleGfxMLDisplayCallbackJob *job)
{
    if (!s || !job) {
        agfx_free_display_callback_job(job);
        return;
    }

    job->next = NULL;
    qemu_mutex_lock(&s->display_callback_mutex);
    if (s->display_callback_tail) {
        s->display_callback_tail->next = job;
    } else {
        s->display_callback_head = job;
    }
    s->display_callback_tail = job;
    qemu_mutex_unlock(&s->display_callback_mutex);
    qemu_sem_post(&s->display_callback_sem);
}

static void agfx_deliver_new_frame_signal(AppleGfxMLState *s)
{
    struct qmu_vulkan_ctx *vk = NULL;

    if (!s) {
        return;
    }

    if (s->qmu_dev) {
        vk = qmu_session_get_vulkan(s->qmu_dev);
    }

    if (vk) {
        qmu_vk_consume_current_frame_signal(vk);
    }

    if (agfx_log_should_emit(&s->new_frame_signal_log_count)) {
        agfx_log(s,
                 "[apple-gfx-ml] new_frame_signal: enqueue pending_frames=%d mmio_wait=%d\n",
                 __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST),
                 qatomic_read(&s->mmio_wait_active));
    }
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            agfx_new_frame_handler_bh, s);
}

static void *agfx_display_callback_thread(void *opaque)
{
    AppleGfxMLState *s = opaque;

    while (true) {
        AppleGfxMLDisplayCallbackJob *job = NULL;

        qemu_sem_wait(&s->display_callback_sem);

        qemu_mutex_lock(&s->display_callback_mutex);
        if (s->display_callback_head) {
            job = s->display_callback_head;
            s->display_callback_head = job->next;
            if (!s->display_callback_head) {
                s->display_callback_tail = NULL;
            }
        }
        qemu_mutex_unlock(&s->display_callback_mutex);

        if (!job) {
            if (s->display_callback_worker_stop) {
                break;
            }
            continue;
        }

        switch (job->kind) {
        case AGFX_DISPLAY_CALLBACK_MODE_CHANGE: {
            AppleGfxMLModeChangeJob bh_job = {
                .state = s,
                .width = job->u.mode_change.width,
                .height = job->u.mode_change.height,
                .iosurface_pixel_format = job->u.mode_change.iosurface_pixel_format,
                .protection_requirements = job->u.mode_change.protection_requirements,
                .wait_for_completion = true,
            };

            /* Reference modeChangeHandler runs on the display queue itself.
             * Round-trip through the main loop here so later callback jobs are
             * not delivered until agfx_publish_display_mode() has completed. */
            qemu_event_init(&bh_job.completion, false);
            aio_bh_schedule_oneshot(qemu_get_aio_context(),
                                    apple_gfx_ml_mode_change_bh, &bh_job);
            qemu_event_wait(&bh_job.completion);
            qemu_event_destroy(&bh_job.completion);
            break;
        }
        case AGFX_DISPLAY_CALLBACK_NEW_FRAME:
            agfx_deliver_new_frame_signal(s);
            break;
        case AGFX_DISPLAY_CALLBACK_CURSOR_GLYPH: {
            AppleGfxMLCursorGlyphJob *bh_job = g_new0(AppleGfxMLCursorGlyphJob, 1);

            bh_job->state = s;
            bh_job->pixels = job->u.cursor_glyph.pixels;
            bh_job->mapped_length = job->u.cursor_glyph.mapped_length;
            bh_job->stride = job->u.cursor_glyph.stride;
            bh_job->width = job->u.cursor_glyph.width;
            bh_job->height = job->u.cursor_glyph.height;
            bh_job->hot_x = job->u.cursor_glyph.hot_x;
            bh_job->hot_y = job->u.cursor_glyph.hot_y;
            bh_job->sum = job->u.cursor_glyph.sum;
            job->u.cursor_glyph.pixels = NULL;
            aio_bh_schedule_oneshot(qemu_get_aio_context(),
                                    apple_gfx_ml_cursor_glyph_bh, bh_job);
            break;
        }
        case AGFX_DISPLAY_CALLBACK_CURSOR_MOVE: {
            AppleGfxMLCursorMoveJob *bh_job = g_new0(AppleGfxMLCursorMoveJob, 1);

            bh_job->state = s;
            bh_job->display_id = job->u.cursor_move.display_id;
            bh_job->x = job->u.cursor_move.x;
            bh_job->y = job->u.cursor_move.y;
            aio_bh_schedule_oneshot(qemu_get_aio_context(),
                                    apple_gfx_ml_cursor_move_bh, bh_job);
            break;
        }
        case AGFX_DISPLAY_CALLBACK_CURSOR_SHOW: {
            AppleGfxMLCursorShowJob *bh_job = g_new0(AppleGfxMLCursorShowJob, 1);

            bh_job->state = s;
            bh_job->display_id = job->u.cursor_show.display_id;
            bh_job->visible = job->u.cursor_show.visible;
            aio_bh_schedule_oneshot(qemu_get_aio_context(),
                                    apple_gfx_ml_cursor_show_bh, bh_job);
            break;
        }
        }

        agfx_free_display_callback_job(job);
    }

    return NULL;
}

static void apple_gfx_ml_update_cursor(AppleGfxMLState *s)
{
    if (!s->con) {
        return;
    }
    dpy_mouse_set(s->con, s->cursor_x, s->cursor_y, s->cursor_show);
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

    agfx_log(s, "[apple-gfx-ml] cursor_glyph: %ux%u stride=%" PRIu64 " hot=%u,%u sum=0x%08x\n",
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
    agfx_log(s, "[apple-gfx-ml] cursor_show: display=%u visible=%d\n",
             job->display_id, job->visible ? 1 : 0);
    apple_gfx_ml_update_cursor(s);
    g_free(job);
}

static void apple_gfx_ml_cursor_move_bh(void *opaque)
{
    AppleGfxMLCursorMoveJob *job = opaque;
    AppleGfxMLState *s = job->state;

    s->cursor_x = job->x;
    s->cursor_y = job->y;
    agfx_log(s, "[apple-gfx-ml] cursor_move: display=%u pos=%u,%u\n",
             job->display_id, job->x, job->y);
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
    AppleGfxMLDisplayCallbackJob *job;

    if (!s || !pixels || mapped_length == 0) {
        return;
    }

    job = g_new0(AppleGfxMLDisplayCallbackJob, 1);
    job->state = s;
    job->kind = AGFX_DISPLAY_CALLBACK_CURSOR_GLYPH;
    job->u.cursor_glyph.pixels = g_memdup2(pixels, mapped_length);
    if (!job->u.cursor_glyph.pixels) {
        g_free(job);
        return;
    }
    job->u.cursor_glyph.mapped_length = mapped_length;
    job->u.cursor_glyph.stride = stride;
    job->u.cursor_glyph.width = width;
    job->u.cursor_glyph.height = height;
    job->u.cursor_glyph.hot_x = hot_x;
    job->u.cursor_glyph.hot_y = hot_y;
    job->u.cursor_glyph.sum = sum;
    agfx_enqueue_display_callback_job(s, job);
}

static void qemu_cursor_show(void *ctx, uint32_t display_id, int visible)
{
    AppleGfxMLState *s = ctx;
    AppleGfxMLDisplayCallbackJob *job;

    if (!s) {
        return;
    }

    job = g_new0(AppleGfxMLDisplayCallbackJob, 1);
    job->state = s;
    job->kind = AGFX_DISPLAY_CALLBACK_CURSOR_SHOW;
    job->u.cursor_show.display_id = display_id;
    job->u.cursor_show.visible = visible != 0;
    agfx_enqueue_display_callback_job(s, job);
}

static void qemu_cursor_move(void *ctx,
                             uint32_t display_id,
                             uint32_t x,
                             uint32_t y)
{
    AppleGfxMLState *s = ctx;
    AppleGfxMLDisplayCallbackJob *job;

    if (!s) {
        return;
    }

    job = g_new0(AppleGfxMLDisplayCallbackJob, 1);
    job->state = s;
    job->kind = AGFX_DISPLAY_CALLBACK_CURSOR_MOVE;
    job->u.cursor_move.display_id = display_id;
    job->u.cursor_move.x = x;
    job->u.cursor_move.y = y;
    agfx_enqueue_display_callback_job(s, job);
}

static void qemu_mode_change(void *ctx,
                             uint32_t width,
                             uint32_t height,
                             uint32_t iosurface_pixel_format,
                             uint64_t protection_requirements)
{
    AppleGfxMLState *s = ctx;
    AppleGfxMLDisplayCallbackJob *job;

    if (!s || width == 0 || height == 0) {
        return;
    }

    job = g_new0(AppleGfxMLDisplayCallbackJob, 1);
    job->state = s;
    job->kind = AGFX_DISPLAY_CALLBACK_MODE_CHANGE;
    job->u.mode_change.width = width;
    job->u.mode_change.height = height;
    job->u.mode_change.iosurface_pixel_format = iosurface_pixel_format;
    job->u.mode_change.protection_requirements = protection_requirements;
    agfx_enqueue_display_callback_job(s, job);
}

static void agfx_request_display_render(AppleGfxMLState *s)
{
    if (!s || !s->qmu_dev) {
        return;
    }

    qemu_mutex_lock(&s->render_mutex);
    s->display_render_requests++;
    qemu_mutex_unlock(&s->render_mutex);
    qemu_sem_post(&s->render_sem);
}

static void agfx_merge_bootstrap_present_source_locked(AppleGfxMLState *s)
{
    if (!s->bootstrap_present_source.pending) {
        s->bootstrap_present_source.pending = true;
        qemu_cond_signal(&s->bootstrap_present_cond);
    }
}

static bool agfx_take_bootstrap_present_source_locked(AppleGfxMLState *s)
{
    if (!s || !s->bootstrap_present_source.pending) {
        return false;
    }

    s->bootstrap_present_source.pending = false;
    return true;
}

static void agfx_schedule_frame_presents_locked(AppleGfxMLState *s, bool *started)
{
    if (!s) {
        return;
    }

    if (!s->bootstrap_present_timer.active) {
        s->bootstrap_present_timer.active = true;
        s->bootstrap_present_timer.next_fire_us = g_get_monotonic_time();
        if (started) {
            *started = true;
        }
        qemu_cond_signal(&s->bootstrap_present_cond);
        return;
    }

    agfx_merge_bootstrap_present_source_locked(s);
}

static void agfx_schedule_frame_presents(AppleGfxMLState *s)
{
    bool started = false;

    if (!s) {
        return;
    }

    qemu_mutex_lock(&s->bootstrap_present_mutex);
    agfx_schedule_frame_presents_locked(s, &started);
    qemu_mutex_unlock(&s->bootstrap_present_mutex);

    if (started) {
        agfx_log(s,
                 "[apple-gfx-ml] display frame timer started (%dms interval, present queue)\n",
                 AGFX_DISPLAY_FRAME_INTERVAL_MS);
    }
}

static void agfx_cancel_frame_presents_locked(AppleGfxMLState *s)
{
    if (!s) {
        return;
    }

    s->bootstrap_present_timer.active = false;
    s->bootstrap_present_timer.next_fire_us = 0;
    s->bootstrap_present_source.pending = false;
}

static void agfx_cancel_frame_presents(AppleGfxMLState *s)
{
    if (!s) {
        return;
    }

    qemu_mutex_lock(&s->bootstrap_present_mutex);
    agfx_cancel_frame_presents_locked(s);
    qemu_cond_signal(&s->bootstrap_present_cond);
    qemu_mutex_unlock(&s->bootstrap_present_mutex);
}

static void agfx_bootstrap_present_on_queue(AppleGfxMLState *s)
{
    if (!s || !s->qmu_dev || !qatomic_read(&s->iosfc_bootstrap_active)) {
        return;
    }

    if (agfx_log_should_emit(&s->bootstrap_present_log_count)) {
        agfx_log(s,
                 "[apple-gfx-ml] bootstrap_present_queue: iosfc_present_tick mmio_wait=%d pending_frames=%d\n",
                 qatomic_read(&s->mmio_wait_active),
                 __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST));
    }

    (void)qmu_iosfc_present_tick(s->qmu_dev);
}

static void *agfx_bootstrap_present_thread(void *opaque)
{
    AppleGfxMLState *s = opaque;
    const int64_t interval_us =
        (int64_t)AGFX_DISPLAY_FRAME_INTERVAL_MS * 1000;

    qemu_mutex_lock(&s->bootstrap_present_mutex);
    while (!s->bootstrap_present_worker_stop) {
        int64_t now_us;
        int64_t wait_ms;

        if (agfx_take_bootstrap_present_source_locked(s)) {
            qemu_mutex_unlock(&s->bootstrap_present_mutex);
            agfx_bootstrap_present_on_queue(s);
            qemu_mutex_lock(&s->bootstrap_present_mutex);
            continue;
        }

        if (!s->bootstrap_present_timer.active) {
            qemu_cond_wait(&s->bootstrap_present_cond,
                           &s->bootstrap_present_mutex);
            continue;
        }

        now_us = g_get_monotonic_time();
        if (s->bootstrap_present_timer.next_fire_us != 0 &&
            now_us >= s->bootstrap_present_timer.next_fire_us) {
            do {
                s->bootstrap_present_timer.next_fire_us += interval_us;
            } while (s->bootstrap_present_timer.next_fire_us <= now_us);

            if (qatomic_read(&s->iosfc_bootstrap_active)) {
                /* Reference timer source only merges the lightweight present
                 * source on PGEFIPresentQueue; the present source handler
                 * performs qmu_iosfc_present_tick() later on the same queue. */
                agfx_merge_bootstrap_present_source_locked(s);
            }
            continue;
        }

        wait_ms = AGFX_DISPLAY_FRAME_INTERVAL_MS;
        if (s->bootstrap_present_timer.next_fire_us != 0) {
            int64_t delta_us = s->bootstrap_present_timer.next_fire_us - now_us;
            if (delta_us <= 0) {
                wait_ms = 0;
            } else {
                wait_ms = (delta_us + 999) / 1000;
            }
        }
        qemu_cond_timedwait(&s->bootstrap_present_cond,
                            &s->bootstrap_present_mutex,
                            wait_ms);
    }
    qemu_mutex_unlock(&s->bootstrap_present_mutex);

    return NULL;
}

/* Reference newFrameEventHandler schedules a BH onto the QEMU main loop from
 * the display queue. The BH does the pending_frames throttle and actual render
 * kickoff. */
static void agfx_new_frame_handler_bh(void *opaque)
{
    AppleGfxMLState *s = opaque;
    struct qmu_vulkan_ctx *vk;
    int pending;
    uint32_t queued_payloads = 0;

    if (!s || !s->qmu_dev) {
        return;
    }

    vk = qmu_session_get_vulkan(s->qmu_dev);
    if (!vk) {
        agfx_log(s, "[apple-gfx-ml] new_frame_handler_bh: vk=NULL\n");
        return;
    }

    qemu_mutex_lock(&s->frame_mutex);
    queued_payloads = s->frame_payload_count;
    qemu_mutex_unlock(&s->frame_mutex);

    if (agfx_log_should_emit(&s->new_frame_handler_log_count)) {
        agfx_log(s,
                 "[apple-gfx-ml] new_frame_handler_bh: pending_frames=%d queued_payloads=%u mmio_wait=%d\n",
                 __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST),
                 queued_payloads,
                 qatomic_read(&s->mmio_wait_active));
    }

    /* Reference throttle: pending_frames >= 2 → drop (apple-gfx.m:2672) */
    pending = __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST);
    if (pending >= 2) {
        if (agfx_log_should_emit(&s->new_frame_handler_log_count)) {
            agfx_log(s, "[apple-gfx-ml] new_frame_handler_bh: drop pending_frames=%d\n",
                     pending);
        }
        return;
    }
    pending = __atomic_add_fetch(&s->pending_frames, 1, __ATOMIC_SEQ_CST);

    /* Reference: if pending > 1, another frame will chain from completion (2678) */
    if (pending > 1) {
        if (agfx_log_should_emit(&s->new_frame_handler_log_count)) {
            agfx_log(s,
                     "[apple-gfx-ml] new_frame_handler_bh: chain-only pending_frames=%d\n",
                     pending);
        }
        return;
    }

    /* First frame — request encode (reference: apple_gfx_render_new_frame) */
    if (agfx_log_should_emit(&s->new_frame_handler_log_count)) {
        agfx_log(s,
                 "[apple-gfx-ml] new_frame_handler_bh: queue_render pending_frames=%d\n",
                 pending);
    }
    agfx_kick_display_render(s, vk);
}

static void qemu_new_frame_signal(void *ctx)
{
    AppleGfxMLState *s = ctx;
    AppleGfxMLDisplayCallbackJob *job;

    if (!s) {
        return;
    }

    job = g_new0(AppleGfxMLDisplayCallbackJob, 1);
    job->state = s;
    job->kind = AGFX_DISPLAY_CALLBACK_NEW_FRAME;
    agfx_enqueue_display_callback_job(s, job);
}

/* Display refresh is handled by qmetal library's internal thread after the
 * display-owned encode path completes. Bootstrap IOSFC presentation is driven
 * by the separate reference-like PGEFIPresentQueue analogue above. */

/* ============================================================
 * Display Completion BH Trampoline
 *
 * Matches reference GCD dispatch_async for presentSurface completion.
 * qmu_session expects this to run in the main-loop BH plane with
 * read_memory_mainloop/write_memory_mainloop semantics.
 * ============================================================ */

static void qemu_schedule_display_completion(void *ctx,
                                              void (*fn)(void *),
                                              void *comp_ctx)
{
    AppleGfxMLState *s = ctx;
    AgfxCompletionJob *job;

    if (!s || !fn) {
        return;
    }

    job = g_new0(AgfxCompletionJob, 1);
    job->fn = fn;
    job->ctx = comp_ctx;

    agfx_log(s, "[apple-gfx-ml] schedule_display_completion: enqueue mmio_wait=%d\n",
             qatomic_read(&s->mmio_wait_active));
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

    (void)ctx;

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

    (void)ctx;
    (void)len;

    RCU_READ_LOCK_GUARD();
    mr = memory_region_from_host(hva, &offset);
    if (!mr) {
        agfx_log_write_direct("[apple-gfx-ml] iosfc_unmap: memory not found\n");
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

static void agfx_enqueue_session_job(AppleGfxMLState *s, AppleGfxMLSessionJob *job)
{
    job->next = NULL;
    qemu_mutex_lock(&s->mmio_job_mutex);
    if (s->session_job_tail) {
        s->session_job_tail->next = job;
    } else {
        s->session_job_head = job;
    }
    s->session_job_tail = job;
    qemu_mutex_unlock(&s->mmio_job_mutex);
    qemu_sem_post(&s->mmio_sem);
}

static void *agfx_mmio_worker_thread(void *opaque)
{
    AppleGfxMLState *s = opaque;
    while (true) {
        AppleGfxMLSessionJob *job = NULL;

        qemu_sem_wait(&s->mmio_sem);

        qemu_mutex_lock(&s->mmio_job_mutex);
        if (s->session_job_head) {
            job = s->session_job_head;
            s->session_job_head = job->next;
            if (!s->session_job_head) {
                s->session_job_tail = NULL;
            }
        }
        qemu_mutex_unlock(&s->mmio_job_mutex);

        if (!job) {
            if (qatomic_read(&s->mmio_worker_stop)) {
                break;
            }
            continue;
        }

        switch (job->kind) {
        case AGFX_SESSION_JOB_MMIO_READ:
            /* Reads now handled synchronously in agfx_mmio_read (no worker queue).
             * This case should not be reached. */
            job->value = qmu_mmio_read(job->state->qmu_dev,
                                       (uint32_t)job->offset, job->size);
            qatomic_set(&job->completed, true);
            aio_wait_kick();
            break;
        case AGFX_SESSION_JOB_MMIO_WRITE:
            /* session_mutex serializes MMIO writes with render_worker
             * (both access shared session state including active_cmd_buffer). */
            qemu_mutex_lock(&s->session_mutex);
            qmu_mmio_write(job->state->qmu_dev,
                           (uint32_t)job->offset, job->value, job->size);
            qemu_mutex_unlock(&s->session_mutex);
            qatomic_set(&job->completed, true);
            aio_wait_kick();
            break;
        }

        if (job->heap_owned) {
            g_free(job);
        }
    }
    return NULL;
}

static void *agfx_render_worker_thread(void *opaque)
{
    AppleGfxMLState *s = opaque;

    while (true) {
        int display_count = 0;
        struct qmu_vulkan_ctx *vk = NULL;

        qemu_sem_wait(&s->render_sem);

        if (qatomic_read(&s->render_worker_stop)) {
            break;
        }

        /* Reference newFrameEventHandler owns the heavy encode path. Bootstrap
         * IOSFC present publication is handled by the lightweight present BH. */
        qemu_mutex_lock(&s->render_mutex);
        if (s->display_render_requests > 0) {
            display_count = 1;
            s->display_render_requests--;
        }
        qemu_mutex_unlock(&s->render_mutex);

        if (display_count <= 0 || !s->qmu_dev) {
            continue;
        }

        vk = qmu_session_get_vulkan(s->qmu_dev);
        if (!vk) {
            agfx_log(s, "[apple-gfx-ml] render_worker: vk=NULL\n");
            continue;
        }

        if (agfx_log_should_emit(&s->render_worker_log_count)) {
            agfx_log(s,
                     "[apple-gfx-ml] render_worker: request_display_frame x%d mmio_wait=%d pending_frames=%d\n",
                     display_count,
                     qatomic_read(&s->mmio_wait_active),
                     __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST));
        }
        if (qmu_vk_request_display_frame(vk) <= 0) {
            if (agfx_log_should_emit(&s->render_worker_log_count)) {
                agfx_log(s,
                         "[apple-gfx-ml] render_worker: request_display_frame retired without callback pending_frames=%d\n",
                         __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST));
            }
            qemu_frame_completed(s, 0);
        }
    }

    return NULL;
}

/*
 * MMIO handlers use stack-allocated jobs. This is safe because
 * AIO_WAIT_WHILE blocks until the worker completes the job,
 * keeping the stack frame alive for the job's entire lifetime.
 */
static uint64_t agfx_mmio_read(void *opaque, hwaddr offset, unsigned size)
{
    AppleGfxMLState *s = opaque;
    uint64_t value;

    /* Reference: mmioReadAtOffset is synchronous — no serial queue, no mutex.
     * Register values are atomic (event_stamps, display_irq) or aligned
     * uint32_t written under mmio_mutex (atomic loads on x86_64).
     * Synchronous reads avoid blocking behind waitStamps on the worker queue. */
    value = qmu_mmio_read(s->qmu_dev, (uint32_t)offset, size);
    if (s->debug_level >= 5) {
        trace_apple_gfx_ml_mmio_read(offset, value, size);
    }
    return value;
}

static void agfx_mmio_write(void *opaque, hwaddr offset,
                             uint64_t val, unsigned size)
{
    AppleGfxMLState *s = opaque;
    AppleGfxMLSessionJob job = {
        .state = s,
        .kind = AGFX_SESSION_JOB_MMIO_WRITE,
        .offset = offset,
        .value = val,
        .size = size,
        .completed = false,
        .heap_owned = false,
    };
    qatomic_set(&s->mmio_wait_active, 1);
    agfx_enqueue_session_job(s, &job);
    AIO_WAIT_WHILE(NULL, !qatomic_read(&job.completed));
    qatomic_set(&s->mmio_wait_active, 0);

    /* Reference scheduleFramePresents is owned by the wrapper plane, not by
     * the IOSurface map/unmap callbacks themselves. Mirror that ownership on
     * the IOSFC MMIO commit points: first MAP_ADDR starts the timer with an
     * immediate first fire, later MAP_ADDR commits merge one pending tick into
     * the already-armed source. */
    if (offset == PVG_REG_IOSFC_MAP_ADDR && val != 0) {
        qatomic_set(&s->iosfc_bootstrap_active, 1);
        agfx_schedule_frame_presents(s);
    } else if ((offset == PVG_REG_IOSFC_ENABLE && val == 0) ||
               offset == PVG_REG_IOSFC_UNMAP) {
        qatomic_set(&s->iosfc_bootstrap_active, 0);
        agfx_cancel_frame_presents(s);
    }

    if (s->debug_level >= 5) {
        trace_apple_gfx_ml_mmio_write(offset, val, size);
    }
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
 * Display Frame Timer (reference: scheduleFramePresents)
 *
 * Reference: IOSFC scheduling timer lives in the wrapper (apple-gfx.m),
 * NOT in PVG library. It fires at ~10Hz continuously, calling
 * encodeCurrentFrameToCommandBuffer. This provides the baseline
 * display frame rate independent of guest DT timing.
 *
 * Without this timer, display frames are only produced by
 * signalCurrentFrame from Transaction3, yielding ~1 frame/DT.
 * Reference achieves ~2.5 frames/DT because this timer adds
 * ~1.5 extra encodes between DTs.
 * ============================================================ */

/* ============================================================
 * Display Operations
 * ============================================================ */

static void agfx_gfx_update(void *opaque)
{
    AppleGfxMLState *s = opaque;
    int pending_frames = 0;

    if (!s) {
        return;
    }

    pending_frames = __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST);
    if (s->new_frame_ready) {
        /* Path 1: Frame ready — push to display, signal done */
        dpy_gfx_update_full(s->con);
        s->new_frame_ready = false;
        graphic_hw_update_done(s->con);
    } else if (pending_frames > 0) {
        /* Reference apple_gfx_fb_update_display defers while render work is
         * still in flight (pending_frames > 0), not only while a completed
         * frame payload is already queued on the wrapper side. */
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
 * Device Lifecycle
 * ============================================================ */

static void agfx_realize(PCIDevice *pci_dev, Error **errp)
{
    AppleGfxMLState *s = APPLE_GFX_ML(pci_dev);
    uint64_t vram_size = (uint64_t)s->vram_size_mb << 20;
    size_t initial_fb_size;
    
    agfx_log_init(s);
    agfx_log(s, "[apple-gfx-ml] Realizing device: %ux%u, VRAM=%uMB\n",
             s->display_width, s->display_height, s->vram_size_mb);
    
    /* OptionROM is handled via inherited 'romfile' property from PCIDevice.
     * Just like Apple does in apple-gfx-pci.m:
     *   pci->romfile = apple_gfx_pci_option_rom_path;
     * 
     * Usage: -device apple-gfx-ml,romfile=/path/to/AppleParavirtEFI.rom
     */
    if (pci_dev->romfile && pci_dev->romfile[0]) {
        agfx_log(s, "[apple-gfx-ml] OptionROM configured: %s\n", pci_dev->romfile);
    }
    
    /* Setup MSI - Apple style: no INTERRUPT_PIN, just msi_init */
    int msi_ret = msi_init(pci_dev, APPLE_GFX_ML_MSI_CAP_AUTO, 1, true, false, errp);
    if (msi_ret == 0) {
        s->msi_used = true;
        agfx_log(s, "[apple-gfx-ml] msi_init OK, 1 vector\n");
    } else {
        agfx_log(s, "[apple-gfx-ml] msi_init FAILED: %d\n", msi_ret);
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
    qemu_mutex_init(&s->mmio_job_mutex);
    qemu_mutex_init(&s->session_mutex);
    qemu_mutex_init(&s->render_mutex);
    qemu_mutex_init(&s->display_callback_mutex);
    qemu_mutex_init(&s->bootstrap_present_mutex);
    qemu_cond_init(&s->bootstrap_present_cond);
    qemu_sem_init(&s->display_callback_sem, 0);
    qemu_sem_init(&s->render_sem, 0);
    s->mmio_wait_active = 0;
    s->iosfc_bootstrap_active = 0;
    s->session_job_head = NULL;
    s->session_job_tail = NULL;
    s->render_worker_stop = false;
    s->display_render_requests = 0;
    s->display_callback_worker_stop = false;
    s->display_callback_head = NULL;
    s->display_callback_tail = NULL;
    s->bootstrap_present_worker_stop = false;
    s->bootstrap_present_timer.active = false;
    s->bootstrap_present_timer.next_fire_us = 0;
    s->bootstrap_present_source.pending = false;
    
    /* Allocate display buffer published to the QEMU surface. */
    initial_fb_size = (size_t)s->display_height * s->display_width * 4;
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
        .cursor_move = qemu_cursor_move,
        .cursor_show = qemu_cursor_show,
        .mode_change = qemu_mode_change,
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
        .frame_completed = qemu_frame_completed,
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
        .long_op_unlock = NULL,
        .long_op_lock = NULL,
        .long_op_ctx = NULL,
        .using_iosurface_mapper = 0,
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

    qmu_log_set_callback(agfx_qmu_log_callback, s);
    qmu_set_debug_level(s->qmu_dev, s->debug_level);

    /* Start wrapper-owned serial worker threads. */
    qemu_sem_init(&s->mmio_sem, 0);
    s->mmio_worker_stop = false;
    qemu_thread_create(&s->mmio_worker, "agfx-io",
                        agfx_mmio_worker_thread, s, QEMU_THREAD_JOINABLE);
    qemu_thread_create(&s->display_callback_worker, "agfx-display-cb",
                       agfx_display_callback_thread, s, QEMU_THREAD_JOINABLE);
    qemu_thread_create(&s->render_worker, "agfx-render",
                       agfx_render_worker_thread, s, QEMU_THREAD_JOINABLE);
    qemu_thread_create(&s->bootstrap_present_worker, "agfx-present",
                       agfx_bootstrap_present_thread, s, QEMU_THREAD_JOINABLE);

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
    
    agfx_log(s, "[apple-gfx-ml] Device realized successfully\n");
}

static void agfx_exit(PCIDevice *pci_dev)
{
    AppleGfxMLState *s = APPLE_GFX_ML(pci_dev);
    AppleGfxMLSessionJob *job;

    agfx_log(s, "[apple-gfx-ml] Device exit\n");

    /* Stop async MMIO worker thread */
    qatomic_set(&s->mmio_worker_stop, true);
    qemu_sem_post(&s->mmio_sem);  /* Wake to exit */
    qemu_thread_join(&s->mmio_worker);
    qemu_sem_destroy(&s->mmio_sem);

    qemu_mutex_lock(&s->bootstrap_present_mutex);
    s->bootstrap_present_worker_stop = true;
    agfx_cancel_frame_presents_locked(s);
    qemu_cond_signal(&s->bootstrap_present_cond);
    qemu_mutex_unlock(&s->bootstrap_present_mutex);
    qemu_thread_join(&s->bootstrap_present_worker);

    qatomic_set(&s->render_worker_stop, true);
    qemu_sem_post(&s->render_sem);
    qemu_thread_join(&s->render_worker);

    qemu_mutex_lock(&s->mmio_job_mutex);
    job = s->session_job_head;
    s->session_job_head = NULL;
    s->session_job_tail = NULL;
    qemu_mutex_unlock(&s->mmio_job_mutex);
    while (job) {
        AppleGfxMLSessionJob *next = job->next;
        if (job->heap_owned) {
            g_free(job);
        }
        job = next;
    }

    qemu_mutex_destroy(&s->mmio_job_mutex);
    qemu_mutex_destroy(&s->session_mutex);

    /* Destroy qmetal device (stops display thread) */
    if (s->qmu_dev) {
        qmu_destroy(s->qmu_dev);
        s->qmu_dev = NULL;
    }
    qemu_mutex_lock(&s->display_callback_mutex);
    s->display_callback_worker_stop = true;
    qemu_mutex_unlock(&s->display_callback_mutex);
    qemu_sem_post(&s->display_callback_sem);
    qemu_thread_join(&s->display_callback_worker);
    agfx_free_queued_display_callback_jobs(s);
    qemu_sem_destroy(&s->display_callback_sem);
    qemu_mutex_destroy(&s->display_callback_mutex);
    qemu_sem_destroy(&s->render_sem);
    qemu_mutex_destroy(&s->render_mutex);
    qemu_cond_destroy(&s->bootstrap_present_cond);
    qemu_mutex_destroy(&s->bootstrap_present_mutex);
    qmu_log_set_callback(NULL, NULL);
    agfx_log_stop(s);

    /* Cleanup framebuffers and mutex */
    qemu_mutex_lock(&s->frame_mutex);
    agfx_free_waiting_frame_completion_jobs(s);
    agfx_free_queued_frame_payloads(s);
    qemu_mutex_unlock(&s->frame_mutex);
    qemu_mutex_destroy(&s->frame_mutex);
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
    s->mmio_wait_active = 0;
    qemu_mutex_lock(&s->render_mutex);
    s->display_render_requests = 0;
    qemu_mutex_unlock(&s->render_mutex);
    qemu_mutex_lock(&s->bootstrap_present_mutex);
    agfx_cancel_frame_presents_locked(s);
    qemu_mutex_unlock(&s->bootstrap_present_mutex);
    qemu_mutex_lock(&s->frame_mutex);
    agfx_free_waiting_frame_completion_jobs(s);
    agfx_free_queued_frame_payloads(s);
    qemu_mutex_unlock(&s->frame_mutex);
    agfx_free_queued_display_callback_jobs(s);
    s->cursor_show = true;
    s->cursor_x = 0;
    s->cursor_y = 0;
    
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
    s->new_frame_ready = false;
    s->gfx_update_requested = false;
    s->mmio_wait_active = 0;
    s->session_job_head = NULL;
    s->session_job_tail = NULL;
    s->render_worker_stop = false;
    s->display_render_requests = 0;
    s->bootstrap_present_worker_stop = false;
    s->bootstrap_present_timer.active = false;
    s->bootstrap_present_timer.next_fire_us = 0;
    s->bootstrap_present_source.pending = false;
    s->pending_frames = 0;
    s->frame_completion_wait_head = NULL;
    s->frame_completion_wait_tail = NULL;
    s->frame_payload_head = NULL;
    s->frame_payload_tail = NULL;
    s->frame_payload_count = 0;
    s->fb_iosurface_pixel_format = 0x42475241u;
    s->fb_protection_requirements = 0;
    s->display_fb = NULL;
    s->cursor = NULL;
    s->cursor_show = true;
    s->cursor_x = 0;
    s->cursor_y = 0;
    s->log_writer_started = false;
    s->log_writer_stop = false;
    s->log_head = NULL;
    s->log_tail = NULL;
    s->log_depth = 0;
    s->log_dropped = 0;
    s->frame_completed_log_count = 0;
    s->new_frame_handler_log_count = 0;
    s->new_frame_signal_log_count = 0;
    s->render_worker_log_count = 0;
    s->bootstrap_present_log_count = 0;
    s->display_callback_worker_stop = false;
    s->display_callback_head = NULL;
    s->display_callback_tail = NULL;
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
