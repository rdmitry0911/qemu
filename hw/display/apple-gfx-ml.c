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
 * v96: Fixed critical display issue - present_frame now uses BH for
 *      thread-safe display updates from qmetal's pthread.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/aio.h"       /* for aio_bh_schedule_oneshot */
#include "qemu/aio-wait.h"
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

/* ============================================================
 * Memory Access Callbacks (QEMU → qmetal)
 * These match Apple's PGDeviceDescriptor callback interface
 * ============================================================ */

static void *qemu_map_gpa(void *ctx, uint64_t gpa, size_t size, int writable)
{
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
    return ptr ? (ptr + xlat) : NULL;
}

static void qemu_unmap_gpa(void *ctx, void *hva, size_t size, int dirty)
{
    /* Memory regions are reference-counted, no explicit unmap needed */
    (void)ctx; (void)hva; (void)size; (void)dirty;
}

static int qemu_read_memory(void *ctx, uint64_t gpa, void *buf, size_t size)
{
    MemTxResult r = address_space_read(&address_space_memory, gpa,
                                        MEMTXATTRS_UNSPECIFIED, buf, size);
    return (r == MEMTX_OK) ? 0 : -1;
}

static int qemu_write_memory(void *ctx, uint64_t gpa, const void *buf, size_t size)
{
    MemTxResult r = address_space_write(&address_space_memory, gpa,
                                         MEMTXATTRS_UNSPECIFIED, buf, size);
    return (r == MEMTX_OK) ? 0 : -1;
}

/* ============================================================
 * v96: Thread-safe Interrupt Delivery (Apple style)
 * IRQ is raised from qmetal pthread, must be delivered via BH
 * ============================================================ */

typedef struct AppleGfxMLInterruptJob {
    PCIDevice *device;
    uint32_t vector;
} AppleGfxMLInterruptJob;

static void apple_gfx_ml_raise_interrupt_bh(void *opaque)
{
    AppleGfxMLInterruptJob *job = opaque;
    
    static int irq_count = 0;
    irq_count++;
    
    if (irq_count <= 10 || (irq_count % 60) == 0) {
        qemu_log("[apple-gfx-ml] raise_irq #%d: msi_enabled=%d\n", 
                 irq_count, msi_enabled(job->device));
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
 * v96: Thread-safe Display Updates (Apple style)
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

    /* Get pending frame parameters under lock */
    pthread_mutex_lock(&s->frame_mutex);
    if (!s->frame_pending) {
        pthread_mutex_unlock(&s->frame_mutex);
        return;
    }

    width = s->pending_width;
    height = s->pending_height;
    stride = s->pending_stride;
    size = (size_t)height * stride;
    s->frame_pending = false;

    /* Reallocate display buffer if frame size increased */
    if (size > s->display_fb_size) {
        uint8_t *new_fb = g_realloc(s->display_fb, size);
        if (new_fb) {
            s->display_fb = new_fb;
            s->display_fb_size = size;
            qemu_log("[apple-gfx-ml] display_fb reallocated: %zu bytes\n", size);
        } else {
            qemu_log("[apple-gfx-ml] display_fb realloc failed!\n");
            pthread_mutex_unlock(&s->frame_mutex);
            return;
        }
    }

    /* Copy from staging to display buffer */
    if (s->staging_fb && s->display_fb) {
        memcpy(s->display_fb, s->staging_fb, size);
    }
    pthread_mutex_unlock(&s->frame_mutex);

    /* Update display state */
    s->fb_width = width;
    s->fb_height = height;
    s->fb_stride = stride;
    s->has_rendered_frame = true;
    s->frame_count++;

    /* Log periodically */
    if (s->frame_count <= 10 || (s->frame_count % 60) == 0) {
        qemu_log("[apple-gfx-ml] present_frame_bh #%lu: %ux%u stride=%u\n",
                 (unsigned long)s->frame_count, width, height, stride);
    }

    /* Update QEMU display - now safe, we're in main loop! */
    if (s->con && s->display_fb) {
        DisplaySurface *surface = qemu_create_displaysurface_from(
            width, height, PIXMAN_x8r8g8b8, stride, s->display_fb);
        dpy_gfx_replace_surface(s->con, surface);
        dpy_gfx_update_full(s->con);
    }
}

static void qemu_present_frame(void *ctx, const void *pixels,
                                uint32_t width, uint32_t height, uint32_t stride)
{
    AppleGfxMLState *s = ctx;
    size_t size = (size_t)height * stride;
    
    /* Log from pthread (before scheduling BH) */
    s->present_count++;
    if (s->present_count <= 10 || (s->present_count % 60) == 0) {
        qemu_log("[apple-gfx-ml] present_frame #%lu: %ux%u stride=%u (from pthread)\n",
                 (unsigned long)s->present_count, width, height, stride);
    }
    
    /* Reallocate staging buffer if needed */
    pthread_mutex_lock(&s->frame_mutex);
    if (!s->staging_fb || s->staging_fb_size < size) {
        g_free(s->staging_fb);
        s->staging_fb = g_malloc(size);
        s->staging_fb_size = s->staging_fb ? size : 0;
        if (!s->staging_fb) {
            pthread_mutex_unlock(&s->frame_mutex);
            qemu_log("[apple-gfx-ml] present_frame: staging alloc failed!\n");
            return;
        }
    }
    
    /* Copy pixels to staging buffer */
    memcpy(s->staging_fb, pixels, size);
    
    /* Store frame parameters */
    s->pending_width = width;
    s->pending_height = height;
    s->pending_stride = stride;
    s->frame_pending = true;
    pthread_mutex_unlock(&s->frame_mutex);
    
    /* Schedule BH to update display in QEMU main loop */
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            apple_gfx_ml_present_frame_bh, s);
}

/* v83: Display refresh is handled by qmetal library's internal thread.
 * No timer needed in QEMU driver - we just receive present_frame callbacks.
 */

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

static uint64_t agfx_mmio_read(void *opaque, hwaddr offset, unsigned size)
{
    AppleGfxMLState *s = opaque;
    AppleGfxMLMMIOJob job = {
        .state = s, .offset = offset, .size = size,
        .func = agfx_do_read, .completed = false,
    };
    qatomic_set(&s->pending_job, &job);
    qemu_sem_post(&s->mmio_sem);
    AIO_WAIT_WHILE(NULL, !qatomic_read(&job.completed));
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
    qatomic_set(&s->pending_job, &job);
    qemu_sem_post(&s->mmio_sem);
    AIO_WAIT_WHILE(NULL, !qatomic_read(&job.completed));
    trace_apple_gfx_ml_mmio_write(offset, val, size);
}

static const MemoryRegionOps agfx_mmio_ops = {
    .read = agfx_mmio_read,
    .write = agfx_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
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
    /* Display updates are pushed from qmetal via present_frame callback + BH */
}

static const GraphicHwOps agfx_gfx_ops = {
    .gfx_update = agfx_gfx_update,
};

/* ============================================================
 * Long Operation Hooks (release BQL during shader compilation)
 *
 * qmetal calls these from the worker thread during slow ops
 * (AIR->SPIR-V translation, vkCreateGraphicsPipelines).
 * mmio_mutex is released by qmu_session.cpp wrappers before
 * these are called, allowing other channels' KICKs to proceed.
 * We additionally kick AIO to let the QEMU main loop process
 * pending BHs (interrupts, disk I/O).
 * ============================================================ */

static void agfx_long_op_unlock(void *ctx)
{
    (void)ctx;
    aio_wait_kick();
}

static void agfx_long_op_lock(void *ctx)
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
    int msi_ret = msi_init(pci_dev, APPLE_GFX_ML_MSI_CAP_OFFSET, 1, true, false, errp);
    if (msi_ret == 0) {
        s->msi_used = true;
        qemu_log("[apple-gfx-ml] msi_init OK, 1 vector\n");
    } else {
        qemu_log("[apple-gfx-ml] msi_init FAILED: %d\n", msi_ret);
    }
    
    /* Setup MMIO BAR (BAR0) - Apple only uses this one BAR */
    memory_region_init_io(&s->mmio, OBJECT(s), &agfx_mmio_ops, s,
                          "apple-gfx-mmio", APPLE_GFX_ML_MMIO_SIZE);
    s->mmio.disable_reentrancy_guard = true;
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
    
    /* Setup host VRAM (not a PCI BAR - internal storage) */
    memory_region_init_ram(&s->host_vram, OBJECT(s), "apple-gfx-vram",
                           vram_size, errp);
    if (*errp) {
        return;
    }
    
    /* v96: Initialize frame mutex for thread-safe display updates */
    pthread_mutex_init(&s->frame_mutex, NULL);
    
    /* v96: Allocate double-buffered framebuffers */
    initial_fb_size = (size_t)s->display_height * s->display_width * 4;
    s->staging_fb = g_malloc0(initial_fb_size);
    s->staging_fb_size = initial_fb_size;
    s->display_fb = g_malloc0(initial_fb_size);
    s->display_fb_size = initial_fb_size;
    
    if (!s->staging_fb || !s->display_fb) {
        error_setg(errp, "Failed to allocate framebuffers");
        return;
    }
    
    /* Create qmetal device with callbacks */
    qmu_extended_callbacks qmu_callbacks = {
        .user_ctx = s,
        .map_gpa = qemu_map_gpa,
        .unmap_gpa = qemu_unmap_gpa,
        .read_memory = qemu_read_memory,
        .write_memory = qemu_write_memory,
        .raise_irq = qemu_raise_irq,
        .present_frame = qemu_present_frame,
        .read_vram = qemu_read_vram,
    };

    qmu_extended_config qmu_config = {
        .struct_size = sizeof(qmu_extended_config),
        .ram_base = 0,
        .ram_size = 0,  /* No restriction */
        .max_protocol_version = 6,
        .iosfc_caps = 0x03,
        .display_port_count = 1,
        .mode_count = 3,
        .mode_data = {
            (s->display_height << 16) | s->display_width,  /* Mode 0 */
            (1080 << 16) | 1440,                            /* Mode 1 */
            (1024 << 16) | 1280,                            /* Mode 2 */
        },
        .vram_size = vram_size,
        .direct_scanout = s->direct_scanout,
        .vsync_enabled = s->vsync_enabled,
        .spirv_cache_dir = s->spirv_cache_dir,
        .long_op_unlock = agfx_long_op_unlock,
        .long_op_lock = agfx_long_op_lock,
        .long_op_ctx = s,
    };

    s->qmu_dev = qmu_create_extended(&qmu_config, &qmu_callbacks);
    if (!s->qmu_dev) {
        error_setg(errp, "Failed to create qmetal device");
        return;
    }

    qmu_set_debug_level(s->qmu_dev, s->debug_level);

    /* Start async MMIO worker thread (replaces GCD dispatch_async_f) */
    qemu_sem_init(&s->mmio_sem, 0);
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
    
    qemu_log("[apple-gfx-ml] Device realized successfully (qmetal unified, v96: thread-safe display)\n");
}

static void agfx_exit(PCIDevice *pci_dev)
{
    AppleGfxMLState *s = APPLE_GFX_ML(pci_dev);

    /* Stop async MMIO worker thread */
    qatomic_set(&s->mmio_worker_stop, true);
    qemu_sem_post(&s->mmio_sem);  /* Wake to exit */
    qemu_thread_join(&s->mmio_worker);
    qemu_sem_destroy(&s->mmio_sem);

    /* Destroy qmetal device (stops display thread) */
    if (s->qmu_dev) {
        qmu_destroy(s->qmu_dev);
        s->qmu_dev = NULL;
    }

    /* v96: Cleanup framebuffers and mutex */
    pthread_mutex_destroy(&s->frame_mutex);
    g_free(s->staging_fb);
    s->staging_fb = NULL;
    g_free(s->display_fb);
    s->display_fb = NULL;
}

static void agfx_reset(Object *obj, ResetType type)
{
    AppleGfxMLState *s = APPLE_GFX_ML(obj);
    
    s->frame_count = 0;
    s->present_count = 0;
    s->display_enabled = false;
    s->has_rendered_frame = false;
    s->frame_pending = false;
    
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
    
    /* v96: Initialize frame state */
    s->frame_pending = false;
    s->staging_fb = NULL;
    s->display_fb = NULL;
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
