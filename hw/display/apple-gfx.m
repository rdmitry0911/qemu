/*
 * QEMU Apple ParavirtualizedGraphics.framework device
 *
 * Copyright © 2023 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * ParavirtualizedGraphics.framework is a set of libraries that macOS provides
 * which implements 3d graphics passthrough to the host as well as a
 * proprietary guest communication channel to drive it. This device model
 * implements support to drive that library from within QEMU.
 */

#include "qemu/osdep.h"
#include "qemu/lockable.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qapi/visitor.h"
#include "qapi/error.h"
#include "qemu/aio-wait.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "system/system.h"
#include "migration/blocker.h"
#include "ui/console.h"
#include "apple-gfx.h"
#include "trace.h"

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <mach/mach.h>
#include <mach/mach_vm.h>
#include <dispatch/dispatch.h>

#import <ParavirtualizedGraphics/ParavirtualizedGraphics.h>

static const AppleGFXDisplayMode apple_gfx_default_modes[] = {
    { 1920, 1080, 60 },
    { 1440, 1080, 60 },
    { 1280, 1024, 60 },
};

static Error *apple_gfx_mig_blocker;
static uint32_t next_pgdisplay_serial_num = 1;

static dispatch_queue_t get_background_queue(void)
{
    return dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
}

/* ------ PVG capture (commands, metallib, resources, frames) ------ */

#define PVG_CAPTURE_DIR_ENV "PVG_CAPTURE_DIR"
#define PVG_CAPTURE_FRAMES_ENV "PVG_CAPTURE_FRAMES"

typedef struct PvgCaptureState {
    bool enabled;
    bool active;
    bool capture_frames;
    char dir[PATH_MAX];
    FILE *events;
    FILE *manifest;
    uint64_t seq;
    uint64_t frame_index;
} PvgCaptureState;

static PvgCaptureState g_pvg_capture;
static QemuMutex g_pvg_capture_mutex;
static Notifier g_pvg_capture_exit_notifier;
static uint32_t g_pvg_capture_task_next_id = 1;

/* ------ PVG sync logging (PVG_SYNC_LOG=1/2/3) ------ */

#define PVG_SYNC_LOG_ENV "PVG_SYNC_LOG"

typedef struct PvgSyncState {
    int level;
    uint64_t boot_abs;  /* mach_absolute_time() at init */
    mach_timebase_info_data_t timebase;

    /* Counters */
    uint64_t kick_count;
    uint64_t frame_count;
    uint64_t encode_count;
    uint64_t completion_count;
    uint64_t dma_read_count;
    uint64_t irq_count;

    /* Root FIFO ring buffer tracking */
    uint64_t fifo_pfn;        /* From MMIO 0x1030 */
    uint32_t fifo_offset;     /* From MMIO 0x1010 */
    uint32_t fifo_ring_size;  /* From MMIO 0x1004 */
    uint32_t fifo_write;      /* From MMIO 0x1008 */
    uint32_t fifo_sniff_ptr;  /* Our read position (monotonic like fifo_write) */
} PvgSyncState;

static PvgSyncState g_pvg_sync;

void pvg_sync_init(void)
{
    const char *env = getenv(PVG_SYNC_LOG_ENV);

    memset(&g_pvg_sync, 0, sizeof(g_pvg_sync));
    if (!env || env[0] == '\0' || env[0] == '0') {
        return;
    }
    g_pvg_sync.level = atoi(env);
    if (g_pvg_sync.level < 1) {
        g_pvg_sync.level = 0;
        return;
    }
    if (g_pvg_sync.level > 3) {
        g_pvg_sync.level = 3;
    }
    mach_timebase_info(&g_pvg_sync.timebase);
    g_pvg_sync.boot_abs = mach_absolute_time();
    fprintf(stderr, "[PVG] sync logging enabled at level %d\n",
            g_pvg_sync.level);
}

void pvg_sync_log(int min_level, const char *fmt, ...)
{
    va_list ap;
    uint64_t now, elapsed_ns;
    double elapsed_ms;

    if (g_pvg_sync.level < min_level) {
        return;
    }

    now = mach_absolute_time();
    elapsed_ns = (now - g_pvg_sync.boot_abs) *
                 g_pvg_sync.timebase.numer / g_pvg_sync.timebase.denom;
    elapsed_ms = (double)elapsed_ns / 1e6;

    fprintf(stderr, "[PVG %10.3f] ", elapsed_ms);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

static const char *pvg_reg_name(uint64_t offset)
{
    switch (offset) {
    case 0x1000: return "FIFO_ENABLE";
    case 0x1004: return "FIFO_SIZE";
    case 0x1008: return "FIFO_WRITE";
    case 0x100c: return "FIFO_READ";
    case 0x1010: return "FIFO_OFFSET";
    case 0x1014: return "DISPLAY_IRQ";
    case 0x1018: return "EVENT_STAMPS";
    case 0x101c: return "SHARED_STATE_PFN";
    case 0x1020: return "KICK";
    case 0x1024: return "PENDING_COMP";
    case 0x1028: return "TRANSACTION_ID";
    case 0x102c: return "FAULT_STATUS";
    case 0x1030: return "FIFO_PFN";
    case 0x1034: return "PROTOCOL_VERSION";
    case 0x1200: return "IOSFC_ENABLE";
    case 0x1204: return "IOSFC_SIZE";
    case 0x1208: return "IOSFC_WRITE";
    case 0x120c: return "IOSFC_READ";
    case 0x1210: return "IOSFC_OFFSET";
    case 0x1214: return "IOSFC_14";
    case 0x1218: return "IOSFC_18";
    case 0x121c: return "IOSFC_1C";
    case 0x1220: return "IOSFC_KICK";
    case 0x1224: return "IOSFC_24";
    case 0x1228: return "IOSFC_28";
    case 0x122c: return "IOSFC_2C";
    default:     return NULL;
    }
}

static const char *pvg_fifo_opcode_name(uint16_t opcode)
{
    switch (opcode) {
    case 0x01: return "DISPLAY_SET_SHARED_STATE";
    case 0x02: return "DISPLAY_ACK";
    case 0x03: return "NOP";
    case 0x04: return "DEBUG";
    case 0x05: return "DISPLAY_CURSOR_GLYPH";
    case 0x06: return "DISPLAY_CURSOR_SHOW";
    case 0x07: return "DISPLAY_TRANSACTION";
    case 0x1E: return "DISPLAY_FLUSH";
    case 0x20: return "DELETE_TASK";
    case 0x22: return "UNMAP_MEMORY";
    case 0x25: return "DELETE_RESOURCE";
    case 0x26: return "EXEC_INDIRECT";
    case 0x28: return "DELETE_OBJECT";
    case 0x2B: return "EXEC_INDIRECT3";
    case 0x30: return "DEFINE_CHILD_FIFO";
    case 0x31: return "DELETE_CHILD_FIFO";
    case 0x33: return "SET_OBJECT_LIST";
    case 0x34: return "INVALIDATE_RESOURCES";
    case 0x35: return "SYNCHRONIZE_RESOURCES";
    case 0x36: return "DISCARD_RESOURCES";
    case 0x38: return "DEFINE_TASK";
    case 0x39: return "MAP_MEMORY";
    case 0x3A: return "GET_DEVICE_INFO";
    case 0x3B: return "GET_COMPUTE_INFO";
    case 0x3C: return "REPLACE_PHYSICAL";
    case 0x97: return "UPDATE_FENCE";
    case 0x98: return "WAIT_FENCE";
    default:   return NULL;
    }
}

/* GPU channel opcodes (different namespace from root FIFO) */
static const char * __attribute__((unused))
pvg_gpu_opcode_name(uint16_t opcode)
{
    switch (opcode) {
    case 0x02: return "GPU_EXEC_INDIRECT2";
    case 0x22: return "GPU_UNMAP_MEMORY";
    case 0x25: return "GPU_DELETE_RESOURCE";
    case 0x28: return "GPU_DELETE_OBJECT";
    case 0x35: return "GPU_SYNCHRONIZE_RESOURCES";
    case 0x36: return "GPU_DISCARD_RESOURCES";
    case 0x37: return "GPU_EXEC_INDIRECT3";
    case 0x39: return "GPU_MAP_MEMORY2";
    case 0x3B: return "GPU_GET_COMPUTE_INFO";
    case 0x3C: return "GPU_REPLACE_PHYSICAL";
    case 0x45: return "GPU_INFO_INDIRECT";
    default:   return NULL;
    }
}

/*
 * Sniff root FIFO ring buffer from guest memory.
 * Called on FIFO_WRITE with BQL held — safe to use dma_memory_read.
 *
 * Ring buffer format: 12-byte PVGRootCmdHeader per command
 *   {u16 opcode, u16 flags, u32 total_size, u32 stamp_value}
 * total_size includes the 12-byte header.
 *
 * fifo_write/fifo_sniff_ptr are monotonic u32 counters.
 * Ring offset = counter % fifo_ring_size.
 */
static void pvg_sync_sniff_root_fifo(void)
{
    uint32_t bytes_to_process;
    uint32_t ring_size;
    uint64_t ring_base_gpa;
    uint32_t offset = 0;
    int cmd_count = 0;

    if (g_pvg_sync.level < 1) {
        return;
    }
    ring_size = g_pvg_sync.fifo_ring_size;
    if (ring_size == 0 || g_pvg_sync.fifo_pfn == 0) {
        return;
    }

    bytes_to_process = g_pvg_sync.fifo_write - g_pvg_sync.fifo_sniff_ptr;
    if (bytes_to_process == 0 || bytes_to_process > ring_size) {
        /* Nothing new, or counter wrapped past ring_size (shouldn't happen) */
        g_pvg_sync.fifo_sniff_ptr = g_pvg_sync.fifo_write;
        return;
    }

    ring_base_gpa = (g_pvg_sync.fifo_pfn << 12) + g_pvg_sync.fifo_offset;

    while (offset < bytes_to_process && cmd_count < 200) {
        uint32_t ring_off;
        uint8_t hdr_buf[12];
        uint16_t opcode;
        uint16_t flags;
        uint32_t total_size;
        uint32_t stamp;
        const char *name;
        MemTxResult r;

        /* Read 12-byte header, handling ring wrap */
        ring_off = (g_pvg_sync.fifo_sniff_ptr + offset) % ring_size;

        if (ring_off + 12 <= ring_size) {
            /* No wrap */
            r = dma_memory_read(&address_space_memory,
                                ring_base_gpa + ring_off,
                                hdr_buf, 12, MEMTXATTRS_UNSPECIFIED);
        } else {
            /* Wraps around ring boundary */
            uint32_t first = ring_size - ring_off;
            r = dma_memory_read(&address_space_memory,
                                ring_base_gpa + ring_off,
                                hdr_buf, first, MEMTXATTRS_UNSPECIFIED);
            if (r == MEMTX_OK) {
                r = dma_memory_read(&address_space_memory,
                                    ring_base_gpa,
                                    hdr_buf + first, 12 - first,
                                    MEMTXATTRS_UNSPECIFIED);
            }
        }

        if (r != MEMTX_OK) {
            pvg_sync_log(1, "FIFO_SNIFF: DMA read failed at ring_off=%u\n",
                         ring_off);
            break;
        }

        memcpy(&opcode, hdr_buf + 0, 2);
        memcpy(&flags, hdr_buf + 2, 2);
        memcpy(&total_size, hdr_buf + 4, 4);
        memcpy(&stamp, hdr_buf + 8, 4);

        if (total_size < 12 || total_size > ring_size) {
            pvg_sync_log(1, "FIFO_SNIFF: bad total_size=%u at offset=%u, "
                         "stopping\n", total_size, offset);
            break;
        }

        name = pvg_fifo_opcode_name(opcode);
        pvg_sync_log(1, "FIFO_CMD: %s (0x%02x) size=%u flags=0x%x "
                     "stamp=%u\n",
                     name ? name : "UNKNOWN", opcode, total_size,
                     flags, stamp);

        offset += total_size;
        cmd_count++;
    }

    g_pvg_sync.fifo_sniff_ptr += offset;
}

/*
 * Sniff child FIFO (GPU channels 1-5) ring buffer.
 * Child FIFO pointers are in shared state at:
 *   SHARED_STATE + 0x400 + (channel_id - 1) * 0x14
 * Layout (5 u32s): {pfn, offset, ring_size, write_ptr, read_ptr}
 */
/*
 * Sniff child FIFO (GPU channels 1-5) ring buffer.
 * Child FIFO ring buffer pointers are in shared state at:
 *   SHARED_STATE + 0x400 + (channel_id - 1) * 0x14
 * Layout (5 u32s): {pfn, offset, ring_size, write_ptr, read_ptr}
 *
 * Currently logs the KICK event. Full sniffing requires tracking
 * shared_state_pfn (MMIO 0x101c) — planned follow-up.
 */
static void pvg_sync_sniff_child_fifo(uint32_t channel_id)
{
    if (g_pvg_sync.level < 2 || channel_id < 1 || channel_id > 5) {
        return;
    }
    pvg_sync_log(2, "CHILD_FIFO_KICK: channel=%u\n", channel_id);
}

typedef struct PvgSceneZone {
    const char *name;
    float x0;
    float y0;
    float x1;
    float y1;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t tol;
} PvgSceneZone;

/* Zones sampled from /srv/project/reference/tahoe_lake_screen.png */
static const PvgSceneZone pvg_scene_zones[] = {
    { "sky", 0.418f, 0.170f, 0.538f, 0.250f,  87, 165, 215, 30 },
    { "mountains", 0.405f, 0.450f, 0.545f, 0.550f,  97, 123, 139, 30 },
    { "pine", 0.880f, 0.503f, 0.980f, 0.623f,  77, 105,  99, 30 },
    { "rocks", 0.547f, 0.600f, 0.687f, 0.700f, 107, 127, 123, 30 },
};

static const size_t pvg_scene_zone_count =
    sizeof(pvg_scene_zones) / sizeof(pvg_scene_zones[0]);

static void pvg_capture_log(const char *fmt, ...)
{
    va_list ap;

    if (!g_pvg_capture.events) {
        return;
    }

    va_start(ap, fmt);
    vfprintf(g_pvg_capture.events, fmt, ap);
    va_end(ap);
    fflush(g_pvg_capture.events);
}

static bool pvg_capture_mkdir(const char *dir, const char *name)
{
    char path[PATH_MAX];
    int rc;

    if (snprintf(path, sizeof(path), "%s/%s", dir, name) >= (int)sizeof(path)) {
        return false;
    }

    rc = mkdir(path, 0755);
    if (rc != 0 && errno != EEXIST) {
        return false;
    }
    return true;
}

static bool pvg_capture_is_metallib(const uint8_t *data, size_t len)
{
    return len >= 4 && memcmp(data, "MTLB", 4) == 0;
}

static bool pvg_capture_is_cmd_buffer(const uint8_t *data, size_t len,
                                      uint32_t *enc_type, uint32_t *buf_len)
{
    uint32_t enc;
    uint32_t total;

    if (len < 8) {
        return false;
    }

    enc = *(const uint32_t *)(data + 0);
    total = *(const uint32_t *)(data + 4);
    if (enc != 0 && enc != 1 && enc != 2 && enc != 4) {
        return false;
    }
    if (total < 8 || total > len) {
        return false;
    }

    if (enc_type) {
        *enc_type = enc;
    }
    if (buf_len) {
        *buf_len = total;
    }
    return true;
}

static bool pvg_capture_write_blob(const char *subdir, uint64_t id,
                                   const void *data, size_t len,
                                   char *out_rel, size_t out_rel_len)
{
    char path[PATH_MAX];
    FILE *f;

    if (snprintf(path, sizeof(path), "%s/%s/%06" PRIu64 ".bin",
                 g_pvg_capture.dir, subdir, id) >= (int)sizeof(path)) {
        return false;
    }

    f = fopen(path, "wb");
    if (!f) {
        return false;
    }
    if (len > 0 && fwrite(data, 1, len, f) != len) {
        fclose(f);
        return false;
    }
    fclose(f);

    if (out_rel && out_rel_len > 0) {
        snprintf(out_rel, out_rel_len, "%s/%06" PRIu64 ".bin", subdir, id);
    }
    return true;
}

static bool pvg_capture_write_frame(uint64_t id, const void *data, size_t len,
                                    char *out_rel, size_t out_rel_len)
{
    char path[PATH_MAX];
    FILE *f;

    if (snprintf(path, sizeof(path), "%s/frames/%06" PRIu64 ".bgra",
                 g_pvg_capture.dir, id) >= (int)sizeof(path)) {
        return false;
    }

    f = fopen(path, "wb");
    if (!f) {
        return false;
    }
    if (len > 0 && fwrite(data, 1, len, f) != len) {
        fclose(f);
        return false;
    }
    fclose(f);

    if (out_rel && out_rel_len > 0) {
        snprintf(out_rel, out_rel_len, "frames/%06" PRIu64 ".bgra", id);
    }
    return true;
}

static void pvg_capture_shutdown(void)
{
    qemu_mutex_lock(&g_pvg_capture_mutex);
    if (!g_pvg_capture.enabled && !g_pvg_capture.events &&
        !g_pvg_capture.manifest) {
        qemu_mutex_unlock(&g_pvg_capture_mutex);
        return;
    }

    if (g_pvg_capture.manifest) {
        fprintf(g_pvg_capture.manifest, "end_time=%ld\n", (long)time(NULL));
        fclose(g_pvg_capture.manifest);
        g_pvg_capture.manifest = NULL;
    }

    if (g_pvg_capture.events) {
        pvg_capture_log("event=shutdown\n");
        fclose(g_pvg_capture.events);
        g_pvg_capture.events = NULL;
    }

    g_pvg_capture.active = false;
    g_pvg_capture.enabled = false;
    qemu_mutex_unlock(&g_pvg_capture_mutex);
}

static void pvg_capture_exit_notify(Notifier *n, void *data)
{
    pvg_capture_shutdown();
}

static void pvg_capture_init(void)
{
    const char *dir = getenv(PVG_CAPTURE_DIR_ENV);
    const char *frames = getenv(PVG_CAPTURE_FRAMES_ENV);
    char path[PATH_MAX];

    if (!dir || dir[0] == '\0') {
        return;
    }

    if (g_pvg_capture.enabled) {
        return;
    }

    memset(&g_pvg_capture, 0, sizeof(g_pvg_capture));
    qemu_mutex_init(&g_pvg_capture_mutex);
    g_pvg_capture.capture_frames =
        (frames && (frames[0] == '1' || frames[0] == 'y' || frames[0] == 'Y'));

    if (snprintf(g_pvg_capture.dir, sizeof(g_pvg_capture.dir), "%s", dir) >=
        (int)sizeof(g_pvg_capture.dir)) {
        return;
    }

    if (mkdir(g_pvg_capture.dir, 0755) != 0 && errno != EEXIST) {
        return;
    }
    if (!pvg_capture_mkdir(g_pvg_capture.dir, "cmd") ||
        !pvg_capture_mkdir(g_pvg_capture.dir, "mem-read") ||
        !pvg_capture_mkdir(g_pvg_capture.dir, "map") ||
        !pvg_capture_mkdir(g_pvg_capture.dir, "unmap") ||
        !pvg_capture_mkdir(g_pvg_capture.dir, "frames") ||
        !pvg_capture_mkdir(g_pvg_capture.dir, "metallib")) {
        return;
    }

    if (snprintf(path, sizeof(path), "%s/events.log", g_pvg_capture.dir) <
        (int)sizeof(path)) {
        g_pvg_capture.events = fopen(path, "a");
    }
    if (snprintf(path, sizeof(path), "%s/manifest.txt", g_pvg_capture.dir) <
        (int)sizeof(path)) {
        g_pvg_capture.manifest = fopen(path, "a");
    }

    if (!g_pvg_capture.events || !g_pvg_capture.manifest) {
        pvg_capture_shutdown();
        return;
    }

    fprintf(g_pvg_capture.manifest, "version=1\n");
    fprintf(g_pvg_capture.manifest, "start_time=%ld\n", (long)time(NULL));
    fprintf(g_pvg_capture.manifest, "capture_frames=%d\n",
            g_pvg_capture.capture_frames ? 1 : 0);
    for (size_t i = 0; i < pvg_scene_zone_count; i++) {
        const PvgSceneZone *zone = &pvg_scene_zones[i];
        fprintf(g_pvg_capture.manifest,
                "zone.%s=%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u\n",
                zone->name, zone->x0, zone->y0, zone->x1, zone->y1,
                zone->r, zone->g, zone->b, zone->tol);
    }
    fflush(g_pvg_capture.manifest);

    g_pvg_capture.enabled = true;
    g_pvg_capture.active = true;
    pvg_capture_log("event=init dir=%s\n", g_pvg_capture.dir);

    g_pvg_capture_exit_notifier.notify = pvg_capture_exit_notify;
    qemu_add_exit_notifier(&g_pvg_capture_exit_notifier);
}

static void pvg_capture_read_memory(hwaddr phys, uint64_t len,
                                    const void *data)
{
    uint64_t id;
    char rel[64];
    uint32_t enc_type = 0;
    uint32_t buf_len = 0;
    const uint8_t *bytes = data;
    const char *dir = "mem-read";
    const char *kind = "mem";

    if (!g_pvg_capture.enabled || !data || len == 0) {
        return;
    }

    if (pvg_capture_is_cmd_buffer(bytes, (size_t)len, &enc_type, &buf_len)) {
        dir = "cmd";
        kind = "cmd";
    } else if (pvg_capture_is_metallib(bytes, (size_t)len)) {
        dir = "metallib";
        kind = "metallib";
    }

    qemu_mutex_lock(&g_pvg_capture_mutex);
    id = ++g_pvg_capture.seq;
    if (pvg_capture_write_blob(dir, id, data, (size_t)len, rel, sizeof(rel))) {
        pvg_capture_log(
            "event=read kind=%s phys=0x%" PRIx64 " len=%" PRIu64
            " file=%s enc=%u buf_len=%u\n",
            kind, (uint64_t)phys, (uint64_t)len, rel, enc_type, buf_len);
    }
    qemu_mutex_unlock(&g_pvg_capture_mutex);
}

static bool pvg_capture_pixel_to_rgb(PixelFormat *pf, uint32_t pixel,
                                     uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint32_t rv;
    uint32_t gv;
    uint32_t bv;

    if (!pf->rmask || !pf->gmask || !pf->bmask) {
        return false;
    }

    rv = (pixel & pf->rmask) >> pf->rshift;
    gv = (pixel & pf->gmask) >> pf->gshift;
    bv = (pixel & pf->bmask) >> pf->bshift;

    if (pf->rmax > 0) {
        rv = (rv * 255) / pf->rmax;
    }
    if (pf->gmax > 0) {
        gv = (gv * 255) / pf->gmax;
    }
    if (pf->bmax > 0) {
        bv = (bv * 255) / pf->bmax;
    }

    *r = (uint8_t)rv;
    *g = (uint8_t)gv;
    *b = (uint8_t)bv;
    return true;
}

static bool pvg_capture_avg_zone(DisplaySurface *surface,
                                 const PvgSceneZone *zone,
                                 uint8_t *out_r, uint8_t *out_g, uint8_t *out_b)
{
    PixelFormat pf;
    uint8_t *data;
    int width;
    int height;
    int bytes_per_pixel;
    size_t stride;
    int x0;
    int y0;
    int x1;
    int y1;
    int zone_w;
    int zone_h;
    int step;
    uint64_t sum_r = 0;
    uint64_t sum_g = 0;
    uint64_t sum_b = 0;
    uint64_t count = 0;

    if (!surface || !zone || !out_r || !out_g || !out_b) {
        return false;
    }

    width = surface_width(surface);
    height = surface_height(surface);
    bytes_per_pixel = surface_bytes_per_pixel(surface);
    stride = (size_t)surface_stride(surface);
    data = surface_data(surface);
    if (!data || width <= 0 || height <= 0 || bytes_per_pixel < 4) {
        return false;
    }

    x0 = (int)(zone->x0 * width);
    y0 = (int)(zone->y0 * height);
    x1 = (int)(zone->x1 * width);
    y1 = (int)(zone->y1 * height);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > width) x1 = width;
    if (y1 > height) y1 = height;
    if (x1 <= x0 || y1 <= y0) {
        return false;
    }

    zone_w = x1 - x0;
    zone_h = y1 - y0;
    step = zone_w < zone_h ? zone_w / 64 : zone_h / 64;
    if (step < 1) {
        step = 1;
    }

    pf = qemu_pixelformat_from_pixman(surface_format(surface));

    for (int y = y0; y < y1; y += step) {
        uint8_t *row = data + (size_t)y * stride;
        for (int x = x0; x < x1; x += step) {
            uint32_t pixel;
            uint8_t r, g, b;
            memcpy(&pixel, row + (size_t)x * bytes_per_pixel, sizeof(pixel));
            if (!pvg_capture_pixel_to_rgb(&pf, pixel, &r, &g, &b)) {
                return false;
            }
            sum_r += r;
            sum_g += g;
            sum_b += b;
            count++;
        }
    }

    if (count == 0) {
        return false;
    }

    *out_r = (uint8_t)(sum_r / count);
    *out_g = (uint8_t)(sum_g / count);
    *out_b = (uint8_t)(sum_b / count);
    return true;
}

static bool pvg_capture_scene_matches(DisplaySurface *surface)
{
    for (size_t i = 0; i < pvg_scene_zone_count; i++) {
        const PvgSceneZone *zone = &pvg_scene_zones[i];
        uint8_t r = 0, g = 0, b = 0;
        int dr, dg, db;

        if (!pvg_capture_avg_zone(surface, zone, &r, &g, &b)) {
            return false;
        }
        dr = abs((int)r - (int)zone->r);
        dg = abs((int)g - (int)zone->g);
        db = abs((int)b - (int)zone->b);
        if (dr > zone->tol || dg > zone->tol || db > zone->tol) {
            return false;
        }
    }

    return true;
}

static void pvg_capture_log_scene(DisplaySurface *surface)
{
    for (size_t i = 0; i < pvg_scene_zone_count; i++) {
        const PvgSceneZone *zone = &pvg_scene_zones[i];
        uint8_t r = 0, g = 0, b = 0;

        if (!pvg_capture_avg_zone(surface, zone, &r, &g, &b)) {
            continue;
        }
        pvg_capture_log(
            "event=scene_zone name=%s avg=%u,%u,%u target=%u,%u,%u tol=%u\n",
            zone->name, r, g, b, zone->r, zone->g, zone->b, zone->tol);
    }
}

static void pvg_capture_map_memory(uint32_t task_id, uint64_t phys,
                                   uint64_t len, const void *data)
{
    uint64_t id;
    char rel[64];
    const char *dir = "map";
    const char *kind = "map";
    const uint8_t *bytes = data;

    if (!g_pvg_capture.enabled || !data || len == 0) {
        return;
    }

    if (pvg_capture_is_metallib(bytes, (size_t)len)) {
        dir = "metallib";
        kind = "metallib";
    }

    qemu_mutex_lock(&g_pvg_capture_mutex);
    id = ++g_pvg_capture.seq;
    if (pvg_capture_write_blob(dir, id, data, (size_t)len, rel, sizeof(rel))) {
        pvg_capture_log(
            "event=map kind=%s task=%u phys=0x%" PRIx64 " len=%" PRIu64
            " file=%s\n",
            kind, task_id, phys, len, rel);
    }
    qemu_mutex_unlock(&g_pvg_capture_mutex);
}

static void pvg_capture_unmap_memory(uint32_t task_id, uint64_t virt_offset,
                                     uint64_t len, const void *data)
{
    uint64_t id;
    char rel[64];

    if (!g_pvg_capture.enabled || !data || len == 0) {
        return;
    }

    qemu_mutex_lock(&g_pvg_capture_mutex);
    id = ++g_pvg_capture.seq;
    if (pvg_capture_write_blob("unmap", id, data, (size_t)len, rel, sizeof(rel))) {
        pvg_capture_log(
            "event=unmap task=%u virt_off=0x%" PRIx64 " len=%" PRIu64
            " file=%s\n",
            task_id, virt_offset, len, rel);
    }
    qemu_mutex_unlock(&g_pvg_capture_mutex);
}

static void pvg_capture_frame(DisplaySurface *surface)
{
    uint64_t id;
    char rel[64];
    size_t stride;
    size_t height;
    size_t len;
    void *data;
    pixman_format_code_t fmt;

    if (!g_pvg_capture.enabled || !g_pvg_capture.active ||
        !g_pvg_capture.capture_frames || !surface) {
        return;
    }

    stride = (size_t)surface_stride(surface);
    height = (size_t)surface_height(surface);
    len = stride * height;
    data = surface_data(surface);
    fmt = surface_format(surface);
    if (!data || len == 0) {
        return;
    }

    qemu_mutex_lock(&g_pvg_capture_mutex);
    id = ++g_pvg_capture.frame_index;
    if (pvg_capture_write_frame(id, data, len, rel, sizeof(rel))) {
        pvg_capture_log(
            "event=frame idx=%" PRIu64 " width=%d height=%d stride=%zu fmt=0x%08x file=%s\n",
            id, surface_width(surface),
            surface_height(surface), stride, (unsigned)fmt, rel);
    }
    qemu_mutex_unlock(&g_pvg_capture_mutex);
}

/* ------ PGTask and task operations: new/destroy/map/unmap ------ */

/*
 * This implements the type declared in <ParavirtualizedGraphics/PGDevice.h>
 * which is opaque from the framework's point of view. It is used in callbacks
 * in the form of its typedef PGTask_t, which also already exists in the
 * framework headers.
 *
 * A "task" in PVG terminology represents a host-virtual contiguous address
 * range which is reserved in a large chunk on task creation. The mapMemory
 * callback then requests ranges of guest system memory (identified by their
 * GPA) to be mapped into subranges of this reserved address space.
 * This type of operation isn't well-supported by QEMU's memory subsystem,
 * but it is fortunately trivial to achieve with Darwin's mach_vm_remap() call,
 * which allows us to refer to the same backing memory via multiple virtual
 * address ranges. The Mach VM APIs are therefore used throughout for managing
 * task memory.
 */
struct PGTask_s {
    QTAILQ_ENTRY(PGTask_s) node;
    AppleGFXState *s;
    mach_vm_address_t address;
    uint64_t len;
    uint32_t capture_id;
    /*
     * All unique MemoryRegions for which a mapping has been created in this
     * task, and on which we have thus called memory_region_ref(). There are
     * usually very few regions of system RAM in total, so we expect this array
     * to be very short. Therefore, no need for sorting or fancy search
     * algorithms, linear search will do.
     * Protected by AppleGFXState's task_mutex.
     */
    GPtrArray *mapped_regions;
};

static PGTask_t *apple_gfx_new_task(AppleGFXState *s, uint64_t len)
{
    mach_vm_address_t task_mem;
    PGTask_t *task;
    kern_return_t r;

    r = mach_vm_allocate(mach_task_self(), &task_mem, len, VM_FLAGS_ANYWHERE);
    if (r != KERN_SUCCESS) {
        return NULL;
    }

    task = g_new0(PGTask_t, 1);
    task->s = s;
    task->address = task_mem;
    task->len = len;
    task->capture_id = g_pvg_capture_task_next_id++;
    task->mapped_regions = g_ptr_array_sized_new(2 /* Usually enough */);

    QEMU_LOCK_GUARD(&s->task_mutex);
    QTAILQ_INSERT_TAIL(&s->tasks, task, node);

    return task;
}

static void apple_gfx_destroy_task(AppleGFXState *s, PGTask_t *task)
{
    GPtrArray *regions = task->mapped_regions;
    MemoryRegion *region;
    size_t i;

    for (i = 0; i < regions->len; ++i) {
        region = g_ptr_array_index(regions, i);
        memory_region_unref(region);
    }
    g_ptr_array_unref(regions);

    mach_vm_deallocate(mach_task_self(), task->address, task->len);

    QEMU_LOCK_GUARD(&s->task_mutex);
    QTAILQ_REMOVE(&s->tasks, task, node);
    g_free(task);
}

void *apple_gfx_host_ptr_for_gpa_range(uint64_t guest_physical,
                                       uint64_t length, bool read_only,
                                       MemoryRegion **mapping_in_region)
{
    MemoryRegion *ram_region;
    char *host_ptr;
    hwaddr ram_region_offset = 0;
    hwaddr ram_region_length = length;

    ram_region = address_space_translate(&address_space_memory,
                                         guest_physical,
                                         &ram_region_offset,
                                         &ram_region_length, !read_only,
                                         MEMTXATTRS_UNSPECIFIED);

    if (!ram_region || ram_region_length < length ||
        !memory_access_is_direct(ram_region, !read_only,
				 MEMTXATTRS_UNSPECIFIED)) {
        return NULL;
    }

    host_ptr = memory_region_get_ram_ptr(ram_region);
    if (!host_ptr) {
        return NULL;
    }
    host_ptr += ram_region_offset;
    *mapping_in_region = ram_region;
    return host_ptr;
}

static bool apple_gfx_task_map_memory(AppleGFXState *s, PGTask_t *task,
                                      uint64_t virtual_offset,
                                      PGPhysicalMemoryRange_t *ranges,
                                      uint32_t range_count, bool read_only)
{
    kern_return_t r;
    void *source_ptr;
    mach_vm_address_t target;
    vm_prot_t cur_protection, max_protection;
    bool success = true;
    MemoryRegion *region;

    RCU_READ_LOCK_GUARD();
    QEMU_LOCK_GUARD(&s->task_mutex);

    trace_apple_gfx_map_memory(task, range_count, virtual_offset, read_only);
    for (int i = 0; i < range_count; i++) {
        PGPhysicalMemoryRange_t *range = &ranges[i];

        target = task->address + virtual_offset;
        virtual_offset += range->physicalLength;

        trace_apple_gfx_map_memory_range(i, range->physicalAddress,
                                         range->physicalLength);

        region = NULL;
        source_ptr = apple_gfx_host_ptr_for_gpa_range(range->physicalAddress,
                                                      range->physicalLength,
                                                      read_only, &region);
        if (!source_ptr) {
            success = false;
            continue;
        }

        if (!g_ptr_array_find(task->mapped_regions, region, NULL)) {
            g_ptr_array_add(task->mapped_regions, region);
            memory_region_ref(region);
        }

        cur_protection = 0;
        max_protection = 0;
        /* Map guest RAM at range->physicalAddress into PG task memory range */
        r = mach_vm_remap(mach_task_self(),
                          &target, range->physicalLength, vm_page_size - 1,
                          VM_FLAGS_FIXED | VM_FLAGS_OVERWRITE,
                          mach_task_self(), (mach_vm_address_t)source_ptr,
                          false /* shared mapping, no copy */,
                          &cur_protection, &max_protection,
                          VM_INHERIT_COPY);
        trace_apple_gfx_remap(r, source_ptr, target);
        g_assert(r == KERN_SUCCESS);

        pvg_capture_map_memory(task->capture_id, range->physicalAddress,
                               range->physicalLength, source_ptr);
    }

    return success;
}

static void apple_gfx_task_unmap_memory(AppleGFXState *s, PGTask_t *task,
                                        uint64_t virtual_offset, uint64_t length)
{
    kern_return_t r;
    mach_vm_address_t range_address;
    void *range_ptr = (void *)(task->address + virtual_offset);

    trace_apple_gfx_unmap_memory(task, virtual_offset, length);

    pvg_capture_unmap_memory(task->capture_id, virtual_offset, length, range_ptr);

    /*
     * Replace task memory range with fresh 0 pages, undoing the mapping
     * from guest RAM.
     */
    range_address = task->address + virtual_offset;
    r = mach_vm_allocate(mach_task_self(), &range_address, length,
                         VM_FLAGS_FIXED | VM_FLAGS_OVERWRITE);
    g_assert(r == KERN_SUCCESS);
}

/* ------ Rendering and frame management ------ */

static void apple_gfx_render_frame_completed_bh(void *opaque);

static void apple_gfx_render_new_frame(AppleGFXState *s)
{
    bool managed_texture = s->using_managed_texture_storage;
    uint32_t width = surface_width(s->surface);
    uint32_t height = surface_height(s->surface);
    MTLRegion region = MTLRegionMake2D(0, 0, width, height);
    id<MTLCommandBuffer> command_buffer = [s->mtl_queue commandBuffer];
    id<MTLTexture> texture = s->texture;

    assert(bql_locked());
    [texture retain];
    [command_buffer retain];

    s->rendering_frame_width = width;
    s->rendering_frame_height = height;

    dispatch_async(get_background_queue(), ^{
        /*
         * This is not safe to call from the BQL/BH due to PVG-internal locks
         * causing deadlocks.
         */
        uint64_t t0 = mach_absolute_time();
        bool r = [s->pgdisp encodeCurrentFrameToCommandBuffer:command_buffer
                                                 texture:texture
                                                  region:region];
        uint64_t t1 = mach_absolute_time();
        if (!r) {
            [texture release];
            [command_buffer release];
            g_pvg_sync.encode_count++;
            pvg_sync_log(1, "ENCODE_FRAME: FAILED (#%llu)\n",
                         (unsigned long long)g_pvg_sync.encode_count);
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: encodeCurrentFrameToCommandBuffer:texture:region: "
                          "failed\n", __func__);
            bql_lock();
            --s->pending_frames;
            if (s->pending_frames > 0) {
                apple_gfx_render_new_frame(s);
            }
            bql_unlock();
            return;
        }

        g_pvg_sync.encode_count++;
        if (g_pvg_sync.level >= 1) {
            uint64_t elapsed_ns = (t1 - t0) *
                g_pvg_sync.timebase.numer / g_pvg_sync.timebase.denom;
            double elapsed_ms = (double)elapsed_ns / 1e6;
            pvg_sync_log(1, "ENCODE_FRAME: OK (#%llu, %.1fms)\n",
                         (unsigned long long)g_pvg_sync.encode_count,
                         elapsed_ms);
        }

        if (managed_texture) {
            /* "Managed" textures exist in both VRAM and RAM and must be synced. */
            id<MTLBlitCommandEncoder> blit = [command_buffer blitCommandEncoder];
            [blit synchronizeResource:texture];
            [blit endEncoding];
        }
        [texture release];
        [command_buffer addCompletedHandler:
            ^(id<MTLCommandBuffer> cb)
            {
                g_pvg_sync.completion_count++;
                pvg_sync_log(1, "FRAME_COMPLETE: #%llu\n",
                             (unsigned long long)g_pvg_sync.completion_count);
                aio_bh_schedule_oneshot(qemu_get_aio_context(),
                                        apple_gfx_render_frame_completed_bh, s);
            }];
        [command_buffer commit];
        [command_buffer release];
    });
}

static void copy_mtl_texture_to_surface_mem(id<MTLTexture> texture, void *vram)
{
    /*
     * TODO: Skip this entirely on a pure Metal or headless/guest-only
     * rendering path, else use a blit command encoder? Needs careful
     * (double?) buffering design.
     */
    size_t width = texture.width, height = texture.height;
    MTLRegion region = MTLRegionMake2D(0, 0, width, height);
    [texture getBytes:vram
          bytesPerRow:(width * 4)
        bytesPerImage:(width * height * 4)
           fromRegion:region
          mipmapLevel:0
                slice:0];
}

static void apple_gfx_render_frame_completed_bh(void *opaque)
{
    AppleGFXState *s = opaque;
    bool scene_match = false;

    @autoreleasepool {
        --s->pending_frames;
        assert(s->pending_frames >= 0);
        pvg_sync_log(1, "FRAME_BH: pending=%d w=%u h=%u\n",
                     (int)s->pending_frames,
                     s->rendering_frame_width,
                     s->rendering_frame_height);

        /* Only update display if mode hasn't changed since we started rendering. */
        if (s->rendering_frame_width == surface_width(s->surface) &&
            s->rendering_frame_height == surface_height(s->surface)) {
            copy_mtl_texture_to_surface_mem(s->texture, surface_data(s->surface));
            scene_match = g_pvg_capture.enabled && g_pvg_capture.active &&
                pvg_capture_scene_matches(s->surface);
            pvg_capture_frame(s->surface);
            if (scene_match) {
                pvg_capture_log_scene(s->surface);
                pvg_capture_log("event=scene_match\n");
                pvg_capture_shutdown();
            }
            if (s->gfx_update_requested) {
                s->gfx_update_requested = false;
                dpy_gfx_update_full(s->con);
                graphic_hw_update_done(s->con);
                s->new_frame_ready = false;
            } else {
                s->new_frame_ready = true;
            }
        }
        if (s->pending_frames > 0) {
            apple_gfx_render_new_frame(s);
        }
    }
}

static void apple_gfx_fb_update_display(void *opaque)
{
    AppleGFXState *s = opaque;

    assert(bql_locked());
    if (s->new_frame_ready) {
        dpy_gfx_update_full(s->con);
        s->new_frame_ready = false;
        graphic_hw_update_done(s->con);
    } else if (s->pending_frames > 0) {
        s->gfx_update_requested = true;
    } else {
        graphic_hw_update_done(s->con);
    }
}

static const GraphicHwOps apple_gfx_fb_ops = {
    .gfx_update = apple_gfx_fb_update_display,
    .gfx_update_async = true,
};

/* ------ Mouse cursor and display mode setting ------ */

static void set_mode(AppleGFXState *s, uint32_t width, uint32_t height)
{
    MTLTextureDescriptor *textureDescriptor;

    if (s->surface &&
        width == surface_width(s->surface) &&
        height == surface_height(s->surface)) {
        return;
    }

    [s->texture release];

    s->surface = qemu_create_displaysurface(width, height);

    @autoreleasepool {
        textureDescriptor =
            [MTLTextureDescriptor
                texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                             width:width
                                            height:height
                                         mipmapped:NO];
        textureDescriptor.usage = s->pgdisp.minimumTextureUsage;
        s->texture = [s->mtl newTextureWithDescriptor:textureDescriptor];
        s->using_managed_texture_storage =
            (s->texture.storageMode == MTLStorageModeManaged);
    }

    dpy_gfx_replace_surface(s->con, s->surface);
}

static void update_cursor(AppleGFXState *s)
{
    assert(bql_locked());
    dpy_mouse_set(s->con, s->pgdisp.cursorPosition.x,
                  s->pgdisp.cursorPosition.y, qatomic_read(&s->cursor_show));
}

static void update_cursor_bh(void *opaque)
{
    AppleGFXState *s = opaque;
    update_cursor(s);
}

typedef struct AppleGFXSetCursorGlyphJob {
    AppleGFXState *s;
    NSBitmapImageRep *glyph;
    PGDisplayCoord_t hotspot;
} AppleGFXSetCursorGlyphJob;

static void set_cursor_glyph(void *opaque)
{
    AppleGFXSetCursorGlyphJob *job = opaque;
    AppleGFXState *s = job->s;
    NSBitmapImageRep *glyph = job->glyph;
    uint32_t bpp = glyph.bitsPerPixel;
    size_t width = glyph.pixelsWide;
    size_t height = glyph.pixelsHigh;
    size_t padding_bytes_per_row = glyph.bytesPerRow - width * 4;
    const uint8_t* px_data = glyph.bitmapData;

    trace_apple_gfx_cursor_set(bpp, width, height);

    if (s->cursor) {
        cursor_unref(s->cursor);
        s->cursor = NULL;
    }

    if (bpp == 32) { /* Shouldn't be anything else, but just to be safe... */
        s->cursor = cursor_alloc(width, height);
        s->cursor->hot_x = job->hotspot.x;
        s->cursor->hot_y = job->hotspot.y;

        uint32_t *dest_px = s->cursor->data;

        for (size_t y = 0; y < height; ++y) {
            for (size_t x = 0; x < width; ++x) {
                /*
                 * NSBitmapImageRep's red & blue channels are swapped
                 * compared to QEMUCursor's.
                 */
                *dest_px =
                    (px_data[0] << 16u) |
                    (px_data[1] <<  8u) |
                    (px_data[2] <<  0u) |
                    (px_data[3] << 24u);
                ++dest_px;
                px_data += 4;
            }
            px_data += padding_bytes_per_row;
        }
        dpy_cursor_define(s->con, s->cursor);
        update_cursor(s);
    }
    [glyph release];

    g_free(job);
}

/* ------ DMA (device reading system memory) ------ */

/*
 * Metal encoder opcode name lookup (render encoder subset — most common).
 * Full table has 200+ opcodes; we decode the most important ones.
 */
static const char *pvg_metal_opcode_name(uint32_t enc_type, uint32_t opcode)
{
    if (enc_type == 0) { /* Render */
        switch (opcode) {
        case 0x00: return "DrawPrimitives64";
        case 0x01: return "DrawPrimitives16";
        case 0x06: return "DrawIndexedPrimitives64";
        case 0x07: return "DrawIndexedPrimitives16";
        case 0x08: return "DrawIndexedInstanced64";
        case 0x09: return "DrawIndexedInstanced16";
        case 0x10: return "DrawPrimitivesIndirect";
        case 0x11: return "DrawIndexedPrimitivesIndirect";
        case 0x1A: return "RenderPassDescriptor";
        case 0x26: return "SetRenderPipelineState";
        case 0x28: return "SetVertexBuffer";
        case 0x2C: return "SetVertexTexture";
        case 0x2E: return "SetVertexSamplerState";
        case 0x32: return "SetViewport";
        case 0x36: return "SetFrontFacingWinding";
        case 0x38: return "SetCullMode";
        case 0x3A: return "SetDepthStencilState";
        case 0x3E: return "SetScissorRect";
        case 0x40: return "SetTriangleFillMode";
        case 0x44: return "SetFragmentBuffer";
        case 0x48: return "SetFragmentTexture";
        case 0x4A: return "SetFragmentSamplerState";
        case 0x4E: return "SetBlendColor";
        case 0x50: return "SetDepthBias";
        case 0x52: return "SetStencilRefValue";
        case 0x72: return "SetVertexBytes";
        case 0x74: return "SetFragmentBytes";
        case 0x86: return "DispatchThreadsPerTile";
        case 0xE2: return "WaitForFence";
        case 0xE3: return "UpdateFence";
        case 0xE6: return "EndEncoding";
        default: return NULL;
        }
    } else if (enc_type == 1 || enc_type == 2) { /* Compute / Blit */
        switch (opcode) {
        case 0xC8: return "DispatchThreadgroups";
        case 0xD0: return "SetComputePipelineState";
        case 0xCB: return "SetComputeBuffer";
        case 0xCE: return "SetComputeTexture";
        case 0xE6: return "EndEncoding";
        /* Blit */
        case 0x12C: return "CopyFromTexture";
        case 0x12D: return "CopyFromBuffer";
        case 0x12E: return "CopyFromTextureToBuffer";
        case 0x12F: return "CopyFromBufferToTexture";
        case 0x132: return "FillBuffer";
        case 0x133: return "GenerateMipmaps";
        case 0x134: return "CopyFromTexture2";
        default: return NULL;
        }
    }
    return NULL;
}

/*
 * Walk Metal command buffer opcodes and log summary/details.
 * Buffer format: {u32 enc_type, u32 total_buf_size, cmd[0], cmd[1], ...}
 * Each cmd: {u32 opcode, u32 cmd_size_including_header, payload...}
 */
static void pvg_sync_log_metal_opcodes(const uint8_t *data, size_t data_len,
                                        uint32_t enc_type, uint32_t buf_len)
{
    size_t pos = 8; /* Skip 8-byte buffer header */
    size_t end = (buf_len < data_len) ? buf_len : data_len;
    int draw_count = 0, state_count = 0, total_count = 0;
    int rpd_count = 0;

    while (pos + 8 <= end) {
        uint32_t opcode, cmd_size;
        const char *name;

        memcpy(&opcode, data + pos, 4);
        memcpy(&cmd_size, data + pos + 4, 4);

        if (cmd_size < 8 || pos + cmd_size > end) {
            break;
        }

        total_count++;
        name = pvg_metal_opcode_name(enc_type, opcode);

        /* Classify */
        if (opcode <= 0x13) {
            draw_count++;
        } else if (opcode == 0x1A) {
            rpd_count++;
        } else {
            state_count++;
        }

        if (g_pvg_sync.level >= 3) {
            pvg_sync_log(3, "  MTL_OP[%d]: %s (0x%02x) size=%u\n",
                         total_count,
                         name ? name : "???",
                         opcode, cmd_size);
        }

        pos += cmd_size;
    }

    pvg_sync_log(2, "  MTL_SUMMARY: %d cmds (%d draws, %d rpd, %d state)\n",
                 total_count, draw_count, rpd_count, state_count);
}

static void pvg_sync_log_dma_read(hwaddr phys, uint64_t len, const void *data)
{
    static const char *enc_names[] = {
        [0] = "render", [1] = "blit", [2] = "compute", [4] = "indirect"
    };
    const uint8_t *bytes = data;
    uint32_t enc_type = 0, buf_len = 0;

    if (g_pvg_sync.level < 1 || !data || len == 0) {
        return;
    }

    g_pvg_sync.dma_read_count++;

    if (pvg_capture_is_cmd_buffer(bytes, (size_t)len, &enc_type, &buf_len)) {
        const char *enc_name = (enc_type < 5) ? enc_names[enc_type] : "?";
        if (!enc_name) {
            enc_name = "?";
        }
        pvg_sync_log(1, "DMA_READ cmd: phys=0x%llx len=%llu enc=%s "
                     "buf_len=%u (#%llu)\n",
                     (unsigned long long)phys, (unsigned long long)len,
                     enc_name, buf_len,
                     (unsigned long long)g_pvg_sync.dma_read_count);

        if (g_pvg_sync.level >= 2) {
            /* Simple hash for correlation with QMetal's EXEC3_HASH */
            uint32_t hash = 0x811c9dc5; /* FNV-1a 32-bit offset basis */
            size_t hash_len = (buf_len < len) ? buf_len : (size_t)len;
            for (size_t i = 0; i < hash_len; i++) {
                hash ^= bytes[i];
                hash *= 0x01000193;
            }
            pvg_sync_log(2, "CMD_HASH: size=%u hash=0x%08x\n",
                         buf_len, hash);

            /* Parse Metal encoder opcodes */
            pvg_sync_log_metal_opcodes(bytes, (size_t)len, enc_type, buf_len);
        }
    } else if (pvg_capture_is_metallib(bytes, (size_t)len)) {
        pvg_sync_log(2, "DMA_READ metallib: phys=0x%llx len=%llu\n",
                     (unsigned long long)phys, (unsigned long long)len);
    } else {
        pvg_sync_log(2, "DMA_READ mem: phys=0x%llx len=%llu\n",
                     (unsigned long long)phys, (unsigned long long)len);
    }

    if (g_pvg_sync.level >= 3) {
        size_t dump_len = (len > 64) ? 64 : (size_t)len;
        char hex[64 * 3 + 1];
        size_t pos = 0;
        for (size_t i = 0; i < dump_len; i++) {
            pos += snprintf(hex + pos, sizeof(hex) - pos, "%02x ", bytes[i]);
        }
        pvg_sync_log(3, "  HEX: %s\n", hex);
    }
}

typedef struct AppleGFXReadMemoryJob {
    QemuEvent event;
    hwaddr physical_address;
    uint64_t length;
    void *dst;
    bool success;
} AppleGFXReadMemoryJob;

static void apple_gfx_do_read_memory(void *opaque)
{
    AppleGFXReadMemoryJob *job = opaque;
    MemTxResult r;

    r = dma_memory_read(&address_space_memory, job->physical_address,
                        job->dst, job->length, MEMTXATTRS_UNSPECIFIED);
    job->success = (r == MEMTX_OK);

    if (job->success) {
        pvg_capture_read_memory(job->physical_address, job->length, job->dst);
        pvg_sync_log_dma_read(job->physical_address, job->length, job->dst);
    }

    qemu_event_set(&job->event);
}

static bool apple_gfx_read_memory(AppleGFXState *s, hwaddr physical_address,
                                  uint64_t length, void *dst)
{
    AppleGFXReadMemoryJob job = {
        .physical_address = physical_address, .length = length, .dst = dst
    };

    trace_apple_gfx_read_memory(physical_address, length, dst);

    /* Performing DMA requires BQL, so do it in a BH. */
    qemu_event_init(&job.event, 0);
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            apple_gfx_do_read_memory, &job);
    qemu_event_wait(&job.event);
    qemu_event_destroy(&job.event);
    return job.success;
}

/* ------ Memory-mapped device I/O operations ------ */

typedef struct AppleGFXIOJob {
    AppleGFXState *state;
    uint64_t offset;
    uint64_t value;
    bool completed;
} AppleGFXIOJob;

static void apple_gfx_do_read(void *opaque)
{
    AppleGFXIOJob *job = opaque;
    job->value = [job->state->pgdev mmioReadAtOffset:job->offset];
    qatomic_set(&job->completed, true);
    aio_wait_kick();
}

static uint64_t apple_gfx_read(void *opaque, hwaddr offset, unsigned size)
{
    AppleGFXIOJob job = {
        .state = opaque,
        .offset = offset,
        .completed = false,
    };
    dispatch_queue_t queue = get_background_queue();

    dispatch_async_f(queue, &job, apple_gfx_do_read);
    AIO_WAIT_WHILE(NULL, !qatomic_read(&job.completed));

    trace_apple_gfx_read(offset, job.value);

    if (g_pvg_sync.level >= 1) {
        uint64_t val = job.value;
        switch (offset) {
        case 0x1014: { /* DISPLAY_IRQ - atomic read-and-clear */
            char flags[64] = "";
            if (val & 0x02) { strlcat(flags, " BLOCK_INVOKE", sizeof(flags)); }
            if (val & 0x04) { strlcat(flags, " RENDER_INITIATED", sizeof(flags)); }
            if (val & 0x08) { strlcat(flags, " CYCLE_COMPLETE", sizeof(flags)); }
            pvg_sync_log(1, "READ DISPLAY_IRQ=0x%02llx [%s]\n",
                         (unsigned long long)val,
                         flags[0] ? flags + 1 : "none");
            break;
        }
        case 0x1018: { /* EVENT_STAMPS - atomic read-and-clear */
            char chans[64] = "";
            for (int i = 0; i < 6; i++) {
                if (val & (1u << i)) {
                    char tmp[8];
                    snprintf(tmp, sizeof(tmp), " ch%d", i);
                    strlcat(chans, tmp, sizeof(chans));
                }
            }
            pvg_sync_log(1, "READ EVENT_STAMPS=0x%02llx [%s]\n",
                         (unsigned long long)val,
                         chans[0] ? chans + 1 : "none");
            break;
        }
        case 0x100c: /* FIFO_READ */
            pvg_sync_log(2, "READ FIFO_READ=0x%08llx\n",
                         (unsigned long long)val);
            break;
        case 0x1024: /* PENDING_COMP */
            pvg_sync_log(2, "READ PENDING_COMP=%llu\n",
                         (unsigned long long)val);
            break;
        default: {
            const char *name = pvg_reg_name(offset);
            pvg_sync_log(2, "READ %s (0x%04llx)=0x%08llx\n",
                         name ? name : "???",
                         (unsigned long long)offset,
                         (unsigned long long)val);
            break;
        }
        }
    }

    return job.value;
}

static void apple_gfx_do_write(void *opaque)
{
    AppleGFXIOJob *job = opaque;
    [job->state->pgdev mmioWriteAtOffset:job->offset value:job->value];
    qatomic_set(&job->completed, true);
    aio_wait_kick();
}

static void apple_gfx_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    /*
     * The methods mmioReadAtOffset: and especially mmioWriteAtOffset: can
     * trigger synchronous operations on other dispatch queues, which in turn
     * may call back out on one or more of the callback blocks. For this reason,
     * and as we are holding the BQL, we invoke the I/O methods on a pool
     * thread and handle AIO tasks while we wait. Any work in the callbacks
     * requiring the BQL will in turn schedule BHs which this thread will
     * process while waiting.
     */

    if (g_pvg_sync.level >= 1) {
        switch (offset) {
        case 0x1020: /* KICK */
            g_pvg_sync.kick_count++;
            pvg_sync_log(1, "KICK: channel=%llu (#%llu)\n",
                         (unsigned long long)val,
                         (unsigned long long)g_pvg_sync.kick_count);
            if (val >= 1 && val <= 5) {
                pvg_sync_sniff_child_fifo((uint32_t)val);
            }
            break;
        case 0x1008: /* FIFO_WRITE */
            g_pvg_sync.fifo_write = (uint32_t)val;
            pvg_sync_log(1, "FIFO_WRITE: ptr=0x%08llx\n",
                         (unsigned long long)val);
            pvg_sync_sniff_root_fifo();
            break;
        case 0x1028: /* TRANSACTION_ID */
            pvg_sync_log(1, "TRANSACTION_ID: val=0x%08llx\n",
                         (unsigned long long)val);
            break;
        case 0x101c: /* SHARED_STATE_PFN */
            pvg_sync_log(1, "SHARED_STATE_PFN: pfn=0x%llx gpa=0x%llx\n",
                         (unsigned long long)val,
                         (unsigned long long)(val << 12));
            break;
        case 0x1030: /* FIFO_PFN */
            g_pvg_sync.fifo_pfn = val;
            pvg_sync_log(1, "FIFO_PFN: pfn=0x%llx gpa=0x%llx\n",
                         (unsigned long long)val,
                         (unsigned long long)(val << 12));
            break;
        case 0x1000: /* FIFO_ENABLE */
            if (val && !g_pvg_sync.fifo_ring_size) {
                /* Reset sniff pointer on enable */
                g_pvg_sync.fifo_sniff_ptr = g_pvg_sync.fifo_write;
            }
            pvg_sync_log(1, "FIFO_ENABLE: %s\n", val ? "ON" : "OFF");
            break;
        case 0x1034: /* PROTOCOL_VERSION */
            pvg_sync_log(1, "PROTOCOL_VERSION: %llu\n",
                         (unsigned long long)val);
            break;
        case 0x1004: /* FIFO_SIZE */
            g_pvg_sync.fifo_ring_size = (uint32_t)val;
            pvg_sync_log(2, "FIFO_SIZE: 0x%08llx\n",
                         (unsigned long long)val);
            break;
        case 0x1010: /* FIFO_OFFSET */
            g_pvg_sync.fifo_offset = (uint32_t)val;
            pvg_sync_log(2, "FIFO_OFFSET: 0x%08llx\n",
                         (unsigned long long)val);
            break;
        default: {
            const char *name = pvg_reg_name(offset);
            if (offset >= 0x1200 && offset <= 0x122c) {
                pvg_sync_log(2, "WRITE %s (0x%04llx)=0x%08llx\n",
                             name ? name : "IOSFC_???",
                             (unsigned long long)offset,
                             (unsigned long long)val);
            } else {
                pvg_sync_log(2, "WRITE %s (0x%04llx)=0x%08llx\n",
                             name ? name : "???",
                             (unsigned long long)offset,
                             (unsigned long long)val);
            }
            break;
        }
        }
    }

    AppleGFXIOJob job = {
        .state = opaque,
        .offset = offset,
        .value = val,
        .completed = false,
    };
    dispatch_queue_t queue = get_background_queue();

    dispatch_async_f(queue, &job, apple_gfx_do_write);
    AIO_WAIT_WHILE(NULL, !qatomic_read(&job.completed));

    trace_apple_gfx_write(offset, val);
}

static const MemoryRegionOps apple_gfx_ops = {
    .read = apple_gfx_read,
    .write = apple_gfx_write,
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

static size_t apple_gfx_get_default_mmio_range_size(void)
{
    size_t mmio_range_size;
    @autoreleasepool {
        PGDeviceDescriptor *desc = [PGDeviceDescriptor new];
        mmio_range_size = desc.mmioLength;
        [desc release];
    }
    return mmio_range_size;
}

/* ------ Initialisation and startup ------ */

void apple_gfx_common_init(Object *obj, AppleGFXState *s, const char* obj_name)
{
    size_t mmio_range_size = apple_gfx_get_default_mmio_range_size();

    trace_apple_gfx_common_init(obj_name, mmio_range_size);
    memory_region_init_io(&s->iomem_gfx, obj, &apple_gfx_ops, s, obj_name,
                          mmio_range_size);

    /* TODO: PVG framework supports serialising device state: integrate it! */
}

static void apple_gfx_register_task_mapping_handlers(AppleGFXState *s,
                                                     PGDeviceDescriptor *desc)
{
    desc.createTask = ^(uint64_t vmSize, void * _Nullable * _Nonnull baseAddress) {
        PGTask_t *task = apple_gfx_new_task(s, vmSize);
        *baseAddress = (void *)task->address;
        trace_apple_gfx_create_task(vmSize, *baseAddress);
        return task;
    };

    desc.destroyTask = ^(PGTask_t * _Nonnull task) {
        trace_apple_gfx_destroy_task(task, task->mapped_regions->len);

        apple_gfx_destroy_task(s, task);
    };

    desc.mapMemory = ^bool(PGTask_t * _Nonnull task, uint32_t range_count,
                           uint64_t virtual_offset, bool read_only,
                           PGPhysicalMemoryRange_t * _Nonnull ranges) {
        return apple_gfx_task_map_memory(s, task, virtual_offset,
                                         ranges, range_count, read_only);
    };

    desc.unmapMemory = ^bool(PGTask_t * _Nonnull task, uint64_t virtual_offset,
                             uint64_t length) {
        apple_gfx_task_unmap_memory(s, task, virtual_offset, length);
        return true;
    };

    desc.readMemory = ^bool(uint64_t physical_address, uint64_t length,
                            void * _Nonnull dst) {
        return apple_gfx_read_memory(s, physical_address, length, dst);
    };
}

static void new_frame_handler_bh(void *opaque)
{
    AppleGFXState *s = opaque;

    /* Drop frames if guest gets too far ahead. */
    if (s->pending_frames >= 2) {
        pvg_sync_log(1, "FRAME_DROP: pending=%d\n", (int)s->pending_frames);
        return;
    }
    ++s->pending_frames;
    pvg_sync_log(1, "FRAME_QUEUE: pending=%d\n", (int)s->pending_frames);
    if (s->pending_frames > 1) {
        return;
    }

    @autoreleasepool {
        apple_gfx_render_new_frame(s);
    }
}

static PGDisplayDescriptor *apple_gfx_prepare_display_descriptor(AppleGFXState *s)
{
    PGDisplayDescriptor *disp_desc = [PGDisplayDescriptor new];

    disp_desc.name = @"QEMU display";
    disp_desc.sizeInMillimeters = NSMakeSize(400., 300.); /* A 20" display */
    disp_desc.queue = dispatch_get_main_queue();
    disp_desc.newFrameEventHandler = ^(void) {
        trace_apple_gfx_new_frame();
        g_pvg_sync.frame_count++;
        pvg_sync_log(1, "NEW_FRAME: #%llu\n",
                     (unsigned long long)g_pvg_sync.frame_count);
        aio_bh_schedule_oneshot(qemu_get_aio_context(), new_frame_handler_bh, s);
    };
    disp_desc.modeChangeHandler = ^(PGDisplayCoord_t sizeInPixels,
                                    OSType pixelFormat) {
        trace_apple_gfx_mode_change(sizeInPixels.x, sizeInPixels.y);

        BQL_LOCK_GUARD();
        set_mode(s, sizeInPixels.x, sizeInPixels.y);
    };
    disp_desc.cursorGlyphHandler = ^(NSBitmapImageRep *glyph,
                                     PGDisplayCoord_t hotspot) {
        AppleGFXSetCursorGlyphJob *job = g_malloc0(sizeof(*job));
        job->s = s;
        job->glyph = glyph;
        job->hotspot = hotspot;
        [glyph retain];
        aio_bh_schedule_oneshot(qemu_get_aio_context(),
                                set_cursor_glyph, job);
    };
    disp_desc.cursorShowHandler = ^(BOOL show) {
        trace_apple_gfx_cursor_show(show);
        qatomic_set(&s->cursor_show, show);
        aio_bh_schedule_oneshot(qemu_get_aio_context(),
                                update_cursor_bh, s);
    };
    disp_desc.cursorMoveHandler = ^(void) {
        trace_apple_gfx_cursor_move();
        aio_bh_schedule_oneshot(qemu_get_aio_context(),
                                update_cursor_bh, s);
    };

    return disp_desc;
}

static NSArray<PGDisplayMode *> *apple_gfx_create_display_mode_array(
    const AppleGFXDisplayMode display_modes[], uint32_t display_mode_count)
{
    PGDisplayMode *mode_obj;
    NSMutableArray<PGDisplayMode *> *mode_array =
        [[NSMutableArray alloc] initWithCapacity:display_mode_count];

    for (unsigned i = 0; i < display_mode_count; i++) {
        const AppleGFXDisplayMode *mode = &display_modes[i];
        trace_apple_gfx_display_mode(i, mode->width_px, mode->height_px);
        PGDisplayCoord_t mode_size = { mode->width_px, mode->height_px };

        mode_obj =
            [[PGDisplayMode alloc] initWithSizeInPixels:mode_size
                                        refreshRateInHz:mode->refresh_rate_hz];
        [mode_array addObject:mode_obj];
        [mode_obj release];
    }

    return mode_array;
}

static id<MTLDevice> copy_suitable_metal_device(void)
{
    id<MTLDevice> dev = nil;
    NSArray<id<MTLDevice>> *devs = MTLCopyAllDevices();

    /* Prefer a unified memory GPU. Failing that, pick a non-removable GPU. */
    for (size_t i = 0; i < devs.count; ++i) {
        if (devs[i].hasUnifiedMemory) {
            dev = devs[i];
            break;
        }
        if (!devs[i].removable) {
            dev = devs[i];
        }
    }

    if (dev != nil) {
        [dev retain];
    } else {
        dev = MTLCreateSystemDefaultDevice();
    }
    [devs release];

    return dev;
}

bool apple_gfx_common_realize(AppleGFXState *s, DeviceState *dev,
                              PGDeviceDescriptor *desc, Error **errp)
{
    PGDisplayDescriptor *disp_desc;
    const AppleGFXDisplayMode *display_modes = apple_gfx_default_modes;
    uint32_t num_display_modes = ARRAY_SIZE(apple_gfx_default_modes);
    NSArray<PGDisplayMode *> *mode_array;

    if (apple_gfx_mig_blocker == NULL) {
        error_setg(&apple_gfx_mig_blocker,
                  "Migration state blocked by apple-gfx display device");
        if (migrate_add_blocker(&apple_gfx_mig_blocker, errp) < 0) {
            return false;
        }
    }

    qemu_mutex_init(&s->task_mutex);
    QTAILQ_INIT(&s->tasks);
    s->mtl = copy_suitable_metal_device();
    s->mtl_queue = [s->mtl newCommandQueue];

    desc.device = s->mtl;

    pvg_sync_init();
    pvg_capture_init();
    apple_gfx_register_task_mapping_handlers(s, desc);

    s->cursor_show = true;

    s->pgdev = PGNewDeviceWithDescriptor(desc);

    disp_desc = apple_gfx_prepare_display_descriptor(s);
    /*
     * Although the framework does, this integration currently does not support
     * multiple virtual displays connected to a single PV graphics device.
     * It is however possible to create
     * more than one instance of the device, each with one display. The macOS
     * guest will ignore these displays if they share the same serial number,
     * so ensure each instance gets a unique one.
     */
    s->pgdisp = [s->pgdev newDisplayWithDescriptor:disp_desc
                                              port:0
                                         serialNum:next_pgdisplay_serial_num++];
    [disp_desc release];

    if (s->display_modes != NULL && s->num_display_modes > 0) {
        trace_apple_gfx_common_realize_modes_property(s->num_display_modes);
        display_modes = s->display_modes;
        num_display_modes = s->num_display_modes;
    }
    s->pgdisp.modeList = mode_array =
        apple_gfx_create_display_mode_array(display_modes, num_display_modes);
    [mode_array release];

    s->con = graphic_console_init(dev, 0, &apple_gfx_fb_ops, s);
    return true;
}

/* ------ Display mode list device property ------ */

static void apple_gfx_get_display_mode(Object *obj, Visitor *v,
                                       const char *name, void *opaque,
                                       Error **errp)
{
    Property *prop = opaque;
    AppleGFXDisplayMode *mode = object_field_prop_ptr(obj, prop);
    /* 3 uint16s (max 5 digits) + 2 separator characters + nul. */
    char buffer[5 * 3 + 2 + 1];
    char *pos = buffer;

    int rc = snprintf(buffer, sizeof(buffer),
                      "%"PRIu16"x%"PRIu16"@%"PRIu16,
                      mode->width_px, mode->height_px,
                      mode->refresh_rate_hz);
    assert(rc < sizeof(buffer));

    visit_type_str(v, name, &pos, errp);
}

static void apple_gfx_set_display_mode(Object *obj, Visitor *v,
                                       const char *name, void *opaque,
                                       Error **errp)
{
    Property *prop = opaque;
    AppleGFXDisplayMode *mode = object_field_prop_ptr(obj, prop);
    const char *endptr;
    g_autofree char *str = NULL;
    int ret;
    int val;

    if (!visit_type_str(v, name, &str, errp)) {
        return;
    }

    endptr = str;

    ret = qemu_strtoi(endptr, &endptr, 10, &val);
    if (ret || val > UINT16_MAX || val <= 0) {
        error_setg(errp, "width in '%s' must be a decimal integer number"
                         " of pixels in the range 1..65535", name);
        return;
    }
    mode->width_px = val;
    if (*endptr != 'x') {
        goto separator_error;
    }

    ret = qemu_strtoi(endptr + 1, &endptr, 10, &val);
    if (ret || val > UINT16_MAX || val <= 0) {
        error_setg(errp, "height in '%s' must be a decimal integer number"
                         " of pixels in the range 1..65535", name);
        return;
    }
    mode->height_px = val;
    if (*endptr != '@') {
        goto separator_error;
    }

    ret = qemu_strtoi(endptr + 1, &endptr, 10, &val);
    if (ret || val > UINT16_MAX || val <= 0) {
        error_setg(errp, "refresh rate in '%s'"
                         " must be a positive decimal integer (Hertz)", name);
        return;
    }
    mode->refresh_rate_hz = val;
    return;

separator_error:
    error_setg(errp,
               "Each display mode takes the format '<width>x<height>@<rate>'");
}

const PropertyInfo qdev_prop_apple_gfx_display_mode = {
    .type  = "display_mode",
    .description =
        "Display mode in pixels and Hertz, as <width>x<height>@<refresh-rate> "
        "Example: 3840x2160@60",
    .get   = apple_gfx_get_display_mode,
    .set   = apple_gfx_set_display_mode,
};
