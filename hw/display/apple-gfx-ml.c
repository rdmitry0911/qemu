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
 * Render completion now arrives as one owner-complete event, and the wrapper
 * keeps the reference-shaped single BH for retirement, display apply, and
 * render chaining.
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
#include "block/thread-pool.h"
#include "qapi/error.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci.h"
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
int qmu_vk_request_display_frame(struct qmu_vulkan_ctx *ctx);
int qmu_vk_capture_display_frame_request(struct qmu_vulkan_ctx *ctx);
int qmu_vk_submit_captured_display_frame(struct qmu_vulkan_ctx *ctx);
int qmu_vk_clockab_runtime_texel_flush_window(struct qmu_vulkan_ctx *ctx);
int qmu_vk_clockab_terminal_operand_begin_window(struct qmu_vulkan_ctx *ctx);
int qmu_vk_clockab_terminal_operand_flush_window(struct qmu_vulkan_ctx *ctx);
void qmu_vk_consume_current_frame_signal(struct qmu_vulkan_ctx *ctx);
typedef struct AppleGfxMLSessionJob AppleGfxMLSessionJob;
typedef struct AgfxLogEntry AgfxLogEntry;
typedef struct AppleGfxMLFrameCompletionJob AppleGfxMLFrameCompletionJob;

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
    bool frame_expected;
    bool chain_needed;
    uint8_t *pixels;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    AppleGfxMLCaptureBridge bridge;
};

static QemuMutex agfx_capture_bridge_mutex;
static gsize agfx_capture_bridge_inited;
static AppleGfxMLCaptureBridge agfx_capture_bridge_latest;
static uint64_t agfx_capture_bridge_next_apply_seq;
static uint64_t agfx_capture_bridge_next_qmp_ordinal;

/* ── AGFX_PRESENT_RECEIPT (diagnostic, s0-frozen spec v2.2 2026-08-23) ──
 * Env AGFX_PRESENT_RECEIPT_FILE=<path>: per delivered present, append one
 * in-memory row {qemu_present_count, host_apply_seq, bridge metadata,
 * canonical ROI SHA-256, ppm path}; flushed ONCE at exit (in-memory ring —
 * per-present file I/O historically suppressed the flicker under study).
 * Canonical hash: row-major RGB (from BGRA bytes b2,b1,b0), ROI
 * x[827,1089) y[147,260), no header. Behavior-neutral: log-only. */
/* local SHA-256 (public-domain style, no external locks — BH-safe) */
typedef struct { uint32_t h[8]; uint64_t len; uint8_t buf[64]; size_t off; } agfx_sha256;
static const uint32_t agfx_sha_k[64] = {
0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
#define AGFX_ROR(x,n) (((x)>>(n))|((x)<<(32-(n))))
static void agfx_sha256_block(agfx_sha256 *c, const uint8_t *p)
{
    uint32_t w[64], a,b,cc,d,e,f,g,h; int i;
    for (i=0;i<16;i++) w[i]=((uint32_t)p[i*4]<<24)|((uint32_t)p[i*4+1]<<16)|((uint32_t)p[i*4+2]<<8)|p[i*4+3];
    for (i=16;i<64;i++){uint32_t s0=AGFX_ROR(w[i-15],7)^AGFX_ROR(w[i-15],18)^(w[i-15]>>3);
        uint32_t s1=AGFX_ROR(w[i-2],17)^AGFX_ROR(w[i-2],19)^(w[i-2]>>10); w[i]=w[i-16]+s0+w[i-7]+s1;}
    a=c->h[0];b=c->h[1];cc=c->h[2];d=c->h[3];e=c->h[4];f=c->h[5];g=c->h[6];h=c->h[7];
    for (i=0;i<64;i++){uint32_t S1=AGFX_ROR(e,6)^AGFX_ROR(e,11)^AGFX_ROR(e,25);
        uint32_t ch=(e&f)^((~e)&g); uint32_t t1=h+S1+ch+agfx_sha_k[i]+w[i];
        uint32_t S0=AGFX_ROR(a,2)^AGFX_ROR(a,13)^AGFX_ROR(a,22);
        uint32_t mj=(a&b)^(a&cc)^(b&cc); uint32_t t2=S0+mj;
        h=g;g=f;f=e;e=d+t1;d=cc;cc=b;b=a;a=t1+t2;}
    c->h[0]+=a;c->h[1]+=b;c->h[2]+=cc;c->h[3]+=d;c->h[4]+=e;c->h[5]+=f;c->h[6]+=g;c->h[7]+=h;
}
static void agfx_sha256_init(agfx_sha256 *c){
    static const uint32_t iv[8]={0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    memcpy(c->h,iv,sizeof iv); c->len=0; c->off=0; }
static void agfx_sha256_update(agfx_sha256 *c,const uint8_t *d,size_t n){
    c->len+=n;
    while(n){ size_t t=64-c->off; if(t>n)t=n; memcpy(c->buf+c->off,d,t); c->off+=t; d+=t; n-=t;
        if(c->off==64){agfx_sha256_block(c,c->buf); c->off=0;} } }
static void agfx_sha256_final(agfx_sha256 *c, uint8_t out[32]){
    uint64_t bits=c->len*8; uint8_t pad=0x80; agfx_sha256_update(c,&pad,1); pad=0;
    while(c->off!=56) agfx_sha256_update(c,&pad,1);
    uint8_t lb[8]; for(int i=0;i<8;i++) lb[i]=(uint8_t)(bits>>(56-8*i));
    agfx_sha256_update(c,lb,8);
    for(int i=0;i<8;i++){out[i*4]=(uint8_t)(c->h[i]>>24);out[i*4+1]=(uint8_t)(c->h[i]>>16);
        out[i*4+2]=(uint8_t)(c->h[i]>>8);out[i*4+3]=(uint8_t)c->h[i];}}
#define AGFX_RCPT_ROI_X0 827
#define AGFX_RCPT_ROI_X1 1089
#define AGFX_RCPT_ROI_Y0 147
#define AGFX_RCPT_ROI_Y1 260
static FILE *agfx_rcpt_fp;
static const char *agfx_rcpt_file;
static int agfx_rcpt_init_done;
static uint64_t agfx_rcpt_rowcount;
static void agfx_rcpt_flush(void)
{
    if (agfx_rcpt_fp) {
        fflush(agfx_rcpt_fp);
    }
}
static void agfx_rcpt_init(void)
{
    if (agfx_rcpt_init_done) return;
    agfx_rcpt_init_done = 1;
    agfx_rcpt_file = getenv("AGFX_PRESENT_RECEIPT_FILE");
    if (agfx_rcpt_file && agfx_rcpt_file[0]) {
        agfx_rcpt_fp = fopen(agfx_rcpt_file, "w");
        if (agfx_rcpt_fp) {
            setvbuf(agfx_rcpt_fp, NULL, _IOFBF, 1 << 16);
            fprintf(agfx_rcpt_fp,
                "present_count\thost_apply_seq\tbridge_valid\tqmetal_delivery_seq\t"
                "display_cookie\tsource_texture_id\tsource_vk_image\tsource_vk_view\t"
                "image_generation\tbacking_id\tbacking_generation\tbacking_va\t"
                "backing_task\tbacking_resource\twriter_seq\tqueue_submit_seq\t"
                "request_epoch\tframe_serial\ttxn3_seq\ttxn3_digest_lo\ttxn3_digest_hi\t"
                "roi_sha256\tppm\n");
            atexit(agfx_rcpt_flush);
        } else {
            agfx_rcpt_file = NULL;
        }
    } else {
        agfx_rcpt_file = NULL;
    }
}

/* Probe #5 QEMU delivery side of the immutable GFXR bridge.  This ring is
 * intentionally fixed-size and flushes once when the explicitly armed window
 * closes (or, when no bounded window was requested, at process exit).
 * Individual display applies perform no bridge-side file I/O, allocation, or
 * logging.
 * It begins only with AGFX_FRAME_CAPTURE_DIR_ALL's explicitly armed capture
 * window; pre-window boot frames are outside the experiment, not rejects.
 * Within that window a missing PPM/hash/token is rejected evidence, never
 * repaired by ordering or a later global framebuffer read. */
#define AGFX_CLOCKAB_BRIDGE_ROWS 256
typedef struct AgfxClockabBridgeRow {
    AppleGfxMLCaptureBridge bridge;
    uint64_t present_seq;
    char roi_sha256[65];
    char ppm[64];
} AgfxClockabBridgeRow;

static AgfxClockabBridgeRow agfx_clockab_rows[AGFX_CLOCKAB_BRIDGE_ROWS];
static const char *agfx_clockab_file;
static uint64_t agfx_clockab_count;
static uint64_t agfx_clockab_dropped;
static uint64_t agfx_clockab_rejected;
static uint64_t agfx_clockab_next_present_seq = 1;
static uint64_t agfx_clockab_capture_count;
static int agfx_clockab_init_done;
static bool agfx_clockab_window_flushed;
static bool agfx_clockab_terminal_operand_window_started;

static void agfx_clockab_flush(void)
{
    FILE *out;
    g_autofree char *stats_path = NULL;
    FILE *stats;

    if (!agfx_clockab_file || !agfx_clockab_file[0]) {
        return;
    }
    out = fopen(agfx_clockab_file, "w");
    if (!out) {
        return;
    }
    for (uint64_t index = 0; index < agfx_clockab_count; ++index) {
        const AgfxClockabBridgeRow *row = &agfx_clockab_rows[index];
        const AppleGfxMLCaptureBridge *b = &row->bridge;
        fprintf(out,
            "{\"schema\":\"CLOCKAB_GFXR_BRIDGE_V1\",\"kind\":\"qemu_apply\","
            "\"boot_uuid\":\"%016" PRIx64 "%016" PRIx64 "\","
            "\"bridge_token\":\"%016" PRIx64 "%016" PRIx64 "%016" PRIx64 "%016" PRIx64 "\","
            "\"selection_seq\":%" PRIu64 ",\"present_seq\":%" PRIu64 ","
            "\"qemu_present_count\":%" PRIu64 ",\"host_apply_seq\":%" PRIu64 ","
            "\"qmetal_submit_seq\":%" PRIu64 ",\"qmetal_queue_submit_seq\":%" PRIu64 ","
            "\"source\":{\"raw_id\":\"0x%" PRIx64 "\",\"generation\":%" PRIu64 ","
            "\"extent\":{\"width\":%u,\"height\":%u},\"format\":\"BGRA8_UNORM\","
            "\"source_rect\":{\"x\":%u,\"y\":%u,\"width\":%u,\"height\":%u},"
            "\"destination_rect\":{\"x\":%u,\"y\":%u,\"width\":%u,\"height\":%u},"
            "\"orientation\":\"IDENTITY\",\"filter\":\"NEAREST\","
            "\"copy_path\":\"DISPLAY_COMPOSITE\",\"composite_flags\":%u},"
            "\"roi_sha256\":\"%s\",\"ppm\":\"%s\"}\n",
            b->clockab_boot_uuid_lo, b->clockab_boot_uuid_hi,
            b->clockab_boot_uuid_lo, b->clockab_boot_uuid_hi,
            b->clockab_selection_seq, b->clockab_qmetal_submit_seq,
            b->clockab_selection_seq, row->present_seq,
            b->qemu_present_count, b->host_apply_seq,
            b->clockab_qmetal_submit_seq, b->clockab_qmetal_queue_submit_seq,
            b->clockab_source_raw_image, b->clockab_source_generation,
            b->clockab_source_width, b->clockab_source_height,
            b->clockab_source_rect_x, b->clockab_source_rect_y,
            b->clockab_source_rect_width, b->clockab_source_rect_height,
            b->clockab_destination_rect_x, b->clockab_destination_rect_y,
            b->clockab_destination_rect_width, b->clockab_destination_rect_height,
            b->clockab_composite_flags, row->roi_sha256, row->ppm);
    }
    fclose(out);

    stats_path = g_strdup_printf("%s.stats.json", agfx_clockab_file);
    stats = fopen(stats_path, "w");
    if (stats) {
        fprintf(stats,
            "{\"schema\":\"CLOCKAB_GFXR_BRIDGE_V1\","
            "\"kind\":\"qemu_apply_stats\",\"rows\":%" PRIu64 ","
            "\"dropped\":%" PRIu64 ",\"rejected\":%" PRIu64 ","
            "\"result\":\"%s\"}\n",
            agfx_clockab_count, agfx_clockab_dropped, agfx_clockab_rejected,
            (agfx_clockab_dropped == 0 && agfx_clockab_rejected == 0)
                ? "COMPLETE" : "INCOMPLETE");
        fclose(stats);
    }
}

static void agfx_clockab_init(void)
{
    if (agfx_clockab_init_done) {
        return;
    }
    agfx_clockab_init_done = 1;
    agfx_clockab_file = getenv("AGFX_CLOCKAB_GFXR_SIDECAR");
    if (!agfx_clockab_file || !agfx_clockab_file[0]) {
        agfx_clockab_file = NULL;
        return;
    }
    const char *count_env = getenv("AGFX_CLOCKAB_GFXR_CAPTURE_COUNT");
    if (count_env && count_env[0]) {
        char *end = NULL;
        errno = 0;
        unsigned long long parsed = strtoull(count_env, &end, 10);
        if (errno == 0 && end && *end == '\0' && parsed > 0 &&
            parsed <= AGFX_CLOCKAB_BRIDGE_ROWS) {
            agfx_clockab_capture_count = parsed;
        }
    }
    atexit(agfx_clockab_flush);
}

static bool agfx_clockab_runtime_texel_bridge_requested(void)
{
    const char *value = getenv("QMU_CLOCKAB_RUNTIME_TEXEL_BRIDGE");
    return value && (strcmp(value, "1") == 0 || strcmp(value, "yes") == 0 ||
                     strcmp(value, "true") == 0);
}

static bool agfx_clockab_terminal_operand_bridge_requested(void)
{
    const char *value = getenv("QMU_CLOCKAB_TERMINAL_OPERAND_BRIDGE");
    return value && (strcmp(value, "1") == 0 || strcmp(value, "yes") == 0 ||
                     strcmp(value, "true") == 0);
}

static int agfx_clockab_runtime_texel_flush_window(AppleGfxMLState *s)
{
    struct qmu_vulkan_ctx *vk;

    if (!s || !s->qmu_dev) {
        return -1;
    }
    vk = qmu_session_get_vulkan(s->qmu_dev);
    if (!vk) {
        return -1;
    }
    return qmu_vk_clockab_runtime_texel_flush_window(vk);
}

static int agfx_clockab_terminal_operand_flush_window(AppleGfxMLState *s)
{
    struct qmu_vulkan_ctx *vk;

    if (!s || !s->qmu_dev) {
        return -1;
    }
    vk = qmu_session_get_vulkan(s->qmu_dev);
    if (!vk) {
        return -1;
    }
    return qmu_vk_clockab_terminal_operand_flush_window(vk);
}

static int agfx_clockab_terminal_operand_begin_window(AppleGfxMLState *s)
{
    struct qmu_vulkan_ctx *vk;

    if (!s || !s->qmu_dev) {
        return -1;
    }
    vk = qmu_session_get_vulkan(s->qmu_dev);
    if (!vk) {
        return -1;
    }
    return qmu_vk_clockab_terminal_operand_begin_window(vk);
}

static void agfx_clockab_flush_armed_window_if_complete(AppleGfxMLState *s)
{
    if (!agfx_clockab_file || agfx_clockab_window_flushed ||
        agfx_clockab_capture_count == 0) {
        return;
    }
    const uint64_t observed = agfx_clockab_count + agfx_clockab_rejected +
        agfx_clockab_dropped;
    if (observed < agfx_clockab_capture_count) {
        return;
    }
    /* These are the sole bounded-window writes.  The runtime texel snapshot
     * precedes the sidecar/stats flush, so the controller cannot observe a
     * complete QEMU window and issue QMP quit while the pixel witness is still
     * only process-local memory. */
    if (agfx_clockab_runtime_texel_bridge_requested() &&
        agfx_clockab_runtime_texel_flush_window(s) != 1) {
        agfx_clockab_rejected++;
    }
    if (agfx_clockab_terminal_operand_bridge_requested() &&
        agfx_clockab_terminal_operand_flush_window(s) != 1) {
        agfx_clockab_rejected++;
    }
    agfx_clockab_flush();
    agfx_clockab_window_flushed = true;
}

static bool agfx_clockab_roi_sha256(const uint8_t *fb, uint32_t width,
                                    uint32_t height, uint32_t stride,
                                    char out_hex[65])
{
    const uint32_t rw = AGFX_RCPT_ROI_X1 - AGFX_RCPT_ROI_X0;
    const uint32_t rh = AGFX_RCPT_ROI_Y1 - AGFX_RCPT_ROI_Y0;
    uint8_t rgb_row[(AGFX_RCPT_ROI_X1 - AGFX_RCPT_ROI_X0) * 3];
    agfx_sha256 hash;
    uint8_t digest[32];

    if (!fb || !out_hex || width < AGFX_RCPT_ROI_X1 ||
        height < AGFX_RCPT_ROI_Y1 || stride < width * 4) {
        return false;
    }
    /* This probe runs on the present BH.  Hash one canonical RGB row at a
     * time instead of allocating a whole ROI per present: the PPM capture is
     * already explicitly armed, but the bridge must not add a heap-allocation
     * timing source that could perturb the observed A/B cadence. */
    agfx_sha256_init(&hash);
    for (uint32_t ry = 0; ry < rh; ++ry) {
        const uint8_t *src = fb + (size_t)(AGFX_RCPT_ROI_Y0 + ry) * stride +
            (size_t)AGFX_RCPT_ROI_X0 * 4;
        for (uint32_t rx = 0; rx < rw; ++rx) {
            rgb_row[rx * 3 + 0] = src[rx * 4 + 2];
            rgb_row[rx * 3 + 1] = src[rx * 4 + 1];
            rgb_row[rx * 3 + 2] = src[rx * 4 + 0];
        }
        agfx_sha256_update(&hash, rgb_row, (size_t)rw * 3);
    }
    agfx_sha256_final(&hash, digest);
    for (int index = 0; index < 32; ++index) {
        snprintf(out_hex + index * 2, 3, "%02x", digest[index]);
    }
    return true;
}

static void agfx_clockab_append(const AppleGfxMLCaptureBridge *bridge,
                                const char *roi_sha256, const char *ppm)
{
    if (!agfx_clockab_file) {
        return;
    }
    if (!bridge || !roi_sha256 || !ppm || !ppm[0] ||
        bridge->clockab_metadata_version != 1 ||
        bridge->clockab_boot_uuid_lo == 0 || bridge->clockab_boot_uuid_hi == 0 ||
        bridge->clockab_selection_seq == 0 ||
        bridge->clockab_qmetal_submit_seq == 0 ||
        bridge->clockab_qmetal_queue_submit_seq == 0 ||
        bridge->clockab_source_raw_image == 0 ||
        bridge->clockab_source_generation == 0 ||
        bridge->clockab_source_kind != 1 ||
        bridge->clockab_source_format != 1 ||
        bridge->clockab_source_width == 0 || bridge->clockab_source_height == 0 ||
        bridge->clockab_source_rect_x != 0 || bridge->clockab_source_rect_y != 0 ||
        bridge->clockab_destination_rect_x != 0 || bridge->clockab_destination_rect_y != 0 ||
        bridge->clockab_source_rect_width != bridge->clockab_source_width ||
        bridge->clockab_source_rect_height != bridge->clockab_source_height ||
        bridge->clockab_destination_rect_width != bridge->clockab_source_width ||
        bridge->clockab_destination_rect_height != bridge->clockab_source_height ||
        bridge->clockab_orientation != 0 || bridge->clockab_filter != 1 ||
        bridge->clockab_copy_path != 1 || bridge->host_apply_seq == 0 ||
        bridge->qemu_present_count == 0) {
        agfx_clockab_rejected++;
        return;
    }
    if (agfx_clockab_count >= AGFX_CLOCKAB_BRIDGE_ROWS) {
        agfx_clockab_dropped++;
        return;
    }
    AgfxClockabBridgeRow *row = &agfx_clockab_rows[agfx_clockab_count++];
    row->bridge = *bridge;
    row->present_seq = agfx_clockab_next_present_seq++;
    pstrcpy(row->roi_sha256, sizeof(row->roi_sha256), roi_sha256);
    pstrcpy(row->ppm, sizeof(row->ppm), ppm);
}

static void agfx_capture_bridge_init_once(void)
{
    if (g_once_init_enter(&agfx_capture_bridge_inited)) {
        qemu_mutex_init(&agfx_capture_bridge_mutex);
        memset(&agfx_capture_bridge_latest, 0, sizeof(agfx_capture_bridge_latest));
        agfx_capture_bridge_next_apply_seq = 1;
        agfx_capture_bridge_next_qmp_ordinal = 1;
        g_once_init_leave(&agfx_capture_bridge_inited, 1);
    }
}

static AppleGfxMLCaptureBridge
agfx_capture_bridge_note_apply(AppleGfxMLState *s,
                               const AppleGfxMLFrameCompletionJob *job,
                               uint64_t present_count)
{
    AppleGfxMLCaptureBridge bridge = { 0 };

    if (!s || !job) {
        return bridge;
    }

    agfx_capture_bridge_init_once();
    bridge = job->bridge;
    qemu_mutex_lock(&agfx_capture_bridge_mutex);
    bridge.host_apply_seq = agfx_capture_bridge_next_apply_seq++;
    bridge.qemu_frame_count = s->frame_count;
    bridge.qemu_present_count = present_count;
    agfx_capture_bridge_latest = bridge;
    qemu_mutex_unlock(&agfx_capture_bridge_mutex);
    return bridge;
}

bool apple_gfx_ml_get_qmp_capture_bridge(AppleGfxMLCaptureBridge *out)
{
    if (!out) {
        return false;
    }
    agfx_capture_bridge_init_once();
    qemu_mutex_lock(&agfx_capture_bridge_mutex);
    *out = agfx_capture_bridge_latest;
    out->qmp_ordinal = agfx_capture_bridge_next_qmp_ordinal++;
    qemu_mutex_unlock(&agfx_capture_bridge_mutex);
    return out->host_apply_seq != 0;
}

static void agfx_free_frame_completion_job(AppleGfxMLFrameCompletionJob *job)
{
    if (!job) {
        return;
    }

    g_free(job->pixels);
    g_free(job);
}

typedef struct AgfxRenderSubmitJob {
    AppleGfxMLState *state;
} AgfxRenderSubmitJob;

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

/* A direct shared MTLBuffer is imported over a pinned guest-RAM HVA.  The GPU
 * can write that allocation without a later CPU copy, but QEMU still needs a
 * main-loop/BQL dirty-accounting edge before the guest observes completion.
 * This deliberately does not release the mapping: Buffer::importGuestPages
 * owns that lifetime and qemu_unmap_gpa remains its sole unpin path. */
typedef struct AgfxMappedDirtyJob {
    void *hva;
    size_t size;
    int result;
    QemuEvent event;
} AgfxMappedDirtyJob;

static void agfx_mark_mapped_dirty_bh(void *opaque)
{
    AgfxMappedDirtyJob *job = opaque;
    if (!job || !job->hva || job->size == 0) {
        if (job) {
            job->result = -1;
            qemu_event_set(&job->event);
        }
        return;
    }

    ram_addr_t offset;
    MemoryRegion *mr = memory_region_from_host(job->hva, &offset);
    if (!mr) {
        job->result = -1;
    } else {
        memory_region_set_dirty(mr, offset, job->size);
        memory_region_unref(mr);
        job->result = 0;
    }
    qemu_event_set(&job->event);
}

static int qemu_mark_mapped_memory_dirty(void *ctx, void *hva, size_t size)
{
    AppleGfxMLState *s = ctx;
    if (!s || !hva || size == 0) {
        return -1;
    }
    AgfxMappedDirtyJob job = {
        .hva = hva,
        .size = size,
        .result = -1,
    };
    qemu_event_init(&job.event, false);
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            agfx_mark_mapped_dirty_bh, &job);
    qemu_event_wait(&job.event);
    qemu_event_destroy(&job.event);
    return job.result;
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

static void agfx_merge_bootstrap_present_source_locked(AppleGfxMLState *s);
static bool agfx_take_bootstrap_present_source_locked(AppleGfxMLState *s);
static struct AgfxBootstrapPresentCommand *
agfx_take_bootstrap_present_command_locked(AppleGfxMLState *s);
static void apple_gfx_ml_frame_completed_bh(void *opaque);
static void apple_gfx_ml_cursor_glyph_bh(void *opaque);
static void apple_gfx_ml_cursor_move_bh(void *opaque);
static void apple_gfx_ml_cursor_show_bh(void *opaque);
static void agfx_new_frame_handler_bh(void *opaque);
static void agfx_schedule_frame_presents(AppleGfxMLState *s);
static void agfx_schedule_frame_presents_locked(AppleGfxMLState *s, bool *started);
static void agfx_cancel_frame_presents(AppleGfxMLState *s);
static void agfx_cancel_frame_presents_locked(AppleGfxMLState *s);
static void agfx_free_bootstrap_present_commands_locked(AppleGfxMLState *s);
static void agfx_enqueue_session_job(AppleGfxMLState *s,
                                     AppleGfxMLSessionJob *job);
static void qemu_render_frame_complete(void *ctx,
                                       const qmu_render_frame_completion *completion);
static void qemu_frame_completed(void *ctx, int frame_expected);
static int agfx_render_submit_job(void *opaque);

static void agfx_kick_display_render(AppleGfxMLState *s, struct qmu_vulkan_ctx *vk)
{
    int rc;
    AgfxRenderSubmitJob *job;

    if (!s || !vk) {
        return;
    }

    /* Reference apple_gfx_render_new_frame captures the exact current frame on
     * the BH edge, then dispatches encode/submit to a background queue because
     * the owner render path is not safe to run inline under the BH/BQL edge. */
    qemu_mutex_lock(&s->session_mutex);
    rc = qmu_vk_capture_display_frame_request(vk);
    qemu_mutex_unlock(&s->session_mutex);
    if (rc > 0) {
        if (agfx_log_should_emit(&s->render_worker_log_count)) {
            agfx_log(s,
                     "[apple-gfx-ml] kick_display_render: captured_display_request pending_frames=%d mmio_wait=%d\n",
                     __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST),
                     qatomic_read(&s->mmio_wait_active));
        }
        job = g_new0(AgfxRenderSubmitJob, 1);
        job->state = s;
        thread_pool_submit_immediate(s->render_pool,
                                     agfx_render_submit_job,
                                     job,
                                     g_free);
        return;
    }

    if (agfx_log_should_emit(&s->render_worker_log_count)) {
        agfx_log(s,
                 "[apple-gfx-ml] kick_display_render: capture_display_request rc=%d pending_frames=%d\n",
                 rc,
                 __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST));
    }
    {
        qemu_frame_completed(s, 0);
    }
}

static int agfx_render_submit_job(void *opaque)
{
    AgfxRenderSubmitJob *job = opaque;
    AppleGfxMLState *s = job ? job->state : NULL;
    struct qmu_vulkan_ctx *vk;
    int rc;

    if (!s || !s->qmu_dev) {
        return 0;
    }

    vk = qmu_session_get_vulkan(s->qmu_dev);
    if (!vk) {
        qemu_frame_completed(s, 0);
        return 0;
    }

    qemu_mutex_lock(&s->session_mutex);
    rc = qmu_vk_submit_captured_display_frame(vk);
    qemu_mutex_unlock(&s->session_mutex);

    if (rc > 0) {
        if (agfx_log_should_emit(&s->render_worker_log_count)) {
            agfx_log(s,
                     "[apple-gfx-ml] render_worker: submit_captured_display_frame pending_frames=%d mmio_wait=%d\n",
                     __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST),
                     qatomic_read(&s->mmio_wait_active));
        }
        return 0;
    }

    if (agfx_log_should_emit(&s->render_worker_log_count)) {
        agfx_log(s,
                 "[apple-gfx-ml] render_worker: submit_captured_display_frame rc=%d pending_frames=%d\n",
                 rc,
                 __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST));
    }
    qemu_frame_completed(s, 0);

    return 0;
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

typedef enum AgfxBootstrapPresentCommandKind {
    AGFX_BOOTSTRAP_PRESENT_CMD_SCHEDULE,
    AGFX_BOOTSTRAP_PRESENT_CMD_CANCEL,
} AgfxBootstrapPresentCommandKind;

typedef struct AgfxBootstrapPresentCommand {
    struct AgfxBootstrapPresentCommand *next;
    AgfxBootstrapPresentCommandKind kind;
} AgfxBootstrapPresentCommand;

static void agfx_display_completion_bh(void *opaque)
{
    AgfxCompletionJob *job = opaque;

    if (!job) {
        return;
    }

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

static void apple_gfx_ml_apply_mode_change(AppleGfxMLState *s,
                                           uint32_t width,
                                           uint32_t height,
                                           uint32_t iosurface_pixel_format,
                                           uint64_t protection_requirements)
{
    if (!s || width == 0 || height == 0) {
        return;
    }

    agfx_log(s,
             "[apple-gfx-ml] mode_change: %ux%u iosurface_pf=0x%08x protection=0x%016" PRIx64 "\n",
             width,
             height,
             iosurface_pixel_format,
             protection_requirements);

    agfx_publish_display_mode(s,
                              width,
                              height,
                              iosurface_pixel_format,
                              protection_requirements);
}

static bool apple_gfx_ml_apply_staged_frame(AppleGfxMLState *s,
                                            AppleGfxMLFrameCompletionJob *job)
{
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    size_t frame_size = 0;
    AppleGfxMLCaptureBridge applied_bridge = { 0 };
    bool clockab_capture_armed = false;
    bool clockab_ppm_written = false;
    char clockab_ppm[64] = { 0 };

    if (!s || !job || !s->qmu_dev) {
        return false;
    }

    /* Reference apple_gfx_render_frame_completed_bh applies the rendered
     * texture that belongs to the completed command buffer. qmetal delivers
     * that immutable per-submit payload to this callback; copy it into the
     * visible QEMU surface on the BH edge instead of late-reading a mutable
     * global latest-frame slot that may already hold another completion. */
    if (s->rendering_frame_width != s->fb_width ||
        s->rendering_frame_height != s->fb_height) {
        return false;
    }

    if (!job->pixels || job->width == 0 || job->height == 0 || job->stride == 0) {
        return false;
    }

    width = job->width;
    height = job->height;
    stride = job->stride;
    frame_size = (size_t)height * stride;

    agfx_publish_display_mode(s, width, height, s->fb_iosurface_pixel_format,
                              s->fb_protection_requirements);

    if (frame_size > s->display_fb_size) {
        return false;
    }
    memcpy(s->display_fb, job->pixels, frame_size);

    /* Update frame counter and log. This is the wrapper analogue of reference
     * frame_completed_bh copying from the completed display texture into the
     * visible surface on the BH edge. */
    s->frame_count++;

    /* AGFX_FRAME_CAPTURE_DIR_ALL — env-gated per-present PPM capture for the
     * frame-480 determinism comparison (aiam methodology). Inert when unset.
     * Writes one P6 PPM per present as frame_%06lu.ppm keyed on frame_count,
     * from the same visible display_fb payload QEMU shows. */
    {
        static const char *agfx_cap_dir;
        static int agfx_cap_init;
        static int agfx_cap_ok;
        static unsigned long agfx_cap_start;
        if (!agfx_cap_init) {
            const char *env = getenv("AGFX_FRAME_CAPTURE_DIR_ALL");
            if (env && env[0]) {
                agfx_cap_dir = env;
                if (mkdir(env, 0755) == 0 || errno == EEXIST) {
                    agfx_cap_ok = 1;
                }
            }
            /* AGFX_FRAME_CAPTURE_START — skip captures below this frame index.
             * Per-present capture on the BH edge delays the interrupt BH on
             * the same main loop and visibly starves guest display-link
             * pacing during boot; a windowed capture (settled tail only)
             * keeps the measurement without perturbing the bring-up. */
            const char *start_env = getenv("AGFX_FRAME_CAPTURE_START");
            if (start_env && start_env[0]) {
                agfx_cap_start = strtoul(start_env, NULL, 0);
            }
            agfx_cap_init = 1;
        }
        if (agfx_cap_ok && agfx_cap_dir &&
            (unsigned long)s->frame_count >= agfx_cap_start) {
            clockab_capture_armed = true;
            char ppm_path[PATH_MAX];
            int pn = snprintf(ppm_path, sizeof(ppm_path), "%s/frame_%06lu.ppm",
                              agfx_cap_dir, (unsigned long)s->frame_count);
            if (pn > 0 && pn < (int)sizeof(ppm_path)) {
                FILE *cf = fopen(ppm_path, "wb");
                if (cf) {
                    bool complete = true;
                    fprintf(cf, "P6\n%u %u\n255\n", width, height);
                    const uint8_t *csrc = (const uint8_t *)s->display_fb;
                    uint8_t *rgb_row = g_malloc((size_t)width * 3);
                    for (uint32_t cy = 0; cy < height; cy++) {
                        const uint8_t *crow = csrc + (size_t)cy * stride;
                        for (uint32_t cx = 0; cx < width; cx++) {
                            rgb_row[cx * 3 + 0] = crow[cx * 4 + 2];
                            rgb_row[cx * 3 + 1] = crow[cx * 4 + 1];
                            rgb_row[cx * 3 + 2] = crow[cx * 4 + 0];
                        }
                        if (fwrite(rgb_row, 1, (size_t)width * 3, cf) !=
                            (size_t)width * 3) {
                            complete = false;
                            break;
                        }
                    }
                    g_free(rgb_row);
                    if (fclose(cf) != 0) {
                        complete = false;
                    }
                    if (complete) {
                        int nn = snprintf(clockab_ppm, sizeof(clockab_ppm),
                                          "frame_%06lu.ppm",
                                          (unsigned long)s->frame_count);
                        clockab_ppm_written = nn > 0 &&
                            nn < (int)sizeof(clockab_ppm);
                    }
                }
            }
        }
    }
    {
        uint64_t pc = qatomic_fetch_inc(&s->present_count) + 1;
        if (pc <= AGFX_LOG_INITIAL_COUNT || (pc % AGFX_LOG_INTERVAL) == 0) {
            agfx_log(s,
                     "[apple-gfx-ml] present_frame #%lu: %ux%u stride=%u (owner payload)\n",
                     (unsigned long)pc,
                     width,
                     height,
                     stride);
        }
        applied_bridge = agfx_capture_bridge_note_apply(s, job, pc);

        /* The bridge joins only the PPM written from this exact BH payload
         * with the completion metadata QMetal attached to this exact apply.
         * If either side is absent, preserve rendering and mark diagnostic
         * evidence incomplete at final flush; never infer it by frame order. */
        agfx_clockab_init();
        if (agfx_clockab_file && clockab_capture_armed &&
            !agfx_clockab_window_flushed) {
            char roi_sha256[65] = { 0 };
            if (!clockab_ppm_written ||
                !agfx_clockab_roi_sha256((const uint8_t *)s->display_fb,
                                          width, height, stride, roi_sha256)) {
                agfx_clockab_rejected++;
            } else if (agfx_clockab_terminal_operand_bridge_requested() &&
                       !agfx_clockab_terminal_operand_window_started &&
                       agfx_clockab_terminal_operand_begin_window(s) != 1) {
                agfx_clockab_rejected++;
            } else {
                if (agfx_clockab_terminal_operand_bridge_requested()) {
                    agfx_clockab_terminal_operand_window_started = true;
                }
                agfx_clockab_append(&applied_bridge, roi_sha256, clockab_ppm);
            }
            agfx_clockab_flush_armed_window_if_complete(s);
        }

        /* AGFX_PRESENT_RECEIPT row (spec v2.2): same display_fb bytes as PPM. */
        agfx_rcpt_init();
        if (agfx_rcpt_file) {
            char sha_hex[65] = "-";
            if (s->display_fb && width >= AGFX_RCPT_ROI_X1 &&
                height >= AGFX_RCPT_ROI_Y1) {
                const uint32_t rw = AGFX_RCPT_ROI_X1 - AGFX_RCPT_ROI_X0;
                const uint32_t rh = AGFX_RCPT_ROI_Y1 - AGFX_RCPT_ROI_Y0;
                uint8_t *roi = g_malloc((size_t)rw * rh * 3);
                const uint8_t *fb = (const uint8_t *)s->display_fb;
                for (uint32_t ry = 0; ry < rh; ry++) {
                    const uint8_t *row =
                        fb + (size_t)(AGFX_RCPT_ROI_Y0 + ry) * stride +
                        (size_t)AGFX_RCPT_ROI_X0 * 4;
                    uint8_t *dst = roi + (size_t)ry * rw * 3;
                    for (uint32_t rx = 0; rx < rw; rx++) {
                        dst[rx * 3 + 0] = row[rx * 4 + 2]; /* R */
                        dst[rx * 3 + 1] = row[rx * 4 + 1]; /* G */
                        dst[rx * 3 + 2] = row[rx * 4 + 0]; /* B */
                    }
                }
                {
                    agfx_sha256 hc;
                    uint8_t digest[32];
                    agfx_sha256_init(&hc);
                    agfx_sha256_update(&hc, roi, (size_t)rw * rh * 3);
                    agfx_sha256_final(&hc, digest);
                    for (int di = 0; di < 32; di++) {
                        snprintf(sha_hex + di * 2, 3, "%02x", digest[di]);
                    }
                }
                g_free(roi);
            }
            if (agfx_rcpt_fp) {
                fprintf(agfx_rcpt_fp,
                    "%" PRIu64 "\t%" PRIu64 "\t%d\t%" PRIu64 "\t%" PRIu64
                    "\t%u\t0x%" PRIx64 "\t0x%" PRIx64 "\t%" PRIu64
                    "\t%u\t%" PRIu64 "\t0x%" PRIx64 "\t%u\t%u"
                    "\t%" PRIu64 "\t%" PRIu64 "\t%" PRIu64 "\t%" PRIu64
                    "\t%" PRIu64 "\t0x%" PRIx64 "\t0x%" PRIx64 "\t%s\tframe_%06lu.ppm\n",
                    pc,
                    applied_bridge.host_apply_seq,
                    job->bridge.valid ? 1 : 0,
                    job->bridge.qmetal_host_delivery_seq,
                    job->bridge.display_cookie,
                    job->bridge.source_texture_id,
                    (uint64_t)job->bridge.source_vk_image,
                    (uint64_t)job->bridge.source_vk_image_view,
                    job->bridge.image_lifetime_generation,
                    job->bridge.backing_id,
                    job->bridge.backing_generation,
                    (uint64_t)job->bridge.backing_va,
                    job->bridge.backing_task,
                    job->bridge.backing_resource,
                    job->bridge.writer_seq,
                    job->bridge.queue_submit_seq,
                    job->bridge.request_epoch,
                    job->bridge.frame_serial,
                    job->bridge.txn3_seq,
                    job->bridge.txn3_digest_lo,
                    job->bridge.txn3_digest_hi,
                    sha_hex,
                    (unsigned long)s->frame_count);
                if ((++agfx_rcpt_rowcount & 127) == 0) {
                    fflush(agfx_rcpt_fp);
                }
            }
        }
    }
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

static void apple_gfx_ml_frame_completed_bh(void *opaque)
{
    AppleGfxMLFrameCompletionJob *job = opaque;
    AppleGfxMLState *s = job ? job->state : NULL;
    bool frame_applied;

    if (!job || !s) {
        agfx_free_frame_completion_job(job);
        return;
    }

    {
        int pending = __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST);
        if (pending > 0) {
            pending = __atomic_sub_fetch(&s->pending_frames, 1, __ATOMIC_SEQ_CST);
        }
        job->chain_needed = pending > 0;
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

static void qemu_render_frame_complete(void *ctx,
                                       const qmu_render_frame_completion *completion)
{
    AppleGfxMLState *s = ctx;
    AppleGfxMLFrameCompletionJob *job;

    if (!s || !completion) {
        return;
    }

    job = g_new0(AppleGfxMLFrameCompletionJob, 1);
    job->state = s;
    job->frame_expected = completion->frame_expected != 0;
    job->bridge.valid = completion->seq_metadata_version != 0;
    job->bridge.qmetal_host_delivery_seq = completion->host_delivery_seq;
    job->bridge.display_cookie = completion->display_submit_seq;
    job->bridge.txn3_seq = completion->txn3_seq;
    job->bridge.txn3_digest_lo = completion->txn3_digest_lo;
    job->bridge.txn3_digest_hi = completion->txn3_digest_hi;
    job->bridge.source_texture_id = completion->source_texture_id;
    job->bridge.source_vk_image = completion->source_vk_image;
    job->bridge.source_vk_image_view = completion->source_vk_image_view;
    job->bridge.source_aspect = completion->source_aspect;
    job->bridge.source_mip = completion->source_mip;
    job->bridge.source_layer = completion->source_layer;
    job->bridge.image_lifetime_generation =
        completion->image_lifetime_generation;
    job->bridge.backing_id = completion->backing_id;
    job->bridge.backing_generation = completion->backing_generation;
    job->bridge.backing_va = completion->backing_va;
    job->bridge.backing_span = completion->backing_span;
    job->bridge.backing_task = completion->backing_task;
    job->bridge.backing_resource = completion->backing_resource;
    job->bridge.backing_plane = completion->backing_plane;
    job->bridge.writer_seq = completion->writer_seq;
    job->bridge.queue_submit_seq = completion->queue_submit_seq;
    job->bridge.request_epoch = completion->request_epoch;
    job->bridge.frame_serial = completion->frame_serial;
    job->bridge.clockab_metadata_version = completion->clockab_metadata_version;
    job->bridge.clockab_source_kind = completion->clockab_source_kind;
    job->bridge.clockab_boot_uuid_lo = completion->clockab_boot_uuid_lo;
    job->bridge.clockab_boot_uuid_hi = completion->clockab_boot_uuid_hi;
    job->bridge.clockab_selection_seq = completion->clockab_selection_seq;
    job->bridge.clockab_qmetal_submit_seq =
        completion->clockab_qmetal_submit_seq;
    job->bridge.clockab_qmetal_queue_submit_seq =
        completion->clockab_qmetal_queue_submit_seq;
    job->bridge.clockab_source_raw_image = completion->clockab_source_raw_image;
    job->bridge.clockab_source_generation =
        completion->clockab_source_generation;
    job->bridge.clockab_source_format = completion->clockab_source_format;
    job->bridge.clockab_source_width = completion->clockab_source_width;
    job->bridge.clockab_source_height = completion->clockab_source_height;
    job->bridge.clockab_source_rect_x = completion->clockab_source_rect_x;
    job->bridge.clockab_source_rect_y = completion->clockab_source_rect_y;
    job->bridge.clockab_source_rect_width =
        completion->clockab_source_rect_width;
    job->bridge.clockab_source_rect_height =
        completion->clockab_source_rect_height;
    job->bridge.clockab_destination_rect_x =
        completion->clockab_destination_rect_x;
    job->bridge.clockab_destination_rect_y =
        completion->clockab_destination_rect_y;
    job->bridge.clockab_destination_rect_width =
        completion->clockab_destination_rect_width;
    job->bridge.clockab_destination_rect_height =
        completion->clockab_destination_rect_height;
    job->bridge.clockab_orientation = completion->clockab_orientation;
    job->bridge.clockab_filter = completion->clockab_filter;
    job->bridge.clockab_copy_path = completion->clockab_copy_path;
    job->bridge.clockab_composite_flags = completion->clockab_composite_flags;
    if (job->frame_expected && completion->pixels &&
        completion->width != 0 && completion->height != 0 &&
        completion->stride != 0) {
        const size_t frame_size = (size_t)completion->height * completion->stride;
        job->pixels = g_memdup2(completion->pixels, frame_size);
        job->width = completion->width;
        job->height = completion->height;
        job->stride = completion->stride;
    }
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            apple_gfx_ml_frame_completed_bh, job);
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
} AppleGfxMLCursorMoveJob;

static void apple_gfx_ml_update_cursor(AppleGfxMLState *s, uint32_t display_id)
{
    uint32_t packed = 0xffffffffu;

    if (!s->con) {
        return;
    }

    if (s->qmu_dev &&
        qmu_get_display_cursor_position(s->qmu_dev, display_id, &packed) == QMU_OK) {
        s->cursor_x = (int16_t)(packed & 0xffffu);
        s->cursor_y = (int16_t)((packed >> 16) & 0xffffu);
    }
    dpy_mouse_set(s->con, s->cursor_x, s->cursor_y, s->cursor_show);
}

static void apple_gfx_ml_cursor_glyph_bh(void *opaque)
{
    AppleGfxMLCursorGlyphJob *job = opaque;
    AppleGfxMLState *s = job ? job->state : NULL;
    const uint8_t *src;
    size_t row_padding = 0;

    if (!job || !s || !job->pixels) {
        if (job) {
            g_free(job->pixels);
        }
        g_free(job);
        return;
    }

    src = job->pixels;
    if (job->stride >= (uint64_t)job->width * 4u) {
        row_padding = (size_t)(job->stride -
                               (uint64_t)job->width * 4u);
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
             job->width,
             job->height,
             job->stride,
             job->hot_x,
             job->hot_y,
             job->sum);

    if (s->con) {
        dpy_cursor_define(s->con, s->cursor);
        apple_gfx_ml_update_cursor(s, s->cursor_display_id);
    }
    g_free(job->pixels);
    g_free(job);
}

static void apple_gfx_ml_cursor_show_bh(void *opaque)
{
    AppleGfxMLCursorShowJob *job = opaque;
    AppleGfxMLState *s = job ? job->state : NULL;

    if (!job || !s) {
        g_free(job);
        return;
    }

    s->cursor_show = job->visible;
    s->cursor_display_id = job->display_id;
    agfx_log(s, "[apple-gfx-ml] cursor_show: display=%u visible=%d\n",
             job->display_id,
             job->visible ? 1 : 0);
    apple_gfx_ml_update_cursor(s, job->display_id);
    g_free(job);
}

static void apple_gfx_ml_cursor_move_bh(void *opaque)
{
    AppleGfxMLCursorMoveJob *job = opaque;
    AppleGfxMLState *s = job ? job->state : NULL;

    if (!job || !s) {
        g_free(job);
        return;
    }

    s->cursor_display_id = job->display_id;
    agfx_log(s, "[apple-gfx-ml] cursor_move: display=%u\n",
             job->display_id);
    apple_gfx_ml_update_cursor(s, job->display_id);
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
    if (!job->pixels) {
        g_free(job);
        return;
    }
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

static void qemu_cursor_move(void *ctx,
                             uint32_t display_id)
{
    AppleGfxMLState *s = ctx;
    AppleGfxMLCursorMoveJob *job;

    if (!s) {
        return;
    }

    job = g_new0(AppleGfxMLCursorMoveJob, 1);
    job->state = s;
    job->display_id = display_id;
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            apple_gfx_ml_cursor_move_bh, job);
}

static void qemu_mode_change(void *ctx,
                             uint32_t width,
                             uint32_t height,
                             uint32_t iosurface_pixel_format,
                             uint64_t protection_requirements)
{
    AppleGfxMLState *s = ctx;

    if (!s || width == 0 || height == 0) {
        return;
    }

    apple_gfx_ml_apply_mode_change(s,
                                   width,
                                   height,
                                   iosurface_pixel_format,
                                   protection_requirements);
}

static void qemu_display_live_takeover(void *ctx)
{
    AppleGfxMLState *s = ctx;

    if (!s) {
        return;
    }

    if (qatomic_read(&s->iosfc_bootstrap_active)) {
        agfx_log(s,
                 "[apple-gfx-ml] display_live_takeover: cancel bootstrap presents\n");
        agfx_cancel_frame_presents(s);
    }
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

static AgfxBootstrapPresentCommand *
agfx_take_bootstrap_present_command_locked(AppleGfxMLState *s)
{
    AgfxBootstrapPresentCommand *cmd;

    if (!s || !s->bootstrap_present_cmd_head) {
        return NULL;
    }

    cmd = s->bootstrap_present_cmd_head;
    s->bootstrap_present_cmd_head = cmd->next;
    if (!s->bootstrap_present_cmd_head) {
        s->bootstrap_present_cmd_tail = NULL;
    }
    cmd->next = NULL;
    return cmd;
}

static void agfx_free_bootstrap_present_commands_locked(AppleGfxMLState *s)
{
    AgfxBootstrapPresentCommand *cmd;

    if (!s) {
        return;
    }

    cmd = s->bootstrap_present_cmd_head;
    s->bootstrap_present_cmd_head = NULL;
    s->bootstrap_present_cmd_tail = NULL;
    while (cmd) {
        AgfxBootstrapPresentCommand *next = cmd->next;
        g_free(cmd);
        cmd = next;
    }
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
    AgfxBootstrapPresentCommand *cmd;

    if (!s) {
        return;
    }

    /* Reference scheduleFramePresents runs on PGEFIPresentQueue itself.
     * Queue one control command instead of mutating timer/source state from
     * the MMIO caller thread. */
    cmd = g_new0(AgfxBootstrapPresentCommand, 1);
    cmd->kind = AGFX_BOOTSTRAP_PRESENT_CMD_SCHEDULE;
    qemu_mutex_lock(&s->bootstrap_present_mutex);
    if (s->bootstrap_present_cmd_tail) {
        s->bootstrap_present_cmd_tail->next = cmd;
    } else {
        s->bootstrap_present_cmd_head = cmd;
    }
    s->bootstrap_present_cmd_tail = cmd;
    qemu_cond_signal(&s->bootstrap_present_cond);
    qemu_mutex_unlock(&s->bootstrap_present_mutex);
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
    AgfxBootstrapPresentCommand *cmd;

    if (!s) {
        return;
    }

    /* Reference cancelFramePresents is also queue-owned. Keep cancellation
     * ordered against any already queued schedule/merge work. */
    cmd = g_new0(AgfxBootstrapPresentCommand, 1);
    cmd->kind = AGFX_BOOTSTRAP_PRESENT_CMD_CANCEL;
    qemu_mutex_lock(&s->bootstrap_present_mutex);
    if (s->bootstrap_present_cmd_tail) {
        s->bootstrap_present_cmd_tail->next = cmd;
    } else {
        s->bootstrap_present_cmd_head = cmd;
    }
    s->bootstrap_present_cmd_tail = cmd;
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
        AgfxBootstrapPresentCommand *cmd;
        int64_t now_us;
        int64_t wait_ms;

        cmd = agfx_take_bootstrap_present_command_locked(s);
        if (cmd) {
            /* The serial present worker is the wrapper analogue of
             * PGEFIPresentQueue. It alone owns timer/source state mutation. */
            switch (cmd->kind) {
            case AGFX_BOOTSTRAP_PRESENT_CMD_SCHEDULE: {
                bool started = false;

                qatomic_set(&s->iosfc_bootstrap_active, 1);
                agfx_schedule_frame_presents_locked(s, &started);
                if (started) {
                    agfx_log(s,
                             "[apple-gfx-ml] display frame timer started (%dms interval, present queue)\n",
                             AGFX_DISPLAY_FRAME_INTERVAL_MS);
                }
                break;
            }
            case AGFX_BOOTSTRAP_PRESENT_CMD_CANCEL:
                qatomic_set(&s->iosfc_bootstrap_active, 0);
                agfx_cancel_frame_presents_locked(s);
                break;
            }
            g_free(cmd);
            continue;
        }

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

    if (!s || !s->qmu_dev) {
        return;
    }

    vk = qmu_session_get_vulkan(s->qmu_dev);
    if (!vk) {
        agfx_log(s, "[apple-gfx-ml] new_frame_handler_bh: vk=NULL\n");
        return;
    }

    if (agfx_log_should_emit(&s->new_frame_handler_log_count)) {
        agfx_log(s,
                 "[apple-gfx-ml] new_frame_handler_bh: pending_frames=%d mmio_wait=%d\n",
                 __atomic_load_n(&s->pending_frames, __ATOMIC_SEQ_CST),
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
    s->rendering_frame_width = s->fb_width;
    s->rendering_frame_height = s->fb_height;
    agfx_kick_display_render(s, vk);
}

static void qemu_new_frame_signal(void *ctx)
{
    AppleGfxMLState *s = ctx;
    struct qmu_vulkan_ctx *vk = NULL;

    if (!s) {
        return;
    }

    if (s->qmu_dev) {
        vk = qmu_session_get_vulkan(s->qmu_dev);
    }

    /* Reference dispatch_get_main_queue/newFrameEventHandler consumes the
     * mergeable signal when the callback itself runs, before the later BH
     * consumer. That exact edge existed briefly in 414f8bc358 and was later
     * moved to the BH edge by 974b600cdb; fresh VIS087 reruns show the BH-edge
     * lifetime still leaves a non-reference merge window and reopens
     * run-dependent extra new_frame_signal/new_frame_handler paths. */
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
            /* Do not hold session_mutex across qmu_mmio_write():
             * FIFO/KICK handling can synchronously call qemu_write_memory(),
             * which waits for a main-loop BH to run under BQL. The display BH
             * may concurrently call agfx_kick_display_render() under that same
             * BQL edge and take session_mutex first. Holding session_mutex here
             * creates a hard inversion:
             *   mmio worker:  session_mutex -> wait main loop/BQL
             *   display BH:   BQL -> wait session_mutex
             * qmu_mmio_write() already owns its internal mmio dispatch/serialize
             * domain, so wrapper-side session serialization must stay limited to
             * the capture/submit owner-render path itself. */
            qmu_mmio_write(job->state->qmu_dev,
                           (uint32_t)job->offset, job->value, job->size);
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
        agfx_schedule_frame_presents(s);
    } else if ((offset == PVG_REG_IOSFC_ENABLE && val == 0) ||
               offset == PVG_REG_IOSFC_UNMAP) {
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

    /* Optional PCI identity override.  pci_qdev_realize has already written
     * the class-default vendor/device into config space before this device
     * realize runs, so override it here.  This does not affect the MMIO/FIFO
     * host backend; it only changes which guest driver personality matches. */
    if (s->pci_vendor_id_override != 0xffffffff) {
        pci_config_set_vendor_id(pci_dev->config,
                                 (uint16_t)s->pci_vendor_id_override);
        agfx_log(s, "[apple-gfx-ml] PCI vendor id overridden to 0x%04x\n",
                 (uint16_t)s->pci_vendor_id_override);
    }
    if (s->pci_device_id_override != 0xffffffff) {
        pci_config_set_device_id(pci_dev->config,
                                 (uint16_t)s->pci_device_id_override);
        agfx_log(s, "[apple-gfx-ml] PCI device id overridden to 0x%04x\n",
                 (uint16_t)s->pci_device_id_override);
    }

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
    
    qemu_mutex_init(&s->mmio_job_mutex);
    qemu_mutex_init(&s->session_mutex);
    qemu_mutex_init(&s->bootstrap_present_mutex);
    qemu_cond_init(&s->bootstrap_present_cond);
    s->mmio_wait_active = 0;
    s->iosfc_bootstrap_active = 0;
    s->session_job_head = NULL;
    s->session_job_tail = NULL;
    s->render_pool = NULL;
    s->bootstrap_present_worker_stop = false;
    s->bootstrap_present_cmd_head = NULL;
    s->bootstrap_present_cmd_tail = NULL;
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
        .mark_mapped_memory_dirty = qemu_mark_mapped_memory_dirty,
        .read_memory = qemu_read_memory,
        .write_memory = qemu_write_memory,
        .raise_irq = qemu_raise_irq,
        .cursor_glyph = qemu_cursor_glyph,
        .cursor_move = qemu_cursor_move,
        .cursor_show = qemu_cursor_show,
        .mode_change = qemu_mode_change,
        .display_live_takeover = qemu_display_live_takeover,
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
        .frame_completed = NULL,
        .render_frame_complete = qemu_render_frame_complete,
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
    s->render_pool = thread_pool_new();
    qemu_thread_create(&s->bootstrap_present_worker, "agfx-present",
                       agfx_bootstrap_present_thread, s, QEMU_THREAD_JOINABLE);

    /* Create QEMU console */
    s->con = graphic_console_init(DEVICE(s), 0, &agfx_gfx_ops, s);
    
    /* Initialize display parameters */
    s->fb_width = s->display_width;
    s->fb_height = s->display_height;
    s->fb_stride = s->display_width * 4;
    s->rendering_frame_width = s->fb_width;
    s->rendering_frame_height = s->fb_height;
    
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

    if (s->render_pool) {
        thread_pool_free(s->render_pool);
        s->render_pool = NULL;
    }

    qemu_mutex_lock(&s->bootstrap_present_mutex);
    s->bootstrap_present_worker_stop = true;
    agfx_cancel_frame_presents_locked(s);
    agfx_free_bootstrap_present_commands_locked(s);
    qemu_cond_signal(&s->bootstrap_present_cond);
    qemu_mutex_unlock(&s->bootstrap_present_mutex);
    qemu_thread_join(&s->bootstrap_present_worker);

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

    /* Destroy qmetal device (stops display thread) */
    if (s->qmu_dev) {
        qmu_destroy(s->qmu_dev);
        s->qmu_dev = NULL;
    }

    qemu_mutex_destroy(&s->mmio_job_mutex);
    qemu_mutex_destroy(&s->session_mutex);
    qemu_cond_destroy(&s->bootstrap_present_cond);
    qemu_mutex_destroy(&s->bootstrap_present_mutex);
    qmu_log_set_callback(NULL, NULL);
    agfx_log_stop(s);

    /* Cleanup framebuffers */
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
    s->rendering_frame_width = 0;
    s->rendering_frame_height = 0;
    qemu_mutex_lock(&s->bootstrap_present_mutex);
    agfx_cancel_frame_presents_locked(s);
    agfx_free_bootstrap_present_commands_locked(s);
    qemu_mutex_unlock(&s->bootstrap_present_mutex);
    qatomic_set(&s->iosfc_bootstrap_active, 0);
    s->cursor_show = true;
    s->cursor_display_id = 0;
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
    s->bootstrap_present_worker_stop = false;
    s->bootstrap_present_cmd_head = NULL;
    s->bootstrap_present_cmd_tail = NULL;
    s->bootstrap_present_timer.active = false;
    s->bootstrap_present_timer.next_fire_us = 0;
    s->bootstrap_present_source.pending = false;
    s->pending_frames = 0;
    s->rendering_frame_width = 0;
    s->rendering_frame_height = 0;
    s->fb_iosurface_pixel_format = 0x42475241u;
    s->fb_protection_requirements = 0;
    s->display_fb = NULL;
    s->cursor = NULL;
    s->cursor_show = true;
    s->cursor_display_id = 0;
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
    s->render_pool = NULL;
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
