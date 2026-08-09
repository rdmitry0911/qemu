/*
 * apple-virgl virtio-gpu to qmetal bridge
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/iov.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "hw/virtio/virtio.h"
#include "hw/virtio/virtio-gpu.h"
#include "hw/virtio/apple-virgl-bridge.h"
#include "hw/virtio/apple-virgl-protocol.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qmu/qmetal_unified.h"

#define APPLE_VIRGL_QMU_ROOT_EXEC_INDIRECT3 0x2b
#define APPLE_VIRGL_QMU_ROOT_SET_OBJECT_LIST 0x33
#define APPLE_VIRGL_QMU_ROOT_GET_COMPUTE_INFO 0x3b
#define APPLE_VIRGL_QMU_ROOT_DELETE_RESOURCE 0x25
#define APPLE_VIRGL_QMU_GPU_UNMAP_MEMORY 0x22
#define APPLE_VIRGL_QMU_GPU_SYNCHRONIZE_RESOURCES 0x35
#define APPLE_VIRGL_QMU_GPU_MAP_MEMORY2 0x39
#define APPLE_VIRGL_QMU_DISPLAY_SET_SHARED_STATE 0x01
#define APPLE_VIRGL_QMU_DISPLAY_TRANSACTION3 0x07

typedef struct AppleVirglResourceState {
    uint32_t resource_id;
    uint64_t declared_size;
    const uint64_t *addrs;
    const struct iovec *iov;
    uint32_t iov_count;
    size_t backing_size;
    uint32_t direct_write_lease_count;
    bool direct_write_release_waiting;
} AppleVirglResourceState;

typedef struct AppleVirglMapping {
    uint64_t gpu_va;
    uint64_t length;
    uint32_t backing_resource_id;
    uint32_t backing_offset;
    uint32_t apple_resource_id;
} AppleVirglMapping;

typedef struct AppleVirglMappingWatch {
    uint32_t task_id;
    uint64_t gpu_va;
} AppleVirglMappingWatch;

/* Default-off, bounded observation of the existing direct callback.  This is
 * deliberately separate from mapping lifecycle logging: it records the
 * mapping the callback actually selects, rather than a prior MapMemory2
 * notification. */
typedef struct AppleVirglDirectReadWatch {
    uint32_t task_id;
    uint64_t gpu_va;
    uint32_t captures;
} AppleVirglDirectReadWatch;

typedef struct AppleVirglDirectReadProvenanceConfig {
    GArray *watches;
    uint32_t max_per_watch;
    uint64_t next_capture_id;
} AppleVirglDirectReadProvenanceConfig;

typedef struct AppleVirglDirectReadMappingSnapshot {
    uint32_t candidate_count;
    uint32_t selected_index;
    AppleVirglMapping selected;
    AppleVirglMapping last;
    uint64_t candidate_hash;
} AppleVirglDirectReadMappingSnapshot;

typedef struct AppleVirglDirectReadProvenanceRecord {
    uint64_t capture_id;
    uint32_t task_id;
    uint64_t gpu_va;
    size_t size;
    int result;
    AppleVirglDirectReadMappingSnapshot mappings;
    uint32_t resource_id;
    uint64_t resource_offset;
    size_t backing_size;
    uint32_t iov_count;
    uint64_t iov_layout_hash;
    uint64_t first_iov_addr;
    size_t first_iov_len;
    uint64_t middle_iov_addr;
    size_t middle_iov_len;
    uint64_t last_iov_addr;
    size_t last_iov_len;
} AppleVirglDirectReadProvenanceRecord;

typedef struct AppleVirglContextState {
    uint32_t transport_context_id;
    uint32_t task_id;
    bool task_bound;
    GHashTable *attached_resources;
    GArray *mappings;
} AppleVirglContextState;

typedef struct AppleVirglCompletionReceiver {
    VirtQueue *vq;
    VirtQueueElement *elem;
} AppleVirglCompletionReceiver;

typedef struct AppleVirglCompletionStamp {
    uint32_t channel_id;
    uint32_t stamp;
} AppleVirglCompletionStamp;

typedef struct AppleVirglDirectWriteLease {
    uint64_t token;
    uint32_t task_id;
    uint64_t gpu_va;
    uint64_t size;
    uint32_t resource_id;
    uint64_t resource_offset;
    const uint64_t *addrs;
    const struct iovec *iov;
    uint32_t iov_count;
    size_t backing_size;
} AppleVirglDirectWriteLease;

struct AppleVirglBridge {
    VirtIOGPU *gpu;
    QemuMutex lock;
    GHashTable *contexts;
    GHashTable *resources;
    GHashTable *direct_write_leases;
    qmu_session *session;
    uint64_t submit_count;
    uint64_t mapping_lifecycle_sequence;
    uint64_t frame_count;
    uint64_t next_direct_write_lease_token;
    QEMUBH *resource_release_bh;
    bool resource_release_bh_scheduled;
    bool resource_release_shutdown;
    bool resource_release_resetting;
    uint32_t resource_release_blocked_count;
    uint32_t resource_release_resume_count;
    QemuMutex completion_lock;
    GQueue completion_receivers;
    GQueue completion_stamps;
    QEMUBH *completion_bh;
    bool completion_bh_scheduled;
    bool completion_shutdown;
    bool completion_resetting;
};

static AppleVirglDirectReadProvenanceConfig
    apple_virgl_direct_read_provenance;
static gsize apple_virgl_direct_read_provenance_initialized;

static uint64_t apple_virgl_direct_read_fnv1a64_bytes(const void *bytes,
                                                       size_t size)
{
    const uint8_t *cursor = bytes;
    uint64_t hash = UINT64_C(0xcbf29ce484222325);
    size_t index;

    for (index = 0; index < size; ++index) {
        hash ^= cursor[index];
        hash *= UINT64_C(0x100000001b3);
    }
    return hash;
}

static uint64_t apple_virgl_direct_read_fnv1a64_u64(uint64_t hash,
                                                     uint64_t value)
{
    uint32_t byte;

    for (byte = 0; byte < sizeof(value); ++byte) {
        hash ^= value & UINT64_C(0xff);
        hash *= UINT64_C(0x100000001b3);
        value >>= 8;
    }
    return hash;
}

static void apple_virgl_direct_read_provenance_initialize(void)
{
    AppleVirglDirectReadProvenanceConfig *config =
        &apple_virgl_direct_read_provenance;
    const char *entries_text = g_getenv("QMU_DIAG_DIRECT_READ_PROVENANCE");
    const char *max_text = g_getenv("QMU_DIAG_DIRECT_READ_PROVENANCE_MAX");
    char *max_end;
    guint64 max_per_watch;
    g_auto(GStrv) tokens = NULL;
    char **token;

    config->watches = g_array_new(false, false,
                                  sizeof(AppleVirglDirectReadWatch));
    if (!entries_text || !*entries_text || !max_text || !*max_text) {
        return;
    }

    errno = 0;
    max_per_watch = g_ascii_strtoull(max_text, &max_end, 10);
    if (errno != 0 || max_end == max_text || *max_end != '\0' ||
        max_per_watch == 0 || max_per_watch > 8) {
        return;
    }
    tokens = g_strsplit(entries_text, ",", -1);
    if (!tokens || g_strv_length(tokens) == 0 || g_strv_length(tokens) > 8) {
        return;
    }

    for (token = tokens; *token; ++token) {
        char *entry = g_strstrip(*token);
        char *separator = strchr(entry, ':');
        char *task_end;
        char *va_end;
        guint64 task;
        guint64 gpu_va;
        AppleVirglDirectReadWatch watch;
        uint32_t index;

        if (!separator || strchr(separator + 1, ':')) {
            goto invalid;
        }
        *separator = '\0';
        errno = 0;
        task = g_ascii_strtoull(g_strstrip(entry), &task_end, 10);
        if (errno != 0 || task_end == entry || *task_end != '\0' ||
            task > UINT32_MAX) {
            goto invalid;
        }
        errno = 0;
        gpu_va = g_ascii_strtoull(g_strstrip(separator + 1), &va_end, 0);
        if (errno != 0 || va_end == separator + 1 || *va_end != '\0' ||
            gpu_va == 0 || (gpu_va & UINT64_C(0xfff)) != 0) {
            goto invalid;
        }
        watch.task_id = task;
        watch.gpu_va = gpu_va;
        watch.captures = 0;
        for (index = 0; index < config->watches->len; ++index) {
            AppleVirglDirectReadWatch *existing =
                &g_array_index(config->watches, AppleVirglDirectReadWatch,
                               index);

            if (existing->task_id == watch.task_id &&
                existing->gpu_va == watch.gpu_va) {
                goto invalid;
            }
        }
        g_array_append_val(config->watches, watch);
    }
    config->max_per_watch = max_per_watch;
    return;

invalid:
    g_array_set_size(config->watches, 0);
    config->max_per_watch = 0;
}

/* The bridge lock serializes direct callback invocation, so per-watch capture
 * accounting has no observable data-path synchronization or mutation. */
static uint64_t apple_virgl_direct_read_provenance_claim(uint32_t task_id,
                                                         uint64_t gpu_va)
{
    AppleVirglDirectReadProvenanceConfig *config =
        &apple_virgl_direct_read_provenance;
    uint64_t page_va = gpu_va & ~UINT64_C(0xfff);
    uint32_t index;

    if (g_once_init_enter(&apple_virgl_direct_read_provenance_initialized)) {
        apple_virgl_direct_read_provenance_initialize();
        g_once_init_leave(&apple_virgl_direct_read_provenance_initialized, 1);
    }
    if (!config->watches || config->max_per_watch == 0) {
        return 0;
    }
    for (index = 0; index < config->watches->len; ++index) {
        AppleVirglDirectReadWatch *watch =
            &g_array_index(config->watches, AppleVirglDirectReadWatch,
                           index);

        if (watch->task_id != task_id || watch->gpu_va != page_va) {
            continue;
        }
        if (watch->captures >= config->max_per_watch) {
            return 0;
        }
        watch->captures++;
        return ++config->next_capture_id;
    }
    return 0;
}

static GArray *apple_virgl_mapping_watch_entries(void)
{
    static gsize initialized;
    static GArray *entries;

    if (g_once_init_enter(&initialized)) {
        const char *environment = g_getenv("QMU_DIAG_MAPPING_WATCH_VAS");

        entries = g_array_new(false, false, sizeof(AppleVirglMappingWatch));
        if (environment && *environment) {
            g_auto(GStrv) tokens = g_strsplit(environment, ",", -1);
            char **token;

            for (token = tokens; token && *token; ++token) {
                char *entry = g_strstrip(*token);
                char *separator = strchr(entry, ':');
                char *task_end;
                char *va_end;
                guint64 task;
                guint64 gpu_va;
                AppleVirglMappingWatch watch;

                if (!separator) {
                    continue;
                }
                *separator = '\0';
                task = g_ascii_strtoull(g_strstrip(entry), &task_end, 0);
                gpu_va = g_ascii_strtoull(g_strstrip(separator + 1), &va_end,
                                          0);
                if (task_end == entry || *task_end != '\0' ||
                    va_end == separator + 1 || *va_end != '\0' ||
                    task > UINT32_MAX) {
                    continue;
                }
                watch.task_id = task;
                watch.gpu_va = gpu_va & ~UINT64_C(0xfff);
                g_array_append_val(entries, watch);
            }
        }
        g_once_init_leave(&initialized, 1);
    }

    return entries;
}

static bool apple_virgl_mapping_lifecycle_enabled(void)
{
    static gsize initialized;
    static bool enabled;

    if (g_once_init_enter(&initialized)) {
        const char *environment = g_getenv("QMU_DIAG_MAPPING_LIFECYCLE");

        enabled = environment && *environment && strcmp(environment, "0") != 0;
        g_once_init_leave(&initialized, 1);
    }

    return enabled;
}

static bool apple_virgl_resource_lifecycle_enabled(void)
{
    static gsize initialized;
    static bool enabled;

    if (g_once_init_enter(&initialized)) {
        const char *environment = g_getenv("QMU_DIAG_RESOURCE_LIFECYCLE");

        enabled = environment && *environment && strcmp(environment, "0") != 0;
        g_once_init_leave(&initialized, 1);
    }

    return enabled;
}

static bool apple_virgl_memory_map_wire_diag_enabled(void)
{
    static gsize initialized;
    static bool enabled;

    if (g_once_init_enter(&initialized)) {
        const char *environment = g_getenv("QMU_DIAG_MEMORY_MAP_WIRE");

        enabled = environment && *environment && strcmp(environment, "0") != 0;
        g_once_init_leave(&initialized, 1);
    }

    return enabled;
}

static void apple_virgl_memory_map_wire_log(AppleVirglContextState *context,
                                            uint16_t opcode,
                                            const AppleVirglSubmitView *view)
{
    const uint8_t *payload;
    uint32_t index;

    if (!context || !view || !view->payload || view->payload_bytes < 20 ||
        !apple_virgl_memory_map_wire_diag_enabled()) {
        return;
    }

    payload = view->payload;
    error_report("apple-virgl memory-map-wire: transport=%u opcode=%u "
                 "raw={task=%u virtual_address=0x%" PRIx64
                 " word_at_12=0x%" PRIx64 "} sideband_count=%u",
                 context->transport_context_id, opcode,
                 ldl_le_p(payload), ldq_le_p(payload + 4),
                 ldq_le_p(payload + 12), view->mapping_count);
    for (index = 0; index < view->mapping_count; ++index) {
        const AppleVirglSubmitMappingV1 *mapping = &view->mappings[index];

        error_report("apple-virgl memory-map-wire: transport=%u opcode=%u "
                     "sideband[%u]={gpu_va=0x%" PRIx64 " length=0x%" PRIx64
                     " backing_resource=%u backing_offset=0x%x apple_resource=%u}",
                     context->transport_context_id, opcode, index,
                     le64_to_cpu(mapping->gpu_va), le64_to_cpu(mapping->length),
                     le32_to_cpu(mapping->backing_resource_id),
                     le32_to_cpu(mapping->backing_offset),
                     le32_to_cpu(mapping->apple_resource_id));
    }
}

static bool apple_virgl_mapping_watch_match(uint32_t task_id,
                                             uint64_t gpu_va,
                                             uint64_t length)
{
    GArray *entries = apple_virgl_mapping_watch_entries();
    uint32_t index;

    if (apple_virgl_mapping_lifecycle_enabled()) {
        return true;
    }
    if (length == 0) {
        return false;
    }
    for (index = 0; index < entries->len; ++index) {
        AppleVirglMappingWatch *watch =
            &g_array_index(entries, AppleVirglMappingWatch, index);

        if (watch->task_id == task_id && watch->gpu_va >= gpu_va &&
            watch->gpu_va - gpu_va < length) {
            return true;
        }
    }
    return false;
}

static void apple_virgl_mapping_watch_log(AppleVirglBridge *bridge,
                                          AppleVirglContextState *context,
                                          const char *event, uint16_t opcode,
                                          const AppleVirglMapping *mapping)
{
    uint64_t sequence;

    if (!bridge || !context || !mapping ||
        !apple_virgl_mapping_watch_match(context->task_id, mapping->gpu_va,
                                         mapping->length)) {
        return;
    }
    sequence = ++bridge->mapping_lifecycle_sequence;
    error_report("apple-virgl mapping-watch: seq=%" PRIu64
                 " event=%s transport=%u task=%u opcode=%u"
                 " va=0x%" PRIx64 " length=0x%" PRIx64
                 " backing=%u offset=0x%x apple=%u live_mappings=%u",
                 sequence, event, context->transport_context_id,
                 context->task_id, opcode, mapping->gpu_va, mapping->length,
                 mapping->backing_resource_id, mapping->backing_offset,
                 mapping->apple_resource_id, context->mappings->len);
}

static void apple_virgl_resource_watch_log(AppleVirglBridge *bridge,
                                           const char *event,
                                           uint32_t context_id,
                                           uint32_t task_id,
                                           uint32_t resource_id,
                                           uint64_t size,
                                           uint32_t iov_count)
{
    if (!bridge || !apple_virgl_resource_lifecycle_enabled()) {
        return;
    }
    error_report("apple-virgl resource-watch: event=%s transport=%u task=%u "
                 "resource=%u size=0x%" PRIx64 " iov_count=%u live_resources=%u",
                 event, context_id, task_id, resource_id, size, iov_count,
                 g_hash_table_size(bridge->resources));
}

static void apple_virgl_direct_write_lease_log(
    AppleVirglBridge *bridge,
    const char *event,
    const AppleVirglDirectWriteLease *lease,
    uint32_t resource_lease_count)
{
    if (!bridge || !lease || !apple_virgl_resource_lifecycle_enabled()) {
        return;
    }
    error_report("apple-virgl direct-write-lease: event=%s token=%" PRIu64
                 " task=%u va=0x%" PRIx64 " resource=%u offset=0x%" PRIx64
                 " size=0x%" PRIx64 " resource_leases=%u",
                 event, lease->token, lease->task_id, lease->gpu_va,
                 lease->resource_id, lease->resource_offset, lease->size,
                 resource_lease_count);
}

static void apple_virgl_completion_receiver_free(
    AppleVirglCompletionReceiver *receiver)
{
    if (!receiver) {
        return;
    }
    g_free(receiver->elem);
    g_free(receiver);
}

static bool apple_virgl_completion_maybe_schedule_locked(
    AppleVirglBridge *bridge)
{
    if (bridge->completion_shutdown || bridge->completion_resetting ||
        bridge->completion_bh_scheduled ||
        g_queue_is_empty(&bridge->completion_receivers) ||
        g_queue_is_empty(&bridge->completion_stamps)) {
        return false;
    }

    bridge->completion_bh_scheduled = true;
    return true;
}

static void apple_virgl_completion_bh(void *opaque)
{
    AppleVirglBridge *bridge = opaque;

    for (;;) {
        AppleVirglCompletionReceiver *receiver;
        AppleVirglCompletionStamp *stamp;
        AppleVirglCompletionEventV3 event;
        size_t written;

        qemu_mutex_lock(&bridge->completion_lock);
        if (bridge->completion_shutdown || bridge->completion_resetting ||
            g_queue_is_empty(&bridge->completion_receivers) ||
            g_queue_is_empty(&bridge->completion_stamps)) {
            bridge->completion_bh_scheduled = false;
            qemu_mutex_unlock(&bridge->completion_lock);
            return;
        }
        receiver = g_queue_pop_head(&bridge->completion_receivers);
        stamp = g_queue_pop_head(&bridge->completion_stamps);
        qemu_mutex_unlock(&bridge->completion_lock);

        event = (AppleVirglCompletionEventV3) {
            .magic = cpu_to_le32(APPLE_VIRGL_COMPLETION_EVENT_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION_V3),
            .type = cpu_to_le16(APPLE_VIRGL_COMPLETION_EVENT_TYPE_STAMP),
            .channel_id = cpu_to_le32(stamp->channel_id),
            .stamp = cpu_to_le32(stamp->stamp),
        };
        written = iov_from_buf(receiver->elem->in_sg,
                               receiver->elem->in_num, 0, &event,
                               sizeof(event));
        if (written != sizeof(event)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "apple-virgl completion receiver write failed (%zu/%zu)\n",
                          written, sizeof(event));
            qemu_mutex_lock(&bridge->completion_lock);
            if (!bridge->completion_shutdown && !bridge->completion_resetting) {
                g_queue_push_head(&bridge->completion_stamps, stamp);
            } else {
                g_free(stamp);
            }
            bridge->completion_bh_scheduled = false;
            qemu_mutex_unlock(&bridge->completion_lock);
            virtqueue_push(receiver->vq, receiver->elem, 0);
            virtio_notify(VIRTIO_DEVICE(bridge->gpu), receiver->vq);
            apple_virgl_completion_receiver_free(receiver);
            return;
        }

        virtqueue_push(receiver->vq, receiver->elem, written);
        virtio_notify(VIRTIO_DEVICE(bridge->gpu), receiver->vq);
        apple_virgl_completion_receiver_free(receiver);
        g_free(stamp);
    }
}

static void apple_virgl_completion_stamp(void *opaque, uint32_t channel_id,
                                         uint32_t stamp_value)
{
    AppleVirglBridge *bridge = opaque;
    AppleVirglCompletionStamp *stamp;
    bool schedule = false;

    if (!bridge || channel_id == 0 || channel_id >= 8 || stamp_value == 0) {
        return;
    }

    stamp = g_new(AppleVirglCompletionStamp, 1);
    stamp->channel_id = channel_id;
    stamp->stamp = stamp_value;

    qemu_mutex_lock(&bridge->completion_lock);
    if (!bridge->completion_shutdown && !bridge->completion_resetting) {
        g_queue_push_tail(&bridge->completion_stamps, stamp);
        schedule = apple_virgl_completion_maybe_schedule_locked(bridge);
        stamp = NULL;
    }
    qemu_mutex_unlock(&bridge->completion_lock);

    g_free(stamp);
    if (schedule) {
        qemu_bh_schedule(bridge->completion_bh);
    }
}

static void apple_virgl_bridge_clear_completion_state(AppleVirglBridge *bridge,
                                                       bool resume)
{
    GQueue receivers = G_QUEUE_INIT;
    GQueue stamps = G_QUEUE_INIT;

    if (!bridge) {
        return;
    }

    qemu_mutex_lock(&bridge->completion_lock);
    bridge->completion_resetting = true;
    qemu_mutex_unlock(&bridge->completion_lock);
    if (bridge->completion_bh) {
        qemu_bh_cancel(bridge->completion_bh);
    }
    qemu_mutex_lock(&bridge->completion_lock);
    receivers = bridge->completion_receivers;
    stamps = bridge->completion_stamps;
    g_queue_init(&bridge->completion_receivers);
    g_queue_init(&bridge->completion_stamps);
    bridge->completion_bh_scheduled = false;
    qemu_mutex_unlock(&bridge->completion_lock);

    while (!g_queue_is_empty(&receivers)) {
        apple_virgl_completion_receiver_free(g_queue_pop_head(&receivers));
    }
    while (!g_queue_is_empty(&stamps)) {
        g_free(g_queue_pop_head(&stamps));
    }

    qemu_mutex_lock(&bridge->completion_lock);
    if (resume && !bridge->completion_shutdown) {
        bridge->completion_resetting = false;
    }
    qemu_mutex_unlock(&bridge->completion_lock);
}

static void apple_virgl_context_free(gpointer opaque)
{
    AppleVirglContextState *context = opaque;

    g_hash_table_destroy(context->attached_resources);
    g_array_unref(context->mappings);
    g_free(context);
}

static void apple_virgl_resource_free(gpointer opaque)
{
    g_free(opaque);
}

static void *apple_virgl_map_gpa(void *opaque, uint64_t gpa,
                                 size_t size, int writable)
{
    AppleVirglBridge *bridge = opaque;
    RCU_READ_LOCK_GUARD();
    MemoryRegion *region = NULL;
    hwaddr translated = 0;
    hwaddr translated_size = size;
    void *ram;

    region = address_space_translate(VIRTIO_DEVICE(bridge->gpu)->dma_as,
                                     gpa, &translated, &translated_size,
                                     writable, MEMTXATTRS_UNSPECIFIED);
    if (!region || translated_size < size ||
        !memory_access_is_direct(region, writable, MEMTXATTRS_UNSPECIFIED)) {
        return NULL;
    }
    ram = memory_region_get_ram_ptr(region);
    if (!ram) {
        return NULL;
    }
    memory_region_ref(region);
    return ram + translated;
}

static void apple_virgl_unmap_gpa(void *opaque, void *host,
                                  size_t size, int dirty)
{
    ram_addr_t offset;
    MemoryRegion *region;

    region = memory_region_from_host(host, &offset);
    if (region) {
        if (dirty) {
            memory_region_set_dirty(region, offset, size);
        }
        memory_region_unref(region);
    }
}

static int apple_virgl_read_memory(void *opaque, uint64_t gpa,
                                   void *bytes, size_t size)
{
    AppleVirglBridge *bridge = opaque;

    return address_space_read(VIRTIO_DEVICE(bridge->gpu)->dma_as, gpa,
                              MEMTXATTRS_UNSPECIFIED, bytes, size) == MEMTX_OK
        ? 0 : -1;
}

static int apple_virgl_write_memory(void *opaque, uint64_t gpa,
                                    const void *bytes, size_t size)
{
    AppleVirglBridge *bridge = opaque;

    return address_space_write(VIRTIO_DEVICE(bridge->gpu)->dma_as, gpa,
                               MEMTXATTRS_UNSPECIFIED, bytes, size) == MEMTX_OK
        ? 0 : -1;
}

static AppleVirglContextState *apple_virgl_context_for_task_locked(
    AppleVirglBridge *bridge, uint32_t task_id)
{
    GHashTableIter iter;
    gpointer value;

    g_hash_table_iter_init(&iter, bridge->contexts);
    while (g_hash_table_iter_next(&iter, NULL, &value)) {
        AppleVirglContextState *context = value;

        if (context->task_bound && context->task_id == task_id) {
            return context;
        }
    }
    return NULL;
}

static bool apple_virgl_range_contains(uint64_t base, uint64_t length,
                                       uint64_t address, size_t size,
                                       uint64_t *delta)
{
    uint64_t offset;

    if (address < base) {
        return false;
    }
    offset = address - base;
    if (offset > length || size > length - offset) {
        return false;
    }
    *delta = offset;
    return true;
}

static void apple_virgl_direct_read_snapshot_mappings_locked(
    AppleVirglBridge *bridge, uint32_t task_id, uint64_t gpu_va, size_t size,
    AppleVirglDirectReadMappingSnapshot *snapshot)
{
    AppleVirglContextState *context;
    uint32_t index;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->candidate_hash = UINT64_C(0xcbf29ce484222325);
    context = apple_virgl_context_for_task_locked(bridge, task_id);
    if (!context) {
        return;
    }
    for (index = 0; index < context->mappings->len; ++index) {
        AppleVirglMapping *mapping =
            &g_array_index(context->mappings, AppleVirglMapping, index);
        uint64_t ignored_delta;

        if (!apple_virgl_range_contains(mapping->gpu_va, mapping->length,
                                        gpu_va, size, &ignored_delta)) {
            continue;
        }
        snapshot->candidate_hash = apple_virgl_direct_read_fnv1a64_u64(
            snapshot->candidate_hash, mapping->gpu_va);
        snapshot->candidate_hash = apple_virgl_direct_read_fnv1a64_u64(
            snapshot->candidate_hash, mapping->length);
        snapshot->candidate_hash = apple_virgl_direct_read_fnv1a64_u64(
            snapshot->candidate_hash, mapping->backing_resource_id);
        snapshot->candidate_hash = apple_virgl_direct_read_fnv1a64_u64(
            snapshot->candidate_hash, mapping->backing_offset);
        snapshot->candidate_hash = apple_virgl_direct_read_fnv1a64_u64(
            snapshot->candidate_hash, mapping->apple_resource_id);
        if (snapshot->candidate_count == 0) {
            snapshot->selected = *mapping;
            snapshot->selected_index = index;
        }
        snapshot->last = *mapping;
        snapshot->candidate_count++;
    }
}

static void apple_virgl_direct_read_snapshot_iov(
    const AppleVirglResourceState *resource,
    AppleVirglDirectReadProvenanceRecord *record)
{
    uint32_t index;
    uint32_t middle;

    if (!resource || !resource->addrs || !resource->iov ||
        resource->iov_count == 0) {
        return;
    }
    record->resource_id = resource->resource_id;
    record->backing_size = resource->backing_size;
    record->iov_count = resource->iov_count;
    record->iov_layout_hash = UINT64_C(0xcbf29ce484222325);
    middle = resource->iov_count / 2;
    for (index = 0; index < resource->iov_count; ++index) {
        uint64_t addr = resource->addrs[index];
        size_t length = resource->iov[index].iov_len;

        record->iov_layout_hash = apple_virgl_direct_read_fnv1a64_u64(
            record->iov_layout_hash, addr);
        record->iov_layout_hash = apple_virgl_direct_read_fnv1a64_u64(
            record->iov_layout_hash, length);
        if (index == 0) {
            record->first_iov_addr = addr;
            record->first_iov_len = length;
        }
        if (index == middle) {
            record->middle_iov_addr = addr;
            record->middle_iov_len = length;
        }
        if (index + 1 == resource->iov_count) {
            record->last_iov_addr = addr;
            record->last_iov_len = length;
        }
    }
}

static uint32_t apple_virgl_direct_read_sample_u32(const uint8_t *bytes,
                                                    size_t size,
                                                    size_t offset)
{
    uint32_t value = 0;

    if (!bytes || size < sizeof(value) || offset > size - sizeof(value)) {
        return 0;
    }
    memcpy(&value, bytes + offset, sizeof(value));
    return value;
}

static void apple_virgl_direct_read_provenance_log(
    const AppleVirglDirectReadProvenanceRecord *record, const void *bytes)
{
    const uint8_t *byte_values = bytes;
    uint64_t bytes_hash = 0;
    uint32_t first = 0;
    uint32_t middle = 0;
    uint32_t last = 0;
    size_t middle_offset = 0;

    if (record->result == 0 && byte_values && record->size != 0) {
        bytes_hash = apple_virgl_direct_read_fnv1a64_bytes(byte_values,
                                                            record->size);
        if (record->size >= sizeof(uint32_t)) {
            middle_offset = (record->size / 2) & ~((size_t)3);
            if (middle_offset > record->size - sizeof(uint32_t)) {
                middle_offset = record->size - sizeof(uint32_t);
            }
            first = apple_virgl_direct_read_sample_u32(byte_values,
                                                        record->size, 0);
            middle = apple_virgl_direct_read_sample_u32(byte_values,
                                                         record->size,
                                                         middle_offset);
            last = apple_virgl_direct_read_sample_u32(
                byte_values, record->size,
                record->size - sizeof(uint32_t));
        }
    }

    error_report("apple-virgl direct-read-provenance: capture=%" PRIu64
                 " task=%u request_va=0x%" PRIx64 " size=0x%zx rc=%d"
                 " candidates=%u selected_index=%u"
                 " selected={va=0x%" PRIx64 " length=0x%" PRIx64
                 " backing=%u offset=0x%x apple=%u}"
                 " last={va=0x%" PRIx64 " length=0x%" PRIx64
                 " backing=%u offset=0x%x apple=%u}"
                 " resource={id=%u offset=0x%" PRIx64
                 " backing_size=0x%zx iov_count=%u"
                 " layout_hash=0x%016" PRIx64
                 " first=0x%" PRIx64 "/0x%zx"
                 " middle=0x%" PRIx64 "/0x%zx"
                 " last=0x%" PRIx64 "/0x%zx}"
                 " bytes={hash=0x%016" PRIx64
                 " first=0x%08x middle=0x%08x last=0x%08x}",
                 record->capture_id, record->task_id, record->gpu_va,
                 record->size, record->result,
                 record->mappings.candidate_count,
                 record->mappings.selected_index,
                 record->mappings.selected.gpu_va,
                 record->mappings.selected.length,
                 record->mappings.selected.backing_resource_id,
                 record->mappings.selected.backing_offset,
                 record->mappings.selected.apple_resource_id,
                 record->mappings.last.gpu_va, record->mappings.last.length,
                 record->mappings.last.backing_resource_id,
                 record->mappings.last.backing_offset,
                 record->mappings.last.apple_resource_id,
                 record->resource_id, record->resource_offset,
                 record->backing_size, record->iov_count,
                 record->iov_layout_hash,
                 record->first_iov_addr, record->first_iov_len,
                 record->middle_iov_addr, record->middle_iov_len,
                 record->last_iov_addr, record->last_iov_len,
                 bytes_hash, first, middle, last);
}

static AppleVirglResourceState *apple_virgl_resolve_gpu_memory_locked(
    AppleVirglBridge *bridge, uint32_t task_id, uint64_t gpu_va, size_t size,
    uint64_t *resource_offset)
{
    AppleVirglContextState *context;
    uint32_t index;

    context = apple_virgl_context_for_task_locked(bridge, task_id);
    if (!context) {
        return NULL;
    }

    for (index = 0; index < context->mappings->len; ++index) {
        AppleVirglMapping *mapping =
            &g_array_index(context->mappings, AppleVirglMapping, index);
        AppleVirglResourceState *resource;
        uint64_t delta;
        uint64_t offset;

        if (!apple_virgl_range_contains(mapping->gpu_va, mapping->length,
                                        gpu_va, size, &delta)) {
            continue;
        }
        resource = g_hash_table_lookup(bridge->resources,
                                       GUINT_TO_POINTER(
                                           mapping->backing_resource_id));
        if (!resource || !resource->iov || !resource->addrs) {
            return NULL;
        }
        offset = (uint64_t)mapping->backing_offset + delta;
        if (offset > resource->backing_size ||
            size > resource->backing_size - offset) {
            return NULL;
        }
        *resource_offset = offset;
        return resource;
    }

    return NULL;
}

static int apple_virgl_read_gpu_memory(void *opaque, uint32_t task_id,
                                       uint64_t gpu_va, void *bytes, size_t size)
{
    AppleVirglBridge *bridge = opaque;
    AppleVirglResourceState *resource;
    uint64_t resource_offset;
    int result = -1;
    AppleVirglDirectReadProvenanceRecord provenance = {
        .task_id = task_id,
        .gpu_va = gpu_va,
        .size = size,
        .result = -1,
    };

    qemu_mutex_lock(&bridge->lock);
    provenance.capture_id = apple_virgl_direct_read_provenance_claim(task_id,
                                                                      gpu_va);
    if (provenance.capture_id != 0) {
        apple_virgl_direct_read_snapshot_mappings_locked(
            bridge, task_id, gpu_va, size, &provenance.mappings);
    }
    resource = apple_virgl_resolve_gpu_memory_locked(
        bridge, task_id, gpu_va, size, &resource_offset);
    if (resource) {
        result = iov_to_buf(resource->iov, resource->iov_count,
                            resource_offset, bytes, size) == size ? 0 : -1;
        if (provenance.capture_id != 0) {
            provenance.resource_offset = resource_offset;
            apple_virgl_direct_read_snapshot_iov(resource, &provenance);
        }
    }
    provenance.result = result;
    qemu_mutex_unlock(&bridge->lock);
    if (provenance.capture_id != 0) {
        apple_virgl_direct_read_provenance_log(&provenance, bytes);
    }
    return result;
}

static int apple_virgl_write_resource_memory_locked(
    AppleVirglBridge *bridge,
    const AppleVirglResourceState *resource,
    uint64_t resource_offset,
    const void *bytes,
    size_t size)
{
    const uint8_t *source = bytes;
    uint64_t segment_offset;
    size_t remaining = size;
    uint32_t index;

    if (!bridge || !resource || !resource->addrs || !resource->iov ||
        resource_offset > resource->backing_size ||
        size > resource->backing_size - resource_offset) {
        return -1;
    }

    segment_offset = resource_offset;
    for (index = 0; index < resource->iov_count && remaining > 0; ++index) {
        size_t segment_length = resource->iov[index].iov_len;
        size_t chunk;

        if (segment_offset >= segment_length) {
            segment_offset -= segment_length;
            continue;
        }
        chunk = MIN(remaining, segment_length - segment_offset);
        if (dma_memory_write(VIRTIO_DEVICE(bridge->gpu)->dma_as,
                             resource->addrs[index] + segment_offset,
                             source, chunk,
                             MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            return -1;
        }
        source += chunk;
        remaining -= chunk;
        segment_offset = 0;
    }
    return remaining == 0 ? 0 : -1;
}

static int apple_virgl_write_gpu_memory(void *opaque, uint32_t task_id,
                                        uint64_t gpu_va, const void *bytes,
                                        size_t size)
{
    AppleVirglBridge *bridge = opaque;
    AppleVirglResourceState *resource;
    uint64_t resource_offset;
    int result = -1;

    qemu_mutex_lock(&bridge->lock);
    resource = apple_virgl_resolve_gpu_memory_locked(
        bridge, task_id, gpu_va, size, &resource_offset);
    if (resource) {
        result = apple_virgl_write_resource_memory_locked(
            bridge, resource, resource_offset, bytes, size);
    }
    qemu_mutex_unlock(&bridge->lock);
    return result;
}

static bool apple_virgl_resource_release_maybe_schedule_locked(
    AppleVirglBridge *bridge)
{
    if (bridge->resource_release_shutdown ||
        bridge->resource_release_resetting ||
        bridge->resource_release_bh_scheduled ||
        bridge->resource_release_resume_count == 0) {
        return false;
    }
    bridge->resource_release_bh_scheduled = true;
    return true;
}

static void apple_virgl_resource_release_bh(void *opaque)
{
    AppleVirglBridge *bridge = opaque;
    VirtIOGPUBase *base;
    uint32_t ready_count = 0;

    if (!bridge) {
        return;
    }

    qemu_mutex_lock(&bridge->lock);
    bridge->resource_release_bh_scheduled = false;
    if (!bridge->resource_release_shutdown &&
        !bridge->resource_release_resetting) {
        ready_count = MIN(bridge->resource_release_resume_count,
                          bridge->resource_release_blocked_count);
        bridge->resource_release_resume_count -= ready_count;
        bridge->resource_release_blocked_count -= ready_count;
    }
    qemu_mutex_unlock(&bridge->lock);

    if (ready_count == 0) {
        return;
    }

    base = VIRTIO_GPU_BASE(bridge->gpu);
    assert(base->renderer_blocked >= ready_count);
    if (apple_virgl_resource_lifecycle_enabled()) {
        error_report("apple-virgl resource-unref: event=resume count=%u "
                     "renderer_blocked=%d",
                     ready_count, base->renderer_blocked);
    }
    base->renderer_blocked -= ready_count;
    virtio_gpu_process_cmdq(bridge->gpu);
}

static int apple_virgl_acquire_direct_write_lease(void *opaque,
                                                   uint32_t task_id,
                                                   uint64_t gpu_va,
                                                   size_t size,
                                                   uint64_t *out_token)
{
    AppleVirglBridge *bridge = opaque;
    AppleVirglResourceState *resource;
    AppleVirglDirectWriteLease *lease;
    uint64_t resource_offset;
    uint64_t token;
    int result = -1;

    if (!bridge || !out_token || size == 0) {
        return -1;
    }
    *out_token = 0;

    qemu_mutex_lock(&bridge->lock);
    if (bridge->resource_release_shutdown ||
        bridge->resource_release_resetting) {
        goto out;
    }
    resource = apple_virgl_resolve_gpu_memory_locked(
        bridge, task_id, gpu_va, size, &resource_offset);
    if (!resource || resource->direct_write_lease_count == UINT32_MAX) {
        goto out;
    }

    do {
        token = ++bridge->next_direct_write_lease_token;
    } while (token == 0 || g_hash_table_contains(bridge->direct_write_leases,
                                                  &token));

    lease = g_new0(AppleVirglDirectWriteLease, 1);
    lease->token = token;
    lease->task_id = task_id;
    lease->gpu_va = gpu_va;
    lease->size = size;
    lease->resource_id = resource->resource_id;
    lease->resource_offset = resource_offset;
    lease->addrs = resource->addrs;
    lease->iov = resource->iov;
    lease->iov_count = resource->iov_count;
    lease->backing_size = resource->backing_size;
    g_hash_table_insert(bridge->direct_write_leases, g_memdup2(&token,
                                                               sizeof(token)),
                        lease);
    resource->direct_write_lease_count++;
    apple_virgl_direct_write_lease_log(bridge, "acquire", lease,
                                       resource->direct_write_lease_count);
    *out_token = token;
    result = 0;

out:
    qemu_mutex_unlock(&bridge->lock);
    return result;
}

static int apple_virgl_write_direct_write_lease(void *opaque,
                                                 uint64_t token,
                                                 const void *bytes,
                                                 size_t size)
{
    AppleVirglBridge *bridge = opaque;
    AppleVirglDirectWriteLease *lease;
    AppleVirglResourceState *resource;
    int result = -1;

    if (!bridge || !bytes || token == 0) {
        return -1;
    }

    qemu_mutex_lock(&bridge->lock);
    lease = g_hash_table_lookup(bridge->direct_write_leases, &token);
    if (!lease || bridge->resource_release_shutdown ||
        bridge->resource_release_resetting || size != lease->size) {
        goto out;
    }
    resource = g_hash_table_lookup(bridge->resources,
                                   GUINT_TO_POINTER(lease->resource_id));
    if (!resource || resource->direct_write_lease_count == 0 ||
        resource->addrs != lease->addrs || resource->iov != lease->iov ||
        resource->iov_count != lease->iov_count ||
        resource->backing_size != lease->backing_size) {
        goto out;
    }
    result = apple_virgl_write_resource_memory_locked(
        bridge, resource, lease->resource_offset, bytes, size);
    apple_virgl_direct_write_lease_log(bridge,
                                       result == 0 ? "write" : "write-failed",
                                       lease, resource->direct_write_lease_count);

out:
    qemu_mutex_unlock(&bridge->lock);
    return result;
}

static void apple_virgl_release_direct_write_lease(void *opaque,
                                                    uint64_t token)
{
    AppleVirglBridge *bridge = opaque;
    AppleVirglDirectWriteLease *lease;
    AppleVirglResourceState *resource;
    bool schedule = false;

    if (!bridge || token == 0) {
        return;
    }

    qemu_mutex_lock(&bridge->lock);
    lease = g_hash_table_lookup(bridge->direct_write_leases, &token);
    if (!lease) {
        qemu_mutex_unlock(&bridge->lock);
        return;
    }
    resource = g_hash_table_lookup(bridge->resources,
                                   GUINT_TO_POINTER(lease->resource_id));
    if (resource && resource->direct_write_lease_count > 0) {
        resource->direct_write_lease_count--;
        apple_virgl_direct_write_lease_log(bridge, "release", lease,
                                           resource->direct_write_lease_count);
        if (resource->direct_write_lease_count == 0 &&
            resource->direct_write_release_waiting &&
            !bridge->resource_release_shutdown &&
            !bridge->resource_release_resetting) {
            resource->direct_write_release_waiting = false;
            bridge->resource_release_resume_count++;
            schedule = apple_virgl_resource_release_maybe_schedule_locked(bridge);
        }
    }
    g_hash_table_remove(bridge->direct_write_leases, &token);
    qemu_mutex_unlock(&bridge->lock);

    if (schedule) {
        qemu_bh_schedule(bridge->resource_release_bh);
    }
}

static void apple_virgl_bridge_clear_direct_write_leases(
    AppleVirglBridge *bridge)
{
    GHashTableIter iter;
    gpointer value;
    uint32_t blocked_count;

    if (!bridge) {
        return;
    }

    qemu_mutex_lock(&bridge->lock);
    bridge->resource_release_resetting = true;
    qemu_mutex_unlock(&bridge->lock);
    if (bridge->resource_release_bh) {
        qemu_bh_cancel(bridge->resource_release_bh);
    }

    qemu_mutex_lock(&bridge->lock);
    blocked_count = bridge->resource_release_blocked_count;
    bridge->resource_release_blocked_count = 0;
    bridge->resource_release_resume_count = 0;
    bridge->resource_release_bh_scheduled = false;
    g_hash_table_iter_init(&iter, bridge->resources);
    while (g_hash_table_iter_next(&iter, NULL, &value)) {
        AppleVirglResourceState *resource = value;

        resource->direct_write_lease_count = 0;
        resource->direct_write_release_waiting = false;
    }
    g_hash_table_remove_all(bridge->direct_write_leases);
    qemu_mutex_unlock(&bridge->lock);

    if (blocked_count != 0) {
        VirtIOGPUBase *base = VIRTIO_GPU_BASE(bridge->gpu);

        assert(base->renderer_blocked >= blocked_count);
        base->renderer_blocked -= blocked_count;
    }
}

static void apple_virgl_capture_present_frame(uint64_t frame,
                                              const void *pixels,
                                              uint32_t width,
                                              uint32_t height,
                                              uint32_t stride)
{
    const char *directory = g_getenv("APPLE_VIRGL_PRESENT_CAPTURE_DIR");
    const char *request = g_getenv("APPLE_VIRGL_PRESENT_CAPTURE_REQUEST");
    const uint8_t *source = pixels;
    g_autofree char *raw_path = NULL;
    g_autofree char *raw_temporary_path = NULL;
    g_autofree char *metadata_path = NULL;
    g_autofree char *metadata_temporary_path = NULL;
    g_autofree char *metadata = NULL;
    FILE *output;
    uint32_t row;

    if (!directory || !*directory || !request || !*request ||
        !g_file_test(request, G_FILE_TEST_EXISTS)) {
        return;
    }
    if (!pixels || width == 0 || height == 0 || stride < width * 4u ||
        !g_file_test(directory, G_FILE_TEST_IS_DIR)) {
        error_report("apple-virgl present-capture: rejected frame=%" PRIu64
                     " pixels=%p size=%ux%u stride=%u directory=%s",
                     frame, pixels, width, height, stride, directory);
        return;
    }

    raw_path = g_build_filename(directory, "qmetal-present-latest.bgra", NULL);
    raw_temporary_path = g_strdup_printf("%s.tmp", raw_path);
    metadata_path = g_build_filename(directory, "qmetal-present-latest.meta", NULL);
    metadata_temporary_path = g_strdup_printf("%s.tmp", metadata_path);

    output = fopen(raw_temporary_path, "wb");
    if (!output) {
        error_report("apple-virgl present-capture: cannot open %s: %s",
                     raw_temporary_path, strerror(errno));
        return;
    }
    for (row = 0; row < height; row++) {
        const size_t written = fwrite(source + (size_t)row * stride, 1,
                                      (size_t)width * 4u, output);

        if (written != (size_t)width * 4u) {
            error_report("apple-virgl present-capture: short write %s row=%u",
                         raw_temporary_path, row);
            fclose(output);
            g_remove(raw_temporary_path);
            return;
        }
    }
    if (fclose(output) != 0 || g_rename(raw_temporary_path, raw_path) != 0) {
        error_report("apple-virgl present-capture: cannot publish %s: %s",
                     raw_path, strerror(errno));
        g_remove(raw_temporary_path);
        return;
    }

    metadata = g_strdup_printf(
        "frame=%" PRIu64 "\nwidth=%u\nheight=%u\nsource_stride=%u\n"
        "stored_stride=%u\npixel_layout=BGRA8\n",
        frame, width, height, stride, width * 4u);
    if (!g_file_set_contents(metadata_temporary_path, metadata, -1, NULL) ||
        g_rename(metadata_temporary_path, metadata_path) != 0) {
        error_report("apple-virgl present-capture: cannot publish metadata %s",
                     metadata_path);
        g_remove(metadata_temporary_path);
        return;
    }

    g_remove(request);
    error_report("apple-virgl present-capture: frame=%" PRIu64
                 " size=%ux%u source_stride=%u raw=%s",
                 frame, width, height, stride, raw_path);
}

static void apple_virgl_present_frame(void *opaque, const void *pixels,
                                      uint32_t width, uint32_t height,
                                      uint32_t stride)
{
    AppleVirglBridge *bridge = opaque;
    uint64_t frame = ++bridge->frame_count;

    if (frame <= 8) {
        fprintf(stderr,
                "apple-virgl-qemu: qmetal frame=%" PRIu64
                " size=%ux%u stride=%u pixels=%p\n",
                frame, width, height, stride, pixels);
    }
    apple_virgl_capture_present_frame(frame, pixels, width, height, stride);
}

AppleVirglBridge *apple_virgl_bridge_new(VirtIOGPU *gpu)
{
    AppleVirglBridge *bridge;
    qmu_config config = {
        .display_width = 1920,
        .display_height = 1080,
        .display_format = 44,
        .enable_validation = 0,
        .verbose = 0,
    };
    qmu_callbacks callbacks = { 0 };

    bridge = g_new0(AppleVirglBridge, 1);
    bridge->gpu = gpu;
    qemu_mutex_init(&bridge->lock);
    qemu_mutex_init(&bridge->completion_lock);
    g_queue_init(&bridge->completion_receivers);
    g_queue_init(&bridge->completion_stamps);
    bridge->completion_bh = qemu_bh_new_guarded(
        apple_virgl_completion_bh, bridge,
        &DEVICE(gpu)->mem_reentrancy_guard);
    bridge->resource_release_bh = qemu_bh_new_guarded(
        apple_virgl_resource_release_bh, bridge,
        &DEVICE(gpu)->mem_reentrancy_guard);
    bridge->contexts = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                              NULL, apple_virgl_context_free);
    bridge->resources = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                               NULL, apple_virgl_resource_free);
    bridge->direct_write_leases = g_hash_table_new_full(
        g_int64_hash, g_int64_equal, g_free, g_free);

    callbacks.user_ctx = bridge;
    callbacks.map_gpa = apple_virgl_map_gpa;
    callbacks.unmap_gpa = apple_virgl_unmap_gpa;
    callbacks.read_memory = apple_virgl_read_memory;
    callbacks.write_memory = apple_virgl_write_memory;
    callbacks.completion_stamp = apple_virgl_completion_stamp;
    callbacks.present_frame = apple_virgl_present_frame;
    bridge->session = qmu_create(&config, &callbacks);
    if (!bridge->session) {
        apple_virgl_bridge_free(bridge);
        return NULL;
    }
    qmu_set_direct_read_callback(bridge->session,
                                 apple_virgl_read_gpu_memory, bridge);
    qmu_set_direct_write_callback(bridge->session,
                                  apple_virgl_write_gpu_memory, bridge);
    qmu_set_direct_write_lease_callbacks(
        bridge->session, apple_virgl_acquire_direct_write_lease,
        apple_virgl_write_direct_write_lease,
        apple_virgl_release_direct_write_lease, bridge);
    return bridge;
}

void apple_virgl_bridge_free(AppleVirglBridge *bridge)
{
    if (!bridge) {
        return;
    }
    qemu_mutex_lock(&bridge->completion_lock);
    bridge->completion_shutdown = true;
    qemu_mutex_unlock(&bridge->completion_lock);
    qemu_mutex_lock(&bridge->lock);
    bridge->resource_release_shutdown = true;
    qemu_mutex_unlock(&bridge->lock);
    if (bridge->session) {
        qmu_session_begin_shutdown(bridge->session);
        qmu_destroy(bridge->session);
    }
    apple_virgl_bridge_clear_direct_write_leases(bridge);
    apple_virgl_bridge_clear_completion_state(bridge, false);
    if (bridge->completion_bh) {
        qemu_bh_delete(bridge->completion_bh);
    }
    if (bridge->resource_release_bh) {
        qemu_bh_delete(bridge->resource_release_bh);
    }
    g_hash_table_destroy(bridge->contexts);
    g_hash_table_destroy(bridge->resources);
    g_hash_table_destroy(bridge->direct_write_leases);
    qemu_mutex_destroy(&bridge->completion_lock);
    qemu_mutex_destroy(&bridge->lock);
    g_free(bridge);
}

void apple_virgl_bridge_reset(AppleVirglBridge *bridge)
{
    GHashTableIter iter;
    gpointer value;

    if (!bridge) {
        return;
    }
    apple_virgl_bridge_clear_direct_write_leases(bridge);
    apple_virgl_bridge_clear_completion_state(bridge, false);
    qemu_mutex_lock(&bridge->lock);
    g_hash_table_iter_init(&iter, bridge->contexts);
    while (g_hash_table_iter_next(&iter, NULL, &value)) {
        AppleVirglContextState *context = value;

        if (context->task_bound) {
            qmu_destroy_task(bridge->session, context->task_id);
        }
    }
    g_hash_table_remove_all(bridge->contexts);
    g_hash_table_remove_all(bridge->resources);
    bridge->submit_count = 0;
    qemu_mutex_unlock(&bridge->lock);
    qemu_mutex_lock(&bridge->completion_lock);
    if (!bridge->completion_shutdown) {
        bridge->completion_resetting = false;
    }
    qemu_mutex_unlock(&bridge->completion_lock);
    qemu_mutex_lock(&bridge->lock);
    if (!bridge->resource_release_shutdown) {
        bridge->resource_release_resetting = false;
    }
    qemu_mutex_unlock(&bridge->lock);
}

bool apple_virgl_bridge_accept_completion_receiver(
    AppleVirglBridge *bridge, VirtQueue *vq, VirtQueueElement *elem)
{
    struct virtio_gpu_ctrl_hdr header;
    AppleVirglCompletionReceiverRequestV3 request;
    AppleVirglCompletionReceiver *receiver;
    size_t copied;
    bool schedule;

    if (!bridge || !vq || !elem || elem->out_num == 0) {
        return false;
    }
    copied = iov_to_buf(elem->out_sg, elem->out_num, 0, &header,
                        sizeof(header));
    if (copied != sizeof(header) ||
        le32_to_cpu(header.type) !=
            APPLE_VIRGL_CURSOR_CMD_COMPLETION_RECEIVE) {
        return false;
    }

    copied = iov_to_buf(elem->out_sg, elem->out_num, sizeof(header),
                        &request, sizeof(request));
    if (iov_size(elem->out_sg, elem->out_num) !=
            sizeof(header) + sizeof(request) ||
        copied != sizeof(request) ||
        iov_size(elem->in_sg, elem->in_num) !=
            sizeof(AppleVirglCompletionEventV3) ||
        le32_to_cpu(header.flags) != 0 ||
        le64_to_cpu(header.fence_id) != 0 ||
        le32_to_cpu(header.ctx_id) != 0 || header.ring_idx != 0 ||
        le32_to_cpu(request.magic) != APPLE_VIRGL_COMPLETION_EVENT_MAGIC ||
        le16_to_cpu(request.version) != APPLE_VIRGL_PROTOCOL_VERSION_V3 ||
        le16_to_cpu(request.reserved) != 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-virgl completion receiver is malformed\n");
        virtqueue_push(vq, elem, 0);
        virtio_notify(VIRTIO_DEVICE(bridge->gpu), vq);
        g_free(elem);
        return true;
    }

    receiver = g_new(AppleVirglCompletionReceiver, 1);
    receiver->vq = vq;
    receiver->elem = elem;
    qemu_mutex_lock(&bridge->completion_lock);
    if (bridge->completion_shutdown || bridge->completion_resetting) {
        qemu_mutex_unlock(&bridge->completion_lock);
        virtqueue_push(vq, elem, 0);
        virtio_notify(VIRTIO_DEVICE(bridge->gpu), vq);
        apple_virgl_completion_receiver_free(receiver);
        return true;
    }
    g_queue_push_tail(&bridge->completion_receivers, receiver);
    schedule = apple_virgl_completion_maybe_schedule_locked(bridge);
    qemu_mutex_unlock(&bridge->completion_lock);

    if (schedule) {
        qemu_bh_schedule(bridge->completion_bh);
    }
    return true;
}

bool apple_virgl_bridge_has_context(AppleVirglBridge *bridge,
                                    uint32_t context_id)
{
    bool found;

    if (!bridge || context_id == 0) {
        return false;
    }
    qemu_mutex_lock(&bridge->lock);
    found = g_hash_table_contains(bridge->contexts,
                                  GUINT_TO_POINTER(context_id));
    qemu_mutex_unlock(&bridge->lock);
    return found;
}

int apple_virgl_bridge_context_create(AppleVirglBridge *bridge,
                                      uint32_t context_id,
                                      const char *name,
                                      uint32_t name_length)
{
    AppleVirglContextState *context;

    if (!bridge || !bridge->session || context_id == 0 || name_length > 64) {
        return -1;
    }
    context = g_new0(AppleVirglContextState, 1);
    context->transport_context_id = context_id;
    context->attached_resources = g_hash_table_new(g_direct_hash,
                                                    g_direct_equal);
    context->mappings = g_array_new(false, false, sizeof(AppleVirglMapping));

    qemu_mutex_lock(&bridge->lock);
    if (g_hash_table_contains(bridge->contexts,
                              GUINT_TO_POINTER(context_id))) {
        qemu_mutex_unlock(&bridge->lock);
        apple_virgl_context_free(context);
        return -1;
    }
    g_hash_table_insert(bridge->contexts, GUINT_TO_POINTER(context_id),
                        context);
    qemu_mutex_unlock(&bridge->lock);

    fprintf(stderr,
            "apple-virgl-qemu: context-create transport=%u name=%.*s\n",
            context_id, (int)name_length, name ? name : "");
    return 0;
}

int apple_virgl_bridge_context_destroy(AppleVirglBridge *bridge,
                                       uint32_t context_id)
{
    AppleVirglContextState *context;
    bool task_bound;
    uint32_t task_id;

    if (!bridge || context_id == 0) {
        return -1;
    }
    qemu_mutex_lock(&bridge->lock);
    context = g_hash_table_lookup(bridge->contexts,
                                  GUINT_TO_POINTER(context_id));
    if (!context) {
        qemu_mutex_unlock(&bridge->lock);
        return -1;
    }
    task_bound = context->task_bound;
    task_id = context->task_id;
    g_hash_table_remove(bridge->contexts, GUINT_TO_POINTER(context_id));
    qemu_mutex_unlock(&bridge->lock);
    if (task_bound) {
        qmu_destroy_task(bridge->session, task_id);
    }
    fprintf(stderr,
            "apple-virgl-qemu: context-destroy transport=%u task=%s%u\n",
            context_id, task_bound ? "" : "unbound/", task_id);
    return 0;
}

int apple_virgl_bridge_resource_create(AppleVirglBridge *bridge,
                                       uint32_t resource_id,
                                       uint64_t declared_size)
{
    AppleVirglResourceState *resource;

    if (!bridge || resource_id == 0 || declared_size == 0) {
        return -1;
    }
    resource = g_new0(AppleVirglResourceState, 1);
    resource->resource_id = resource_id;
    resource->declared_size = declared_size;
    qemu_mutex_lock(&bridge->lock);
    if (g_hash_table_contains(bridge->resources,
                              GUINT_TO_POINTER(resource_id))) {
        qemu_mutex_unlock(&bridge->lock);
        g_free(resource);
        return -1;
    }
    g_hash_table_insert(bridge->resources, GUINT_TO_POINTER(resource_id),
                        resource);
    apple_virgl_resource_watch_log(bridge, "create", 0, 0, resource_id,
                                   declared_size, 0);
    qemu_mutex_unlock(&bridge->lock);
    return 0;
}

bool apple_virgl_bridge_defer_resource_unref(AppleVirglBridge *bridge,
                                             uint32_t resource_id)
{
    AppleVirglResourceState *resource;
    bool deferred = false;

    if (!bridge || resource_id == 0) {
        return false;
    }

    qemu_mutex_lock(&bridge->lock);
    resource = g_hash_table_lookup(bridge->resources,
                                   GUINT_TO_POINTER(resource_id));
    if (resource && resource->direct_write_lease_count != 0 &&
        !bridge->resource_release_shutdown &&
        !bridge->resource_release_resetting) {
        if (!resource->direct_write_release_waiting) {
            resource->direct_write_release_waiting = true;
            bridge->resource_release_blocked_count++;
            VIRTIO_GPU_BASE(bridge->gpu)->renderer_blocked++;
            if (apple_virgl_resource_lifecycle_enabled()) {
                error_report("apple-virgl resource-unref: event=defer "
                             "resource=%u leases=%u renderer_blocked=%d",
                             resource_id, resource->direct_write_lease_count,
                             VIRTIO_GPU_BASE(bridge->gpu)->renderer_blocked);
            }
        }
        deferred = true;
    }
    qemu_mutex_unlock(&bridge->lock);
    return deferred;
}

void apple_virgl_bridge_resource_destroy(AppleVirglBridge *bridge,
                                         uint32_t resource_id)
{
    GHashTableIter iter;
    gpointer value;

    if (!bridge || resource_id == 0) {
        return;
    }
    qemu_mutex_lock(&bridge->lock);
    g_hash_table_iter_init(&iter, bridge->contexts);
    while (g_hash_table_iter_next(&iter, NULL, &value)) {
        AppleVirglContextState *context = value;
        uint32_t index = 0;

        g_hash_table_remove(context->attached_resources,
                            GUINT_TO_POINTER(resource_id));
        while (index < context->mappings->len) {
            AppleVirglMapping *mapping =
                &g_array_index(context->mappings, AppleVirglMapping, index);
            if (mapping->backing_resource_id == resource_id) {
                apple_virgl_mapping_watch_log(bridge, context,
                                              "resource-destroy-remove", 0,
                                              mapping);
                g_array_remove_index_fast(context->mappings, index);
            } else {
                ++index;
            }
        }
    }
    apple_virgl_resource_watch_log(bridge, "destroy", 0, 0, resource_id,
                                   0, 0);
    g_hash_table_remove(bridge->resources, GUINT_TO_POINTER(resource_id));
    qemu_mutex_unlock(&bridge->lock);
}

int apple_virgl_bridge_resource_attach_backing(AppleVirglBridge *bridge,
                                                uint32_t resource_id,
                                                const uint64_t *addrs,
                                                const struct iovec *iov,
                                                uint32_t iov_count)
{
    AppleVirglResourceState *resource;

    if (!bridge || !addrs || !iov || iov_count == 0) {
        return -1;
    }
    qemu_mutex_lock(&bridge->lock);
    resource = g_hash_table_lookup(bridge->resources,
                                   GUINT_TO_POINTER(resource_id));
    if (!resource || resource->iov) {
        qemu_mutex_unlock(&bridge->lock);
        return -1;
    }
    resource->backing_size = iov_size(iov, iov_count);
    if (resource->backing_size != resource->declared_size) {
        qemu_mutex_unlock(&bridge->lock);
        return -1;
    }
    resource->addrs = addrs;
    resource->iov = iov;
    resource->iov_count = iov_count;
    apple_virgl_resource_watch_log(bridge, "attach-backing", 0, 0,
                                   resource_id, resource->backing_size,
                                   resource->iov_count);
    qemu_mutex_unlock(&bridge->lock);
    return 0;
}

void apple_virgl_bridge_resource_detach_backing(AppleVirglBridge *bridge,
                                                 uint32_t resource_id)
{
    AppleVirglResourceState *resource;

    if (!bridge) {
        return;
    }
    qemu_mutex_lock(&bridge->lock);
    resource = g_hash_table_lookup(bridge->resources,
                                   GUINT_TO_POINTER(resource_id));
    if (resource) {
        apple_virgl_resource_watch_log(bridge, "detach-backing", 0, 0,
                                       resource_id, resource->backing_size,
                                       resource->iov_count);
        resource->addrs = NULL;
        resource->iov = NULL;
        resource->iov_count = 0;
        resource->backing_size = 0;
    }
    qemu_mutex_unlock(&bridge->lock);
}

int apple_virgl_bridge_context_attach_resource(AppleVirglBridge *bridge,
                                                uint32_t context_id,
                                                uint32_t resource_id)
{
    AppleVirglContextState *context;
    int result = -1;

    if (!bridge || context_id == 0 || resource_id == 0) {
        return -1;
    }
    qemu_mutex_lock(&bridge->lock);
    context = g_hash_table_lookup(bridge->contexts,
                                  GUINT_TO_POINTER(context_id));
    if (context && g_hash_table_contains(bridge->resources,
                                         GUINT_TO_POINTER(resource_id))) {
        g_hash_table_add(context->attached_resources,
                         GUINT_TO_POINTER(resource_id));
        apple_virgl_resource_watch_log(bridge, "context-attach", context_id,
                                       context->task_id, resource_id, 0, 0);
        result = 0;
    }
    qemu_mutex_unlock(&bridge->lock);
    return result;
}

int apple_virgl_bridge_context_detach_resource(AppleVirglBridge *bridge,
                                                uint32_t context_id,
                                                uint32_t resource_id)
{
    AppleVirglContextState *context;
    bool removed = false;

    if (!bridge || context_id == 0 || resource_id == 0) {
        return -1;
    }
    qemu_mutex_lock(&bridge->lock);
    context = g_hash_table_lookup(bridge->contexts,
                                  GUINT_TO_POINTER(context_id));
    if (context) {
        removed = g_hash_table_remove(context->attached_resources,
                                      GUINT_TO_POINTER(resource_id));
        if (removed) {
            apple_virgl_resource_watch_log(bridge, "context-detach",
                                           context_id, context->task_id,
                                           resource_id, 0, 0);
        }
    }
    qemu_mutex_unlock(&bridge->lock);
    return removed ? 0 : -1;
}

static void apple_virgl_store_mapping(AppleVirglBridge *bridge,
                                      AppleVirglContextState *context,
                                      uint16_t opcode,
                                      const AppleVirglMapping *mapping)
{
    uint32_t index;

    for (index = 0; index < context->mappings->len; ++index) {
        AppleVirglMapping *existing =
            &g_array_index(context->mappings, AppleVirglMapping, index);
        if (existing->gpu_va == mapping->gpu_va &&
            existing->length == mapping->length &&
            existing->backing_resource_id == mapping->backing_resource_id) {
            apple_virgl_mapping_watch_log(bridge, context, "map-replace",
                                          opcode, mapping);
            *existing = *mapping;
            return;
        }
    }
    apple_virgl_mapping_watch_log(bridge, context, "map-store", opcode,
                                  mapping);
    g_array_append_val(context->mappings, *mapping);
}

static bool apple_virgl_remove_mapping(AppleVirglBridge *bridge,
                                       AppleVirglContextState *context,
                                       uint64_t gpu_va, uint64_t length)
{
    uint32_t index = 0;
    bool removed = false;
    uint64_t end = gpu_va + length;
    AppleVirglMapping requested = {
        .gpu_va = gpu_va,
        .length = length,
    };

    apple_virgl_mapping_watch_log(bridge, context, "unmap-attempt",
                                  APPLE_VIRGL_SUBMIT_UNMAP_MEMORY,
                                  &requested);

    while (index < context->mappings->len) {
        AppleVirglMapping *mapping =
            &g_array_index(context->mappings, AppleVirglMapping, index);

        if (mapping->gpu_va >= gpu_va &&
            mapping->gpu_va + mapping->length <= end) {
            apple_virgl_mapping_watch_log(bridge, context, "unmap-remove",
                                          APPLE_VIRGL_SUBMIT_UNMAP_MEMORY,
                                          mapping);
            g_array_remove_index(context->mappings, index);
            removed = true;
        } else {
            ++index;
        }
    }
    if (!removed) {
        apple_virgl_mapping_watch_log(bridge, context, "unmap-miss",
                                      APPLE_VIRGL_SUBMIT_UNMAP_MEMORY,
                                      &requested);
    }
    return removed;
}

static uint32_t apple_virgl_fnv1a(const void *bytes, size_t size)
{
    const uint8_t *cursor = bytes;
    uint32_t hash = 0x811c9dc5u;
    size_t index;

    for (index = 0; index < size; ++index) {
        hash ^= cursor[index];
        hash *= 0x01000193u;
    }
    return hash;
}

static int apple_virgl_bridge_bind_task(AppleVirglBridge *bridge,
                                        uint32_t context_id,
                                        const AppleVirglTaskBindV4 *wire)
{
    AppleVirglContextState *context;
    const uint8_t *payload = (const uint8_t *)wire;
    uint32_t task_id_encoded = ldl_le_p(payload);
    uint32_t task_id = task_id_encoded >> 1;
    uint64_t vm_size = ldq_le_p(payload + 4);
    uint32_t task_root_pfn = ldl_le_p(payload + 12);
    bool is_kernel = (task_id_encoded & 1) != 0;

    qemu_mutex_lock(&bridge->lock);
    context = g_hash_table_lookup(bridge->contexts,
                                  GUINT_TO_POINTER(context_id));
    if (!context || context->task_bound ||
        apple_virgl_context_for_task_locked(bridge, task_id)) {
        qemu_mutex_unlock(&bridge->lock);
        return -1;
    }
    context->task_id = task_id;
    context->task_bound = true;
    qemu_mutex_unlock(&bridge->lock);

    if (qmu_define_task(bridge->session, task_id, task_root_pfn, vm_size) !=
        QMU_OK) {
        qemu_mutex_lock(&bridge->lock);
        context = g_hash_table_lookup(bridge->contexts,
                                      GUINT_TO_POINTER(context_id));
        if (context && context->task_bound && context->task_id == task_id) {
            context->task_bound = false;
            context->task_id = 0;
        }
        qemu_mutex_unlock(&bridge->lock);
        return -1;
    }

    fprintf(stderr,
            "apple-virgl-qemu: task-bind transport=%u task=%u kernel=%u root=0x%x vm=0x%" PRIx64 "\n",
            context_id, task_id, is_kernel ? 1 : 0, task_root_pfn, vm_size);
    return 0;
}

int apple_virgl_bridge_submit(AppleVirglBridge *bridge,
                              uint32_t context_id,
                              const void *bytes,
                              size_t size)
{
    AppleVirglSubmitView view;
    AppleVirglContextState *context;
    GArray *new_mappings = NULL;
    Error *local_err = NULL;
    uint32_t index;
    uint32_t task_id;
    qmu_status status;
    uint64_t submit;
    uint16_t opcode;
    uint32_t qmu_opcode;
    bool gpu_channel = false;
    bool display_channel = false;

    if (!bridge || !bridge->session || context_id == 0 ||
        !apple_virgl_protocol_decode_submit(bytes, size, &view, &local_err)) {
        if (local_err) {
            error_report_err(local_err);
        }
        return -1;
    }
    opcode = le16_to_cpu(view.header->opcode);
    if (opcode == APPLE_VIRGL_SUBMIT_BIND_TASK) {
        return apple_virgl_bridge_bind_task(
            bridge, context_id,
            (const AppleVirglTaskBindV4 *)view.payload);
    }
    switch (opcode) {
    case APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3:
        qmu_opcode = APPLE_VIRGL_QMU_ROOT_EXEC_INDIRECT3;
        break;
    case APPLE_VIRGL_SUBMIT_SET_OBJECT_LIST:
        qmu_opcode = APPLE_VIRGL_QMU_ROOT_SET_OBJECT_LIST;
        break;
    case APPLE_VIRGL_SUBMIT_GET_COMPUTE_INFO:
        qmu_opcode = APPLE_VIRGL_QMU_ROOT_GET_COMPUTE_INFO;
        break;
    case APPLE_VIRGL_SUBMIT_DELETE_RESOURCE:
        qmu_opcode = APPLE_VIRGL_QMU_ROOT_DELETE_RESOURCE;
        break;
    case APPLE_VIRGL_SUBMIT_MAP_MEMORY2:
        qmu_opcode = APPLE_VIRGL_QMU_GPU_MAP_MEMORY2;
        gpu_channel = true;
        break;
    case APPLE_VIRGL_SUBMIT_UNMAP_MEMORY:
        qmu_opcode = APPLE_VIRGL_QMU_GPU_UNMAP_MEMORY;
        gpu_channel = true;
        break;
    case APPLE_VIRGL_SUBMIT_SYNCHRONIZE_RESOURCES:
        qmu_opcode = APPLE_VIRGL_QMU_GPU_SYNCHRONIZE_RESOURCES;
        gpu_channel = true;
        break;
    case APPLE_VIRGL_SUBMIT_DISPLAY_SET_SHARED_STATE:
        qmu_opcode = APPLE_VIRGL_QMU_DISPLAY_SET_SHARED_STATE;
        display_channel = true;
        break;
    case APPLE_VIRGL_SUBMIT_DISPLAY_TRANSACTION3:
        qmu_opcode = APPLE_VIRGL_QMU_DISPLAY_TRANSACTION3;
        display_channel = true;
        break;
    default:
        return -1;
    }

    qemu_mutex_lock(&bridge->lock);
    context = g_hash_table_lookup(bridge->contexts,
                                  GUINT_TO_POINTER(context_id));
    if (!context || !context->task_bound) {
        qemu_mutex_unlock(&bridge->lock);
        return -1;
    }
    task_id = context->task_id;
    if (display_channel ? task_id != 0 : ldl_le_p(view.payload) != task_id) {
        qemu_mutex_unlock(&bridge->lock);
        error_report("apple-virgl submit task/transport mismatch: payload=%u transport=%u bound-task=%u display=%u",
                     ldl_le_p(view.payload), context_id, task_id,
                     display_channel ? 1 : 0);
        return -1;
    }
    if (opcode == APPLE_VIRGL_SUBMIT_MAP_MEMORY2 ||
        opcode == APPLE_VIRGL_SUBMIT_UNMAP_MEMORY) {
        apple_virgl_memory_map_wire_log(context, opcode, &view);
    }
    new_mappings = g_array_sized_new(false, false,
                                     sizeof(AppleVirglMapping),
                                     view.mapping_count);
    for (index = 0; index < view.mapping_count; ++index) {
        const AppleVirglSubmitMappingV1 *wire = &view.mappings[index];
        AppleVirglMapping mapping = {
            .gpu_va = le64_to_cpu(wire->gpu_va),
            .length = le64_to_cpu(wire->length),
            .backing_resource_id =
                le32_to_cpu(wire->backing_resource_id),
            .backing_offset = le32_to_cpu(wire->backing_offset),
            .apple_resource_id = le32_to_cpu(wire->apple_resource_id),
        };
        AppleVirglResourceState *resource =
            g_hash_table_lookup(bridge->resources,
                                GUINT_TO_POINTER(
                                    mapping.backing_resource_id));
        bool mapping_shape_valid =
            mapping.gpu_va != 0 && mapping.length != 0 &&
            mapping.length <= UINT64_MAX - mapping.gpu_va &&
            mapping.backing_resource_id != 0 &&
            le32_to_cpu(wire->reserved) == 0;
        bool resource_has_iov = resource && resource->iov;
        bool resource_attached = resource &&
            g_hash_table_contains(context->attached_resources,
                                  GUINT_TO_POINTER(
                                      mapping.backing_resource_id));
        bool mapping_range_valid = resource &&
            mapping.backing_offset <= resource->backing_size &&
            mapping.length <= resource->backing_size - mapping.backing_offset;

        uint32_t previous;

        if (!mapping_shape_valid || !resource_has_iov ||
            !resource_attached || !mapping_range_valid) {
            uint64_t backing_size = resource ? resource->backing_size : 0;
            uint32_t iov_count = resource ? resource->iov_count : 0;

            qemu_mutex_unlock(&bridge->lock);
            g_array_unref(new_mappings);
            error_report("apple-virgl reject mapping: transport=%u task=%u opcode=%u index=%u "
                         "va=0x%" PRIx64 " length=0x%" PRIx64
                         " backing=%u offset=0x%x apple=%u reserved=0x%x "
                         "shape=%u resource=%u iov=%u iov_count=%u attached=%u "
                         "range=%u backing_size=0x%" PRIx64,
                         context_id, task_id, opcode, index,
                         mapping.gpu_va, mapping.length,
                         mapping.backing_resource_id, mapping.backing_offset,
                         mapping.apple_resource_id, le32_to_cpu(wire->reserved),
                         mapping_shape_valid ? 1 : 0, resource ? 1 : 0,
                         resource_has_iov ? 1 : 0, iov_count,
                         resource_attached ? 1 : 0,
                         mapping_range_valid ? 1 : 0, backing_size);
            return -1;
        }
        for (previous = 0; previous < new_mappings->len; ++previous) {
            AppleVirglMapping *other =
                &g_array_index(new_mappings, AppleVirglMapping, previous);

            if (mapping.apple_resource_id != 0 &&
                other->apple_resource_id == mapping.apple_resource_id) {
                qemu_mutex_unlock(&bridge->lock);
                g_array_unref(new_mappings);
                error_report("apple-virgl reject duplicate mapping: transport=%u task=%u "
                             "opcode=%u index=%u apple=%u previous=%u",
                             context_id, task_id, opcode, index,
                             mapping.apple_resource_id, previous);
                return -1;
            }
        }
        g_array_append_val(new_mappings, mapping);
    }
    for (index = 0; index < new_mappings->len; ++index) {
        AppleVirglMapping *mapping =
            &g_array_index(new_mappings, AppleVirglMapping, index);

        apple_virgl_store_mapping(bridge, context, opcode, mapping);
    }
    if (opcode == APPLE_VIRGL_SUBMIT_UNMAP_MEMORY &&
        !apple_virgl_remove_mapping(bridge, context,
                                    ldq_le_p(view.payload + 4),
                                    ldq_le_p(view.payload + 12))) {
        qemu_mutex_unlock(&bridge->lock);
        g_array_unref(new_mappings);
        error_report("apple-virgl reject unmap: transport=%u task=%u va=0x%" PRIx64
                     " length=0x%" PRIx64,
                     context_id, task_id, ldq_le_p(view.payload + 4),
                     ldq_le_p(view.payload + 12));
        return -1;
    }
    submit = ++bridge->submit_count;
    qemu_mutex_unlock(&bridge->lock);
    g_array_unref(new_mappings);

    for (index = 0; index < view.mapping_count; ++index) {
        const AppleVirglSubmitMappingV1 *wire = &view.mappings[index];
        uint32_t apple_resource_id =
            le32_to_cpu(wire->apple_resource_id);

        if (apple_resource_id != 0) {
            qmu_register_buffer(bridge->session, apple_resource_id,
                                task_id,
                                le64_to_cpu(wire->gpu_va),
                                le64_to_cpu(wire->length));
        }
    }

    if (submit <= 64) {
        fprintf(stderr,
                "apple-virgl-qemu: submit=%" PRIu64
                " transport=%u task=%u opcode=%u mappings=%u payload=%u"
                " completion=%u/%u hash=0x%08x\n",
                submit, context_id, task_id, opcode, view.mapping_count,
                view.payload_bytes, view.completion_channel_id,
                view.completion_stamp,
                apple_virgl_fnv1a(view.payload, view.payload_bytes));
    }
    if (display_channel) {
        status = qmu_submit_display_channel(bridge->session, qmu_opcode,
                                            view.payload,
                                            view.payload_bytes);
    } else if (gpu_channel) {
        status = qmu_submit_gpu_channel(bridge->session, 0, qmu_opcode,
                                        view.payload, view.payload_bytes);
    } else if (opcode == APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3 &&
               view.completion_channel_id != 0) {
        status = qmu_submit_root_exec_indirect3_with_completion(
            bridge->session, view.payload, view.payload_bytes,
            view.completion_channel_id, view.completion_stamp);
    } else {
        status = qmu_submit_root_fifo(bridge->session, qmu_opcode,
                                      view.payload, view.payload_bytes);
    }
    if (status != QMU_OK) {
        qemu_mutex_lock(&bridge->lock);
        context = g_hash_table_lookup(bridge->contexts,
                                      GUINT_TO_POINTER(context_id));
        if (context && context->task_id == task_id) {
            if (opcode == APPLE_VIRGL_SUBMIT_UNMAP_MEMORY) {
                AppleVirglMapping requested = {
                    .gpu_va = ldq_le_p(view.payload + 4),
                    .length = ldq_le_p(view.payload + 12),
                };

                apple_virgl_mapping_watch_log(bridge, context,
                                              "qmu-submit-failed", opcode,
                                              &requested);
            }
            for (index = 0; index < view.mapping_count; ++index) {
                const AppleVirglSubmitMappingV1 *wire = &view.mappings[index];
                AppleVirglMapping mapping = {
                    .gpu_va = le64_to_cpu(wire->gpu_va),
                    .length = le64_to_cpu(wire->length),
                    .backing_resource_id =
                        le32_to_cpu(wire->backing_resource_id),
                    .backing_offset = le32_to_cpu(wire->backing_offset),
                    .apple_resource_id = le32_to_cpu(wire->apple_resource_id),
                };

                apple_virgl_mapping_watch_log(bridge, context,
                                              "qmu-submit-failed", opcode,
                                              &mapping);
            }
        }
        qemu_mutex_unlock(&bridge->lock);
        error_report("apple-virgl qmu submit failed: transport=%u task=%u opcode=%u "
                     "qmu_opcode=0x%x status=%d payload=%u completion=%u/%u",
                     context_id, task_id, opcode, qmu_opcode, status,
                     view.payload_bytes, view.completion_channel_id,
                     view.completion_stamp);
        return -1;
    }
    qemu_mutex_lock(&bridge->lock);
    context = g_hash_table_lookup(bridge->contexts,
                                  GUINT_TO_POINTER(context_id));
    if (context && context->task_id == task_id) {
        if (opcode == APPLE_VIRGL_SUBMIT_UNMAP_MEMORY) {
            AppleVirglMapping requested = {
                .gpu_va = ldq_le_p(view.payload + 4),
                .length = ldq_le_p(view.payload + 12),
            };

            apple_virgl_mapping_watch_log(bridge, context, "qmu-submit-ok",
                                          opcode, &requested);
        }
        for (index = 0; index < view.mapping_count; ++index) {
            const AppleVirglSubmitMappingV1 *wire = &view.mappings[index];
            AppleVirglMapping mapping = {
                .gpu_va = le64_to_cpu(wire->gpu_va),
                .length = le64_to_cpu(wire->length),
                .backing_resource_id = le32_to_cpu(wire->backing_resource_id),
                .backing_offset = le32_to_cpu(wire->backing_offset),
                .apple_resource_id = le32_to_cpu(wire->apple_resource_id),
            };

            apple_virgl_mapping_watch_log(bridge, context, "qmu-submit-ok",
                                          opcode, &mapping);
        }
    }
    qemu_mutex_unlock(&bridge->lock);
    return 0;
}
