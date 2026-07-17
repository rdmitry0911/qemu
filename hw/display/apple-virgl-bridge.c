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
#define APPLE_VIRGL_QMU_GPU_DELETE_RESOURCE 0x25
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
} AppleVirglResourceState;

typedef struct AppleVirglMapping {
    uint64_t gpu_va;
    uint64_t length;
    uint32_t backing_resource_id;
    uint32_t backing_offset;
    uint32_t apple_resource_id;
} AppleVirglMapping;

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

struct AppleVirglBridge {
    VirtIOGPU *gpu;
    QemuMutex lock;
    GHashTable *contexts;
    GHashTable *resources;
    qmu_session *session;
    uint64_t submit_count;
    uint64_t frame_count;
    QemuMutex completion_lock;
    GQueue completion_receivers;
    GQueue completion_stamps;
    QEMUBH *completion_bh;
    bool completion_bh_scheduled;
    bool completion_shutdown;
    bool completion_resetting;
};

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

    qemu_mutex_lock(&bridge->lock);
    resource = apple_virgl_resolve_gpu_memory_locked(
        bridge, task_id, gpu_va, size, &resource_offset);
    if (resource) {
        result = iov_to_buf(resource->iov, resource->iov_count,
                            resource_offset, bytes, size) == size ? 0 : -1;
    }

    qemu_mutex_unlock(&bridge->lock);
    return result;
}

static int apple_virgl_write_gpu_memory(void *opaque, uint32_t task_id,
                                        uint64_t gpu_va, const void *bytes,
                                        size_t size)
{
    AppleVirglBridge *bridge = opaque;
    AppleVirglResourceState *resource;
    const uint8_t *source = bytes;
    uint64_t resource_offset;
    uint64_t segment_offset;
    size_t remaining = size;
    uint32_t index;
    int result = -1;

    qemu_mutex_lock(&bridge->lock);
    resource = apple_virgl_resolve_gpu_memory_locked(
        bridge, task_id, gpu_va, size, &resource_offset);
    if (!resource) {
        goto out;
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
            goto out;
        }
        source += chunk;
        remaining -= chunk;
        segment_offset = 0;
    }
    result = remaining == 0 ? 0 : -1;

out:
    qemu_mutex_unlock(&bridge->lock);
    return result;
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
    bridge->contexts = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                              NULL, apple_virgl_context_free);
    bridge->resources = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                               NULL, apple_virgl_resource_free);

    callbacks.user_ctx = bridge;
    callbacks.map_gpa = apple_virgl_map_gpa;
    callbacks.unmap_gpa = apple_virgl_unmap_gpa;
    callbacks.read_memory = apple_virgl_read_memory;
    callbacks.write_memory = apple_virgl_write_memory;
    callbacks.read_gpu_memory = apple_virgl_read_gpu_memory;
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
    if (bridge->session) {
        qmu_session_begin_shutdown(bridge->session);
        qmu_destroy(bridge->session);
    }
    apple_virgl_bridge_clear_completion_state(bridge, false);
    if (bridge->completion_bh) {
        qemu_bh_delete(bridge->completion_bh);
    }
    g_hash_table_destroy(bridge->contexts);
    g_hash_table_destroy(bridge->resources);
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
    qemu_mutex_unlock(&bridge->lock);
    return 0;
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
                g_array_remove_index_fast(context->mappings, index);
            } else {
                ++index;
            }
        }
    }
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
    }
    qemu_mutex_unlock(&bridge->lock);
    return removed ? 0 : -1;
}

static void apple_virgl_store_mapping(AppleVirglContextState *context,
                                      const AppleVirglMapping *mapping)
{
    uint32_t index;

    for (index = 0; index < context->mappings->len; ++index) {
        AppleVirglMapping *existing =
            &g_array_index(context->mappings, AppleVirglMapping, index);
        if (existing->gpu_va == mapping->gpu_va &&
            existing->length == mapping->length &&
            existing->backing_resource_id == mapping->backing_resource_id) {
            *existing = *mapping;
            return;
        }
    }
    g_array_append_val(context->mappings, *mapping);
}

static bool apple_virgl_remove_mapping(AppleVirglContextState *context,
                                       uint64_t gpu_va, uint64_t length)
{
    uint32_t index = 0;
    bool removed = false;
    uint64_t end = gpu_va + length;

    while (index < context->mappings->len) {
        AppleVirglMapping *mapping =
            &g_array_index(context->mappings, AppleVirglMapping, index);

        if (mapping->gpu_va >= gpu_va &&
            mapping->gpu_va + mapping->length <= end) {
            g_array_remove_index(context->mappings, index);
            removed = true;
        } else {
            ++index;
        }
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
    uint32_t gpu_channel_id = 0;
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
        /* Native PVG sends DELETE_RESOURCE on its Immediate child FIFO
         * (channel 2), where 0x25 is GPU_DELETE_RESOURCE.  The numerical
         * root opcode is the same, but the FIFO namespace is not. */
        qmu_opcode = APPLE_VIRGL_QMU_GPU_DELETE_RESOURCE;
        gpu_channel_id = 2;
        gpu_channel = true;
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

        uint32_t previous;

        if (mapping.gpu_va == 0 || mapping.length == 0 ||
            mapping.length > UINT64_MAX - mapping.gpu_va ||
            mapping.backing_resource_id == 0 ||
            le32_to_cpu(wire->reserved) != 0 || !resource ||
            !resource->iov ||
            !g_hash_table_contains(context->attached_resources,
                                   GUINT_TO_POINTER(
                                       mapping.backing_resource_id)) ||
            mapping.backing_offset > resource->backing_size ||
            mapping.length > resource->backing_size - mapping.backing_offset) {
            qemu_mutex_unlock(&bridge->lock);
            g_array_unref(new_mappings);
            return -1;
        }
        for (previous = 0; previous < new_mappings->len; ++previous) {
            AppleVirglMapping *other =
                &g_array_index(new_mappings, AppleVirglMapping, previous);

            if (mapping.apple_resource_id != 0 &&
                other->apple_resource_id == mapping.apple_resource_id) {
                qemu_mutex_unlock(&bridge->lock);
                g_array_unref(new_mappings);
                return -1;
            }
        }
        g_array_append_val(new_mappings, mapping);
    }
    for (index = 0; index < new_mappings->len; ++index) {
        AppleVirglMapping *mapping =
            &g_array_index(new_mappings, AppleVirglMapping, index);

        apple_virgl_store_mapping(context, mapping);
    }
    if (opcode == APPLE_VIRGL_SUBMIT_UNMAP_MEMORY &&
        !apple_virgl_remove_mapping(context, ldq_le_p(view.payload + 4),
                                    ldq_le_p(view.payload + 12))) {
        qemu_mutex_unlock(&bridge->lock);
        g_array_unref(new_mappings);
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
        status = qmu_submit_gpu_channel(bridge->session, gpu_channel_id,
                                        qmu_opcode, view.payload,
                                        view.payload_bytes);
    } else if (opcode == APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3 &&
               view.completion_channel_id != 0) {
        status = qmu_submit_root_exec_indirect3_with_completion(
            bridge->session, view.payload, view.payload_bytes,
            view.completion_channel_id, view.completion_stamp);
    } else {
        status = qmu_submit_root_fifo(bridge->session, qmu_opcode,
                                      view.payload, view.payload_bytes);
    }
    return status == QMU_OK ? 0 : -1;
}
