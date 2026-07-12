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

struct AppleVirglBridge {
    VirtIOGPU *gpu;
    QemuMutex lock;
    GHashTable *contexts;
    GHashTable *resources;
    qmu_session *session;
    uint64_t submit_count;
    uint64_t frame_count;
};

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
    if (bridge->session) {
        qmu_session_begin_shutdown(bridge->session);
        qmu_destroy(bridge->session);
    }
    g_hash_table_destroy(bridge->contexts);
    g_hash_table_destroy(bridge->resources);
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
                                        uint32_t task_id)
{
    AppleVirglContextState *context;

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

    if (qmu_define_task(bridge->session, task_id, 0, 0) != QMU_OK) {
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
            "apple-virgl-qemu: task-bind transport=%u task=%u\n",
            context_id, task_id);
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
            bridge, context_id, ldl_le_p(view.payload));
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
    return status == QMU_OK ? 0 : -1;
}
