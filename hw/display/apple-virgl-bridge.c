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
#include "block/thread-pool.h"
#include "hw/virtio/virtio.h"
#include "hw/virtio/virtio-gpu.h"
#include "hw/virtio/apple-virgl-bridge.h"
#include "hw/virtio/apple-virgl-protocol.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "ui/console.h"
#include "qmu/qmetal_unified.h"
#include "apple-virgl-frame-pump.h"
#include "hw/display/apple-virgl-cursor-stage.h"
#include "hw/display/apple-virgl-present-stage.h"
#include "hw/display/apple-virgl-qmetal-abi.h"

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

typedef struct AppleVirglFrameSubmitJob {
    AppleVirglBridge *bridge;
} AppleVirglFrameSubmitJob;

struct AppleVirglBridge {
    VirtIOGPU *gpu;
    QemuMutex lock;
    /* Serialize every QMetal producer, including the V5 IOSurface backing
     * retirement and the frame-pump capture/submit edges.  The one exception
     * is new_frame_signal's required QMetal acknowledgement: it is callback
     * re-entry and must not take a host lock before consume returns. */
    QemuRecMutex command_lock;
    /* Reset and final teardown are device-lifecycle operations.  QEMU invokes
     * them with the bridge lifetime serialized; this lock additionally keeps
     * two reset paths from interleaving their quiesce/invalidate sequences. */
    QemuMutex lifecycle_lock;
    GHashTable *contexts;
    GHashTable *resources;
    qmu_session *session;
    uint64_t submit_count;
    /* A counted session lease closes every QMU producer before transport
     * reset clears QMetal's display-delivery source. The mutex only protects
     * state; it is never held across a QMU call. */
    QemuMutex session_gate_lock;
    QemuCond session_gate_idle;
    uint32_t session_gate_leases;
    bool session_gate_resetting;
    bool session_gate_shutdown;
    /* QEMU-BH capture and serial owner-worker submit are separated from the
     * QMetal callback. The state machine itself is guarded by this mutex. */
    QemuMutex frame_pump_lock;
    QemuCond frame_pump_idle;
    AppleVirglFramePump frame_pump;
    QEMUBH *new_frame_bh;
    ThreadPool *frame_submit_pool;
    /* QMetal owner-completion callbacks run outside the QEMU main loop.  They
     * copy into this FIFO under present_lock; only present_bh touches UI APIs.
     * present_surface is a console-owned, non-owning identity marker. */
    QemuMutex present_lock;
    AppleVirglPresentStage present_stage;
    QEMUBH *present_bh;
    DisplaySurface *present_surface;
    bool present_bh_scheduled;
    /* Cursor callbacks stage ephemeral bytes for the guarded QEMU UI BH. */
    QemuMutex cursor_lock;
    AppleVirglCursorStage cursor_stage;
    QEMUBH *cursor_bh;
    bool cursor_bh_scheduled;
    bool cursor_visible;
    QemuMutex completion_lock;
    GQueue completion_receivers;
    GQueue completion_stamps;
    QEMUBH *completion_bh;
    bool completion_bh_scheduled;
    bool completion_shutdown;
    bool completion_resetting;
};

static bool apple_virgl_session_lease_begin(AppleVirglBridge *bridge,
                                             qmu_session **out_session)
{
    qmu_session *session;

    if (!bridge) {
        return false;
    }

    qemu_mutex_lock(&bridge->session_gate_lock);
    if (bridge->session_gate_resetting || bridge->session_gate_shutdown ||
        !bridge->session) {
        qemu_mutex_unlock(&bridge->session_gate_lock);
        return false;
    }
    bridge->session_gate_leases++;
    session = bridge->session;
    qemu_mutex_unlock(&bridge->session_gate_lock);
    if (out_session) {
        *out_session = session;
    }
    return true;
}

static void apple_virgl_session_lease_end(AppleVirglBridge *bridge)
{
    if (!bridge) {
        return;
    }

    qemu_mutex_lock(&bridge->session_gate_lock);
    if (bridge->session_gate_leases != 0) {
        bridge->session_gate_leases--;
    }
    if (bridge->session_gate_leases == 0) {
        qemu_cond_broadcast(&bridge->session_gate_idle);
    }
    qemu_mutex_unlock(&bridge->session_gate_lock);
}

static qmu_session *apple_virgl_session_gate_close(AppleVirglBridge *bridge,
                                                    bool shutdown)
{
    qmu_session *session;

    qemu_mutex_lock(&bridge->session_gate_lock);
    bridge->session_gate_resetting = true;
    bridge->session_gate_shutdown = bridge->session_gate_shutdown || shutdown;
    session = bridge->session;
    qemu_mutex_unlock(&bridge->session_gate_lock);
    return session;
}

static void apple_virgl_session_gate_wait_idle(AppleVirglBridge *bridge)
{
    qemu_mutex_lock(&bridge->session_gate_lock);
    while (bridge->session_gate_leases != 0) {
        qemu_cond_wait(&bridge->session_gate_idle,
                       &bridge->session_gate_lock);
    }
    qemu_mutex_unlock(&bridge->session_gate_lock);
}

static void apple_virgl_session_gate_resume(AppleVirglBridge *bridge)
{
    qemu_mutex_lock(&bridge->session_gate_lock);
    if (!bridge->session_gate_shutdown) {
        bridge->session_gate_resetting = false;
    }
    qemu_mutex_unlock(&bridge->session_gate_lock);
}

static void apple_virgl_session_gate_detach(AppleVirglBridge *bridge,
                                            qmu_session *session)
{
    qemu_mutex_lock(&bridge->session_gate_lock);
    if (bridge->session == session) {
        bridge->session = NULL;
    }
    qemu_mutex_unlock(&bridge->session_gate_lock);
}

static void apple_virgl_frame_pump_schedule_actions(
    AppleVirglBridge *bridge, AppleVirglFramePumpActions actions);

static void apple_virgl_frame_pump_quiesce(AppleVirglBridge *bridge,
                                            bool shutdown)
{
    qemu_mutex_lock(&bridge->frame_pump_lock);
    apple_virgl_frame_pump_begin_reset(&bridge->frame_pump, shutdown);
    qemu_mutex_unlock(&bridge->frame_pump_lock);

    if (bridge->new_frame_bh) {
        qemu_bh_cancel(bridge->new_frame_bh);
    }
    if (bridge->frame_submit_pool) {
        thread_pool_wait(bridge->frame_submit_pool);
    }

    qemu_mutex_lock(&bridge->frame_pump_lock);
    while (!apple_virgl_frame_pump_is_idle(&bridge->frame_pump)) {
        qemu_cond_wait(&bridge->frame_pump_idle, &bridge->frame_pump_lock);
    }
    qemu_mutex_unlock(&bridge->frame_pump_lock);
}

static void apple_virgl_bridge_frame_pump_resume(AppleVirglBridge *bridge)
{
    qemu_mutex_lock(&bridge->frame_pump_lock);
    apple_virgl_frame_pump_resume(&bridge->frame_pump);
    qemu_mutex_unlock(&bridge->frame_pump_lock);
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

static int apple_virgl_frame_submit_job(void *opaque)
{
    AppleVirglFrameSubmitJob *job = opaque;
    AppleVirglBridge *bridge = job ? job->bridge : NULL;
    AppleVirglFramePumpActions actions = { 0 };
    qmu_session *session = NULL;
    struct qmu_vulkan_ctx *vk = NULL;
    uint64_t generation = 0;
    int submit_result = -1;

    if (!bridge) {
        return 0;
    }

    qemu_mutex_lock(&bridge->frame_pump_lock);
    if (!apple_virgl_frame_pump_begin_submit(&bridge->frame_pump,
                                              &generation)) {
        qemu_cond_broadcast(&bridge->frame_pump_idle);
        qemu_mutex_unlock(&bridge->frame_pump_lock);
        return 0;
    }
    qemu_mutex_unlock(&bridge->frame_pump_lock);

    if (apple_virgl_session_lease_begin(bridge, &session)) {
        qemu_rec_mutex_lock(&bridge->command_lock);
        vk = qmu_session_get_vulkan(session);
        if (vk) {
            submit_result = qmu_vk_submit_captured_display_frame(vk);
        }
        qemu_rec_mutex_unlock(&bridge->command_lock);
        apple_virgl_session_lease_end(bridge);
    }

    qemu_mutex_lock(&bridge->frame_pump_lock);
    actions = apple_virgl_frame_pump_finish_submit(&bridge->frame_pump,
                                                    generation,
                                                    submit_result);
    qemu_cond_broadcast(&bridge->frame_pump_idle);
    qemu_mutex_unlock(&bridge->frame_pump_lock);
    apple_virgl_frame_pump_schedule_actions(bridge, actions);
    return 0;
}

static void apple_virgl_frame_pump_schedule_actions(
    AppleVirglBridge *bridge, AppleVirglFramePumpActions actions)
{
    AppleVirglFrameSubmitJob *job = NULL;

    if (!bridge) {
        return;
    }

    /* Queue the action while holding the same mutex reset uses to fence the
     * state machine.  Otherwise reset could cancel/drain just before an old
     * action becomes visible, leaving a BH/job with a freed bridge.  The BH
     * and worker acquire this mutex before doing any QMU work. */
    qemu_mutex_lock(&bridge->frame_pump_lock);
    if (!bridge->frame_pump.resetting && !bridge->frame_pump.shutdown) {
        if (actions.schedule_bh && bridge->frame_pump.bh_scheduled &&
            bridge->new_frame_bh) {
            qemu_bh_schedule(bridge->new_frame_bh);
        }
        if (actions.schedule_submit && bridge->frame_pump.submit_scheduled &&
            bridge->frame_submit_pool) {
            job = g_new0(AppleVirglFrameSubmitJob, 1);
            job->bridge = bridge;
            thread_pool_submit(bridge->frame_submit_pool,
                               apple_virgl_frame_submit_job,
                               job,
                               g_free);
        }
    }
    qemu_mutex_unlock(&bridge->frame_pump_lock);
}

static void apple_virgl_new_frame_bh(void *opaque)
{
    AppleVirglBridge *bridge = opaque;
    AppleVirglFramePumpActions actions = { 0 };
    qmu_session *session = NULL;
    struct qmu_vulkan_ctx *vk = NULL;
    uint64_t generation = 0;
    int capture_result = -1;

    if (!bridge) {
        return;
    }

    qemu_mutex_lock(&bridge->frame_pump_lock);
    if (!apple_virgl_frame_pump_begin_bh(&bridge->frame_pump, &generation)) {
        qemu_cond_broadcast(&bridge->frame_pump_idle);
        qemu_mutex_unlock(&bridge->frame_pump_lock);
        return;
    }
    qemu_mutex_unlock(&bridge->frame_pump_lock);

    /* This is the reference-shaped main-loop edge: capture only. Submission
     * stays on the bridge-owned serial worker below. */
    if (apple_virgl_session_lease_begin(bridge, &session)) {
        qemu_rec_mutex_lock(&bridge->command_lock);
        vk = qmu_session_get_vulkan(session);
        if (vk) {
            capture_result = qmu_vk_capture_display_frame_request(vk);
        }
        qemu_rec_mutex_unlock(&bridge->command_lock);
        apple_virgl_session_lease_end(bridge);
    }

    qemu_mutex_lock(&bridge->frame_pump_lock);
    actions = apple_virgl_frame_pump_finish_capture(&bridge->frame_pump,
                                                     generation,
                                                     capture_result);
    qemu_cond_broadcast(&bridge->frame_pump_idle);
    qemu_mutex_unlock(&bridge->frame_pump_lock);
    apple_virgl_frame_pump_schedule_actions(bridge, actions);
}

static void apple_virgl_new_frame_signal(void *opaque)
{
    AppleVirglBridge *bridge = opaque;
    qmu_session *session = NULL;
    struct qmu_vulkan_ctx *vk = NULL;
    AppleVirglFramePumpActions actions = { 0 };

    if (!bridge || !apple_virgl_session_lease_begin(bridge, &session)) {
        return;
    }

    vk = qmu_session_get_vulkan(session);
    if (vk) {
        /* The one permitted callback re-entry: signal acknowledgement takes
         * QMetal's recursive signal lock and must happen before BH handoff. */
        qmu_vk_consume_current_frame_signal(vk);
    }
    apple_virgl_session_lease_end(bridge);

    if (!vk) {
        return;
    }
    qemu_mutex_lock(&bridge->frame_pump_lock);
    actions.schedule_bh =
        apple_virgl_frame_pump_new_signal(&bridge->frame_pump);
    qemu_mutex_unlock(&bridge->frame_pump_lock);
    apple_virgl_frame_pump_schedule_actions(bridge, actions);
}

static bool apple_virgl_presenter_apply_job(
    AppleVirglBridge *bridge, const AppleVirglPresentStageJob *job)
{
    QemuConsole *con;
    DisplaySurface *surface;
    bool disable_gl_scanout;
    uint32_t row;

    assert(bql_locked());
    if (!bridge || !job || !job->payload_valid || !job->pixels) {
        return false;
    }

    con = bridge->gpu ? bridge->gpu->parent_obj.scanout[0].con : NULL;
    if (!con) {
        return false;
    }

    surface = qemu_console_surface(con);
    if (!surface || surface != bridge->present_surface ||
        surface_width(surface) != (int)job->width ||
        surface_height(surface) != (int)job->height ||
        surface_format(surface) != PIXMAN_x8r8g8b8 ||
        surface_stride(surface) < (int)job->stride) {
        /* A GL/DMABUF scanout has no CPU surface.  Replacing it selects the
         * CPU-surface path; notify its GL listener exactly on that transition. */
        disable_gl_scanout = surface == NULL;
        surface = qemu_create_displaysurface((int)job->width,
                                             (int)job->height);
        if (!surface) {
            return false;
        }
        dpy_gfx_replace_surface(con, surface);
        bridge->present_surface = surface;
        if (disable_gl_scanout) {
            dpy_gl_scanout_disable(con);
        }
    }

    if (!surface_data(surface)) {
        return false;
    }
    for (row = 0; row < job->height; row++) {
        memcpy((uint8_t *)surface_data(surface) +
                   (size_t)row * surface_stride(surface),
               job->pixels + (size_t)row * job->stride, job->stride);
    }
    dpy_gfx_update_full(con);
    return true;
}

static void apple_virgl_present_bh(void *opaque)
{
    AppleVirglBridge *bridge = opaque;
    AppleVirglFramePumpActions pending_actions = { 0 };

    if (!bridge) {
        return;
    }
    assert(bql_locked());

    for (;;) {
        AppleVirglPresentStageJob *job;
        AppleVirglFramePumpActions actions;

        qemu_mutex_lock(&bridge->present_lock);
        job = apple_virgl_present_stage_take(&bridge->present_stage);
        if (!job) {
            bridge->present_bh_scheduled = false;
            qemu_mutex_unlock(&bridge->present_lock);
            break;
        }
        qemu_mutex_unlock(&bridge->present_lock);

        (void)apple_virgl_presenter_apply_job(bridge, job);

        /* This is the only owner-completion retirement edge.  It runs after
         * the console update/drop so the next QMetal submit cannot overtake
         * the visible frame hand-off. */
        qemu_mutex_lock(&bridge->frame_pump_lock);
        actions = apple_virgl_frame_pump_frame_completed(&bridge->frame_pump);
        qemu_mutex_unlock(&bridge->frame_pump_lock);
        pending_actions.schedule_bh |= actions.schedule_bh;
        pending_actions.schedule_submit |= actions.schedule_submit;
        apple_virgl_present_stage_job_free(job);
    }

    apple_virgl_frame_pump_schedule_actions(bridge, pending_actions);
}

static void apple_virgl_render_frame_complete(
    void *opaque, const qmu_render_frame_completion *completion)
{
    AppleVirglBridge *bridge = opaque;
    bool queued;

    if (!bridge || !completion) {
        return;
    }

    /* QMetal retains completion pixels only for this callback.  Keep the
     * lock through copy and schedule so reset cannot free the guarded BH in
     * between an accepted owner completion and its main-loop hand-off. */
    qemu_mutex_lock(&bridge->present_lock);
    queued = apple_virgl_present_stage_enqueue(
        &bridge->present_stage, completion->frame_expected != 0,
        completion->pixels, completion->width, completion->height,
        completion->stride);
    if (queued && !bridge->present_bh_scheduled && bridge->present_bh) {
        bridge->present_bh_scheduled = true;
        qemu_bh_schedule(bridge->present_bh);
    }
    qemu_mutex_unlock(&bridge->present_lock);
}

static void apple_virgl_presenter_quiesce(AppleVirglBridge *bridge,
                                          bool shutdown)
{
    if (!bridge) {
        return;
    }

    qemu_mutex_lock(&bridge->present_lock);
    apple_virgl_present_stage_begin_reset(&bridge->present_stage, shutdown);
    bridge->present_bh_scheduled = false;
    /* This is a non-owning marker only.  Generic virtio/virgl reset owns the
     * actual console replacement and may already have freed this surface. */
    bridge->present_surface = NULL;
    qemu_mutex_unlock(&bridge->present_lock);

    if (bridge->present_bh) {
        qemu_bh_cancel(bridge->present_bh);
    }
}

static void apple_virgl_presenter_resume(AppleVirglBridge *bridge)
{
    if (!bridge) {
        return;
    }

    qemu_mutex_lock(&bridge->present_lock);
    apple_virgl_present_stage_resume(&bridge->present_stage);
    qemu_mutex_unlock(&bridge->present_lock);
}

static void apple_virgl_cursor_update(AppleVirglBridge *bridge,
                                      uint32_t display_id)
{
    QemuConsole *con;
    qmu_session *session = NULL;
    uint32_t packed_position = 0xffffffffu;
    qmu_status status = QMU_ERROR;

    assert(bql_locked());
    if (!bridge || display_id != 0) {
        return;
    }
    con = bridge->gpu ? bridge->gpu->parent_obj.scanout[0].con : NULL;
    if (!con || !apple_virgl_session_lease_begin(bridge, &session)) {
        return;
    }
    status = qmu_get_display_cursor_position(session, display_id,
                                             &packed_position);
    apple_virgl_session_lease_end(bridge);
    if (status != QMU_OK) {
        return;
    }

    /* Decode packed y:x; QMetal's 0xffffffff sentinel stays (-1, -1). */
    dpy_mouse_set(con, (int)(int16_t)(packed_position & 0xffffu),
                  (int)(int16_t)(packed_position >> 16),
                  bridge->cursor_visible);
}

static void apple_virgl_cursor_apply_glyph(
    AppleVirglBridge *bridge, const AppleVirglCursorStageJob *job)
{
    QemuConsole *con;
    QEMUCursor *cursor;

    assert(bql_locked());
    if (!bridge || !job || job->kind != APPLE_VIRGL_CURSOR_STAGE_GLYPH ||
        job->width > UINT16_MAX || job->height > UINT16_MAX) {
        return;
    }
    con = bridge->gpu ? bridge->gpu->parent_obj.scanout[0].con : NULL;
    if (!con) {
        return;
    }
    cursor = cursor_alloc((uint16_t)job->width, (uint16_t)job->height);
    if (!cursor) {
        return;
    }
    cursor->hot_x = (int)job->hot_x;
    cursor->hot_y = (int)job->hot_y;
    if (apple_virgl_cursor_stage_fill_qemu_cursor(
            job, cursor->data, (size_t)cursor->width * cursor->height)) {
        dpy_cursor_define(con, cursor);
        apple_virgl_cursor_update(bridge, 0);
    }
    cursor_unref(cursor);
}

static void apple_virgl_cursor_bh(void *opaque)
{
    AppleVirglBridge *bridge = opaque;

    if (!bridge) {
        return;
    }
    assert(bql_locked());

    for (;;) {
        AppleVirglCursorStageJob *job;

        qemu_mutex_lock(&bridge->cursor_lock);
        job = apple_virgl_cursor_stage_take(&bridge->cursor_stage);
        if (!job) {
            bridge->cursor_bh_scheduled = false;
            qemu_mutex_unlock(&bridge->cursor_lock);
            return;
        }
        qemu_mutex_unlock(&bridge->cursor_lock);

        switch (job->kind) {
        case APPLE_VIRGL_CURSOR_STAGE_GLYPH:
            apple_virgl_cursor_apply_glyph(bridge, job);
            break;
        case APPLE_VIRGL_CURSOR_STAGE_SHOW:
            bridge->cursor_visible = job->visible;
            apple_virgl_cursor_update(bridge, job->display_id);
            break;
        case APPLE_VIRGL_CURSOR_STAGE_MOVE:
            apple_virgl_cursor_update(bridge, job->display_id);
            break;
        }
        apple_virgl_cursor_stage_job_free(job);
    }
}

static void apple_virgl_cursor_schedule_locked(AppleVirglBridge *bridge,
                                                bool queued)
{
    if (queued && !bridge->cursor_bh_scheduled && bridge->cursor_bh) {
        bridge->cursor_bh_scheduled = true;
        qemu_bh_schedule(bridge->cursor_bh);
    }
}

static void apple_virgl_cursor_glyph(void *opaque, const void *pixels,
                                     uint64_t mapped_length, uint64_t stride,
                                     uint32_t width, uint32_t height,
                                     uint32_t hot_x, uint32_t hot_y,
                                     uint32_t sum)
{
    AppleVirglBridge *bridge = opaque;
    bool queued;

    (void)sum;
    if (!bridge) {
        return;
    }
    qemu_mutex_lock(&bridge->cursor_lock);
    queued = apple_virgl_cursor_stage_enqueue_glyph(
        &bridge->cursor_stage, pixels, mapped_length, stride, width, height,
        hot_x, hot_y);
    apple_virgl_cursor_schedule_locked(bridge, queued);
    qemu_mutex_unlock(&bridge->cursor_lock);
}

static void apple_virgl_cursor_show(void *opaque, uint32_t display_id,
                                    int visible)
{
    AppleVirglBridge *bridge = opaque;
    bool queued;

    if (!bridge || display_id != 0) {
        return;
    }
    qemu_mutex_lock(&bridge->cursor_lock);
    queued = apple_virgl_cursor_stage_enqueue_show(&bridge->cursor_stage,
                                                   display_id,
                                                   visible != 0);
    apple_virgl_cursor_schedule_locked(bridge, queued);
    qemu_mutex_unlock(&bridge->cursor_lock);
}

static void apple_virgl_cursor_move(void *opaque, uint32_t display_id)
{
    AppleVirglBridge *bridge = opaque;
    bool queued;

    if (!bridge || display_id != 0) {
        return;
    }
    qemu_mutex_lock(&bridge->cursor_lock);
    queued = apple_virgl_cursor_stage_enqueue_move(&bridge->cursor_stage,
                                                   display_id);
    apple_virgl_cursor_schedule_locked(bridge, queued);
    qemu_mutex_unlock(&bridge->cursor_lock);
}

static void apple_virgl_cursor_quiesce(AppleVirglBridge *bridge,
                                       bool shutdown)
{
    if (!bridge) {
        return;
    }
    qemu_mutex_lock(&bridge->cursor_lock);
    apple_virgl_cursor_stage_begin_reset(&bridge->cursor_stage, shutdown);
    bridge->cursor_bh_scheduled = false;
    qemu_mutex_unlock(&bridge->cursor_lock);

    if (bridge->cursor_bh) {
        qemu_bh_cancel(bridge->cursor_bh);
    }
}

static void apple_virgl_cursor_resume(AppleVirglBridge *bridge)
{
    if (!bridge) {
        return;
    }
    qemu_mutex_lock(&bridge->cursor_lock);
    apple_virgl_cursor_stage_resume(&bridge->cursor_stage);
    qemu_mutex_unlock(&bridge->cursor_lock);
}

AppleVirglBridge *apple_virgl_bridge_new(VirtIOGPU *gpu)
{
    AppleVirglBridge *bridge;
    qmu_session *session;
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
    qemu_rec_mutex_init(&bridge->command_lock);
    qemu_mutex_init(&bridge->lifecycle_lock);
    qemu_mutex_init(&bridge->session_gate_lock);
    qemu_cond_init(&bridge->session_gate_idle);
    qemu_mutex_init(&bridge->frame_pump_lock);
    qemu_cond_init(&bridge->frame_pump_idle);
    apple_virgl_frame_pump_init(&bridge->frame_pump);
    qemu_mutex_init(&bridge->present_lock);
    apple_virgl_present_stage_init(&bridge->present_stage);
    qemu_mutex_init(&bridge->cursor_lock);
    apple_virgl_cursor_stage_init(&bridge->cursor_stage);
    bridge->cursor_visible = true;
    qemu_mutex_init(&bridge->completion_lock);
    g_queue_init(&bridge->completion_receivers);
    g_queue_init(&bridge->completion_stamps);
    bridge->contexts = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                              NULL, apple_virgl_context_free);
    bridge->resources = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                               NULL, apple_virgl_resource_free);
    bridge->new_frame_bh = qemu_bh_new_guarded(
        apple_virgl_new_frame_bh, bridge,
        &DEVICE(gpu)->mem_reentrancy_guard);
    bridge->present_bh = qemu_bh_new_guarded(
        apple_virgl_present_bh, bridge,
        &DEVICE(gpu)->mem_reentrancy_guard);
    bridge->cursor_bh = qemu_bh_new_guarded(
        apple_virgl_cursor_bh, bridge,
        &DEVICE(gpu)->mem_reentrancy_guard);
    bridge->completion_bh = qemu_bh_new_guarded(
        apple_virgl_completion_bh, bridge,
        &DEVICE(gpu)->mem_reentrancy_guard);
    bridge->frame_submit_pool = thread_pool_new();
    if (!thread_pool_set_max_threads(bridge->frame_submit_pool, 1)) {
        apple_virgl_bridge_free(bridge);
        return NULL;
    }

    callbacks.user_ctx = bridge;
    callbacks.map_gpa = apple_virgl_map_gpa;
    callbacks.unmap_gpa = apple_virgl_unmap_gpa;
    callbacks.read_memory = apple_virgl_read_memory;
    callbacks.write_memory = apple_virgl_write_memory;
    callbacks.read_gpu_memory = apple_virgl_read_gpu_memory;
    callbacks.completion_stamp = apple_virgl_completion_stamp;
    callbacks.cursor_glyph = apple_virgl_cursor_glyph;
    callbacks.cursor_show = apple_virgl_cursor_show;
    callbacks.cursor_move = apple_virgl_cursor_move;
    callbacks.new_frame_signal = apple_virgl_new_frame_signal;
    callbacks.render_frame_complete = apple_virgl_render_frame_complete;
    session = qmu_create(&config, &callbacks);
    if (!session) {
        apple_virgl_bridge_free(bridge);
        return NULL;
    }
    qemu_mutex_lock(&bridge->session_gate_lock);
    bridge->session = session;
    qemu_mutex_unlock(&bridge->session_gate_lock);
    qmu_set_direct_read_callback(session,
                                 apple_virgl_read_gpu_memory, bridge);
    qmu_set_direct_write_callback(session,
                                  apple_virgl_write_gpu_memory, bridge);
    return bridge;
}

void apple_virgl_bridge_free(AppleVirglBridge *bridge)
{
    qmu_session *session;

    if (!bridge) {
        return;
    }

    assert(bql_locked());
    qemu_mutex_lock(&bridge->lifecycle_lock);
    qemu_mutex_lock(&bridge->completion_lock);
    bridge->completion_shutdown = true;
    qemu_mutex_unlock(&bridge->completion_lock);
    session = apple_virgl_session_gate_close(bridge, true);
    apple_virgl_bridge_clear_completion_state(bridge, false);
    apple_virgl_cursor_quiesce(bridge, true);
    apple_virgl_presenter_quiesce(bridge, true);
    apple_virgl_frame_pump_quiesce(bridge, true);
    apple_virgl_session_gate_wait_idle(bridge);
    if (session) {
        /* invalidate may wait for an in-progress QMetal callback.  The gate
         * has already drained every producer and the pump is quiescent, so do
         * not hold a host mutex across that wait. */
        if (qmu_session_invalidate_display_delivery(session) != QMU_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "apple-virgl display delivery invalidation failed during teardown\n");
        }
        qmu_session_begin_shutdown(session);
        apple_virgl_session_gate_detach(bridge, session);
        qmu_destroy(session);
    }

    if (bridge->new_frame_bh) {
        qemu_bh_delete(bridge->new_frame_bh);
        bridge->new_frame_bh = NULL;
    }
    if (bridge->present_bh) {
        qemu_bh_delete(bridge->present_bh);
        bridge->present_bh = NULL;
    }
    if (bridge->cursor_bh) {
        qemu_bh_delete(bridge->cursor_bh);
        bridge->cursor_bh = NULL;
    }
    if (bridge->completion_bh) {
        qemu_bh_delete(bridge->completion_bh);
        bridge->completion_bh = NULL;
    }
    if (bridge->frame_submit_pool) {
        thread_pool_free(bridge->frame_submit_pool);
        bridge->frame_submit_pool = NULL;
    }
    if (bridge->contexts) {
        g_hash_table_destroy(bridge->contexts);
    }
    if (bridge->resources) {
        g_hash_table_destroy(bridge->resources);
    }

    qemu_mutex_destroy(&bridge->completion_lock);
    qemu_mutex_destroy(&bridge->cursor_lock);
    qemu_mutex_destroy(&bridge->present_lock);
    qemu_cond_destroy(&bridge->frame_pump_idle);
    qemu_mutex_destroy(&bridge->frame_pump_lock);
    qemu_cond_destroy(&bridge->session_gate_idle);
    qemu_mutex_destroy(&bridge->session_gate_lock);
    qemu_rec_mutex_destroy(&bridge->command_lock);
    qemu_mutex_destroy(&bridge->lock);
    qemu_mutex_unlock(&bridge->lifecycle_lock);
    qemu_mutex_destroy(&bridge->lifecycle_lock);
    g_free(bridge);
}

void apple_virgl_bridge_reset(AppleVirglBridge *bridge)
{
    GArray *task_ids;
    GHashTableIter iter;
    gpointer value;
    qmu_session *session;
    uint32_t index;

    if (!bridge) {
        return;
    }

    assert(bql_locked());
    qemu_mutex_lock(&bridge->lifecycle_lock);
    session = apple_virgl_session_gate_close(bridge, false);
    apple_virgl_bridge_clear_completion_state(bridge, false);
    apple_virgl_cursor_quiesce(bridge, false);
    apple_virgl_presenter_quiesce(bridge, false);
    apple_virgl_frame_pump_quiesce(bridge, false);
    apple_virgl_session_gate_wait_idle(bridge);
    /* As above, delivery invalidation may wait for an old callback; normal
     * command producers are already excluded by the closed/drained gate. */
    if (session && qmu_session_invalidate_display_delivery(session) != QMU_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-virgl display delivery invalidation failed during reset\n");
    }

    task_ids = g_array_new(false, false, sizeof(uint32_t));
    qemu_mutex_lock(&bridge->lock);
    g_hash_table_iter_init(&iter, bridge->contexts);
    while (g_hash_table_iter_next(&iter, NULL, &value)) {
        AppleVirglContextState *context = value;

        if (context->task_bound) {
            g_array_append_val(task_ids, context->task_id);
        }
    }
    g_hash_table_remove_all(bridge->contexts);
    g_hash_table_remove_all(bridge->resources);
    bridge->submit_count = 0;
    qemu_mutex_unlock(&bridge->lock);
    if (session) {
        for (index = 0; index < task_ids->len; ++index) {
            uint32_t task_id = g_array_index(task_ids, uint32_t, index);

            qmu_destroy_task(session, task_id);
        }
    }
    g_array_unref(task_ids);

    apple_virgl_bridge_frame_pump_resume(bridge);
    apple_virgl_presenter_resume(bridge);
    apple_virgl_cursor_resume(bridge);
    qemu_mutex_lock(&bridge->completion_lock);
    if (!bridge->completion_shutdown) {
        bridge->completion_resetting = false;
    }
    qemu_mutex_unlock(&bridge->completion_lock);
    apple_virgl_session_gate_resume(bridge);
    qemu_mutex_unlock(&bridge->lifecycle_lock);
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

    if (!bridge || context_id == 0 || name_length > 64) {
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
    qmu_session *session;
    bool task_bound;
    uint32_t task_id;

    if (!bridge || context_id == 0 ||
        !apple_virgl_session_lease_begin(bridge, &session)) {
        return -1;
    }
    qemu_rec_mutex_lock(&bridge->command_lock);
    qemu_mutex_lock(&bridge->lock);
    context = g_hash_table_lookup(bridge->contexts,
                                  GUINT_TO_POINTER(context_id));
    if (!context) {
        qemu_mutex_unlock(&bridge->lock);
        qemu_rec_mutex_unlock(&bridge->command_lock);
        apple_virgl_session_lease_end(bridge);
        return -1;
    }
    task_bound = context->task_bound;
    task_id = context->task_id;
    g_hash_table_remove(bridge->contexts, GUINT_TO_POINTER(context_id));
    qemu_mutex_unlock(&bridge->lock);
    if (task_bound) {
        qmu_destroy_task(session, task_id);
    }
    qemu_rec_mutex_unlock(&bridge->command_lock);
    apple_virgl_session_lease_end(bridge);
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
    qmu_session *session;
    const uint8_t *payload = (const uint8_t *)wire;
    uint32_t task_id_encoded = ldl_le_p(payload);
    uint32_t task_id = task_id_encoded >> 1;
    uint64_t vm_size = ldq_le_p(payload + 4);
    uint32_t task_root_pfn = ldl_le_p(payload + 12);
    bool is_kernel = (task_id_encoded & 1) != 0;

    if (!apple_virgl_session_lease_begin(bridge, &session)) {
        return -1;
    }
    qemu_rec_mutex_lock(&bridge->command_lock);
    qemu_mutex_lock(&bridge->lock);
    context = g_hash_table_lookup(bridge->contexts,
                                  GUINT_TO_POINTER(context_id));
    if (!context || context->task_bound ||
        apple_virgl_context_for_task_locked(bridge, task_id)) {
        qemu_mutex_unlock(&bridge->lock);
        qemu_rec_mutex_unlock(&bridge->command_lock);
        apple_virgl_session_lease_end(bridge);
        return -1;
    }
    context->task_id = task_id;
    context->task_bound = true;
    qemu_mutex_unlock(&bridge->lock);

    if (qmu_define_task(session, task_id, task_root_pfn, vm_size) !=
        QMU_OK) {
        qemu_mutex_lock(&bridge->lock);
        context = g_hash_table_lookup(bridge->contexts,
                                      GUINT_TO_POINTER(context_id));
        if (context && context->task_bound && context->task_id == task_id) {
            context->task_bound = false;
            context->task_id = 0;
        }
        qemu_mutex_unlock(&bridge->lock);
        qemu_rec_mutex_unlock(&bridge->command_lock);
        apple_virgl_session_lease_end(bridge);
        return -1;
    }

    qemu_rec_mutex_unlock(&bridge->command_lock);
    apple_virgl_session_lease_end(bridge);
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
    qmu_session *session = NULL;
    uint32_t index;
    uint32_t task_id;
    qmu_status status = QMU_ERROR;
    uint64_t submit;
    uint16_t opcode;
    uint32_t qmu_opcode = 0;
    uint32_t gpu_channel_id = 0;
    uint32_t payload_task_id;
    bool gpu_channel = false;
    bool display_channel = false;
    bool iosurface_backing_retire = false;
    bool bridge_locked = false;
    int result = -1;

    if (!bridge || context_id == 0 ||
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
    case APPLE_VIRGL_SUBMIT_DELETE_IOSURFACE_BACKING:
        /* This V5 envelope carries the reference type-6 lifetime semantic.
         * It is deliberately not synthesized as a numeric QMU FIFO opcode:
         * native 0x36/0x37/0x41 use other FIFO namespaces. */
        iosurface_backing_retire = true;
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

    /* Keep one normal-producer lease across registration and the resulting
     * command.  Reset cannot invalidate QMetal display delivery between the
     * two halves of a guest submission.  The serial mutation lock also covers
     * V5 backing retirement and the frame-pump producer edges. */
    if (!apple_virgl_session_lease_begin(bridge, &session)) {
        return -1;
    }
    qemu_rec_mutex_lock(&bridge->command_lock);
    qemu_mutex_lock(&bridge->lock);
    bridge_locked = true;
    context = g_hash_table_lookup(bridge->contexts,
                                  GUINT_TO_POINTER(context_id));
    if (!context || !context->task_bound) {
        goto out;
    }
    task_id = context->task_id;
    payload_task_id = iosurface_backing_retire ?
        ldl_le_p(view.payload + sizeof(uint32_t)) : ldl_le_p(view.payload);
    if (display_channel ? task_id != 0 : payload_task_id != task_id) {
        qemu_mutex_unlock(&bridge->lock);
        bridge_locked = false;
        error_report("apple-virgl submit task/transport mismatch: payload=%u transport=%u bound-task=%u display=%u retire=%u",
                     payload_task_id, context_id, task_id,
                     display_channel ? 1 : 0,
                     iosurface_backing_retire ? 1 : 0);
        goto out;
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
            goto out;
        }
        for (previous = 0; previous < new_mappings->len; ++previous) {
            AppleVirglMapping *other =
                &g_array_index(new_mappings, AppleVirglMapping, previous);

            if (mapping.apple_resource_id != 0 &&
                other->apple_resource_id == mapping.apple_resource_id) {
                goto out;
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
        goto out;
    }
    submit = ++bridge->submit_count;
    qemu_mutex_unlock(&bridge->lock);
    bridge_locked = false;
    g_array_unref(new_mappings);
    new_mappings = NULL;

    for (index = 0; index < view.mapping_count; ++index) {
        const AppleVirglSubmitMappingV1 *wire = &view.mappings[index];
        uint32_t apple_resource_id =
            le32_to_cpu(wire->apple_resource_id);

        if (apple_resource_id != 0) {
            qmu_register_buffer(session, apple_resource_id,
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
    if (iosurface_backing_retire) {
        status = qmu_retire_iosurface_backing(session, task_id,
                                              ldl_le_p(view.payload));
    } else if (display_channel) {
        status = qmu_submit_display_channel(session, qmu_opcode,
                                            view.payload,
                                            view.payload_bytes);
    } else if (gpu_channel) {
        status = qmu_submit_gpu_channel(session, gpu_channel_id,
                                        qmu_opcode, view.payload,
                                        view.payload_bytes);
    } else if (opcode == APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3 &&
               view.completion_channel_id != 0) {
        status = qmu_submit_root_exec_indirect3_with_completion(
            session, view.payload, view.payload_bytes,
            view.completion_channel_id, view.completion_stamp);
    } else {
        status = qmu_submit_root_fifo(session, qmu_opcode,
                                      view.payload, view.payload_bytes);
    }
    result = status == QMU_OK ? 0 : -1;

out:
    if (bridge_locked) {
        qemu_mutex_unlock(&bridge->lock);
    }
    if (new_mappings) {
        g_array_unref(new_mappings);
    }
    qemu_rec_mutex_unlock(&bridge->command_lock);
    apple_virgl_session_lease_end(bridge);
    return result;
}
