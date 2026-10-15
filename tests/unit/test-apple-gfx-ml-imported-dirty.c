/* One actual imported GPU -> original canonical RAM -> owned dirty-BH RETURN
 * component control. Not a guest/presentation/whole-B1 or performance proof.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <dlfcn.h>
#include <vulkan/vulkan.h>
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/rcu.h"
#include "qemu/thread.h"
#include "system/ram_addr.h"
#include "system/replay.h"
#include "system/system.h"
#include "hw/display/apple-gfx-ml.h"
#include "hw/pci/pci_host.h"
#include "hw/pci/msi.h"
#include "apple-gfx-ml-imported-dirty-bridge.h"

#define TYPE_IMPORT_HOST "test-agfx-import-host"
#define TYPE_IMPORT_DEVICE "test-agfx-import-device"
#define TYPE_IMPORT_RAM "test-agfx-import-ram"
#define RAM_GPA UINT64_C(0x20000000)
#define ALIAS_GPA UINT64_C(0x30000000)
#define GPU_VA UINT64_C(0x400000)
#define MAX_FENCES 16

typedef struct ImportRam {
    Object parent;
    MemoryRegion ram, alias;
} ImportRam;
typedef struct ObservedFence {
    VkDevice device;
    VkFence fence;
    unsigned phase;
    bool submitted, terminal, destroyed;
} ObservedFence;
typedef struct ImportControl {
    AppleGfxMLState *state;
    qmu_extended_callbacks original;
    ImportRam *owner; /* borrowed after AS detach and external unref */
    uint8_t *base, *mapped;
    uint64_t page, offset, bytes, total;
    AgfxImportLayout layout;
    AgfxImportIdentity identity;
    GMutex mutex;
    GCond cond;
    bool done, ram_finalized, state_finalized, detached;
    int result;
    unsigned maps, unmaps, allocations, memory_frees;
    unsigned dirty_entries, dirty_returns, schedules, leaf_returns;
    unsigned invoke_returns, child_releases, bh_returns;
    VkDeviceMemory imported_memory;
    VkDevice imported_device;
    ObservedFence fences[MAX_FENCES];
    unsigned fence_count, binds, submits;
    qmu_session_work *dirty_work; /* borrowed, never retained by test */
} ImportControl;
typedef struct ObservedBH {
    ImportControl *c;
    QEMUBHFunc *fn;
    void *opaque;
} ObservedBH;
typedef struct ObservedLeaf {
    ImportControl *c;
    void (*fn)(void *);
    void *opaque;
} ObservedLeaf;
typedef struct Worker {
    ImportControl *c;
    bool fill;
    QemuThread thread;
} Worker;

typedef struct UnmapControl {
    ImportControl *c;
    bool iosfc, missing, dirty_probe, done;
    int result;
    QemuThread thread;
} UnmapControl;

static ImportControl *active;
static PCIHostState *host;
static __thread unsigned native_phase;
static __thread ImportControl *dirty_forward, *current_bh;
static bool lease_partial, lease_first_map, lease_continue;
static unsigned unmap_bh_entries, unmap_bh_returns, lease_unmaps;
static qmu_direct_write_lease_acquire_fn real_lease_acquire;
static void *real_lease_context;
static void (*real_set_lease)(struct qmu_vulkan_ctx *,
    qmu_direct_write_lease_acquire_fn, qmu_direct_write_lease_write_fn,
    qmu_direct_write_lease_release_fn, void *);
static PFN_vkCreateFence real_create;
static PFN_vkDestroyFence real_destroy;
static PFN_vkQueueBindSparse real_bind;
static PFN_vkQueueSubmit real_submit;
static PFN_vkWaitForFences real_wait;
static PFN_vkAllocateMemory real_allocate;
static PFN_vkFreeMemory real_free;
int (*qemu_main)(void);

/* Transparently capture the real source-owned lease endpoint installed by
 * qmu_create. No callback replacement, copied TU or private layout. */
void qmu_vk_set_direct_write_lease_callbacks(struct qmu_vulkan_ctx *,
    qmu_direct_write_lease_acquire_fn, qmu_direct_write_lease_write_fn,
    qmu_direct_write_lease_release_fn, void *);
void qmu_vk_set_direct_write_lease_callbacks(struct qmu_vulkan_ctx *vk,
    qmu_direct_write_lease_acquire_fn acquire,
    qmu_direct_write_lease_write_fn write,
    qmu_direct_write_lease_release_fn release, void *ctx)
{
    g_assert_nonnull(real_set_lease);
    real_set_lease(vk, acquire, write, release, ctx);
    if (active) {
        g_assert_null(real_lease_acquire);
        real_lease_acquire = acquire;
        real_lease_context = ctx;
    }
}

qmu_session *__real_qmu_create_extended(const qmu_extended_config *,
                                       const qmu_extended_callbacks *);
qmu_session *__wrap_qmu_create_extended(const qmu_extended_config *,
                                       const qmu_extended_callbacks *);
void __real_aio_bh_schedule_oneshot_full(AioContext *, QEMUBHFunc *, void *,
                                        const char *);
void __wrap_aio_bh_schedule_oneshot_full(AioContext *, QEMUBHFunc *, void *,
                                        const char *);
void __real_qmu_session_invoke_host_work(qmu_session_work *, void (*)(void *), void *);
void __wrap_qmu_session_invoke_host_work(qmu_session_work *, void (*)(void *), void *);
void __real_qmu_session_release_host_work(qmu_session_work *);
void __wrap_qmu_session_release_host_work(qmu_session_work *);

static void drain_rcu(void)
{
    bql_unlock();
    drain_call_rcu();
    bql_lock();
}

static void changed(ImportControl *c)
{
    g_cond_broadcast(&c->cond);
    aio_wait_kick();
}

static int fence_index(ImportControl *c, VkFence fence)
{
    for (unsigned i = c->fence_count; i; --i) {
        if (c->fences[i - 1].fence == fence && !c->fences[i - 1].destroyed)
            return i - 1;
    }
    return -1;
}

VKAPI_ATTR VkResult VKAPI_CALL vkCreateFence(VkDevice dev,
    const VkFenceCreateInfo *info, const VkAllocationCallbacks *alloc, VkFence *f)
{
    VkResult result = real_create(dev, info, alloc, f);
    if (native_phase && result == VK_SUCCESS) {
        ImportControl *c = active;
        g_mutex_lock(&c->mutex);
        g_assert_cmpuint(c->fence_count, <, MAX_FENCES);
        c->fences[c->fence_count++] = (ObservedFence){
            .device = dev, .fence = *f, .phase = native_phase};
        g_mutex_unlock(&c->mutex);
    }
    return result;
}

VKAPI_ATTR void VKAPI_CALL vkDestroyFence(VkDevice dev, VkFence f,
    const VkAllocationCallbacks *alloc)
{
    real_destroy(dev, f, alloc);
    if (active) {
        g_mutex_lock(&active->mutex);
        int i = fence_index(active, f);
        if (i >= 0) {
            g_assert_true(active->fences[i].device == dev);
            g_assert_true(active->fences[i].terminal);
            active->fences[i].destroyed = true;
        }
        g_mutex_unlock(&active->mutex);
    }
}

VKAPI_ATTR VkResult VKAPI_CALL vkQueueBindSparse(VkQueue queue, uint32_t count,
    const VkBindSparseInfo *info, VkFence fence)
{
    VkResult result = real_bind(queue, count, info, fence);
    if (native_phase) {
        ImportControl *c = active;
        g_assert_cmpuint(native_phase, ==, 1);
        g_assert_cmpint(result, ==, VK_SUCCESS);
        g_assert_cmpuint(count, ==, 1);
        g_assert_cmpuint(info->bufferBindCount, ==, 1);
        g_mutex_lock(&c->mutex);
        int i = fence_index(c, fence);
        g_assert_cmpint(i, >=, 0);
        g_assert_false(c->fences[i].submitted);
        c->fences[i].submitted = true;
        ++c->binds;
        g_mutex_unlock(&c->mutex);
    }
    return result;
}

VKAPI_ATTR VkResult VKAPI_CALL vkQueueSubmit(VkQueue queue, uint32_t count,
    const VkSubmitInfo *info, VkFence fence)
{
    VkResult result = real_submit(queue, count, info, fence);
    if (native_phase) {
        ImportControl *c = active;
        g_assert_cmpuint(native_phase, ==, 2);
        g_assert_cmpint(result, ==, VK_SUCCESS);
        g_assert_cmpuint(count, ==, 1);
        g_assert_cmpuint(info->commandBufferCount, ==, 1);
        g_mutex_lock(&c->mutex);
        int i = fence_index(c, fence);
        g_assert_cmpint(i, >=, 0);
        g_assert_false(c->fences[i].submitted);
        c->fences[i].submitted = true;
        ++c->submits;
        g_mutex_unlock(&c->mutex);
    }
    return result;
}

VKAPI_ATTR VkResult VKAPI_CALL vkWaitForFences(VkDevice dev, uint32_t count,
    const VkFence *fences, VkBool32 all, uint64_t timeout)
{
    VkResult result = real_wait(dev, count, fences, all, timeout);
    if (active && result == VK_SUCCESS) {
        g_mutex_lock(&active->mutex);
        for (unsigned n = 0; n < count; ++n) {
            int i = fence_index(active, fences[n]);
            if (i >= 0) {
                g_assert_true(active->fences[i].device == dev);
                g_assert_true(active->fences[i].submitted);
                g_assert_true(count == 1 || all);
                active->fences[i].terminal = true;
            }
        }
        g_mutex_unlock(&active->mutex);
    }
    return result;
}

VKAPI_ATTR VkResult VKAPI_CALL vkAllocateMemory(VkDevice dev,
    const VkMemoryAllocateInfo *info, const VkAllocationCallbacks *alloc,
    VkDeviceMemory *memory)
{
    VkResult result = real_allocate(dev, info, alloc, memory);
    if (native_phase == 1) {
        const VkBaseInStructure *next = info->pNext;
        for (; next; next = next->pNext) {
            if (next->sType != VK_STRUCTURE_TYPE_IMPORT_MEMORY_HOST_POINTER_INFO_EXT)
                continue;
            const VkImportMemoryHostPointerInfoEXT *import = (const void *)next;
            ImportControl *c = active;
            g_assert_cmpint(result, ==, VK_SUCCESS);
            g_assert_true(import->pHostPointer == c->mapped);
            g_assert_cmpuint(info->allocationSize, ==, c->bytes);
            g_assert_cmpuint(info->memoryTypeIndex, <, c->layout.memory_count);
            uint32_t needed = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                              VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
            g_assert_cmpuint(c->layout.memory_flags[info->memoryTypeIndex] & needed,
                             ==, needed);
            g_assert_cmpuint(c->allocations++, ==, 0);
            c->imported_memory = *memory;
            c->imported_device = dev;
        }
    }
    return result;
}

VKAPI_ATTR void VKAPI_CALL vkFreeMemory(VkDevice dev, VkDeviceMemory memory,
    const VkAllocationCallbacks *alloc)
{
    real_free(dev, memory, alloc);
    if (active && memory != VK_NULL_HANDLE && memory == active->imported_memory) {
        g_assert_true(dev == active->imported_device);
        g_assert_cmpuint(active->memory_frees++, ==, 0);
    }
}

static void *observe_map(void *ctx, uint64_t gpa, size_t bytes, int writable)
{
    ImportControl *c = active;
    void *ptr = c->original.map_gpa(ctx, gpa, bytes, writable);
    if (lease_partial && gpa == RAM_GPA) {
        g_assert_true(ptr == c->base);
        g_assert_cmpuint(bytes, ==, 0x1000);
        g_assert_cmpint(writable, ==, 1);
        g_assert_null(qmu_session_current_host_work(c->state->qmu_dev));
        g_mutex_lock(&c->mutex);
        g_assert_false(lease_first_map);
        lease_first_map = true;
        changed(c);
        const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
        while (!lease_continue) {
            g_assert_true(g_cond_wait_until(&c->cond, &c->mutex, deadline));
        }
        g_mutex_unlock(&c->mutex);
    }
    if (gpa == ALIAS_GPA) {
        g_assert_true(ctx == c->state);
        g_assert_cmpuint(c->maps++, ==, 0);
        g_assert_cmpuint(bytes, ==, c->bytes);
        g_assert_cmpint(writable, ==, 0);
        g_assert_true(ptr == c->mapped);
        ram_addr_t offset;
        g_assert_true(memory_region_from_host(ptr, &offset) == &c->owner->ram);
        g_assert_cmpuint(offset, ==, c->offset);
    }
    return ptr;
}

static void observe_unmap(void *ctx, void *ptr, size_t bytes, int dirty)
{
    ImportControl *c = active;
    if (lease_partial) {
        g_assert_true(ctx == c->state);
        g_assert_true(ptr == c->base);
        g_assert_cmpuint(bytes, ==, 0x1000);
        g_assert_cmpint(dirty, ==, 0);
        g_assert_null(qmu_session_current_host_work(c->state->qmu_dev));
        g_assert_true(c->detached);
        g_assert_false(c->ram_finalized);
        g_assert_cmpuint(OBJECT(c->owner)->ref, ==, 1);
        g_assert_cmpuint(lease_unmaps++, ==, 0);
        c->original.unmap_gpa(ctx, ptr, bytes, dirty);
        g_assert_true(c->ram_finalized);
        return;
    }
    if (ptr == c->mapped) {
        g_assert_true(ctx == c->state);
        g_assert_true(c->detached);
        g_assert_false(c->ram_finalized);
        g_assert_cmpuint(bytes, ==, c->bytes);
        g_assert_cmpint(dirty, ==, 1);
        g_assert_cmpuint(c->memory_frees, ==, 1);
        g_assert_cmpuint(c->bh_returns, ==, 1);
        g_assert_cmpuint(c->unmaps++, ==, 0);
        g_assert_cmpuint(OBJECT(c->owner)->ref, ==, 1);
        c->original.unmap_gpa(ctx, ptr, bytes, dirty);
        g_assert_true(c->ram_finalized);
    } else {
        c->original.unmap_gpa(ctx, ptr, bytes, dirty);
    }
}

static int observe_dirty(void *ctx, void *ptr, size_t bytes)
{
    ImportControl *c = active;
    g_assert_true(ctx == c->state);
    g_assert_true(ptr == c->mapped);
    g_assert_cmpuint(bytes, ==, c->bytes);
    g_assert_cmpuint(native_phase, ==, 2);
    g_assert_cmpuint(c->dirty_entries++, ==, 0);
    c->dirty_work = qmu_session_current_host_work(c->state->qmu_dev);
    g_assert_nonnull(c->dirty_work);
    g_assert_null(dirty_forward);
    dirty_forward = c;
    int result = c->original.mark_mapped_memory_dirty(ctx, ptr, bytes);
    dirty_forward = NULL;
    g_assert_cmpint(result, ==, 0);
    ++c->dirty_returns;
    return result;
}

qmu_session *__wrap_qmu_create_extended(const qmu_extended_config *config,
    const qmu_extended_callbacks *callbacks)
{
    ImportControl *c = active;
    g_assert_true(callbacks->user_ctx == c->state);
    g_assert_cmpuint(config->abi_version, ==, QMU_EXTENDED_ABI_VERSION);
    g_assert_cmpuint(config->struct_size, ==, sizeof(*config));
    g_assert_cmpuint(callbacks->struct_size, ==, sizeof(*callbacks));
    c->original = *callbacks;
    g_assert_nonnull(c->original.mark_mapped_memory_dirty);
    qmu_extended_callbacks observed = *callbacks;
    observed.map_gpa = observe_map;
    observed.unmap_gpa = observe_unmap;
    observed.mark_mapped_memory_dirty = observe_dirty;
    return __real_qmu_create_extended(config, &observed);
}

static void verify_original_output(ImportControl *c)
{
    ram_addr_t offset;
    g_assert_true(bql_locked());
    g_assert_false(c->ram_finalized);
    g_assert_cmpuint(OBJECT(c->owner)->ref, ==, 1);
    g_assert_true(memory_region_from_host(c->mapped, &offset) == &c->owner->ram);
    g_assert_cmpuint(offset, ==, c->offset);
    g_assert_cmpuint(c->unmaps, ==, 0);
    g_assert_cmpuint(c->binds, ==, 1);
    g_assert_cmpuint(c->submits, ==, 1);
    g_assert_cmpuint(c->fence_count, ==, 2);
    for (unsigned i = 0; i < c->fence_count; ++i)
        g_assert_true(c->fences[i].terminal);
    for (uint64_t i = 0; i < c->total; ++i) {
        uint8_t expected = 0x47;
        if (i >= c->offset && i < c->offset + 64) {
            static const uint8_t word[] = {0x78, 0x56, 0x34, 0x12};
            expected = word[(i - c->offset) % 4];
        }
        g_assert_cmpuint(c->base[i], ==, expected);
    }
    DirtyBitmapSnapshot *snap = memory_region_snapshot_and_clear_dirty(
        &c->owner->ram, 0, c->total, DIRTY_MEMORY_VGA);
    for (uint64_t p = 0; p < c->total; p += c->page) {
        bool expected = p >= c->offset && p < c->offset + c->bytes;
        g_assert_cmpint(memory_region_snapshot_get_dirty(&c->owner->ram, snap,
                         p, c->page), ==, expected);
    }
    g_free(snap);
}

static void observe_leaf(void *opaque)
{
    ObservedLeaf *leaf = opaque;
    ImportControl *c = leaf->c;
    g_assert_true(qmu_session_current_host_work(c->state->qmu_dev) == c->dirty_work);
    leaf->fn(leaf->opaque); /* original stack job is NEVER read after RETURN */
    g_assert_cmpuint(c->leaf_returns++, ==, 0);
    verify_original_output(c);
    const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    g_mutex_lock(&c->mutex);
    while (!c->done) {
        g_assert_true(g_cond_wait_until(&c->cond, &c->mutex, deadline));
    }
    g_mutex_unlock(&c->mutex);
    g_assert_cmpint(c->result, ==, 0);
    g_assert_cmpuint(c->dirty_returns, ==, 1);
    g_assert_cmpuint(c->child_releases, ==, 0);
    g_assert_false(c->ram_finalized);
    g_assert_cmpuint(OBJECT(c->owner)->ref, ==, 1);
    g_assert_true(qmu_session_current_host_work(c->state->qmu_dev) == c->dirty_work);
}

void __wrap_qmu_session_invoke_host_work(qmu_session_work *work,
    void (*fn)(void *), void *opaque)
{
    ImportControl *c = current_bh;
    if (!c) {
        __real_qmu_session_invoke_host_work(work, fn, opaque);
        return;
    }
    g_assert_true(work == c->dirty_work);
    g_assert_cmpuint(c->invoke_returns, ==, 0);
    ObservedLeaf leaf = {c, fn, opaque};
    __real_qmu_session_invoke_host_work(work, observe_leaf, &leaf);
    g_assert_null(qmu_session_current_host_work(c->state->qmu_dev));
    ++c->invoke_returns;
}

void __wrap_qmu_session_release_host_work(qmu_session_work *work)
{
    ImportControl *c = current_bh;
    if (c) {
        g_assert_true(work == c->dirty_work);
        g_assert_cmpuint(c->invoke_returns, ==, 1);
        g_assert_cmpuint(c->leaf_returns, ==, 1);
        g_assert_cmpuint(c->child_releases, ==, 0);
    }
    __real_qmu_session_release_host_work(work);
    if (c) ++c->child_releases; /* no access to released token */
}

static void observe_bh(void *opaque)
{
    ObservedBH *bh = opaque;
    ImportControl *c = bh->c;
    g_assert_null(current_bh);
    g_assert_null(qmu_session_current_host_work(c->state->qmu_dev));
    current_bh = c;
    bh->fn(bh->opaque); /* exact original agfx_host_bh envelope */
    current_bh = NULL;
    g_assert_cmpuint(c->child_releases, ==, 1);
    g_assert_cmpuint(c->bh_returns++, ==, 0);
    g_assert_null(qmu_session_current_host_work(c->state->qmu_dev));
    g_free(bh);
}

static void observe_unmap_bh(void *opaque)
{
    ObservedBH *bh = opaque;

    g_assert_true(bql_locked());
    ++unmap_bh_entries;
    bh->fn(bh->opaque); /* actual unmap trampoline and original leaf */
    ++unmap_bh_returns; /* signal inside fn is NOT this RETURN */
    g_free(bh);
}

void __wrap_aio_bh_schedule_oneshot_full(AioContext *ctx, QEMUBHFunc *fn,
    void *opaque, const char *name)
{
    if (active && !strcmp(name, "agfx_unmap_bql_bh")) {
        g_assert_true(ctx == qemu_get_aio_context());
        ObservedBH *bh = g_new0(ObservedBH, 1);
        *bh = (ObservedBH){active, fn, opaque};
        __real_aio_bh_schedule_oneshot_full(ctx, observe_unmap_bh, bh, name);
        return;
    }
    if (!dirty_forward) {
        __real_aio_bh_schedule_oneshot_full(ctx, fn, opaque, name);
        return;
    }
    ImportControl *c = dirty_forward;
    g_assert_true(ctx == qemu_get_aio_context());
    g_assert_cmpstr(name, ==, "agfx_host_bh");
    g_assert_cmpuint(c->schedules++, ==, 0);
    ObservedBH *bh = g_new0(ObservedBH, 1);
    *bh = (ObservedBH){c, fn, opaque};
    __real_aio_bh_schedule_oneshot_full(ctx, observe_bh, bh, name);
}

static void prepare_in_scope(void *opaque)
{
    ImportControl *c = opaque;
    c->result = agfx_import_prepare(c->state->qmu_dev, GPU_VA, ALIAS_GPA,
                                    c->bytes, &c->identity);
}

static void *worker_run(void *opaque)
{
    Worker *worker = opaque;
    ImportControl *c = worker->c;
    rcu_register_thread();
    native_phase = worker->fill ? 2 : 1;
    g_assert_null(qmu_session_current_host_work(c->state->qmu_dev));
    if (worker->fill) {
        c->result = agfx_import_fill(c->state->qmu_dev, c->identity.id);
    } else {
        qmu_session_work *setup = qmu_session_acquire_host_work(c->state->qmu_dev);
        g_assert_nonnull(setup);
        __real_qmu_session_invoke_host_work(setup, prepare_in_scope, c);
        __real_qmu_session_release_host_work(setup);
    }
    g_assert_null(qmu_session_current_host_work(c->state->qmu_dev));
    native_phase = 0;
    g_mutex_lock(&c->mutex);
    c->done = true; /* PUBLIC producer and its own scope have returned */
    changed(c);
    g_mutex_unlock(&c->mutex);
    rcu_unregister_thread();
    return NULL;
}

static void run_worker(ImportControl *c, bool fill)
{
    Worker worker = {.c = c, .fill = fill};
    g_mutex_lock(&c->mutex);
    c->done = false;
    g_mutex_unlock(&c->mutex);
    qemu_thread_create(&worker.thread, "import-dirty", worker_run, &worker,
                        QEMU_THREAD_JOINABLE);
    const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    for (;;) {
        g_mutex_lock(&c->mutex);
        bool done = c->done;
        g_mutex_unlock(&c->mutex);
        if (done && (!fill || c->bh_returns)) break;
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&worker.thread);
    g_assert_cmpint(c->result, ==, 0);
}

static void ram_finalize(Object *object)
{
    g_assert_true(object == OBJECT(active->owner));
    g_assert_false(active->ram_finalized);
    active->ram_finalized = true;
}

static void state_finalize(Object *object)
{
    g_assert_true(object == OBJECT(active->state));
    g_assert_false(active->state_finalized);
    active->state_finalized = true;
}

static const TypeInfo host_type = {
    .name = TYPE_IMPORT_HOST, .parent = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(PCIHostState),
};
static const TypeInfo device_type = {
    .name = TYPE_IMPORT_DEVICE, .parent = TYPE_APPLE_GFX_ML,
    .instance_size = sizeof(AppleGfxMLState), .instance_finalize = state_finalize,
};
static const TypeInfo ram_type = {
    .name = TYPE_IMPORT_RAM, .parent = TYPE_OBJECT,
    .instance_size = sizeof(ImportRam), .instance_finalize = ram_finalize,
};

static uint64_t gcd(uint64_t a, uint64_t b)
{
    while (b) { uint64_t next = a % b; a = b; b = next; }
    return a;
}

static void control_ram_new(ImportControl *c)
{
    c->owner = (ImportRam *)object_new(TYPE_IMPORT_RAM);
    c->ram_finalized = false;
    c->detached = false;
    c->total = 0x1000;
    g_assert_true(memory_region_init_ram_nomigrate(&c->owner->ram,
        OBJECT(c->owner), "unpin-control-ram", c->total, &error_abort));
    c->base = memory_region_get_ram_ptr(&c->owner->ram);
    memset(c->base, 0x47, c->total);
    memory_region_add_subregion(get_system_memory(), RAM_GPA, &c->owner->ram);
    memory_region_set_log(&c->owner->ram, true, DIRTY_MEMORY_VGA);
    drain_rcu();
    memory_region_reset_dirty(&c->owner->ram, 0, c->total, DIRTY_MEMORY_VGA);
}

static void control_ram_detach(ImportControl *c)
{
    memory_region_del_subregion(get_system_memory(), &c->owner->ram);
    drain_rcu();
    object_unref(OBJECT(c->owner));
    c->detached = true;
    g_assert_false(c->ram_finalized);
    g_assert_cmpuint(OBJECT(c->owner)->ref, ==, 1);
}

static void control_unmap_call(UnmapControl *control)
{
    ImportControl *c = control->c;
    void *ptr = control->missing ? NULL : c->base;

    g_assert_null(qmu_session_current_host_work(c->state->qmu_dev));
    if (control->iosfc) {
        /* len is deliberately zero: the original mapper ignores it. */
        control->result = c->original.iosfc_unmap_memory(c->state, ptr, 0);
    } else {
        c->original.unmap_gpa(c->state, ptr,
            control->dirty_probe ? c->total : 0, control->dirty_probe);
        control->result = 0;
    }
}

static void *control_unmap_worker(void *opaque)
{
    UnmapControl *control = opaque;
    rcu_register_thread();
    g_assert_false(bql_locked());
    control_unmap_call(control);
    g_mutex_lock(&control->c->mutex);
    control->done = true;
    changed(control->c);
    g_mutex_unlock(&control->c->mutex);
    rcu_unregister_thread();
    return NULL;
}

static void control_unmap_run(UnmapControl *control, bool worker)
{
    unsigned before = unmap_bh_returns;
    if (worker) {
        qemu_thread_create(&control->thread, "unpin-control",
            control_unmap_worker, control, QEMU_THREAD_JOINABLE);
        const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
        for (;;) {
            g_mutex_lock(&control->c->mutex);
            bool done = control->done;
            g_mutex_unlock(&control->c->mutex);
            if (done && unmap_bh_returns == before + 1) break;
            g_assert_cmpint(g_get_monotonic_time(), <, deadline);
            aio_poll(qemu_get_aio_context(), false);
            g_thread_yield();
        }
        qemu_thread_join(&control->thread);
    } else {
        g_assert_true(bql_locked());
        control_unmap_call(control);
    }
    g_assert_cmpuint(unmap_bh_returns, ==, before + worker);
    g_assert_cmpuint(unmap_bh_entries, ==, unmap_bh_returns);
    g_assert_cmpint(control->result, ==,
                   control->missing && control->iosfc ? -1 : 0);
}

static void *partial_lease_worker(void *opaque)
{
    UnmapControl *control = opaque;
    ImportControl *c = control->c;
    uint64_t token = UINT64_MAX;

    rcu_register_thread();
    g_assert_false(bql_locked());
    g_assert_null(qmu_session_current_host_work(c->state->qmu_dev));
    g_assert_true(real_lease_context == c->state->qmu_dev);
    control->result = real_lease_acquire(real_lease_context, 42,
        UINT64_C(0x900000), 0x2000, &token);
    g_assert_cmpint(control->result, ==, -1);
    g_assert_cmpuint(token, ==, 0);
    g_assert_null(qmu_session_current_host_work(c->state->qmu_dev));
    g_assert_true(c->ram_finalized);
    g_mutex_lock(&c->mutex);
    control->done = true;
    changed(c);
    g_mutex_unlock(&c->mutex);
    rcu_unregister_thread();
    return NULL;
}

static void test_unmap_callbacks(ImportControl *c)
{
    unsigned cases = 0;
    for (unsigned worker = 0; worker < 2; ++worker) {
        for (unsigned iosfc = 0; iosfc < 2; ++iosfc) {
            UnmapControl control = {.c = c, .iosfc = iosfc};
            control_ram_new(c);
            void *ptr = iosfc ? c->original.iosfc_map_memory(
                c->state, RAM_GPA, c->total, 0) :
                c->original.map_gpa(c->state, RAM_GPA, c->total, 1);
            g_assert_true(ptr == c->base);
            control_ram_detach(c);
            control_unmap_run(&control, worker);
            g_assert_true(c->ram_finalized);
            drain_rcu();
            ++cases;

            control = (UnmapControl){.c = c, .iosfc = iosfc, .missing = true};
            control_unmap_run(&control, worker);
            ++cases;
        }
        /* Non-last-pin dirty-argument control, deliberately not a lifetime
         * oracle. The four cases above have NO outside owner after detach. */
        control_ram_new(c);
        g_assert_true(c->original.map_gpa(c->state, RAM_GPA, c->total, 1) == c->base);
        UnmapControl dirty = {.c = c, .dirty_probe = true};
        control_unmap_run(&dirty, worker);
        DirtyBitmapSnapshot *snap = memory_region_snapshot_and_clear_dirty(
            &c->owner->ram, 0, c->total, DIRTY_MEMORY_VGA);
        g_assert_true(memory_region_snapshot_get_dirty(
            &c->owner->ram, snap, 0, c->total));
        g_free(snap);
        memory_region_del_subregion(get_system_memory(), &c->owner->ram);
        drain_rcu();
        object_unref(OBJECT(c->owner));
        g_assert_true(c->ram_finalized);
        drain_rcu();
        ++cases;
    }

    /* Real source-owned acquisition fails on its second, genuinely unmapped
     * GPA page. Its unique_ptr destructor retains SessionWork but has no TLS
     * Scope. Do not fabricate a lease/token or substitute a custom endpoint. */
    control_ram_new(c);
    g_assert_nonnull(real_lease_acquire);
    g_assert_cmpint(qmu_define_task(c->state->qmu_dev, 42, 0, 0), ==, QMU_OK);
    g_assert_cmpint(qmu_map_memory(c->state->qmu_dev, 42, UINT64_C(0x900000),
        RAM_GPA, 0x2000), ==, QMU_OK);
    lease_partial = true;
    unsigned before = unmap_bh_returns;
    UnmapControl partial = {.c = c};
    qemu_thread_create(&partial.thread, "partial-lease", partial_lease_worker,
        &partial, QEMU_THREAD_JOINABLE);
    const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    bool detached = false;
    for (;;) {
        g_mutex_lock(&c->mutex);
        bool ready = lease_first_map;
        bool done = partial.done;
        g_mutex_unlock(&c->mutex);
        if (ready && !detached) {
            control_ram_detach(c);
            detached = true;
            g_mutex_lock(&c->mutex);
            lease_continue = true;
            changed(c);
            g_mutex_unlock(&c->mutex);
        }
        if (done && unmap_bh_returns == before + 1) break;
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&partial.thread);
    g_assert_true(detached);
    g_assert_cmpuint(lease_unmaps, ==, 1);
    g_assert_cmpuint(unmap_bh_entries, ==, unmap_bh_returns);
    g_assert_true(c->ram_finalized);
    lease_partial = false;
    drain_rcu();
    ++cases;
    g_assert_cmpuint(cases, ==, 11);
    g_assert_cmpuint(unmap_bh_returns, ==, 6);
    g_test_message("UNPIN_CALLBACKS_PASS cases=11 direct=5 worker=5 tokenless_owned_partial=1 bh_return=6 no_extra_lastpin_owner=1");
    c->owner = NULL;
    c->base = NULL;
    c->total = 0;
    c->ram_finalized = false;
    c->detached = false;
}

static void test_imported_dirty(void)
{
    ImportControl c = {0};
    active = &c;
    g_mutex_init(&c.mutex);
    g_cond_init(&c.cond);
    c.state = APPLE_GFX_ML(object_new(TYPE_IMPORT_DEVICE));
    DEVICE(c.state)->hotplugged = false;
    c.state->vram_size_mb = 1;
    c.state->display_width = 16;
    c.state->display_height = 12;
    g_assert_true(qdev_realize(DEVICE(c.state), BUS(host->bus), &error_abort));
    g_assert_nonnull(c.state->qmu_dev);
    g_assert_nonnull(c.state->con);
    g_assert_nonnull(c.original.iosfc_map_memory);
    g_assert_nonnull(c.original.iosfc_unmap_memory);
    test_unmap_callbacks(&c);
    g_assert_cmpint(agfx_import_layout(c.state->qmu_dev, &c.layout), ==, 0);
    c.page = qemu_real_host_page_size();
    uint64_t alignment = c.layout.sparse / gcd(c.layout.sparse,
                                              c.layout.host_alignment);
    g_assert_cmpuint(alignment, <=, UINT64_MAX / c.layout.host_alignment);
    alignment *= c.layout.host_alignment;
    g_assert_cmpuint(alignment % c.page, ==, 0);
    c.bytes = MAX(UINT64_C(4) * 1024 * 1024, alignment);
    g_assert_cmpuint(c.bytes, <=, UINT64_MAX - alignment);
    c.bytes = ((c.bytes + alignment - 1) / alignment) * alignment;
    g_assert_cmpuint(c.bytes, <, UINT64_C(1) << 30);
    c.total = c.bytes + alignment + 2 * c.page;
    c.owner = (ImportRam *)object_new(TYPE_IMPORT_RAM);
    g_assert_true(memory_region_init_ram_nomigrate(&c.owner->ram,
        OBJECT(c.owner), "import-canonical-ram", c.total, &error_abort));
    c.base = memory_region_get_ram_ptr(&c.owner->ram);
    uintptr_t start = (uintptr_t)c.base + c.page;
    g_assert_cmpuint(start, <=, UINTPTR_MAX - alignment);
    c.mapped = (void *)(((start + alignment - 1) / alignment) * alignment);
    c.offset = c.mapped - c.base;
    g_assert_cmpuint(c.offset, >=, c.page);
    g_assert_cmpuint(c.offset + c.bytes + c.page, <=, c.total);
    memset(c.base, 0x47, c.total);
    memory_region_add_subregion(get_system_memory(), RAM_GPA, &c.owner->ram);
    memory_region_init_alias(&c.owner->alias, OBJECT(c.owner), "import-alias",
                             &c.owner->ram, c.offset, c.bytes);
    memory_region_add_subregion(get_system_memory(), ALIAS_GPA, &c.owner->alias);
    memory_region_set_log(&c.owner->ram, true, DIRTY_MEMORY_VGA);
    drain_rcu();
    uint32_t baseline_refs = OBJECT(c.owner)->ref;
    memory_region_reset_dirty(&c.owner->ram, 0, c.total, DIRTY_MEMORY_VGA);
    run_worker(&c, false);
    g_assert_cmpuint(c.maps, ==, 1);
    g_assert_cmpuint(c.unmaps, ==, 0);
    g_assert_cmpuint(c.allocations, ==, 1);
    g_assert_cmpuint(OBJECT(c.owner)->ref, ==, baseline_refs + 1);
    memory_region_del_subregion(get_system_memory(), &c.owner->alias);
    memory_region_del_subregion(get_system_memory(), &c.owner->ram);
    drain_rcu();
    object_unref(OBJECT(c.owner));
    c.detached = true;
    g_assert_false(c.ram_finalized);
    g_assert_cmpuint(OBJECT(c.owner)->ref, ==, 1);
    run_worker(&c, true);
    g_assert_cmpuint(c.dirty_entries, ==, 1);
    g_assert_cmpuint(c.dirty_returns, ==, 1);
    g_assert_cmpuint(c.schedules, ==, 1);
    g_assert_cmpuint(c.leaf_returns, ==, 1);
    g_assert_cmpuint(c.invoke_returns, ==, 1);
    g_assert_cmpuint(c.child_releases, ==, 1);
    g_assert_cmpuint(c.bh_returns, ==, 1);
    qdev_unrealize(DEVICE(c.state)); /* actual pumped barrier and original unmap */
    g_assert_null(c.state->qmu_dev);
    g_assert_null(c.state->render_pool);
    g_assert_null(c.state->con);
    g_assert_false(msi_present(PCI_DEVICE(c.state)));
    g_assert_cmpuint(c.unmaps, ==, 1);
    g_assert_true(c.ram_finalized);
    g_assert_true(agfx_import_generation_expired());
    g_assert_cmpuint(unmap_bh_entries, ==, 7);
    g_assert_cmpuint(unmap_bh_returns, ==, 7);
    for (unsigned i = 0; i < c.fence_count; ++i)
        g_assert_true(c.fences[i].destroyed);
    object_unparent(OBJECT(c.state));
    object_unref(OBJECT(c.state));
    drain_rcu();
    g_assert_true(c.state_finalized);
    g_test_message("CANONICAL_MR_PASS maps=%u unmaps=%u bind=%u submit=%u dirty=%u leaf_return=%u producer_return=1 invoke_return=%u child_release=%u bh_return=%u bytes=%" PRIu64 " offset=%" PRIu64 " sparse=%" PRIu64 " host_align=%" PRIu64 " buffer=%" PRIx64 " native=%" PRIx64 " generation=%" PRIx64 " runtime=%" PRIu64 "; component-only NOT presentation/whole-B1/main",
        c.maps, c.unmaps, c.binds, c.submits, c.dirty_returns, c.leaf_returns,
        c.invoke_returns, c.child_releases, c.bh_returns, c.bytes, c.offset,
        c.layout.sparse, c.layout.host_alignment, c.identity.object,
        c.identity.native_buffer, c.identity.generation, c.identity.runtime_generation);
    active = NULL;
    g_cond_clear(&c.cond);
    g_mutex_clear(&c.mutex);
}

static void resolved(const char *label, void *symbol, const char *expected)
{
    Dl_info info = {0};
    g_assert_nonnull(symbol);
    g_assert_nonnull(expected);
    g_assert_true(dladdr(symbol, &info) != 0);
    char *actual = realpath(info.dli_fname, NULL);
    char *wanted = realpath(expected, NULL);
    g_assert_nonnull(actual);
    g_assert_nonnull(wanted);
    g_assert_cmpstr(actual, ==, wanted);
    g_test_message("RESOLUTION %s path=%s", label, actual);
    free(actual);
    free(wanted);
}

int main(int argc, char **argv)
{
    char options[][16] = {"-machine", "none", "-accel", "tcg", "-nodefaults",
                          "-display", "none", "-S"};
    char *qemu_argv[] = {argv[0], options[0], options[1], options[2], options[3],
                        options[4], options[5], options[6], options[7], NULL};
    g_test_init(&argc, &argv, NULL);
    const char *dso = g_getenv("AGFX_CREDIT_QMETAL_DSO");
    const char *loader = g_getenv("AGFX_CREDIT_VULKAN_DSO");
    const char *air = g_getenv("AGFX_CREDIT_AIR_DSO");
#define LOAD(member, name, type) do { \
    member = (type)dlsym(RTLD_NEXT, name); resolved(name, (void *)member, loader); \
} while (0)
    LOAD(real_create, "vkCreateFence", PFN_vkCreateFence);
    LOAD(real_destroy, "vkDestroyFence", PFN_vkDestroyFence);
    LOAD(real_bind, "vkQueueBindSparse", PFN_vkQueueBindSparse);
    LOAD(real_submit, "vkQueueSubmit", PFN_vkQueueSubmit);
    LOAD(real_wait, "vkWaitForFences", PFN_vkWaitForFences);
    LOAD(real_allocate, "vkAllocateMemory", PFN_vkAllocateMemory);
    LOAD(real_free, "vkFreeMemory", PFN_vkFreeMemory);
#undef LOAD
    resolved("normal-factory", (void *)&__real_qmu_create_extended, dso);
    resolved("normal-invoke", (void *)&__real_qmu_session_invoke_host_work, dso);
    resolved("normal-release", (void *)&__real_qmu_session_release_host_work, dso);
    resolved("normal-vulkan-context", (void *)&qmu_session_get_vulkan, dso);
    real_set_lease = (void *)dlsym(RTLD_NEXT,
        "qmu_vk_set_direct_write_lease_callbacks");
    resolved("normal-lease-registration", (void *)real_set_lease, dso);
    resolved("interposed-lease-registration",
        (void *)&qmu_vk_set_direct_write_lease_callbacks, argv[0]);
    resolved("matched-AIR", dlsym(RTLD_DEFAULT, "air2spirv_context_create"), air);
    resolved("interposed-submit", (void *)&vkQueueSubmit, argv[0]);
    qemu_init(ARRAY_SIZE(qemu_argv) - 1, qemu_argv);
    type_register_static(&host_type);
    type_register_static(&device_type);
    type_register_static(&ram_type);
    host = PCI_HOST_BRIDGE(object_new(TYPE_IMPORT_HOST));
    host->bus = pci_root_bus_new(DEVICE(host), "import.pcie", get_system_memory(),
                                 get_system_io(), 0, TYPE_PCIE_BUS);
    g_assert_true(qbus_realize(BUS(host->bus), &error_abort));
    bool saved_msi = msi_nonbroken;
    msi_nonbroken = true;
    g_test_add_func("/agfx/imported-dirty/original-mr-owned-return", test_imported_dirty);
    int status = g_test_run();
    msi_nonbroken = saved_msi;
    pci_root_bus_cleanup(host->bus);
    object_unref(OBJECT(host));
    drain_rcu();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
