/* Connected component control, not a VM or a GPU-busy/visual witness.
 * Actual QEMU factory/NFS/BH/ThreadPool/reset/transport and normal QMetal DSO.
 * Vulkan interposition forwards the REAL submit, wait, arguments and result.
 * It delays observation of one accepted fence, never invents a terminal.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <dlfcn.h>
#include <vulkan/vulkan.h>
#include "qapi/error.h"
#include "qemu/aio-wait.h"
#include "qemu/main-loop.h"
#include "qemu/rcu.h"
#include "block/thread-pool.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/replay.h"
#include "system/system.h"
#include "hw/core/resettable.h"
#include "hw/display/apple-gfx-ml.h"
#include "hw/pci/pci_host.h"
#include "hw/pci/msi.h"
#include "ui/console.h"
#include "pvg/pvg_fifo.h"
#include "qmu/pvg_regs.h"
#include "apple-gfx-ml-credit-test-bridge.h"

#define TYPE_CREDIT_HOST "test-agfx-credit-host"
#define TYPE_CREDIT_DEVICE "test-agfx-credit-device"
#define TYPE_CREDIT_RAM "test-agfx-credit-ram"
#define CREDIT_GPA UINT64_C(0x10000000)
#define CREDIT_WIDTH 16
#define CREDIT_HEIGHT 12
#define MAX_FENCES 16

typedef struct CreditRam { Object parent; MemoryRegion ram; } CreditRam;
typedef struct FenceObservation {
    VkDevice device;
    VkFence fence;
    VkQueue queue;
    VkCommandBuffer command;
    bool submitted, terminal, destroyed;
    unsigned waits;
} FenceObservation;
typedef struct CreditControl {
    AppleGfxMLState *state;
    qmu_extended_callbacks original;
    GMutex mutex;
    GCond cond;
    FenceObservation fences[MAX_FENCES];
    unsigned fence_count, selected;
    unsigned render_jobs, render_retired, positive, nonpositive;
    unsigned scheduled, original_returns, callbacks, callback_returns, irq_calls;
    unsigned pending_at_callback[MAX_FENCES];
    bool pixels_at_callback[MAX_FENCES];
    void *shared_maps[2];
    unsigned map_count, unmap_count;
    bool arm, selected_valid, wait_entered, release_wait;
    bool destroying, finalized;
    CreditRam *ram;
    uint32_t texture;
} CreditControl;
typedef struct OriginalCall {
    CreditControl *control;
    void (*fn)(void *);
    void *opaque;
} OriginalCall;
typedef struct RenderCall {
    CreditControl *control;
    ThreadPoolFunc *fn;
    void *opaque;
    GDestroyNotify destroy;
} RenderCall;
typedef struct TransportCall {
    CreditControl *control;
    uint32_t pfn;
    bool invalidate;
    bool done;
    qmu_status result;
    QemuThread thread;
} TransportCall;

static CreditControl *active;
static __thread CreditControl *render_caller;
static PCIHostState *host;
static void (*parent_exit)(PCIDevice *);
static PFN_vkCreateFence real_create_fence;
static PFN_vkDestroyFence real_destroy_fence;
static PFN_vkQueueSubmit real_submit;
static PFN_vkWaitForFences real_wait;
int (*qemu_main)(void);

qmu_session *__real_qmu_create_extended(const qmu_extended_config *,
                                       const qmu_extended_callbacks *);
qmu_session *__wrap_qmu_create_extended(const qmu_extended_config *,
                                       const qmu_extended_callbacks *);
int __real_qmu_vk_submit_captured_display_frame(void *);
int __wrap_qmu_vk_submit_captured_display_frame(void *);
void __real_thread_pool_submit_immediate(ThreadPool *, ThreadPoolFunc *, void *,
                                         GDestroyNotify);
void __wrap_thread_pool_submit_immediate(ThreadPool *, ThreadPoolFunc *, void *,
                                         GDestroyNotify);

static void changed(CreditControl *c)
{
    g_cond_broadcast(&c->cond);
    aio_wait_kick();
}

static int fence_index(CreditControl *c, VkFence fence)
{
    /* A driver may reuse a destroyed handle. Its newest creation is the
     * applicable identity; a numeric fence value alone is not generation. */
    for (unsigned i = c->fence_count; i; --i) {
        if (c->fences[i - 1].fence == fence && !c->fences[i - 1].destroyed) {
            return i - 1;
        }
    }
    return -1;
}

VKAPI_ATTR VkResult VKAPI_CALL vkCreateFence(
    VkDevice device, const VkFenceCreateInfo *info,
    const VkAllocationCallbacks *allocator, VkFence *fence)
{
    VkResult result = real_create_fence(device, info, allocator, fence);
    CreditControl *c = render_caller;
    if (c && result == VK_SUCCESS) {
        g_mutex_lock(&c->mutex);
        g_assert_cmpuint(c->fence_count, <, MAX_FENCES);
        c->fences[c->fence_count++] = (FenceObservation) {
            .device = device, .fence = *fence,
        };
        changed(c);
        g_mutex_unlock(&c->mutex);
    }
    return result;
}

VKAPI_ATTR void VKAPI_CALL vkDestroyFence(
    VkDevice device, VkFence fence, const VkAllocationCallbacks *allocator)
{
    real_destroy_fence(device, fence, allocator);
    CreditControl *c = active;
    if (c && fence != VK_NULL_HANDLE) {
        g_mutex_lock(&c->mutex);
        int index = fence_index(c, fence);
        if (index >= 0) {
            g_assert_true(c->fences[index].device == device);
            g_assert_true(c->fences[index].terminal);
            c->fences[index].destroyed = true;
            changed(c);
        }
        g_mutex_unlock(&c->mutex);
    }
}

VKAPI_ATTR VkResult VKAPI_CALL vkQueueSubmit(
    VkQueue queue, uint32_t count, const VkSubmitInfo *info, VkFence fence)
{
    VkResult result = real_submit(queue, count, info, fence);
    CreditControl *c = render_caller;
    if (c) {
        g_assert_cmpint(result, ==, VK_SUCCESS);
        g_assert_cmpuint(count, ==, 1);
        g_assert_cmpuint(info->commandBufferCount, ==, 1);
        g_mutex_lock(&c->mutex);
        int index = fence_index(c, fence);
        g_assert_cmpint(index, >=, 0);
        FenceObservation *observed = &c->fences[index];
        g_assert_false(observed->submitted);
        observed->submitted = true;
        observed->queue = queue;
        observed->command = info->pCommandBuffers[0];
        changed(c);
        g_mutex_unlock(&c->mutex);
    }
    return result;
}

VKAPI_ATTR VkResult VKAPI_CALL vkWaitForFences(
    VkDevice device, uint32_t count, const VkFence *fences,
    VkBool32 all, uint64_t timeout)
{
    CreditControl *c = active;
    int index = -1;
    if (c && count == 1) {
        g_mutex_lock(&c->mutex);
        index = fence_index(c, fences[0]);
        if (index >= 0) {
            FenceObservation *observed = &c->fences[index];
            g_assert_true(observed->device == device);
            g_assert_true(observed->submitted);
            g_assert_cmpuint(all, ==, VK_TRUE);
            g_assert_cmpuint(timeout, ==, UINT64_MAX);
            /* Display framebuffer construction may submit a synchronous
             * black-init utility from the render caller before the actual
             * accepted Display record. Forward that utility immediately.
             * In this display-only input domain, only the native completion
             * worker observes a render-origin fence without render TLS. */
            if (c->arm && !render_caller) {
                c->selected = index;
                c->selected_valid = true;
                c->arm = false;
            }
            if (c->selected_valid && c->selected == index && !c->release_wait) {
                const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
                c->wait_entered = true;
                changed(c);
                while (!c->release_wait) {
                    g_assert_true(g_cond_wait_until(&c->cond, &c->mutex, deadline));
                }
            }
        }
        g_mutex_unlock(&c->mutex);
    }
    /* This is the only producer of the observed result. No injected API
     * outcome, synthetic signalled fence, or claim the GPU stayed busy. */
    VkResult result = real_wait(device, count, fences, all, timeout);
    if (c && index >= 0) {
        g_assert_cmpint(result, ==, VK_SUCCESS);
        g_mutex_lock(&c->mutex);
        ++c->fences[index].waits;
        c->fences[index].terminal = true;
        changed(c);
        g_mutex_unlock(&c->mutex);
    }
    return result;
}

static void original_return(void *opaque)
{
    OriginalCall *call = opaque;
    CreditControl *c = call->control;
    g_assert_true(bql_locked());
    call->fn(call->opaque); /* Original typed fn alone retires original ctx. */
    g_mutex_lock(&c->mutex);
    ++c->original_returns;
    changed(c);
    g_mutex_unlock(&c->mutex);
    g_free(call);
}

static void observe_schedule(void *opaque, void (*fn)(void *), void *context)
{
    CreditControl *c = active;
    g_assert_true(opaque == c->state);
    OriginalCall *call = g_new(OriginalCall, 1);
    *call = (OriginalCall) {c, fn, context};
    g_mutex_lock(&c->mutex);
    ++c->scheduled;
    g_mutex_unlock(&c->mutex);
    c->original.schedule_display_completion(opaque, original_return, call);
}

static void observe_completion(void *opaque,
                                const qmu_render_frame_completion *completion)
{
    CreditControl *c = active;
    g_assert_true(opaque == c->state);
    g_mutex_lock(&c->mutex);
    unsigned index = c->callbacks++;
    g_assert_cmpuint(index, <, MAX_FENCES);
    c->pending_at_callback[index] =
        __atomic_load_n(&c->state->pending_frames, __ATOMIC_SEQ_CST);
    c->pixels_at_callback[index] = completion->frame_expected != 0;
    if (!completion->frame_expected) {
        g_assert_null(completion->pixels);
        g_assert_cmpuint(completion->width, ==, 0);
        g_assert_cmpuint(completion->height, ==, 0);
    }
    changed(c);
    g_mutex_unlock(&c->mutex);
    /* Never called independently and never blocked during reset. */
    c->original.render_frame_complete(opaque, completion);
    g_mutex_lock(&c->mutex);
    ++c->callback_returns;
    changed(c);
    g_mutex_unlock(&c->mutex);
}

static void observe_irq(void *opaque, uint32_t vector)
{
    CreditControl *c = active;
    g_assert_true(opaque == c->state);
    g_mutex_lock(&c->mutex);
    ++c->irq_calls;
    g_mutex_unlock(&c->mutex);
    c->original.raise_irq(opaque, vector);
}

static void *observe_map(void *opaque, uint64_t gpa, size_t size, int writable)
{
    CreditControl *c = active;
    void *mapped = c->original.map_gpa(opaque, gpa, size, writable);
    if (gpa == CREDIT_GPA || gpa == CREDIT_GPA + 0x1000) {
        unsigned index = (gpa - CREDIT_GPA) / 0x1000;
        g_assert_nonnull(c->ram);
        g_assert_nonnull(mapped);
        g_assert_cmpuint(size, ==, 0x1000);
        g_assert_cmpint(writable, ==, 1);
        g_assert_true(mapped == (uint8_t *)memory_region_get_ram_ptr(
            &c->ram->ram) + index * 0x1000);
        g_mutex_lock(&c->mutex);
        g_assert_null(c->shared_maps[index]);
        c->shared_maps[index] = mapped;
        ++c->map_count;
        g_mutex_unlock(&c->mutex);
    }
    return mapped;
}

static void observe_unmap(void *opaque, void *mapped, size_t size, int dirty)
{
    CreditControl *c = active;
    g_mutex_lock(&c->mutex);
    for (unsigned i = 0; i < 2; ++i) {
        if (mapped && c->shared_maps[i] == mapped) {
            g_assert_cmpuint(size, ==, 0x1000);
            g_assert_cmpint(dirty, ==, 0);
            c->shared_maps[i] = NULL;
            ++c->unmap_count;
            break;
        }
    }
    g_mutex_unlock(&c->mutex);
    c->original.unmap_gpa(opaque, mapped, size, dirty);
}

qmu_session *__wrap_qmu_create_extended(const qmu_extended_config *config,
                                       const qmu_extended_callbacks *callbacks)
{
    CreditControl *c = active;
    g_assert_true(callbacks->user_ctx == c->state);
    g_assert_cmpuint(config->abi_version, ==, QMU_EXTENDED_ABI_VERSION);
    g_assert_cmpuint(config->struct_size, ==, sizeof(*config));
    g_assert_cmpuint(callbacks->struct_size, ==, sizeof(*callbacks));
    g_assert_null(callbacks->eeee_task_allocate);
    g_assert_nonnull(callbacks->schedule_display_completion);
    g_assert_nonnull(callbacks->render_frame_complete);
    c->original = *callbacks;
    qmu_extended_callbacks observed = *callbacks;
    observed.schedule_display_completion = observe_schedule;
    observed.render_frame_complete = observe_completion;
    observed.raise_irq = observe_irq;
    observed.map_gpa = observe_map;
    observed.unmap_gpa = observe_unmap;
    /* Real factory/admission/empty-tail guard, unchanged State context. */
    return __real_qmu_create_extended(config, &observed);
}

int __wrap_qmu_vk_submit_captured_display_frame(void *context)
{
    CreditControl *c = render_caller;
    g_assert_nonnull(c);
    g_assert_false(bql_locked());
    int result = __real_qmu_vk_submit_captured_display_frame(context);
    g_mutex_lock(&c->mutex);
    if (result > 0) {
        ++c->positive;
    } else {
        ++c->nonpositive;
    }
    changed(c);
    g_mutex_unlock(&c->mutex);
    return result;
}

static int render_original(void *opaque)
{
    RenderCall *call = opaque;
    g_assert_null(render_caller);
    render_caller = call->control;
    int result = call->fn(call->opaque);
    render_caller = NULL;
    return result;
}

static void render_original_destroy(void *opaque)
{
    RenderCall *call = opaque;
    CreditControl *c = call->control;
    if (call->destroy) {
        call->destroy(call->opaque);
    }
    g_mutex_lock(&c->mutex);
    ++c->render_retired;
    changed(c);
    g_mutex_unlock(&c->mutex);
    g_free(call);
}

void __wrap_thread_pool_submit_immediate(ThreadPool *pool, ThreadPoolFunc *fn,
                                         void *opaque, GDestroyNotify destroy)
{
    CreditControl *c = active;
    g_assert_true(pool == c->state->render_pool);
    RenderCall *call = g_new(RenderCall, 1);
    *call = (RenderCall) {c, fn, opaque, destroy};
    g_mutex_lock(&c->mutex);
    ++c->render_jobs;
    g_mutex_unlock(&c->mutex);
    __real_thread_pool_submit_immediate(pool, render_original, call,
                                         render_original_destroy);
}

static bool idle(CreditControl *c)
{
    bool result;
    unsigned irq_calls;
    g_mutex_lock(&c->mutex);
    result = c->render_jobs == c->render_retired &&
        c->scheduled == c->original_returns &&
        c->positive == c->callbacks && c->callbacks == c->callback_returns;
    for (unsigned i = 0; i < c->fence_count; ++i) {
        result &= c->fences[i].submitted && c->fences[i].terminal;
    }
    irq_calls = c->irq_calls;
    g_mutex_unlock(&c->mutex);
    return result && __atomic_load_n(&c->state->pending_frames,
                                     __ATOMIC_SEQ_CST) == 0 &&
           c->state->irq_count == irq_calls;
}

static bool first_accepted(CreditControl *c)
{
    bool result;
    g_mutex_lock(&c->mutex);
    result = c->selected_valid && c->wait_entered &&
             c->positive == 1 && c->render_retired == 1;
    g_mutex_unlock(&c->mutex);
    return result;
}

static void pump_until(CreditControl *c, bool (*condition)(CreditControl *))
{
    gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!condition(c)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
}

static void *transport_caller(void *opaque)
{
    TransportCall *call = opaque;
    CreditControl *c = call->control;
    qemu_mutex_lock(&c->state->session_mutex);
    if (call->invalidate) {
        call->result = qmu_session_invalidate_display_delivery(c->state->qmu_dev);
    } else {
        const uint32_t payload[2] = {0, call->pfn};
        call->result = qmu_submit_gpu_channel(c->state->qmu_dev, 5,
            PVG_DISP_CMD_SET_SHARED_STATE, payload, sizeof(payload));
    }
    qemu_mutex_unlock(&c->state->session_mutex);
    qatomic_set(&call->done, true);
    aio_wait_kick();
    return NULL;
}

static void start_transport(TransportCall *call, CreditControl *c, uint32_t pfn)
{
    *call = (TransportCall) {.control = c, .pfn = pfn, .invalidate = !pfn};
    qemu_thread_create(&call->thread, "credit-transport", transport_caller,
                       call, QEMU_THREAD_JOINABLE);
}

static void finish_transport(TransportCall *call)
{
    gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!qatomic_read(&call->done)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&call->thread);
    g_assert_cmpint(call->result, ==, QMU_OK);
}

static void release_observation(CreditControl *c)
{
    g_mutex_lock(&c->mutex);
    g_assert_true(c->wait_entered);
    c->release_wait = true;
    changed(c);
    g_mutex_unlock(&c->mutex);
}

static void drain_rcu(void)
{
    bql_unlock();
    drain_call_rcu();
    bql_lock();
}

static void fixture_exit(PCIDevice *device)
{
    CreditControl *c = active;
    g_assert_true(device == PCI_DEVICE(c->state));
    g_assert_true(c->destroying);
    g_assert_true(idle(c));
    parent_exit(device); /* Unchanged actual agfx_exit, not a fake stop. */
    g_assert_null(c->state->qmu_dev);
    g_assert_null(c->state->render_pool);
    g_assert_true(idle(c));
    g_assert_cmpuint(c->map_count, ==, c->unmap_count);
    g_assert_null(c->shared_maps[0]);
    g_assert_null(c->shared_maps[1]);
    for (unsigned i = 0; i < c->fence_count; ++i) {
        g_assert_true(c->fences[i].destroyed);
    }
    /* Test-only owners, after real producer stop and while PCI config/MR
     * remain alive. Product exit and parent PCI unregister are unchanged. */
    graphic_console_close(c->state->con);
    vmstate_unregister_ram(&c->state->host_vram, DEVICE(c->state));
    g_assert_true(msi_present(device));
    msi_uninit(device);
}

static void fixture_class_init(ObjectClass *klass, const void *data)
{
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);
    parent_exit = pc->exit;
    g_assert_nonnull(parent_exit);
    pc->exit = fixture_exit;
}

static void fixture_finalize(Object *object)
{
    g_assert_true(object == OBJECT(active->state));
    g_assert_false(active->finalized);
    active->finalized = true;
}

static const TypeInfo host_type = {
    .name = TYPE_CREDIT_HOST, .parent = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(PCIHostState),
};
static const TypeInfo device_type = {
    .name = TYPE_CREDIT_DEVICE, .parent = TYPE_APPLE_GFX_ML,
    .instance_size = sizeof(AppleGfxMLState),
    .class_init = fixture_class_init, .instance_finalize = fixture_finalize,
};
static const TypeInfo ram_type = {
    .name = TYPE_CREDIT_RAM, .parent = TYPE_OBJECT,
    .instance_size = sizeof(CreditRam),
};

static void create(CreditControl *c, enum AgfxCreditFamily family)
{
    *c = (CreditControl) {0};
    g_mutex_init(&c->mutex);
    g_cond_init(&c->cond);
    active = c;
    c->state = APPLE_GFX_ML(object_new(TYPE_CREDIT_DEVICE));
    DEVICE(c->state)->hotplugged = false;
    c->state->vram_size_mb = 1;
    c->state->display_width = CREDIT_WIDTH;
    c->state->display_height = CREDIT_HEIGHT;
    g_assert_true(qdev_realize(DEVICE(c->state), BUS(host->bus), &error_abort));
    g_assert_nonnull(c->state->qmu_dev); /* No alternate factory/skip. */
    g_assert_nonnull(c->state->con);
    g_assert_cmpint(c->state->pending_frames, ==, 0);
    c->ram = (CreditRam *)object_new(TYPE_CREDIT_RAM);
    g_assert_true(memory_region_init_ram_nomigrate(&c->ram->ram,
        OBJECT(c->ram), "credit-shared-state", 0x2000, &error_abort));
    memory_region_add_subregion(get_system_memory(), CREDIT_GPA, &c->ram->ram);
    memset(memory_region_get_ram_ptr(&c->ram->ram), 0, 0x2000);
    if (family == AGFX_CREDIT_TEXTURE) {
        c->texture = agfx_credit_prepare_texture(c->state->qmu_dev,
                                                  CREDIT_WIDTH, CREDIT_HEIGHT);
        g_assert_cmpuint(c->texture, !=, 0);
    }
}

static void dispose(CreditControl *c)
{
    TransportCall control;
    pump_until(c, idle);
    /* This fixture has no guest/vCPU, mapped IOSFC timer, EXEC, transaction,
     * stamp or further producer input. Last real invalidate retires its
     * bounded display records/source callbacks, not a global B1 stop API. */
    start_transport(&control, c, 0);
    finish_transport(&control);
    pump_until(c, idle);
    c->destroying = true;
    qdev_unrealize(DEVICE(c->state));
    object_unparent(OBJECT(c->state));
    object_unref(OBJECT(c->state));
    drain_rcu();
    drain_rcu();
    g_assert_true(c->finalized);
    memory_region_del_subregion(get_system_memory(), &c->ram->ram);
    object_unref(OBJECT(c->ram));
    drain_rcu();
    g_test_message("backend_transfers=%u render_native_submits=%u callbacks=%u local_nonpositive=%u; all render-native fences actually terminal; finite fixture cleanup only, NOT global B1 quiescence",
                     c->positive, c->fence_count, c->callbacks, c->nonpositive);
    active = NULL;
    g_cond_clear(&c->cond);
    g_mutex_clear(&c->mutex);
}

static void publish(CreditControl *c, enum AgfxCreditFamily family, uint32_t seed)
{
    g_assert_cmpint(agfx_credit_publish(c->state->qmu_dev, family, c->texture,
                      CREDIT_WIDTH, CREDIT_HEIGHT, seed), ==, 0);
}

static void first(CreditControl *c, enum AgfxCreditFamily family)
{
    g_mutex_lock(&c->mutex);
    c->arm = true;
    g_mutex_unlock(&c->mutex);
    publish(c, family, 0xff102030);
    pump_until(c, first_accepted);
    g_assert_cmpint(__atomic_load_n(&c->state->pending_frames,
                                    __ATOMIC_SEQ_CST), ==, 1);
    g_assert_cmpuint(c->callbacks, ==, 0);
}

static void test_normal(void)
{
    CreditControl c;
    create(&c, AGFX_CREDIT_IOSFC);
    first(&c, AGFX_CREDIT_IOSFC);
    release_observation(&c);
    pump_until(&c, idle);
    g_assert_cmpuint(c.positive, ==, 1);
    g_assert_cmpuint(c.nonpositive, ==, 0);
    g_assert_true(c.pixels_at_callback[0]);
    dispose(&c);
}

static bool two_credits(CreditControl *c)
{
    return __atomic_load_n(&c->state->pending_frames, __ATOMIC_SEQ_CST) == 2;
}

static bool second_completed(CreditControl *c)
{
    bool completed;
    g_mutex_lock(&c->mutex);
    completed = c->positive == 2 && c->callbacks == 2;
    g_mutex_unlock(&c->mutex);
    /* Publishing only queues NFS; its child BH has not admitted credit yet. */
    return completed && idle(c);
}

static void test_reset_two(void)
{
    CreditControl c;
    create(&c, AGFX_CREDIT_IOSFC);
    first(&c, AGFX_CREDIT_IOSFC);
    publish(&c, AGFX_CREDIT_IOSFC, 0xff405060);
    pump_until(&c, two_credits);
    g_assert_cmpuint(c.render_jobs, ==, 1);
    device_cold_reset(DEVICE(c.state)); /* Never assign private pending. */
    g_assert_cmpint(__atomic_load_n(&c.state->pending_frames,
                                    __ATOMIC_SEQ_CST), ==, 2);
    release_observation(&c);
    pump_until(&c, idle);
    g_assert_cmpuint(c.positive, ==, 2);
    g_assert_cmpuint(c.nonpositive, ==, 0);
    g_assert_cmpuint(c.pending_at_callback[0], ==, 2);
    g_assert_cmpuint(c.pending_at_callback[1], ==, 1);
    dispose(&c);
}

static void test_port(enum AgfxCreditFamily family)
{
    CreditControl c;
    TransportCall call;
    create(&c, family);
    start_transport(&call, &c, CREDIT_GPA >> 12);
    finish_transport(&call);
    g_assert_cmpuint(c.map_count, ==, 1);
    g_assert_cmpuint(c.unmap_count, ==, 0);
    first(&c, family);
    const uint64_t epoch = agfx_credit_delivery_epoch(c.state->qmu_dev);
    start_transport(&call, &c, (CREDIT_GPA >> 12) + 1);
    const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (agfx_credit_delivery_epoch(c.state->qmu_dev) == epoch) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    g_assert_false(qatomic_read(&call.done));
    release_observation(&c);
    finish_transport(&call); /* Main AIO pumps real RAM/DMA/IRQ BHs. */
    g_assert_cmpuint(c.map_count, ==, 2);
    g_assert_cmpuint(c.unmap_count, ==, 1);
    pump_until(&c, idle);
    g_assert_cmpuint(c.callbacks, ==, 1);
    g_assert_false(c.pixels_at_callback[0]);
    g_assert_cmpuint(c.pending_at_callback[0], ==, 1);
    publish(&c, family, 0xff708090);
    pump_until(&c, second_completed);
    g_assert_cmpuint(c.positive, ==, 2);
    g_assert_cmpuint(c.callbacks, ==, 2);
    g_assert_true(c.pixels_at_callback[1]);
    g_assert_cmpuint(c.nonpositive, ==, 0);
    dispose(&c);
}

static void test_port_iosfc(void) { test_port(AGFX_CREDIT_IOSFC); }
static void test_port_texture(void) { test_port(AGFX_CREDIT_TEXTURE); }

static char *provider_path(void *symbol)
{
    Dl_info info = {0};
    g_assert_nonnull(symbol);
    g_assert_true(dladdr(symbol, &info) != 0);
    char *actual_path = realpath(info.dli_fname, NULL);
    g_assert_nonnull(actual_path);
    return actual_path;
}

static void matching_path(const char *actual_path, const char *expected)
{
    g_assert_nonnull(expected);
    g_assert_cmpint(expected[0], !=, '\0');
    char *expected_path = realpath(expected, NULL);
    g_assert_nonnull(expected_path);
    g_assert_cmpstr(actual_path, ==, expected_path);
    free(expected_path);
}

static void resolved(const char *label, void *symbol, const char *expected)
{
    Dl_info info = {0};
    char *actual_path = provider_path(symbol);
    matching_path(actual_path, expected);
    g_assert_true(dladdr(symbol, &info) != 0);
    g_test_message("RESOLUTION %s path=%s symbol=%s", label, actual_path,
                     info.dli_sname ? info.dli_sname : "(local)");
    free(actual_path);
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
    /* Ordinary Meson runs use independent, uninterposed provider anchors.
     * Controlled runs additionally require the complete explicit pin set;
     * never infer an expected provider from the endpoint being checked. */
    char *dso_provider = provider_path((void *)&qmu_session_get_vulkan);
    char *loader_provider = provider_path(
        dlsym(RTLD_NEXT, "vkGetInstanceProcAddr"));
    char *air_provider = provider_path(
        dlsym(RTLD_DEFAULT, "air2spirv_version"));
    if (dso || loader || air) {
        matching_path(dso_provider, dso);
        matching_path(loader_provider, loader);
        matching_path(air_provider, air);
    }
    real_create_fence = (PFN_vkCreateFence)dlsym(RTLD_NEXT, "vkCreateFence");
    real_destroy_fence = (PFN_vkDestroyFence)dlsym(RTLD_NEXT, "vkDestroyFence");
    real_submit = (PFN_vkQueueSubmit)dlsym(RTLD_NEXT, "vkQueueSubmit");
    real_wait = (PFN_vkWaitForFences)dlsym(RTLD_NEXT, "vkWaitForFences");
    resolved("real-create", (void *)real_create_fence, loader_provider);
    resolved("real-destroy", (void *)real_destroy_fence, loader_provider);
    resolved("real-submit", (void *)real_submit, loader_provider);
    resolved("real-wait", (void *)real_wait, loader_provider);
    resolved("normal-capture-submit",
             (void *)&__real_qmu_vk_submit_captured_display_frame, dso_provider);
    resolved("real-factory", (void *)&__real_qmu_create_extended, dso_provider);
    resolved("matched-AIR", dlsym(RTLD_DEFAULT, "air2spirv_context_create"),
             air_provider);
    resolved("interposed-wait", (void *)&vkWaitForFences, argv[0]);
    free(dso_provider);
    free(loader_provider);
    free(air_provider);
    qemu_init(ARRAY_SIZE(qemu_argv) - 1, qemu_argv);
    type_register_static(&host_type);
    type_register_static(&device_type);
    type_register_static(&ram_type);
    host = PCI_HOST_BRIDGE(object_new(TYPE_CREDIT_HOST));
    host->bus = pci_root_bus_new(DEVICE(host), "credit.pcie", get_system_memory(),
                                 get_system_io(), 0, TYPE_PCIE_BUS);
    g_assert_true(qbus_realize(BUS(host->bus), &error_abort));
    bool saved_msi = msi_nonbroken;
    msi_nonbroken = true;
    g_test_add_func("/agfx/frame-credit/normal-real-submit", test_normal);
    g_test_add_func("/agfx/frame-credit/reset-two", test_reset_two);
    g_test_add_func("/agfx/frame-credit/port-iosfc", test_port_iosfc);
    g_test_add_func("/agfx/frame-credit/port-texture", test_port_texture);
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
