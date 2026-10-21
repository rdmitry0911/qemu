/* Connected owned-output control derived from retained frame-credit fixture, not a VM or a GPU-busy/visual witness.
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
#include "qemu/memfd.h"
#include "qemu/units.h"
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
#include "qmu/qmu_efi_source.h"
#include "qmu/qmu_owned_display_frame.h"
#include "hw/display/apple-gfx-efi-source.h"
#include "apple-gfx-ml-credit-test-bridge.h"
#include "apple-gfx-ml-connected-gamma-test-bridge.h"

#define TYPE_CREDIT_HOST "test-agfx-credit-host"
#define TYPE_CREDIT_DEVICE "test-agfx-credit-device"
#define TYPE_CREDIT_RAM "test-agfx-credit-ram"
#define TYPE_CONNECTED_SOURCE "test-agfx-connected-source"
#define CREDIT_GPA UINT64_C(0x10000000)
#define CREDIT_WIDTH 16
#define CREDIT_HEIGHT 12
#define MAX_FENCES 16

typedef struct CreditRam { Object parent; MemoryRegion ram; } CreditRam;
typedef struct ConnectedSourceRam {
    CreditRam parent;
    bool *finalized;
} ConnectedSourceRam;
typedef struct FenceObservation {
    VkDevice device;
    VkFence fence;
    VkQueue queue;
    VkCommandBuffer command;
    bool submitted, terminal, destroyed;
    unsigned waits;
} FenceObservation;
typedef struct OwnedDoneSlot {
    void *control;
    qmu_owned_display_done original;
    void *context;
    bool entered, returned;
    qmu_owned_display_frame *frame;
    qmu_display_target *prepared_target;
    qmu_display_target *selected_target;
    bool read_entered, leaf_returned, bh_released;
} OwnedDoneSlot;
typedef struct OwnedInvoke {
    struct OwnedInvoke *parent;
    OwnedDoneSlot *slot;
} OwnedInvoke;
static __thread OwnedInvoke *owned_invoke;
typedef struct CreditControl {
    AppleGfxMLState *state;
    qmu_extended_callbacks original;
    qmu_efi_provider_v2 provider;
    OwnedDoneSlot owned[MAX_FENCES];
    unsigned preparations;
    GMutex mutex;
    GCond cond;
    FenceObservation fences[MAX_FENCES];
    unsigned fence_count, selected;
    unsigned render_jobs, render_retired, positive, nonpositive;
    unsigned scheduled, original_returns, callbacks, callback_returns, irq_calls;
    unsigned original_entries, original_depth, max_original_depth;
    unsigned quiesce_entries, quiesce_returns;
    bool nested_probe, nested_probe_ran, stop_accepted;
    qmu_session_work *deferred_work;
    unsigned pending_at_callback[MAX_FENCES];
    bool pixels_at_callback[MAX_FENCES];
    void *shared_maps[2];
    unsigned map_count, unmap_count;
    bool arm, selected_valid, wait_entered, release_wait;
    bool destroying, finalized;
    bool reject_provider;
    bool mode_aba, mode_aba_published, mode_aba_returned;
    qmu_session_work *mode_aba_work;
    bool hold_submit_return, submit_return_waiting, release_submit_return;
    bool hold_present_return, present_waiting, release_present_return;
    bool present_returned;
    unsigned stop_leaf_entries, inline_leaf_entries;
    CreditRam *ram;
    uint32_t texture;
} CreditControl;
typedef struct OriginalCall {
    CreditControl *control;
    void (*fn)(void *);
    void *opaque;
    unsigned ordinal;
} OriginalCall;
typedef struct RenderCall {
    CreditControl *control;
    ThreadPoolFunc *fn;
    void *opaque;
    GDestroyNotify destroy;
    qmu_session_work *expected_work;
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
static bool connected_efi;
static __thread CreditControl *render_caller;
static __thread qmu_session_work *render_work;
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
int __real_qmu_vk_encode_submit_owned_display_frame(qmu_owned_display_frame *);
int __wrap_qmu_vk_encode_submit_owned_display_frame(qmu_owned_display_frame *);
int __real_qmu_vk_submit_captured_display_frame(void *);
int __wrap_qmu_vk_submit_captured_display_frame(void *);
int __real_qmu_vk_prepare_owned_display_frame(qmu_vulkan_ctx *, qmu_display_target *,
    qmu_owned_display_done, void *, qmu_owned_display_frame **);
int __wrap_qmu_vk_prepare_owned_display_frame(qmu_vulkan_ctx *, qmu_display_target *,
    qmu_owned_display_done, void *, qmu_owned_display_frame **);
void __real_qmu_session_invoke_host_work(qmu_session_work *, void (*)(void *), void *);
void __wrap_qmu_session_invoke_host_work(qmu_session_work *, void (*)(void *), void *);
void __real_qmu_owned_display_frame_release(qmu_owned_display_frame *);
void __wrap_qmu_owned_display_frame_release(qmu_owned_display_frame *);
qmu_status __real_qmu_display_copy_selected_current_target(qmu_owned_display_frame *,
    qmu_display_target *, void *, uint32_t, uint32_t, uint32_t,
    qmu_render_frame_completion *);
qmu_status __wrap_qmu_display_copy_selected_current_target(qmu_owned_display_frame *,
    qmu_display_target *, void *, uint32_t, uint32_t, uint32_t,
    qmu_render_frame_completion *);
qmu_session *__real_qmu_create_extended_efi_v2(const qmu_extended_config *,
    const qmu_extended_callbacks *, const qmu_efi_provider_v2 *);
qmu_session *__wrap_qmu_create_extended_efi_v2(const qmu_extended_config *,
    const qmu_extended_callbacks *, const qmu_efi_provider_v2 *);
qmu_status __real_qmu_iosfc_present_tick(qmu_session *);
qmu_status __wrap_qmu_iosfc_present_tick(qmu_session *);
void __real_thread_pool_submit_immediate(ThreadPool *, ThreadPoolFunc *, void *,
                                         GDestroyNotify);
void __wrap_thread_pool_submit_immediate(ThreadPool *, ThreadPoolFunc *, void *,
                                         GDestroyNotify);
qmu_status __real_qmu_session_quiesce_host_callback_producers_deferred(qmu_session *);
qmu_status __wrap_qmu_session_quiesce_host_callback_producers_deferred(qmu_session *);

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
    g_assert_nonnull(qmu_session_current_host_work(c->state->qmu_dev));
    g_mutex_lock(&c->mutex);
    g_assert_cmpuint(call->ordinal, ==, ++c->original_entries);
    ++c->original_depth;
    c->max_original_depth = MAX(c->max_original_depth, c->original_depth);
    bool probe = c->nested_probe && !c->nested_probe_ran;
    c->nested_probe_ran |= probe;
    if (probe) {
        /* Fresh IOSFC publication synchronously appends Mode THEN NFS before
         * returning to first()'s pump. Require an actual independent original
         * already published, so depth1 during nested AIO is not vacuous. */
        g_assert_cmpuint(c->scheduled, >, c->original_entries);
    }
    g_mutex_unlock(&c->mutex);
    if (probe) {
        /* Actual nested main-AIO polling, not direct invocation of a queue
         * leaf. A second accepted original must not enter before this returns. */
        aio_poll(qemu_get_aio_context(), false);
    }
    call->fn(call->opaque); /* Original typed fn alone retires original ctx. */
    g_mutex_lock(&c->mutex);
    --c->original_depth;
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
    *call = (OriginalCall) {.control = c, .fn = fn, .opaque = context};
    g_mutex_lock(&c->mutex);
    call->ordinal = ++c->scheduled;
    /* Establish known publication order through the actual scheduler. It
     * only appends, never calls an original inline under this observer lock. */
    c->original.schedule_display_completion(opaque, original_return, call);
    g_mutex_unlock(&c->mutex);
}

static void observe_completion(void *opaque,
                                const qmu_render_frame_completion *completion)
{
    CreditControl *c = active;
    g_assert_true(opaque == c->state);
    g_assert_nonnull(qmu_session_current_host_work(c->state->qmu_dev));
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

static void connected_mode_aba_call(void *opaque)
{
    CreditControl *c = opaque;
    g_assert_true(bql_locked());
    c->original.mode_change(c->state, CREDIT_WIDTH+4, CREDIT_HEIGHT+4,
        c->state->fb_iosurface_pixel_format, c->state->fb_protection_requirements);
    c->original.mode_change(c->state, CREDIT_WIDTH, CREDIT_HEIGHT,
        c->state->fb_iosurface_pixel_format, c->state->fb_protection_requirements);
}

static void connected_mode_aba_bh(void *opaque)
{
    CreditControl *c = opaque;
    qmu_session_invoke_host_work(c->mode_aba_work, connected_mode_aba_call, c);
    qmu_session_release_host_work(c->mode_aba_work);
    c->mode_aba_work = NULL;
    c->mode_aba_returned = true;
}

static void observe_owned_done(void *opaque, qmu_owned_display_frame *frame,
                                const qmu_render_frame_completion *completion)
{
    OwnedDoneSlot *slot = opaque;
    CreditControl *c = slot->control;
    g_assert_nonnull(frame);
    g_assert_nonnull(qmu_session_current_host_work(c->state->qmu_dev));
    g_mutex_lock(&c->mutex);
    g_assert_false(slot->entered);
    slot->entered = true;
    slot->frame = frame;
    unsigned index = c->callbacks++;
    g_assert_cmpuint(index, <, MAX_FENCES);
    c->pending_at_callback[index] = c->state->pending_frames;
    c->pixels_at_callback[index] = completion->frame_expected != 0;
    bool mode_aba = c->mode_aba && !c->mode_aba_published;
    if (mode_aba) c->mode_aba_published = true;
    changed(c);
    g_mutex_unlock(&c->mutex);
    if (mode_aba) {
        /* Preowned actual accepted operation, queued before the real frame
         * BH. Both changes run the actual State mode callback under BQL. */
        g_assert_nonnull(c->mode_aba_work);
        aio_bh_schedule_oneshot(qemu_get_aio_context(), connected_mode_aba_bh, c);
    }
    /* Actual transferred frame and pre-native QEMU packet, no substitute. */
    slot->original(slot->context, frame, completion);
    g_mutex_lock(&c->mutex);
    slot->returned = true;
    ++c->callback_returns;
    changed(c);
    g_mutex_unlock(&c->mutex);
}

void __wrap_qmu_session_invoke_host_work(qmu_session_work *work,
    void (*fn)(void *), void *opaque)
{
    OwnedInvoke call = {.parent=owned_invoke};
    owned_invoke = &call;
    __real_qmu_session_invoke_host_work(work, fn, opaque);
    if (call.slot) {
        g_assert_true(bql_locked());
        call.slot->leaf_returned = true;
    }
    owned_invoke = call.parent;
}

qmu_status __wrap_qmu_display_copy_selected_current_target(
    qmu_owned_display_frame *frame, qmu_display_target *target, void *pixels,
    uint32_t width, uint32_t height, uint32_t stride,
    qmu_render_frame_completion *metadata)
{
    CreditControl *c = active;
    g_assert_true(bql_locked());
    g_assert_nonnull(owned_invoke);
    OwnedDoneSlot *slot = NULL;
    for (unsigned i=0; i<c->preparations; ++i) {
        if (c->owned[i].frame == frame) slot=&c->owned[i];
    }
    g_assert_nonnull(slot);
    g_assert_false(slot->leaf_returned);
    g_assert_false(slot->read_entered);
    slot->read_entered = true;
    slot->selected_target = target;
    g_assert_true(target == c->state->display_target);
    if (c->mode_aba && slot == &c->owned[0]) {
        g_assert_true(c->mode_aba_returned);
        g_assert_true(target != slot->prepared_target);
    }
    owned_invoke->slot = slot;
    /* Real CURRENT read, original arguments and original status. */
    return __real_qmu_display_copy_selected_current_target(frame, target,
        pixels, width, height, stride, metadata);
}

void __wrap_qmu_owned_display_frame_release(qmu_owned_display_frame *frame)
{
    OwnedDoneSlot *slot = NULL;
    CreditControl *c = active;
    if (c && frame && bql_locked()) {
        for (unsigned i=0; i<c->preparations; ++i) {
            if (c->owned[i].frame == frame) {
                slot=&c->owned[i];
            }
        }
        if (slot && !slot->read_entered) slot=NULL;
        if (slot) {
            /* This specifically rejects donor release inside the leaf: the
             * original invoke must actually RETURN before terminal release. */
            g_assert_true(slot->leaf_returned);
            g_assert_false(slot->bh_released);
        }
    }
    __real_qmu_owned_display_frame_release(frame);
    if (slot) slot->bh_released = true; /* only release RETURN */
}

int __wrap_qmu_vk_prepare_owned_display_frame(qmu_vulkan_ctx *context,
    qmu_display_target *target, qmu_owned_display_done done, void *opaque,
    qmu_owned_display_frame **out)
{
    CreditControl *c = active;
    g_assert_true(bql_locked());
    g_assert_nonnull(done);
    g_assert_nonnull(opaque);
    g_assert_nonnull(qmu_session_current_host_work(c->state->qmu_dev));
    g_mutex_lock(&c->mutex);
    g_assert_cmpuint(c->preparations, <, MAX_FENCES);
    OwnedDoneSlot *slot = &c->owned[c->preparations++];
    *slot = (OwnedDoneSlot) {.control=c, .original=done, .context=opaque,
        .prepared_target=target};
    g_mutex_unlock(&c->mutex);
    return __real_qmu_vk_prepare_owned_display_frame(context, target,
        observe_owned_done, slot, out);
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

qmu_session *__wrap_qmu_create_extended_efi_v2(const qmu_extended_config *config,
    const qmu_extended_callbacks *callbacks, const qmu_efi_provider_v2 *provider)
{
    CreditControl *c = active;
    g_assert_true(c->state->efi_task_source);
    g_assert_nonnull(c->state->eeee_provider_state);
    g_assert_cmpuint(provider->abi_version, ==, QMU_EFI_PROVIDER_ABI_VERSION);
    g_assert_cmpuint(provider->struct_size, ==, sizeof(*provider));
    g_assert_cmpuint(provider->source_kind, ==, QMU_EFI_SOURCE_HOST_ALLOCATION);
    g_assert_cmpuint(provider->reserved, ==, 0);
    g_assert_true(provider->native_facilities.user_ctx == c->state);
    g_assert_true(callbacks->user_ctx == c->state);
    g_assert_null(callbacks->eeee_task_allocate);
    c->provider = *provider;
    c->original = *callbacks;
    qmu_extended_callbacks observed = *callbacks;
    observed.schedule_display_completion = observe_schedule;
    observed.render_frame_complete = observe_completion;
    observed.raise_irq = observe_irq;
    observed.map_gpa = observe_map;
    observed.unmap_gpa = observe_unmap;
    if (c->reject_provider) {
        qmu_efi_provider_v2 rejected = *provider;
        rejected.source_kind = QMU_EFI_SOURCE_NONE;
        /* REAL factory rejects before base Session publication. No injected
         * NULL native result or callback-retaining substitute factory. */
        return __real_qmu_create_extended_efi_v2(config, &observed, &rejected);
    }
    return __real_qmu_create_extended_efi_v2(config, &observed, provider);
}

qmu_status __wrap_qmu_iosfc_present_tick(qmu_session *session)
{
    CreditControl *c = active;
    qmu_status result = __real_qmu_iosfc_present_tick(session);
    if (c && c->hold_present_return) {
        g_assert_true(session == c->state->qmu_dev);
        g_mutex_lock(&c->mutex);
        qatomic_set(&c->present_waiting, true);
        changed(c);
        const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
        while (!c->release_present_return) {
            g_assert_true(g_cond_wait_until(&c->cond, &c->mutex, deadline));
        }
        c->present_returned = true;
        changed(c);
        g_mutex_unlock(&c->mutex);
    }
    return result;
}

int __wrap_qmu_vk_encode_submit_owned_display_frame(qmu_owned_display_frame *context)
{
    CreditControl *c = render_caller;
    g_assert_nonnull(c);
    g_assert_false(bql_locked());
    g_assert_nonnull(render_work);
    g_assert_true(qmu_session_current_host_work(c->state->qmu_dev) == render_work);
    int result = __real_qmu_vk_encode_submit_owned_display_frame(context);
    g_mutex_lock(&c->mutex);
    if (result > 0) {
        ++c->positive;
    } else {
        ++c->nonpositive;
    }
    changed(c);
    if (result > 0 && c->hold_submit_return) {
        c->submit_return_waiting = true;
        changed(c);
        const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
        while (!c->release_submit_return) {
            g_assert_true(g_cond_wait_until(&c->cond, &c->mutex, deadline));
        }
    }
    g_mutex_unlock(&c->mutex);
    return result;
}

int __wrap_qmu_vk_submit_captured_display_frame(void *context)
{
    CreditControl *c = render_caller;
    g_assert_nonnull(c);
    g_assert_false(bql_locked());
    g_assert_true(qmu_session_current_host_work(c->state->qmu_dev) == render_work);
    int result = __real_qmu_vk_submit_captured_display_frame(context);
    g_mutex_lock(&c->mutex);
    if (result > 0) ++c->positive;
    else ++c->nonpositive;
    changed(c);
    g_mutex_unlock(&c->mutex);
    return result;
}

static int render_original(void *opaque)
{
    RenderCall *call = opaque;
    g_assert_null(render_caller);
    render_caller = call->control;
    render_work = call->expected_work;
    int result = call->fn(call->opaque);
    render_caller = NULL;
    render_work = NULL;
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
    *call = (RenderCall) {.control = c, .fn = fn, .opaque = opaque,
                         .destroy = destroy};
    call->expected_work = qmu_session_current_host_work(c->state->qmu_dev);
    g_assert_nonnull(call->expected_work);
    g_mutex_lock(&c->mutex);
    if (c->render_jobs == 1 && c->deferred_work) {
        g_assert_true(call->expected_work == c->deferred_work);
    }
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

static void start_transport(TransportCall *call, CreditControl *c, uint32_t pfn,
                            bool invalidate)
{
    /* SET page0 detaches the actual Nub. The separate Session invalidate API
     * is still used by dispose; those are not interchangeable operations. */
    *call = (TransportCall) {.control = c, .pfn = pfn, .invalidate = invalidate};
    qemu_thread_create(&call->thread, "credit-transport", transport_caller,
                       call, QEMU_THREAD_JOINABLE);
}

static void finish_transport(TransportCall *call, qmu_status expected)
{
    gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!qatomic_read(&call->done)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&call->thread);
    g_assert_cmpint(call->result, ==, expected);
}

static void release_observation(CreditControl *c)
{
    g_mutex_lock(&c->mutex);
    g_assert_true(c->wait_entered);
    c->release_wait = true;
    changed(c);
    g_mutex_unlock(&c->mutex);
}

static void release_during_quiesce_bh(void *opaque)
{
    CreditControl *c = opaque;

    g_assert_true(c->destroying);
    g_assert_nonnull(c->state->qmu_dev);
    g_assert_nonnull(c->state->render_pool);
    g_assert_nonnull(c->state->con);
    g_assert_true(msi_present(PCI_DEVICE(c->state)));
    g_assert_cmpint(c->state->pending_frames, ==, 2);
    g_assert_true(c->state->deferred_frame_owner.state == c->state);
    g_assert_cmpuint(c->callbacks, ==, 0);
    release_observation(c);
}

qmu_status __wrap_qmu_session_quiesce_host_callback_producers_deferred(
    qmu_session *session)
{
    CreditControl *c = active;

    g_assert_true(session == c->state->qmu_dev);
    g_assert_false(bql_locked());
    ++c->quiesce_entries;
    if (c->stop_accepted) {
        /* Control only when real observation is released. The real barrier,
         * QueueSubmit, Wait, result and production callbacks are unchanged. */
        aio_bh_schedule_oneshot(qemu_get_aio_context(),
                               release_during_quiesce_bh, c);
    }
    qmu_status status =
        __real_qmu_session_quiesce_host_callback_producers_deferred(session);
    ++c->quiesce_returns;
    return status;
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
    if (!c->stop_accepted) {
        g_assert_true(idle(c));
    }
    parent_exit(device); /* Actual pumped stop and physical/child owner barrier. */
    g_assert_null(c->state->qmu_dev);
    g_assert_null(c->state->render_pool);
    g_assert_true(idle(c));
    g_assert_cmpuint(c->quiesce_entries, ==, 1);
    g_assert_cmpuint(c->quiesce_returns, ==, 1);
    g_assert_null(c->state->deferred_frame_owner.state);
    g_assert_cmpuint(c->max_original_depth, ==, c->scheduled ? 1 : 0);
    g_assert_null(c->state->con);
    g_assert_false(msi_present(device));
    g_assert_cmpuint(c->map_count, ==, c->unmap_count);
    g_assert_null(c->shared_maps[0]);
    g_assert_null(c->shared_maps[1]);
    for (unsigned i = 0; i < c->fence_count; ++i) {
        g_assert_true(c->fences[i].destroyed);
    }
    /* Product exit now retires its own matching Console/RAM/MSI owners while
     * PCI config/MR remain live. Do not perform a second test-only retirement. */
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

static void connected_source_finalize(Object *object)
{
    ConnectedSourceRam *source = (ConnectedSourceRam *)object;
    g_assert_nonnull(source->finalized);
    g_assert_false(*source->finalized);
    *source->finalized = true;
}

static const TypeInfo connected_source_type = {
    .name = TYPE_CONNECTED_SOURCE, .parent = TYPE_CREDIT_RAM,
    .instance_size = sizeof(ConnectedSourceRam),
    .instance_finalize = connected_source_finalize,
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
    c->state->efi_task_source = connected_efi;
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
    if (!c->stop_accepted) {
        pump_until(c, idle);
        /* These cases revoke their finite output before final stop. The new
         * active-stop case deliberately performs no such pre-drain. */
        if (!c->state->efi_task_source) {
            start_transport(&control, c, 0, true);
            finish_transport(&control, QMU_OK);
        }
        pump_until(c, idle);
    }
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
    g_test_message("backend_transfers=%u render_native_submits=%u callbacks=%u local_nonpositive=%u; real component owner barrier and terminal fences, NOT VM/whole runtime closure",
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
    if (__atomic_load_n(&c->state->pending_frames, __ATOMIC_SEQ_CST) != 2) {
        return false;
    }
    g_assert_true(c->state->deferred_frame_owner.state == c->state);
    g_assert_nonnull(c->state->deferred_frame_owner.work);
    c->deferred_work = c->state->deferred_frame_owner.work;
    return true;
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
    start_transport(&call, &c, CREDIT_GPA >> 12, false);
    finish_transport(&call, QMU_OK);
    g_assert_cmpuint(c.map_count, ==, 1);
    g_assert_cmpuint(c.unmap_count, ==, 0);
    first(&c, family);
    const uint64_t epoch = agfx_credit_delivery_epoch(c.state->qmu_dev);
    const uint32_t busy_pfns[] = {CREDIT_GPA >> 12, (CREDIT_GPA >> 12) + 1};
    /* Primary Device rejects occupied SET, including the same PFN. It must
     * not replace the original page or invalidate its accepted frame. */
    for (unsigned i = 0; i < ARRAY_SIZE(busy_pfns); ++i) {
        start_transport(&call, &c, busy_pfns[i], false);
        finish_transport(&call, QMU_ERROR);
        g_assert_cmpuint(agfx_credit_delivery_epoch(c.state->qmu_dev), ==, epoch);
        g_assert_cmpuint(c.map_count, ==, 1);
        g_assert_cmpuint(c.unmap_count, ==, 0);
        g_assert_cmpuint(c.callbacks, ==, 0);
        g_assert_cmpint(c.state->pending_frames, ==, 1);
    }
    /* Pinned x86 setNub's waitForPendingFrames is an empty six-byte return,
     * not a native fence drain. CURRENT is revoked, but the accepted native
     * command and its original logical credit still own their real terminal.
     * Both SET0 and the successor SET must RETURN before fence observation. */
    start_transport(&call, &c, 0, false);
    finish_transport(&call, QMU_OK); /* Main AIO pumps real RAM/DMA/IRQ BHs. */
    g_assert_cmpuint(c.map_count, ==, 1);
    g_assert_cmpuint(c.unmap_count, ==, 1);
    g_assert_cmpuint(c.callbacks, ==, 0);
    g_assert_cmpint(c.state->pending_frames, ==, 1);
    start_transport(&call, &c, (CREDIT_GPA >> 12) + 1, false);
    finish_transport(&call, QMU_OK);
    g_assert_cmpuint(c.map_count, ==, 2);
    g_assert_cmpuint(c.unmap_count, ==, 1);
    g_assert_cmpuint(c.callbacks, ==, 0);
    g_assert_cmpint(c.state->pending_frames, ==, 1);
    release_observation(&c);
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

static void test_owner_stop_two(void)
{
    CreditControl c;

    create(&c, AGFX_CREDIT_IOSFC);
    c.nested_probe = true;
    first(&c, AGFX_CREDIT_IOSFC);
    g_assert_true(c.nested_probe_ran);
    publish(&c, AGFX_CREDIT_IOSFC, 0xff405060);
    pump_until(&c, two_credits);
    c.stop_accepted = true;
    dispose(&c); /* No reset/invalidate/idle pre-drain before actual exit. */
    g_assert_cmpuint(c.positive, ==, 2);
    g_assert_cmpuint(c.callbacks, ==, 2);
    g_assert_cmpuint(c.nonpositive, ==, 0);
}

#ifndef AGFX_OWNED_CREDIT_COMPAT_ONLY
static void test_provider_null_unwind(void)
{
    CreditControl c = {.reject_provider=true};
    Error *error = NULL;
    g_mutex_init(&c.mutex);
    g_cond_init(&c.cond);
    active = &c;
    c.state = APPLE_GFX_ML(object_new(TYPE_CREDIT_DEVICE));
    DEVICE(c.state)->hotplugged = false;
    c.state->vram_size_mb = 1;
    c.state->efi_task_source = true;
    g_assert_false(qdev_realize(DEVICE(c.state), BUS(host->bus), &error));
    g_assert_nonnull(error);
    g_assert_nonnull(strstr(error_get_pretty(error), "Failed to create qmetal device"));
    g_assert_null(c.state->eeee_provider_state);
    g_assert_null(c.state->qmu_dev);
    g_assert_null(c.state->render_pool);
    g_assert_null(c.state->con);
    g_assert_false(c.state->log_writer_started);
    g_assert_false(c.state->msi_used);
    g_assert_cmpuint(c.scheduled, ==, 0);
    g_assert_cmpuint(c.callbacks, ==, 0);
    error_free(error);
    object_unparent(OBJECT(c.state));
    object_unref(OBJECT(c.state));
    drain_rcu();
    g_assert_true(c.finalized);
    active = NULL;
    g_cond_clear(&c.cond);
    g_mutex_clear(&c.mutex);
}

static bool bh_before_worker_return(CreditControl *c)
{
    g_mutex_lock(&c->mutex);
    bool complete = c->submit_return_waiting && c->callbacks >= 1 &&
        c->callback_returns >= 1 && c->render_retired == 0;
    g_mutex_unlock(&c->mutex);
    return complete && c->state->frame_count != 0;
}

static void connected_original_host(bool hold_submit_return, bool mode_aba);

static void test_callback_bh_before_encode_return(void)
{
    /* Same real original HOST/MMIO source, never generation-less legacy
     * helper publication masquerading as a v2 physical source. */
    connected_original_host(true, false);
}

typedef struct ConnectedStop {
    CreditControl *control;
    qmu_session_work *work;
    QemuThread thread;
    bool entered, done;
} ConnectedStop;

static void connected_inline_leaf(void *opaque)
{
    CreditControl *c = opaque;
    g_assert_false(bql_locked());
    g_assert_true(qemu_thread_is_self(&c->state->bootstrap_present_worker));
    g_assert_cmpint(qemu_mutex_trylock(&c->state->bootstrap_present_mutex), ==, 0);
    qemu_mutex_unlock(&c->state->bootstrap_present_mutex);
    ++c->inline_leaf_entries;
}

static void connected_stop_leaf(void *opaque)
{
    CreditControl *c = opaque;
    g_assert_true(c->present_returned);
    g_assert_cmpuint(c->stop_leaf_entries++, ==, 0);
    g_assert_false(c->state->bootstrap_present_timer.active);
    /* Real self-lane branch, not an analogous queue/helper implementation. */
    agfx_efi_source_stop_on_present_queue(c->state, connected_inline_leaf, c);
    g_assert_cmpuint(c->inline_leaf_entries, ==, 1);
}

static void connected_stop_call(void *opaque)
{
    ConnectedStop *call = opaque;
    qatomic_set(&call->entered, true);
    agfx_efi_source_stop_on_present_queue(call->control->state,
        connected_stop_leaf, call->control);
}

static void *connected_stop_thread(void *opaque)
{
    ConnectedStop *call = opaque;
    qmu_session_invoke_host_work(call->work, connected_stop_call, call);
    qmu_session_release_host_work(call->work);
    qatomic_set(&call->done, true);
    aio_wait_kick();
    return NULL;
}

static void connected_schedule_call(void *opaque)
{
    agfx_efi_source_schedule_present(((CreditControl *)opaque)->state);
}

static void test_serial_stop_reset_inline(void)
{
    CreditControl c;
    connected_efi = true;
    create(&c, AGFX_CREDIT_IOSFC);
    connected_efi = false;
    g_assert_true(c.state->efi_task_source);
    c.hold_present_return = true;
    qmu_session_work *work = qmu_session_acquire_host_work(c.state->qmu_dev);
    g_assert_nonnull(work);
    qmu_session_invoke_host_work(work, connected_schedule_call, &c);
    qmu_session_release_host_work(work);
    gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!qatomic_read(&c.present_waiting)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    ConnectedStop stop = {.control=&c,
        .work=qmu_session_acquire_host_work(c.state->qmu_dev)};
    g_assert_nonnull(stop.work);
    qemu_thread_create(&stop.thread, "efi-stop-control", connected_stop_thread,
        &stop, QEMU_THREAD_JOINABLE);
    while (true) {
        qemu_mutex_lock(&c.state->bootstrap_present_mutex);
        bool queued = c.state->bootstrap_present_cmd_tail != NULL;
        qemu_mutex_unlock(&c.state->bootstrap_present_mutex);
        if (queued) break;
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    g_assert_true(qatomic_read(&stop.entered));
    g_assert_false(qatomic_read(&stop.done));
    g_assert_cmpuint(c.stop_leaf_entries, ==, 0);
    /* Actual device reset must preserve the already accepted STOP waiter. */
    device_cold_reset(DEVICE(c.state));
    g_mutex_lock(&c.mutex);
    c.release_present_return = true;
    changed(&c);
    g_mutex_unlock(&c.mutex);
    while (!qatomic_read(&stop.done)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&stop.thread);
    g_assert_cmpuint(c.stop_leaf_entries, ==, 1);
    g_assert_cmpuint(c.inline_leaf_entries, ==, 1);
    g_assert_false(c.state->bootstrap_present_timer.active);
    c.hold_present_return = false;
    dispose(&c);
}

static void connected_mmio(AppleGfxMLState *s, hwaddr offset, uint32_t value)
{
    g_assert_cmpint(memory_region_dispatch_write(&s->mmio, offset, value,
        MO_32, MEMTXATTRS_UNSPECIFIED), ==, MEMTX_OK);
}

static bool connected_presented(CreditControl *c)
{
    return c->state->frame_count != 0;
}

typedef struct ConnectedGammaPhase {
    const char *name;
    unsigned layer;
    uint32_t expected;
} ConnectedGammaPhase;

typedef struct ConnectedGammaCall {
    CreditControl *control;
    qmu_session_work *work;
    QemuThread thread;
    int result;
    bool done;
} ConnectedGammaCall;

static uint32_t connected_read32(AppleGfxMLState *s, hwaddr offset)
{
    uint64_t value;
    g_assert_cmpint(memory_region_dispatch_read(&s->mmio, offset, &value,
        MO_32, MEMTXATTRS_UNSPECIFIED), ==, MEMTX_OK);
    return value;
}

static void connected_gamma_call(void *opaque)
{
    ConnectedGammaCall *call = opaque;
    g_assert_false(bql_locked());
    g_assert_true(qmu_session_current_host_work(call->control->state->qmu_dev)
        == call->work);
    call->result = agfx_connected_gamma_update(call->control->state->qmu_dev,
        0x5000, 24, 2, 0x7f000000);
}

static void *connected_gamma_thread(void *opaque)
{
    ConnectedGammaCall *call = opaque;
    qmu_session_invoke_host_work(call->work, connected_gamma_call, call);
    qmu_session_release_host_work(call->work);
    qatomic_set(&call->done, true);
    aio_wait_kick();
    return NULL;
}

static uint64_t connected_prepare_gamma(CreditControl *c, uint8_t *source,
    uint64_t source_gpa, const ConnectedGammaPhase *phase)
{
    const uint32_t root_pfn = (source_gpa + 0x1000) >> 12;
    uint64_t epoch = 0;
    g_assert_cmpint(agfx_connected_gamma_task_state(c->state->qmu_dev, 0,
        0, 0, &epoch), ==, 0); /* Fresh real P0 task0 must be absent. */
    const uint32_t descriptor[2] = {(source_gpa + 0x2000) >> 12, 1};
    memcpy(source + 0x1000, descriptor, sizeof(descriptor));
    memset(source + 0x2000, 0, 4096);
    const uint32_t leaf_pfn = (source_gpa + 0x3000) >> 12;
    memcpy(source + 0x2000 + 5 * sizeof(leaf_pfn), &leaf_pfn, sizeof(leaf_pfn));
    uint32_t table[6] = {0};
    table[phase->layer * 2] = table[phase->layer * 2 + 1] = 0x3f800000;
    uint32_t sum = 0;
    for (unsigned i = 0; i < G_N_ELEMENTS(table); ++i) sum += table[i];
    g_assert_cmphex(sum, ==, 0x7f000000);
    memcpy(source + 0x3000, table, sizeof(table));
    memset(source + 0x4000, 0, 8192); /* Separate stamp page and ROOT ring. */
    uint32_t queued = agfx_connected_gamma_define_task(source + 0x5000,
        root_pfn);
    g_assert_cmpuint(queued, ==, 28);
    /* All original frame/PT/LUT/ring writes precede real physical ingress. */
    connected_mmio(c->state, PVG_REG_PROTOCOL_VERSION, 6);
    g_assert_cmphex(connected_read32(c->state, PVG_REG_PROTOCOL_VERSION), ==, 6);
    connected_mmio(c->state, PVG_REG_FIFO_PFN, (source_gpa + 0x4000) >> 12);
    connected_mmio(c->state, PVG_REG_FIFO_SIZE, 4096);
    connected_mmio(c->state, PVG_REG_FIFO_OFFSET, 4096);
    connected_mmio(c->state, PVG_REG_FIFO_ENABLE, 1);
    connected_mmio(c->state, PVG_REG_FIFO_WRITE, queued);
    g_assert_cmpuint(connected_read32(c->state, PVG_REG_FIFO_READ), ==, queued);
    g_assert_cmphex(connected_read32(c->state, PVG_REG_FAULT_STATUS), ==, 0);
    uint32_t stamp;
    memcpy(&stamp, source + 0x4000, sizeof(stamp));
    g_assert_cmpuint(stamp, ==, 1);
    g_assert_cmpint(agfx_connected_gamma_task_state(c->state->qmu_dev, 1,
        root_pfn, 4 * MiB, &epoch), ==, 0);
    g_assert_cmpuint(epoch, >, 0);
    ConnectedGammaCall call = {.control = c,
        .work = qmu_session_acquire_host_work(c->state->qmu_dev), .result = -1};
    g_assert_nonnull(call.work);
    qemu_thread_create(&call.thread, "efi-gamma-upload", connected_gamma_thread,
        &call, QEMU_THREAD_JOINABLE);
    gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!qatomic_read(&call.done)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false); /* Actual DMA read BHs. */
        g_thread_yield();
    }
    qemu_thread_join(&call.thread);
    g_assert_cmpint(call.result, ==, 0);
    g_assert_cmphex(connected_read32(c->state, PVG_REG_FAULT_STATUS), ==, 0);
    return epoch;
}

/* Finite original HOST read: stores finish before MMIO/worker publication,
 * no source stores overlap any encode or GPU access. This is NOT live KVM
 * guest-writer/access-domain evidence. Run on an exact admitted eligible HOST
 * tuple; no access flag, vendor fallback, replacement or simulated native. */
static void connected_original_host_phase(bool hold_submit_return, bool mode_aba,
    const ConnectedGammaPhase *gamma)
{
    const uint64_t source_gpa = CREDIT_GPA + 64 * MiB;
    const uint64_t span = 64 * MiB;
    gint64 gamma_started = gamma ? g_get_monotonic_time() : 0;
    uint64_t gamma_epoch = 0;
    CreditControl c;
    connected_efi = true;
    create(&c, AGFX_CREDIT_IOSFC);
    connected_efi = false;
    c.hold_submit_return = hold_submit_return;
    c.mode_aba = mode_aba;
    if (mode_aba) {
        c.mode_aba_work = qmu_session_acquire_host_work(c.state->qmu_dev);
        g_assert_nonnull(c.mode_aba_work);
    }
    bool source_finalized = false;
    ConnectedSourceRam *owner = (ConnectedSourceRam *)object_new(TYPE_CONNECTED_SOURCE);
    owner->finalized = &source_finalized;
    CreditRam *source = &owner->parent;
    int fd = qemu_memfd_create("efi-connected-original", span, false, 0,
        F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL, &error_abort);
    g_assert_cmpint(fd, >=, 0);
    g_assert_true(memory_region_init_ram_from_fd(&source->ram, OBJECT(source),
        "efi-connected-original", span, RAM_SHARED, fd, 0, &error_abort));
    memory_region_add_subregion(get_system_memory(), source_gpa, &source->ram);
    uint32_t *pixels = memory_region_get_ram_ptr(&source->ram);
    for (unsigned i=0; i < CREDIT_WIDTH*CREDIT_HEIGHT; ++i) pixels[i]=0xff102030;
    if (gamma) {
        gamma_epoch = connected_prepare_gamma(&c, (uint8_t *)pixels,
            source_gpa, gamma);
    }
    /* Original host stores precede actual MMIO queue/mutex publication and
     * native HOST_WRITE->SHADER_READ barrier. No later source write exists. */
    connected_mmio(c.state, 0x1200, 1);
    connected_mmio(c.state, 0x1210, source_gpa >> 12);
    connected_mmio(c.state, 0x1214, span);
    connected_mmio(c.state, 0x1218, 32);
    connected_mmio(c.state, 0x1228, CREDIT_WIDTH*4);
    connected_mmio(c.state, 0x121c, (CREDIT_HEIGHT<<16)|CREDIT_WIDTH);
    pump_until(&c, hold_submit_return ? bh_before_worker_return : connected_presented);
    if (hold_submit_return) {
        g_assert_true(c.owned[0].read_entered);
        g_assert_true(c.owned[0].leaf_returned);
        g_assert_true(c.owned[0].bh_released);
        g_mutex_lock(&c.mutex);
        c.release_submit_return = true;
        changed(&c);
        g_mutex_unlock(&c.mutex);
    }
    g_assert_cmpuint(c.preparations, >, 0);
    g_assert_cmpuint(c.positive, >, 0);
    g_assert_nonnull(c.state->display_fb);
    for (unsigned y=0; !mode_aba && y<CREDIT_HEIGHT; ++y) {
        const uint32_t *row = (const uint32_t *)(c.state->display_fb +
            y*c.state->fb_stride);
        for (unsigned x=0; x<CREDIT_WIDTH; ++x) {
            if (gamma) {
                g_assert_cmphex(row[x], ==, gamma->expected);
            } else {
                g_assert_cmphex(row[x], ==, 0xff102030);
            }
        }
    }
    if (mode_aba) {
        g_assert_true(c.mode_aba_returned);
        g_assert_true(c.owned[0].read_entered);
        g_assert_true(c.owned[0].leaf_returned);
        g_assert_true(c.owned[0].bh_released);
        g_assert_true(c.owned[0].selected_target != c.owned[0].prepared_target);
        /* A newly allocated CURRENT output's initial bytes are not asserted
         * as a visual/black-clear contract. This gate proves exact identity. */
    }
    connected_mmio(c.state, 0x1200, 0); /* Actual serial stop + task teardown. */
    pump_until(&c, idle);
    dispose(&c); /* True canonical source/drainer/native/done/BH barrier. */
    g_assert_false(source_finalized); /* caller still owns actual source MR */
    memory_region_del_subregion(get_system_memory(), &source->ram);
    object_unref(OBJECT(source));
    drain_rcu();
    drain_rcu();
    g_assert_true(source_finalized);
    if (gamma) {
        g_test_message("GAMMA phase=%s layer=%u expected_BGRA=0x%08x "
            "task0_epoch=%" PRIu64 " elapsed_us=%" G_GINT64_FORMAT
            "; exact native Console rows, canonical Session/task/source teardown",
            gamma->name, gamma->layer, gamma->expected, gamma_epoch,
            g_get_monotonic_time() - gamma_started);
    }
}

static void connected_original_host(bool hold_submit_return, bool mode_aba)
{
    connected_original_host_phase(hold_submit_return, mode_aba, NULL);
}

static void test_original_host_mmio_native_gamma(void)
{
    const ConnectedGammaPhase phases[] = {
        {"red-layer0", 0, 0xffff0000},
        {"green-layer1", 1, 0xff00ff00},
        {"blue-layer2", 2, 0xff0000ff},
    };
    for (unsigned i = 0; i < G_N_ELEMENTS(phases); ++i) {
        /* Independent Session/source/native lifecycles, no in-flight rewrite. */
        connected_original_host_phase(false, false, &phases[i]);
    }
}

static void test_original_host_mmio_native(void)
{
    connected_original_host(false, false);
}

static void test_mode_aba_selected_current(void)
{
    connected_original_host(false, true);
}
#endif

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
    int capture_setup = agfx_credit_exit_capture_initialize(argc, argv);
    if (capture_setup != 0) {
        return capture_setup;
    }
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
    resolved("normal-owned-encode-submit",
             (void *)&__real_qmu_vk_encode_submit_owned_display_frame, dso_provider);
    resolved("real-factory", (void *)&__real_qmu_create_extended, dso_provider);
    resolved("real-owner-barrier",
             (void *)&__real_qmu_session_quiesce_host_callback_producers_deferred,
             dso_provider);
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
    type_register_static(&connected_source_type);
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
    g_test_add_func("/agfx/frame-credit/owner-stop-two", test_owner_stop_two);
#ifndef AGFX_OWNED_CREDIT_COMPAT_ONLY
    g_test_add_func("/agfx/owned-connected/provider-null-unwind",
        test_provider_null_unwind);
    g_test_add_func("/agfx/owned-connected/callback-bh-before-encode-return",
        test_callback_bh_before_encode_return);
    g_test_add_func("/agfx/owned-connected/serial-stop-reset-inline",
        test_serial_stop_reset_inline);
    g_test_add_func("/agfx/owned-connected/original-host-mmio-native",
        test_original_host_mmio_native);
    g_test_add_func("/agfx/owned-connected/original-host-mmio-native-gamma",
        test_original_host_mmio_native_gamma);
    g_test_add_func("/agfx/owned-connected/mode-aba-selected-current",
        test_mode_aba_selected_current);
#endif
    int status = g_test_run();
    msi_nonbroken = saved_msi;
    pci_root_bus_cleanup(host->bus);
    object_unref(OBJECT(host));
    drain_rcu();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return agfx_credit_exit_capture_result(status);
}
