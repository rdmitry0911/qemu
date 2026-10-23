/*
 * Apple PVG EEEE persistent process-VM provider.
 *
 * This is the QEMU half of the source-derived native task facilities.  The
 * captured PCI eeee personality selects legacy PGTask, not PGMappingTask's
 * protect-NONE profile.  This source-private carrier may be
 * published only together with the complete QMetal physical-session owner
 * graph.  In particular, filling one ABI field or one callback from this
 * carrier is not a valid compatibility mode.
 */

#ifndef HW_DISPLAY_APPLE_GFX_EEEE_PROCESS_VM_H
#define HW_DISPLAY_APPLE_GFX_EEEE_PROCESS_VM_H

#include <stdbool.h>
#include <stdint.h>
#include "apple-gfx-eeee-source-backing-caps.h"

typedef struct AgfxEeeeProviderState AgfxEeeeProviderState;
typedef struct qmu_extended_callbacks qmu_extended_callbacks;
typedef struct AgfxEeeeSourceBacking AgfxEeeeSourceBacking;

/* Source-private native-resource ownership, NOT an extended callback ABI.
 * Call capture outside task_lock, while the caller still owns State lifetime.
 * It captures only admitted shared-RAM alias intervals under that lock. NULL
 * means the exact complete range could not be captured; logical task mappings
 * are unchanged. No GPA or current task-VA lookup is performed.
 *
 * The returned CPU view aliases the original shared backing (not a snapshot).
 * It and all its metadata remain valid through the final source reference,
 * independently of logical overwrite/deallocate and provider State lifetime.
 * Native GPU import eligibility and transfer completion are NOT established.
 */
AgfxEeeeSourceBacking *agfx_eeee_source_backing_capture(
    AgfxEeeeProviderState *state, uint64_t address, uint64_t length,
    uint64_t alignment);
void agfx_eeee_source_backing_ref(AgfxEeeeSourceBacking *source);
/* Final release may synchronously return through BQL. Never hold a provider
 * lock, or another lock needed by the main loop, across this call. */
void agfx_eeee_source_backing_unref(AgfxEeeeSourceBacking *source);
void *agfx_eeee_source_backing_pointer(const AgfxEeeeSourceBacking *source);
uint64_t agfx_eeee_source_backing_length(const AgfxEeeeSourceBacking *source);
/* The allocation base/size satisfy caller alignment (power of two, at least
 * native page size). The visible range may have a nonzero offset. Every byte
 * of the rounded allocation must be covered by saved admitted aliases; absent
 * padding is a construction failure, never zero-filled or looked up afresh. */
void *agfx_eeee_source_backing_allocation(const AgfxEeeeSourceBacking *source);
uint64_t agfx_eeee_source_backing_allocation_length(
    const AgfxEeeeSourceBacking *source);
uint64_t agfx_eeee_source_backing_visible_offset(
    const AgfxEeeeSourceBacking *source);

typedef struct AgfxEeeeSourceInterval {
    /* Borrowed identities/descriptor; do not close or unref. Valid while the
     * source is held. No synthetic external backing generation is claimed. */
    void *region;
    int fd;
    uint64_t region_offset;
    uint64_t file_offset;
    uint64_t allocation_offset;
    uint64_t length;
} AgfxEeeeSourceInterval;

unsigned agfx_eeee_source_backing_interval_count(
    const AgfxEeeeSourceBacking *source);
bool agfx_eeee_source_backing_interval(const AgfxEeeeSourceBacking *source,
                                     unsigned index,
                                     AgfxEeeeSourceInterval *interval);

/* Dirty only the exact saved physical pieces, including after overwrite,
 * task deallocate or State destruction. Offsets are in the visible range.
 * Caller supplies actual GPU completion/visibility; this function adds none. */
bool agfx_eeee_source_backing_mark_dirty(AgfxEeeeSourceBacking *source,
                                        uint64_t offset, uint64_t length);

/* State creation has no device-visible effect.  Realize installs it only in
 * the atomic EEEE session cutover, after the complete QMetal session has
 * accepted its source-derived receivers and reverse teardown owner. */
AgfxEeeeProviderState *agfx_eeee_provider_state_create(void);

/* Returns false rather than dropping a surviving allocation, alias, retained
 * MemoryRegion, or published task.  The caller must reject teardown in that
 * case; it must never manufacture an empty provider state.  A false return
 * does not change state or prohibit draining existing owners.  The session
 * owner must quiesce callback invocations before freeing this state; an empty
 * table alone does not establish that external lifetime condition. */
bool agfx_eeee_provider_state_destroy(AgfxEeeeProviderState *state);

/* Fill all eighteen callbacks as one indivisible process-VM provider.  The
 * caller owns user_ctx and the versioned ABI prefix. */
void agfx_eeee_provider_fill_callbacks(qmu_extended_callbacks *callbacks);


/* Dormant private full18 context owner; no active Session/PCI exit insertion.
 * The explicit owner caller must retire BEFORE its callback fields disappear.
 * Production publication requires the separately source-complete root/caller.
 * This is not a qmu public ABI or Session factory.
 * create/retire run under BQL and require an actual QOM AppleGfxMLState.
 * Acquire copies the real complete table and adds a context reference. Its
 * caller must release only AFTER all callback/provider/root destructors return.
 * stop prevents new acquisitions; existing owners may share their receipt.
 * retire pumps main AIO until acquired owners return, then preserves false
 * provider-destroy busy as false. It must precede callback-field destruction.
 * No new read/dirty producer may start after Session quiesce; those calls still
 * require their original accepted SessionWork/TLS scope. Destructor UNMAP/MR
 * callbacks use the existing no-new-root BQL path. */
typedef struct AgfxEeeeNativeContext AgfxEeeeNativeContext;
struct AppleGfxMLState;
AgfxEeeeNativeContext *agfx_eeee_native_context_create(struct AppleGfxMLState *s);
bool agfx_eeee_native_context_acquire(AgfxEeeeNativeContext *context,
                                     qmu_extended_callbacks *callbacks);
struct qmu_source_dmabuf_caps_v1;
bool agfx_eeee_native_context_source_dmabuf_caps(
    AgfxEeeeNativeContext *, struct qmu_source_dmabuf_caps_v1 *);
void agfx_eeee_native_context_release(void *context);
/* Caller already owns the SAME acquired Context through full18 facilities.
 * No new reference/public callback schema/active publication is introduced. */
bool agfx_eeee_native_context_source_backing_caps(
    AgfxEeeeNativeContext *context, struct qmu_source_backing_caps_v1 *caps);
void agfx_eeee_native_context_stop_acquiring(AgfxEeeeNativeContext *context);
bool agfx_eeee_native_context_retire(AgfxEeeeNativeContext *context);

#endif /* HW_DISPLAY_APPLE_GFX_EEEE_PROCESS_VM_H */
