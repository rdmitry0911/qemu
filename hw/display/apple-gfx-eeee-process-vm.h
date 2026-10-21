/*
 * Apple PVG EEEE persistent process-VM provider.
 *
 * This is the QEMU half of the source-derived PGMappingTask adapter.  It is
 * deliberately separate from the legacy apple-gfx callback set: it may be
 * published only together with the complete QMetal physical-session owner
 * graph.  In particular, filling one ABI field or one callback from this
 * carrier is not a valid compatibility mode.
 */

#ifndef HW_DISPLAY_APPLE_GFX_EEEE_PROCESS_VM_H
#define HW_DISPLAY_APPLE_GFX_EEEE_PROCESS_VM_H

#include <stdbool.h>

typedef struct AgfxEeeeProviderState AgfxEeeeProviderState;
typedef struct qmu_extended_callbacks qmu_extended_callbacks;

/* State creation has no device-visible effect.  Realize installs it only in
 * the atomic EEEE session cutover, after the complete QMetal session has
 * accepted all 39 literal receivers and its reverse teardown owner. */
AgfxEeeeProviderState *agfx_eeee_provider_state_create(void);

/* Returns false rather than dropping a surviving allocation, alias, retained
 * MemoryRegion, or published task.  The caller must reject teardown in that
 * case; it must never manufacture an empty provider state. */
bool agfx_eeee_provider_state_destroy(AgfxEeeeProviderState *state);

/* Fill all eighteen callbacks as one indivisible process-VM provider.  The
 * caller owns user_ctx and the versioned ABI prefix. */
void agfx_eeee_provider_fill_callbacks(qmu_extended_callbacks *callbacks);

#endif /* HW_DISPLAY_APPLE_GFX_EEEE_PROCESS_VM_H */
