/* Source-private adapter for the explicit PCI EFI task/mapping constructor.
 * It does not activate a provider or change the legacy session constructor. */
#ifndef HW_DISPLAY_APPLE_GFX_EFI_SOURCE_H
#define HW_DISPLAY_APPLE_GFX_EFI_SOURCE_H

#include "qmu/qmu_efi_source.h"

typedef struct AppleGfxMLState AppleGfxMLState;

/* The caller must already have installed the actual provider State in this
 * same device. The descriptor copies all18 native facilities as one table;
 * its capture callback owns original saved physical backing, not GPA/HVA
 * lookup or pixels. Device/State and callback quiescence remain owned by the
 * explicit constructor and its successful reverse teardown, not this fill. */
void agfx_efi_source_fill_provider(qmu_efi_provider_v2 *provider,
                                 AppleGfxMLState *state);

/* Same actual serial-present lane and accepted Session operation. Leaf runs
 * once outside queue/BQL locks, after prior present RETURN; the original
 * QMetal caller handles its own stored exception after this callback RETURN. */
void agfx_efi_source_stop_on_present_queue(void *state,
                                         void (*invoke_leaf)(void *),
                                         void *leaf_ctx);
void agfx_efi_source_schedule_present(void *state);

#endif /* HW_DISPLAY_APPLE_GFX_EFI_SOURCE_H */
