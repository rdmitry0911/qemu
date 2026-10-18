/* Source-private ordinary EFI physical construction. This callback does not
 * publish a provider, select a backend or admit native access to its pixels. */
#ifndef HW_DISPLAY_APPLE_GFX_EFI_CAPTURE_H
#define HW_DISPLAY_APPLE_GFX_EFI_CAPTURE_H

#include "qmu/qmu_efi_capture.h"

/* actual_state_context is the SAME installed AppleGfxMLState used by the
 * complete native task facilities. Its provider State must remain live for
 * the synchronous capture. Successful physical owners do not borrow State.
 * No provider/task/P0/CURRENT or main-loop-dependent lock may span this call. */
int32_t agfx_efi_source_capture_v2(
    void *actual_state_context, const qmu_efi_capture_request_v2 *request,
    qmu_efi_capture_result_v2 *out_preowned_result);

#endif /* HW_DISPLAY_APPLE_GFX_EFI_CAPTURE_H */
