/* Exact saved task-alias backing for the ordinary PCI EFI Buffer source.
 * No device activation, new mapping algorithm, pixel copy or GPU fallback. */
#include "qemu/osdep.h"

#include "hw/display/apple-gfx-ml.h"
#include "hw/display/apple-gfx-eeee-process-vm.h"
#include "hw/display/apple-gfx-efi-capture.h"
#include "hw/display/apple-gfx-efi-source.h"

void agfx_efi_source_fill_provider(qmu_efi_provider_v2 *provider,
                                 AppleGfxMLState *s)
{
    assert(provider && s && s->eeee_provider_state);
    *provider = (qmu_efi_provider_v2) {
        .abi_version = QMU_EFI_PROVIDER_ABI_VERSION,
        .struct_size = sizeof(*provider),
        /* Construction operand only, not live-access proof or vendor choice. */
        .source_kind = QMU_EFI_SOURCE_HOST_ALLOCATION,
        .native_facilities = {
            .abi_magic = QMU_EXTENDED_CALLBACKS_ABI_MAGIC,
            .abi_version = QMU_EXTENDED_ABI_VERSION,
            .struct_size = sizeof(qmu_extended_callbacks),
            .user_ctx = s,
        },
        .capture_source = agfx_efi_source_capture_v2,
        .stop_on_present_queue = agfx_efi_source_stop_on_present_queue,
        .schedule_present = agfx_efi_source_schedule_present,
    };
    agfx_eeee_provider_fill_callbacks(&provider->native_facilities);
}
