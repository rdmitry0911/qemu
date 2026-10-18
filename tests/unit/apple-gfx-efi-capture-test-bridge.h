/* New typed construction fixture. Accepted native551 sources stay intact. */
#ifndef APPLE_GFX_EFI_CAPTURE_TEST_BRIDGE_H
#define APPLE_GFX_EFI_CAPTURE_TEST_BRIDGE_H
#include <stddef.h>
#include <stdint.h>
#include "qmu/qmetal_unified.h"
#include "qmu/qmu_efi_capture.h"

#ifdef __cplusplus
#define EFI_LAYOUT_ASSERT static_assert
#define EFI_LAYOUT_ALIGNOF alignof
extern "C" {
#else
#define EFI_LAYOUT_ASSERT _Static_assert
#define EFI_LAYOUT_ALIGNOF _Alignof
#endif
#define EFI_OFFSET(type, member, expected) \
    EFI_LAYOUT_ASSERT(offsetof(type, member) == (expected), #type "." #member)
EFI_LAYOUT_ASSERT(sizeof(void *) == 8, "LP64 pointer");
EFI_LAYOUT_ASSERT(sizeof(qmu_efi_capture_source_v2) == 8, "LP64 capture callback");
EFI_LAYOUT_ASSERT(sizeof(((qmu_efi_capture_result_v2 *)0)->release_owner) == 8,
                  "LP64 release callback");
EFI_LAYOUT_ASSERT(sizeof(qmu_efi_capture_request_v2) == 56, "request56");
EFI_LAYOUT_ASSERT(EFI_LAYOUT_ALIGNOF(qmu_efi_capture_request_v2) == 8, "request alignment");
EFI_OFFSET(qmu_efi_capture_request_v2, abi_version, 0);
EFI_OFFSET(qmu_efi_capture_request_v2, struct_size, 4);
EFI_OFFSET(qmu_efi_capture_request_v2, source_kind, 8);
EFI_OFFSET(qmu_efi_capture_request_v2, reserved, 12);
EFI_OFFSET(qmu_efi_capture_request_v2, host_page_bytes, 16);
EFI_OFFSET(qmu_efi_capture_request_v2, task_virtual_address, 24);
EFI_OFFSET(qmu_efi_capture_request_v2, frame_storage_bytes, 32);
EFI_OFFSET(qmu_efi_capture_request_v2, native_minimum_allocation_bytes, 40);
EFI_OFFSET(qmu_efi_capture_request_v2, native_import_alignment, 48);
EFI_LAYOUT_ASSERT(sizeof(qmu_efi_capture_payload_v2) == 8, "payload8");
EFI_LAYOUT_ASSERT(EFI_LAYOUT_ALIGNOF(qmu_efi_capture_payload_v2) == 8, "payload alignment");
EFI_OFFSET(qmu_efi_capture_payload_v2, host_allocation_base, 0);
EFI_OFFSET(qmu_efi_capture_payload_v2, dma_buf.original_fd, 0);
EFI_OFFSET(qmu_efi_capture_payload_v2, dma_buf.reserved, 4);
EFI_LAYOUT_ASSERT(sizeof(qmu_efi_capture_result_v2) == 64, "result64");
EFI_LAYOUT_ASSERT(EFI_LAYOUT_ALIGNOF(qmu_efi_capture_result_v2) == 8, "result alignment");
EFI_OFFSET(qmu_efi_capture_result_v2, abi_version, 0);
EFI_OFFSET(qmu_efi_capture_result_v2, struct_size, 4);
EFI_OFFSET(qmu_efi_capture_result_v2, source_kind, 8);
EFI_OFFSET(qmu_efi_capture_result_v2, reserved, 12);
EFI_OFFSET(qmu_efi_capture_result_v2, allocation_bytes, 16);
EFI_OFFSET(qmu_efi_capture_result_v2, visible_offset, 24);
EFI_OFFSET(qmu_efi_capture_result_v2, visible_bytes, 32);
EFI_OFFSET(qmu_efi_capture_result_v2, payload, 40);
EFI_OFFSET(qmu_efi_capture_result_v2, opaque_owner, 48);
EFI_OFFSET(qmu_efi_capture_result_v2, release_owner, 56);
#undef EFI_OFFSET
#undef EFI_LAYOUT_ASSERT
#undef EFI_LAYOUT_ALIGNOF

/* Called on the fixture worker with no QEMU/provider lock. after_bound waits
 * while the main fixture retires its real logical task/RAM owner and pumps
 * AIO; the original physical source must survive until explicit retirement. */
void agfx_test_efi_storage_run(
    const qmu_extended_callbacks *facilities, qmu_efi_capture_source_v2 capture,
    uint64_t task_va, uint64_t literal_length,
    const qmu_efi_capture_result_v2 *observed,
    void (*after_bound)(void *), void *after_bound_context);
#ifdef __cplusplus
}
#endif
#endif /* APPLE_GFX_EFI_CAPTURE_TEST_BRIDGE_H */
