/* Reuse the saved ordinary EFI capture adapter with the accepted immutable
 * extent/export owners. No active factory, source selection or pixel copy. */
#include "qemu/osdep.h"

#include "hw/display/apple-gfx-ml.h"
#include "hw/display/apple-gfx-eeee-process-vm.h"
#include "hw/display/apple-gfx-eeee-source-dmabuf.h"
#include "hw/display/apple-gfx-efi-capture.h"

static void agfx_efi_host_source_release(void *owner)
{
    assert(owner);
    /* Actual native free RETURN and every physical reader precede this
     * callback. Final release may synchronously return through BQL. */
    agfx_eeee_source_backing_unref(owner);
}

static void agfx_efi_dma_source_release(void *owner)
{
    assert(owner);
    agfx_eeee_source_dmabuf_unref(owner);
}

int32_t agfx_efi_source_capture_v2(
    void *actual_state_context, const qmu_efi_capture_request_v2 *request,
    qmu_efi_capture_result_v2 *out_preowned_result)
{
    AppleGfxMLState *s = actual_state_context;
    AgfxEeeeSourceBacking *source;
    qmu_efi_capture_result_v2 result = { 0 };
    const uint64_t page = qemu_real_host_page_size();
    uint64_t visible_offset, buffer_bytes, alignment;

    /* The typed caller supplies the entire preowned output, independently
     * of the request prefix. An invalid provider lifetime is not a recoverable
     * guest mapping failure or authority to select a replacement State. */
    assert(s && s->eeee_provider_state && out_preowned_result);
    *out_preowned_result = (qmu_efi_capture_result_v2) { 0 };
    if (!request || request->abi_version != QMU_EFI_CAPTURE_ABI_VERSION ||
        request->struct_size != sizeof(*request)) {
        return 0;
    }
    if (request->reserved || !page || (page & (page - 1)) ||
        request->host_page_bytes != page ||
        !request->task_virtual_address || !request->frame_storage_bytes ||
        !request->native_import_alignment ||
        (request->native_import_alignment &
         (request->native_import_alignment - 1)) ||
        (request->source_kind != QMU_EFI_SOURCE_HOST_ALLOCATION &&
         request->source_kind != QMU_EFI_SOURCE_DMA_BUF)) {
        return 0;
    }
    visible_offset = request->task_virtual_address % page;
    if (request->frame_storage_bytes > UINT64_MAX - visible_offset) {
        return 0;
    }
    buffer_bytes = visible_offset + request->frame_storage_bytes;
    if ((request->source_kind == QMU_EFI_SOURCE_HOST_ALLOCATION &&
         request->native_minimum_allocation_bytes != 0) ||
        (request->source_kind == QMU_EFI_SOURCE_DMA_BUF &&
         request->native_minimum_allocation_bytes < buffer_bytes)) {
        return 0;
    }
    /* Native alignment is literal. The independent CPU alias also requires
     * actual host-page alignment; two powers of two have this exact common
     * alignment. V remains taskVA modulo PAGE, never modulo GPU alignment. */
    alignment = MAX(page, request->native_import_alignment);
    source = agfx_eeee_source_backing_capture_extent(
        s->eeee_provider_state, request->task_virtual_address,
        request->frame_storage_bytes,
        request->native_minimum_allocation_bytes, alignment);
    if (!source) {
        return 0;
    }

    result.abi_version = QMU_EFI_CAPTURE_ABI_VERSION;
    result.struct_size = sizeof(result);
    result.source_kind = request->source_kind;
    result.allocation_bytes =
        agfx_eeee_source_backing_allocation_length(source);
    result.visible_offset = agfx_eeee_source_backing_visible_offset(source);
    result.visible_bytes = agfx_eeee_source_backing_length(source);
    if (request->source_kind == QMU_EFI_SOURCE_HOST_ALLOCATION) {
        result.payload.host_allocation_base =
            agfx_eeee_source_backing_allocation(source);
        result.opaque_owner = source;
        result.release_owner = agfx_efi_host_source_release;
    } else {
        /* Keep the helper's actual stage/errno/interval in this local record
         * before cleanup. It is not a new consumer-visible error ABI, global
         * last-error cache or invented native failure. Output stays all-zero
         * on returned rejection, including actual open/ioctl failures. */
        AgfxEeeeDmaBufFailure failure = { 0 };
        AgfxEeeeSourceDmaBuf *export =
            agfx_eeee_source_dmabuf_create(source, &failure);

        if (!export) {
            agfx_eeee_source_backing_unref(source);
            return 0;
        }
        /* The export already owns this SAME physical capture. Drop only our
         * local source reference; do not consume or duplicate its borrowed FD. */
        result.payload.dma_buf.original_fd =
            agfx_eeee_source_dmabuf_fd(export);
        result.opaque_owner = export;
        result.release_owner = agfx_efi_dma_source_release;
        agfx_eeee_source_backing_unref(source);
    }
    /* Only a complete tagged record transfers one exact physical reference.
     * On rejection a zero union is NOT fd0: no DMA tag/owner was published. */
    *out_preowned_result = result;
    return 1;
}
