/* Source-private immutable SourceBacking -> owning Linux DMA FD.
 * Neither a public callback ABI nor a Vulkan/import/visibility contract. */
#ifndef HW_DISPLAY_APPLE_GFX_EEEE_SOURCE_DMABUF_H
#define HW_DISPLAY_APPLE_GFX_EEEE_SOURCE_DMABUF_H

#include "apple-gfx-eeee-process-vm.h"

typedef struct AgfxEeeeSourceDmaBuf AgfxEeeeSourceDmaBuf;

typedef enum AgfxEeeeDmaBufFailureStage {
    AGFX_DMABUF_NONE,
    AGFX_DMABUF_ALLOCATION,
    AGFX_DMABUF_SOURCE_LAYOUT,
    AGFX_DMABUF_FILE_STAT,
    AGFX_DMABUF_FILE_EXTENT,
    AGFX_DMABUF_FILE_SEALS,
    AGFX_DMABUF_SEAL_POLICY,
    AGFX_DMABUF_CONTROL_OPEN,
    AGFX_DMABUF_CREATE_LIST,
} AgfxEeeeDmaBufFailureStage;

typedef struct AgfxEeeeDmaBufFailure {
    AgfxEeeeDmaBufFailureStage stage;
    /* Actual syscall errno, or ENOMEM for allocation. Construction-only
     * rejections have zero here; they are not invented kernel errors. */
    int error_no;
    unsigned interval; /* UINT_MAX when no individual interval is selected */
} AgfxEeeeDmaBufFailure;

/* Caller owns source on entry. This function takes its own strong reference
 * before reading borrowed saved fds. It opens ONLY /dev/udmabuf once, submits
 * ONE exact CREATE_LIST, and never mutates/remaps/copies the source or seals.
 * No module-config list/size limit is assumed: the actual kernel decides.
 *
 * create/failure/final-unref may synchronously return through BQL when dropping
 * the last source. Do not hold provider/task/P0/current or any main-loop-needed
 * lock. The caller must keep every export reader alive through its actual
 * terminal/callback RETURN; an imported fd duplicate is not that custody.
 * No retry/cache/alternative device path or native import is performed. */
AgfxEeeeSourceDmaBuf *agfx_eeee_source_dmabuf_create(
    AgfxEeeeSourceBacking *source, AgfxEeeeDmaBufFailure *failure);
void agfx_eeee_source_dmabuf_ref(AgfxEeeeSourceDmaBuf *export);
void agfx_eeee_source_dmabuf_unref(AgfxEeeeSourceDmaBuf *export);
/* Borrowed, may be fd0; caller must not close/consume it. Vulkan ownership
 * transfer, if later admitted, needs a separate explicitly owned duplicate. */
int agfx_eeee_source_dmabuf_fd(const AgfxEeeeSourceDmaBuf *export);
const AgfxEeeeSourceBacking *agfx_eeee_source_dmabuf_source(
    const AgfxEeeeSourceDmaBuf *export);
uint64_t agfx_eeee_source_dmabuf_allocation_length(
    const AgfxEeeeSourceDmaBuf *export);
uint64_t agfx_eeee_source_dmabuf_visible_offset(
    const AgfxEeeeSourceDmaBuf *export);
uint64_t agfx_eeee_source_dmabuf_length(const AgfxEeeeSourceDmaBuf *export);

#endif
