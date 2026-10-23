/* Private exact saved-source DMA export capability; never the public C ABI. */
#ifndef QMU_PRIVATE_SOURCE_DMABUF_CAPS_V1_H
#define QMU_PRIVATE_SOURCE_DMABUF_CAPS_V1_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define QMU_PRIVATE_SOURCE_DMABUF_CAPS_VERSION 1u
struct qmu_source_dmabuf_result_v1 {
    void *owner;
    uint32_t failure_stage;
    int32_t error_no;
    uint32_t failed_interval;
};
struct qmu_source_dmabuf_view_v1 {
    int32_t borrowed_fd; /* fd0 is valid; never close/consume this descriptor */
    uint64_t allocation_length, visible_offset, visible_length;
};
struct qmu_source_dmabuf_caps_v1 {
    uint32_t version, struct_size;
    void *owner_context; /* same acquired context as saved capture table */
    /* Only the saved source is read; no context/state access after capture. */
    void (*create)(void *saved_source, struct qmu_source_dmabuf_result_v1 *);
    void (*release)(void *export_owner);
    bool (*describe)(const void *, struct qmu_source_dmabuf_view_v1 *);
};
#ifdef __cplusplus
}
#endif
#endif
