/* Private saved physical-backing capability. This is not the public
 * qmu_extended_callbacks ABI. Both host components use this identical header. */
#ifndef QMU_PRIVATE_SOURCE_BACKING_CAPS_V1_H
#define QMU_PRIVATE_SOURCE_BACKING_CAPS_V1_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define QMU_PRIVATE_SOURCE_BACKING_CAPS_VERSION 1u
struct qmu_source_backing_view_v1 {
    void *allocation;
    uint64_t allocation_length;
    void *visible_pointer;
    uint64_t visible_offset;
    uint64_t visible_length;
};
struct qmu_source_backing_caps_v1 {
    uint32_t version;
    uint32_t struct_size;
    /* Borrowed exact acquired NativeContext; caller retains its actual lease
     * through capture RETURN/escape. Saved backing itself outlives Context. */
    void *owner_context;
    void (*capture)(void *, uint64_t, uint64_t, uint64_t, void **);
    void (*release)(void *);
    bool (*describe)(const void *, struct qmu_source_backing_view_v1 *);
    /* Must be called after the real GPU visibility/fence edge. This marks
     * saved original RAM only; it neither waits nor establishes visibility. */
    bool (*mark_dirty)(void *, uint64_t, uint64_t);
};
#ifdef __cplusplus
}
#endif
#endif
