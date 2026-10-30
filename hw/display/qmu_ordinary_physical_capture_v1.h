/* Complete PRIVATE matched-peer protocol. Public qmu_config/full18 unchanged. */
#ifndef QMU_PRIVATE_ORDINARY_PHYSICAL_CAPTURE_V1_H
#define QMU_PRIVATE_ORDINARY_PHYSICAL_CAPTURE_V1_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct qmu_session qmu_session;
#define QMU_PRIVATE_ORDINARY_PHYSICAL_CAPTURE_VERSION 1u
struct qmu_ordinary_physical_piece_v1 {
    uint64_t gpa, logical_offset, length;
    /* Original map RETURN witness, never an address to reverse-translate. */
    const void *mapped_pointer;
};
struct qmu_ordinary_physical_request_v1 {
    uint32_t version, struct_size;
    uint32_t task_id, resource_id;
    uint64_t task_epoch, object_epoch, source_va;
    /* Every byte of allocation_length is covered by captured pieces. */
    uint64_t allocation_length, visible_offset, literal_length, native_alignment;
    uint64_t piece_count;
    const struct qmu_ordinary_physical_piece_v1 *pieces;
};
struct qmu_ordinary_physical_view_v1 {
    void *allocation;
    uint64_t allocation_length;
    void *visible_pointer;
    uint64_t visible_offset, visible_length;
};
struct qmu_ordinary_physical_handoff_v1 {
    uint32_t version, struct_size;
    /* One genuinely acquired reference, consumed only by nonnull prepare. */
    void *acquired_context;
    void *memory_ctx_identity; /* exact ordinary callbacks.user_ctx */
    bool (*stop)(void *);
    void (*release_context)(void *);
    /* Publish partial owner BEFORE first MR/FD/mapping effect. A false
     * describe leaves that SAME owner for explicit returned retirement. */
    void (*capture)(void *, const struct qmu_ordinary_physical_request_v1 *, void **);
    bool (*describe)(const void *, struct qmu_ordinary_physical_view_v1 *);
    bool (*mark_dirty)(void *, uint64_t, uint64_t);
    void (*release_span)(void *);
};
enum qmu_ordinary_physical_install_result_v1 {
    QMU_ORDINARY_PHYSICAL_INSTALLED = 1,
    QMU_ORDINARY_PHYSICAL_REJECTED = 2,
    QMU_ORDINARY_PHYSICAL_ESCAPE_RETAINED = 3,
    QMU_ORDINARY_PHYSICAL_UNRETURNED = 4
};
struct qmu_private_ordinary_physical_capture_v1 {
    uint32_t version, struct_size;
    void *(*prepare)(qmu_session *, const struct qmu_ordinary_physical_handoff_v1 *);
    uint32_t (*install)(void *);
    /* False preserves original receipt/context/exception. No retry. */
    bool (*dispose_returned)(void *);
    /* Synchronous owned-worker call: true requires actual final delete leaf
     * AND normal qmu_destroy RETURN. Null receipt covers prepare rejection;
     * negotiate this complete getter before creating a Session. False retains
     * Session/receipt/context; it is never permission to retry or free them. */
    bool (*destroy_returned)(void *, qmu_session *);
};
#if defined(__GNUC__)
__attribute__((visibility("default")))
#endif
const struct qmu_private_ordinary_physical_capture_v1 *
qmu_get_private_ordinary_physical_capture_v1(void);
#ifdef __cplusplus
}
#endif
#endif
