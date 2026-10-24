/* PRIVATE peer protocol v1; not public18 or qmu_config layout. New export
 * is intentional and requires exact matched QEMU/QMetal source pair. */
#pragma once
#include "qmu/qmetal_unified.h"
#include "apple-gfx-eeee-source-backing-caps.h"
#include "apple-gfx-eeee-source-dmabuf-caps.h"
#ifdef __cplusplus
extern "C" {
#endif
#define QMU_PRIVATE_NATIVE_EEEE_FACTORY_VERSION 1u
enum qmu_native_eeee_prepare_profile_v1 {
    QMU_NATIVE_EEEE_PREPARE_IOACCEL_CONFIG14_NATIVE_OPTIONS0=1
};
struct qmu_native_eeee_handoff_v1 {
    uint32_t version,struct_size;
    uint32_t prepare_profile; /* mandatory source branch, independent of DMA/backend caps */
    void* acquired_context;
    void (*release_context)(void*);
    qmu_extended_callbacks full18;
    struct qmu_source_backing_caps_v1 backing;
    struct qmu_source_dmabuf_caps_v1 dmabuf;
};
enum qmu_native_eeee_open_result_v1 {
    QMU_NATIVE_EEEE_HOST_SESSION_RETURNED=1,
    QMU_NATIVE_EEEE_HOST_FACTORY_RETURNED_NULL=2,
    QMU_NATIVE_EEEE_CPP_ESCAPE_RETAINED=3,
    QMU_NATIVE_EEEE_SOURCE_PROFILE_UNRETURNED=4
};
struct qmu_private_native_eeee_factory_v1 {
    uint32_t version,struct_size;
    /* Null => handoff token NOT consumed. Nonnull receipt owns it before
     * open/native; caller stores receipt before calling open. */
    void* (*prepare)(const qmu_extended_config*,const qmu_extended_callbacks*,
                     const struct qmu_native_eeee_handoff_v1*);
    uint32_t (*open)(void*);
    qmu_session* (*take_returned_host_session)(void*);
    /* False preserves SAME receipt/session/context on unknown escape. */
    bool (*dispose_returned_receipt)(void*);
};
#if defined(__GNUC__)
__attribute__((visibility("default")))
#endif
const struct qmu_private_native_eeee_factory_v1* qmu_get_private_native_eeee_factory_v1(void);
#ifdef __cplusplus
}
#endif
