/* Test-only public-call glue and borrowed identity observations. */
#ifndef TEST_APPLE_GFX_ML_IMPORTED_DIRTY_BRIDGE_H
#define TEST_APPLE_GFX_ML_IMPORTED_DIRTY_BRIDGE_H
#include "qmu/qmetal_unified.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct AgfxImportLayout {
    uint64_t sparse, host_alignment;
    uint32_t memory_count, memory_flags[32];
} AgfxImportLayout;
typedef struct AgfxImportIdentity {
    uint32_t id;
    uint64_t object, native_buffer, generation, runtime_generation;
} AgfxImportIdentity;
int agfx_import_layout(qmu_session *, AgfxImportLayout *);
int agfx_import_prepare(qmu_session *, uint64_t, uint64_t, uint64_t,
                        AgfxImportIdentity *);
int agfx_import_fill(qmu_session *, uint32_t);
int agfx_import_generation_expired(void);
#ifdef __cplusplus
}
#endif
#endif
