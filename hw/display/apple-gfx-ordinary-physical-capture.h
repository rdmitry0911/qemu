/* Independent ordinary physical capture. No EEEE provider publication. */
#ifndef APPLE_GFX_ORDINARY_PHYSICAL_CAPTURE_H
#define APPLE_GFX_ORDINARY_PHYSICAL_CAPTURE_H
#include "qmu_ordinary_physical_capture_v2.h"
#include "qmu_ordinary_physical_comparison_v1.h"
typedef struct AgfxOrdinaryPhysicalContext AgfxOrdinaryPhysicalContext;
typedef struct AgfxOrdinaryPhysicalOrigin AgfxOrdinaryPhysicalOrigin;
AgfxOrdinaryPhysicalOrigin *agfx_ordinary_physical_origin_new(const void *);
AgfxOrdinaryPhysicalOrigin *agfx_ordinary_physical_origin_ref(AgfxOrdinaryPhysicalOrigin *);
void agfx_ordinary_physical_origin_unref(AgfxOrdinaryPhysicalOrigin *);
void agfx_ordinary_physical_origin_stop(AgfxOrdinaryPhysicalOrigin *);
AgfxOrdinaryPhysicalOrigin *agfx_ordinary_physical_context_origin(void *);
struct AppleGfxMLState;
AgfxOrdinaryPhysicalContext *agfx_ordinary_physical_context_create(struct AppleGfxMLState *);
bool agfx_ordinary_physical_context_acquire(AgfxOrdinaryPhysicalContext *,
    struct qmu_ordinary_physical_handoff_v2 *);
bool agfx_ordinary_physical_context_stop(void *);
void agfx_ordinary_physical_context_release(void *);
void agfx_ordinary_physical_context_retire(AgfxOrdinaryPhysicalContext *);
/* Begin/end only count the already-acquired synchronous call; no BQL/AIO. */
bool agfx_ordinary_physical_capture_enter(void *);
void agfx_ordinary_physical_capture_leave(void *);
void agfx_ordinary_physical_capture(void *,
    const struct qmu_ordinary_physical_request_v2 *, void **);
bool agfx_ordinary_physical_describe(const void *, struct qmu_ordinary_physical_view_v2 *);
bool agfx_ordinary_physical_dirty(void *, uint64_t, uint64_t);
void agfx_ordinary_physical_release_span(void *);
uint32_t agfx_ordinary_physical_compare(const void *,
    const void *, uint64_t, uint64_t, const void *, uint64_t, uint64_t);
#endif
