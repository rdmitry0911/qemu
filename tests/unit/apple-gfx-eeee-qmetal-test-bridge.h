/* Test-only C/C++ glue. This is not a QEMU or QMetal production API. */
#ifndef TEST_APPLE_GFX_EEEE_QMETAL_BRIDGE_H
#define TEST_APPLE_GFX_EEEE_QMETAL_BRIDGE_H

#include "qmu/qmetal_unified.h"

#ifdef __cplusplus
extern "C" {
#endif

/* All actual provider/adapter owners leave before return. The caller keeps
 * the actual QEMU context, state and shared RAM alive for the entire call. */
int agfx_test_qmetal_legacy_native_window(
    const qmu_extended_callbacks *callbacks, uint64_t physical,
    uint64_t length, uint64_t offset, uint8_t expected, uint8_t written);

/* Adopts exactly one actually acquired token. The real release leaf is
 * called once only after the last facilities/descendant/provider RETURN. */
typedef struct AgfxTestOwnedContext AgfxTestOwnedContext;
AgfxTestOwnedContext *agfx_test_owned_context_adopt(
    const qmu_extended_callbacks *callbacks, void *context,
    void (*release)(void *));
int agfx_test_owned_context_drop_parent(AgfxTestOwnedContext *owner);
int agfx_test_owned_context_native_window(
    AgfxTestOwnedContext *owner, uint64_t physical, uint64_t length,
    uint64_t offset, uint8_t expected, uint8_t written, int keep_alias,
    void **actual_region, uint64_t *actual_region_offset);
void agfx_test_owned_context_destroy(AgfxTestOwnedContext *owner);

#ifdef __cplusplus
}
#endif
#endif
