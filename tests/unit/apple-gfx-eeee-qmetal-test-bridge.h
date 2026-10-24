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

/* Append inside the existing test bridge extern-C block. Test glue only. */
typedef struct AgfxTestPendingParentCall AgfxTestPendingParentCall;
typedef enum AgfxTestPendingParentAction {
    AGFX_PENDING_CREATE_SESSION,
    AGFX_PENDING_RUN_PARENT_PHASE,
    AGFX_PENDING_QUIESCE_SESSION,
    AGFX_PENDING_DESTROY_SESSION
} AgfxTestPendingParentAction;
typedef enum AgfxTestPendingParentReturn {
    AGFX_PENDING_CPP_RETURNED,
    AGFX_PENDING_CPP_ESCAPE_RETAINED
} AgfxTestPendingParentReturn;
typedef void (*AgfxTestPendingParentDrive)(void *, AgfxTestPendingParentCall *,
                                           AgfxTestPendingParentAction);
AgfxTestPendingParentCall *agfx_test_pending_parent_new(
    AgfxTestOwnedContext *, uint32_t encoded_task_id, uint64_t task_length,
    uint32_t root_pfn, void *actual_root_pointer, uint32_t page_table_root_pfn);
void agfx_test_pending_parent_worker_cpp(AgfxTestPendingParentCall *,
                                         AgfxTestPendingParentAction);
AgfxTestPendingParentReturn agfx_test_pending_parent_main_cpp(
    AgfxTestPendingParentCall *, AgfxTestPendingParentAction,
    AgfxTestPendingParentDrive, void *);
qmu_session *agfx_test_pending_parent_session(const AgfxTestPendingParentCall *);
int agfx_test_pending_parent_completed(const AgfxTestPendingParentCall *);
int agfx_test_pending_parent_quiesced(const AgfxTestPendingParentCall *);
int agfx_test_pending_parent_delete_returned(AgfxTestPendingParentCall *);

#ifdef __cplusplus
}
#endif
#endif
