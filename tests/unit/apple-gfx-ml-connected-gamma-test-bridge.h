/* Test-only calls into the matched normal QMetal DSO. No product ABI. */
#ifndef TEST_APPLE_GFX_ML_CONNECTED_GAMMA_BRIDGE_H
#define TEST_APPLE_GFX_ML_CONNECTED_GAMMA_BRIDGE_H

#include "qmu/qmetal_unified.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t agfx_connected_gamma_define_task(void *ring, uint32_t root_pfn);
int agfx_connected_gamma_task_state(qmu_session *session, int present,
    uint32_t root_pfn, uint64_t address_space_size, uint64_t *epoch);
int agfx_connected_gamma_update(qmu_session *session, uint64_t table_va,
    uint64_t mapped_length, uint32_t entries, uint32_t sum);

#ifdef __cplusplus
}
#endif
#endif
