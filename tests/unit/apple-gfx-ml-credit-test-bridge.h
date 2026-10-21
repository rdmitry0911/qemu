/* Test-only C/C++ glue to the existing public QMetal calls. No product ABI. */
#ifndef TEST_APPLE_GFX_ML_CREDIT_BRIDGE_H
#define TEST_APPLE_GFX_ML_CREDIT_BRIDGE_H

#include "qmu/qmetal_unified.h"

#ifdef __cplusplus
extern "C" {
#endif

enum AgfxCreditFamily {
    AGFX_CREDIT_IOSFC,
    AGFX_CREDIT_TEXTURE,
};

/* Private test process capture; neither wrapper changes QMetal calls. */
int agfx_credit_exit_capture_initialize(int argc, char **argv);
int agfx_credit_exit_capture_result(int status);

/* Texture IDs and source selectors below are fixture inputs, not guest IDs. */
uint32_t agfx_credit_prepare_texture(qmu_session *session,
                                     uint32_t width, uint32_t height);
int agfx_credit_publish(qmu_session *session, enum AgfxCreditFamily family,
                        uint32_t texture, uint32_t width, uint32_t height,
                        uint32_t seed);
uint64_t agfx_credit_delivery_epoch(qmu_session *session);

#ifdef __cplusplus
}
#endif
#endif
