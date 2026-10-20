/* Source-private test bridge, not a QEMU/QMetal runtime callback ABI. */
#ifndef APPLE_GFX_EEEE_DMA_IMPORT_TEST_BRIDGE_H
#define APPLE_GFX_EEEE_DMA_IMPORT_TEST_BRIDGE_H
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct AgfxNativeDmaOwner AgfxNativeDmaOwner;
typedef struct AgfxNativeDmaRequirements {
    uint64_t buffer_bytes, requirements_bytes, alignment;
    uint32_t memory_type_bits;
    bool dedicated;
} AgfxNativeDmaRequirements;
AgfxNativeDmaOwner *agfx_test_dma_prepare(uint64_t V, uint64_t L,
                                        AgfxNativeDmaRequirements *out);
/* Consumes the test owner. Original export/source relation is adopted by the
 * production primitive; release runs exactly once after native retirement.
 * Caller holds no BQL/provider/task lock. No GPU submission/read is performed. */
void agfx_test_dma_import_retire(AgfxNativeDmaOwner *owner, int fd,
    uint64_t A, uint64_t V, uint64_t L, void *source, void (*release)(void *));
#ifdef __cplusplus
}
#endif
#endif
