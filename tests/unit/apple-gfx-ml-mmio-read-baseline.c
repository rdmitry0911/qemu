/*
 * Exact complete agfx_mmio_read from committed01d7f575, mechanically renamed
 * for a safe direct-context/cost witness. Its misleading historical comment
 * is deliberately preserved as part of that original body.
 * Not a production legacy branch or a hand-written old algorithm.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/display/apple-gfx-ml.h"
#include "qmu/qmetal_unified.h"
#include "hw/display/trace.h"

uint64_t agfx_test_baseline_mmio_read(void *, hwaddr, unsigned);

uint64_t agfx_test_baseline_mmio_read(void *opaque, hwaddr offset, unsigned size)
{
    AppleGfxMLState *s = opaque;
    uint64_t value;

    /* Reference: mmioReadAtOffset is synchronous — no serial queue, no mutex.
     * Register values are atomic (event_stamps, display_irq) or aligned
     * uint32_t written under mmio_mutex (atomic loads on x86_64).
     * Synchronous reads avoid blocking behind waitStamps on the worker queue. */
    value = qmu_mmio_read(s->qmu_dev, (uint32_t)offset, size);
    if (s->debug_level >= 5) {
        trace_apple_gfx_ml_mmio_read(offset, value, size);
    }
    return value;
}
