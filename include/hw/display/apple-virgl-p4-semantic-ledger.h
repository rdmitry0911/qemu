/*
 * AppleVirgl P4 owner-to-CPU-surface semantic-ledger capture
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_APPLE_VIRGL_P4_SEMANTIC_LEDGER_H
#define HW_DISPLAY_APPLE_VIRGL_P4_SEMANTIC_LEDGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * This is intentionally a host-only, opt-in diagnostic.  QMetal provides a
 * nonzero immutable semantic-ledger id with a frame completion.  QEMU carries
 * that scalar through its existing owner-completion FIFO and records the two
 * already-owned packed BGRA images only after the CPU DisplaySurface copy has
 * completed.  It never calls back into QMetal and it never participates in
 * frame-pump retirement.
 */
#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_DEFAULT_CAP 32u
/* Bounded failure/duplicate receipts are separate from packed raw captures. */
#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_RECEIPT_CAP 256u

/*
 * P4 accepts any packed BGRA surface up to the one reviewed 1920x1080
 * measurement mode.  The complete owner/CPU staging for these slots is
 * allocated at ledger initialization, before QMetal workers start.
 */
#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_MAX_WIDTH 1920u
#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_MAX_HEIGHT 1080u
#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_CAPTURE_SLOTS 2u

/*
 * A QEMU-owned deep copy of qmu_p4_owner_backing_identity.  Keep this as
 * scalars rather than retaining the QMetal callback object: QMetal releases
 * the completion as soon as render_frame_complete returns.
 */
typedef struct AppleVirglP4OwnerBackingIdentity {
    uint64_t owner_object_id;
    uint64_t owner_image_id;
    uint64_t owner_generation;
    uint64_t backing_id;
    uint64_t backing_generation;
    uint64_t guest_va;
    uint64_t physical_image_id;
    uint64_t physical_allocation_id;
    uint32_t physical_width;
    uint32_t physical_height;
    uint32_t physical_pixel_format;
    uint32_t physical_row_bytes;
} AppleVirglP4OwnerBackingIdentity;

typedef struct AppleVirglP4AsyncState AppleVirglP4AsyncState;

typedef struct AppleVirglP4SemanticLedger {
    bool enabled;
    bool finalized;
    bool write_error;
    int dirfd;
    uint32_t cap;
    uint32_t unique_tags;
    uint32_t receipt_count;
    uint64_t capture_sequence;
    uint64_t capture_count;
    uint64_t artifact_sequence;
    uint64_t temporary_sequence;
    uint64_t raw_capture_loss_count;
    uint64_t receipt_loss_count;
    bool raw_capture_saturated;
    bool raw_capture_saturation_emitted;
    bool receipt_saturated;
    bool receipt_saturation_emitted;
    AppleVirglP4AsyncState *async;
#ifdef APPLE_VIRGL_P4_UNIT_TEST
    /* Test-only post-rename directory-fsync fault injection. */
    bool test_fail_final_summary_dirsync_once;
#endif
} AppleVirglP4SemanticLedger;

/*
 * Enable only when APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR names a
 * pre-existing,
 * absolute, owner-private directory.  No directory is created implicitly.
 * The process also must explicitly set APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER=1.
 */
void apple_virgl_p4_semantic_ledger_init(AppleVirglP4SemanticLedger *ledger);
void apple_virgl_p4_semantic_ledger_destroy(
    AppleVirglP4SemanticLedger *ledger);

/*
 * Record a completed CPU-surface application.  @owner_pixels is the already
 * packed stage payload; @cpu_pixels has @cpu_stride bytes per DisplaySurface
 * row.  The caller copies exactly width * 4 bytes per row into fixed staging;
 * a bounded writer compares, hashes, and persists it after the caller returns.
 * It is a best-effort diagnostic and never changes the caller's result.
 */
void apple_virgl_p4_semantic_ledger_record_applied(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const uint8_t *owner_pixels, uint32_t width, uint32_t height,
    uint32_t packed_stride, size_t packed_bytes, const uint8_t *cpu_pixels,
    uint32_t cpu_stride,
    const AppleVirglP4OwnerBackingIdentity *owner_backing);

/* Record a nonzero tag whose QEMU CPU-surface application was rejected. */
void apple_virgl_p4_semantic_ledger_record_apply_failure(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing,
    const char *reason);

#ifdef APPLE_VIRGL_P4_UNIT_TEST
/* This symbol and state do not exist in production QEMU builds. */
void apple_virgl_p4_semantic_ledger_test_fail_final_summary_dirsync_once(
    AppleVirglP4SemanticLedger *ledger);
#endif

#endif /* HW_DISPLAY_APPLE_VIRGL_P4_SEMANTIC_LEDGER_H */
