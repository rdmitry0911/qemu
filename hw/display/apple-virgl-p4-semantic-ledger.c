/*
 * AppleVirgl P4 owner-to-CPU-surface semantic-ledger capture
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/thread.h"
#include "hw/display/apple-virgl-p4-semantic-ledger.h"

#include <inttypes.h>

#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_ENABLE_ENV \
    "APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER"
#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_DIR_ENV \
    "APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR"
#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_SCHEMA \
    "apple-virgl.p4-qemu-scanout/v1"
#define APPLE_VIRGL_P4_FINAL_SUMMARY_NAME "qemu-ledger-summary.json"
/*
 * This guard is deliberately a hidden direct child.  The host validator
 * rejects hidden entries, so a failed terminal publication cannot be mistaken
 * for a complete QEMU tail even if a filesystem exposes a stale summary after
 * a failed directory sync.
 */
#define APPLE_VIRGL_P4_FINALIZATION_GUARD_NAME ".p4-finalization-pending"
#define APPLE_VIRGL_P4_CAPTURE_BYTES \
    ((size_t)APPLE_VIRGL_P4_SEMANTIC_LEDGER_MAX_WIDTH * \
     APPLE_VIRGL_P4_SEMANTIC_LEDGER_MAX_HEIGHT * 4u)
#define APPLE_VIRGL_P4_WORK_CAP \
    (APPLE_VIRGL_P4_SEMANTIC_LEDGER_RECEIPT_CAP + \
     APPLE_VIRGL_P4_SEMANTIC_LEDGER_CAPTURE_SLOTS + 2u)

typedef enum AppleVirglP4Reservation {
    APPLE_VIRGL_P4_RESERVATION_NONE,
    APPLE_VIRGL_P4_RESERVATION_NEW,
    APPLE_VIRGL_P4_RESERVATION_DUPLICATE,
    APPLE_VIRGL_P4_RESERVATION_SATURATED,
} AppleVirglP4Reservation;

typedef enum AppleVirglP4ReceiptResult {
    APPLE_VIRGL_P4_RECEIPT_WRITTEN,
    APPLE_VIRGL_P4_RECEIPT_SATURATED,
    APPLE_VIRGL_P4_RECEIPT_ERROR,
} AppleVirglP4ReceiptResult;

typedef enum AppleVirglP4WorkKind {
    APPLE_VIRGL_P4_WORK_CAPTURE,
    APPLE_VIRGL_P4_WORK_STATUS,
} AppleVirglP4WorkKind;

typedef struct AppleVirglP4CaptureSlot {
    bool in_use;
    uint8_t *owner_pixels;
    uint8_t *cpu_pixels;
    uint64_t ledger_id;
    uint32_t width;
    uint32_t height;
    uint32_t packed_stride;
    uint32_t cpu_stride;
    size_t packed_bytes;
    AppleVirglP4OwnerBackingIdentity owner_backing;
} AppleVirglP4CaptureSlot;

typedef struct AppleVirglP4WorkItem {
    AppleVirglP4WorkKind kind;
    uint32_t capture_slot;
    uint64_t ledger_id;
    AppleVirglP4OwnerBackingIdentity owner_backing;
    const char *status;
    const char *reason;
    bool saturated;
    bool loss;
} AppleVirglP4WorkItem;

struct AppleVirglP4AsyncState {
    QemuMutex lock;
    QemuSemaphore work_sem;
    QemuThread worker;
    bool synchronization_initialized;
    bool worker_started;
    bool accepting;
    bool stopping;
    uint64_t seen_tags[APPLE_VIRGL_P4_SEMANTIC_LEDGER_DEFAULT_CAP];
    uint32_t seen_tag_count;
    AppleVirglP4CaptureSlot
        capture_slots[APPLE_VIRGL_P4_SEMANTIC_LEDGER_CAPTURE_SLOTS];
    AppleVirglP4WorkItem work[APPLE_VIRGL_P4_WORK_CAP];
    uint32_t work_head;
    uint32_t work_tail;
    uint32_t work_count;
};

static bool apple_virgl_p4_path_is_valid(const char *path)
{
    const char *component;

    if (!path || path[0] != '/' || strlen(path) >= PATH_MAX) {
        return false;
    }

    component = path + 1;
    while (*component) {
        const char *end = strchr(component, '/');
        size_t length = end ? (size_t)(end - component) : strlen(component);

        if (length == 0 || (length == 1 && component[0] == '.') ||
            (length == 2 && component[0] == '.' && component[1] == '.')) {
            return false;
        }
        if (!end) {
            break;
        }
        component = end + 1;
    }
    return true;
}

static void apple_virgl_p4_log_write_failure(uint64_t ledger_id,
                                              const char *what)
{
    error_report("apple-virgl P4 semantic-ledger tag %016" PRIx64
                 ": %s", ledger_id, what);
}

static const char *apple_virgl_p4_apply_failure_reason(const char *reason)
{
    /* Keep externally supplied text out of a JSON artifact and pathname. */
    static const char *const allowed[] = {
        "cpu_surface_apply_failed",
        "present_stage_enqueue_rejected",
        "present_stage_quiesced",
    };
    size_t index;

    for (index = 0; index < ARRAY_SIZE(allowed); ++index) {
        if (reason && strcmp(reason, allowed[index]) == 0) {
            /* The queued writer may outlive the caller's string storage. */
            return allowed[index];
        }
    }
    return "cpu_surface_apply_failed";
}

static bool apple_virgl_p4_write_all(int fd, const void *data, size_t length)
{
    const uint8_t *cursor = data;

    while (length) {
        ssize_t written = write(fd, cursor, length);

        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (written == 0) {
            return false;
        }
        cursor += written;
        length -= written;
    }
    return true;
}

static bool apple_virgl_p4_write_file_at(int dirfd, const char *name,
                                          const void *data, size_t length)
{
    int fd;
    bool result;

    fd = openat(dirfd, name, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC |
                O_NOFOLLOW, 0600);
    if (fd < 0) {
        return false;
    }
    result = apple_virgl_p4_write_all(fd, data, length) && fsync(fd) == 0;
    if (close(fd) < 0) {
        result = false;
    }
    return result;
}

static void apple_virgl_p4_remove_capture_dir(int rootfd,
                                               const char *temporary_name)
{
    int temporary_fd;

    temporary_fd = openat(rootfd, temporary_name,
                          O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (temporary_fd >= 0) {
        (void)unlinkat(temporary_fd, "owner-packed.bgra", 0);
        (void)unlinkat(temporary_fd, "cpu-surface-packed.bgra", 0);
        (void)unlinkat(temporary_fd, "metadata.json", 0);
        close(temporary_fd);
    }
    (void)unlinkat(rootfd, temporary_name, AT_REMOVEDIR);
}

static bool apple_virgl_p4_fsync_directory(
    AppleVirglP4SemanticLedger *ledger, const char *name)
{
#ifdef APPLE_VIRGL_P4_UNIT_TEST
    if (ledger->test_fail_final_summary_dirsync_once &&
        strcmp(name, APPLE_VIRGL_P4_FINAL_SUMMARY_NAME) == 0) {
        ledger->test_fail_final_summary_dirsync_once = false;
        errno = EIO;
        return false;
    }
#endif
    return fsync(ledger->dirfd) == 0;
}

static bool apple_virgl_p4_atomic_write_file(AppleVirglP4SemanticLedger *ledger,
                                              const char *name,
                                              const void *data, size_t length)
{
    char temporary_name[NAME_MAX + 1];
    int attempt;

    if (fstatat(ledger->dirfd, name, &(struct stat) { 0 },
                AT_SYMLINK_NOFOLLOW) == 0 || errno != ENOENT) {
        return false;
    }

    for (attempt = 0; attempt < 16; ++attempt) {
        int fd;
        bool result;

        if (snprintf(temporary_name, sizeof(temporary_name),
                     ".%s.tmp-%ld-%" PRIu64, name, (long)getpid(),
                     ++ledger->temporary_sequence) >=
            (int)sizeof(temporary_name)) {
            return false;
        }
        fd = openat(ledger->dirfd, temporary_name,
                    O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                    0600);
        if (fd < 0) {
            if (errno == EEXIST) {
                continue;
            }
            return false;
        }
        result = apple_virgl_p4_write_all(fd, data, length) && fsync(fd) == 0;
        if (close(fd) < 0) {
            result = false;
        }
        if (!result) {
            (void)unlinkat(ledger->dirfd, temporary_name, 0);
            return false;
        }
        if (renameat(ledger->dirfd, temporary_name, ledger->dirfd, name) < 0) {
            (void)unlinkat(ledger->dirfd, temporary_name, 0);
            return false;
        }
        return apple_virgl_p4_fsync_directory(ledger, name);
    }
    return false;
}

/*
 * A final summary is an acceptance token, not merely an informational file.
 * If its atomic publication reports failure after rename, remove the token
 * before returning.  The finalization guard remains armed on every failure,
 * so an unlink/fsync failure cannot leave a plausible-looking terminal set.
 */
static bool apple_virgl_p4_revoke_final_summary(
    AppleVirglP4SemanticLedger *ledger)
{
    if (!ledger || ledger->dirfd < 0) {
        return false;
    }
    if (unlinkat(ledger->dirfd, APPLE_VIRGL_P4_FINAL_SUMMARY_NAME, 0) < 0 &&
        errno != ENOENT) {
        return false;
    }
    return fsync(ledger->dirfd) == 0;
}

static bool apple_virgl_p4_arm_finalization_guard(
    AppleVirglP4SemanticLedger *ledger)
{
    static const char guard[] = "apple-virgl P4 finalization pending\n";

    return apple_virgl_p4_atomic_write_file(
        ledger, APPLE_VIRGL_P4_FINALIZATION_GUARD_NAME, guard,
        sizeof(guard) - 1u);
}

static bool apple_virgl_p4_disarm_finalization_guard(
    AppleVirglP4SemanticLedger *ledger)
{
    if (unlinkat(ledger->dirfd, APPLE_VIRGL_P4_FINALIZATION_GUARD_NAME, 0) <
        0) {
        return false;
    }
    return fsync(ledger->dirfd) == 0;
}

static void apple_virgl_p4_note_write_error(AppleVirglP4SemanticLedger *ledger)
{
    if (ledger) {
        ledger->write_error = true;
    }
}

/*
 * Called with ledger->async->lock held.  The fixed ring is sized for every
 * bounded receipt, both one-shot saturation receipts, and every raw slot.
 */
static bool apple_virgl_p4_queue_work_locked(
    AppleVirglP4SemanticLedger *ledger, const AppleVirglP4WorkItem *item)
{
    AppleVirglP4AsyncState *async = ledger ? ledger->async : NULL;

    if (!async || !async->worker_started ||
        async->work_count >= APPLE_VIRGL_P4_WORK_CAP) {
        apple_virgl_p4_note_write_error(ledger);
        return false;
    }
    async->work[async->work_tail] = *item;
    async->work_tail = (async->work_tail + 1u) % APPLE_VIRGL_P4_WORK_CAP;
    async->work_count++;
    qemu_sem_post(&async->work_sem);
    return true;
}

static bool apple_virgl_p4_queue_status_unbounded_locked(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing,
    const char *status, const char *reason, bool saturated, bool loss)
{
    AppleVirglP4WorkItem item = {
        .kind = APPLE_VIRGL_P4_WORK_STATUS,
        .ledger_id = ledger_id,
        .status = status,
        .reason = reason,
        .saturated = saturated,
        .loss = loss,
    };

    if (owner_backing) {
        item.owner_backing = *owner_backing;
    }
    return apple_virgl_p4_queue_work_locked(ledger, &item);
}

/* This writer does not consume the bounded receipt budget. */
static bool apple_virgl_p4_emit_status_unbounded(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing,
    const char *status, const char *reason, bool saturated, bool loss,
    bool write_error)
{
    char filename[NAME_MAX + 1];
    char json[4096];
    AppleVirglP4OwnerBackingIdentity zero_identity = { 0 };
    const char *state;
    bool duplicate;
    int json_length;
    int attempt;

    for (attempt = 0; attempt < 16; ++attempt) {
        uint64_t sequence = ++ledger->artifact_sequence;
        int name_length;

        name_length = snprintf(filename, sizeof(filename),
                               "p4-%016" PRIx64 "-%s-%" PRIu64 ".json",
                               ledger_id, status, sequence);
        if (name_length < 0 || name_length >= (int)sizeof(filename)) {
            return false;
        }
        if (fstatat(ledger->dirfd, filename, &(struct stat) { 0 },
                    AT_SYMLINK_NOFOLLOW) < 0 && errno == ENOENT) {
            break;
        }
    }
    if (attempt == 16) {
        return false;
    }

    if (!owner_backing) {
        owner_backing = &zero_identity;
    }
    duplicate = strcmp(status, "duplicate") == 0;
    state = duplicate ? "qemu-duplicate" :
            strcmp(status, "failure") == 0 ? "qemu-capture-failed" :
            strcmp(status, "raw-capture-saturated") == 0
                ? "qemu-raw-capture-saturated"
                : "qemu-receipt-saturated";
    json_length = snprintf(
        json, sizeof(json),
        "{\n"
        "  \"schema\": \"%s\",\n"
        "  \"state\": \"%s\",\n"
        "  \"p4_ledger_id\": %" PRIu64 ",\n"
        "  \"p4_ledger_id_hex\": \"%016" PRIx64 "\",\n"
        "  \"qemu_capture_seq\": 0,\n"
        "  \"status\": \"%s\",\n"
        "  \"reason\": \"%s\",\n"
        "  \"owner_backing_identity\": {\n"
        "    \"owner_object_id\": %" PRIu64 ",\n"
        "    \"owner_image_id\": %" PRIu64 ",\n"
        "    \"owner_generation\": %" PRIu64 ",\n"
        "    \"backing_id\": %" PRIu64 ",\n"
        "    \"backing_generation\": %" PRIu64 ",\n"
        "    \"guest_va\": %" PRIu64 ",\n"
        "    \"physical_image_id\": %" PRIu64 ",\n"
        "    \"physical_allocation_id\": %" PRIu64 ",\n"
        "    \"physical_width\": %u,\n"
        "    \"physical_height\": %u,\n"
        "    \"physical_pixel_format\": %u,\n"
        "    \"physical_row_bytes\": %u\n"
        "  },\n"
        "  \"bounded\": {\n"
        "    \"cap\": %u,\n"
        "    \"receipt_cap\": %u,\n"
        "    \"saturated\": %s,\n"
        "    \"loss\": %s,\n"
        "    \"duplicate\": %s,\n"
        "    \"write_error\": %s\n"
        "  }\n"
        "}\n",
        APPLE_VIRGL_P4_SEMANTIC_LEDGER_SCHEMA, state, ledger_id, ledger_id,
        status, reason,
        owner_backing->owner_object_id, owner_backing->owner_image_id,
        owner_backing->owner_generation, owner_backing->backing_id,
        owner_backing->backing_generation, owner_backing->guest_va,
        owner_backing->physical_image_id,
        owner_backing->physical_allocation_id,
        owner_backing->physical_width, owner_backing->physical_height,
        owner_backing->physical_pixel_format,
        owner_backing->physical_row_bytes, ledger->cap,
        APPLE_VIRGL_P4_SEMANTIC_LEDGER_RECEIPT_CAP,
        saturated ? "true" : "false", loss ? "true" : "false",
        duplicate ? "true" : "false",
        write_error ? "true" : "false");
    if (json_length < 0 || json_length >= (int)sizeof(json)) {
        return false;
    }
    return apple_virgl_p4_atomic_write_file(ledger, filename, json,
                                             (size_t)json_length);
}

static void apple_virgl_p4_emit_raw_capture_saturation(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing, const char *reason)
{
    if (ledger->raw_capture_saturation_emitted) {
        return;
    }
    ledger->raw_capture_saturation_emitted = true;
    if (!apple_virgl_p4_queue_status_unbounded_locked(
            ledger, ledger_id, owner_backing, "raw-capture-saturated", reason,
            true, true)) {
        apple_virgl_p4_log_write_failure(
            ledger_id, "could not queue raw-capture saturation receipt");
    }
}

static void apple_virgl_p4_emit_receipt_saturation(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing)
{
    if (ledger->receipt_saturation_emitted) {
        return;
    }
    ledger->receipt_saturation_emitted = true;
    if (!apple_virgl_p4_queue_status_unbounded_locked(
            ledger, ledger_id, owner_backing, "receipt-saturated",
            "receipt_budget_exhausted", true, true)) {
        apple_virgl_p4_log_write_failure(
            ledger_id, "could not queue receipt saturation receipt");
    }
}

static bool apple_virgl_p4_reserve_receipt(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing)
{
    if (ledger->receipt_count < APPLE_VIRGL_P4_SEMANTIC_LEDGER_RECEIPT_CAP) {
        ledger->receipt_count++;
        return true;
    }
    ledger->receipt_saturated = true;
    ledger->receipt_loss_count++;
    apple_virgl_p4_emit_receipt_saturation(ledger, ledger_id, owner_backing);
    return false;
}

static AppleVirglP4ReceiptResult apple_virgl_p4_emit_status(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing,
    const char *status, const char *reason)
{
    if (!apple_virgl_p4_reserve_receipt(ledger, ledger_id, owner_backing)) {
        return APPLE_VIRGL_P4_RECEIPT_SATURATED;
    }
    if (apple_virgl_p4_queue_status_unbounded_locked(
            ledger, ledger_id, owner_backing, status, reason, false, false)) {
        return APPLE_VIRGL_P4_RECEIPT_WRITTEN;
    }
    return APPLE_VIRGL_P4_RECEIPT_ERROR;
}

static AppleVirglP4Reservation
apple_virgl_p4_reserve(AppleVirglP4SemanticLedger *ledger,
                       uint64_t ledger_id,
                       const AppleVirglP4OwnerBackingIdentity *owner_backing)
{
    AppleVirglP4AsyncState *async;
    uint32_t index;

    if (!ledger || !ledger->enabled || ledger_id == 0) {
        return APPLE_VIRGL_P4_RESERVATION_NONE;
    }
    async = ledger->async;
    if (!async || !async->accepting) {
        return APPLE_VIRGL_P4_RESERVATION_NONE;
    }
    for (index = 0; index < async->seen_tag_count; ++index) {
        if (async->seen_tags[index] == ledger_id) {
            return APPLE_VIRGL_P4_RESERVATION_DUPLICATE;
        }
    }
    if (ledger->unique_tags >= ledger->cap) {
        ledger->raw_capture_saturated = true;
        ledger->raw_capture_loss_count++;
        apple_virgl_p4_emit_raw_capture_saturation(
            ledger, ledger_id, owner_backing, "raw_capture_capacity_exhausted");
        if (apple_virgl_p4_emit_status(ledger, ledger_id, owner_backing,
                                       "failure",
                                       "raw_capture_capacity_exhausted") ==
            APPLE_VIRGL_P4_RECEIPT_ERROR) {
            apple_virgl_p4_log_write_failure(
                ledger_id, "could not persist raw-capacity failure receipt");
        }
        return APPLE_VIRGL_P4_RESERVATION_SATURATED;
    }
    if (async->seen_tag_count >= ARRAY_SIZE(async->seen_tags)) {
        ledger->raw_capture_saturated = true;
        ledger->raw_capture_loss_count++;
        apple_virgl_p4_note_write_error(ledger);
        apple_virgl_p4_emit_raw_capture_saturation(
            ledger, ledger_id, owner_backing,
            "raw_capture_tag_reservation_failed");
        if (apple_virgl_p4_emit_status(ledger, ledger_id, owner_backing,
                                       "failure",
                                       "raw_capture_tag_reservation_failed") ==
            APPLE_VIRGL_P4_RECEIPT_ERROR) {
            apple_virgl_p4_log_write_failure(
                ledger_id, "could not persist raw-reservation failure receipt");
        }
        apple_virgl_p4_log_write_failure(ledger_id,
                                         "could not reserve ledger tag");
        return APPLE_VIRGL_P4_RESERVATION_SATURATED;
    }
    async->seen_tags[async->seen_tag_count++] = ledger_id;
    ledger->unique_tags++;
    return APPLE_VIRGL_P4_RESERVATION_NEW;
}

static void apple_virgl_p4_report_duplicate(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing,
    const char *reason)
{
    if (!ledger || !ledger->enabled) {
        return;
    }
    if (apple_virgl_p4_emit_status(ledger, ledger_id, owner_backing,
                                   "duplicate", reason) ==
        APPLE_VIRGL_P4_RECEIPT_ERROR) {
        apple_virgl_p4_log_write_failure(ledger_id,
                                         "could not persist duplicate receipt");
    }
}

static bool apple_virgl_p4_valid_geometry(uint32_t width, uint32_t height,
                                          uint32_t packed_stride,
                                          size_t packed_bytes,
                                          uint32_t cpu_stride)
{
    uint64_t expected_bytes;

    if (width == 0 || height == 0 ||
        width > APPLE_VIRGL_P4_SEMANTIC_LEDGER_MAX_WIDTH ||
        height > APPLE_VIRGL_P4_SEMANTIC_LEDGER_MAX_HEIGHT ||
        width > UINT32_MAX / 4u ||
        packed_stride != width * 4u || cpu_stride < packed_stride) {
        return false;
    }
    expected_bytes = (uint64_t)packed_stride * height;
    return expected_bytes <= APPLE_VIRGL_P4_CAPTURE_BYTES &&
           packed_bytes == (size_t)expected_bytes;
}

static bool apple_virgl_p4_write_capture(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const uint8_t *owner_pixels, const uint8_t *cpu_packed,
    uint32_t width, uint32_t height, uint32_t packed_stride,
    uint32_t cpu_stride, size_t packed_bytes, bool equal,
    const AppleVirglP4OwnerBackingIdentity *owner_backing)
{
    char final_name[NAME_MAX + 1];
    char temporary_name[NAME_MAX + 1];
    char metadata[4096];
    char *owner_sha256 = NULL;
    char *cpu_sha256 = NULL;
    int temporary_fd = -1;
    int metadata_length;
    int attempt;
    bool result = false;
    uint64_t capture_sequence;

    if (snprintf(final_name, sizeof(final_name), "p4-%016" PRIx64,
                 ledger_id) >= (int)sizeof(final_name)) {
        return false;
    }
    if (fstatat(ledger->dirfd, final_name, &(struct stat) { 0 },
                AT_SYMLINK_NOFOLLOW) == 0 || errno != ENOENT) {
        return false;
    }

    for (attempt = 0; attempt < 16; ++attempt) {
        if (snprintf(temporary_name, sizeof(temporary_name),
                     ".%s.tmp-%ld-%" PRIu64, final_name, (long)getpid(),
                     ++ledger->temporary_sequence) >=
            (int)sizeof(temporary_name)) {
            return false;
        }
        if (mkdirat(ledger->dirfd, temporary_name, 0700) == 0) {
            break;
        }
        if (errno != EEXIST) {
            return false;
        }
    }
    if (attempt == 16) {
        return false;
    }

    temporary_fd = openat(ledger->dirfd, temporary_name,
                          O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (temporary_fd < 0 ||
        !apple_virgl_p4_write_file_at(temporary_fd, "owner-packed.bgra",
                                       owner_pixels, packed_bytes) ||
        !apple_virgl_p4_write_file_at(temporary_fd, "cpu-surface-packed.bgra",
                                       cpu_packed, packed_bytes)) {
        goto out;
    }

    owner_sha256 = g_compute_checksum_for_data(G_CHECKSUM_SHA256,
                                               owner_pixels, packed_bytes);
    cpu_sha256 = g_compute_checksum_for_data(G_CHECKSUM_SHA256, cpu_packed,
                                             packed_bytes);
    if (!owner_sha256 || !cpu_sha256) {
        goto out;
    }
    capture_sequence = ++ledger->capture_sequence;
    metadata_length = snprintf(
        metadata, sizeof(metadata),
        "{\n"
        "  \"schema\": \"%s\",\n"
        "  \"state\": \"qemu-captured\",\n"
        "  \"p4_ledger_id\": %" PRIu64 ",\n"
        "  \"p4_ledger_id_hex\": \"%016" PRIx64 "\",\n"
        "  \"qemu_capture_seq\": %" PRIu64 ",\n"
        "  \"status\": \"%s\",\n"
        "  \"owner_cpu_equal\": %s,\n"
        "  \"owner_backing_identity\": {\n"
        "    \"owner_object_id\": %" PRIu64 ",\n"
        "    \"owner_image_id\": %" PRIu64 ",\n"
        "    \"owner_generation\": %" PRIu64 ",\n"
        "    \"backing_id\": %" PRIu64 ",\n"
        "    \"backing_generation\": %" PRIu64 ",\n"
        "    \"guest_va\": %" PRIu64 ",\n"
        "    \"physical_image_id\": %" PRIu64 ",\n"
        "    \"physical_allocation_id\": %" PRIu64 ",\n"
        "    \"physical_width\": %u,\n"
        "    \"physical_height\": %u,\n"
        "    \"physical_pixel_format\": %u,\n"
        "    \"physical_row_bytes\": %u\n"
        "  },\n"
        "  \"bounded\": {\n"
        "    \"cap\": %u,\n"
        "    \"receipt_cap\": %u,\n"
        "    \"saturated\": false,\n"
        "    \"loss\": false,\n"
        "    \"duplicate\": false,\n"
        "    \"write_error\": false\n"
        "  },\n"
        "  \"display_surface\": {\n"
        "    \"width\": %u,\n"
        "    \"height\": %u,\n"
        "    \"format\": \"BGRA8888\",\n"
        "    \"packed_row_bytes\": %u,\n"
        "    \"surface_row_bytes\": %u\n"
        "  },\n"
        "  \"owner_raw\": {\n"
        "    \"path\": \"owner-packed.bgra\",\n"
        "    \"sha256\": \"%s\",\n"
        "    \"bytes\": %zu,\n"
        "    \"row_bytes\": %u\n"
        "  },\n"
        "  \"raw\": {\n"
        "    \"path\": \"cpu-surface-packed.bgra\",\n"
        "    \"sha256\": \"%s\",\n"
        "    \"bytes\": %zu,\n"
        "    \"row_bytes\": %u\n"
        "  }\n"
        "}\n",
        APPLE_VIRGL_P4_SEMANTIC_LEDGER_SCHEMA, ledger_id, ledger_id,
        capture_sequence, equal ? "applied" : "owner_cpu_mismatch",
        equal ? "true" : "false", owner_backing->owner_object_id,
        owner_backing->owner_image_id, owner_backing->owner_generation,
        owner_backing->backing_id, owner_backing->backing_generation,
        owner_backing->guest_va, owner_backing->physical_image_id,
        owner_backing->physical_allocation_id,
        owner_backing->physical_width, owner_backing->physical_height,
        owner_backing->physical_pixel_format,
        owner_backing->physical_row_bytes, ledger->cap,
        APPLE_VIRGL_P4_SEMANTIC_LEDGER_RECEIPT_CAP, width, height,
        packed_stride, cpu_stride, owner_sha256, packed_bytes, packed_stride,
        cpu_sha256, packed_bytes, packed_stride);
    if (metadata_length < 0 || metadata_length >= (int)sizeof(metadata) ||
        !apple_virgl_p4_write_file_at(temporary_fd, "metadata.json", metadata,
                                       (size_t)metadata_length) ||
        fsync(temporary_fd) < 0) {
        goto out;
    }
    if (close(temporary_fd) < 0) {
        temporary_fd = -1;
        goto out;
    }
    temporary_fd = -1;
    if (renameat(ledger->dirfd, temporary_name, ledger->dirfd, final_name) <
            0 ||
        fsync(ledger->dirfd) < 0) {
        goto out;
    }
    result = true;

out:
    if (temporary_fd >= 0) {
        close(temporary_fd);
    }
    if (!result) {
        apple_virgl_p4_remove_capture_dir(ledger->dirfd, temporary_name);
    }
    g_free(owner_sha256);
    g_free(cpu_sha256);
    return result;
}

static void apple_virgl_p4_process_status_work(
    AppleVirglP4SemanticLedger *ledger, const AppleVirglP4WorkItem *item)
{
    AppleVirglP4AsyncState *async = ledger->async;
    bool write_error;

    qemu_mutex_lock(&async->lock);
    write_error = ledger->write_error;
    qemu_mutex_unlock(&async->lock);
    if (!apple_virgl_p4_emit_status_unbounded(
            ledger, item->ledger_id, &item->owner_backing, item->status,
            item->reason, item->saturated, item->loss, write_error)) {
        qemu_mutex_lock(&async->lock);
        apple_virgl_p4_note_write_error(ledger);
        qemu_mutex_unlock(&async->lock);
        apple_virgl_p4_log_write_failure(item->ledger_id,
                                         "could not persist status receipt");
    }
}

static void apple_virgl_p4_process_capture_work(
    AppleVirglP4SemanticLedger *ledger, const AppleVirglP4WorkItem *item)
{
    AppleVirglP4AsyncState *async = ledger->async;
    AppleVirglP4CaptureSlot *slot;
    bool equal;
    bool result;

    if (item->capture_slot >= ARRAY_SIZE(async->capture_slots)) {
        qemu_mutex_lock(&async->lock);
        apple_virgl_p4_note_write_error(ledger);
        qemu_mutex_unlock(&async->lock);
        return;
    }
    slot = &async->capture_slots[item->capture_slot];
    equal = memcmp(slot->owner_pixels, slot->cpu_pixels,
                   slot->packed_bytes) == 0;
    result = apple_virgl_p4_write_capture(
        ledger, slot->ledger_id, slot->owner_pixels, slot->cpu_pixels,
        slot->width, slot->height, slot->packed_stride, slot->cpu_stride,
        slot->packed_bytes, equal, &slot->owner_backing);

    qemu_mutex_lock(&async->lock);
    if (result) {
        ledger->capture_count++;
    } else {
        apple_virgl_p4_note_write_error(ledger);
        if (apple_virgl_p4_emit_status(
                ledger, slot->ledger_id, &slot->owner_backing, "failure",
                "capture_persist_failed") == APPLE_VIRGL_P4_RECEIPT_ERROR) {
            apple_virgl_p4_log_write_failure(
                slot->ledger_id, "could not queue capture failure metadata");
        }
    }
    slot->in_use = false;
    slot->ledger_id = 0;
    slot->packed_bytes = 0;
    qemu_mutex_unlock(&async->lock);
}

static void *apple_virgl_p4_writer_thread(void *opaque)
{
    AppleVirglP4SemanticLedger *ledger = opaque;
    AppleVirglP4AsyncState *async = ledger->async;

    for (;;) {
        AppleVirglP4WorkItem item;
        bool have_work = false;
        bool stopping;

        qemu_sem_wait(&async->work_sem);
        qemu_mutex_lock(&async->lock);
        if (async->work_count != 0) {
            item = async->work[async->work_head];
            async->work_head =
                (async->work_head + 1u) % APPLE_VIRGL_P4_WORK_CAP;
            async->work_count--;
            have_work = true;
        }
        stopping = async->stopping && async->work_count == 0;
        qemu_mutex_unlock(&async->lock);

        if (have_work) {
            if (item.kind == APPLE_VIRGL_P4_WORK_CAPTURE) {
                apple_virgl_p4_process_capture_work(ledger, &item);
            } else {
                apple_virgl_p4_process_status_work(ledger, &item);
            }
            continue;
        }
        if (stopping) {
            break;
        }
    }
    return NULL;
}

static bool apple_virgl_p4_async_init(AppleVirglP4SemanticLedger *ledger)
{
    AppleVirglP4AsyncState *async;
    uint32_t index;

    async = g_try_new0(AppleVirglP4AsyncState, 1);
    if (!async) {
        return false;
    }
    qemu_mutex_init(&async->lock);
    qemu_sem_init(&async->work_sem, 0);
    async->synchronization_initialized = true;
    for (index = 0; index < ARRAY_SIZE(async->capture_slots); ++index) {
        async->capture_slots[index].owner_pixels =
            g_try_malloc(APPLE_VIRGL_P4_CAPTURE_BYTES);
        async->capture_slots[index].cpu_pixels =
            g_try_malloc(APPLE_VIRGL_P4_CAPTURE_BYTES);
        if (!async->capture_slots[index].owner_pixels ||
            !async->capture_slots[index].cpu_pixels) {
            goto fail;
        }
    }
    async->accepting = true;
    async->worker_started = true;
    ledger->async = async;
    qemu_thread_create(&async->worker, "av-p4-writer",
                       apple_virgl_p4_writer_thread, ledger,
                       QEMU_THREAD_JOINABLE);
    return true;

fail:
    for (index = 0; index < ARRAY_SIZE(async->capture_slots); ++index) {
        g_free(async->capture_slots[index].owner_pixels);
        g_free(async->capture_slots[index].cpu_pixels);
    }
    qemu_sem_destroy(&async->work_sem);
    qemu_mutex_destroy(&async->lock);
    g_free(async);
    return false;
}

static void apple_virgl_p4_async_stop(AppleVirglP4SemanticLedger *ledger)
{
    AppleVirglP4AsyncState *async = ledger ? ledger->async : NULL;

    if (!async || !async->worker_started) {
        return;
    }
    qemu_mutex_lock(&async->lock);
    async->accepting = false;
    async->stopping = true;
    qemu_mutex_unlock(&async->lock);
    qemu_sem_post(&async->work_sem);
    qemu_thread_join(&async->worker);
    async->worker_started = false;
}

static void apple_virgl_p4_async_destroy(AppleVirglP4SemanticLedger *ledger)
{
    AppleVirglP4AsyncState *async = ledger ? ledger->async : NULL;
    uint32_t index;

    if (!async) {
        return;
    }
    for (index = 0; index < ARRAY_SIZE(async->capture_slots); ++index) {
        g_free(async->capture_slots[index].owner_pixels);
        g_free(async->capture_slots[index].cpu_pixels);
    }
    if (async->synchronization_initialized) {
        qemu_sem_destroy(&async->work_sem);
        qemu_mutex_destroy(&async->lock);
    }
    g_free(async);
    ledger->async = NULL;
}

void apple_virgl_p4_semantic_ledger_init(AppleVirglP4SemanticLedger *ledger)
{
    const char *enable;
    const char *directory;
    struct stat st;
    int flags = O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW;

    if (!ledger) {
        return;
    }
    *ledger = (AppleVirglP4SemanticLedger) {
        .dirfd = -1,
        .cap = APPLE_VIRGL_P4_SEMANTIC_LEDGER_DEFAULT_CAP,
    };
    enable = getenv(APPLE_VIRGL_P4_SEMANTIC_LEDGER_ENABLE_ENV);
    directory = getenv(APPLE_VIRGL_P4_SEMANTIC_LEDGER_DIR_ENV);
    if ((!enable || enable[0] == '\0') &&
        (!directory || directory[0] == '\0')) {
        return;
    }
    if (!enable || strcmp(enable, "1") != 0 || !directory ||
        directory[0] == '\0') {
        error_report("apple-virgl P4 semantic-ledger: require %s=1 and %s",
                     APPLE_VIRGL_P4_SEMANTIC_LEDGER_ENABLE_ENV,
                     APPLE_VIRGL_P4_SEMANTIC_LEDGER_DIR_ENV);
        return;
    }
    if (!apple_virgl_p4_path_is_valid(directory)) {
        error_report("apple-virgl P4 semantic-ledger: %s must be an absolute "
                     "path without dot components",
                     APPLE_VIRGL_P4_SEMANTIC_LEDGER_DIR_ENV);
        return;
    }
    ledger->dirfd = open(directory, flags);
    if (ledger->dirfd < 0 || fstat(ledger->dirfd, &st) < 0 ||
        !S_ISDIR(st.st_mode) || st.st_uid != geteuid() ||
        (st.st_mode & (S_IWGRP | S_IWOTH)) != 0) {
        error_report("apple-virgl P4 semantic-ledger: refusing unsafe output "
                     "directory %s", directory);
        if (ledger->dirfd >= 0) {
            close(ledger->dirfd);
        }
        ledger->dirfd = -1;
        return;
    }
    if (!apple_virgl_p4_async_init(ledger)) {
        apple_virgl_p4_note_write_error(ledger);
        close(ledger->dirfd);
        ledger->dirfd = -1;
        error_report("apple-virgl P4 semantic-ledger: could not allocate "
                     "fixed asynchronous capture staging");
        return;
    }
    ledger->enabled = true;
}

static void apple_virgl_p4_semantic_ledger_finalize(
    AppleVirglP4SemanticLedger *ledger)
{
    char json[2048];
    int length;

    if (!ledger || !ledger->enabled || ledger->finalized) {
        return;
    }
    ledger->finalized = true;
    if (!apple_virgl_p4_arm_finalization_guard(ledger)) {
        apple_virgl_p4_note_write_error(ledger);
        (void)apple_virgl_p4_revoke_final_summary(ledger);
        apple_virgl_p4_log_write_failure(
            0, "could not arm final-summary fail-closed guard");
        return;
    }
    length = snprintf(
        json, sizeof(json),
        "{\n"
        "  \"schema\": \"%s\",\n"
        "  \"state\": \"qemu-final\",\n"
        "  \"raw_capture\": {\n"
        "    \"cap\": %u,\n"
        "    \"unique_tags\": %u,\n"
        "    \"captures\": %" PRIu64 ",\n"
        "    \"saturated\": %s,\n"
        "    \"loss\": %" PRIu64 "\n"
        "  },\n"
        "  \"receipts\": {\n"
        "    \"cap\": %u,\n"
        "    \"written\": %u,\n"
        "    \"saturated\": %s,\n"
        "    \"loss\": %" PRIu64 "\n"
        "  },\n"
        "  \"write_error\": %s\n"
        "}\n",
        APPLE_VIRGL_P4_SEMANTIC_LEDGER_SCHEMA, ledger->cap,
        ledger->unique_tags, ledger->capture_count,
        ledger->raw_capture_saturated ? "true" : "false",
        ledger->raw_capture_loss_count,
        APPLE_VIRGL_P4_SEMANTIC_LEDGER_RECEIPT_CAP,
        ledger->receipt_count,
        ledger->receipt_saturated ? "true" : "false",
        ledger->receipt_loss_count,
        ledger->write_error ? "true" : "false");
    if (length < 0 || length >= (int)sizeof(json) ||
        !apple_virgl_p4_atomic_write_file(ledger,
                                           APPLE_VIRGL_P4_FINAL_SUMMARY_NAME,
                                           json, (size_t)length)) {
        apple_virgl_p4_note_write_error(ledger);
        if (!apple_virgl_p4_revoke_final_summary(ledger)) {
            apple_virgl_p4_log_write_failure(
                0, "could not revoke incomplete final summary");
        }
        apple_virgl_p4_log_write_failure(0,
                                         "could not persist final summary");
        return;
    }
    if (!apple_virgl_p4_disarm_finalization_guard(ledger)) {
        apple_virgl_p4_note_write_error(ledger);
        if (!apple_virgl_p4_revoke_final_summary(ledger)) {
            apple_virgl_p4_log_write_failure(
                0, "could not revoke summary after guard-disarm failure");
        }
        apple_virgl_p4_log_write_failure(
            0, "could not disarm final-summary fail-closed guard");
    }
}

#ifdef APPLE_VIRGL_P4_UNIT_TEST
void apple_virgl_p4_semantic_ledger_test_fail_final_summary_dirsync_once(
    AppleVirglP4SemanticLedger *ledger)
{
    if (ledger) {
        ledger->test_fail_final_summary_dirsync_once = true;
    }
}
#endif

void apple_virgl_p4_semantic_ledger_destroy(AppleVirglP4SemanticLedger *ledger)
{
    if (!ledger) {
        return;
    }
    /*
     * Stop admission first, then drain every queued raw/status artifact outside
     * BQL in the writer before the terminal counters are serialized.
     */
    apple_virgl_p4_async_stop(ledger);
    apple_virgl_p4_semantic_ledger_finalize(ledger);
    if (ledger->dirfd >= 0) {
        close(ledger->dirfd);
    }
    apple_virgl_p4_async_destroy(ledger);
    *ledger = (AppleVirglP4SemanticLedger) { .dirfd = -1 };
}

void apple_virgl_p4_semantic_ledger_record_applied(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const uint8_t *owner_pixels, uint32_t width, uint32_t height,
    uint32_t packed_stride, size_t packed_bytes, const uint8_t *cpu_pixels,
    uint32_t cpu_stride,
    const AppleVirglP4OwnerBackingIdentity *owner_backing)
{
    AppleVirglP4AsyncState *async;
    AppleVirglP4CaptureSlot *slot = NULL;
    AppleVirglP4WorkItem item = { .kind = APPLE_VIRGL_P4_WORK_CAPTURE };
    AppleVirglP4Reservation reservation;
    AppleVirglP4OwnerBackingIdentity zero_identity = { 0 };
    uint32_t slot_index;
    uint32_t row;

    if (!owner_backing) {
        owner_backing = &zero_identity;
    }
    async = ledger ? ledger->async : NULL;
    if (!async) {
        return;
    }
    qemu_mutex_lock(&async->lock);
    reservation = apple_virgl_p4_reserve(ledger, ledger_id, owner_backing);
    if (reservation == APPLE_VIRGL_P4_RESERVATION_DUPLICATE) {
        apple_virgl_p4_report_duplicate(
            ledger, ledger_id, owner_backing,
            "duplicate_completion_after_cpu_apply");
        goto out;
    }
    if (reservation != APPLE_VIRGL_P4_RESERVATION_NEW) {
        goto out;
    }
    if (!owner_pixels || !cpu_pixels ||
        !apple_virgl_p4_valid_geometry(width, height, packed_stride,
                                       packed_bytes, cpu_stride)) {
        if (apple_virgl_p4_emit_status(ledger, ledger_id, owner_backing,
                                       "failure",
                                       "capture_input_invalid") ==
            APPLE_VIRGL_P4_RECEIPT_ERROR) {
            apple_virgl_p4_log_write_failure(ledger_id,
                                             "could not persist "
                                             "invalid-input metadata");
        }
        goto out;
    }
    for (slot_index = 0; slot_index < ARRAY_SIZE(async->capture_slots);
         ++slot_index) {
        if (!async->capture_slots[slot_index].in_use) {
            slot = &async->capture_slots[slot_index];
            break;
        }
    }
    if (!slot) {
        ledger->raw_capture_saturated = true;
        ledger->raw_capture_loss_count++;
        apple_virgl_p4_emit_raw_capture_saturation(
            ledger, ledger_id, owner_backing,
            "raw_capture_staging_exhausted");
        if (apple_virgl_p4_emit_status(ledger, ledger_id, owner_backing,
                                       "failure",
                                       "raw_capture_staging_exhausted") ==
            APPLE_VIRGL_P4_RECEIPT_ERROR) {
            apple_virgl_p4_log_write_failure(ledger_id,
                                             "could not queue staging failure");
        }
        goto out;
    }
    slot->in_use = true;
    slot->ledger_id = ledger_id;
    slot->width = width;
    slot->height = height;
    slot->packed_stride = packed_stride;
    slot->cpu_stride = cpu_stride;
    slot->packed_bytes = packed_bytes;
    slot->owner_backing = *owner_backing;
    memcpy(slot->owner_pixels, owner_pixels, packed_bytes);
    for (row = 0; row < height; ++row) {
        memcpy(slot->cpu_pixels + (size_t)row * packed_stride,
               cpu_pixels + (size_t)row * cpu_stride, packed_stride);
    }
    item.capture_slot = slot_index;
    item.ledger_id = ledger_id;
    item.owner_backing = *owner_backing;
    if (!apple_virgl_p4_queue_work_locked(ledger, &item)) {
        slot->in_use = false;
        slot->ledger_id = 0;
        slot->packed_bytes = 0;
        ledger->raw_capture_saturated = true;
        ledger->raw_capture_loss_count++;
        apple_virgl_p4_emit_raw_capture_saturation(
            ledger, ledger_id, owner_backing, "raw_capture_work_queue_full");
        if (apple_virgl_p4_emit_status(ledger, ledger_id, owner_backing,
                                       "failure",
                                       "raw_capture_work_queue_full") ==
            APPLE_VIRGL_P4_RECEIPT_ERROR) {
            apple_virgl_p4_log_write_failure(ledger_id,
                                             "could not queue work failure");
        }
    }

out:
    qemu_mutex_unlock(&async->lock);
}

void apple_virgl_p4_semantic_ledger_record_apply_failure(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing,
    const char *reason)
{
    AppleVirglP4AsyncState *async;
    AppleVirglP4Reservation reservation;
    AppleVirglP4OwnerBackingIdentity zero_identity = { 0 };
    const char *safe_reason = apple_virgl_p4_apply_failure_reason(reason);

    if (!owner_backing) {
        owner_backing = &zero_identity;
    }
    async = ledger ? ledger->async : NULL;
    if (!async) {
        return;
    }
    qemu_mutex_lock(&async->lock);
    reservation = apple_virgl_p4_reserve(ledger, ledger_id, owner_backing);
    if (reservation == APPLE_VIRGL_P4_RESERVATION_DUPLICATE) {
        apple_virgl_p4_report_duplicate(ledger, ledger_id, owner_backing,
                                        safe_reason);
        goto out;
    }
    if (reservation != APPLE_VIRGL_P4_RESERVATION_NEW) {
        goto out;
    }
    if (apple_virgl_p4_emit_status(ledger, ledger_id, owner_backing,
                                   "failure", safe_reason) ==
        APPLE_VIRGL_P4_RECEIPT_ERROR) {
        apple_virgl_p4_log_write_failure(ledger_id,
                                         "could not persist apply-failure "
                                         "metadata");
    }
out:
    qemu_mutex_unlock(&async->lock);
}
