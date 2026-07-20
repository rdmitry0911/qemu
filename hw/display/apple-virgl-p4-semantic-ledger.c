/*
 * AppleVirgl P4 owner-to-CPU-surface semantic-ledger capture
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "hw/display/apple-virgl-p4-semantic-ledger.h"

#include <inttypes.h>

#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_ENABLE_ENV \
    "APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER"
#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_DIR_ENV \
    "APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR"
#define APPLE_VIRGL_P4_SEMANTIC_LEDGER_SCHEMA \
    "apple-virgl.p4-qemu-scanout/v1"

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
            return reason;
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
        return fsync(ledger->dirfd) == 0;
    }
    return false;
}

static void apple_virgl_p4_note_write_error(AppleVirglP4SemanticLedger *ledger)
{
    if (ledger) {
        ledger->write_error = true;
    }
}

/* This writer does not consume the bounded receipt budget. */
static bool apple_virgl_p4_emit_status_unbounded(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing,
    const char *status, const char *reason, bool saturated, bool loss)
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
        ledger->write_error ? "true" : "false");
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
    if (!apple_virgl_p4_emit_status_unbounded(
            ledger, ledger_id, owner_backing, "raw-capture-saturated", reason,
            true, true)) {
        apple_virgl_p4_note_write_error(ledger);
        apple_virgl_p4_log_write_failure(
            ledger_id, "could not persist raw-capture "
            "saturation receipt");
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
    if (!apple_virgl_p4_emit_status_unbounded(
            ledger, ledger_id, owner_backing, "receipt-saturated",
            "receipt_budget_exhausted", true, true)) {
        apple_virgl_p4_note_write_error(ledger);
        apple_virgl_p4_log_write_failure(
            ledger_id, "could not persist receipt "
            "saturation receipt");
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
    if (apple_virgl_p4_emit_status_unbounded(ledger, ledger_id, owner_backing,
                                             status, reason, false, false)) {
        return APPLE_VIRGL_P4_RECEIPT_WRITTEN;
    }
    apple_virgl_p4_note_write_error(ledger);
    return APPLE_VIRGL_P4_RECEIPT_ERROR;
}

static bool apple_virgl_p4_tag_add(GHashTable *tags, uint64_t ledger_id)
{
    uint64_t *key = g_try_new(uint64_t, 1);

    if (!key) {
        return false;
    }
    *key = ledger_id;
    g_hash_table_add(tags, key);
    return true;
}

static AppleVirglP4Reservation
apple_virgl_p4_reserve(AppleVirglP4SemanticLedger *ledger,
                       uint64_t ledger_id,
                       const AppleVirglP4OwnerBackingIdentity *owner_backing)
{
    if (!ledger || !ledger->enabled || ledger_id == 0) {
        return APPLE_VIRGL_P4_RESERVATION_NONE;
    }
    if (g_hash_table_contains(ledger->seen_tags, &ledger_id)) {
        return APPLE_VIRGL_P4_RESERVATION_DUPLICATE;
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
    if (!apple_virgl_p4_tag_add(ledger->seen_tags, ledger_id)) {
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

    if (width == 0 || height == 0 || width > UINT32_MAX / 4u ||
        packed_stride != width * 4u || cpu_stride < packed_stride) {
        return false;
    }
    expected_bytes = (uint64_t)packed_stride * height;
    return expected_bytes <= SIZE_MAX && packed_bytes == (size_t)expected_bytes;
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
    ledger->seen_tags = g_hash_table_new_full(g_int64_hash, g_int64_equal,
                                              g_free, NULL);
    if (!ledger->seen_tags) {
        apple_virgl_p4_note_write_error(ledger);
        close(ledger->dirfd);
        ledger->dirfd = -1;
        error_report("apple-virgl P4 semantic-ledger: could not allocate "
                     "tag receipt state");
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
        ledger->unique_tags, ledger->capture_sequence,
        ledger->raw_capture_saturated ? "true" : "false",
        ledger->raw_capture_loss_count,
        APPLE_VIRGL_P4_SEMANTIC_LEDGER_RECEIPT_CAP,
        ledger->receipt_count,
        ledger->receipt_saturated ? "true" : "false",
        ledger->receipt_loss_count,
        ledger->write_error ? "true" : "false");
    if (length < 0 || length >= (int)sizeof(json) ||
        !apple_virgl_p4_atomic_write_file(ledger, "qemu-ledger-summary.json",
                                           json, (size_t)length)) {
        apple_virgl_p4_note_write_error(ledger);
        apple_virgl_p4_log_write_failure(0,
                                         "could not persist final summary");
    }
}

void apple_virgl_p4_semantic_ledger_destroy(AppleVirglP4SemanticLedger *ledger)
{
    if (!ledger) {
        return;
    }
    apple_virgl_p4_semantic_ledger_finalize(ledger);
    if (ledger->seen_tags) {
        g_hash_table_destroy(ledger->seen_tags);
    }
    if (ledger->dirfd >= 0) {
        close(ledger->dirfd);
    }
    *ledger = (AppleVirglP4SemanticLedger) { .dirfd = -1 };
}

void apple_virgl_p4_semantic_ledger_record_applied(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const uint8_t *owner_pixels, uint32_t width, uint32_t height,
    uint32_t packed_stride, size_t packed_bytes, const uint8_t *cpu_pixels,
    uint32_t cpu_stride,
    const AppleVirglP4OwnerBackingIdentity *owner_backing)
{
    AppleVirglP4Reservation reservation;
    AppleVirglP4OwnerBackingIdentity zero_identity = { 0 };
    uint8_t *cpu_packed;
    uint32_t row;
    bool equal;

    if (!owner_backing) {
        owner_backing = &zero_identity;
    }
    reservation = apple_virgl_p4_reserve(ledger, ledger_id, owner_backing);
    if (reservation == APPLE_VIRGL_P4_RESERVATION_DUPLICATE) {
        apple_virgl_p4_report_duplicate(
            ledger, ledger_id, owner_backing,
            "duplicate_completion_after_cpu_apply");
        return;
    }
    if (reservation != APPLE_VIRGL_P4_RESERVATION_NEW) {
        return;
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
        return;
    }

    cpu_packed = g_try_malloc(packed_bytes);
    if (!cpu_packed) {
        if (apple_virgl_p4_emit_status(ledger, ledger_id, owner_backing,
                                       "failure",
                                       "capture_allocation_failed") ==
            APPLE_VIRGL_P4_RECEIPT_ERROR) {
            apple_virgl_p4_log_write_failure(ledger_id,
                                             "could not persist "
                                             "allocation-failure metadata");
        }
        return;
    }
    for (row = 0; row < height; ++row) {
        memcpy(cpu_packed + (size_t)row * packed_stride,
               cpu_pixels + (size_t)row * cpu_stride, packed_stride);
    }
    equal = memcmp(owner_pixels, cpu_packed, packed_bytes) == 0;
    if (!apple_virgl_p4_write_capture(ledger, ledger_id, owner_pixels,
                                      cpu_packed, width, height, packed_stride,
                                      cpu_stride, packed_bytes, equal,
                                      owner_backing)) {
        apple_virgl_p4_note_write_error(ledger);
        if (apple_virgl_p4_emit_status(ledger, ledger_id, owner_backing,
                                       "failure",
                                       "capture_persist_failed") ==
            APPLE_VIRGL_P4_RECEIPT_ERROR) {
            apple_virgl_p4_log_write_failure(ledger_id,
                                             "could not persist capture "
                                             "failure metadata");
        }
    }
    g_free(cpu_packed);
}

void apple_virgl_p4_semantic_ledger_record_apply_failure(
    AppleVirglP4SemanticLedger *ledger, uint64_t ledger_id,
    const AppleVirglP4OwnerBackingIdentity *owner_backing,
    const char *reason)
{
    AppleVirglP4Reservation reservation;
    AppleVirglP4OwnerBackingIdentity zero_identity = { 0 };
    const char *safe_reason = apple_virgl_p4_apply_failure_reason(reason);

    if (!owner_backing) {
        owner_backing = &zero_identity;
    }
    reservation = apple_virgl_p4_reserve(ledger, ledger_id, owner_backing);
    if (reservation == APPLE_VIRGL_P4_RESERVATION_DUPLICATE) {
        apple_virgl_p4_report_duplicate(ledger, ledger_id, owner_backing,
                                        safe_reason);
        return;
    }
    if (reservation != APPLE_VIRGL_P4_RESERVATION_NEW) {
        return;
    }
    if (apple_virgl_p4_emit_status(ledger, ledger_id, owner_backing,
                                   "failure", safe_reason) ==
        APPLE_VIRGL_P4_RECEIPT_ERROR) {
        apple_virgl_p4_log_write_failure(ledger_id,
                                         "could not persist apply-failure "
                                         "metadata");
    }
}
