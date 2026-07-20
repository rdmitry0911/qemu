/*
 * AppleVirgl P4 owner-to-CPU-surface semantic-ledger capture tests
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/display/apple-virgl-p4-semantic-ledger.h"
#include "qobject/qjson.h"
#include "qobject/qobject.h"

static void assert_valid_json(const char *json)
{
    Error *error = NULL;
    QObject *object = qobject_from_json(json, &error);

    g_assert_null(error);
    g_assert_nonnull(object);
    qobject_unref(object);
}

static void assert_valid_json_file(const char *path)
{
    g_autofree char *json = NULL;

    g_assert_true(g_file_get_contents(path, &json, NULL, NULL));
    assert_valid_json(json);
}

static void remove_tree(const char *path)
{
    GDir *directory;
    const char *name;

    directory = g_dir_open(path, 0, NULL);
    if (!directory) {
        return;
    }
    while ((name = g_dir_read_name(directory))) {
        g_autofree char *child = g_build_filename(path, name, NULL);

        if (g_file_test(child, G_FILE_TEST_IS_DIR)) {
            remove_tree(child);
        } else {
            g_assert_cmpint(g_remove(child), ==, 0);
        }
    }
    g_dir_close(directory);
    g_assert_cmpint(g_rmdir(path), ==, 0);
}

static char *make_capture_dir(void)
{
    GError *error = NULL;
    char *directory = g_dir_make_tmp("apple-virgl-p4-ledger-XXXXXX", &error);

    g_assert_no_error(error);
    g_assert_nonnull(directory);
    return directory;
}

static void configure_capture_dir(const char *directory)
{
    g_assert_true(g_setenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER", "1",
                           true));
    g_assert_true(g_setenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR", directory,
                           true));
}

static void test_default_off_ignores_nonzero_tag(void)
{
    AppleVirglP4SemanticLedger ledger;
    const uint8_t pixels[] = { 0x10, 0x11, 0x12, 0x13 };

    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER");
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR");
    apple_virgl_p4_semantic_ledger_init(&ledger);
    g_assert_false(ledger.enabled);
    g_assert_null(ledger.async);
    apple_virgl_p4_semantic_ledger_record_applied(&ledger, 1, pixels, 1, 1,
                                                   4, sizeof(pixels), pixels,
                                                   4, NULL);
    apple_virgl_p4_semantic_ledger_destroy(&ledger);
}

static void test_zero_tag_emits_no_per_tag_artifact(void)
{
    AppleVirglP4SemanticLedger ledger;
    g_autofree char *directory = make_capture_dir();
    g_autofree char *summary_path = NULL;
    g_autofree char *summary = NULL;
    GDir *entries;
    const char *name;
    uint32_t entry_count = 0;
    const uint8_t pixels[] = { 0x10, 0x11, 0x12, 0x13 };

    configure_capture_dir(directory);
    apple_virgl_p4_semantic_ledger_init(&ledger);
    g_assert_true(ledger.enabled);
    apple_virgl_p4_semantic_ledger_record_applied(&ledger, 0, pixels, 1, 1,
                                                   4, sizeof(pixels), pixels,
                                                   4, NULL);
    apple_virgl_p4_semantic_ledger_record_apply_failure(
        &ledger, 0, NULL, "cpu_surface_apply_failed");
    apple_virgl_p4_semantic_ledger_destroy(&ledger);
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER");
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR");

    summary_path = g_build_filename(directory, "qemu-ledger-summary.json",
                                    NULL);
    g_assert_true(g_file_get_contents(summary_path, &summary, NULL, NULL));
    assert_valid_json(summary);
    g_assert_nonnull(strstr(summary, "\"unique_tags\": 0"));
    g_assert_nonnull(strstr(summary, "\"captures\": 0"));
    entries = g_dir_open(directory, 0, NULL);
    g_assert_nonnull(entries);
    while ((name = g_dir_read_name(entries))) {
        g_assert_cmpstr(name, ==, "qemu-ledger-summary.json");
        entry_count++;
    }
    g_dir_close(entries);
    g_assert_cmpuint(entry_count, ==, 1);
    remove_tree(directory);
}

static void test_packed_capture_is_atomic_and_row_exact(void)
{
    AppleVirglP4SemanticLedger ledger;
    g_autofree char *directory = make_capture_dir();
    g_autofree char *record_dir = NULL;
    g_autofree char *owner_path = NULL;
    g_autofree char *cpu_path = NULL;
    g_autofree char *metadata_path = NULL;
    g_autofree char *owner = NULL;
    g_autofree char *cpu = NULL;
    g_autofree char *metadata = NULL;
    g_autofree char *summary_path = NULL;
    g_autofree char *summary = NULL;
    gsize owner_length;
    gsize cpu_length;
    const AppleVirglP4OwnerBackingIdentity identity = {
        .owner_object_id = UINT64_C(0x1111222233334444),
        .owner_image_id = UINT64_C(0x5555666677778888),
        .owner_generation = UINT64_C(0x9999aaaabbbbcccc),
        .backing_id = UINT64_C(0xddddeeeeffff0001),
        .backing_generation = UINT64_C(0x0203040506070809),
        .guest_va = UINT64_C(0x123456789abcdef0),
        .physical_image_id = UINT64_C(0x0fedcba987654321),
        .physical_allocation_id = UINT64_C(0x1020304050607080),
        .physical_width = 2,
        .physical_height = 2,
        .physical_pixel_format = 0x42475241u,
        .physical_row_bytes = 8,
    };
    const uint8_t packed[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    };
    const uint8_t cpu_surface[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0xaa, 0xaa, 0xaa, 0xaa,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0xbb, 0xbb, 0xbb, 0xbb,
    };

    configure_capture_dir(directory);
    apple_virgl_p4_semantic_ledger_init(&ledger);
    g_assert_true(ledger.enabled);
    apple_virgl_p4_semantic_ledger_record_applied(
        &ledger, UINT64_C(0x42), packed, 2, 2, 8, sizeof(packed),
        cpu_surface, 12, &identity);
    apple_virgl_p4_semantic_ledger_destroy(&ledger);
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER");
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR");

    record_dir = g_build_filename(directory, "p4-0000000000000042", NULL);
    owner_path = g_build_filename(record_dir, "owner-packed.bgra", NULL);
    cpu_path = g_build_filename(record_dir, "cpu-surface-packed.bgra", NULL);
    metadata_path = g_build_filename(record_dir, "metadata.json", NULL);
    g_assert_true(g_file_get_contents(owner_path, &owner, &owner_length, NULL));
    g_assert_true(g_file_get_contents(cpu_path, &cpu, &cpu_length, NULL));
    g_assert_cmpuint(owner_length, ==, sizeof(packed));
    g_assert_cmpuint(cpu_length, ==, sizeof(packed));
    g_assert_cmpmem(owner, owner_length, packed, sizeof(packed));
    g_assert_cmpmem(cpu, cpu_length, packed, sizeof(packed));
    g_assert_true(g_file_get_contents(metadata_path, &metadata, NULL, NULL));
    assert_valid_json(metadata);
    g_assert_nonnull(strstr(metadata,
                            "\"schema\": \"apple-virgl.p4-qemu-scanout/v1\""));
    g_assert_nonnull(strstr(metadata, "\"state\": \"qemu-captured\""));
    g_assert_nonnull(strstr(metadata, "\"status\": \"applied\""));
    g_assert_nonnull(strstr(metadata, "\"owner_cpu_equal\": true"));
    g_assert_nonnull(strstr(metadata,
                            "\"owner_object_id\": 1229801703532086340"));
    g_assert_nonnull(strstr(metadata, "\"qemu_capture_seq\": 1"));
    summary_path = g_build_filename(directory, "qemu-ledger-summary.json",
                                    NULL);
    g_assert_true(g_file_get_contents(summary_path, &summary, NULL, NULL));
    assert_valid_json(summary);
    g_assert_nonnull(strstr(summary, "\"state\": \"qemu-final\""));
    g_assert_nonnull(strstr(summary, "\"saturated\": false"));
    g_assert_nonnull(strstr(summary, "\"write_error\": false"));
    remove_tree(directory);
}

static void test_mismatch_duplicate_and_saturation_are_explicit(void)
{
    AppleVirglP4SemanticLedger ledger;
    g_autofree char *directory = make_capture_dir();
    g_autofree char *record_dir = NULL;
    g_autofree char *metadata_path = NULL;
    g_autofree char *metadata = NULL;
    g_autofree char *duplicate_path = NULL;
    g_autofree char *duplicate_second_path = NULL;
    g_autofree char *saturated_path = NULL;
    g_autofree char *failure_path = NULL;
    g_autofree char *summary_path = NULL;
    g_autofree char *summary = NULL;
    const uint8_t owner[] = { 0x10, 0x11, 0x12, 0x13 };
    const uint8_t mismatched_cpu[] = { 0x10, 0x11, 0x12, 0xff };

    configure_capture_dir(directory);
    g_assert_true(g_setenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_CAP", "1",
                           true));
    apple_virgl_p4_semantic_ledger_init(&ledger);
    g_assert_true(ledger.enabled);
    /* The production profile ignores caller-tunable cap environment state. */
    g_assert_cmpuint(ledger.cap, ==,
                     APPLE_VIRGL_P4_SEMANTIC_LEDGER_DEFAULT_CAP);
    ledger.cap = 1; /* Unit-only saturation seam. */
    apple_virgl_p4_semantic_ledger_record_applied(
        &ledger, UINT64_C(0x77), owner, 1, 1, 4, sizeof(owner),
        mismatched_cpu, 4, NULL);
    apple_virgl_p4_semantic_ledger_record_apply_failure(
        &ledger, UINT64_C(0x77), NULL, "cpu_surface_apply_failed");
    apple_virgl_p4_semantic_ledger_record_apply_failure(
        &ledger, UINT64_C(0x77), NULL, "cpu_surface_apply_failed");
    apple_virgl_p4_semantic_ledger_record_apply_failure(
        &ledger, UINT64_C(0x78), NULL, "cpu_surface_apply_failed");
    apple_virgl_p4_semantic_ledger_destroy(&ledger);
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER");
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR");
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_CAP");

    record_dir = g_build_filename(directory, "p4-0000000000000077", NULL);
    metadata_path = g_build_filename(record_dir, "metadata.json", NULL);
    g_assert_true(g_file_get_contents(metadata_path, &metadata, NULL, NULL));
    g_assert_nonnull(strstr(metadata, "\"status\": \"owner_cpu_mismatch\""));
    duplicate_path = g_build_filename(
        directory, "p4-0000000000000077-duplicate-1.json", NULL);
    duplicate_second_path = g_build_filename(
        directory, "p4-0000000000000077-duplicate-2.json", NULL);
    saturated_path = g_build_filename(
        directory, "p4-0000000000000078-raw-capture-saturated-3.json", NULL);
    failure_path = g_build_filename(
        directory, "p4-0000000000000078-failure-4.json", NULL);
    summary_path = g_build_filename(directory, "qemu-ledger-summary.json",
                                    NULL);
    g_assert_true(g_file_test(duplicate_path, G_FILE_TEST_EXISTS));
    g_assert_true(g_file_test(duplicate_second_path, G_FILE_TEST_EXISTS));
    g_assert_true(g_file_test(saturated_path, G_FILE_TEST_EXISTS));
    g_assert_true(g_file_test(failure_path, G_FILE_TEST_EXISTS));
    assert_valid_json_file(duplicate_path);
    assert_valid_json_file(duplicate_second_path);
    assert_valid_json_file(saturated_path);
    assert_valid_json_file(failure_path);
    g_assert_true(g_file_get_contents(summary_path, &summary, NULL, NULL));
    assert_valid_json(summary);
    g_assert_nonnull(strstr(summary, "\"raw_capture\": {"));
    g_assert_nonnull(strstr(summary, "\"loss\": 1"));
    remove_tree(directory);
}

static void test_receipt_budget_is_bounded_and_fail_closed(void)
{
    AppleVirglP4SemanticLedger ledger;
    g_autofree char *directory = make_capture_dir();
    g_autofree char *last_duplicate_path = NULL;
    g_autofree char *saturated_path = NULL;
    g_autofree char *summary_path = NULL;
    g_autofree char *summary = NULL;
    const uint8_t pixels[] = { 0x10, 0x11, 0x12, 0x13 };
    uint32_t index;

    configure_capture_dir(directory);
    apple_virgl_p4_semantic_ledger_init(&ledger);
    g_assert_true(ledger.enabled);
    apple_virgl_p4_semantic_ledger_record_applied(
        &ledger, UINT64_C(0x99), pixels, 1, 1, 4, sizeof(pixels), pixels, 4,
        NULL);
    for (index = 0; index <= APPLE_VIRGL_P4_SEMANTIC_LEDGER_RECEIPT_CAP;
         ++index) {
        apple_virgl_p4_semantic_ledger_record_apply_failure(
            &ledger, UINT64_C(0x99), NULL, "cpu_surface_apply_failed");
    }
    apple_virgl_p4_semantic_ledger_destroy(&ledger);
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER");
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR");

    last_duplicate_path = g_strdup_printf(
        "%s/p4-0000000000000099-duplicate-%u.json", directory,
        APPLE_VIRGL_P4_SEMANTIC_LEDGER_RECEIPT_CAP);
    saturated_path = g_strdup_printf(
        "%s/p4-0000000000000099-receipt-saturated-%u.json", directory,
        APPLE_VIRGL_P4_SEMANTIC_LEDGER_RECEIPT_CAP + 1u);
    summary_path = g_build_filename(directory, "qemu-ledger-summary.json",
                                    NULL);
    g_assert_true(g_file_test(last_duplicate_path, G_FILE_TEST_EXISTS));
    g_assert_true(g_file_test(saturated_path, G_FILE_TEST_EXISTS));
    g_assert_true(g_file_get_contents(summary_path, &summary, NULL, NULL));
    assert_valid_json(summary);
    g_assert_nonnull(strstr(summary, "\"written\": 256"));
    g_assert_nonnull(strstr(summary, "\"saturated\": true"));
    g_assert_nonnull(strstr(summary, "\"loss\": 1"));
    g_assert_nonnull(strstr(summary, "\"write_error\": false"));
    remove_tree(directory);
}

static void test_async_capture_drains_before_terminal_summary(void)
{
    AppleVirglP4SemanticLedger ledger;
    g_autofree char *directory = make_capture_dir();
    g_autofree char *record_dir = NULL;
    g_autofree char *owner_path = NULL;
    g_autofree char *cpu_path = NULL;
    g_autofree char *summary_path = NULL;
    g_autofree char *summary = NULL;
    g_autofree uint8_t *owner = NULL;
    g_autofree uint8_t *cpu = NULL;
    GStatBuf stat_buffer;
    const uint32_t width = APPLE_VIRGL_P4_SEMANTIC_LEDGER_MAX_WIDTH;
    const uint32_t height = APPLE_VIRGL_P4_SEMANTIC_LEDGER_MAX_HEIGHT;
    const uint32_t stride = width * 4u;
    const size_t bytes = (size_t)stride * height;

    owner = g_malloc0(bytes);
    cpu = g_malloc0(bytes);
    owner[0] = cpu[0] = 0x31;
    owner[bytes - 1] = cpu[bytes - 1] = 0x7f;

    configure_capture_dir(directory);
    apple_virgl_p4_semantic_ledger_init(&ledger);
    g_assert_true(ledger.enabled);
    g_assert_nonnull(ledger.async);
    apple_virgl_p4_semantic_ledger_record_applied(
        &ledger, UINT64_C(0xabcdef), owner, width, height, stride, bytes, cpu,
        stride, NULL);

    /*
     * Destroy is the exit-notifier lifecycle primitive: it must stop
     * admission, join the writer, and only then publish the final summary.
     */
    apple_virgl_p4_semantic_ledger_destroy(&ledger);
    apple_virgl_p4_semantic_ledger_destroy(&ledger); /* Idempotent unrealize. */
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER");
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR");

    record_dir = g_build_filename(directory, "p4-0000000000abcdef", NULL);
    owner_path = g_build_filename(record_dir, "owner-packed.bgra", NULL);
    cpu_path = g_build_filename(record_dir, "cpu-surface-packed.bgra", NULL);
    summary_path = g_build_filename(directory, "qemu-ledger-summary.json",
                                    NULL);
    g_assert_true(g_file_test(owner_path, G_FILE_TEST_IS_REGULAR));
    g_assert_true(g_file_test(cpu_path, G_FILE_TEST_IS_REGULAR));
    g_assert_cmpint(g_stat(owner_path, &stat_buffer), ==, 0);
    g_assert_cmpuint(stat_buffer.st_size, ==, bytes);
    g_assert_cmpint(g_stat(cpu_path, &stat_buffer), ==, 0);
    g_assert_cmpuint(stat_buffer.st_size, ==, bytes);
    g_assert_true(g_file_get_contents(summary_path, &summary, NULL, NULL));
    assert_valid_json(summary);
    g_assert_nonnull(strstr(summary, "\"captures\": 1"));
    g_assert_nonnull(strstr(summary, "\"write_error\": false"));
    remove_tree(directory);
}

static void test_oversize_capture_fails_without_dynamic_staging(void)
{
    AppleVirglP4SemanticLedger ledger;
    g_autofree char *directory = make_capture_dir();
    g_autofree char *pixels = NULL;
    g_autofree char *record_dir = NULL;
    g_autofree char *failure_path = NULL;
    g_autofree char *summary_path = NULL;
    g_autofree char *summary = NULL;
    const uint32_t width = APPLE_VIRGL_P4_SEMANTIC_LEDGER_MAX_WIDTH + 1u;
    const uint32_t stride = width * 4u;

    pixels = g_malloc0(stride);
    configure_capture_dir(directory);
    apple_virgl_p4_semantic_ledger_init(&ledger);
    g_assert_true(ledger.enabled);
    apple_virgl_p4_semantic_ledger_record_applied(
        &ledger, UINT64_C(0xdead), (const uint8_t *)pixels, width, 1, stride,
        stride, (const uint8_t *)pixels, stride, NULL);
    apple_virgl_p4_semantic_ledger_destroy(&ledger);
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER");
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR");

    record_dir = g_build_filename(directory, "p4-000000000000dead", NULL);
    failure_path = g_build_filename(
        directory, "p4-000000000000dead-failure-1.json", NULL);
    summary_path = g_build_filename(directory, "qemu-ledger-summary.json",
                                    NULL);
    g_assert_false(g_file_test(record_dir, G_FILE_TEST_EXISTS));
    g_assert_true(g_file_test(failure_path, G_FILE_TEST_IS_REGULAR));
    assert_valid_json_file(failure_path);
    g_assert_true(g_file_get_contents(summary_path, &summary, NULL, NULL));
    assert_valid_json(summary);
    g_assert_nonnull(strstr(summary, "\"unique_tags\": 1"));
    g_assert_nonnull(strstr(summary, "\"captures\": 0"));
    g_assert_nonnull(strstr(summary, "\"write_error\": false"));
    remove_tree(directory);
}

static void test_async_capture_persist_failure_has_receipt(void)
{
    AppleVirglP4SemanticLedger ledger;
    g_autofree char *directory = make_capture_dir();
    g_autofree char *record_dir = NULL;
    g_autofree char *failure_path = NULL;
    g_autofree char *failure = NULL;
    g_autofree char *summary_path = NULL;
    g_autofree char *summary = NULL;
    const uint8_t pixels[] = { 0x10, 0x11, 0x12, 0x13 };

    record_dir = g_build_filename(directory, "p4-000000000000f00d", NULL);
    g_assert_cmpint(g_mkdir(record_dir, 0700), ==, 0);
    configure_capture_dir(directory);
    apple_virgl_p4_semantic_ledger_init(&ledger);
    g_assert_true(ledger.enabled);
    apple_virgl_p4_semantic_ledger_record_applied(
        &ledger, UINT64_C(0xf00d), pixels, 1, 1, 4, sizeof(pixels), pixels, 4,
        NULL);
    apple_virgl_p4_semantic_ledger_destroy(&ledger);
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER");
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR");

    failure_path = g_build_filename(
        directory, "p4-000000000000f00d-failure-1.json", NULL);
    summary_path = g_build_filename(directory, "qemu-ledger-summary.json",
                                    NULL);
    g_assert_true(g_file_get_contents(failure_path, &failure, NULL, NULL));
    assert_valid_json(failure);
    g_assert_nonnull(strstr(failure,
                            "\"reason\": \"capture_persist_failed\""));
    g_assert_nonnull(strstr(failure, "\"write_error\": true"));
    g_assert_true(g_file_get_contents(summary_path, &summary, NULL, NULL));
    assert_valid_json(summary);
    g_assert_nonnull(strstr(summary, "\"unique_tags\": 1"));
    g_assert_nonnull(strstr(summary, "\"captures\": 0"));
    g_assert_nonnull(strstr(summary, "\"write_error\": true"));
    remove_tree(directory);
}

static void test_final_summary_dirsync_failure_is_fail_closed(void)
{
    AppleVirglP4SemanticLedger ledger;
    g_autofree char *directory = make_capture_dir();
    g_autofree char *summary_path = NULL;
    g_autofree char *guard_path = NULL;
    const uint8_t pixels[] = { 0x10, 0x11, 0x12, 0x13 };

    configure_capture_dir(directory);
    apple_virgl_p4_semantic_ledger_init(&ledger);
    g_assert_true(ledger.enabled);
    apple_virgl_p4_semantic_ledger_record_applied(
        &ledger, UINT64_C(0xcafe), pixels, 1, 1, 4, sizeof(pixels), pixels,
        4, NULL);
    /* Fault is injected only after qemu-ledger-summary.json has been renamed. */
    apple_virgl_p4_semantic_ledger_test_fail_final_summary_dirsync_once(
        &ledger);
    apple_virgl_p4_semantic_ledger_destroy(&ledger);
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER");
    g_unsetenv("APPLE_VIRGL_QEMU_P4_SEMANTIC_LEDGER_DIR");

    summary_path = g_build_filename(directory, "qemu-ledger-summary.json",
                                    NULL);
    guard_path = g_build_filename(directory, ".p4-finalization-pending", NULL);
    /* No canonical acceptance token remains after the post-rename fsync fault. */
    g_assert_false(g_file_test(summary_path, G_FILE_TEST_EXISTS));
    /* If unlink/fsync ever also fails, this validator-visible guard is fail-closed. */
    g_assert_true(g_file_test(guard_path, G_FILE_TEST_IS_REGULAR));
    remove_tree(directory);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/apple-virgl/p4-semantic-ledger/default-off",
                    test_default_off_ignores_nonzero_tag);
    g_test_add_func("/apple-virgl/p4-semantic-ledger/zero-tag-no-artifact",
                    test_zero_tag_emits_no_per_tag_artifact);
    g_test_add_func("/apple-virgl/p4-semantic-ledger/packed-row-exact",
                    test_packed_capture_is_atomic_and_row_exact);
    g_test_add_func("/apple-virgl/p4-semantic-ledger/mismatch-duplicate-cap",
                    test_mismatch_duplicate_and_saturation_are_explicit);
    g_test_add_func("/apple-virgl/p4-semantic-ledger/receipt-budget",
                    test_receipt_budget_is_bounded_and_fail_closed);
    g_test_add_func("/apple-virgl/p4-semantic-ledger/async-drain-summary",
                    test_async_capture_drains_before_terminal_summary);
    g_test_add_func("/apple-virgl/p4-semantic-ledger/oversize-fixed-staging",
                    test_oversize_capture_fails_without_dynamic_staging);
    g_test_add_func("/apple-virgl/p4-semantic-ledger/persist-failure-receipt",
                    test_async_capture_persist_failure_has_receipt);
    g_test_add_func("/apple-virgl/p4-semantic-ledger/final-summary-dirsync-fail-closed",
                    test_final_summary_dirsync_failure_is_fail_closed);
    return g_test_run();
}
