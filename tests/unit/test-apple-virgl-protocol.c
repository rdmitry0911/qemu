/*
 * apple-virgl protocol unit tests
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/virtio/apple-virgl-protocol.h"

typedef struct QEMU_PACKED OneMappingSubmit {
    AppleVirglSubmitHeaderV1 header;
    AppleVirglSubmitMappingV1 mapping;
    uint32_t task_id;
    uint32_t resource_count;
    uint32_t command_count;
    uint64_t command_gpu_va;
    uint64_t command_length;
} OneMappingSubmit;

static OneMappingSubmit valid_submit(void)
{
    OneMappingSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3),
            .mapping_count = cpu_to_le32(1),
            .payload_bytes = cpu_to_le32(28),
        },
        .mapping = {
            .gpu_va = cpu_to_le64(0x4000),
            .length = cpu_to_le64(0x1000),
            .backing_resource_id = cpu_to_le32(17),
            .backing_offset = 0,
            .apple_resource_id = cpu_to_le32(7),
            .reserved = 0,
        },
        .task_id = cpu_to_le32(3),
        .resource_count = 0,
        .command_count = cpu_to_le32(1),
        .command_gpu_va = cpu_to_le64(0x4000),
        .command_length = cpu_to_le64(600),
    };

    return submit;
}

static void test_valid_submit(void)
{
    OneMappingSubmit submit = valid_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(view.mapping_count, ==, 1);
    g_assert_cmpuint(view.payload_bytes, ==, 28);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 3);
}

static void assert_rejected(const void *bytes, size_t size)
{
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_false(apple_virgl_protocol_decode_submit(bytes, size, &view, &err));
    g_assert_nonnull(err);
    error_free(err);
}

static void test_short_header(void)
{
    OneMappingSubmit submit = valid_submit();
    assert_rejected(&submit, sizeof(submit.header) - 1);
}

static void test_invalid_magic(void)
{
    OneMappingSubmit submit = valid_submit();
    submit.header.magic = cpu_to_le32(0xdeadbeef);
    assert_rejected(&submit, sizeof(submit));
}

static void test_invalid_version(void)
{
    OneMappingSubmit submit = valid_submit();
    submit.header.version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION + 1);
    assert_rejected(&submit, sizeof(submit));
}

static void test_size_mismatch(void)
{
    OneMappingSubmit submit = valid_submit();
    submit.header.payload_bytes = cpu_to_le32(29);
    assert_rejected(&submit, sizeof(submit));
}

static void test_exec3_size_mismatch(void)
{
    OneMappingSubmit submit = valid_submit();
    submit.command_count = cpu_to_le32(2);
    assert_rejected(&submit, sizeof(submit));
}

static void test_mapping_limit(void)
{
    OneMappingSubmit submit = valid_submit();
    submit.header.mapping_count = cpu_to_le32(APPLE_VIRGL_MAX_SUBMIT_MAPPINGS + 1);
    assert_rejected(&submit, sizeof(submit));
}

static void test_mapping_reserved(void)
{
    OneMappingSubmit submit = valid_submit();
    submit.mapping.reserved = cpu_to_le32(1);
    assert_rejected(&submit, sizeof(submit));
}

static void test_transport_only_mapping(void)
{
    OneMappingSubmit submit = valid_submit();
    submit.mapping.apple_resource_id = 0;

    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(le32_to_cpu(view.mappings[0].apple_resource_id), ==, 0);
}

static void test_mapping_backing_resource_id(void)
{
    OneMappingSubmit submit = valid_submit();

    submit.mapping.backing_resource_id = 0;
    assert_rejected(&submit, sizeof(submit));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/apple-virgl/protocol/valid", test_valid_submit);
    g_test_add_func("/apple-virgl/protocol/short-header", test_short_header);
    g_test_add_func("/apple-virgl/protocol/invalid-magic", test_invalid_magic);
    g_test_add_func("/apple-virgl/protocol/invalid-version", test_invalid_version);
    g_test_add_func("/apple-virgl/protocol/size-mismatch", test_size_mismatch);
    g_test_add_func("/apple-virgl/protocol/exec3-size-mismatch",
                    test_exec3_size_mismatch);
    g_test_add_func("/apple-virgl/protocol/mapping-limit", test_mapping_limit);
    g_test_add_func("/apple-virgl/protocol/mapping-reserved",
                    test_mapping_reserved);
    g_test_add_func("/apple-virgl/protocol/transport-only-mapping",
                    test_transport_only_mapping);
    g_test_add_func("/apple-virgl/protocol/mapping-backing-resource-id",
                    test_mapping_backing_resource_id);
    return g_test_run();
}
