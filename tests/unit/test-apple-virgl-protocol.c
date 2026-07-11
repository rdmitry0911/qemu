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

typedef struct QEMU_PACKED ObjectListSubmit {
    AppleVirglSubmitHeaderV1 header;
    AppleVirglSubmitMappingV1 mapping;
    uint32_t task_id;
    uint32_t heap_pfn;
    uint32_t heap_length;
} ObjectListSubmit;

typedef struct QEMU_PACKED MemoryMapSubmit {
    AppleVirglSubmitHeaderV1 header;
    AppleVirglSubmitMappingV1 mapping;
    uint32_t task_id;
    uint64_t gpu_va;
    uint64_t length;
} MemoryMapSubmit;

typedef struct QEMU_PACKED MemoryUnmapSubmit {
    AppleVirglSubmitHeaderV1 header;
    uint32_t task_id;
    uint64_t gpu_va;
    uint64_t length;
} MemoryUnmapSubmit;

typedef struct QEMU_PACKED BindTaskSubmit {
    AppleVirglSubmitHeaderV1 header;
    uint32_t task_id;
} BindTaskSubmit;

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

static ObjectListSubmit valid_object_list_submit(void)
{
    ObjectListSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(APPLE_VIRGL_SUBMIT_SET_OBJECT_LIST),
            .mapping_count = cpu_to_le32(1),
            .payload_bytes = cpu_to_le32(12),
        },
        .mapping = {
            .gpu_va = cpu_to_le64(0x12345000),
            .length = cpu_to_le64(0x100000),
            .backing_resource_id = cpu_to_le32(23),
        },
        .task_id = cpu_to_le32(3),
        .heap_pfn = cpu_to_le32(0x12345),
        .heap_length = cpu_to_le32(0x100000),
    };

    return submit;
}

static MemoryMapSubmit valid_memory_map_submit(void)
{
    MemoryMapSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(APPLE_VIRGL_SUBMIT_MAP_MEMORY2),
            .mapping_count = cpu_to_le32(1),
            .payload_bytes = cpu_to_le32(20),
        },
        .mapping = {
            .gpu_va = cpu_to_le64(0x1b0000),
            .length = cpu_to_le64(0x40000),
            .backing_resource_id = cpu_to_le32(31),
        },
        .task_id = cpu_to_le32(3),
        .gpu_va = cpu_to_le64(0x1b0000),
        .length = cpu_to_le64(0x40000),
    };

    return submit;
}

static MemoryUnmapSubmit valid_memory_unmap_submit(void)
{
    MemoryUnmapSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(APPLE_VIRGL_SUBMIT_UNMAP_MEMORY),
            .mapping_count = 0,
            .payload_bytes = cpu_to_le32(20),
        },
        .task_id = cpu_to_le32(3),
        .gpu_va = cpu_to_le64(0x1b0000),
        .length = cpu_to_le64(0x40000),
    };

    return submit;
}

static BindTaskSubmit valid_bind_task_submit(uint32_t task_id)
{
    BindTaskSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(APPLE_VIRGL_SUBMIT_BIND_TASK),
            .mapping_count = 0,
            .payload_bytes = cpu_to_le32(sizeof(uint32_t)),
        },
        .task_id = cpu_to_le32(task_id),
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

static void test_valid_object_list(void)
{
    ObjectListSubmit submit = valid_object_list_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(le16_to_cpu(view.header->opcode), ==,
                     APPLE_VIRGL_SUBMIT_SET_OBJECT_LIST);
    g_assert_cmpuint(view.mapping_count, ==, 1);
}

static void test_object_list_size(void)
{
    ObjectListSubmit submit = valid_object_list_submit();
    submit.header.payload_bytes = cpu_to_le32(8);
    assert_rejected(&submit, sizeof(submit));
}

static void test_object_list_kernel_task(void)
{
    ObjectListSubmit submit = valid_object_list_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    submit.task_id = 0;
    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 0);
}

static void test_object_list_heap_alignment(void)
{
    ObjectListSubmit submit = valid_object_list_submit();
    submit.heap_length = cpu_to_le32(0x100001);
    submit.mapping.length = cpu_to_le64(0x100001);
    assert_rejected(&submit, sizeof(submit));
}

static void test_object_list_mapping(void)
{
    ObjectListSubmit submit = valid_object_list_submit();
    submit.mapping.gpu_va = cpu_to_le64(0x12346000);
    assert_rejected(&submit, sizeof(submit));
}

static void test_object_list_mapping_count(void)
{
    ObjectListSubmit submit = valid_object_list_submit();
    submit.header.mapping_count = 0;
    assert_rejected(&submit, sizeof(submit));
}

static void test_valid_memory_map(void)
{
    MemoryMapSubmit submit = valid_memory_map_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(le16_to_cpu(view.header->opcode), ==,
                     APPLE_VIRGL_SUBMIT_MAP_MEMORY2);
    g_assert_cmpuint(view.mapping_count, ==, 1);
}

static void test_memory_map_range_mismatch(void)
{
    MemoryMapSubmit submit = valid_memory_map_submit();
    submit.mapping.gpu_va = cpu_to_le64(0x1c0000);
    assert_rejected(&submit, sizeof(submit));
}

static void test_memory_map_resource_id(void)
{
    MemoryMapSubmit submit = valid_memory_map_submit();
    submit.mapping.apple_resource_id = cpu_to_le32(9);
    assert_rejected(&submit, sizeof(submit));
}

static void test_memory_map_alignment(void)
{
    MemoryMapSubmit submit = valid_memory_map_submit();
    submit.gpu_va = cpu_to_le64(0x1b0001);
    submit.mapping.gpu_va = cpu_to_le64(0x1b0001);
    assert_rejected(&submit, sizeof(submit));
}

static void test_memory_map_kernel_task(void)
{
    MemoryMapSubmit submit = valid_memory_map_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    submit.task_id = 0;
    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 0);
}

static void test_valid_memory_unmap(void)
{
    MemoryUnmapSubmit submit = valid_memory_unmap_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(le16_to_cpu(view.header->opcode), ==,
                     APPLE_VIRGL_SUBMIT_UNMAP_MEMORY);
    g_assert_cmpuint(view.mapping_count, ==, 0);
}

static void test_memory_unmap_kernel_task(void)
{
    MemoryUnmapSubmit submit = valid_memory_unmap_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    submit.task_id = 0;
    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 0);
}

static void test_memory_unmap_size(void)
{
    MemoryUnmapSubmit submit = valid_memory_unmap_submit();
    submit.header.payload_bytes = cpu_to_le32(16);
    assert_rejected(&submit, sizeof(submit));
}

static void test_bind_kernel_task(void)
{
    BindTaskSubmit submit = valid_bind_task_submit(0);
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(le16_to_cpu(view.header->opcode), ==,
                     APPLE_VIRGL_SUBMIT_BIND_TASK);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 0);
}

static void test_bind_task_mapping(void)
{
    BindTaskSubmit submit = valid_bind_task_submit(7);

    submit.header.mapping_count = cpu_to_le32(1);
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
    g_test_add_func("/apple-virgl/protocol/object-list-valid",
                    test_valid_object_list);
    g_test_add_func("/apple-virgl/protocol/object-list-size",
                    test_object_list_size);
    g_test_add_func("/apple-virgl/protocol/object-list-kernel-task",
                    test_object_list_kernel_task);
    g_test_add_func("/apple-virgl/protocol/object-list-heap-alignment",
                    test_object_list_heap_alignment);
    g_test_add_func("/apple-virgl/protocol/object-list-mapping",
                    test_object_list_mapping);
    g_test_add_func("/apple-virgl/protocol/object-list-mapping-count",
                    test_object_list_mapping_count);
    g_test_add_func("/apple-virgl/protocol/memory-map-valid",
                    test_valid_memory_map);
    g_test_add_func("/apple-virgl/protocol/memory-map-range-mismatch",
                    test_memory_map_range_mismatch);
    g_test_add_func("/apple-virgl/protocol/memory-map-resource-id",
                    test_memory_map_resource_id);
    g_test_add_func("/apple-virgl/protocol/memory-map-alignment",
                    test_memory_map_alignment);
    g_test_add_func("/apple-virgl/protocol/memory-map-kernel-task",
                    test_memory_map_kernel_task);
    g_test_add_func("/apple-virgl/protocol/memory-unmap-valid",
                    test_valid_memory_unmap);
    g_test_add_func("/apple-virgl/protocol/memory-unmap-kernel-task",
                    test_memory_unmap_kernel_task);
    g_test_add_func("/apple-virgl/protocol/memory-unmap-size",
                    test_memory_unmap_size);
    g_test_add_func("/apple-virgl/protocol/bind-kernel-task",
                    test_bind_kernel_task);
    g_test_add_func("/apple-virgl/protocol/bind-task-mapping",
                    test_bind_task_mapping);
    return g_test_run();
}
