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

typedef struct QEMU_PACKED ExecCompletionSubmitV2 {
    AppleVirglSubmitHeaderV1 header;
    AppleVirglSubmitMappingV1 mapping;
    AppleVirglExecCompletionV2 completion;
    uint32_t task_id;
    uint32_t resource_count;
    uint32_t command_count;
    uint64_t command_gpu_va;
    uint64_t command_length;
} ExecCompletionSubmitV2;

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

typedef struct QEMU_PACKED SynchronizeResourcesSubmit {
    AppleVirglSubmitHeaderV1 header;
    AppleVirglSynchronizeResourcesV1 payload;
} SynchronizeResourcesSubmit;

typedef struct QEMU_PACKED DeleteResourceSubmit {
    AppleVirglSubmitHeaderV1 header;
    AppleVirglDeleteResourceV1 payload;
} DeleteResourceSubmit;

typedef struct QEMU_PACKED BindTaskSubmit {
    AppleVirglSubmitHeaderV1 header;
    AppleVirglTaskBindV4 payload;
} BindTaskSubmit;

typedef struct QEMU_PACKED DisplaySharedStateSubmit {
    AppleVirglSubmitHeaderV1 header;
    uint32_t display_id;
    uint32_t shared_state_pfn;
} DisplaySharedStateSubmit;

typedef struct QEMU_PACKED DisplayTransaction3Submit {
    AppleVirglSubmitHeaderV1 header;
    uint32_t display_id;
    uint32_t task_id;
    uint32_t surface_id;
    uint64_t gamma_va;
    uint64_t gamma_length;
    uint32_t gamma_entries;
    uint32_t gamma_sum;
} DisplayTransaction3Submit;

typedef struct QEMU_PACKED ComputeInfoSubmit {
    AppleVirglSubmitHeaderV1 header;
    AppleVirglSubmitMappingV1 mappings[2];
    AppleVirglComputeInfoV1 payload;
} ComputeInfoSubmit;

typedef struct QEMU_PACKED ComputeInfoOneMappingSubmit {
    AppleVirglSubmitHeaderV1 header;
    AppleVirglSubmitMappingV1 mapping;
    AppleVirglComputeInfoV1 payload;
} ComputeInfoOneMappingSubmit;

static OneMappingSubmit valid_submit(void)
{
    OneMappingSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION_V1),
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

static ExecCompletionSubmitV2 valid_exec_completion_submit_v2(void)
{
    ExecCompletionSubmitV2 submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION_V2),
            .opcode = cpu_to_le16(APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3),
            .mapping_count = cpu_to_le32(1),
            .payload_bytes = cpu_to_le32(36),
        },
        .mapping = {
            .gpu_va = cpu_to_le64(0x4000),
            .length = cpu_to_le64(0x1000),
            .backing_resource_id = cpu_to_le32(17),
            .apple_resource_id = cpu_to_le32(7),
        },
        .completion = {
            .channel_id = cpu_to_le32(1),
            .stamp = cpu_to_le32(0x5a5a),
        },
        .task_id = cpu_to_le32(3),
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

static SynchronizeResourcesSubmit valid_synchronize_resources_submit(void)
{
    SynchronizeResourcesSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(
                APPLE_VIRGL_SUBMIT_SYNCHRONIZE_RESOURCES),
            .mapping_count = 0,
            .payload_bytes = cpu_to_le32(
                sizeof(AppleVirglSynchronizeResourcesV1)),
        },
        .payload = {
            .task_id = cpu_to_le32(3),
            .resource_count = cpu_to_le32(1),
            .resource_id = cpu_to_le32(7),
        },
    };

    return submit;
}

static DeleteResourceSubmit valid_delete_resource_submit(void)
{
    DeleteResourceSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(APPLE_VIRGL_SUBMIT_DELETE_RESOURCE),
            .mapping_count = 0,
            .payload_bytes = cpu_to_le32(
                sizeof(AppleVirglDeleteResourceV1)),
        },
        .payload = {
            .task_id = cpu_to_le32(3),
            .resource_id = cpu_to_le32(7),
        },
    };

    return submit;
}

static BindTaskSubmit valid_bind_task_submit(uint32_t task_id, bool is_kernel)
{
    BindTaskSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION_V4),
            .opcode = cpu_to_le16(APPLE_VIRGL_SUBMIT_BIND_TASK),
            .mapping_count = 0,
            .payload_bytes = cpu_to_le32(sizeof(AppleVirglTaskBindV4)),
        },
        .payload = {
            .task_id_encoded = cpu_to_le32((task_id << 1) |
                                           (is_kernel ? 1u : 0u)),
            .vm_size = cpu_to_le64(0x400000000ULL),
            .task_root_pfn = cpu_to_le32(0x12345),
        },
    };

    return submit;
}

static DisplaySharedStateSubmit valid_display_shared_state_submit(void)
{
    DisplaySharedStateSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(
                APPLE_VIRGL_SUBMIT_DISPLAY_SET_SHARED_STATE),
            .mapping_count = 0,
            .payload_bytes = cpu_to_le32(8),
        },
        .display_id = 0,
        .shared_state_pfn = cpu_to_le32(0x12345),
    };

    return submit;
}

static DisplayTransaction3Submit valid_display_transaction3_submit(void)
{
    DisplayTransaction3Submit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(
                APPLE_VIRGL_SUBMIT_DISPLAY_TRANSACTION3),
            .mapping_count = 0,
            .payload_bytes = cpu_to_le32(0x24),
        },
        .display_id = 0,
        .task_id = 0,
        .surface_id = cpu_to_le32(37),
    };

    return submit;
}

static ComputeInfoSubmit valid_compute_info_submit(void)
{
    ComputeInfoSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(
                APPLE_VIRGL_SUBMIT_GET_COMPUTE_INFO),
            .mapping_count = cpu_to_le32(2),
            .payload_bytes = cpu_to_le32(
                sizeof(AppleVirglComputeInfoV1)),
        },
        .mappings = {
            {
                .gpu_va = cpu_to_le64(0x8000),
                .length = cpu_to_le64(0x1000),
                .backing_resource_id = cpu_to_le32(41),
                .apple_resource_id = cpu_to_le32(11),
            },
            {
                .gpu_va = cpu_to_le64(0x9000),
                .length = cpu_to_le64(0x1000),
                .backing_resource_id = cpu_to_le32(42),
                .apple_resource_id = cpu_to_le32(12),
            },
        },
        .payload = {
            .task_id = cpu_to_le32(3),
            .pipeline_ref = cpu_to_le32(37),
            .max_key = cpu_to_le32(5),
            .pair_count = cpu_to_le32(6),
            .reply_gpu_va = cpu_to_le64(0x9080),
        },
    };

    return submit;
}

static ComputeInfoOneMappingSubmit valid_compute_info_one_mapping_submit(void)
{
    ComputeInfoOneMappingSubmit submit = {
        .header = {
            .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
            .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
            .opcode = cpu_to_le16(
                APPLE_VIRGL_SUBMIT_GET_COMPUTE_INFO),
            .mapping_count = cpu_to_le32(1),
            .payload_bytes = cpu_to_le32(
                sizeof(AppleVirglComputeInfoV1)),
        },
        .mapping = {
            .gpu_va = cpu_to_le64(0x8000),
            .length = cpu_to_le64(0x2000),
            .backing_resource_id = cpu_to_le32(41),
            .apple_resource_id = cpu_to_le32(1),
        },
        .payload = {
            .task_id = cpu_to_le32(3),
            .pipeline_ref = cpu_to_le32(37),
            .max_key = cpu_to_le32(5),
            .pair_count = cpu_to_le32(6),
            .reply_gpu_va = cpu_to_le64(0x9080),
        },
    };

    return submit;
}

static void assert_rejected(const void *bytes, size_t size);

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
    g_assert_cmpuint(view.version, ==, APPLE_VIRGL_PROTOCOL_VERSION_V1);
    g_assert_cmpuint(view.completion_channel_id, ==, 0);
    g_assert_cmpuint(view.completion_stamp, ==, 0);
}

static void test_valid_exec_completion_v2(void)
{
    ExecCompletionSubmitV2 submit = valid_exec_completion_submit_v2();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(view.version, ==, APPLE_VIRGL_PROTOCOL_VERSION_V2);
    g_assert_cmpuint(view.payload_bytes, ==, 28);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 3);
    g_assert_cmpuint(view.completion_channel_id, ==, 1);
    g_assert_cmpuint(view.completion_stamp, ==, 0x5a5a);
}

static void test_valid_exec_completion_v3(void)
{
    ExecCompletionSubmitV2 submit = valid_exec_completion_submit_v2();
    AppleVirglSubmitView view;
    Error *err = NULL;

    submit.header.version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION_V3);
    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(view.version, ==, APPLE_VIRGL_PROTOCOL_VERSION_V3);
    g_assert_cmpuint(view.payload_bytes, ==, 28);
    g_assert_cmpuint(view.completion_channel_id, ==, 1);
    g_assert_cmpuint(view.completion_stamp, ==, 0x5a5a);
}

static void test_exec_completion_v2_requires_identity(void)
{
    ExecCompletionSubmitV2 submit = valid_exec_completion_submit_v2();

    submit.completion.channel_id = 0;
    assert_rejected(&submit, sizeof(submit));
    submit = valid_exec_completion_submit_v2();
    submit.completion.stamp = 0;
    assert_rejected(&submit, sizeof(submit));
}

static void test_capset_versions(void)
{
    AppleVirglCapsetV1 capset;

    apple_virgl_protocol_fill_capset(&capset,
                                     APPLE_VIRGL_PROTOCOL_VERSION_V1);
    g_assert_cmpuint(le16_to_cpu(capset.version), ==,
                     APPLE_VIRGL_PROTOCOL_VERSION_V1);
    g_assert_cmpuint(le16_to_cpu(capset.flags), ==, 0);
    apple_virgl_protocol_fill_capset(&capset,
                                     APPLE_VIRGL_PROTOCOL_VERSION_V2);
    g_assert_cmpuint(le16_to_cpu(capset.version), ==,
                     APPLE_VIRGL_PROTOCOL_VERSION_V2);
    g_assert_cmpuint(le16_to_cpu(capset.flags), ==,
                     APPLE_VIRGL_CAPSET_FLAG_EXEC_COMPLETION_STAMP);
    apple_virgl_protocol_fill_capset(&capset,
                                     APPLE_VIRGL_PROTOCOL_VERSION_V3);
    g_assert_cmpuint(le16_to_cpu(capset.version), ==,
                     APPLE_VIRGL_PROTOCOL_VERSION_V3);
    g_assert_cmpuint(le16_to_cpu(capset.flags), ==,
                     APPLE_VIRGL_CAPSET_FLAG_EXEC_COMPLETION_STAMP |
                     APPLE_VIRGL_CAPSET_FLAG_EXEC_COMPLETION_EVENT);
    apple_virgl_protocol_fill_capset(&capset,
                                     APPLE_VIRGL_PROTOCOL_VERSION_V4);
    g_assert_cmpuint(le16_to_cpu(capset.version), ==,
                     APPLE_VIRGL_PROTOCOL_VERSION_V4);
    g_assert_cmpuint(le16_to_cpu(capset.flags), ==,
                     APPLE_VIRGL_CAPSET_FLAG_EXEC_COMPLETION_STAMP |
                     APPLE_VIRGL_CAPSET_FLAG_EXEC_COMPLETION_EVENT);
}

static void test_compute_info_valid(void)
{
    ComputeInfoSubmit submit = valid_compute_info_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_cmpuint(sizeof(AppleVirglComputeInfoV1), ==, 24);
    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(view.mapping_count, ==, 2);
    g_assert_cmpuint(view.payload_bytes, ==, 24);
    g_assert_cmpuint(ldl_le_p(view.payload + 4), ==, 37);
}

static void test_compute_info_reply_range(void)
{
    ComputeInfoSubmit submit = valid_compute_info_submit();

    submit.payload.reply_gpu_va = cpu_to_le64(0xa000);
    assert_rejected(&submit, sizeof(submit));
}

static void test_compute_info_one_mapping(void)
{
    ComputeInfoOneMappingSubmit submit =
        valid_compute_info_one_mapping_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(view.mapping_count, ==, 1);
    g_assert_cmpuint(le32_to_cpu(view.mappings[0].apple_resource_id), ==, 1);
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

static void test_synchronize_resources_valid(void)
{
    SynchronizeResourcesSubmit submit = valid_synchronize_resources_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(le16_to_cpu(view.header->opcode), ==,
                     APPLE_VIRGL_SUBMIT_SYNCHRONIZE_RESOURCES);
    g_assert_cmpuint(view.mapping_count, ==, 0);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 3);
    g_assert_cmpuint(ldl_le_p(view.payload + 4), ==, 1);
    g_assert_cmpuint(ldl_le_p(view.payload + 8), ==, 7);
}

static void test_synchronize_resources_shape(void)
{
    SynchronizeResourcesSubmit submit = valid_synchronize_resources_submit();

    submit.payload.resource_count = 0;
    assert_rejected(&submit, sizeof(submit));
    submit = valid_synchronize_resources_submit();
    submit.payload.resource_count = cpu_to_le32(2);
    assert_rejected(&submit, sizeof(submit));
    submit = valid_synchronize_resources_submit();
    submit.payload.resource_id = 0;
    assert_rejected(&submit, sizeof(submit));
    submit = valid_synchronize_resources_submit();
    submit.header.mapping_count = cpu_to_le32(1);
    assert_rejected(&submit, sizeof(submit));
    submit = valid_synchronize_resources_submit();
    submit.header.payload_bytes = cpu_to_le32(8);
    assert_rejected(&submit, sizeof(submit));
}

static void test_delete_resource_valid(void)
{
    DeleteResourceSubmit submit = valid_delete_resource_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(le16_to_cpu(view.header->opcode), ==,
                     APPLE_VIRGL_SUBMIT_DELETE_RESOURCE);
    g_assert_cmpuint(view.mapping_count, ==, 0);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 3);
    g_assert_cmpuint(ldl_le_p(view.payload + 4), ==, 7);
}

static void test_delete_resource_kernel_task(void)
{
    DeleteResourceSubmit submit = valid_delete_resource_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    submit.payload.task_id = 0;
    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 0);
}

static void test_delete_resource_zero_resource_is_well_formed(void)
{
    DeleteResourceSubmit submit = valid_delete_resource_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    /* The reference host treats the two-word payload as an identity, rather
     * than adding a transport-level nonzero resource-ID policy. */
    submit.payload.resource_id = 0;
    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(ldl_le_p(view.payload + 4), ==, 0);
}

static void test_delete_resource_shape(void)
{
    DeleteResourceSubmit submit = valid_delete_resource_submit();

    submit.header.mapping_count = cpu_to_le32(1);
    assert_rejected(&submit, sizeof(submit));
    submit = valid_delete_resource_submit();
    submit.header.payload_bytes = cpu_to_le32(4);
    assert_rejected(&submit, sizeof(submit) - sizeof(uint32_t));
}

static void test_bind_kernel_task(void)
{
    BindTaskSubmit submit = valid_bind_task_submit(0, true);
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(le16_to_cpu(view.header->opcode), ==,
                     APPLE_VIRGL_SUBMIT_BIND_TASK);
    g_assert_cmpuint(view.version, ==, APPLE_VIRGL_PROTOCOL_VERSION_V4);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 1);
    g_assert_cmpuint(ldq_le_p(view.payload + 4), ==, 0x400000000ULL);
    g_assert_cmpuint(ldl_le_p(view.payload + 12), ==, 0x12345);
}

static void test_bind_user_task(void)
{
    BindTaskSubmit submit = valid_bind_task_submit(7, false);
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(ldl_le_p(view.payload), ==, 14);
}

static void test_bind_task_mapping(void)
{
    BindTaskSubmit submit = valid_bind_task_submit(7, false);

    submit.header.mapping_count = cpu_to_le32(1);
    assert_rejected(&submit, sizeof(submit));
}

static void test_bind_task_shape(void)
{
    BindTaskSubmit submit = valid_bind_task_submit(7, false);

    submit.payload.task_root_pfn = 0;
    assert_rejected(&submit, sizeof(submit));
    submit = valid_bind_task_submit(7, false);
    submit.payload.vm_size = 0;
    assert_rejected(&submit, sizeof(submit));
    submit = valid_bind_task_submit(7, false);
    submit.payload.task_id_encoded = 0;
    assert_rejected(&submit, sizeof(submit));
    submit = valid_bind_task_submit(7, false);
    submit.payload.task_id_encoded = cpu_to_le32(3);
    assert_rejected(&submit, sizeof(submit));
    submit = valid_bind_task_submit(7, false);
    submit.header.version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION_V3);
    assert_rejected(&submit, sizeof(submit));
    submit = valid_bind_task_submit(7, false);
    submit.header.payload_bytes = cpu_to_le32(12);
    assert_rejected(&submit, sizeof(submit) - sizeof(uint32_t));
}

static void test_display_shared_state_valid(void)
{
    DisplaySharedStateSubmit submit = valid_display_shared_state_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(view.payload_bytes, ==, 8);
    g_assert_cmpuint(ldl_le_p(view.payload + 4), ==, 0x12345);
}

static void test_display_shared_state_identity(void)
{
    DisplaySharedStateSubmit submit = valid_display_shared_state_submit();

    submit.display_id = cpu_to_le32(8);
    assert_rejected(&submit, sizeof(submit));
    submit = valid_display_shared_state_submit();
    submit.shared_state_pfn = 0;
    assert_rejected(&submit, sizeof(submit));
}

static void test_display_shared_state_size(void)
{
    DisplaySharedStateSubmit submit = valid_display_shared_state_submit();

    submit.header.payload_bytes = cpu_to_le32(4);
    assert_rejected(&submit, sizeof(submit));
}

static void test_display_transaction3_valid(void)
{
    DisplayTransaction3Submit submit = valid_display_transaction3_submit();
    AppleVirglSubmitView view;
    Error *err = NULL;

    g_assert_true(apple_virgl_protocol_decode_submit(&submit, sizeof(submit),
                                                     &view, &err));
    g_assert_null(err);
    g_assert_cmpuint(view.payload_bytes, ==, 0x24);
    g_assert_cmpuint(ldl_le_p(view.payload + 8), ==, 37);
}

static void test_display_transaction3_identity(void)
{
    DisplayTransaction3Submit submit = valid_display_transaction3_submit();

    submit.task_id = cpu_to_le32(1);
    assert_rejected(&submit, sizeof(submit));
    submit = valid_display_transaction3_submit();
    submit.surface_id = 0;
    assert_rejected(&submit, sizeof(submit));
    submit = valid_display_transaction3_submit();
    submit.display_id = cpu_to_le32(8);
    assert_rejected(&submit, sizeof(submit));
}

static void test_display_transaction3_size(void)
{
    DisplayTransaction3Submit submit = valid_display_transaction3_submit();

    submit.header.payload_bytes = cpu_to_le32(0x20);
    assert_rejected(&submit, sizeof(submit));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/apple-virgl/protocol/valid", test_valid_submit);
    g_test_add_func("/apple-virgl/protocol/exec-completion-v2-valid",
                    test_valid_exec_completion_v2);
    g_test_add_func("/apple-virgl/protocol/exec-completion-v3-valid",
                    test_valid_exec_completion_v3);
    g_test_add_func("/apple-virgl/protocol/exec-completion-v2-identity",
                    test_exec_completion_v2_requires_identity);
    g_test_add_func("/apple-virgl/protocol/capset-versions",
                    test_capset_versions);
    g_test_add_func("/apple-virgl/protocol/compute-info-valid",
                    test_compute_info_valid);
    g_test_add_func("/apple-virgl/protocol/compute-info-reply-range",
                    test_compute_info_reply_range);
    g_test_add_func("/apple-virgl/protocol/compute-info-one-mapping",
                    test_compute_info_one_mapping);
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
    g_test_add_func("/apple-virgl/protocol/synchronize-resources-valid",
                    test_synchronize_resources_valid);
    g_test_add_func("/apple-virgl/protocol/synchronize-resources-shape",
                    test_synchronize_resources_shape);
    g_test_add_func("/apple-virgl/protocol/delete-resource-valid",
                    test_delete_resource_valid);
    g_test_add_func("/apple-virgl/protocol/delete-resource-kernel-task",
                    test_delete_resource_kernel_task);
    g_test_add_func("/apple-virgl/protocol/delete-resource-zero-resource-is-well-formed",
                    test_delete_resource_zero_resource_is_well_formed);
    g_test_add_func("/apple-virgl/protocol/delete-resource-shape",
                    test_delete_resource_shape);
    g_test_add_func("/apple-virgl/protocol/bind-kernel-task",
                    test_bind_kernel_task);
    g_test_add_func("/apple-virgl/protocol/bind-user-task",
                    test_bind_user_task);
    g_test_add_func("/apple-virgl/protocol/bind-task-mapping",
                    test_bind_task_mapping);
    g_test_add_func("/apple-virgl/protocol/bind-task-shape",
                    test_bind_task_shape);
    g_test_add_func("/apple-virgl/protocol/display-shared-state-valid",
                    test_display_shared_state_valid);
    g_test_add_func("/apple-virgl/protocol/display-shared-state-identity",
                    test_display_shared_state_identity);
    g_test_add_func("/apple-virgl/protocol/display-shared-state-size",
                    test_display_shared_state_size);
    g_test_add_func("/apple-virgl/protocol/display-transaction3-valid",
                    test_display_transaction3_valid);
    g_test_add_func("/apple-virgl/protocol/display-transaction3-identity",
                    test_display_transaction3_identity);
    g_test_add_func("/apple-virgl/protocol/display-transaction3-size",
                    test_display_transaction3_size);
    return g_test_run();
}
