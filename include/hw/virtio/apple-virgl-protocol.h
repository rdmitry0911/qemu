/*
 * apple-virgl virtio-gpu capset protocol
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_VIRTIO_APPLE_VIRGL_PROTOCOL_H
#define HW_VIRTIO_APPLE_VIRGL_PROTOCOL_H

#include "qapi/error.h"

#define VIRTIO_GPU_CAPSET_APPLE_VIRGL 0x80u

#define APPLE_VIRGL_CAPSET_MAGIC 0x4c475641u /* "AVGL", little-endian */
#define APPLE_VIRGL_PROTOCOL_VERSION_V1 1u
#define APPLE_VIRGL_PROTOCOL_VERSION_V2 2u
#define APPLE_VIRGL_PROTOCOL_VERSION_V3 3u
#define APPLE_VIRGL_PROTOCOL_VERSION_V4 4u
#define APPLE_VIRGL_PROTOCOL_VERSION APPLE_VIRGL_PROTOCOL_VERSION_V4
#define APPLE_VIRGL_CAPSET_FLAG_EXEC_COMPLETION_STAMP (1u << 0)
#define APPLE_VIRGL_CAPSET_FLAG_EXEC_COMPLETION_EVENT (1u << 1)
#define APPLE_VIRGL_CURSOR_CMD_COMPLETION_RECEIVE 0xa11e0001u
#define APPLE_VIRGL_COMPLETION_EVENT_MAGIC 0x45435641u /* "AVCE", little-endian */
#define APPLE_VIRGL_COMPLETION_EVENT_TYPE_STAMP 1u
#define APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3 1u
#define APPLE_VIRGL_SUBMIT_SET_OBJECT_LIST 2u
#define APPLE_VIRGL_SUBMIT_MAP_MEMORY2 3u
#define APPLE_VIRGL_SUBMIT_UNMAP_MEMORY 4u
#define APPLE_VIRGL_SUBMIT_BIND_TASK 5u
#define APPLE_VIRGL_SUBMIT_DISPLAY_SET_SHARED_STATE 6u
#define APPLE_VIRGL_SUBMIT_DISPLAY_TRANSACTION3 7u
#define APPLE_VIRGL_SUBMIT_GET_COMPUTE_INFO 8u
#define APPLE_VIRGL_SUBMIT_SYNCHRONIZE_RESOURCES 9u

#define APPLE_VIRGL_MAX_SUBMIT_MAPPINGS 4096u
#define APPLE_VIRGL_MAX_SUBMIT_PAYLOAD (16u * 1024u * 1024u)
#define APPLE_VIRGL_MAX_COMPUTE_INFO_PAIRS 4096u
#define APPLE_VIRGL_MAX_SUBMIT_BYTES \
    (sizeof(AppleVirglSubmitHeaderV1) + \
     APPLE_VIRGL_MAX_SUBMIT_MAPPINGS * sizeof(AppleVirglSubmitMappingV1) + \
     APPLE_VIRGL_MAX_SUBMIT_PAYLOAD)

typedef struct QEMU_PACKED AppleVirglCapsetV1 {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint32_t max_mappings;
    uint32_t max_payload_bytes;
} AppleVirglCapsetV1;

typedef struct QEMU_PACKED AppleVirglSubmitHeaderV1 {
    uint32_t magic;
    uint16_t version;
    uint16_t opcode;
    uint32_t mapping_count;
    uint32_t payload_bytes;
} AppleVirglSubmitHeaderV1;

typedef struct QEMU_PACKED AppleVirglSubmitMappingV1 {
    uint64_t gpu_va;
    uint64_t length;
    uint32_t backing_resource_id;
    uint32_t backing_offset;
    uint32_t apple_resource_id;
    uint32_t reserved;
} AppleVirglSubmitMappingV1;

typedef struct QEMU_PACKED AppleVirglComputeInfoV1 {
    uint32_t task_id;
    uint32_t pipeline_ref;
    uint32_t max_key;
    uint32_t pair_count;
    uint64_t reply_gpu_va;
} AppleVirglComputeInfoV1;

/* Exact APV CmdSynchronizeResources payload for one resource. */
typedef struct QEMU_PACKED AppleVirglSynchronizeResourcesV1 {
    uint32_t task_id;
    uint32_t resource_count;
    uint32_t resource_id;
} AppleVirglSynchronizeResourcesV1;

/* Version-2 EXEC payload prefix. The following bytes remain ExecIndirect3. */
typedef struct QEMU_PACKED AppleVirglExecCompletionV2 {
    uint32_t channel_id;
    uint32_t stamp;
} AppleVirglExecCompletionV2;

/* Native APV DefineTask layout carried by the BIND_TASK transport opcode. */
typedef struct QEMU_PACKED AppleVirglTaskBindV4 {
    uint32_t task_id_encoded;
    uint64_t vm_size;
    uint32_t task_root_pfn;
} AppleVirglTaskBindV4;

/* Version-3 cursor-queue receive-credit payload, after virtio_gpu_ctrl_hdr. */
typedef struct QEMU_PACKED AppleVirglCompletionReceiverRequestV3 {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
} AppleVirglCompletionReceiverRequestV3;

typedef struct QEMU_PACKED AppleVirglCompletionEventV3 {
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint32_t channel_id;
    uint32_t stamp;
} AppleVirglCompletionEventV3;

typedef struct AppleVirglSubmitView {
    const AppleVirglSubmitHeaderV1 *header;
    const AppleVirglSubmitMappingV1 *mappings;
    const uint8_t *payload;
    uint16_t version;
    uint32_t mapping_count;
    uint32_t payload_bytes;
    uint32_t completion_channel_id;
    uint32_t completion_stamp;
} AppleVirglSubmitView;

void apple_virgl_protocol_fill_capset(AppleVirglCapsetV1 *capset,
                                      uint16_t version);

bool apple_virgl_protocol_decode_submit(const void *bytes,
                                        size_t size,
                                        AppleVirglSubmitView *view,
                                        Error **errp);

#endif
