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
#define APPLE_VIRGL_PROTOCOL_VERSION 1u
#define APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3 1u
#define APPLE_VIRGL_SUBMIT_SET_OBJECT_LIST 2u
#define APPLE_VIRGL_SUBMIT_MAP_MEMORY2 3u
#define APPLE_VIRGL_SUBMIT_UNMAP_MEMORY 4u

#define APPLE_VIRGL_MAX_SUBMIT_MAPPINGS 4096u
#define APPLE_VIRGL_MAX_SUBMIT_PAYLOAD (16u * 1024u * 1024u)
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

typedef struct AppleVirglSubmitView {
    const AppleVirglSubmitHeaderV1 *header;
    const AppleVirglSubmitMappingV1 *mappings;
    const uint8_t *payload;
    uint32_t mapping_count;
    uint32_t payload_bytes;
} AppleVirglSubmitView;

void apple_virgl_protocol_fill_capset(AppleVirglCapsetV1 *capset);

bool apple_virgl_protocol_decode_submit(const void *bytes,
                                        size_t size,
                                        AppleVirglSubmitView *view,
                                        Error **errp);

#endif
