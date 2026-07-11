/*
 * apple-virgl virtio-gpu capset protocol
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/virtio/apple-virgl-protocol.h"

QEMU_BUILD_BUG_ON(sizeof(AppleVirglCapsetV1) != 16);
QEMU_BUILD_BUG_ON(sizeof(AppleVirglSubmitHeaderV1) != 16);
QEMU_BUILD_BUG_ON(sizeof(AppleVirglSubmitMappingV1) != 32);

void apple_virgl_protocol_fill_capset(AppleVirglCapsetV1 *capset)
{
    *capset = (AppleVirglCapsetV1) {
        .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
        .version = cpu_to_le16(APPLE_VIRGL_PROTOCOL_VERSION),
        .flags = 0,
        .max_mappings = cpu_to_le32(APPLE_VIRGL_MAX_SUBMIT_MAPPINGS),
        .max_payload_bytes = cpu_to_le32(APPLE_VIRGL_MAX_SUBMIT_PAYLOAD),
    };
}

static bool apple_virgl_protocol_validate_exec3(const uint8_t *payload,
                                                uint32_t payload_bytes,
                                                Error **errp)
{
    uint32_t resource_count;
    uint32_t command_count;
    uint64_t expected;

    if (payload_bytes < 12) {
        error_setg(errp, "apple-virgl ExecIndirect3 payload is shorter than 12 bytes");
        return false;
    }

    if (ldl_le_p(payload) == 0) {
        error_setg(errp, "apple-virgl ExecIndirect3 task ID is zero");
        return false;
    }

    resource_count = ldl_le_p(payload + 4);
    command_count = ldl_le_p(payload + 8);
    expected = 12u + (uint64_t)resource_count * 24u +
               (uint64_t)command_count * 16u;
    if (expected != payload_bytes) {
        error_setg(errp,
                   "apple-virgl ExecIndirect3 size mismatch: expected %" PRIu64
                   ", received %u",
                   expected, payload_bytes);
        return false;
    }

    return true;
}

static bool apple_virgl_protocol_validate_object_list(
    const AppleVirglSubmitMappingV1 *mappings,
    uint32_t mapping_count,
    const uint8_t *payload,
    uint32_t payload_bytes,
    Error **errp)
{
    uint32_t heap_pfn;
    uint32_t heap_length;
    uint64_t heap_va;

    if (payload_bytes != 12) {
        error_setg(errp,
                   "apple-virgl SET_OBJECT_LIST payload must be 12 bytes");
        return false;
    }
    if (ldl_le_p(payload) == 0) {
        error_setg(errp, "apple-virgl SET_OBJECT_LIST task ID is zero");
        return false;
    }
    if (mapping_count != 1) {
        error_setg(errp,
                   "apple-virgl SET_OBJECT_LIST requires exactly one mapping");
        return false;
    }

    heap_pfn = ldl_le_p(payload + 4);
    heap_length = ldl_le_p(payload + 8);
    heap_va = (uint64_t)heap_pfn << 12;
    if (heap_pfn == 0 || heap_length == 0 || (heap_length & 0xfff) != 0) {
        error_setg(errp,
                   "apple-virgl SET_OBJECT_LIST heap range is invalid");
        return false;
    }
    if (le64_to_cpu(mappings[0].gpu_va) != heap_va ||
        le64_to_cpu(mappings[0].length) != heap_length ||
        le32_to_cpu(mappings[0].backing_offset) != 0 ||
        le32_to_cpu(mappings[0].apple_resource_id) != 0) {
        error_setg(errp,
                   "apple-virgl SET_OBJECT_LIST mapping does not match its heap");
        return false;
    }
    return true;
}

bool apple_virgl_protocol_decode_submit(const void *bytes,
                                        size_t size,
                                        AppleVirglSubmitView *view,
                                        Error **errp)
{
    const AppleVirglSubmitHeaderV1 *header = bytes;
    uint32_t mapping_count;
    uint32_t payload_bytes;
    uint16_t opcode;
    uint64_t mapping_bytes;
    uint64_t expected;
    const uint8_t *payload;
    const AppleVirglSubmitMappingV1 *mappings;
    uint32_t index;

    if (!bytes || !view) {
        error_setg(errp, "apple-virgl submit arguments are null");
        return false;
    }
    memset(view, 0, sizeof(*view));

    if (size < sizeof(*header)) {
        error_setg(errp, "apple-virgl submit is shorter than its header");
        return false;
    }
    if (le32_to_cpu(header->magic) != APPLE_VIRGL_CAPSET_MAGIC) {
        error_setg(errp, "apple-virgl submit magic is invalid");
        return false;
    }
    if (le16_to_cpu(header->version) != APPLE_VIRGL_PROTOCOL_VERSION) {
        error_setg(errp, "apple-virgl submit version is unsupported");
        return false;
    }
    opcode = le16_to_cpu(header->opcode);
    if (opcode != APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3 &&
        opcode != APPLE_VIRGL_SUBMIT_SET_OBJECT_LIST) {
        error_setg(errp, "apple-virgl submit opcode is unsupported");
        return false;
    }

    mapping_count = le32_to_cpu(header->mapping_count);
    payload_bytes = le32_to_cpu(header->payload_bytes);
    if (mapping_count > APPLE_VIRGL_MAX_SUBMIT_MAPPINGS) {
        error_setg(errp, "apple-virgl submit has too many mappings");
        return false;
    }
    if (payload_bytes > APPLE_VIRGL_MAX_SUBMIT_PAYLOAD) {
        error_setg(errp, "apple-virgl submit payload is too large");
        return false;
    }

    mapping_bytes = (uint64_t)mapping_count * sizeof(AppleVirglSubmitMappingV1);
    expected = sizeof(*header) + mapping_bytes + payload_bytes;
    if (expected != size) {
        error_setg(errp,
                   "apple-virgl submit size mismatch: expected %" PRIu64
                   ", received %zu",
                   expected, size);
        return false;
    }

    mappings = (const AppleVirglSubmitMappingV1 *)
        ((const uint8_t *)bytes + sizeof(*header));
    for (index = 0; index < mapping_count; ++index) {
        uint64_t gpu_va = le64_to_cpu(mappings[index].gpu_va);
        uint64_t length = le64_to_cpu(mappings[index].length);

        if (gpu_va == 0 || length == 0 || length > UINT64_MAX - gpu_va ||
            le32_to_cpu(mappings[index].backing_resource_id) == 0 ||
            le32_to_cpu(mappings[index].reserved) != 0) {
            error_setg(errp, "apple-virgl submit mapping %u is invalid",
                       index);
            return false;
        }
    }

    payload = (const uint8_t *)bytes + sizeof(*header) + mapping_bytes;
    switch (opcode) {
    case APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3:
        if (!apple_virgl_protocol_validate_exec3(payload, payload_bytes,
                                                  errp)) {
            return false;
        }
        break;
    case APPLE_VIRGL_SUBMIT_SET_OBJECT_LIST:
        if (!apple_virgl_protocol_validate_object_list(
                mappings, mapping_count, payload, payload_bytes, errp)) {
            return false;
        }
        break;
    default:
        g_assert_not_reached();
    }

    view->header = header;
    view->mappings = mappings;
    view->payload = payload;
    view->mapping_count = mapping_count;
    view->payload_bytes = payload_bytes;
    return true;
}
