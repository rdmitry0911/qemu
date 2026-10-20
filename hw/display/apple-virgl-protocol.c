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
QEMU_BUILD_BUG_ON(sizeof(AppleVirglComputeInfoV1) != 24);
QEMU_BUILD_BUG_ON(sizeof(AppleVirglSynchronizeResourcesV1) != 12);
QEMU_BUILD_BUG_ON(sizeof(AppleVirglDeleteResourceV1) != 8);
QEMU_BUILD_BUG_ON(sizeof(AppleVirglDeleteIOSurfaceBackingV5) != 8);
QEMU_BUILD_BUG_ON(sizeof(AppleVirglExecCompletionV2) != 8);
QEMU_BUILD_BUG_ON(sizeof(AppleVirglTaskBindV4) != 16);
QEMU_BUILD_BUG_ON(sizeof(AppleVirglCompletionReceiverRequestV3) != 8);
QEMU_BUILD_BUG_ON(sizeof(AppleVirglCompletionEventV3) != 16);

void apple_virgl_protocol_fill_capset(AppleVirglCapsetV1 *capset,
                                      uint16_t version)
{
    g_assert(version >= APPLE_VIRGL_PROTOCOL_VERSION_V1);
    g_assert(version <= APPLE_VIRGL_PROTOCOL_VERSION);

    uint16_t flags = version >= APPLE_VIRGL_PROTOCOL_VERSION_V3 ?
        APPLE_VIRGL_CAPSET_FLAG_EXEC_COMPLETION_STAMP |
            APPLE_VIRGL_CAPSET_FLAG_EXEC_COMPLETION_EVENT :
        version >= APPLE_VIRGL_PROTOCOL_VERSION_V2 ?
            APPLE_VIRGL_CAPSET_FLAG_EXEC_COMPLETION_STAMP : 0;

    if (version >= APPLE_VIRGL_PROTOCOL_VERSION_V5) {
        flags |= APPLE_VIRGL_CAPSET_FLAG_DELETE_IOSURFACE_BACKING;
    }

    *capset = (AppleVirglCapsetV1) {
        .magic = cpu_to_le32(APPLE_VIRGL_CAPSET_MAGIC),
        .version = cpu_to_le16(version),
        .flags = cpu_to_le16(flags),
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

static bool apple_virgl_protocol_validate_memory_range(
    uint16_t opcode,
    const AppleVirglSubmitMappingV1 *mappings,
    uint32_t mapping_count,
    const uint8_t *payload,
    uint32_t payload_bytes,
    Error **errp)
{
    bool is_map = opcode == APPLE_VIRGL_SUBMIT_MAP_MEMORY2;
    uint32_t expected_mappings = is_map ? 1 : 0;
    uint64_t gpu_va;
    uint64_t length;

    if (payload_bytes != 20) {
        error_setg(errp,
                   "apple-virgl memory-map payload must be 20 bytes");
        return false;
    }
    if (mapping_count != expected_mappings) {
        error_setg(errp,
                   "apple-virgl memory-map opcode requires %u mappings",
                   expected_mappings);
        return false;
    }

    gpu_va = ldq_le_p(payload + 4);
    length = ldq_le_p(payload + 12);
    if (gpu_va == 0 || length == 0 || length > UINT64_MAX - gpu_va ||
        (gpu_va & 0xfff) != 0 || (length & 0xfff) != 0) {
        error_setg(errp, "apple-virgl memory-map range is invalid");
        return false;
    }
    if (is_map &&
        (le64_to_cpu(mappings[0].gpu_va) != gpu_va ||
         le64_to_cpu(mappings[0].length) != length ||
         le32_to_cpu(mappings[0].backing_offset) != 0 ||
         le32_to_cpu(mappings[0].apple_resource_id) != 0)) {
        error_setg(errp,
                   "apple-virgl MAP_MEMORY2 mapping does not match its range");
        return false;
    }
    return true;
}

static bool apple_virgl_protocol_validate_bind_task(
    uint16_t version,
    uint32_t mapping_count,
    const uint8_t *payload,
    uint32_t payload_bytes,
    Error **errp)
{
    uint32_t task_id_encoded;
    uint64_t vm_size;
    uint32_t task_root_pfn;

    if (version != APPLE_VIRGL_PROTOCOL_VERSION_V4 ||
        mapping_count != 0 || payload_bytes != sizeof(AppleVirglTaskBindV4)) {
        error_setg(errp,
                   "apple-virgl BIND_TASK requires a version-4 DefineTask payload and no mappings");
        return false;
    }

    task_id_encoded = ldl_le_p(payload);
    vm_size = ldq_le_p(payload + 4);
    task_root_pfn = ldl_le_p(payload + 12);
    if (vm_size == 0 || task_root_pfn == 0 ||
        ((task_id_encoded & 1) != 0 && task_id_encoded != 1) ||
        ((task_id_encoded & 1) == 0 && task_id_encoded == 0)) {
        error_setg(errp, "apple-virgl BIND_TASK identity is invalid");
        return false;
    }
    return true;
}

static bool apple_virgl_protocol_validate_display_shared_state(
    uint32_t mapping_count,
    const uint8_t *payload,
    uint32_t payload_bytes,
    Error **errp)
{
    if (mapping_count != 0 || payload_bytes != 8) {
        error_setg(errp,
                   "apple-virgl display shared-state command requires an eight-byte payload and no mappings");
        return false;
    }
    if (ldl_le_p(payload) > 7 || ldl_le_p(payload + 4) == 0) {
        error_setg(errp,
                   "apple-virgl display shared-state identity is invalid");
        return false;
    }
    return true;
}

static bool apple_virgl_protocol_validate_display_transaction3(
    uint32_t mapping_count,
    const uint8_t *payload,
    uint32_t payload_bytes,
    Error **errp)
{
    if (mapping_count != 0 || payload_bytes != 0x24) {
        error_setg(errp,
                   "apple-virgl Transaction3 requires a 36-byte payload and no mappings");
        return false;
    }
    if (ldl_le_p(payload) > 7 || ldl_le_p(payload + 4) != 0 ||
        ldl_le_p(payload + 8) == 0) {
        error_setg(errp,
                   "apple-virgl Transaction3 display, kernel-task, or surface identity is invalid");
        return false;
    }
    return true;
}

static bool apple_virgl_protocol_mapping_contains(
    const AppleVirglSubmitMappingV1 *mapping,
    uint64_t address,
    uint64_t length)
{
    uint64_t base = le64_to_cpu(mapping->gpu_va);
    uint64_t mapping_length = le64_to_cpu(mapping->length);
    uint64_t offset;

    if (address < base) {
        return false;
    }
    offset = address - base;
    return offset <= mapping_length && length <= mapping_length - offset;
}

static bool apple_virgl_protocol_validate_compute_info(
    const AppleVirglSubmitMappingV1 *mappings,
    uint32_t mapping_count,
    const uint8_t *payload,
    uint32_t payload_bytes,
    Error **errp)
{
    uint32_t pair_count;
    uint64_t reply_gpu_va;
    uint64_t reply_bytes;
    uint32_t containing_mappings = 0;
    uint32_t index;

    if (payload_bytes != sizeof(AppleVirglComputeInfoV1)) {
        error_setg(errp,
                   "apple-virgl GET_COMPUTE_INFO payload must be 24 bytes");
        return false;
    }
    if (mapping_count < 1 || mapping_count > 2) {
        error_setg(errp,
                   "apple-virgl GET_COMPUTE_INFO requires one or two mappings");
        return false;
    }
    if (ldl_le_p(payload) == 0 || ldl_le_p(payload + 4) == 0 ||
        ldl_le_p(payload + 8) == 0) {
        error_setg(errp,
                   "apple-virgl GET_COMPUTE_INFO identity or key bound is invalid");
        return false;
    }

    pair_count = ldl_le_p(payload + 12);
    reply_gpu_va = ldq_le_p(payload + 16);
    if (pair_count == 0 || pair_count > APPLE_VIRGL_MAX_COMPUTE_INFO_PAIRS ||
        reply_gpu_va == 0) {
        error_setg(errp,
                   "apple-virgl GET_COMPUTE_INFO reply range is invalid");
        return false;
    }
    reply_bytes = (uint64_t)pair_count * 8u;

    for (index = 0; index < mapping_count; ++index) {
        if (le32_to_cpu(mappings[index].apple_resource_id) == 0) {
            error_setg(errp,
                       "apple-virgl GET_COMPUTE_INFO mapping %u has no Apple resource ID",
                       index);
            return false;
        }
        if (apple_virgl_protocol_mapping_contains(&mappings[index],
                                                  reply_gpu_va,
                                                  reply_bytes)) {
            ++containing_mappings;
        }
    }
    if (containing_mappings == 0) {
        error_setg(errp,
                   "apple-virgl GET_COMPUTE_INFO reply range is not mapped");
        return false;
    }
    return true;
}

static bool apple_virgl_protocol_validate_synchronize_resources(
    uint32_t mapping_count,
    const uint8_t *payload,
    uint32_t payload_bytes,
    Error **errp)
{
    if (mapping_count != 0 ||
        payload_bytes != sizeof(AppleVirglSynchronizeResourcesV1)) {
        error_setg(errp,
                   "apple-virgl SYNCHRONIZE_RESOURCES requires a 12-byte payload and no mappings");
        return false;
    }
    if (ldl_le_p(payload + 4) != 1 || ldl_le_p(payload + 8) == 0) {
        error_setg(errp,
                   "apple-virgl SYNCHRONIZE_RESOURCES must name exactly one resource");
        return false;
    }
    return true;
}

static bool apple_virgl_protocol_validate_delete_resource(
    uint32_t mapping_count,
    const uint8_t *payload,
    uint32_t payload_bytes,
    Error **errp)
{
    if (mapping_count != 0 ||
        payload_bytes != sizeof(AppleVirglDeleteResourceV1)) {
        error_setg(errp,
                   "apple-virgl DELETE_RESOURCE requires an 8-byte payload and no mappings");
        return false;
    }
    return true;
}

static bool apple_virgl_protocol_validate_delete_iosurface_backing(
    uint16_t version,
    uint32_t mapping_count,
    const uint8_t *payload,
    uint32_t payload_bytes,
    Error **errp)
{
    if (version != APPLE_VIRGL_PROTOCOL_VERSION_V5 ||
        mapping_count != 0 ||
        payload_bytes != sizeof(AppleVirglDeleteIOSurfaceBackingV5)) {
        error_setg(errp,
                   "apple-virgl DELETE_IOSURFACE_BACKING requires the V5 exact eight-byte payload and no mappings");
        return false;
    }
    if (ldl_le_p(payload) == 0) {
        error_setg(errp,
                   "apple-virgl DELETE_IOSURFACE_BACKING backing ID is invalid");
        return false;
    }
    /* The only UUID-gated parent-free join currently proves the task-0
     * backing lifetime.  Do not advertise a wider task contract before its
     * reference ownership and runtime ordering are independently joined. */
    if (ldl_le_p(payload + sizeof(uint32_t)) != 0) {
        error_setg(errp,
                   "apple-virgl DELETE_IOSURFACE_BACKING supports task 0 only");
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
    uint16_t version;
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
    version = le16_to_cpu(header->version);
    if (version < APPLE_VIRGL_PROTOCOL_VERSION_V1 ||
        version > APPLE_VIRGL_PROTOCOL_VERSION) {
        error_setg(errp, "apple-virgl submit version is unsupported");
        return false;
    }
    opcode = le16_to_cpu(header->opcode);
    if (opcode != APPLE_VIRGL_SUBMIT_EXEC_INDIRECT3 &&
        opcode != APPLE_VIRGL_SUBMIT_SET_OBJECT_LIST &&
        opcode != APPLE_VIRGL_SUBMIT_MAP_MEMORY2 &&
        opcode != APPLE_VIRGL_SUBMIT_UNMAP_MEMORY &&
        opcode != APPLE_VIRGL_SUBMIT_BIND_TASK &&
        opcode != APPLE_VIRGL_SUBMIT_DISPLAY_SET_SHARED_STATE &&
        opcode != APPLE_VIRGL_SUBMIT_DISPLAY_TRANSACTION3 &&
        opcode != APPLE_VIRGL_SUBMIT_GET_COMPUTE_INFO &&
        opcode != APPLE_VIRGL_SUBMIT_SYNCHRONIZE_RESOURCES &&
        opcode != APPLE_VIRGL_SUBMIT_DELETE_RESOURCE &&
        opcode != APPLE_VIRGL_SUBMIT_DELETE_IOSURFACE_BACKING) {
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
        if (version >= APPLE_VIRGL_PROTOCOL_VERSION_V2) {
            const AppleVirglExecCompletionV2 *completion;

            if (payload_bytes < sizeof(*completion)) {
                error_setg(errp,
                           "apple-virgl completion-enabled ExecIndirect3 prefix is missing");
                return false;
            }
            completion = (const AppleVirglExecCompletionV2 *)payload;
            view->completion_channel_id = le32_to_cpu(completion->channel_id);
            view->completion_stamp = le32_to_cpu(completion->stamp);
            if (view->completion_channel_id == 0 ||
                view->completion_channel_id >= 8 ||
                view->completion_stamp == 0) {
                error_setg(errp,
                           "apple-virgl completion-enabled ExecIndirect3 identity is invalid");
                return false;
            }
            payload += sizeof(*completion);
            payload_bytes -= sizeof(*completion);
        }
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
    case APPLE_VIRGL_SUBMIT_MAP_MEMORY2:
    case APPLE_VIRGL_SUBMIT_UNMAP_MEMORY:
        if (!apple_virgl_protocol_validate_memory_range(
                opcode, mappings, mapping_count, payload, payload_bytes,
                errp)) {
            return false;
        }
        break;
    case APPLE_VIRGL_SUBMIT_BIND_TASK:
        if (!apple_virgl_protocol_validate_bind_task(
                version, mapping_count, payload, payload_bytes, errp)) {
            return false;
        }
        break;
    case APPLE_VIRGL_SUBMIT_DISPLAY_SET_SHARED_STATE:
        if (!apple_virgl_protocol_validate_display_shared_state(
                mapping_count, payload, payload_bytes, errp)) {
            return false;
        }
        break;
    case APPLE_VIRGL_SUBMIT_DISPLAY_TRANSACTION3:
        if (!apple_virgl_protocol_validate_display_transaction3(
                mapping_count, payload, payload_bytes, errp)) {
            return false;
        }
        break;
    case APPLE_VIRGL_SUBMIT_GET_COMPUTE_INFO:
        if (!apple_virgl_protocol_validate_compute_info(
                mappings, mapping_count, payload, payload_bytes, errp)) {
            return false;
        }
        break;
    case APPLE_VIRGL_SUBMIT_SYNCHRONIZE_RESOURCES:
        if (!apple_virgl_protocol_validate_synchronize_resources(
                mapping_count, payload, payload_bytes, errp)) {
            return false;
        }
        break;
    case APPLE_VIRGL_SUBMIT_DELETE_RESOURCE:
        if (!apple_virgl_protocol_validate_delete_resource(
                mapping_count, payload, payload_bytes, errp)) {
            return false;
        }
        break;
    case APPLE_VIRGL_SUBMIT_DELETE_IOSURFACE_BACKING:
        if (!apple_virgl_protocol_validate_delete_iosurface_backing(
                version, mapping_count, payload, payload_bytes, errp)) {
            return false;
        }
        break;
    default:
        g_assert_not_reached();
    }

    view->header = header;
    view->mappings = mappings;
    view->payload = payload;
    view->version = version;
    view->mapping_count = mapping_count;
    view->payload_bytes = payload_bytes;
    return true;
}
