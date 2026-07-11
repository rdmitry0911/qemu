/*
 * apple-virgl virtio-gpu to qmetal bridge
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_VIRTIO_APPLE_VIRGL_BRIDGE_H
#define HW_VIRTIO_APPLE_VIRGL_BRIDGE_H

#include "qemu/typedefs.h"

typedef struct VirtIOGPU VirtIOGPU;
typedef struct AppleVirglBridge AppleVirglBridge;
struct iovec;

AppleVirglBridge *apple_virgl_bridge_new(VirtIOGPU *gpu);
void apple_virgl_bridge_free(AppleVirglBridge *bridge);
void apple_virgl_bridge_reset(AppleVirglBridge *bridge);

bool apple_virgl_bridge_has_context(AppleVirglBridge *bridge,
                                    uint32_t context_id);
int apple_virgl_bridge_context_create(AppleVirglBridge *bridge,
                                      uint32_t context_id,
                                      const char *name,
                                      uint32_t name_length);
int apple_virgl_bridge_context_destroy(AppleVirglBridge *bridge,
                                       uint32_t context_id);

int apple_virgl_bridge_resource_create(AppleVirglBridge *bridge,
                                       uint32_t resource_id,
                                       uint64_t declared_size);
void apple_virgl_bridge_resource_destroy(AppleVirglBridge *bridge,
                                         uint32_t resource_id);
int apple_virgl_bridge_resource_attach_backing(AppleVirglBridge *bridge,
                                                uint32_t resource_id,
                                                const uint64_t *addrs,
                                                const struct iovec *iov,
                                                uint32_t iov_count);
void apple_virgl_bridge_resource_detach_backing(AppleVirglBridge *bridge,
                                                 uint32_t resource_id);

int apple_virgl_bridge_context_attach_resource(AppleVirglBridge *bridge,
                                                uint32_t context_id,
                                                uint32_t resource_id);
int apple_virgl_bridge_context_detach_resource(AppleVirglBridge *bridge,
                                                uint32_t context_id,
                                                uint32_t resource_id);

int apple_virgl_bridge_submit(AppleVirglBridge *bridge,
                              uint32_t context_id,
                              const void *bytes,
                              size_t size);

#endif
