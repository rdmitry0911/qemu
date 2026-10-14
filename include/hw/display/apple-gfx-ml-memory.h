/*
 * Direct guest-RAM mapping leaves shared by the device and native controls.
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_DISPLAY_APPLE_GFX_ML_MEMORY_H
#define HW_DISPLAY_APPLE_GFX_ML_MEMORY_H

#include "qemu/rcu.h"
#include "system/address-spaces.h"
#include "system/memory.h"

/* A successful map owns one canonical MemoryRegion reference. */
static inline void *agfx_memory_map_gpa(uint64_t gpa, size_t size, int writable)
{
    RCU_READ_LOCK_GUARD();
    MemoryRegion *mr;
    hwaddr xlat = 0;
    hwaddr xlat_len = size;
    void *ptr;

    mr = address_space_translate(&address_space_memory, gpa, &xlat, &xlat_len,
                                 writable, MEMTXATTRS_UNSPECIFIED);
    if (!mr || xlat_len < size ||
        !memory_access_is_direct(mr, writable, MEMTXATTRS_UNSPECIFIED)) {
        return NULL;
    }
    ptr = memory_region_get_ram_ptr(mr);
    if (!ptr) {
        return NULL;
    }
    memory_region_ref(mr);
    return (uint8_t *)ptr + xlat;
}

/* This is the sole matching unpin for agfx_memory_map_gpa(). */
static inline void agfx_memory_unmap_gpa(void *hva, size_t size, int dirty)
{
    ram_addr_t offset;
    MemoryRegion *mr = memory_region_from_host(hva, &offset);

    if (mr) {
        if (dirty) {
            memory_region_set_dirty(mr, offset, size);
        }
        memory_region_unref(mr);
    }
}

/* Called under BQL by the synchronous dirty-notification BH. The incoming
 * mapping owns the region pin; memory_region_from_host() only borrows it.
 * Notifying a host write must never consume that pin. */
static inline int agfx_memory_mark_mapped_dirty(void *hva, size_t size)
{
    ram_addr_t offset;
    MemoryRegion *mr;

    if (!hva || size == 0) {
        return -1;
    }
    mr = memory_region_from_host(hva, &offset);
    if (!mr) {
        return -1;
    }
    memory_region_set_dirty(mr, offset, size);
    return 0;
}

#endif /* HW_DISPLAY_APPLE_GFX_ML_MEMORY_H */
