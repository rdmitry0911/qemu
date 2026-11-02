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

/* QMetal workers and the agfx-io MMIO worker are not QEMU-created RCU
 * readers. This registers the calling thread for exactly the outer interval
 * when it is not already registered and nests on the same thread; the
 * definitions live with the existing task-provider users. */
void agfx_memory_rcu_enter(void);
void agfx_memory_rcu_leave(void);

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

/* The incoming mapping owns the region pin; memory_region_from_host() only
 * borrows it. Notifying a host write must never consume that pin. Needs an
 * RCU read-side section (dirty bitmap blocks) but not BQL: it is the same
 * atomic dirty-range operation a vCPU write performs. */
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

/* Any-thread forms used by the QMetal callbacks. The map form preserves the
 * leaf's admission and its one MR reference; the dirty form keeps borrowing
 * the caller's live pin and never unmaps. Final unpin stays the BQL path. */
static inline void *agfx_memory_map_gpa_any_thread(uint64_t gpa, size_t size,
                                                   int writable)
{
    void *ptr;

    agfx_memory_rcu_enter();
    ptr = agfx_memory_map_gpa(gpa, size, writable);
    agfx_memory_rcu_leave();
    return ptr;
}

static inline int agfx_memory_mark_mapped_dirty_any_thread(void *hva,
                                                           size_t size)
{
    int result;

    agfx_memory_rcu_enter();
    result = agfx_memory_mark_mapped_dirty(hva, size);
    agfx_memory_rcu_leave();
    return result;
}

/* RAM-copy leaf of the device physical read: every span of [gpa, gpa+size)
 * that translates to directly accessible RAM (memory_access_is_direct: RAM or
 * ROMD, never RAM-device or I/O) is copied by the calling thread inside a
 * registered RCU read section, so the read needs neither the BQL nor a
 * main-loop BH. Returns false at the first span that is not direct; the caller
 * then performs the WHOLE read on its unchanged BQL device path, which rewrites
 * the entire destination (a read has no guest-visible side effect, so the
 * abandoned prefix is never observable). Writes and mappings are unaffected. */
static inline bool agfx_memory_read_ram_any_thread(uint64_t gpa, void *buf,
                                                   size_t size)
{
    uint8_t *dst = buf;
    bool copied = true;

    agfx_memory_rcu_enter();
    FlatView *view = address_space_to_flatview(&address_space_memory);
    while (size > 0) {
        hwaddr offset = 0;
        hwaddr length = size;
        MemoryRegion *mr = view ?
            flatview_translate(view, gpa, &offset, &length, false,
                               MEMTXATTRS_UNSPECIFIED) : NULL;
        const void *source;

        if (!mr || length == 0 || length > size || !mr->ram_block ||
            !memory_access_is_direct(mr, false, MEMTXATTRS_UNSPECIFIED)) {
            copied = false;
            break;
        }
        source = qemu_map_ram_ptr(mr->ram_block, offset);
        if (!source) {
            copied = false;
            break;
        }
        memcpy(dst, source, length);
        dst += length;
        gpa += length;
        size -= length;
    }
    agfx_memory_rcu_leave();
    return copied;
}

/* Seqlock over address-space publication. memory_region_transaction_commit
 * (under BQL) calls every listener's begin before any address_space_set_flatview
 * and commit after all current_map stores: odd = publication in progress. */
typedef struct AgfxTopologySeq {
    MemoryListener listener;
    uint64_t seq;
    bool registered;
} AgfxTopologySeq;

static inline void agfx_topology_seq_advance(MemoryListener *listener)
{
    qatomic_inc(&container_of(listener, AgfxTopologySeq, listener)->seq);
}

/* Under BQL. Registration itself runs begin/commit once (even result). */
static inline void agfx_topology_seq_register(AgfxTopologySeq *topology,
                                              const char *name)
{
    topology->listener = (MemoryListener) {
        .name = name,
        .begin = agfx_topology_seq_advance,
        .commit = agfx_topology_seq_advance,
        .priority = MEMORY_LISTENER_PRIORITY_MIN,
    };
    memory_listener_register(&topology->listener, &address_space_memory);
    topology->registered = true;
}

static inline void agfx_topology_seq_unregister(AgfxTopologySeq *topology)
{
    if (topology->registered) {
        memory_listener_unregister(&topology->listener);
        topology->registered = false;
    }
}

static inline uint64_t agfx_topology_seq_read(AgfxTopologySeq *topology)
{
    return qatomic_load_acquire(&topology->seq);
}

/* Retained capture: one direct RAM alias + its MR reference, accepted only
 * when no publication was in progress at its start and none began before its
 * end. A mapping acquired but rejected is handed back in *rejected for the
 * caller's own unpin path (never unpinned here). */
static inline void *agfx_memory_capture_retained_with(
    AgfxTopologySeq *topology, uint64_t gpa, size_t size, int writable,
    uint64_t *sequence, void **rejected,
    void *(*map)(uint64_t gpa, size_t size, int writable))
{
    uint64_t begin = agfx_topology_seq_read(topology);
    void *ptr;

    *rejected = NULL;
    if (begin & 1) {
        return NULL;
    }
    ptr = map(gpa, size, writable);
    if (!ptr) {
        return NULL;
    }
    smp_rmb();
    if (qatomic_read(&topology->seq) != begin) {
        *rejected = ptr;
        return NULL;
    }
    *sequence = begin;
    return ptr;
}

static inline void *agfx_memory_capture_retained(AgfxTopologySeq *topology,
                                                 uint64_t gpa, size_t size,
                                                 int writable,
                                                 uint64_t *sequence,
                                                 void **rejected)
{
    return agfx_memory_capture_retained_with(topology, gpa, size, writable,
                                             sequence, rejected,
                                             agfx_memory_map_gpa_any_thread);
}

#endif /* HW_DISPLAY_APPLE_GFX_ML_MEMORY_H */
