/*
 * Captured source witness, not a production fallback.
 * The typedef and agfx_mark_mapped_dirty_bh below are unchanged from QEMU
 * main2e18af2c89740b6f68267c5e7228c71da3ae63f8,
 * hw/display/apple-gfx-ml.c:760-788. Verify against that Git object.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
typedef struct AgfxMappedDirtyJob {
    void *hva;
    size_t size;
    int result;
    QemuEvent event;
} AgfxMappedDirtyJob;

static void agfx_mark_mapped_dirty_bh(void *opaque)
{
    AgfxMappedDirtyJob *job = opaque;
    if (!job || !job->hva || job->size == 0) {
        if (job) {
            job->result = -1;
            qemu_event_set(&job->event);
        }
        return;
    }

    ram_addr_t offset;
    MemoryRegion *mr = memory_region_from_host(job->hva, &offset);
    if (!mr) {
        job->result = -1;
    } else {
        memory_region_set_dirty(mr, offset, job->size);
        memory_region_unref(mr);
        job->result = 0;
    }
    qemu_event_set(&job->event);
}

/* Native controls hold BQL, the same context in which the original BH runs.
 * The first refcount assertion fails before a second notice could consume a
 * different live owner. No unsafe historical loop or GPU work is executed. */
static int agfx_memory_historical_dirty(void *hva, size_t size)
{
    AgfxMappedDirtyJob job = { .hva = hva, .size = size, .result = -1 };
    int result;

    qemu_event_init(&job.event, false);
    agfx_mark_mapped_dirty_bh(&job);
    result = job.result;
    qemu_event_destroy(&job.event);
    return result;
}
