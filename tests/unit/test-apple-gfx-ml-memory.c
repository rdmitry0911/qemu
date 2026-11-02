/*
 * Direct-map lifetime controls against actual QEMU RAM/QOM and dirty tracking.
 * No graphics device, QMetal factory, guest disk or GPU command is created.
 * Fixture initialization and RAM/alias setup reuse native process-VM controls
 * from 918042817ab90aa26285f69ca67875c89ec5cd92.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/rcu.h"
#include "qemu/thread.h"
#include "system/ram_addr.h"
#include "system/replay.h"
#include "system/system.h"
#include "hw/display/apple-gfx-ml-memory.h"

#ifdef AGFX_MEMORY_BASELINE_CONTROL
#include "test-apple-gfx-ml-memory-baseline.h"
#define agfx_memory_mark_mapped_dirty agfx_memory_historical_dirty
#endif

#define TYPE_MEMORY_OWNER "test-agfx-memory-owner"
#define RAM_GPA UINT64_C(0x10000000)
#define ALIAS_A_GPA UINT64_C(0x20000000)
#define ALIAS_B_GPA UINT64_C(0x30000000)
#define ROM_GPA UINT64_C(0x40000000)

typedef struct NativeMemoryOwner {
    Object parent_obj;
    MemoryRegion ram;
    MemoryRegion alias_a;
    MemoryRegion alias_b;
    MemoryRegion rom;
    bool *finalized;
} NativeMemoryOwner;

typedef struct Fixture {
    NativeMemoryOwner *owner;
    uint64_t page;
    uint32_t baseline_refs;
} Fixture;

int (*qemu_main)(void);

static void native_owner_finalize(Object *object)
{
    NativeMemoryOwner *owner = (NativeMemoryOwner *)object;

    if (owner->finalized) {
        *owner->finalized = true;
    }
}

static const TypeInfo native_owner_type = {
    .name = TYPE_MEMORY_OWNER,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(NativeMemoryOwner),
    .instance_finalize = native_owner_finalize,
};

static void drain_flatviews(void)
{
    bql_unlock();
    drain_call_rcu();
    bql_lock();
}

static Fixture *fixture_new(void)
{
    Fixture *f = g_new0(Fixture, 1);
    NativeMemoryOwner *owner;

    f->page = qemu_real_host_page_size();
    f->owner = owner = (NativeMemoryOwner *)object_new(TYPE_MEMORY_OWNER);
    g_assert_true(memory_region_init_ram_nomigrate(
        &owner->ram, OBJECT(owner), "agfx-memory-ram", 4 * f->page,
        &error_abort));
    memset(memory_region_get_ram_ptr(&owner->ram), 0x47, 4 * f->page);
    memory_region_add_subregion(get_system_memory(), RAM_GPA, &owner->ram);
    memory_region_init_alias(&owner->alias_a, OBJECT(owner), "agfx-memory-a",
                             &owner->ram, f->page, f->page);
    memory_region_init_alias(&owner->alias_b, OBJECT(owner), "agfx-memory-b",
                             &owner->ram, f->page, f->page);
    memory_region_add_subregion(get_system_memory(), ALIAS_A_GPA,
                                &owner->alias_a);
    memory_region_add_subregion(get_system_memory(), ALIAS_B_GPA,
                                &owner->alias_b);
    g_assert_true(memory_region_init_rom_nomigrate(
        &owner->rom, OBJECT(owner), "agfx-memory-rom", f->page, &error_abort));
    memset(memory_region_get_ram_ptr(&owner->rom), 0x63, f->page);
    memory_region_add_subregion(get_system_memory(), ROM_GPA, &owner->rom);
    drain_flatviews();
    f->baseline_refs = OBJECT(owner)->ref;
    return f;
}

static void fixture_detach(Fixture *f)
{
    NativeMemoryOwner *owner = f->owner;

    memory_region_del_subregion(get_system_memory(), &owner->rom);
    memory_region_del_subregion(get_system_memory(), &owner->alias_b);
    memory_region_del_subregion(get_system_memory(), &owner->alias_a);
    memory_region_del_subregion(get_system_memory(), &owner->ram);
    drain_flatviews();
}

static void fixture_free(Fixture *f)
{
    g_assert_cmpuint(OBJECT(f->owner)->ref, ==, f->baseline_refs);
    fixture_detach(f);
    object_unref(OBJECT(f->owner));
    drain_flatviews();
    g_free(f);
}

static void test_repeated_dirty_preserves_mapping_pin(void)
{
    Fixture *f = fixture_new();
    Object *owner = OBJECT(f->owner);
    uint8_t *mapped = agfx_memory_map_gpa(RAM_GPA + f->page, f->page, 1);
    uint8_t *ram = memory_region_get_ram_ptr(&f->owner->ram);

    g_assert_true(mapped == ram + f->page);
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs + 1);
    mapped[7] = 0x91;
    for (unsigned i = 0; i < 128; ++i) {
        g_assert_cmpint(agfx_memory_mark_mapped_dirty(mapped, f->page), ==, 0);
        g_assert_cmpuint(owner->ref, ==, f->baseline_refs + 1);
        g_assert_cmpuint(ram[f->page + 7], ==, 0x91);
    }
    agfx_memory_unmap_gpa(mapped, f->page, 0);
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs);
    fixture_free(f);
}

static void test_canonical_aliases_and_dirty_span(void)
{
    Fixture *f = fixture_new();
    Object *owner = OBJECT(f->owner);
    uint8_t *a, *b;
    ram_addr_t offset;
    DirtyBitmapSnapshot *snap;

    /* Logging changes the FlatView graph. Retire its old region references
     * before measuring the two independent mapping pins. */
    memory_region_set_log(&f->owner->ram, true, DIRTY_MEMORY_VGA);
    drain_flatviews();
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs);
    memory_region_reset_dirty(&f->owner->ram, 0, 4 * f->page, DIRTY_MEMORY_VGA);
    a = agfx_memory_map_gpa(ALIAS_A_GPA, f->page, 1);
    b = agfx_memory_map_gpa(ALIAS_B_GPA, f->page, 1);
    g_assert_nonnull(a);
    g_assert_true(a == b);
    g_assert_true(memory_region_from_host(a, &offset) == &f->owner->ram);
    g_assert_cmpuint(offset, ==, f->page);
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs + 2);
    a[31] = 0x72;
    g_assert_cmpuint(b[31], ==, 0x72);
    g_assert_cmpint(agfx_memory_mark_mapped_dirty(a + 31, 64), ==, 0);
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs + 2);
    snap = memory_region_snapshot_and_clear_dirty(&f->owner->ram, 0,
                                                   4 * f->page,
                                                   DIRTY_MEMORY_VGA);
    for (unsigned i = 0; i < 4; ++i) {
        g_assert_cmpint(memory_region_snapshot_get_dirty(&f->owner->ram, snap,
                         i * f->page, f->page), ==, i == 1);
    }
    g_free(snap);
    agfx_memory_unmap_gpa(a, f->page, 0);
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs + 1);
    g_assert_cmpint(agfx_memory_mark_mapped_dirty(b, f->page), ==, 0);
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs + 1);
    agfx_memory_unmap_gpa(b, f->page, 1);
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs);
    snap = memory_region_snapshot_and_clear_dirty(&f->owner->ram, 0,
                                                   4 * f->page,
                                                   DIRTY_MEMORY_VGA);
    g_assert_true(memory_region_snapshot_get_dirty(&f->owner->ram, snap,
                                                   f->page, f->page));
    g_free(snap);
    memory_region_set_log(&f->owner->ram, false, DIRTY_MEMORY_VGA);
    drain_flatviews();
    fixture_free(f);
}

static void test_admission_and_rom(void)
{
    Fixture *f = fixture_new();
    Object *owner = OBJECT(f->owner);
    void *rom;

    g_assert_null(agfx_memory_map_gpa(ALIAS_A_GPA, 2 * f->page, 1));
    g_assert_null(agfx_memory_map_gpa(ROM_GPA, f->page, 1));
    g_assert_null(agfx_memory_map_gpa(UINT64_C(0x50000000), f->page, 1));
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs);
    rom = agfx_memory_map_gpa(ROM_GPA, f->page, 0);
    g_assert_true(rom == memory_region_get_ram_ptr(&f->owner->rom));
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs + 1);
    agfx_memory_unmap_gpa(rom, f->page, 0);
    g_assert_cmpint(agfx_memory_mark_mapped_dirty(NULL, f->page), ==, -1);
    g_assert_cmpint(agfx_memory_mark_mapped_dirty(rom, 0), ==, -1);
    fixture_free(f);
}

static void test_detached_owner_survives_until_unmap(void)
{
    Fixture *f = fixture_new();
    bool finalized = false;
    uint8_t *mapped = agfx_memory_map_gpa(ALIAS_A_GPA, f->page, 1);

    g_assert_nonnull(mapped);
    f->owner->finalized = &finalized;
    fixture_detach(f);
    object_unref(OBJECT(f->owner));
    g_assert_false(finalized);
    g_assert_cmpuint(OBJECT(f->owner)->ref, ==, 1);
    mapped[19] = 0x85;
    for (unsigned i = 0; i < 16; ++i) {
        g_assert_cmpint(agfx_memory_mark_mapped_dirty(mapped, f->page), ==, 0);
        g_assert_false(finalized);
        g_assert_cmpuint(OBJECT(f->owner)->ref, ==, 1);
        g_assert_cmpuint(mapped[19], ==, 0x85);
    }
    agfx_memory_unmap_gpa(mapped, f->page, 0);
    g_assert_true(finalized);
    drain_flatviews();
    g_free(f);
}

#ifndef AGFX_MEMORY_BASELINE_CONTROL
/* A QMetal worker is a plain thread: neither BQL holder nor a QEMU-created
 * RCU reader. The main thread keeps BQL for the whole interval and only joins,
 * so any hidden BH/BQL edge in either operation would deadlock this case. */
typedef struct ForeignDirty {
    Fixture *f;
    uint8_t *mapped;
    bool registered_before;
    bool registered_inside;
    bool registered_after;
    bool held_bql;
    int failures;
} ForeignDirty;

static void *foreign_dirty_thread(void *opaque)
{
    ForeignDirty *job = opaque;

    job->registered_before = rcu_thread_is_registered();
    job->held_bql = bql_locked();
    agfx_memory_rcu_enter();
    agfx_memory_rcu_enter();
    job->registered_inside = rcu_thread_is_registered();
    agfx_memory_rcu_leave();
    job->registered_inside &= rcu_thread_is_registered();
    agfx_memory_rcu_leave();
    job->mapped = agfx_memory_map_gpa_any_thread(RAM_GPA + 2 * job->f->page,
                                                 job->f->page, 1);
    if (job->mapped) {
        job->mapped[11] = 0x5a;
        for (unsigned i = 0; i < 256; ++i) {
            if (agfx_memory_mark_mapped_dirty_any_thread(job->mapped + 8,
                                                         32) != 0) {
                job->failures++;
            }
        }
    }
    job->registered_after = rcu_thread_is_registered();
    return NULL;
}

static void test_foreign_thread_map_and_dirty_without_bql(void)
{
    Fixture *f = fixture_new();
    Object *owner = OBJECT(f->owner);
    uint8_t *ram = memory_region_get_ram_ptr(&f->owner->ram);
    ForeignDirty job = { .f = f };
    DirtyBitmapSnapshot *snap;
    QemuThread thread;

    memory_region_set_log(&f->owner->ram, true, DIRTY_MEMORY_VGA);
    drain_flatviews();
    memory_region_reset_dirty(&f->owner->ram, 0, 4 * f->page, DIRTY_MEMORY_VGA);
    g_assert_true(bql_locked());
    qemu_thread_create(&thread, "agfx-foreign", foreign_dirty_thread, &job,
                       QEMU_THREAD_JOINABLE);
    qemu_thread_join(&thread);
    g_assert_false(job.registered_before);
    g_assert_false(job.held_bql);
    g_assert_true(job.registered_inside);
    g_assert_false(job.registered_after);
    g_assert_true(job.mapped == ram + 2 * f->page);
    g_assert_cmpint(job.failures, ==, 0);
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs + 1);
    g_assert_cmpuint(ram[2 * f->page + 11], ==, 0x5a);
    snap = memory_region_snapshot_and_clear_dirty(&f->owner->ram, 0,
                                                   4 * f->page,
                                                   DIRTY_MEMORY_VGA);
    for (unsigned i = 0; i < 4; ++i) {
        g_assert_cmpint(memory_region_snapshot_get_dirty(&f->owner->ram, snap,
                         i * f->page, f->page), ==, i == 2);
    }
    g_free(snap);
    agfx_memory_unmap_gpa(job.mapped, f->page, 0);
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs);
    memory_region_set_log(&f->owner->ram, false, DIRTY_MEMORY_VGA);
    drain_flatviews();
    fixture_free(f);
}

/* Seqlock over publication: a listener's region_add runs between every
 * listener's begin and commit, so it must observe an odd sequence. */
static uint64_t probe_seen_seq;
static AgfxTopologySeq *probe_topology;
static void probe_region_add(MemoryListener *listener,
                             MemoryRegionSection *section)
{
    if (probe_topology) {
        probe_seen_seq = agfx_topology_seq_read(probe_topology);
    }
}

static void test_topology_seq_brackets_publication(void)
{
    Fixture *f = fixture_new();
    AgfxTopologySeq topology = { 0 };
    MemoryListener probe = { .name = "agfx-probe",
                             .region_add = probe_region_add,
                             .priority = MEMORY_LISTENER_PRIORITY_MIN };
    MemoryRegion extra;
    uint64_t before;

    agfx_topology_seq_register(&topology, "agfx-test-topology");
    before = agfx_topology_seq_read(&topology);
    g_assert_cmpuint(before & 1, ==, 0);
    memory_listener_register(&probe, &address_space_memory);
    g_assert_cmpuint(agfx_topology_seq_read(&topology), ==, before);
    probe_topology = &topology;
    probe_seen_seq = 0;
    g_assert_true(memory_region_init_ram_nomigrate(
        &extra, OBJECT(f->owner), "agfx-memory-extra", f->page, &error_abort));
    memory_region_add_subregion(get_system_memory(), UINT64_C(0x60000000),
                                &extra);
    g_assert_cmpuint(probe_seen_seq & 1, ==, 1);
    g_assert_cmpuint(agfx_topology_seq_read(&topology), ==, before + 2);
    memory_region_del_subregion(get_system_memory(), &extra);
    g_assert_cmpuint(agfx_topology_seq_read(&topology), ==, before + 4);
    probe_topology = NULL;
    memory_listener_unregister(&probe);
    agfx_topology_seq_unregister(&topology);
    drain_flatviews();
    object_unparent(OBJECT(&extra));
    fixture_free(f);
}

/* Retained capture: refused during a publication; an accepted alias keeps
 * its backing after the GPA is republished to a different RAM region. */
static void test_capture_retained_keeps_backing(void)
{
    Fixture *f = fixture_new();
    Object *owner = OBJECT(f->owner);
    AgfxTopologySeq topology = { 0 };
    uint8_t *ram = memory_region_get_ram_ptr(&f->owner->ram);
    MemoryRegion replacement;
    uint64_t seq_a = 0, seq_b = 0;
    void *rejected = (void *)1;
    uint8_t *a, *b;

    agfx_topology_seq_register(&topology, "agfx-test-topology");
    qatomic_inc(&topology.seq);  /* publication in progress */
    g_assert_null(agfx_memory_capture_retained(&topology, ALIAS_A_GPA,
                                               f->page, 1, &seq_a, &rejected));
    g_assert_null(rejected);
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs);
    qatomic_inc(&topology.seq);

    a = agfx_memory_capture_retained(&topology, ALIAS_A_GPA, f->page, 1,
                                     &seq_a, &rejected);
    g_assert_true(a == ram + f->page);
    g_assert_null(rejected);
    g_assert_cmpuint(seq_a, ==, agfx_topology_seq_read(&topology));
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs + 1);

    /* Republish GPA ALIAS_A_GPA onto a different RAM region. */
    g_assert_true(memory_region_init_ram_nomigrate(
        &replacement, OBJECT(f->owner), "agfx-memory-b", f->page,
        &error_abort));
    memory_region_transaction_begin();
    memory_region_del_subregion(get_system_memory(), &f->owner->alias_a);
    memory_region_add_subregion(get_system_memory(), ALIAS_A_GPA,
                                &replacement);
    memory_region_transaction_commit();
    g_assert_cmpuint(agfx_topology_seq_read(&topology), ==, seq_a + 2);

    b = agfx_memory_capture_retained(&topology, ALIAS_A_GPA, f->page, 1,
                                     &seq_b, &rejected);
    g_assert_true(b == memory_region_get_ram_ptr(&replacement));
    g_assert_cmpuint(seq_b, ==, seq_a + 2);
    a[5] = 0xa5;  /* retained alias still writes backing A */
    g_assert_cmpuint(ram[f->page + 5], ==, 0xa5);
    g_assert_cmpuint(b[5], !=, 0xa5);

    agfx_memory_unmap_gpa(b, f->page, 0);
    agfx_memory_unmap_gpa(a, f->page, 0);
    memory_region_transaction_begin();
    memory_region_del_subregion(get_system_memory(), &replacement);
    memory_region_add_subregion(get_system_memory(), ALIAS_A_GPA,
                                &f->owner->alias_a);
    memory_region_transaction_commit();
    agfx_topology_seq_unregister(&topology);
    drain_flatviews();
    object_unparent(OBJECT(&replacement));
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs);
    fixture_free(f);
}
/* A publication completing after the map acquired its pin and before the
 * final sequence comparison: the capture is rejected and the exact acquired
 * alias is handed back for exactly one matching release. */
static MemoryRegion *overlap_region;
static void *overlap_map(uint64_t gpa, size_t size, int writable)
{
    void *ptr = agfx_memory_map_gpa_any_thread(gpa, size, writable);

    memory_region_add_subregion(get_system_memory(), UINT64_C(0x70000000),
                                overlap_region);  /* begin..commit inside */
    return ptr;
}

static void test_capture_retained_overlap_rejected(void)
{
    Fixture *f = fixture_new();
    Object *owner = OBJECT(f->owner);
    AgfxTopologySeq topology = { 0 };
    uint8_t *ram = memory_region_get_ram_ptr(&f->owner->ram);
    MemoryRegion extra;
    uint64_t sequence = 0, before;
    uint32_t with_pin;
    void *rejected = NULL;

    g_assert_true(memory_region_init_ram_nomigrate(
        &extra, OBJECT(f->owner), "agfx-memory-overlap", f->page,
        &error_abort));
    overlap_region = &extra;
    agfx_topology_seq_register(&topology, "agfx-test-topology");
    before = agfx_topology_seq_read(&topology);
    g_assert_null(agfx_memory_capture_retained_with(
        &topology, ALIAS_A_GPA, f->page, 1, &sequence, &rejected,
        overlap_map));
    g_assert_true(rejected == ram + f->page);
    g_assert_cmpuint(sequence, ==, 0);
    g_assert_cmpuint(agfx_topology_seq_read(&topology), ==, before + 2);
    /* The new FlatView also references this owner; measure the pin alone. */
    with_pin = owner->ref;
    agfx_memory_unmap_gpa(rejected, f->page, 0);  /* the caller's release */
    g_assert_cmpuint(owner->ref, ==, with_pin - 1);
    memory_region_del_subregion(get_system_memory(), &extra);
    agfx_topology_seq_unregister(&topology);
    drain_flatviews();
    object_unparent(OBJECT(&extra));
    drain_flatviews();
    g_assert_cmpuint(owner->ref, ==, f->baseline_refs);
    fixture_free(f);
}
#endif

int main(int argc, char **argv)
{
    char options[][16] = {"-machine", "none", "-accel", "tcg", "-nodefaults",
                         "-display", "none", "-S"};
    char *qemu_argv[] = {argv[0], options[0], options[1], options[2], options[3],
                        options[4], options[5], options[6], options[7], NULL};
    int status;

    g_test_init(&argc, &argv, NULL);
    qemu_init(ARRAY_SIZE(qemu_argv) - 1, qemu_argv);
    type_register_static(&native_owner_type);
    g_test_add_func("/agfx/memory/repeated-dirty-keeps-pin",
                    test_repeated_dirty_preserves_mapping_pin);
    g_test_add_func("/agfx/memory/canonical-alias-dirty-span",
                    test_canonical_aliases_and_dirty_span);
    g_test_add_func("/agfx/memory/admission-rom", test_admission_and_rom);
    g_test_add_func("/agfx/memory/detached-owner-last-pin",
                    test_detached_owner_survives_until_unmap);
#ifndef AGFX_MEMORY_BASELINE_CONTROL
    g_test_add_func("/agfx/memory/foreign-thread-map-dirty-without-bql",
                    test_foreign_thread_map_and_dirty_without_bql);
    g_test_add_func("/agfx/memory/topology-seq-brackets-publication",
                    test_topology_seq_brackets_publication);
    g_test_add_func("/agfx/memory/capture-retained-keeps-backing",
                    test_capture_retained_keeps_backing);
    g_test_add_func("/agfx/memory/capture-retained-overlap-rejected",
                    test_capture_retained_overlap_rejected);
#endif
    status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
