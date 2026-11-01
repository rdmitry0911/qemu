/* Source-first real QEMU comparator controls; proposed companion ABI held.
 * Real MR/memfd/V2 spans only. No Session/GPU/VM or mirrored span layout. */
#include "qemu/osdep.h"
#include "qemu/aio-wait.h"
#include "qemu/main-loop.h"
#include "qemu/memfd.h"
#include "qemu/rcu.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/ram_addr.h"
#include "system/replay.h"
#include "system/system.h"
#include "hw/display/apple-gfx-ml.h"
#include "hw/display/apple-gfx-ordinary-physical-capture.h"
#include "hw/display/apple-gfx-ml-memory.h"
#include "hw/display/qmu_ordinary_physical_comparison_v1.h"
#include <sys/stat.h>
#include <linux/dma-buf.h>
#include <linux/memfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#define TYPE_PHYSICAL_RAM "test-ordinary-comparison-ram"
#define GPA UINT64_C(0x10000000)
int (*qemu_main)(void);

typedef struct PhysicalRam {
    Object parent;
    MemoryRegion ram;
    unsigned *finalized;
} PhysicalRam;

static void ram_finalize(Object *object)
{
    PhysicalRam *ram = (PhysicalRam *)object;
    g_assert_true(bql_locked());
    g_assert_cmpuint((*ram->finalized)++, ==, 0);
}

static const TypeInfo ram_type = {
    .name = TYPE_PHYSICAL_RAM, .parent = TYPE_OBJECT,
    .instance_size = sizeof(PhysicalRam), .instance_finalize = ram_finalize,
};

static void gfx_finalize(Object *object, const char *name, void *opaque)
{
    unsigned *finalized = opaque;
    g_assert_true(bql_locked());
    g_assert_cmpuint((*finalized)++, ==, 0);
}

static PhysicalRam *new_ram(uint64_t page, unsigned *finalized, bool sealed,
                            uint8_t value)
{
    PhysicalRam *ram = (PhysicalRam *)object_new(TYPE_PHYSICAL_RAM);
    int fd = qemu_memfd_create("ordinary-v2-real-pages", 4 * page, false, 0,
        sealed ? F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL : 0, &error_abort);
    ram->finalized = finalized;
    g_assert_cmpint(fd, >=, 0);
    g_assert_true(memory_region_init_ram_from_fd(&ram->ram, OBJECT(ram),
        "ordinary-v2-real-ram", 3 * page, RAM_SHARED, fd, page, &error_abort));
    memset(memory_region_get_ram_ptr(&ram->ram), value, 3 * page);
    return ram;
}

typedef struct CaptureWork {
    struct qmu_ordinary_physical_handoff_v2 caps;
    struct qmu_ordinary_physical_request_v2 request;
    void *output;
    bool registered_nested;
} CaptureWork;

static void *capture_worker(void *opaque)
{
    CaptureWork *work = opaque;
    /* Do not hide a missing producer registration with fixture setup. */
    g_assert_false(rcu_thread_is_registered());
    g_assert_cmpuint(get_ptr_rcu_reader()->depth, ==, 0);
    if (work->registered_nested) {
        rcu_register_thread();
        rcu_read_lock();
    }
    work->caps.capture(work->caps.acquired_context, &work->request, &work->output);
    if (work->registered_nested) {
        g_assert_true(rcu_thread_is_registered());
        g_assert_cmpuint(get_ptr_rcu_reader()->depth, ==, 1);
        rcu_read_unlock();
        g_assert_cmpuint(get_ptr_rcu_reader()->depth, ==, 0);
        rcu_unregister_thread();
    }
    g_assert_false(rcu_thread_is_registered());
    g_assert_cmpuint(get_ptr_rcu_reader()->depth, ==, 0);
    return NULL;
}

static void capture(CaptureWork *work)
{
    QemuThread thread;
    qemu_thread_create(&thread, "physical-capture", capture_worker, work,
                       QEMU_THREAD_JOINABLE);
    /* BQL stays held: capture may not synchronously dispatch back to main. */
    qemu_thread_join(&thread);
}

typedef struct ReleaseWork {
    void (*release)(void *);
    void *span;
    int done;
} ReleaseWork;

static void *release_worker(void *opaque)
{
    ReleaseWork *work = opaque;
    g_assert_false(bql_locked());
    work->release(work->span);
    qatomic_set(&work->done, 1);
    aio_wait_kick();
    return NULL;
}

static void release_span(ReleaseWork *work)
{
    QemuThread thread;
    qemu_thread_create(&thread, "physical-release", release_worker, work,
                       QEMU_THREAD_JOINABLE);
    const gint64 until = g_get_monotonic_time() + 5 * G_USEC_PER_SEC;
    while (!qatomic_read(&work->done)) {
        g_assert_cmpint(g_get_monotonic_time(), <, until);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&thread);
}

static void drain(void)
{
    bql_unlock();
    drain_call_rcu();
    bql_lock();
}


/* Every comparator call runs off-BQL while main keeps BQL, just like capture.
 * No owner/renderer/RT mutex is held by this fixture worker. */
typedef struct CompareWork {
    const void *context, *left, *right;
    uint64_t left_offset, left_length, right_offset, right_length;
    uint32_t result;
} CompareWork;

static void *compare_worker(void *opaque)
{
    CompareWork *work = opaque;
    g_assert_false(bql_locked());
    work->result = agfx_ordinary_physical_compare(work->context,
        work->left, work->left_offset, work->left_length,
        work->right, work->right_offset, work->right_length);
    return NULL;
}

static void expect_compare(const void *context, const void *left,
    uint64_t lo, uint64_t ll, const void *right, uint64_t ro, uint64_t rl,
    uint32_t expected)
{
    CompareWork work = { context, left, right, lo, ll, ro, rl, UINT32_MAX };
    QemuThread thread;
    qemu_thread_create(&thread, "physical-compare", compare_worker, &work,
                       QEMU_THREAD_JOINABLE);
    qemu_thread_join(&thread);
    g_assert_cmpuint(work.result, ==, expected);
}

static void *capture_range(struct qmu_ordinary_physical_handoff_v2 caps,
    uint64_t page, uint64_t gpa, bool reverse, uint64_t visible,
    uint64_t literal, bool ready)
{
    uint64_t first = gpa + (reverse ? 0 : 2 * page);
    uint64_t second = gpa + (reverse ? 2 * page : 0);
    struct qmu_ordinary_physical_piece_v2 pieces[] = {
        { first, 0, page, agfx_memory_map_gpa(first, page, 0) },
        { second, page, page, agfx_memory_map_gpa(second, page, 0) },
    };
    g_assert_nonnull(pieces[0].mapped_pointer);
    g_assert_nonnull(pieces[1].mapped_pointer);
    CaptureWork work = { .caps = caps, .request = {
        .version = QMU_PRIVATE_ORDINARY_PHYSICAL_CAPTURE_V2_VERSION,
        .struct_size = sizeof(work.request), .task_id = 2, .resource_id = 30,
        .task_epoch = 1, .object_epoch = 1, .source_va = 0x20000,
        .allocation_length = 2 * page, .visible_offset = visible,
        .literal_length = literal, .allocation_alignment = page,
        .piece_count = ARRAY_SIZE(pieces), .pieces = pieces,
    } };
    capture(&work);
    g_assert_nonnull(work.output);
    struct qmu_ordinary_physical_view_v2 view = { 0 };
    g_assert_cmpint(caps.describe(work.output, &view), ==, ready);
    agfx_memory_unmap_gpa((void *)pieces[0].mapped_pointer, page, 0);
    agfx_memory_unmap_gpa((void *)pieces[1].mapped_pointer, page, 0);
    return work.output;
}

static void release_one(struct qmu_ordinary_physical_handoff_v2 caps, void *span)
{
    ReleaseWork returned = { .release = caps.release_span, .span = span };
    release_span(&returned);
}

static void real_physical_comparison(void)
{
    const uint64_t page = qemu_real_host_page_size();
    const uint64_t literal = 2 * page - 34;
    const uint64_t partial_gpa = GPA + 0x100000;
    unsigned old_final = 0, next_final = 0, partial_final = 0;
    unsigned gfx_final = 0, other_final = 0;
    AppleGfxMLState *state = APPLE_GFX_ML(object_new(TYPE_APPLE_GFX_ML));
    AppleGfxMLState *other = APPLE_GFX_ML(object_new(TYPE_APPLE_GFX_ML));
    object_property_add(OBJECT(state), "comparison-finalize", "bool", NULL,
                        NULL, gfx_finalize, &gfx_final);
    object_property_add(OBJECT(other), "comparison-finalize", "bool", NULL,
                        NULL, gfx_finalize, &other_final);
    AgfxOrdinaryPhysicalContext *context = agfx_ordinary_physical_context_create(state);
    AgfxOrdinaryPhysicalContext *other_context = agfx_ordinary_physical_context_create(other);
    struct qmu_ordinary_physical_handoff_v2 caps = { 0 }, other_caps = { 0 };
    g_assert_true(agfx_ordinary_physical_context_acquire(context, &caps));
    g_assert_true(agfx_ordinary_physical_context_acquire(other_context, &other_caps));
    PhysicalRam *old = new_ram(page, &old_final, true, 0x47);
    PhysicalRam *next = new_ram(page, &next_final, true, 0x92);
    PhysicalRam *partial = new_ram(page, &partial_final, false, 0x31);
    memory_region_add_subregion(get_system_memory(), GPA, &old->ram);
    void *a = capture_range(caps, page, GPA, false, 17, literal, true);
    void *b = capture_range(caps, page, GPA, false, 17, literal, true);
    void *reordered = capture_range(caps, page, GPA, true, 17, literal, true);
    void *prefix = capture_range(caps, page, GPA, false, 0, 17, true);
    void *suffix = capture_range(caps, page, GPA, false, 2 * page - 17, 17, true);
    struct qmu_ordinary_physical_view_v2 av = { 0 }, bv = { 0 };
    struct stat ast, bst;
    g_assert_true(caps.describe(a, &av));
    g_assert_true(caps.describe(b, &bv));
    g_assert_cmpint(fstat(av.dma_buf_fd, &ast), ==, 0);
    g_assert_cmpint(fstat(bv.dma_buf_fd, &bst), ==, 0);
    /* Separate real udmabuf exports are NOT a backing equality oracle. */
    g_assert_true(ast.st_dev != bst.st_dev || ast.st_ino != bst.st_ino);
    const void *ctx = caps.acquired_context;
    expect_compare(ctx, a, 0, 32, b, 0, 32,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_OVERLAP);
    expect_compare(ctx, a, 0, 31, b, 31, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_DISJOINT);
    expect_compare(ctx, a, 0, 32, b, 31, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_OVERLAP);
    expect_compare(ctx, a, 0, page - 17, b, page - 17, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_DISJOINT);
    expect_compare(ctx, a, page, 32, reordered, 0, 32,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_OVERLAP);
    expect_compare(ctx, a, 0, 32, reordered, 0, 32,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_DISJOINT);
    /* Aligned storage is identical but only each declared literal participates. */
    expect_compare(ctx, a, 0, literal, prefix, 0, 17,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_DISJOINT);
    expect_compare(ctx, a, 0, literal, suffix, 0, 17,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_DISJOINT);
    memory_region_del_subregion(get_system_memory(), &old->ram);
    memory_region_add_subregion(get_system_memory(), GPA, &next->ram);
    void *fresh = capture_range(caps, page, GPA, false, 17, literal, true);
    expect_compare(ctx, a, 0, literal, fresh, 0, literal,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_DISJOINT);
    memory_region_add_subregion(get_system_memory(), partial_gpa, &partial->ram);
    void *incomplete = capture_range(caps, page, partial_gpa, false, 17,
                                     literal, false);
    expect_compare(ctx, incomplete, 0, 1, a, 0, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_UNAVAILABLE);
    expect_compare(ctx, a, 0, 1, incomplete, 0, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_UNAVAILABLE);
    expect_compare(other_caps.acquired_context, a, 0, 1, b, 0, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_UNAVAILABLE);
    expect_compare(NULL, a, 0, 1, b, 0, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_UNAVAILABLE);
    expect_compare(ctx, NULL, 0, 1, b, 0, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_UNAVAILABLE);
    expect_compare(ctx, a, UINT64_MAX, 2, b, 0, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_UNAVAILABLE);
    expect_compare(ctx, a, 0, UINT64_MAX, b, 0, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_UNAVAILABLE);
    expect_compare(ctx, a, literal, 1, b, 0, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_UNAVAILABLE);
    expect_compare(ctx, a, 0, literal + 1, b, 0, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_UNAVAILABLE);
    memory_region_del_subregion(get_system_memory(), &next->ram);
    memory_region_del_subregion(get_system_memory(), &partial->ram);
    object_unref(OBJECT(old)); object_unref(OBJECT(next)); object_unref(OBJECT(partial));
    drain();
    g_assert_cmpuint(old_final, ==, 0);
    g_assert_cmpuint(next_final, ==, 0);
    g_assert_cmpuint(partial_final, ==, 0);
    /* Stop and return the actual Context while genuine captured spans still
     * retain their independent origin token and MR/FD owners. ctx below is
     * only the original opaque value: comparator must not dereference the
     * retired Context or accept a stopped token as a new generation. */
    g_assert_true(caps.stop(caps.acquired_context));
    caps.release_context(caps.acquired_context);
    agfx_ordinary_physical_context_retire(context);
    expect_compare(ctx, a, 0, 1, b, 0, 1,
                   QMU_ORDINARY_PHYSICAL_COMPARISON_UNAVAILABLE);
    g_assert_cmpuint(old_final, ==, 0);
    g_assert_cmpuint(next_final, ==, 0);
    g_assert_cmpuint(partial_final, ==, 0);
    void *old_spans[] = { a, b, reordered, prefix, suffix };
    for (unsigned i = 0; i < ARRAY_SIZE(old_spans); ++i) {
        release_one(caps, old_spans[i]);
        g_assert_cmpuint(old_final, ==, i + 1 == ARRAY_SIZE(old_spans));
    }
    release_one(caps, fresh); g_assert_cmpuint(next_final, ==, 1);
    release_one(caps, incomplete); g_assert_cmpuint(partial_final, ==, 1);
    g_assert_true(other_caps.stop(other_caps.acquired_context));
    other_caps.release_context(other_caps.acquired_context);
    agfx_ordinary_physical_context_retire(other_context);
    object_unref(OBJECT(state)); object_unref(OBJECT(other));
    g_assert_cmpuint(gfx_final, ==, 1);
    g_assert_cmpuint(other_final, ==, 1);
    g_test_message("ORDINARY_COMPARISON_RETURNED realMR=1 realMemfd=1 offBQL=1 originalOwners_RETURN=1 GPU=0 VM=0");
}

int main(int argc, char **argv)
{
    char options[][16] = { "-machine", "none", "-accel", "tcg", "-nodefaults",
                           "-display", "none", "-S" };
    char *qemu_argv[] = { argv[0], options[0], options[1], options[2], options[3],
                         options[4], options[5], options[6], options[7], NULL };
    g_test_init(&argc, &argv, NULL);
    qemu_init(ARRAY_SIZE(qemu_argv) - 1, qemu_argv);
    type_register_static(&ram_type);
    g_test_add_func("/ordinary-comparison/real-backing-visible-ranges",
                    real_physical_comparison);
    int status = g_test_run();
    qemu_cleanup(status);
    bql_unlock(); replay_mutex_unlock();
    return status;
}
