/*
 * Actual normal AppleGfxML MemoryRegion.read and ThreadPoolAIO controls.
 * Real failed cold-plug leaves an externally retained, initialized MMIO MR.
 * A unit-only opaque getter is attached to that MR, not a successful factory
 * or native/GPU session. No production leaf is copied into this fixture.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/rcu.h"
#include "block/thread-pool.h"
#include "system/address-spaces.h"
#include "system/replay.h"
#include "system/system.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_host.h"
#include "hw/pci/msi.h"
#include "hw/display/apple-gfx-ml.h"
#include "qmu/qmetal_unified.h"

#define TYPE_TEST_HOST "test-agfx-read-host"

typedef struct ReadControl {
    AppleGfxMLState *state;
    qmu_session *opaque_session;
    const MemoryRegionOps *candidate_ops;
    const AIOCBInfo *request_type;
    unsigned accepted;
    unsigned reaped;
    unsigned pending;
    unsigned getter_calls;
    unsigned expected_offset;
    unsigned expected_size;
    unsigned held_release;
    unsigned foreign_release;
    uint32_t sparse_value;
    bool baseline;
    bool held;
    bool strict_args;
    QemuEvent release;
} ReadControl;

static ReadControl control;
static PCIHostState *host;
static char opaque_token;
int (*qemu_main)(void);

uint64_t agfx_test_baseline_mmio_read(void *, hwaddr, unsigned);
qmu_session *__wrap_qmu_create_extended(const qmu_extended_config *,
                                       const qmu_extended_callbacks *);
uint64_t __wrap_qmu_mmio_read(qmu_session *, uint32_t, unsigned);
BlockAIOCB *__real_thread_pool_submit_aio(ThreadPoolFunc *, void *,
                                         BlockCompletionFunc *, void *);
BlockAIOCB *__wrap_thread_pool_submit_aio(ThreadPoolFunc *, void *,
                                         BlockCompletionFunc *, void *);
void __real_qemu_aio_unref(void *);
void __wrap_qemu_aio_unref(void *);

static uint64_t now_ns(void)
{
    struct timespec value;
    g_assert_cmpint(clock_gettime(CLOCK_MONOTONIC_RAW, &value), ==, 0);
    return (uint64_t)value.tv_sec * 1000000000 + value.tv_nsec;
}

qmu_session *__wrap_qmu_create_extended(const qmu_extended_config *cfg,
                                       const qmu_extended_callbacks *cb)
{
    g_assert_cmpuint(cfg->abi_magic, ==, QMU_EXTENDED_CONFIG_ABI_MAGIC);
    g_assert_cmpuint(cb->abi_magic, ==, QMU_EXTENDED_CALLBACKS_ABI_MAGIC);
    g_assert_cmpuint(cfg->abi_version, ==, QMU_EXTENDED_ABI_VERSION);
    g_assert_cmpuint(cb->abi_version, ==, QMU_EXTENDED_ABI_VERSION);
    g_assert_cmpuint(cfg->struct_size, ==, sizeof(*cfg));
    g_assert_cmpuint(cb->struct_size, ==, sizeof(*cb));
    g_assert_true(cb->user_ctx == control.state);
    /* No callback or context retained, no real factory/GPU initialization. */
    return NULL;
}

static void release_getter(void *opaque)
{
    ReadControl *c = opaque;
    g_assert_true(bql_locked());
    g_assert_true(qemu_get_current_aio_context() == qemu_get_aio_context());
    g_assert_cmpuint(c->held_release, ==, 0);
    ++c->held_release;
    qemu_event_set(&c->release);
}

static void foreign_context(void *opaque)
{
    ReadControl *c = opaque;
    ++c->foreign_release;
}

uint64_t __wrap_qmu_mmio_read(qmu_session *session, uint32_t offset,
                              unsigned size)
{
    g_assert_true(session == control.opaque_session);
    g_assert_true(control.state->qmu_dev == session);
    g_assert_cmpint(bql_locked(), ==, control.baseline);
    if (control.strict_args) {
        g_assert_cmpuint(offset, ==, control.expected_offset);
        g_assert_cmpuint(size, ==, control.expected_size);
    }
    qatomic_inc(&control.getter_calls);
    if (control.held) {
        g_assert_false(control.baseline);
        /* Unit transport rendezvous, not a claim that the current pure
         * QMetal getter waits for DMA or that GPU work is quiescent. */
        aio_bh_schedule_oneshot(qemu_get_aio_context(), release_getter, &control);
        qemu_event_wait(&control.release);
    }
    return offset == 0x2000 ? control.sparse_value :
        ((UINT32_C(0x53470000) ^ offset) + size);
}

BlockAIOCB *__wrap_thread_pool_submit_aio(ThreadPoolFunc *fn, void *arg,
                                         BlockCompletionFunc *cb, void *opaque)
{
    BlockAIOCB *request;
    g_assert_true(bql_locked());
    g_assert_true(qemu_get_current_aio_context() == qemu_get_aio_context());
    g_assert_nonnull(fn);
    g_assert_nonnull(arg);
    g_assert_null(cb);
    g_assert_null(opaque);
    ++control.accepted;
    ++control.pending;
    request = __real_thread_pool_submit_aio(fn, arg, cb, opaque);
    g_assert_nonnull(request);
    if (!control.request_type) {
        control.request_type = request->aiocb_info;
    }
    g_assert_true(control.request_type == request->aiocb_info);
    return request;
}

void __wrap_qemu_aio_unref(void *opaque)
{
    BlockAIOCB *request = opaque;
    bool observed = request->aiocb_info == control.request_type;

    if (observed) {
        g_assert_true(bql_locked());
        g_assert_cmpuint(control.pending, >, 0);
        g_assert_cmpint(request->refcnt, ==, 1);
    }
    __real_qemu_aio_unref(opaque);
    if (observed) {
        /* Actual request metadata reaped, distinct from getter completion
         * and emphatically not a physical GPU completion marker. */
        --control.pending;
        ++control.reaped;
    }
}

static void drain_metadata(void)
{
    AIO_WAIT_WHILE(NULL, control.pending != 0);
    g_assert_cmpuint(control.accepted, ==, control.reaped);
}

static uint64_t read32(uint32_t offset)
{
    uint64_t value;
    g_assert_true(bql_locked());
    g_assert_cmpint(memory_region_dispatch_read(&control.state->mmio, offset,
                   &value, MO_32, MEMTXATTRS_UNSPECIFIED), ==, MEMTX_OK);
    return value;
}

static void test_values_and_worker(void)
{
    const uint32_t offsets[] = { 0x1000, 0x100c, 0x1014, 0x1018, 0x122c };
    const unsigned before = qatomic_read(&control.getter_calls);
    const unsigned accepted = control.accepted;

    control.strict_args = true;
    control.expected_size = 4;
    for (unsigned i = 0; i < ARRAY_SIZE(offsets); ++i) {
        control.expected_offset = offsets[i];
        g_assert_cmphex(read32(offsets[i]), ==,
                       (UINT32_C(0x53470000) ^ offsets[i]) + 4);
    }
    control.strict_args = false;
    g_assert_cmpuint(qatomic_read(&control.getter_calls) - before, ==,
                     ARRAY_SIZE(offsets));
    g_assert_cmpuint(control.accepted - accepted, ==, ARRAY_SIZE(offsets));
    drain_metadata();
}

static void test_main_context_progress(void)
{
    control.held = true;
    control.held_release = 0;
    control.foreign_release = 0;
    qemu_event_init(&control.release, false);
    aio_bh_schedule_oneshot(iohandler_get_aio_context(), foreign_context, &control);
    g_assert_cmphex(read32(0x100c), ==, (UINT32_C(0x53470000) ^ 0x100c) + 4);
    g_assert_cmpuint(control.held_release, ==, 1);
    g_assert_cmpuint(control.foreign_release, ==, 0);
    control.held = false;
    qemu_event_destroy(&control.release);
    drain_metadata();
    /* The read pump intentionally serviced main AIO, not IOHANDLER/QMP. */
    aio_poll(iohandler_get_aio_context(), false);
    g_assert_cmpuint(control.foreign_release, ==, 1);
}

static void test_dispatch_size_and_sparse_value(void)
{
    const uint32_t granularities[] = { 0, 4096, 65536 };
    uint64_t value;
    unsigned before = qatomic_read(&control.getter_calls);
    unsigned accepted = control.accepted;

    g_assert_cmpint(memory_region_dispatch_read(&control.state->mmio, 0x1000,
                   &value, MO_64, MEMTXATTRS_UNSPECIFIED), ==, MEMTX_OK);
    g_assert_cmphex(value, ==,
       ((uint64_t)((UINT32_C(0x53470000) ^ 0x1004) + 4) << 32) |
                  ((UINT32_C(0x53470000) ^ 0x1000) + 4));
    g_assert_cmpuint(qatomic_read(&control.getter_calls) - before, ==, 2);
    g_assert_cmpuint(control.accepted - accepted, ==, 2);
    control.strict_args = true;
    control.expected_offset = 0x2000;
    control.expected_size = 4;
    for (unsigned i = 0; i < ARRAY_SIZE(granularities); ++i) {
        control.sparse_value = granularities[i];
        g_assert_cmpuint(read32(0x2000), ==, granularities[i]);
    }
    control.strict_args = false;
    drain_metadata();
    g_test_message("0x2000 values are opaque unit forwarding cases, not physical adapter capability evidence");
}

static void test_preserved_direct_baseline(void)
{
    const MemoryRegionOps *saved = control.state->mmio.ops;
    MemoryRegionOps old_ops = *saved;
    unsigned accepted = control.accepted;

    old_ops.read = agfx_test_baseline_mmio_read;
    control.state->mmio.ops = &old_ops;
    control.baseline = true;
    g_assert_cmphex(read32(0x1018), ==, (UINT32_C(0x53470000) ^ 0x1018) + 4);
    g_assert_cmpuint(control.accepted, ==, accepted);
    control.baseline = false;
    control.state->mmio.ops = saved;
    g_test_message("exact committed old read calls getter on BQL; candidate uses a real concurrent ThreadPool worker");
}

static uint64_t measure(bool baseline, unsigned iterations)
{
    MemoryRegionOps old_ops = *control.candidate_ops;
    uint64_t start;
    uint64_t elapsed;

    old_ops.read = agfx_test_baseline_mmio_read;
    control.state->mmio.ops = baseline ? &old_ops : control.candidate_ops;
    control.baseline = baseline;
    start = now_ns();
    for (unsigned i = 0; i < iterations; ++i) {
        g_assert_cmphex(read32(0x100c), ==, (UINT32_C(0x53470000) ^ 0x100c) + 4);
    }
    elapsed = now_ns() - start;
    control.baseline = false;
    control.state->mmio.ops = control.candidate_ops;
    /* Completion metadata may trail the last getter result; drain it before
     * either replacing the unit owner or starting another measurement pair. */
    drain_metadata();
    return elapsed;
}

static void test_cold_warm_cost(void)
{
    const unsigned pairs = 15;
    const unsigned iterations = 2000;
    uint64_t old_cold, new_cold;

    g_assert_null(qemu_get_aio_context()->thread_pool);
    old_cold = measure(true, 1);
    new_cold = measure(false, 1);
    g_test_message("COLD baseline_ns=%" PRIu64 " dispatch_ns=%" PRIu64,
                   old_cold, new_cold);
    /* Warm the same real pool, no new device/factory or old suite rerun. */
    (void)measure(false, 64);
    for (unsigned i = 0; i < pairs; ++i) {
        uint64_t old_time, new_time;
        if (i & 1) {
            new_time = measure(false, iterations);
            old_time = measure(true, iterations);
        } else {
            old_time = measure(true, iterations);
            new_time = measure(false, iterations);
        }
        g_test_message("PAIR index=%u iterations=%u baseline_ns=%" PRIu64
                       " dispatch_ns=%" PRIu64,
                       i, iterations, old_time, new_time);
    }
    g_test_message("UNIT CPU dispatch cost includes typed getter and lightweight submit/reap observation; no Device/GPU throughput or nonregression proof");
}

static const TypeInfo test_host_type = {
    .name = TYPE_TEST_HOST,
    .parent = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(PCIHostState),
};

static void drain_rcu(void)
{
    bql_unlock();
    drain_call_rcu();
    bql_lock();
}

int main(int argc, char **argv)
{
    char options[][16] = {"-machine", "none", "-accel", "tcg", "-nodefaults",
                         "-display", "none", "-S"};
    char *qemu_argv[] = {argv[0], options[0], options[1], options[2], options[3],
                        options[4], options[5], options[6], options[7], NULL};
    bool cost = false;
    bool saved_msi;
    Error *error = NULL;
    int status;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--cost-mode")) {
            cost = true;
            memmove(argv + i, argv + i + 1, (argc - i) * sizeof(*argv));
            --argc;
            --i;
        }
    }
    g_test_init(&argc, &argv, NULL);
    qemu_init(ARRAY_SIZE(qemu_argv) - 1, qemu_argv);
    type_register_static(&test_host_type);
    host = PCI_HOST_BRIDGE(object_new(TYPE_TEST_HOST));
    host->bus = pci_root_bus_new(DEVICE(host), "read.pcie", get_system_memory(),
                                 get_system_io(), 0, TYPE_PCIE_BUS);
    g_assert_true(qbus_realize(BUS(host->bus), &error_abort));
    control.state = APPLE_GFX_ML(object_new(TYPE_APPLE_GFX_ML));
    DEVICE(control.state)->hotplugged = false;
    control.state->vram_size_mb = 1;
    saved_msi = msi_nonbroken;
    msi_nonbroken = true;
    g_assert_false(qdev_realize(DEVICE(control.state), BUS(host->bus), &error));
    g_assert_nonnull(error);
    g_assert_nonnull(strstr(error_get_pretty(error), "Failed to create qmetal device"));
    error_free(error);
    g_assert_false(DEVICE(control.state)->realized);
    g_assert_null(control.state->qmu_dev);
    g_assert_null(control.state->render_pool);
    g_assert_null(control.state->con);
    g_assert_false(control.state->log_writer_started);
    control.candidate_ops = control.state->mmio.ops;
    g_assert_nonnull(control.candidate_ops->read);
    control.opaque_session = (qmu_session *)&opaque_token;
    control.state->qmu_dev = control.opaque_session;

    if (cost) {
        g_test_add_func("/agfx/mmio-read/cold-warm-cost", test_cold_warm_cost);
    } else {
        g_test_add_func("/agfx/mmio-read/value-worker", test_values_and_worker);
        g_test_add_func("/agfx/mmio-read/main-AIO", test_main_context_progress);
        g_test_add_func("/agfx/mmio-read/size-sparse", test_dispatch_size_and_sparse_value);
        g_test_add_func("/agfx/mmio-read/direct-baseline", test_preserved_direct_baseline);
    }
    status = g_test_run();
    drain_metadata();
    control.state->qmu_dev = NULL;
    object_unparent(OBJECT(control.state));
    object_unref(OBJECT(control.state));
    /* Explicit AS owner retirement followed by queued RAM reclaim, as already
     * source-proven. This setup cleanup is not a repeated old unwind suite. */
    drain_rcu();
    drain_rcu();
    control.state = NULL;
    msi_nonbroken = saved_msi;
    pci_root_bus_cleanup(host->bus);
    object_unref(OBJECT(host));
    drain_rcu();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
