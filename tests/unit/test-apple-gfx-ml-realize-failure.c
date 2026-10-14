/*
 * Real cold-plug PCI/qdev/MemoryRegion/logger failure ownership controls.
 * Only RAM allocation and returned factory NULL are injected; all acquired
 * QEMU resources and their retirement execute the normal implementation.
 * No real QMetal factory, graphics session, guest or GPU quiescence claim.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/rcu.h"
#include "qemu/main-loop.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/ramblock.h"
#include "system/replay.h"
#include "system/system.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_host.h"
#include "hw/pci/msi.h"
#include "hw/display/apple-gfx-ml.h"
#include "qmu/qmetal_unified.h"

#define TYPE_TEST_HOST "test-agfx-unwind-host"
#define TYPE_TEST_DEVICE "test-agfx-unwind-device"

typedef enum FailureStage {
    FAIL_MSI,
    FAIL_RAM,
    FAIL_FACTORY,
} FailureStage;

typedef struct Control {
    AppleGfxMLState *state;
    FailureStage stage;
    unsigned logger_created;
    unsigned logger_joined;
    unsigned factory_calls;
    unsigned ram_calls;
    unsigned msi_retired;
    unsigned ram_unregistered;
    unsigned ram_retired;
    unsigned backing_retired;
    unsigned finalized;
    unsigned reverse[4];
    unsigned reverse_count;
    RAMBlock *ram;
    void *ram_host;
} Control;

static Control *active;
static PCIHostState *host;
int (*qemu_main)(void);

void __real_qemu_thread_create(QemuThread *, const char *, void *(*)(void *),
                               void *, int);
void __wrap_qemu_thread_create(QemuThread *, const char *, void *(*)(void *),
                               void *, int);
void *__real_qemu_thread_join(QemuThread *);
void *__wrap_qemu_thread_join(QemuThread *);
void __real_qemu_mutex_destroy(QemuMutex *);
void __wrap_qemu_mutex_destroy(QemuMutex *);
void __real_qemu_cond_destroy(QemuCond *);
void __wrap_qemu_cond_destroy(QemuCond *);
RAMBlock *__real_qemu_ram_alloc(ram_addr_t, uint32_t, MemoryRegion *, Error **);
RAMBlock *__wrap_qemu_ram_alloc(ram_addr_t, uint32_t, MemoryRegion *, Error **);
void __real_qemu_ram_free(RAMBlock *);
void __wrap_qemu_ram_free(RAMBlock *);
void __real_qemu_anon_ram_free(void *, size_t);
void __wrap_qemu_anon_ram_free(void *, size_t);
void __real_vmstate_unregister_ram(MemoryRegion *, DeviceState *);
void __wrap_vmstate_unregister_ram(MemoryRegion *, DeviceState *);
void __real_msi_uninit(PCIDevice *);
void __wrap_msi_uninit(PCIDevice *);
qmu_session *__wrap_qmu_create_extended(const qmu_extended_config *,
                                       const qmu_extended_callbacks *);

static void record_reverse(unsigned step)
{
    g_assert_cmpuint(active->reverse_count, <, ARRAY_SIZE(active->reverse));
    active->reverse[active->reverse_count++] = step;
}

void __wrap_qemu_thread_create(QemuThread *thread, const char *name,
                               void *(*start)(void *), void *arg, int mode)
{
    if (active && name && g_str_has_prefix(name, "agfx-")) {
        g_assert_cmpstr(name, ==, "agfx-log");
        g_assert_true(thread == &active->state->log_writer);
        g_assert_true(arg == active->state);
        g_assert_cmpint(mode, ==, QEMU_THREAD_JOINABLE);
        ++active->logger_created;
    }
    __real_qemu_thread_create(thread, name, start, arg, mode);
}

void *__wrap_qemu_thread_join(QemuThread *thread)
{
    bool logger = active && thread == &active->state->log_writer;

    if (logger) {
        g_assert_true(active->state->log_writer_stop);
        g_assert_false(active->state->msi_used);
        g_assert_null(active->state->qmu_dev);
        g_assert_null(active->state->con);
        g_assert_cmpuint(active->reverse_count, ==,
                         active->stage == FAIL_FACTORY ? 4 : 0);
    }
    void *result = __real_qemu_thread_join(thread);
    if (logger) {
        ++active->logger_joined;
        g_assert_null(result);
        g_assert_null(active->state->log_head);
        g_assert_null(active->state->log_tail);
        g_assert_cmpuint(active->state->log_depth, ==, 0);
    }
    return result;
}

void __wrap_qemu_cond_destroy(QemuCond *cond)
{
    if (active && cond == &active->state->bootstrap_present_cond) {
        record_reverse(1);
    }
    __real_qemu_cond_destroy(cond);
}

void __wrap_qemu_mutex_destroy(QemuMutex *mutex)
{
    if (active) {
        AppleGfxMLState *s = active->state;

        if (mutex == &s->bootstrap_present_mutex) {
            record_reverse(2);
        } else if (mutex == &s->session_mutex) {
            record_reverse(3);
        } else if (mutex == &s->mmio_job_mutex) {
            record_reverse(4);
        }
    }
    __real_qemu_mutex_destroy(mutex);
}

RAMBlock *__wrap_qemu_ram_alloc(ram_addr_t size, uint32_t flags,
                                MemoryRegion *mr, Error **errp)
{
    if (active && mr == &active->state->host_vram) {
        ++active->ram_calls;
        g_assert_cmpstr(memory_region_name(mr), ==, "apple-gfx-vram");
        if (active->stage == FAIL_RAM) {
            error_setg(errp, "test-only host_vram allocation failure");
            return NULL;
        }
    }
    return __real_qemu_ram_alloc(size, flags, mr, errp);
}

qmu_session *__wrap_qmu_create_extended(const qmu_extended_config *cfg,
                                       const qmu_extended_callbacks *cb)
{
    g_assert_nonnull(active);
    g_assert_cmpint(active->stage, ==, FAIL_FACTORY);
    ++active->factory_calls;
    g_assert_cmpuint(cfg->abi_magic, ==, QMU_EXTENDED_CONFIG_ABI_MAGIC);
    g_assert_cmpuint(cb->abi_magic, ==, QMU_EXTENDED_CALLBACKS_ABI_MAGIC);
    g_assert_cmpuint(cfg->abi_version, ==, QMU_EXTENDED_ABI_VERSION);
    g_assert_cmpuint(cb->abi_version, ==, QMU_EXTENDED_ABI_VERSION);
    g_assert_cmpuint(cfg->struct_size, ==, sizeof(*cfg));
    g_assert_cmpuint(cb->struct_size, ==, sizeof(*cb));
    g_assert_true(cb->user_ctx == active->state);
    g_assert_nonnull(cb->map_gpa);
    g_assert_nonnull(cb->read_memory);
    g_assert_nonnull(cb->schedule_display_completion);
    g_assert_null(cb->eeee_task_allocate);
    g_assert_null(cb->eeee_task_remap_shared_fixed_overwrite);
    g_assert_null(cb->eeee_task_protect);
    g_assert_null(cb->eeee_task_deallocate);
    g_assert_null(cb->eeee_task_host_page_size);
    g_assert_null(cb->eeee_task_rcu_enter);
    g_assert_null(cb->eeee_task_rcu_leave);
    g_assert_null(cb->eeee_task_lock);
    g_assert_null(cb->eeee_task_unlock);
    g_assert_null(cb->eeee_task_translate);
    g_assert_null(cb->eeee_task_direct);
    g_assert_null(cb->eeee_task_ram_pointer);
    g_assert_null(cb->eeee_task_region_ref);
    g_assert_null(cb->eeee_task_region_unref);
    g_assert_null(cb->eeee_task_publish);
    g_assert_null(cb->eeee_task_remove);
    g_assert_null(cb->eeee_task_host_write);
    g_assert_null(cb->eeee_task_fatal);
    g_assert_null(active->state->eeee_provider_state);
    g_assert_null(active->state->con);
    g_assert_null(active->state->display_fb);
    active->ram = active->state->host_vram.ram_block;
    g_assert_nonnull(active->ram);
    g_assert_true(qemu_ram_is_migratable(active->ram));
    g_assert_false(qemu_ram_is_shared(active->ram));
    g_assert_nonnull(active->ram->cpr_blocker);
    active->ram_host = active->ram->host;
    /* No callback is retained/invoked by this injected returned-NULL branch.
     * Actual legacy153 NULL reach remains separately pinned source evidence. */
    return NULL;
}

void __wrap_vmstate_unregister_ram(MemoryRegion *mr, DeviceState *dev)
{
    if (active && mr == &active->state->host_vram) {
        g_assert_true(dev == DEVICE(active->state));
        g_assert_true(mr->ram_block == active->ram);
        g_assert_cmpuint(active->reverse_count, ==, 4);
        for (unsigned i = 0; i < 4; ++i) {
            g_assert_cmpuint(active->reverse[i], ==, i + 1);
        }
        ++active->ram_unregistered;
    }
    __real_vmstate_unregister_ram(mr, dev);
    if (active && mr == &active->state->host_vram) {
        g_assert_false(qemu_ram_is_migratable(active->ram));
        g_assert_null(active->ram->cpr_blocker);
    }
}

void __wrap_msi_uninit(PCIDevice *dev)
{
    if (active && dev == PCI_DEVICE(active->state)) {
        g_assert_true(active->state->msi_used);
        g_assert_true(msi_present(dev));
        g_assert_nonnull(dev->config);
        if (active->stage == FAIL_FACTORY) {
            g_assert_cmpuint(active->ram_unregistered, ==, 1);
        }
        ++active->msi_retired;
    }
    __real_msi_uninit(dev);
}

void __wrap_qemu_ram_free(RAMBlock *ram)
{
    if (active && ram && ram == active->ram) {
        g_assert_cmpuint(active->logger_joined, ==, 1);
        g_assert_cmpuint(active->ram_unregistered, ==, 1);
        g_assert_null(ram->cpr_blocker);
        ++active->ram_retired;
    }
    __real_qemu_ram_free(ram);
}

void __wrap_qemu_anon_ram_free(void *data, size_t size)
{
    if (active && data && data == active->ram_host) {
        g_assert_cmpuint(active->ram_retired, ==, 1);
        ++active->backing_retired;
    }
    __real_qemu_anon_ram_free(data, size);
}

static void failed_device_finalize(Object *obj)
{
    g_assert_nonnull(active);
    g_assert_true(obj == OBJECT(active->state));
    g_assert_cmpuint(active->logger_joined, ==, 1);
    g_assert_false(active->state->log_writer_started);
    ++active->finalized;
}

static const TypeInfo test_host_type = {
    .name = TYPE_TEST_HOST, .parent = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(PCIHostState),
};
static const TypeInfo test_device_type = {
    .name = TYPE_TEST_DEVICE, .parent = TYPE_APPLE_GFX_ML,
    .instance_size = sizeof(AppleGfxMLState),
    .instance_finalize = failed_device_finalize,
};

static void drain_rcu(void)
{
    bql_unlock();
    drain_call_rcu();
    bql_lock();
}

static void run_failure(FailureStage stage)
{
    Control control = { .stage = stage };
    Error *error = NULL;
    int64_t start = g_get_monotonic_time();
    bool saved_msi = msi_nonbroken;
    AppleGfxMLState *s = APPLE_GFX_ML(object_new(TYPE_TEST_DEVICE));

    control.state = s;
    active = &control;
    /* Model ordinary cold-plug, not unsupported runtime hotplug. The fixture
     * adds this actual Device after machine-none bootstrap has completed. */
    DEVICE(s)->hotplugged = false;
    s->vram_size_mb = 1;
    msi_nonbroken = stage != FAIL_MSI;
    g_assert_false(qdev_realize(DEVICE(s), BUS(host->bus), &error));
    g_assert_nonnull(error);
    const char *expected_error[] = {
        "MSI is not supported by interrupt controller",
        "test-only host_vram allocation failure",
        "Failed to create qmetal device",
    };
    g_assert_nonnull(strstr(error_get_pretty(error), expected_error[stage]));
    g_assert_false(DEVICE(s)->realized);
    g_assert_cmpuint(control.logger_created, ==, 1);
    g_assert_cmpuint(control.logger_joined, ==, 1);
    g_assert_false(s->log_writer_started);
    g_assert_false(s->msi_used);
    g_assert_false(msi_present(PCI_DEVICE(s)));
    g_assert_null(s->qmu_dev);
    g_assert_null(s->render_pool);
    g_assert_null(s->con);
    g_assert_null(s->display_fb);
    g_assert_cmpuint(s->display_fb_size, ==, 0);
    g_assert_cmpuint(control.factory_calls, ==, stage == FAIL_FACTORY ? 1 : 0);
    g_assert_cmpuint(control.ram_calls, ==, stage == FAIL_MSI ? 0 : 1);
    g_assert_cmpuint(control.msi_retired, ==, stage == FAIL_MSI ? 0 : 1);
    g_assert_cmpuint(control.reverse_count, ==, stage == FAIL_FACTORY ? 4 : 0);
    g_assert_cmpuint(control.ram_unregistered, ==, stage == FAIL_FACTORY ? 1 : 0);
    error_free(error);
    /* Match the real failed device_add caller, not success pc.exit. */
    object_unparent(OBJECT(s));
    object_unref(OBJECT(s));
    drain_rcu();
    g_assert_cmpuint(control.finalized, ==, 1);
    g_assert_cmpuint(control.ram_retired, ==, stage == FAIL_FACTORY ? 1 : 0);
    if (stage == FAIL_FACTORY) {
        /* The AS callback finalized the owner and queued RAM reclaim under
         * BQL. That descendant can follow the first drain marker; after BQL
         * reacquisition its enqueue is complete, so one second marker suffices. */
        drain_rcu();
    }
    g_assert_cmpuint(control.backing_retired, ==, stage == FAIL_FACTORY ? 1 : 0);
    active = NULL;
    msi_nonbroken = saved_msi;
    g_test_message("failure-stage=%u constructor_attempt_to_owner_retirement_us=%" PRId64,
                   stage, g_get_monotonic_time() - start);
}

static void test_msi_error(void) { run_failure(FAIL_MSI); }
static void test_ram_error(void) { run_failure(FAIL_RAM); }
static void test_factory_null(void) { run_failure(FAIL_FACTORY); }
static void test_repeated_fresh_failures(void)
{
    for (unsigned i = 0; i < 12; ++i) {
        run_failure(i % 3);
    }
    g_test_message("12 fresh actual cold-plug failures retired before parent/RCU release; not retrying a partly initialized object or proving native/GPU quiescence");
}

int main(int argc, char **argv)
{
    char options[][16] = {"-machine", "none", "-accel", "tcg", "-nodefaults",
                         "-display", "none", "-S"};
    char *qemu_argv[] = {argv[0], options[0], options[1], options[2], options[3],
                        options[4], options[5], options[6], options[7], NULL};
    int status;

    g_test_init(&argc, &argv, NULL);
    qemu_init(ARRAY_SIZE(qemu_argv) - 1, qemu_argv);
    type_register_static(&test_host_type);
    type_register_static(&test_device_type);
    host = PCI_HOST_BRIDGE(object_new(TYPE_TEST_HOST));
    host->bus = pci_root_bus_new(DEVICE(host), "unwind.pcie", get_system_memory(),
                                 get_system_io(), 0, TYPE_PCIE_BUS);
    g_assert_true(qbus_realize(BUS(host->bus), &error_abort));
    g_test_add_func("/agfx/realize-failure/msi", test_msi_error);
    g_test_add_func("/agfx/realize-failure/ram", test_ram_error);
    g_test_add_func("/agfx/realize-failure/factory-null", test_factory_null);
    g_test_add_func("/agfx/realize-failure/repeated-fresh", test_repeated_fresh_failures);
    status = g_test_run();
    pci_root_bus_cleanup(host->bus);
    object_unref(OBJECT(host));
    drain_rcu();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
