/*
 * Native task facility component tests against actual QEMU MemoryRegion/QOM,
 * RAMBlock, RCU and dirty tracking.  This executable links the same system
 * objects as QEMU but realizes no apple-gfx device, calls no QMetal session
 * factory, boots no guest and performs no GPU or graphical acceptance.
 */
#include "qemu/osdep.h"
#include "block/aio-wait.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/memfd.h"
#include "qemu/rcu.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/ram_addr.h"
#include "system/replay.h"
#include "system/system.h"
#include "hw/display/apple-gfx-ml.h"
#include "hw/display/apple-gfx-eeee-process-vm.h"
#include "qmu/qmetal_unified.h"

#include <sys/mman.h>

#define TYPE_NATIVE_RAM_OWNER "test-eeee-native-ram-owner"
#define RAM_GPA UINT64_C(0x10000000)
#define ALIAS_A_GPA UINT64_C(0x20000000)
#define ALIAS_B_GPA UINT64_C(0x30000000)

typedef struct NativeRamOwner {
    Object parent_obj;
    MemoryRegion ram;
    MemoryRegion alias_a;
    MemoryRegion alias_b;
    qmu_extended_callbacks *reentry;
    bool *finalized;
} NativeRamOwner;

typedef struct Fixture {
    AppleGfxMLState ctx;
    qmu_extended_callbacks cb;
    NativeRamOwner *owner;
    uint64_t page;
} Fixture;

int (*qemu_main)(void);

static void native_owner_finalize(Object *object)
{
    NativeRamOwner *owner = (NativeRamOwner *)object;

    /* If alias retirement invokes this under the task mutex, the bounded
     * test times out.  This is a controlled QOM finalizer, not instrumentation
     * in the production provider or device. */
    if (owner->reentry) {
        owner->reentry->eeee_task_lock(owner->reentry->user_ctx);
        owner->reentry->eeee_task_unlock(owner->reentry->user_ctx);
    }
    if (owner->finalized) {
        *owner->finalized = true;
    }
}

static const TypeInfo native_owner_type = {
    .name = TYPE_NATIVE_RAM_OWNER,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(NativeRamOwner),
    .instance_finalize = native_owner_finalize,
};

static void callbacks_init(AppleGfxMLState *ctx, qmu_extended_callbacks *cb)
{
    *cb = (qmu_extended_callbacks){
        .abi_magic = QMU_EXTENDED_CALLBACKS_ABI_MAGIC,
        .abi_version = QMU_EXTENDED_ABI_VERSION,
        .struct_size = sizeof(*cb),
        .user_ctx = ctx,
    };
    ctx->eeee_provider_state = agfx_eeee_provider_state_create();
    agfx_eeee_provider_fill_callbacks(cb);
}

static uint64_t allocate(qmu_extended_callbacks *cb, uint64_t length)
{
    uint64_t address = UINT64_MAX; /* ANYWHERE must ignore this input. */

    g_assert_cmpint(cb->eeee_task_allocate(cb->user_ctx, &address, length,
                                         QMU_EEEE_TASK_ALLOCATE_ANYWHERE), ==, 1);
    return address;
}

static NativeRamOwner *ram_owner_new(uint64_t page)
{
    NativeRamOwner *owner = (NativeRamOwner *)object_new(TYPE_NATIVE_RAM_OWNER);
    int fd = qemu_memfd_create("eeee-real-ram", 5 * page, false, 0, 0,
                               &error_abort);

    g_assert_cmpint(fd, >=, 0);
    /* The nonzero file offset is material: remap must use fd_offset plus
     * region_offset, not the file start or the GPA. QEMU owns the fd. */
    g_assert_true(memory_region_init_ram_from_fd(&owner->ram, OBJECT(owner),
                                                "eeee-native-ram", 4 * page,
                                                RAM_SHARED, fd, page,
                                                &error_abort));
    memset(memory_region_get_ram_ptr(&owner->ram), 0x47, 4 * page);
    return owner;
}

static Fixture *fixture_new(void)
{
    Fixture *f = g_new0(Fixture, 1);

    callbacks_init(&f->ctx, &f->cb);
    f->page = f->cb.eeee_task_host_page_size(&f->ctx);
    f->owner = ram_owner_new(f->page);
    memory_region_add_subregion(get_system_memory(), RAM_GPA, &f->owner->ram);
    memory_region_init_alias(&f->owner->alias_a, OBJECT(f->owner), "eeee-alias-a",
                             &f->owner->ram, f->page, f->page);
    memory_region_init_alias(&f->owner->alias_b, OBJECT(f->owner), "eeee-alias-b",
                             &f->owner->ram, f->page, f->page);
    memory_region_add_subregion(get_system_memory(), ALIAS_A_GPA,
                                &f->owner->alias_a);
    memory_region_add_subregion(get_system_memory(), ALIAS_B_GPA,
                                &f->owner->alias_b);
    /* Finish old FlatView refs before measuring QOM ref deltas. */
    bql_unlock();
    drain_call_rcu();
    bql_lock();
    return f;
}

static void fixture_free(Fixture *f)
{
    g_assert_true(agfx_eeee_provider_state_destroy(f->ctx.eeee_provider_state));
    memory_region_del_subregion(get_system_memory(), &f->owner->alias_b);
    memory_region_del_subregion(get_system_memory(), &f->owner->alias_a);
    memory_region_del_subregion(get_system_memory(), &f->owner->ram);
    object_unref(OBJECT(f->owner));
    bql_unlock();
    drain_call_rcu();
    bql_lock();
    g_free(f);
}

static void remap(qmu_extended_callbacks *cb, uint64_t target, uint64_t source,
                   uint64_t length)
{
    int current = 0, maximum = 0;
    uint64_t original = target;
    uint64_t page = cb->eeee_task_host_page_size(cb->user_ctx);

    g_assert_cmpint(cb->eeee_task_remap_shared_fixed_overwrite(
        cb->user_ctx, &target, length, page - 1, source, &current, &maximum), ==, 1);
    g_assert_cmpuint(target, ==, original);
    g_assert_cmpint(current, ==, PROT_READ | PROT_WRITE);
    g_assert_cmpint(maximum, ==, PROT_READ | PROT_WRITE);
}

static void test_complete_state(void)
{
    AppleGfxMLState ctx = {0};
    qmu_extended_callbacks cb;

    g_assert_true(agfx_eeee_provider_state_destroy(NULL));
    callbacks_init(&ctx, &cb);
    g_assert_cmpuint(cb.abi_version, ==, 3);
    g_assert_cmpuint(cb.struct_size, ==, sizeof(cb));
    g_assert_true(cb.user_ctx == &ctx);
#define PRESENT(name) g_assert_nonnull(cb.eeee_task_##name)
    PRESENT(allocate);
    PRESENT(remap_shared_fixed_overwrite);
    PRESENT(protect);
    PRESENT(deallocate);
    PRESENT(host_page_size);
    PRESENT(rcu_enter);
    PRESENT(rcu_leave);
    PRESENT(lock);
    PRESENT(unlock);
    PRESENT(translate);
    PRESENT(direct);
    PRESENT(ram_pointer);
    PRESENT(region_ref);
    PRESENT(region_unref);
    PRESENT(publish);
    PRESENT(remove);
    PRESENT(host_write);
    PRESENT(fatal);
#undef PRESENT
    cb.eeee_task_lock(&ctx);
    cb.eeee_task_publish(&ctx, &cb);
    cb.eeee_task_unlock(&ctx);
    g_assert_false(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));
    cb.eeee_task_lock(&ctx);
    cb.eeee_task_remove(&ctx, &cb);
    cb.eeee_task_unlock(&ctx);
    g_assert_true(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));
}

static void test_literal_allocation_and_busy_drain(void)
{
    AppleGfxMLState ctx = {0};
    qmu_extended_callbacks cb;
    uint64_t page, length, address, target;
    uint8_t *bytes;

    callbacks_init(&ctx, &cb);
    page = cb.eeee_task_host_page_size(&ctx);
    length = page + 7;
    address = allocate(&cb, length);
    bytes = (uint8_t *)(uintptr_t)address;
    for (uint64_t i = 0; i < length; ++i) {
        g_assert_cmpuint(bytes[i], ==, 0);
    }
    memset(bytes, 0x52, length);
    g_assert_false(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));
    /* Refused teardown must not latch a state that prevents valid drainage. */
    target = address;
    g_assert_cmpint(cb.eeee_task_allocate(&ctx, &target, page,
                        QMU_EEEE_TASK_ALLOCATE_FIXED_OVERWRITE), ==, 1);
    g_assert_cmpuint(bytes[0], ==, 0);
    g_assert_cmpuint(bytes[page], ==, 0x52);
    g_assert_cmpint(cb.eeee_task_deallocate(&ctx, address, 2 * page), ==, -1);
    g_assert_cmpint(cb.eeee_task_deallocate(&ctx, address, length), ==, 0);
    g_assert_true(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));
}

static void test_other_allocation_is_not_owned(void)
{
    AppleGfxMLState a = {0}, b = {0};
    qmu_extended_callbacks ca, cb, *low;
    uint64_t aa, ab, target, page;

    callbacks_init(&a, &ca);
    callbacks_init(&b, &cb);
    page = ca.eeee_task_host_page_size(&a);
    aa = allocate(&ca, page);
    ab = allocate(&cb, page);
    g_assert_cmpuint(aa, !=, ab);
    /* Choose by actual mmap order, never assume descending host VA allocation.
     * A foreign higher range catches unsigned contains subtraction wrap. */
    low = aa < ab ? &ca : &cb;
    target = MAX(aa, ab);
    *(uint8_t *)(uintptr_t)target = 0x6a;
    g_assert_cmpint(low->eeee_task_allocate(low->user_ctx, &target, page,
                        QMU_EEEE_TASK_ALLOCATE_FIXED_OVERWRITE), ==, 0);
    g_assert_cmpuint(*(uint8_t *)(uintptr_t)target, ==, 0x6a);
    g_assert_cmpint(ca.eeee_task_deallocate(&a, aa, page), ==, 0);
    g_assert_cmpint(cb.eeee_task_deallocate(&b, ab, page), ==, 0);
    g_assert_true(agfx_eeee_provider_state_destroy(a.eeee_provider_state));
    g_assert_true(agfx_eeee_provider_state_destroy(b.eeee_provider_state));
}

static void test_real_qemu_canonical_alias_identity(void)
{
    Fixture *f = fixture_new();
    void *a = NULL, *b = NULL;
    uint64_t offset_a = 0, offset_b = 0, covered = 0;
    uint64_t address = allocate(&f->cb, 2 * f->page);

    f->cb.eeee_task_rcu_enter(&f->ctx);
    f->cb.eeee_task_lock(&f->ctx);
    g_assert_cmpint(f->cb.eeee_task_translate(&f->ctx, ALIAS_A_GPA,
                        2 * f->page, 1, &a, &offset_a, &covered), ==, 1);
    g_assert_cmpuint(covered, ==, f->page);
    g_assert_cmpint(f->cb.eeee_task_translate(&f->ctx, ALIAS_B_GPA,
                        f->page, 1, &b, &offset_b, &covered), ==, 1);
    /* Actual QEMU flattens both address-space aliases to the canonical RAM
     * region. A reverse HVA lookup must name the same actual region here. */
    g_assert_true(a == &f->owner->ram && b == a);
    g_assert_cmpuint(offset_a, ==, f->page);
    g_assert_cmpuint(offset_b, ==, offset_a);
    g_assert_cmpint(f->cb.eeee_task_direct(&f->ctx, a, 1), ==, 1);
    g_assert_true(f->cb.eeee_task_ram_pointer(&f->ctx, a) ==
                  memory_region_get_ram_ptr(&f->owner->ram));
    f->cb.eeee_task_region_ref(&f->ctx, a);
    uint64_t source = (uintptr_t)f->cb.eeee_task_ram_pointer(&f->ctx, a) + offset_a;
    remap(&f->cb, address, source, f->page);
    remap(&f->cb, address + f->page, source, f->page);
    f->cb.eeee_task_unlock(&f->ctx);
    f->cb.eeee_task_rcu_leave(&f->ctx);
    uint8_t *bytes = (uint8_t *)(uintptr_t)address;
    bytes[7] = 0x91;
    g_assert_cmpuint(bytes[f->page + 7], ==, 0x91);
    g_assert_cmpuint(((uint8_t *)memory_region_get_ram_ptr(a))[f->page + 7], ==, 0x91);
    f->cb.eeee_task_region_unref(&f->ctx, a);
    g_assert_cmpint(f->cb.eeee_task_deallocate(&f->ctx, address, 2 * f->page), ==, 0);
    fixture_free(f);
}

static void test_shared_split_zero_and_dirty(void)
{
    Fixture *f = fixture_new();
    Object *owner = OBJECT(f->owner);
    uint32_t baseline = owner->ref;
    uint64_t address = allocate(&f->cb, 3 * f->page);
    uint8_t *bytes = (uint8_t *)(uintptr_t)address;
    uint8_t *ram = memory_region_get_ram_ptr(&f->owner->ram);
    uint64_t middle = address + f->page;
    DirtyBitmapSnapshot *snap;

    f->cb.eeee_task_rcu_enter(&f->ctx);
    f->cb.eeee_task_lock(&f->ctx);
    f->cb.eeee_task_region_ref(&f->ctx, &f->owner->ram);
    remap(&f->cb, address, (uintptr_t)ram, 3 * f->page);
    f->cb.eeee_task_unlock(&f->ctx);
    f->cb.eeee_task_rcu_leave(&f->ctx);
    g_assert_cmpuint(owner->ref, ==, baseline + 2); /* unique set + alias */
    bytes[0] = 0x23;
    g_assert_cmpuint(ram[0], ==, 0x23);
    g_assert_cmpint(f->cb.eeee_task_allocate(&f->ctx, &middle, f->page,
                        QMU_EEEE_TASK_ALLOCATE_FIXED_OVERWRITE), ==, 1);
    g_assert_cmpuint(owner->ref, ==, baseline + 3); /* unique + prefix/suffix */
    g_assert_cmpuint(bytes[f->page], ==, 0);
    g_assert_cmpuint(ram[f->page], ==, 0x47);
    bytes[f->page] = 0x72;
    g_assert_cmpuint(ram[f->page], ==, 0x47);
    bytes[2 * f->page] = 0x38;
    g_assert_cmpuint(ram[2 * f->page], ==, 0x38);
    f->cb.eeee_task_region_unref(&f->ctx, &f->owner->ram);
    g_assert_cmpuint(owner->ref, ==, baseline + 2);
    g_assert_false(agfx_eeee_provider_state_destroy(f->ctx.eeee_provider_state));
    memory_region_set_log(&f->owner->ram, true, DIRTY_MEMORY_VGA);
    memory_region_reset_dirty(&f->owner->ram, 0, 4 * f->page, DIRTY_MEMORY_VGA);
    f->cb.eeee_task_host_write(&f->ctx, address, 3 * f->page);
    snap = memory_region_snapshot_and_clear_dirty(&f->owner->ram, 0,
                                                  4 * f->page, DIRTY_MEMORY_VGA);
    g_assert_true(memory_region_snapshot_get_dirty(&f->owner->ram, snap, 0,
                                                    f->page));
    g_assert_false(memory_region_snapshot_get_dirty(&f->owner->ram, snap, f->page,
                                                     f->page));
    g_assert_true(memory_region_snapshot_get_dirty(&f->owner->ram, snap,
                                                    2 * f->page, f->page));
    g_assert_false(memory_region_snapshot_get_dirty(&f->owner->ram, snap,
                                                     3 * f->page, f->page));
    g_free(snap);
    memory_region_set_log(&f->owner->ram, false, DIRTY_MEMORY_VGA);
    g_assert_cmpint(f->cb.eeee_task_deallocate(&f->ctx, address, 3 * f->page), ==, 0);
    fixture_free(f);
}

/* No address-space registration here: the only extra owner refs are the
 * actual provider unique set and native aliases. This isolates QOM finalization
 * ordering from asynchronous FlatView retirement. */
static void finalizer_case(unsigned terminal)
{
    AppleGfxMLState ctx = {0};
    qmu_extended_callbacks cb;
    NativeRamOwner *owner, *next = NULL;
    uint64_t page, address;
    bool finalized = false;

    callbacks_init(&ctx, &cb);
    page = cb.eeee_task_host_page_size(&ctx);
    address = allocate(&cb, page);
    owner = ram_owner_new(page);
    owner->reentry = &cb;
    owner->finalized = &finalized;
    cb.eeee_task_rcu_enter(&ctx);
    cb.eeee_task_lock(&ctx);
    cb.eeee_task_region_ref(&ctx, &owner->ram);
    remap(&cb, address, (uintptr_t)memory_region_get_ram_ptr(&owner->ram), page);
    cb.eeee_task_unlock(&ctx);
    cb.eeee_task_rcu_leave(&ctx);
    cb.eeee_task_region_unref(&ctx, &owner->ram);
    object_unref(OBJECT(owner));
    g_assert_false(finalized); /* Only the native alias now retains this owner. */
    if (terminal == 0) {
        g_assert_cmpint(cb.eeee_task_deallocate(&ctx, address, page), ==, 0);
    } else if (terminal == 1) {
        g_assert_cmpint(cb.eeee_task_allocate(&ctx, &address, page,
                            QMU_EEEE_TASK_ALLOCATE_FIXED_OVERWRITE), ==, 1);
    } else {
        next = ram_owner_new(page);
        cb.eeee_task_rcu_enter(&ctx);
        cb.eeee_task_lock(&ctx);
        cb.eeee_task_region_ref(&ctx, &next->ram);
        remap(&cb, address, (uintptr_t)memory_region_get_ram_ptr(&next->ram), page);
        g_assert_false(finalized); /* Deferred until task unlock, not under it. */
        cb.eeee_task_unlock(&ctx);
        cb.eeee_task_rcu_leave(&ctx);
        cb.eeee_task_region_unref(&ctx, &next->ram);
    }
    g_assert_true(finalized);
    if (terminal != 0) {
        g_assert_cmpint(cb.eeee_task_deallocate(&ctx, address, page), ==, 0);
    }
    if (next) {
        object_unref(OBJECT(next));
    }
    g_assert_true(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));
}

static void test_deallocate_finalizer(void) { finalizer_case(0); }
static void test_overwrite_finalizer(void) { finalizer_case(1); }
static void test_remap_finalizer(void) { finalizer_case(2); }

static void test_separate_protect_facility(void)
{
    AppleGfxMLState ctx = {0};
    qmu_extended_callbacks cb;
    uint64_t page, address;

    callbacks_init(&ctx, &cb);
    page = cb.eeee_task_host_page_size(&ctx);
    address = allocate(&cb, page);
    cb.eeee_task_lock(&ctx);
    g_assert_cmpint(cb.eeee_task_protect(&ctx, address, page,
                                        QMU_EEEE_TASK_PROTECT_NONE), ==, 1);
    g_assert_cmpint(cb.eeee_task_protect(&ctx, address, page,
                                        QMU_EEEE_TASK_PROTECT_READ_ONLY), ==, 1);
    g_assert_cmpuint(*(uint8_t *)(uintptr_t)address, ==, 0);
    g_assert_cmpint(cb.eeee_task_protect(&ctx, address, page,
                                        QMU_EEEE_TASK_PROTECT_READ_WRITE), ==, 1);
    g_assert_cmpint(cb.eeee_task_protect(&ctx, address, 0,
                                        QMU_EEEE_TASK_PROTECT_NONE), ==, 1);
    cb.eeee_task_unlock(&ctx);
    *(uint8_t *)(uintptr_t)address = 0x64;
    g_assert_cmpint(cb.eeee_task_deallocate(&ctx, address, page), ==, 0);
    g_assert_true(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));
}

static void native_callback_rcu_read(Fixture *f)
{
    void *region = NULL;
    uint64_t offset = 0, covered = 0;

    f->cb.eeee_task_rcu_enter(&f->ctx);
    f->cb.eeee_task_rcu_enter(&f->ctx);
    g_assert_true(rcu_thread_is_registered());
    g_assert_cmpint(f->cb.eeee_task_translate(&f->ctx, RAM_GPA + 17, 32, 0,
                                            &region, &offset, &covered), ==, 1);
    g_assert_true(region == &f->owner->ram);
    g_assert_cmpuint(offset, ==, 17);
    g_assert_cmpuint(covered, ==, 32);
    g_assert_cmpint(f->cb.eeee_task_direct(&f->ctx, region, 0), ==, 1);
    const uint8_t *actual = f->cb.eeee_task_ram_pointer(&f->ctx, region);
    for (unsigned i = 0; i < 32; ++i) {
        g_assert_cmpuint(actual[offset + i], ==, 0x47);
    }
    f->cb.eeee_task_rcu_leave(&f->ctx);
    g_assert_true(rcu_thread_is_registered()); /* inner leave keeps outer */
    f->cb.eeee_task_rcu_leave(&f->ctx);
}

typedef struct NativeCallbackRcuWork {
    Fixture *fixture;
    int done;
} NativeCallbackRcuWork;

static void *native_callback_rcu_worker(void *opaque)
{
    NativeCallbackRcuWork *work = opaque;

    /* Deliberately no external registration: this is the QMetal worker
     * callback domain which the old dormant provider did not own. */
    g_assert_false(rcu_thread_is_registered());
    for (unsigned round = 0; round < 2; ++round) {
        native_callback_rcu_read(work->fixture);
        g_assert_false(rcu_thread_is_registered());
    }
    qatomic_set(&work->done, 1);
    aio_wait_kick();
    return NULL;
}

static void test_native_callback_rcu_thread_domains(void)
{
    Fixture *f = fixture_new();
    NativeCallbackRcuWork work = { .fixture = f };
    QemuThread worker;

    /* Preserve an already-registered real QEMU reader across nested scopes. */
    g_assert_true(rcu_thread_is_registered());
    native_callback_rcu_read(f);
    g_assert_true(rcu_thread_is_registered());
    qemu_thread_create(&worker, "eeee-callback-rcu", native_callback_rcu_worker,
                       &work, QEMU_THREAD_JOINABLE);
    const gint64 deadline = g_get_monotonic_time() + 5 * G_USEC_PER_SEC;
    while (!qatomic_read(&work.done)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&worker);
    fixture_free(f);
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
    type_register_static(&native_owner_type);
    g_test_add_func("/eeee/native/complete-state", test_complete_state);
    g_test_add_func("/eeee/native/literal-length-busy-drain",
                    test_literal_allocation_and_busy_drain);
    g_test_add_func("/eeee/native/foreign-higher-allocation",
                    test_other_allocation_is_not_owned);
    g_test_add_func("/eeee/native/real-qemu-canonical-aliases",
                    test_real_qemu_canonical_alias_identity);
    g_test_add_func("/eeee/native/shared-split-zero-dirty",
                    test_shared_split_zero_and_dirty);
    g_test_add_func("/eeee/native/deallocate-finalizer", test_deallocate_finalizer);
    g_test_add_func("/eeee/native/overwrite-finalizer", test_overwrite_finalizer);
    g_test_add_func("/eeee/native/remap-finalizer", test_remap_finalizer);
    g_test_add_func("/eeee/mapped/separate-protect", test_separate_protect_facility);
    g_test_add_func("/eeee/native/callback-rcu-thread-domains",
                    test_native_callback_rcu_thread_domains);
    status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
