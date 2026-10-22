/* One missing real producer->actual compiled adapter/provider invocation.
 * Reuses the actual QEMU RAM/QOM fixture mechanics from918; it does not
 * repeat its nine standalone groups, realize a graphics device, initialize
 * a QMetal session/GPU, boot a guest or claim the containing FIFO lifetime. */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/memfd.h"
#include "qemu/rcu.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/replay.h"
#include "system/system.h"
#include "hw/display/apple-gfx-ml.h"
#include "hw/display/apple-gfx-eeee-process-vm.h"
#include "apple-gfx-eeee-qmetal-test-bridge.h"
#include <sys/mman.h>

#define TYPE_JOIN_RAM_OWNER "test-eeee-qmetal-join-ram-owner"
#define JOIN_RAM_GPA UINT64_C(0x10000000)
#define JOIN_ALIAS_GPA UINT64_C(0x20000000)

typedef struct JoinRamOwner {
    Object parent_obj;
    MemoryRegion ram;
    MemoryRegion alias;
    unsigned *finalized;
} JoinRamOwner;

static void join_owner_finalize(Object *obj)
{
    JoinRamOwner *owner = (JoinRamOwner *)obj;

    if (owner->finalized) {
        g_assert_true(bql_locked());
        g_assert_cmpuint((*owner->finalized)++, ==, 0);
    }
}

static const TypeInfo join_owner_type = {
    .name = TYPE_JOIN_RAM_OWNER,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(JoinRamOwner),
    .instance_finalize = join_owner_finalize,
};

int (*qemu_main)(void);

/* Test-only link wrapper. Armed solely during synchronous source capture;
 * all original real-RAM/adapter/release cases pass through unchanged. */
static unsigned source_fail_fixed_ordinal, source_fixed_calls;

/* Normal Linux QEMU builds use _FILE_OFFSET_BITS=64: libc redirects mmap
 * calls to mmap64. Wrap the actual imported symbol, not the C spelling. */
void *__real_mmap64(void *, size_t, int, int, int, off64_t);
void *__wrap_mmap64(void *, size_t, int, int, int, off64_t);
void *__wrap_mmap64(void *address, size_t length, int protection,
                     int flags, int fd, off64_t offset)
{
    if (source_fail_fixed_ordinal && (flags & MAP_FIXED) &&
        ++source_fixed_calls == source_fail_fixed_ordinal) {
        errno = ENOMEM;
        return MAP_FAILED;
    }
    return __real_mmap64(address, length, protection, flags, fd, offset);
}

typedef enum FinalReleaseKind {
    FINAL_PROVIDER_REGION,
    FINAL_ALIAS_DEALLOCATE,
    FINAL_ALIAS_ZERO_OVERWRITE,
    FINAL_ALIAS_REMAP_OVERWRITE,
} FinalReleaseKind;

typedef struct FinalRelease {
    AppleGfxMLState *ctx;
    qmu_extended_callbacks *cb;
    MemoryRegion *old_region, *new_region;
    uint64_t address, page, new_source;
    unsigned finalized;
    FinalReleaseKind kind;
    int done;
} FinalRelease;

static void *final_release_worker(void *opaque)
{
    FinalRelease *r = opaque;
    qmu_extended_callbacks *cb = r->cb;

    rcu_register_thread();
    g_assert_false(bql_locked());
    switch (r->kind) {
    case FINAL_PROVIDER_REGION:
        cb->eeee_task_region_unref(r->ctx, r->old_region);
        break;
    case FINAL_ALIAS_DEALLOCATE:
        g_assert_cmpint(cb->eeee_task_deallocate(r->ctx, r->address, r->page), ==, 0);
        break;
    case FINAL_ALIAS_ZERO_OVERWRITE:
        g_assert_cmpint(cb->eeee_task_allocate(r->ctx, &r->address, r->page,
            QMU_EEEE_TASK_ALLOCATE_FIXED_OVERWRITE), ==, 1);
        break;
    case FINAL_ALIAS_REMAP_OVERWRITE: {
        int current = 0, maximum = 0;

        cb->eeee_task_rcu_enter(r->ctx);
        cb->eeee_task_lock(r->ctx);
        cb->eeee_task_region_ref(r->ctx, r->new_region);
        g_assert_cmpint(cb->eeee_task_remap_shared_fixed_overwrite(r->ctx,
            &r->address, r->page, r->page - 1, r->new_source,
            &current, &maximum), ==, 1);
        /* Retired original backing is consumed by unlock only AFTER mutex
         * release. A BQL wait inside the provider lock would be a deadlock. */
        cb->eeee_task_unlock(r->ctx);
        cb->eeee_task_rcu_leave(r->ctx);
        break;
    }
    }
    /* The original final-unpin leaf returned, not merely a BH was queued. */
    g_assert_cmpuint(r->finalized, ==, 1);
    g_assert_false(bql_locked());
    qatomic_set(&r->done, 1);
    rcu_unregister_thread();
    return NULL;
}

static JoinRamOwner *new_final_release_ram(uint64_t page)
{
    JoinRamOwner *owner = (JoinRamOwner *)object_new(TYPE_JOIN_RAM_OWNER);
    int fd = qemu_memfd_create("eeee-physical-release532", page,
                               false, 0, 0, &error_abort);

    g_assert_cmpint(fd, >=, 0);
    g_assert_true(memory_region_init_ram_from_fd(&owner->ram, OBJECT(owner),
        "eeee-physical-release532", page, RAM_SHARED, fd, 0, &error_abort));
    return owner;
}

static void test_worker_final_physical_release(gconstpointer data)
{
    AppleGfxMLState ctx = {};
    qmu_extended_callbacks cb = { .user_ctx = &ctx };
    QemuThread worker;
    FinalRelease r = { .ctx = &ctx, .cb = &cb, .kind = GPOINTER_TO_INT(data) };
    JoinRamOwner *replacement = NULL;
    int current = 0, maximum = 0;

    ctx.eeee_provider_state = agfx_eeee_provider_state_create();
    agfx_eeee_provider_fill_callbacks(&cb);
    r.page = cb.eeee_task_host_page_size(&ctx);
    JoinRamOwner *owner = new_final_release_ram(r.page);
    owner->finalized = &r.finalized;
    r.old_region = &owner->ram;
    memory_region_add_subregion(get_system_memory(), JOIN_RAM_GPA, &owner->ram);
    cb.eeee_task_lock(&ctx);
    cb.eeee_task_region_ref(&ctx, &owner->ram);
    cb.eeee_task_unlock(&ctx);
    if (r.kind != FINAL_PROVIDER_REGION) {
        g_assert_cmpint(cb.eeee_task_allocate(&ctx, &r.address, r.page,
            QMU_EEEE_TASK_ALLOCATE_ANYWHERE), ==, 1);
        cb.eeee_task_rcu_enter(&ctx);
        cb.eeee_task_lock(&ctx);
        g_assert_cmpint(cb.eeee_task_remap_shared_fixed_overwrite(&ctx,
            &r.address, r.page, r.page - 1,
            (uint64_t)(uintptr_t)memory_region_get_ram_ptr(&owner->ram),
            &current, &maximum), ==, 1);
        cb.eeee_task_unlock(&ctx);
        cb.eeee_task_rcu_leave(&ctx);
        cb.eeee_task_region_unref(&ctx, &owner->ram);
    }
    if (r.kind == FINAL_ALIAS_REMAP_OVERWRITE) {
        replacement = new_final_release_ram(r.page);
        r.new_region = &replacement->ram;
        r.new_source = (uint64_t)(uintptr_t)memory_region_get_ram_ptr(&replacement->ram);
    }
    memory_region_del_subregion(get_system_memory(), &owner->ram);
    object_unref(OBJECT(owner));
    bql_unlock();
    drain_call_rcu();
    bql_lock();
    g_assert_cmpuint(r.finalized, ==, 0);
    g_assert_cmpuint(OBJECT(owner)->ref, ==, 1); /* actual last native pin */

    qemu_thread_create(&worker, "eeee-final-pin", final_release_worker, &r,
                       QEMU_THREAD_JOINABLE);
    /* Reuse accepted imported-dirty control's bounded main-loop/BQL polling.
     * No device/session/GPU, no fixed wait after the required RETURN exists. */
    const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!qatomic_read(&r.done)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&worker);
    g_assert_cmpuint(r.finalized, ==, 1);
    if (r.kind == FINAL_ALIAS_ZERO_OVERWRITE || r.kind == FINAL_ALIAS_REMAP_OVERWRITE) {
        if (replacement) cb.eeee_task_region_unref(&ctx, &replacement->ram);
        g_assert_cmpint(cb.eeee_task_deallocate(&ctx, r.address, r.page), ==, 0);
    }
    if (replacement) object_unref(OBJECT(replacement));
    g_assert_true(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));
    ctx.eeee_provider_state = NULL;
    bql_unlock();
    drain_call_rcu();
    bql_lock();
}

static void test_real_producer_adapter_native_window(void)
{
    AppleGfxMLState ctx = {};
    qmu_extended_callbacks cb = {
        .abi_magic = QMU_EXTENDED_CALLBACKS_ABI_MAGIC,
        .abi_version = QMU_EXTENDED_ABI_VERSION,
        .struct_size = sizeof(cb),
        .user_ctx = &ctx,
    };
    JoinRamOwner *owner = (JoinRamOwner *)object_new(TYPE_JOIN_RAM_OWNER);
    uint64_t page;
    uint8_t *canonical;
    int fd;

    ctx.eeee_provider_state = agfx_eeee_provider_state_create();
    agfx_eeee_provider_fill_callbacks(&cb);
    page = cb.eeee_task_host_page_size(&ctx);
    g_assert_cmpuint(page, >, 0);
    g_assert_cmpuint(page, <=, UINT64_C(0x1000000) / 3);
    g_test_message("actual native host page bytes=%" PRIu64, page);
    fd = qemu_memfd_create("eeee-qmetal-join-real-ram", 5 * page,
                           false, 0, 0, &error_abort);
    g_assert_cmpint(fd, >=, 0);
    /* Nonzero file offset PLUS canonical alias offset must both be used. */
    g_assert_true(memory_region_init_ram_from_fd(&owner->ram, OBJECT(owner),
        "eeee-qmetal-join-ram", 4 * page, RAM_SHARED, fd, page, &error_abort));
    canonical = memory_region_get_ram_ptr(&owner->ram);
    memset(canonical, 0x47, 4 * page);
    memory_region_add_subregion(get_system_memory(), JOIN_RAM_GPA, &owner->ram);
    memory_region_init_alias(&owner->alias, OBJECT(owner), "eeee-qmetal-join-alias",
                             &owner->ram, page, page);
    memory_region_add_subregion(get_system_memory(), JOIN_ALIAS_GPA, &owner->alias);
    bql_unlock();
    drain_call_rcu();
    bql_lock();

    g_assert_cmpint(agfx_test_qmetal_legacy_native_window(
        &cb, JOIN_ALIAS_GPA, page, 2 * page, 0x47, 0x69), ==, 0);
    for (uint64_t index = 0; index < 4 * page; ++index) {
        const uint8_t expected = index >= page && index < 2 * page ? 0x69 : 0x47;
        g_assert_cmpuint(canonical[index], ==, expected);
    }
    /* Synchronous fixture quiescence: C++ provider/facilities scopes left.
     * Empty registries alone are not a production callback/FIFO drain. */
    g_assert_true(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));
    ctx.eeee_provider_state = NULL;
    memory_region_del_subregion(get_system_memory(), &owner->alias);
    memory_region_del_subregion(get_system_memory(), &owner->ram);
    object_unref(OBJECT(owner));
    bql_unlock();
    drain_call_rcu();
    bql_lock();
}

static JoinRamOwner *new_source_ram(uint64_t length, uint64_t file_offset)
{
    JoinRamOwner *owner = (JoinRamOwner *)object_new(TYPE_JOIN_RAM_OWNER);
    int fd = qemu_memfd_create("eeee-persistent-source532", length + file_offset,
                               false, 0, 0, &error_abort);

    g_assert_cmpint(fd, >=, 0);
    g_assert_true(memory_region_init_ram_from_fd(&owner->ram, OBJECT(owner),
        "eeee-persistent-source532", length, RAM_SHARED, fd, file_offset,
        &error_abort));
    return owner;
}

static void source_remap(AppleGfxMLState *ctx, qmu_extended_callbacks *cb,
                         uint64_t address, uint64_t length, void *canonical)
{
    int current = 0, maximum = 0;

    cb->eeee_task_rcu_enter(ctx);
    cb->eeee_task_lock(ctx);
    g_assert_cmpint(cb->eeee_task_remap_shared_fixed_overwrite(ctx,
        &address, length, cb->eeee_task_host_page_size(ctx) - 1,
        (uint64_t)(uintptr_t)canonical, &current, &maximum), ==, 1);
    cb->eeee_task_unlock(ctx);
    cb->eeee_task_rcu_leave(ctx);
}

typedef struct SourceRelease {
    AgfxEeeeSourceBacking *source;
    unsigned *finalized;
    int done;
} SourceRelease;

static void *source_release_worker(void *opaque)
{
    SourceRelease *r = opaque;

    rcu_register_thread();
    g_assert_false(bql_locked());
    agfx_eeee_source_backing_unref(r->source);
    g_assert_cmpuint(*r->finalized, ==, 1);
    g_assert_false(bql_locked());
    qatomic_set(&r->done, 1);
    rcu_unregister_thread();
    return NULL;
}

static void test_persistent_source_backing(void)
{
    AppleGfxMLState ctx = {};
    qmu_extended_callbacks cb = { .user_ctx = &ctx };
    uint64_t address, page, alignment, span, overwritten;
    AgfxEeeeSourceBacking *source;
    AgfxEeeeSourceInterval first, second;
    unsigned finalized = 0;
    QemuThread worker;

    ctx.eeee_provider_state = agfx_eeee_provider_state_create();
    agfx_eeee_provider_fill_callbacks(&cb);
    page = cb.eeee_task_host_page_size(&ctx);
    alignment = MAX(UINT64_C(65536), page);
    span = 2 * alignment;
    JoinRamOwner *owner = new_source_ram(span + 2 * page, page);
    JoinRamOwner *replacement = new_source_ram(span, 0);
    uint8_t *canonical = memory_region_get_ram_ptr(&owner->ram);
    uint8_t *new_bytes = memory_region_get_ram_ptr(&replacement->ram);

    owner->finalized = &finalized;
    memset(canonical, 0x47, span + 2 * page);
    memset(new_bytes, 0xc2, span);
    memory_region_add_subregion(get_system_memory(), JOIN_RAM_GPA, &owner->ram);
    cb.eeee_task_lock(&ctx);
    cb.eeee_task_region_ref(&ctx, &owner->ram);
    cb.eeee_task_region_ref(&ctx, &replacement->ram);
    cb.eeee_task_unlock(&ctx);
    g_assert_cmpint(cb.eeee_task_allocate(&ctx, &address, span,
        QMU_EEEE_TASK_ALLOCATE_ANYWHERE), ==, 1);
    source_remap(&ctx, &cb, address, alignment, canonical + page);
    /* The second half is zero reservation, NOT admitted RAM. It may not be
     * silently filled from current GPA, task VA or synthetic padding. */
    g_assert_null(agfx_eeee_source_backing_capture(ctx.eeee_provider_state,
                                                 address, span, alignment));
    g_assert_cmpuint(*(uint8_t *)(uintptr_t)address, ==, 0x47);
    source_remap(&ctx, &cb, address + alignment, alignment,
                  canonical + page + alignment);
    for (unsigned fail_at = 1; fail_at <= 2; ++fail_at) {
        source_fail_fixed_ordinal = fail_at;
        source_fixed_calls = 0;
        g_assert_null(agfx_eeee_source_backing_capture(ctx.eeee_provider_state,
            address + 17, span - 34, alignment));
        source_fail_fixed_ordinal = 0;
        g_assert_cmpuint(source_fixed_calls, ==, fail_at);
        g_assert_cmpuint(*(uint8_t *)(uintptr_t)address, ==, 0x47);
        g_assert_cmpuint(*(uint8_t *)(uintptr_t)(address + alignment), ==, 0x47);
    }
    source = agfx_eeee_source_backing_capture(ctx.eeee_provider_state,
                                             address + 17, span - 34, alignment);
    g_assert_nonnull(source);
    uint8_t *allocation = agfx_eeee_source_backing_allocation(source);
    uint8_t *visible = agfx_eeee_source_backing_pointer(source);
    g_assert_cmpuint((uintptr_t)allocation % alignment, ==, 0);
    g_assert_cmpuint(agfx_eeee_source_backing_allocation_length(source), ==, span);
    g_assert_cmpuint(agfx_eeee_source_backing_visible_offset(source), ==, 17);
    g_assert_cmpuint(agfx_eeee_source_backing_length(source), ==, span - 34);
    g_assert_true(visible == allocation + 17);
    g_assert_cmpuint(agfx_eeee_source_backing_interval_count(source), ==, 2);
    g_assert_true(agfx_eeee_source_backing_interval(source, 0, &first));
    g_assert_true(agfx_eeee_source_backing_interval(source, 1, &second));
    g_assert_true(first.region == &owner->ram && second.region == &owner->ram);
    g_assert_cmpuint(first.region_offset, ==, page);
    g_assert_cmpuint(first.file_offset, ==, 2 * page);
    g_assert_cmpuint(first.allocation_offset, ==, 0);
    g_assert_cmpuint(first.length, ==, alignment);
    g_assert_cmpuint(second.region_offset, ==, page + alignment);
    g_assert_cmpuint(second.file_offset, ==, 2 * page + alignment);
    g_assert_cmpuint(second.allocation_offset, ==, alignment);
    g_assert_cmpuint(second.length, ==, alignment);
    struct stat original_stat, first_stat, second_stat;
    g_assert_cmpint(fstat(memory_region_get_fd(&owner->ram), &original_stat), ==, 0);
    g_assert_cmpint(fstat(first.fd, &first_stat), ==, 0);
    g_assert_cmpint(fstat(second.fd, &second_stat), ==, 0);
    g_assert_cmpuint(first_stat.st_dev, ==, original_stat.st_dev);
    g_assert_cmpuint(first_stat.st_ino, ==, original_stat.st_ino);
    g_assert_cmpuint(second_stat.st_dev, ==, original_stat.st_dev);
    g_assert_cmpuint(second_stat.st_ino, ==, original_stat.st_ino);
    g_assert_cmpint(first.fd, !=, second.fd); /* two actual admissions */
    int fd_flags = fcntl(first.fd, F_GETFD);
    g_assert_cmpint(fd_flags, >=, 0);
    g_assert_true(fd_flags & FD_CLOEXEC);
    for (uint64_t i = 0; i < span; ++i) g_assert_cmpuint(allocation[i], ==, 0x47);
    canonical[page + 7] = 0x6a;
    g_assert_cmpuint(allocation[7], ==, 0x6a);
    allocation[alignment + 9] = 0x9b;
    g_assert_cmpuint(canonical[page + alignment + 9], ==, 0x9b);

    /* Bounds and missing allocation padding fail before native mapping. */
    g_assert_null(agfx_eeee_source_backing_capture(NULL, address, span, alignment));
    g_assert_null(agfx_eeee_source_backing_capture(ctx.eeee_provider_state,
                                                 address, 0, alignment));
    g_assert_null(agfx_eeee_source_backing_capture(ctx.eeee_provider_state,
                                                 UINT64_MAX - 2, 8, alignment));
    g_assert_null(agfx_eeee_source_backing_capture(ctx.eeee_provider_state,
                                                 address, span, 3 * page));
    g_assert_null(agfx_eeee_source_backing_capture(ctx.eeee_provider_state,
                                                 address + span, page, page));
    g_assert_null(agfx_eeee_source_backing_capture(ctx.eeee_provider_state,
                                                 address + span - 1, 1, alignment));
    g_assert_false(agfx_eeee_source_backing_interval(source, 2, &first));
    g_assert_false(agfx_eeee_source_backing_interval(source, 0, NULL));
    g_assert_false(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));

    /* A native remap failure must not publish a different backing record. */
    int current = 0, maximum = 0;
    overwritten = address;
    source_fail_fixed_ordinal = 1;
    source_fixed_calls = 0;
    cb.eeee_task_rcu_enter(&ctx);
    cb.eeee_task_lock(&ctx);
    int failed_remap = cb.eeee_task_remap_shared_fixed_overwrite(&ctx,
        &overwritten, page, page - 1, (uint64_t)(uintptr_t)new_bytes,
        &current, &maximum);
    cb.eeee_task_unlock(&ctx);
    cb.eeee_task_rcu_leave(&ctx);
    source_fail_fixed_ordinal = 0;
    g_assert_cmpint(failed_remap, ==, 0);
    g_assert_cmpuint(source_fixed_calls, ==, 1);
    g_assert_cmpuint(*(uint8_t *)(uintptr_t)address, ==, 0x47);

    /* Exercise split aliases, same-VA different RAM and same-VA zero storage.
     * Captured exact backing and its still-live writes must not change. */
    source_remap(&ctx, &cb, address + page, page, new_bytes);
    g_assert_cmpuint(*(uint8_t *)(uintptr_t)(address + page), ==, 0xc2);
    g_assert_cmpuint(allocation[page], ==, 0x47);
    AgfxEeeeSourceBacking *mixed = agfx_eeee_source_backing_capture(
        ctx.eeee_provider_state, address, span, alignment);
    g_assert_nonnull(mixed);
    g_assert_cmpuint(agfx_eeee_source_backing_interval_count(mixed), ==, 4);
    AgfxEeeeSourceInterval prefix, changed, suffix;
    g_assert_true(agfx_eeee_source_backing_interval(mixed, 0, &prefix));
    g_assert_true(agfx_eeee_source_backing_interval(mixed, 1, &changed));
    g_assert_true(agfx_eeee_source_backing_interval(mixed, 2, &suffix));
    g_assert_true(prefix.region == &owner->ram);
    g_assert_true(changed.region == &replacement->ram);
    g_assert_true(suffix.region == &owner->ram);
    g_assert_cmpint(prefix.fd, ==, suffix.fd); /* actual shared split owner */
    g_assert_cmpuint(prefix.region_offset, ==, page);
    g_assert_cmpuint(changed.region_offset, ==, 0);
    g_assert_cmpuint(suffix.region_offset, ==, 3 * page);
    uint8_t *mixed_bytes = agfx_eeee_source_backing_pointer(mixed);
    g_assert_cmpuint(mixed_bytes[page], ==, 0xc2);
    g_assert_cmpuint(mixed_bytes[2 * page], ==, 0x47);
    mixed_bytes[page + 5] = 0xa4;
    g_assert_cmpuint(new_bytes[5], ==, 0xa4);
    g_assert_cmpuint(allocation[page + 5], ==, 0x47);
    agfx_eeee_source_backing_unref(mixed);
    overwritten = address;
    g_assert_cmpint(cb.eeee_task_allocate(&ctx, &overwritten, span,
        QMU_EEEE_TASK_ALLOCATE_FIXED_OVERWRITE), ==, 1);
    g_assert_cmpuint(*(uint8_t *)(uintptr_t)address, ==, 0);
    g_assert_cmpuint(allocation[7], ==, 0x6a);
    g_assert_cmpuint(allocation[alignment + 9], ==, 0x9b);
    cb.eeee_task_region_unref(&ctx, &owner->ram);
    cb.eeee_task_region_unref(&ctx, &replacement->ram);
    g_assert_cmpint(cb.eeee_task_deallocate(&ctx, address, span), ==, 0);
    g_assert_true(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));
    ctx.eeee_provider_state = NULL;
    canonical[page + 7] = 0x35;
    g_assert_cmpuint(allocation[7], ==, 0x35);

    /* This owner survives State destruction and address-space detachment. */
    memory_region_del_subregion(get_system_memory(), &owner->ram);
    object_unref(OBJECT(owner));
    object_unref(OBJECT(replacement));
    bql_unlock();
    drain_call_rcu();
    bql_lock();
    g_assert_cmpuint(finalized, ==, 0);
    g_assert_cmpuint(allocation[7], ==, 0x35);
    agfx_eeee_source_backing_ref(source);
    agfx_eeee_source_backing_unref(source);
    g_assert_cmpuint(finalized, ==, 0);
    SourceRelease release = { .source = source, .finalized = &finalized };
    qemu_thread_create(&worker, "eeee-source-last", source_release_worker,
                       &release, QEMU_THREAD_JOINABLE);
    const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!qatomic_read(&release.done)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&worker);
    g_assert_cmpuint(finalized, ==, 1);
    g_assert_cmpint(fcntl(first.fd, F_GETFD), ==, -1);
    g_assert_cmpint(errno, ==, EBADF);
    g_assert_cmpint(fcntl(second.fd, F_GETFD), ==, -1);
    g_assert_cmpint(errno, ==, EBADF);
    agfx_eeee_source_backing_unref(NULL);
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
    int status;

    g_test_init(&argc, &argv, NULL);
    qemu_init(ARRAY_SIZE(qemu_argv) - 1, qemu_argv);
    type_register_static(&join_owner_type);
    g_test_add_func("/eeee/join/actual-producer-adapter-native-window",
                    test_real_producer_adapter_native_window);
    g_test_add_data_func("/eeee/physical-final-release/provider-region",
        GINT_TO_POINTER(FINAL_PROVIDER_REGION), test_worker_final_physical_release);
    g_test_add_data_func("/eeee/physical-final-release/alias-deallocate",
        GINT_TO_POINTER(FINAL_ALIAS_DEALLOCATE), test_worker_final_physical_release);
    g_test_add_data_func("/eeee/physical-final-release/alias-zero-overwrite",
        GINT_TO_POINTER(FINAL_ALIAS_ZERO_OVERWRITE), test_worker_final_physical_release);
    g_test_add_data_func("/eeee/physical-final-release/alias-remap-overwrite",
        GINT_TO_POINTER(FINAL_ALIAS_REMAP_OVERWRITE), test_worker_final_physical_release);
    g_test_add_func("/eeee/persistent-source/exact-backing-independent-of-task",
                    test_persistent_source_backing);
    status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
