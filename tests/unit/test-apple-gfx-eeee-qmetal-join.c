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


/* Independent dormant-context gates. No device realize, session, GPU or
 * canonical task/MAP activation. All context/provider calls are real. */
static unsigned owned_context_release_calls;

static void owned_context_release_counted(void *context)
{
    g_assert_cmpuint(qatomic_fetch_inc(&owned_context_release_calls), ==, 0);
    agfx_eeee_native_context_release(context);
}

static void owned_gfx_finalized(Object *object, const char *name, void *opaque)
{
    unsigned *finalized = opaque;

    (void)object;
    (void)name;
    g_assert_true(bql_locked());
    g_assert_cmpuint((*finalized)++, ==, 0);
}

static AppleGfxMLState *new_owned_gfx(unsigned *finalized)
{
    AppleGfxMLState *state = APPLE_GFX_ML(object_new(TYPE_APPLE_GFX_ML));

    object_property_add(OBJECT(state), "owned-context-test-finalize", "bool",
                        NULL, NULL, owned_gfx_finalized, finalized);
    qatomic_set(&owned_context_release_calls, 0);
    return state;
}

static AgfxTestOwnedContext *new_owned_bridge(AgfxEeeeNativeContext *context,
                                              qmu_extended_callbacks *callbacks)
{
    g_assert_true(agfx_eeee_native_context_acquire(context, callbacks));
    AgfxTestOwnedContext *bridge = agfx_test_owned_context_adopt(
        callbacks, context, owned_context_release_counted);

    g_assert_nonnull(bridge);
    g_assert_cmpint(agfx_test_owned_context_drop_parent(bridge), ==, 0);
    /* The original shared owner is now gone. Only its actual descendant
     * owns the same compiled facilities and real acquired context token. */
    g_assert_cmpuint(qatomic_read(&owned_context_release_calls), ==, 0);
    return bridge;
}

static void assert_owned_acquisition_stopped(AgfxEeeeNativeContext *context)
{
    qmu_extended_callbacks sentinel = { .abi_magic = 0x12345678 };
    qmu_extended_callbacks before = sentinel;

    agfx_eeee_native_context_stop_acquiring(context);
    g_assert_false(agfx_eeee_native_context_acquire(context, &sentinel));
    g_assert_cmpmem(&sentinel, sizeof(sentinel), &before, sizeof(before));
}

static JoinRamOwner *new_owned_source_ram(uint64_t page)
{
    JoinRamOwner *owner = new_source_ram(4 * page, page);

    memset(memory_region_get_ram_ptr(&owner->ram), 0x47, 4 * page);
    memory_region_add_subregion(get_system_memory(), JOIN_RAM_GPA, &owner->ram);
    memory_region_init_alias(&owner->alias, OBJECT(owner), "owned-context-alias",
                             &owner->ram, page, page);
    memory_region_add_subregion(get_system_memory(), JOIN_ALIAS_GPA, &owner->alias);
    bql_unlock();
    drain_call_rcu();
    bql_lock();
    return owner;
}

static void detach_owned_source_ram(JoinRamOwner *owner)
{
    memory_region_del_subregion(get_system_memory(), &owner->alias);
    memory_region_del_subregion(get_system_memory(), &owner->ram);
    object_unref(OBJECT(owner));
    bql_unlock();
    drain_call_rcu();
    bql_lock();
}

static void assert_owned_native_window(AgfxTestOwnedContext *bridge,
                                       JoinRamOwner *owner, uint64_t page,
                                       bool keep_alias)
{
    void *actual_region = NULL;
    uint64_t actual_offset = UINT64_MAX;
    uint8_t *canonical = memory_region_get_ram_ptr(&owner->ram);

    g_assert_cmpint(agfx_test_owned_context_native_window(bridge,
        JOIN_ALIAS_GPA, page, 2 * page, 0x47, 0x69, keep_alias,
        &actual_region, &actual_offset), ==, 0);
    g_assert_true(actual_region == &owner->ram);
    g_assert_cmpuint(actual_offset, ==, page);
    for (uint64_t index = 0; index < 4 * page; ++index) {
        const uint8_t expected = index >= page && index < 2 * page ? 0x69 : 0x47;
        g_assert_cmpuint(canonical[index], ==, expected);
    }
}

static void test_owned_context_descendant_native_window(void)
{
    unsigned gfx_finalized = 0;
    AppleGfxMLState *state = new_owned_gfx(&gfx_finalized);
    AgfxEeeeNativeContext *context = agfx_eeee_native_context_create(state);
    qmu_extended_callbacks callbacks = {};

    g_assert_nonnull(context);
    AgfxTestOwnedContext *bridge = new_owned_bridge(context, &callbacks);
    const uint64_t page = callbacks.eeee_task_host_page_size(state);
    JoinRamOwner *owner = new_owned_source_ram(page);

    assert_owned_acquisition_stopped(context);
    assert_owned_native_window(bridge, owner, page, false);
    g_assert_cmpuint(qatomic_read(&owned_context_release_calls), ==, 0);
    agfx_test_owned_context_destroy(bridge);
    g_assert_cmpuint(qatomic_read(&owned_context_release_calls), ==, 1);
    detach_owned_source_ram(owner);
    object_unref(OBJECT(state));
    g_assert_cmpuint(gfx_finalized, ==, 0);
    g_assert_cmpuint(OBJECT(state)->ref, ==, 1); /* actual context QOM root */
    g_assert_true(agfx_eeee_native_context_retire(context));
    g_assert_cmpuint(gfx_finalized, ==, 1);
    g_assert_cmpuint(qatomic_read(&owned_context_release_calls), ==, 1);
}

typedef struct OwnedContextRelease {
    AgfxTestOwnedContext *bridge;
    QemuEvent go;
    unsigned *ram_finalized;
    int retire_entered, retire_returned, release_bh_seen, done;
} OwnedContextRelease;

static void owned_context_allow_final_return(void *opaque)
{
    OwnedContextRelease *release = opaque;

    g_assert_true(bql_locked());
    g_assert_cmpint(qatomic_read(&release->retire_entered), ==, 1);
    g_assert_cmpint(qatomic_read(&release->retire_returned), ==, 0);
    g_assert_cmpuint(qatomic_read(&owned_context_release_calls), ==, 0);
    qatomic_set(&release->release_bh_seen, 1);
    qemu_event_set(&release->go);
}

static void *owned_context_final_return_worker(void *opaque)
{
    OwnedContextRelease *release = opaque;

    rcu_register_thread();
    g_assert_false(bql_locked());
    qemu_event_wait(&release->go);
    /* Provider destruction drops the last actual RAM MR off BQL. The real
     * leaf waits for main-AIO/BQL RETURN before releasing its context token. */
    agfx_test_owned_context_destroy(release->bridge);
    g_assert_cmpuint(*release->ram_finalized, ==, 1);
    g_assert_cmpuint(qatomic_read(&owned_context_release_calls), ==, 1);
    qatomic_set(&release->done, 1);
    rcu_unregister_thread();
    return NULL;
}

static void test_owned_context_retire_waits_last_mr_return(void)
{
    unsigned gfx_finalized = 0, ram_finalized = 0;
    AppleGfxMLState *state = new_owned_gfx(&gfx_finalized);
    AgfxEeeeNativeContext *context = agfx_eeee_native_context_create(state);
    qmu_extended_callbacks callbacks = {};
    QemuThread worker;

    g_assert_nonnull(context);
    AgfxTestOwnedContext *bridge = new_owned_bridge(context, &callbacks);
    const uint64_t page = callbacks.eeee_task_host_page_size(state);
    JoinRamOwner *owner = new_owned_source_ram(page);
    owner->finalized = &ram_finalized;
    assert_owned_native_window(bridge, owner, page, true);
    detach_owned_source_ram(owner);
    g_assert_cmpuint(ram_finalized, ==, 0);
    object_unref(OBJECT(state));
    g_assert_cmpuint(gfx_finalized, ==, 0);
    g_assert_cmpuint(OBJECT(state)->ref, ==, 1);
    assert_owned_acquisition_stopped(context);

    OwnedContextRelease release = {
        .bridge = bridge, .ram_finalized = &ram_finalized,
    };
    qemu_event_init(&release.go, false);
    qemu_thread_create(&worker, "eeee-owned-last", owned_context_final_return_worker,
                       &release, QEMU_THREAD_JOINABLE);
    /* This BH cannot run until retire really enters its pending-owner AIO
     * wait. Only then can the worker begin its last MR/provider release. */
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            owned_context_allow_final_return, &release);
    qatomic_set(&release.retire_entered, 1);
    g_assert_true(agfx_eeee_native_context_retire(context));
    qatomic_set(&release.retire_returned, 1);
    qemu_thread_join(&worker);
    g_assert_cmpint(qatomic_read(&release.release_bh_seen), ==, 1);
    g_assert_cmpint(qatomic_read(&release.done), ==, 1);
    g_assert_cmpuint(ram_finalized, ==, 1);
    g_assert_cmpuint(gfx_finalized, ==, 1);
    g_assert_cmpuint(qatomic_read(&owned_context_release_calls), ==, 1);
    qemu_event_destroy(&release.go);
}

static void test_owned_context_busy_preserves_root_retry(void)
{
    unsigned gfx_finalized = 0;
    AppleGfxMLState *state = new_owned_gfx(&gfx_finalized);
    AgfxEeeeNativeContext *context = agfx_eeee_native_context_create(state);
    qmu_extended_callbacks callbacks = {};
    uint64_t address = 0;

    g_assert_nonnull(context);
    AgfxTestOwnedContext *bridge = new_owned_bridge(context, &callbacks);
    const uint64_t page = callbacks.eeee_task_host_page_size(state);
    void *provider_state = state->eeee_provider_state;
    g_assert_cmpint(callbacks.eeee_task_allocate(state, &address, page,
        QMU_EEEE_TASK_ALLOCATE_ANYWHERE), ==, 1);
    memset((void *)(uintptr_t)address, 0x5d, page);
    agfx_test_owned_context_destroy(bridge);
    g_assert_cmpuint(qatomic_read(&owned_context_release_calls), ==, 1);
    assert_owned_acquisition_stopped(context);
    object_unref(OBJECT(state));
    g_assert_false(agfx_eeee_native_context_retire(context));
    g_assert_true(state->eeee_native_context == context);
    g_assert_true(state->eeee_provider_state == provider_state);
    g_assert_cmpuint(OBJECT(state)->ref, ==, 1);
    g_assert_cmpuint(gfx_finalized, ==, 0);
    for (uint64_t index = 0; index < page; ++index) {
        g_assert_cmpuint(((uint8_t *)(uintptr_t)address)[index], ==, 0x5d);
    }
    /* Prior acquired table remains valid under the SAME preserved root.
     * No reacquisition, reconstructed provider or replacement context. */
    g_assert_cmpint(callbacks.eeee_task_deallocate(state, address, page), ==, 0);
    g_assert_true(agfx_eeee_native_context_retire(context));
    g_assert_cmpuint(gfx_finalized, ==, 1);
    g_assert_cmpuint(qatomic_read(&owned_context_release_calls), ==, 1);
}

/* New group only. Actual machine-none PCI factory from mmio-read323..357,
 * actual RAM, original physical Root/child grammar, real callback transport. */
#include "hw/pci/pci_host.h"
#include "hw/pci/msi.h"
#include "pvg/pvg_fifo.h"
#include "pvg/pvg_regs.h"
#include <fcntl.h>

#define MAP_RAM_GPA UINT64_C(0x100000)
#define MAP_RAM_SIZE UINT64_C(0x500000)
#define MAP_ROOT_GPA UINT64_C(0x220000)
#define MAP_NODE_GPA UINT64_C(0x36d414)
#define MAP_TABLE_GPA UINT64_C(0x36e000)
#define MAP_RING_GPA UINT64_C(0x36f000)

typedef struct MapProbeControl {
    AppleGfxMLState *state;
    qmu_extended_callbacks original;
    qmu_session_work *expected_work;
    AgfxTestMapRead reads[128];
    unsigned read_count;
    int vector;
} MapProbeControl;
static MapProbeControl *map_probe;

qmu_session *__real_qmu_create_extended(const qmu_extended_config *,
                                        const qmu_extended_callbacks *);
qmu_session *__wrap_qmu_create_extended(const qmu_extended_config *,
                                        const qmu_extended_callbacks *);

static int map_probe_read_memory(void *opaque, uint64_t gpa, void *bytes, size_t size)
{
    MapProbeControl *c = map_probe;
    qmu_session_work *before = qmu_session_current_host_work(c->state->qmu_dev);
    AgfxTestMapRead *row = NULL;
    g_assert_true(opaque == c->state && c->original.user_ctx == opaque);
    if (c->vector >= 0 && before == c->expected_work) {
        g_assert_nonnull(before);
        g_assert_false(bql_locked());
        g_assert_cmpuint(c->read_count, <, G_N_ELEMENTS(c->reads));
        row = &c->reads[c->read_count++];
        *row = (AgfxTestMapRead) { .vector = c->vector, .gpa = gpa,
                                  .bytes = size, .entered = 1 };
    }
    /* No injected value/failure: exact saved production callback and arguments. */
    int result = c->original.read_memory(opaque, gpa, bytes, size);
    if (row) {
        g_assert_true(qmu_session_current_host_work(c->state->qmu_dev) == before);
        row->result = result;
        row->returned = 1;
        if (result == 0) {
            row->prefix_bytes = MIN(size, sizeof(row->prefix));
            memcpy(row->prefix, bytes, row->prefix_bytes);
        }
    }
    return result;
}
qmu_session *__wrap_qmu_create_extended(const qmu_extended_config *cfg,
                                        const qmu_extended_callbacks *callbacks)
{
    if (!map_probe) return __real_qmu_create_extended(cfg, callbacks);
    g_assert_true(callbacks->user_ctx == map_probe->state);
    g_assert_nonnull(callbacks->read_memory);
    g_assert_nonnull(callbacks->write_memory);
    g_assert_nonnull(callbacks->raise_irq);
    g_assert_cmpuint(callbacks->struct_size, ==, sizeof(*callbacks));
    map_probe->original = *callbacks;
    qmu_extended_callbacks observed = *callbacks;
    observed.read_memory = map_probe_read_memory;
    return __real_qmu_create_extended(cfg, &observed);
}

typedef struct MapProbeWorker { AgfxTestMapMMIO *call; int done; } MapProbeWorker;
static void *map_probe_worker(void *opaque)
{
    MapProbeWorker *job = opaque;
    rcu_register_thread();
    g_assert_false(bql_locked());
    agfx_test_map_mmio_worker(job->call); /* Entire product call/catch is C++. */
    rcu_unregister_thread();
    qatomic_set(&job->done, 1);
    aio_wait_kick();
    return NULL;
}
static uint64_t map_probe_mmio(uint32_t offset, uint64_t value, bool read)
{
    MapProbeControl *c = map_probe;
    AgfxTestMapMMIO *call = agfx_test_map_mmio_new(c->state->qmu_dev, offset, value, read);
    g_assert_nonnull(call);
    c->expected_work = agfx_test_map_mmio_work(call);
    MapProbeWorker job = { .call = call };
    QemuThread worker;
    qemu_thread_create(&worker, "map-observer", map_probe_worker, &job, QEMU_THREAD_JOINABLE);
    const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!qatomic_read(&job.done)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&worker); /* Not merely event-set / worker done-store. */
    uint64_t result = 0;
    g_assert_cmpint(agfx_test_map_mmio_finish(call, &result), ==, 1);
    c->expected_work = NULL;
    return result;
}
static uint8_t *map_probe_ram(JoinRamOwner *ram, uint64_t gpa, size_t bytes)
{
    g_assert_cmpuint(gpa, >=, MAP_RAM_GPA);
    g_assert_cmpuint(gpa - MAP_RAM_GPA + bytes, <=, MAP_RAM_SIZE);
    return (uint8_t *)memory_region_get_ram_ptr(&ram->ram) + (gpa - MAP_RAM_GPA);
}
static void map_probe_root(JoinRamOwner *ram, uint32_t *cursor, uint32_t stamp,
                           uint16_t opcode, const void *payload, size_t length)
{
    PVGRootCmdHeader header = { .opcode = opcode, .total_size = sizeof(header) + length,
                               .stamp_value = stamp };
    g_assert_cmpuint(*cursor + header.total_size, <=, 4096);
    uint8_t *out = map_probe_ram(ram, MAP_ROOT_GPA + *cursor, header.total_size);
    memcpy(out, &header, sizeof(header));
    memcpy(out + sizeof(header), payload, length);
    *cursor += header.total_size;
    (void)map_probe_mmio(PVG_REG_FIFO_WRITE, *cursor, false);
}

static void test_map_first_leaf_original_mmio(void)
{
    static const TypeInfo host_type = { .name = "test-map-first-leaf-host",
        .parent = TYPE_PCI_HOST_BRIDGE, .instance_size = sizeof(PCIHostState) };
    const char *path = g_getenv("AGFX_MAP_GATE_RECORDS");
    g_assert_nonnull(path); /* Root-owned durable path; no silent /tmp artifact. */
    int fd = open(path, O_CREAT | O_EXCL | O_RDWR | O_APPEND | O_CLOEXEC, 0600);
    g_assert_cmpint(fd, >=, 0);
    char fd_text[32];
    g_snprintf(fd_text, sizeof(fd_text), "%d", fd);
    g_setenv("QMU_MAP_FIRST_LEAF_FD", fd_text, true);
    g_setenv("QMU_MAP_FIRST_LEAF_TASK", "1", true);
    g_setenv("QMU_MAP_FIRST_LEAF_LIMIT", "10", true);
    type_register_static(&host_type);
    PCIHostState *host = PCI_HOST_BRIDGE(object_new(host_type.name));
    host->bus = pci_root_bus_new(DEVICE(host), "map.pcie", get_system_memory(),
                                 get_system_io(), 0, TYPE_PCIE_BUS);
    g_assert_true(qbus_realize(BUS(host->bus), &error_abort));
    MapProbeControl c = { .vector = -1 };
    map_probe = &c;
    c.state = APPLE_GFX_ML(object_new(TYPE_APPLE_GFX_ML));
    DEVICE(c.state)->hotplugged = false;
    c.state->vram_size_mb = 1;
    c.state->display_width = 16;
    c.state->display_height = 12;
    bool saved_msi = msi_nonbroken;
    msi_nonbroken = true;
    g_assert_true(qdev_realize(DEVICE(c.state), BUS(host->bus), &error_abort));
    g_assert_nonnull(c.state->qmu_dev); /* Actual production factory, never skip. */
    JoinRamOwner *ram = (JoinRamOwner *)object_new(TYPE_JOIN_RAM_OWNER);
    g_assert_true(memory_region_init_ram_nomigrate(&ram->ram, OBJECT(ram),
                    "map-first-leaf-input", MAP_RAM_SIZE, &error_abort));
    memory_region_add_subregion(get_system_memory(), MAP_RAM_GPA, &ram->ram);
    memset(memory_region_get_ram_ptr(&ram->ram), 0, MAP_RAM_SIZE);
    bql_unlock(); drain_call_rcu(); bql_lock();
    (void)map_probe_mmio(PVG_REG_IOSFC_ENABLE, 0, false);
    (void)map_probe_mmio(PVG_REG_IOSFC_ENABLE, 1, false);
    (void)map_probe_mmio(PVG_REG_IOSFC_TASK_PFN, 0x100, false);
    (void)map_probe_mmio(PVG_REG_IOSFC_TASK_SIZE, 0x1000, false);
    (void)map_probe_mmio(PVG_REG_PROTOCOL_VERSION, 6, false);
    (void)map_probe_mmio(PVG_REG_FIFO_PFN, MAP_ROOT_GPA >> 12, false);
    (void)map_probe_mmio(PVG_REG_FIFO_SIZE, 0x1000, false);
    (void)map_probe_mmio(PVG_REG_FIFO_OFFSET, 0, false);
    (void)map_probe_mmio(PVG_REG_FIFO_ENABLE, 1, false);
    (void)map_probe_mmio(PVG_REG_SHARED_STATE_PFN, 0x36d, false);
    uint32_t root_cursor = 0, root_stamp = 1;
    PVGRootCmdDefineTask task = { .task_id_encoded = 2, .task_size = 0x400000,
                                 .page_table_pfn = 0x445 };
    map_probe_root(ram, &root_cursor, root_stamp++, PVG_ROOT_CMD_DEFINE_TASK, &task, sizeof(task));
    uint32_t node[5] = { 0, 0, 0, 2, MAP_TABLE_GPA >> 12 };
    uint32_t pfn_table[16] = { MAP_RING_GPA >> 12 };
    memcpy(map_probe_ram(ram, MAP_NODE_GPA, sizeof(node)), node, sizeof(node));
    memcpy(map_probe_ram(ram, MAP_TABLE_GPA, sizeof(pfn_table)), pfn_table, sizeof(pfn_table));
    PVGRootCmdDefineChildFIFO child = { .channel_id = 2 };
    map_probe_root(ram, &root_cursor, root_stamp++, PVG_ROOT_CMD_DEFINE_CHILD_FIFO,
                   &child, sizeof(child));
    const uint32_t root_info[2] = { 0x470, 1 };
    memcpy(map_probe_ram(ram, 0x445000, sizeof(root_info)), root_info, sizeof(root_info));
    uint32_t child_cursor = 0;
    AgfxTestMapTask actual_tasks[3];
    for (unsigned vector = 0; vector != 3; ++vector) {
        if (vector == 2) {
            PVGRootCmdDeleteTask erase = { .task_id = 1 };
            map_probe_root(ram, &root_cursor, root_stamp++, PVG_ROOT_CMD_DELETE_TASK,
                           &erase, sizeof(erase));
            task.page_table_pfn = 0xdeadbeef; /* Unmapped real AS => actual DMA failure. */
            map_probe_root(ram, &root_cursor, root_stamp++, PVG_ROOT_CMD_DEFINE_TASK,
                           &task, sizeof(task));
        } else {
            const uint32_t raw = 0x80000000U | (vector ? 1 : 0x472);
            memcpy(map_probe_ram(ram, 0x470008, sizeof(raw)), &raw, sizeof(raw));
        }
        struct QEMU_PACKED { uint32_t task; uint64_t offset, length; } map =
            { 1, 0x2000, 0x1000 };
        G_STATIC_ASSERT(sizeof(map) == 20);
        PVGRootCmdHeader header = { .opcode = PVG_GPU_CMD_MAP_MEMORY2,
            .total_size = sizeof(header) + sizeof(map), .stamp_value = 46 + vector };
        uint8_t *packet = map_probe_ram(ram, MAP_RING_GPA + child_cursor, header.total_size);
        memcpy(packet, &header, sizeof(header));
        memcpy(packet + sizeof(header), &map, sizeof(map));
        child_cursor += header.total_size;
        memcpy(map_probe_ram(ram, MAP_NODE_GPA, sizeof(child_cursor)),
               &child_cursor, sizeof(child_cursor)); // Preserve actual read/fault words.
        g_assert_cmpint(agfx_test_map_task_snapshot(c.state->qmu_dev,
                            &actual_tasks[vector]), ==, 0);
        c.vector = vector;
        (void)map_probe_mmio(PVG_REG_KICK, 2, false);
        c.vector = -1;
        uint32_t stamp, read, fault;
        memcpy(&stamp, map_probe_ram(ram, MAP_ROOT_GPA + 8, 4), 4);
        memcpy(&read, map_probe_ram(ram, MAP_NODE_GPA + 4, 4), 4);
        memcpy(&fault, map_probe_ram(ram, MAP_NODE_GPA + 8, 4), 4);
        g_assert_cmpuint(stamp, ==, 46 + vector);
        g_assert_cmpuint(read, ==, child_cursor);
        g_assert_cmpuint(fault, ==, UINT32_MAX);
        g_assert_cmpuint(map_probe_mmio(PVG_REG_FAULT_STATUS, 0, true), ==, 0);
    }
    int oracle = agfx_test_map_verify_records(fd, c.reads, c.read_count, actual_tasks);
    g_test_message("record_oracle_line=%d real_read_RETURNs=%u; PFN1 is observed, never D success",
                   oracle, c.read_count);
    g_assert_cmpint(oracle, ==, 0);
    /* Same original actual lifecycle teardown; FD stays open until Session exit. */
    qdev_unrealize(DEVICE(c.state));
    g_assert_null(c.state->qmu_dev);
    object_unparent(OBJECT(c.state)); object_unref(OBJECT(c.state));
    memory_region_del_subregion(get_system_memory(), &ram->ram);
    object_unref(OBJECT(ram));
    bql_unlock(); drain_call_rcu(); bql_lock();
    bql_unlock(); drain_call_rcu(); bql_lock();
    pci_root_bus_cleanup(host->bus); object_unref(OBJECT(host));
    bql_unlock(); drain_call_rcu(); bql_lock();
    msi_nonbroken = saved_msi;
    map_probe = NULL;
    g_unsetenv("QMU_MAP_FIRST_LEAF_FD");
    g_unsetenv("QMU_MAP_FIRST_LEAF_TASK");
    g_unsetenv("QMU_MAP_FIRST_LEAF_LIMIT");
    g_assert_cmpint(close(fd), ==, 0); // Retain artifact bytes; no unlink.
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
    g_test_add_func("/eeee/owned-context/descendant-native-window",
                    test_owned_context_descendant_native_window);
    g_test_add_func("/eeee/owned-context/retire-waits-last-mr-return",
                    test_owned_context_retire_waits_last_mr_return);
    g_test_add_func("/eeee/owned-context/busy-preserves-root-retry",
                    test_owned_context_busy_preserves_root_retry);
    g_test_add_func("/eeee/map-first-leaf/original-mmio-three-vectors",
                    test_map_first_leaf_original_mmio);
    status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
