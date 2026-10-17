/* Reuse the actual RAM/provider/source fixture; do not copy its implementation
 * or run its old mock groups. Its syscall wrappers remain UNARMED throughout. */
int agfx_retained_join_donor_main(int argc, char **argv);
#define main agfx_retained_join_donor_main
#include "test-apple-gfx-eeee-qmetal-join.c"
#undef main
#include "apple-gfx-eeee-dma-import-test-bridge.h"
#include "qemu/host-utils.h"
#include "qemu/units.h"

typedef struct NativeSourceOwner {
    AgfxEeeeSourceDmaBuf *export;
    unsigned releases;
} NativeSourceOwner;

static void native_source_release(void *opaque)
{
    NativeSourceOwner *source = opaque;
    g_assert_false(bql_locked());
    g_assert_cmpuint(source->releases++, ==, 0);
    g_assert_nonnull(source->export);
    agfx_eeee_source_dmabuf_unref(source->export);
    source->export = NULL;
    g_assert_false(bql_locked());
}

typedef struct NativeImportRelease {
    AgfxNativeDmaOwner *import;
    NativeSourceOwner *source;
    int fd;
    uint64_t A, V, L;
    int done;
} NativeImportRelease;

static void *native_import_retire_worker(void *opaque)
{
    NativeImportRelease *call = opaque;
    rcu_register_thread();
    g_assert_false(bql_locked());
    agfx_test_dma_import_retire(call->import, call->fd,
        call->A, call->V, call->L, call->source, native_source_release);
    g_assert_false(bql_locked());
    qatomic_set(&call->done, 1);
    rcu_unregister_thread();
    return NULL;
}

static void test_native_dma_import(void)
{
    AppleGfxMLState ctx = {};
    qmu_extended_callbacks cb = { .user_ctx = &ctx };
    AgfxNativeDmaRequirements requirements = {};
    AgfxEeeeDmaBufFailure failure = {};
    NativeSourceOwner exported = {};
    unsigned finalized = 0;
    uint64_t address, span, alignment;
    const uint64_t page = qemu_real_host_page_size(), V = 17;
    g_assert_cmpuint(page, >, 34);
    const uint64_t L = page - 34;
    g_assert_false(export_control.armed);
    g_assert_cmpuint(source_fail_fixed_ordinal, ==, 0);

    /* No QEMU/provider lock crosses foreign native entry. Requirements precede
     * source acquisition, so the entire allocation is real admitted RAM. */
    bql_unlock();
    AgfxNativeDmaOwner *import = agfx_test_dma_prepare(V, L, &requirements);
    bql_lock();
    g_assert_nonnull(import);
    alignment = MAX(page, requirements.alignment);
    g_assert_true(is_power_of_2(alignment));
    g_assert_cmpuint(requirements.requirements_bytes, >=, requirements.buffer_bytes);
    g_assert_cmpuint(requirements.buffer_bytes, ==, V+L);
    g_assert_cmpuint(requirements.requirements_bytes, <=, 64 * MiB);
    g_assert_cmpuint(alignment, <=, 64 * MiB);
    span = ROUND_UP(MAX(V+L, requirements.requirements_bytes), alignment);
    g_assert_cmpuint(span, <=, 64 * MiB); /* bounded fixture resource, not a product limit */
    ctx.eeee_provider_state = agfx_eeee_provider_state_create();
    agfx_eeee_provider_fill_callbacks(&cb);
    JoinRamOwner *owner = (JoinRamOwner *)object_new(TYPE_JOIN_RAM_OWNER);
    owner->finalized = &finalized;
    int fd = qemu_memfd_create("eeee-native-dma-original-pages", span+page,
        false, 0, F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL, &error_abort);
    g_assert_cmpint(fd, >=, 0);
    g_assert_true(memory_region_init_ram_from_fd(&owner->ram, OBJECT(owner),
        "eeee-native-dma-original-pages", span, RAM_SHARED, fd, page, &error_abort));
    uint8_t *canonical = memory_region_get_ram_ptr(&owner->ram);
    memset(canonical, 0x47, span); /* before native export/attachment, never a live overwrite */
    cb.eeee_task_lock(&ctx);
    cb.eeee_task_region_ref(&ctx, &owner->ram);
    cb.eeee_task_unlock(&ctx);
    g_assert_cmpint(cb.eeee_task_allocate(&ctx, &address, span,
        QMU_EEEE_TASK_ALLOCATE_ANYWHERE), ==, 1);
    source_remap(&ctx, &cb, address, span, canonical);
    AgfxEeeeSourceBacking *source = agfx_eeee_source_backing_capture_extent(
        ctx.eeee_provider_state, address+V, L,
        requirements.requirements_bytes, alignment);
    g_assert_nonnull(source);
    g_assert_cmpuint(agfx_eeee_source_backing_allocation_length(source), ==, span);
    g_assert_cmpuint(agfx_eeee_source_backing_visible_offset(source), ==, V);
    g_assert_cmpuint(agfx_eeee_source_backing_length(source), ==, L);
    g_assert_cmpuint(agfx_eeee_source_backing_interval_count(source), ==, 1);
    AgfxEeeeSourceInterval interval;
    g_assert_true(agfx_eeee_source_backing_interval(source, 0, &interval));
    g_assert_true(interval.region == &owner->ram);
    g_assert_cmpuint(interval.file_offset, ==, page);
    g_assert_cmpuint(interval.allocation_offset, ==, 0);
    g_assert_cmpuint(interval.length, ==, span);
    struct stat original_stat, captured_stat;
    g_assert_cmpint(fstat(memory_region_get_fd(&owner->ram), &original_stat), ==, 0);
    g_assert_cmpint(fstat(interval.fd, &captured_stat), ==, 0);
    g_assert_cmpuint(original_stat.st_dev, ==, captured_stat.st_dev);
    g_assert_cmpuint(original_stat.st_ino, ==, captured_stat.st_ino);
    bql_unlock();
    exported.export = agfx_eeee_source_dmabuf_create(source, &failure);
    g_test_message("actual export failure stage=%u errno=%d interval=%u",
        failure.stage, failure.error_no, failure.interval);
    g_assert_nonnull(exported.export);
    g_assert_cmpint(failure.stage, ==, AGFX_DMABUF_NONE);
    const int native_fd = agfx_eeee_source_dmabuf_fd(exported.export);
    g_assert_cmpint(native_fd, >=, 0);
    g_assert_true(agfx_eeee_source_dmabuf_source(exported.export) == source);
    g_assert_cmpuint(agfx_eeee_source_dmabuf_allocation_length(exported.export), ==, span);
    bql_lock();
    agfx_eeee_source_backing_unref(source); /* only original export retains the source */
    cb.eeee_task_region_unref(&ctx, &owner->ram);
    g_assert_cmpint(cb.eeee_task_deallocate(&ctx, address, span), ==, 0);
    g_assert_true(agfx_eeee_provider_state_destroy(ctx.eeee_provider_state));
    ctx.eeee_provider_state = NULL;
    object_unref(OBJECT(owner));
    bql_unlock();
    drain_call_rcu();
    g_assert_cmpuint(finalized, ==, 0); /* real original source survives logical owner */
    bql_lock();
    /* Reuse the original export-owner fixture's worker/mainloop split. The
     * final source unref synchronously schedules its BQL leaf on this loop;
     * calling the foreign bridge on this loop would wait for itself. */
    NativeImportRelease release = { .import = import, .source = &exported,
        .fd = native_fd, .A = span, .V = V, .L = L };
    QemuThread worker;
    qemu_thread_create(&worker, "eeee-native-dma", native_import_retire_worker,
                       &release, QEMU_THREAD_JOINABLE);
    const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!qatomic_read(&release.done)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
    qemu_thread_join(&worker);
    g_assert_cmpuint(exported.releases, ==, 1);
    g_assert_null(exported.export);
    g_assert_cmpuint(finalized, ==, 1); /* actual final native/source RETURN */
    g_assert_false(export_control.armed);
    g_test_message("real original QEMU pages -> DMA FD -> same Vulkan buffer -> source retirement; NO GPU/access parity");
}

int main(int argc, char **argv)
{
    char options[][16] = {"-machine", "none", "-accel", "tcg", "-nodefaults",
                          "-display", "none", "-S"};
    char *qemu_argv[] = {argv[0], options[0], options[1], options[2], options[3],
                        options[4], options[5], options[6], options[7], NULL};
    g_test_init(&argc, &argv, NULL);
    qemu_init(ARRAY_SIZE(qemu_argv)-1, qemu_argv);
    type_register_static(&join_owner_type);
    g_test_add_func("/eeee/native-dma/original-pages-import-retire", test_native_dma_import);
    int status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
