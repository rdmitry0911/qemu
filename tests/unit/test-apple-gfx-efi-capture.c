/* New connected typed fixture, derived from accepted native551. The old
 * fixtures/raw evidence remain unchanged. Syscall controls are CPU-only;
 * native construction runs separately with every wrapper unarmed. */
int agfx_retained_capture_donor_main(int argc, char **argv);
#define main agfx_retained_capture_donor_main
#include "test-apple-gfx-eeee-qmetal-join.c"
#undef main
#include "hw/display/apple-gfx-efi-capture.h"
#include "apple-gfx-efi-capture-test-bridge.h"
#include "qemu/host-utils.h"
#include "qemu/units.h"

typedef struct TypedFixture {
    AppleGfxMLState state; /* actual callback context, always first */
    qmu_extended_callbacks facilities;
    JoinRamOwner *ram_owner;
    uint8_t *canonical;
    uint64_t address, span, page;
    unsigned finalized, captures;
    qmu_efi_capture_request_v2 observed_request;
    qmu_efi_capture_result_v2 observed_result; /* borrowed facts, no extra ref */
    int bound, allow_retire, done;
} TypedFixture;

AgfxEeeeSourceDmaBuf *__real_agfx_eeee_source_dmabuf_create(
    AgfxEeeeSourceBacking *, AgfxEeeeDmaBufFailure *);
AgfxEeeeSourceDmaBuf *__wrap_agfx_eeee_source_dmabuf_create(
    AgfxEeeeSourceBacking *, AgfxEeeeDmaBufFailure *);

AgfxEeeeSourceDmaBuf *__wrap_agfx_eeee_source_dmabuf_create(
    AgfxEeeeSourceBacking *source, AgfxEeeeDmaBufFailure *failure)
{
    /* Observe the actual adapter's one capture, then forward unchanged to
     * the real production exporter. Only existing syscall wrappers simulate
     * descriptors while explicitly armed by a CPU-only test below. */
    if (export_control.armed) {
        g_assert_null(export_control.source);
        export_control.source = source;
    }
    return __real_agfx_eeee_source_dmabuf_create(source, failure);
}

static void typed_fixture_create(TypedFixture *f, uint64_t span)
{
    f->page = qemu_real_host_page_size();
    f->span = span;
    f->state.eeee_provider_state = agfx_eeee_provider_state_create();
    f->facilities = (qmu_extended_callbacks) {
        .abi_magic = QMU_EXTENDED_CALLBACKS_ABI_MAGIC,
        .abi_version = QMU_EXTENDED_ABI_VERSION,
        .struct_size = sizeof(f->facilities),
        .user_ctx = &f->state,
    };
    agfx_eeee_provider_fill_callbacks(&f->facilities);
    f->ram_owner = (JoinRamOwner *)object_new(TYPE_JOIN_RAM_OWNER);
    f->ram_owner->finalized = &f->finalized;
    int fd = qemu_memfd_create("efi-typed-original-pages", span + f->page,
        false, 0, F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL, &error_abort);
    g_assert_cmpint(fd, >=, 0);
    g_assert_true(memory_region_init_ram_from_fd(&f->ram_owner->ram,
        OBJECT(f->ram_owner), "efi-typed-original-pages", span, RAM_SHARED,
        fd, f->page, &error_abort));
    f->canonical = memory_region_get_ram_ptr(&f->ram_owner->ram);
    memset(f->canonical, 0x47, span);
    f->facilities.eeee_task_lock(&f->state);
    f->facilities.eeee_task_region_ref(&f->state, &f->ram_owner->ram);
    f->facilities.eeee_task_unlock(&f->state);
    g_assert_cmpint(f->facilities.eeee_task_allocate(&f->state, &f->address,
        span, QMU_EEEE_TASK_ALLOCATE_ANYWHERE), ==, 1);
    source_remap(&f->state, &f->facilities, f->address, span, f->canonical);
}

static void typed_fixture_destroy_logical(TypedFixture *f)
{
    f->facilities.eeee_task_region_unref(&f->state, &f->ram_owner->ram);
    g_assert_cmpint(f->facilities.eeee_task_deallocate(&f->state,
        f->address, f->span), ==, 0);
    g_assert_true(agfx_eeee_provider_state_destroy(f->state.eeee_provider_state));
    f->state.eeee_provider_state = NULL;
    object_unref(OBJECT(f->ram_owner));
    f->ram_owner = NULL;
}

static qmu_efi_capture_request_v2 typed_request(TypedFixture *f, uint32_t kind)
{
    return (qmu_efi_capture_request_v2) {
        .abi_version = QMU_EFI_CAPTURE_ABI_VERSION,
        .struct_size = sizeof(qmu_efi_capture_request_v2),
        .source_kind = kind,
        .host_page_bytes = f->page,
        .task_virtual_address = f->address + 17,
        .frame_storage_bytes = f->page - 34,
        .native_minimum_allocation_bytes =
            kind == QMU_EFI_SOURCE_DMA_BUF ? 2 * f->page : 0,
        .native_import_alignment = 2 * f->page,
    };
}

static void typed_assert_zero(const qmu_efi_capture_result_v2 *result)
{
    const qmu_efi_capture_result_v2 zero = { 0 };
    g_assert_cmpmem(result, sizeof(*result), &zero, sizeof(zero));
}

static void test_typed_capture_rejections(void)
{
    TypedFixture f = {};
    typed_fixture_create(&f, 4 * qemu_real_host_page_size());
    const qmu_efi_capture_request_v2 valid =
        typed_request(&f, QMU_EFI_SOURCE_HOST_ALLOCATION);
    for (unsigned variant = 0; variant < 12; ++variant) {
        qmu_efi_capture_request_v2 request = valid;
        qmu_efi_capture_result_v2 result;
        memset(&result, 0xa5, sizeof(result));
        switch (variant) {
        case 0: request.abi_version = 1; break;
        case 1: --request.struct_size; break;
        case 2: request.reserved = 1; break;
        case 3: request.host_page_bytes *= 2; break;
        case 4: request.source_kind = QMU_EFI_SOURCE_NONE; break;
        case 5: request.native_import_alignment = 0; break;
        case 6: request.native_import_alignment = f.page + 1; break;
        case 7: request.native_minimum_allocation_bytes = f.page; break;
        case 8: request.frame_storage_bytes = UINT64_MAX; break;
        case 9: request.source_kind = QMU_EFI_SOURCE_DMA_BUF;
                request.native_minimum_allocation_bytes = 1; break;
        case 10: request.source_kind = QMU_EFI_SOURCE_DMA_BUF;
                 request.native_minimum_allocation_bytes = 8 * f.page; break;
        case 11: request.task_virtual_address = f.address + f.span + 17; break;
        default: g_assert_not_reached();
        }
        bql_unlock();
        g_assert_cmpint(agfx_efi_source_capture_v2(&f.state,
            &request, &result), ==, 0);
        bql_lock();
        typed_assert_zero(&result);
    }
    qmu_efi_capture_result_v2 result;
    memset(&result, 0xa5, sizeof(result));
    bql_unlock();
    g_assert_cmpint(agfx_efi_source_capture_v2(&f.state, NULL, &result), ==, 0);
    bql_lock();
    typed_assert_zero(&result);
    typed_fixture_destroy_logical(&f);
    g_assert_cmpuint(f.finalized, ==, 1);
    g_assert_false(export_control.armed);
}

static void test_typed_host_capture(void)
{
    TypedFixture f = {};
    typed_fixture_create(&f, 4 * qemu_real_host_page_size());
    qmu_efi_capture_request_v2 request =
        typed_request(&f, QMU_EFI_SOURCE_HOST_ALLOCATION);
    qmu_efi_capture_result_v2 result = {};
    bql_unlock();
    g_assert_cmpint(agfx_efi_source_capture_v2(&f.state, &request, &result), ==, 1);
    bql_lock();
    g_assert_cmpuint(result.abi_version, ==, QMU_EFI_CAPTURE_ABI_VERSION);
    g_assert_cmpuint(result.struct_size, ==, sizeof(result));
    g_assert_cmpuint(result.source_kind, ==, QMU_EFI_SOURCE_HOST_ALLOCATION);
    g_assert_cmpuint(result.reserved, ==, 0);
    g_assert_cmpuint(result.allocation_bytes, ==, 2 * f.page);
    g_assert_cmpuint(result.visible_offset, ==, 17);
    g_assert_cmpuint(result.visible_bytes, ==, request.frame_storage_bytes);
    g_assert_cmpuint((uintptr_t)result.payload.host_allocation_base % (2 * f.page), ==, 0);
    g_assert_nonnull(result.opaque_owner);
    g_assert_nonnull(result.release_owner);
    /* Actual retained shared CPU alias, no native import/access inference. */
    f.canonical[17] = 0x81;
    g_assert_cmpuint(((const uint8_t *)result.payload.host_allocation_base)[17], ==, 0x81);
    bql_unlock();
    result.release_owner(result.opaque_owner);
    bql_lock();
    typed_fixture_destroy_logical(&f);
    g_assert_cmpuint(f.finalized, ==, 1);
}

static void test_typed_export_control(gconstpointer opaque)
{
    const ExportControl mode = GPOINTER_TO_INT(opaque);
    TypedFixture f = {};
    typed_fixture_create(&f, 4 * qemu_real_host_page_size());
    qmu_efi_capture_request_v2 request =
        typed_request(&f, QMU_EFI_SOURCE_DMA_BUF);
    qmu_efi_capture_result_v2 result;
    memset(&result, 0xa5, sizeof(result));
    export_control = (typeof(export_control)) {
        .armed = true, .mode = mode, .control_fd = INT_MAX - 1,
        .output_fd = mode == EXPORT_FD_ZERO ? 0 : INT_MAX,
    };
    bql_unlock();
    const int32_t returned = agfx_efi_source_capture_v2(&f.state, &request, &result);
    const bool success = mode == EXPORT_FD_ZERO;
    g_assert_cmpint(returned, ==, success);
    if (success) {
        g_assert_cmpuint(result.source_kind, ==, QMU_EFI_SOURCE_DMA_BUF);
        g_assert_cmpint(result.payload.dma_buf.original_fd, ==, 0);
        g_assert_cmpuint(result.payload.dma_buf.reserved, ==, 0);
        g_assert_cmpuint(result.allocation_bytes, ==, 2 * f.page);
        g_assert_cmpuint(result.visible_offset, ==, 17);
        g_assert_cmpuint(result.visible_bytes, ==, request.frame_storage_bytes);
        g_assert_nonnull(result.opaque_owner);
        g_assert_nonnull(result.release_owner);
        g_assert_true(agfx_eeee_source_dmabuf_source(result.opaque_owner) ==
                      export_control.source);
        result.release_owner(result.opaque_owner);
        g_assert_cmpuint(export_control.output_closes, ==, 1);
    } else {
        typed_assert_zero(&result);
        g_assert_cmpuint(export_control.output_closes, ==, 0);
    }
    g_assert_cmpuint(export_control.opens, ==, 1);
    g_assert_cmpuint(export_control.ioctls, ==, mode == EXPORT_OPEN_ERROR ? 0 : 1);
    g_assert_cmpuint(export_control.control_closes, ==,
        mode == EXPORT_OPEN_ERROR ? 0 : 1);
    export_control.armed = false;
    bql_lock();
    typed_fixture_destroy_logical(&f);
    g_assert_cmpuint(f.finalized, ==, 1);
    g_test_message("CPU syscall boundary only: simulated fd0/failure, not native DMA evidence");
}

static int32_t typed_observed_capture(void *context,
    const qmu_efi_capture_request_v2 *request, qmu_efi_capture_result_v2 *result)
{
    TypedFixture *f = context; /* state is first, same actual State address */
    g_assert_false(export_control.armed);
    g_assert_cmpuint(f->captures++, ==, 0);
    f->observed_request = *request;
    const int32_t returned = agfx_efi_source_capture_v2(&f->state, request, result);
    g_assert_cmpint(returned, ==, 1);
    f->observed_result = *result; /* borrowed observations, not a second owner */
    g_assert_cmpuint(request->host_page_bytes, ==, f->page);
    g_assert_cmpuint(request->native_minimum_allocation_bytes, >=,
        result->visible_offset + result->visible_bytes);
    const uint64_t alignment = MAX(f->page, request->native_import_alignment);
    const uint64_t expected_A = ROUND_UP(MAX(result->visible_offset +
        result->visible_bytes, request->native_minimum_allocation_bytes), alignment);
    g_assert_cmpuint(result->allocation_bytes, ==, expected_A);
    g_assert_cmpuint(result->allocation_bytes, <=, f->span);
    g_assert_cmpuint(result->visible_offset, ==, 20);
    g_assert_cmpuint(result->visible_bytes, ==, f->page - 40);
    g_assert_cmpuint(result->source_kind, ==, QMU_EFI_SOURCE_DMA_BUF);
    g_assert_cmpuint(result->reserved, ==, 0);
    g_assert_cmpuint(result->payload.dma_buf.reserved, ==, 0);
    g_assert_cmpint(result->payload.dma_buf.original_fd, >=, 0);
    const AgfxEeeeSourceBacking *source =
        agfx_eeee_source_dmabuf_source(result->opaque_owner);
    g_assert_nonnull(source);
    g_assert_cmpuint(agfx_eeee_source_backing_interval_count(source), ==, 1);
    AgfxEeeeSourceInterval interval;
    g_assert_true(agfx_eeee_source_backing_interval(source, 0, &interval));
    g_assert_true(interval.region == &f->ram_owner->ram);
    g_assert_cmpuint(interval.file_offset, ==, f->page);
    g_assert_cmpuint(interval.allocation_offset, ==, 0);
    g_assert_cmpuint(interval.length, ==, expected_A);
    struct stat canonical_stat, source_stat;
    g_assert_cmpint(fstat(memory_region_get_fd(&f->ram_owner->ram), &canonical_stat), ==, 0);
    g_assert_cmpint(fstat(interval.fd, &source_stat), ==, 0);
    g_assert_cmpuint(canonical_stat.st_dev, ==, source_stat.st_dev);
    g_assert_cmpuint(canonical_stat.st_ino, ==, source_stat.st_ino);
    return returned;
}

static void typed_after_bound(void *opaque)
{
    TypedFixture *f = opaque;
    qatomic_set(&f->bound, 1);
    const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!qatomic_read(&f->allow_retire)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        g_thread_yield();
    }
}

static void *typed_native_worker(void *opaque)
{
    TypedFixture *f = opaque;
    rcu_register_thread();
    g_assert_false(bql_locked());
    agfx_test_efi_storage_run(&f->facilities, typed_observed_capture,
        f->address + 20, f->page - 40, &f->observed_result,
        typed_after_bound, f);
    g_assert_false(bql_locked());
    qatomic_set(&f->done, 1);
    rcu_unregister_thread();
    return NULL;
}

static void typed_pump_until(int *flag)
{
    const gint64 deadline = g_get_monotonic_time() + 10 * G_USEC_PER_SEC;
    while (!qatomic_read(flag)) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        aio_poll(qemu_get_aio_context(), false);
        g_thread_yield();
    }
}

static void test_typed_native_storage(void)
{
    TypedFixture f = {};
    g_assert_false(export_control.armed);
    g_assert_cmpuint(source_fail_fixed_ordinal, ==, 0);
    /* Bounded real saved backing exists before native requirements. The
     * actual storage class queries R for its ONE buffer then captures only
     * true A, verified by the callback observer. This is not a guessed R or
     * product allocation limit, and does not repeat a second native query. */
    typed_fixture_create(&f, 64 * MiB);
    QemuThread worker;
    qemu_thread_create(&worker, "efi-typed-native", typed_native_worker,
                       &f, QEMU_THREAD_JOINABLE);
    typed_pump_until(&f.bound);
    typed_fixture_destroy_logical(&f);
    bql_unlock();
    drain_call_rcu();
    g_assert_cmpuint(f.finalized, ==, 0);
    bql_lock();
    qatomic_set(&f.allow_retire, 1);
    typed_pump_until(&f.done);
    qemu_thread_join(&worker);
    g_assert_cmpuint(f.captures, ==, 1);
    g_assert_cmpuint(f.finalized, ==, 1);
    g_assert_false(export_control.armed);
    g_test_message("actual typed source/export/import/view/retire; no draw, shader read or cache-access parity");
}

int main(int argc, char **argv)
{
    char options[][16] = {"-machine", "none", "-accel", "tcg", "-nodefaults",
                          "-display", "none", "-S"};
    char *qemu_argv[] = {argv[0], options[0], options[1], options[2], options[3],
                        options[4], options[5], options[6], options[7], NULL};
    g_test_init(&argc, &argv, NULL);
    qemu_init(ARRAY_SIZE(qemu_argv) - 1, qemu_argv);
    type_register_static(&join_owner_type);
    g_test_add_func("/eeee/typed-capture/rejections", test_typed_capture_rejections);
    g_test_add_func("/eeee/typed-capture/host-live-alias", test_typed_host_capture);
    g_test_add_data_func("/eeee/typed-capture/cpu-fd0", GINT_TO_POINTER(EXPORT_FD_ZERO),
                         test_typed_export_control);
    g_test_add_data_func("/eeee/typed-capture/cpu-open-error", GINT_TO_POINTER(EXPORT_OPEN_ERROR),
                         test_typed_export_control);
    g_test_add_data_func("/eeee/typed-capture/cpu-ioctl-error", GINT_TO_POINTER(EXPORT_IOCTL_INVALID),
                         test_typed_export_control);
    g_test_add_func("/eeee/typed-capture/native-storage-view-retire", test_typed_native_storage);
    const int status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
