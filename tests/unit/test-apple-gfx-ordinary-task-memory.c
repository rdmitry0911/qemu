/* Test-only compilation of the ACTUAL producer TU exposes its static leaves.
 * The fixture link replaces, never duplicates, hw_display_apple-gfx-ml.c.o.
 * Mock peer/session are explicit: no QMetal final-delete/GPU proof is claimed.
 */
#include "../../hw/display/apple-gfx-ml.c"
#include "system/system.h"
#include "system/replay.h"

int (*qemu_main)(void);
const struct qmu_private_ordinary_task_memory_v1 *ordinary_fixture_peer;

typedef struct OrdinaryControl {
    AppleGfxMLState *state;
    struct qmu_ordinary_task_memory_handoff_v1 handoff;
    const char *mode;
    unsigned prepare_calls, install_calls, dispose_calls;
    unsigned destroy_calls, quiesce_calls, rethrow_calls, finalized;
    bool lease_returned;
    char opaque_session;
} OrdinaryControl;
static OrdinaryControl ctl;

static qmu_session *mock_session(void)
{
    return (qmu_session *)&ctl.opaque_session;
}

qmu_status __wrap_qmu_session_quiesce_host_callback_producers_deferred(qmu_session *);
void __wrap_qmu_session_rethrow_quiesce(qmu_session *);
qmu_status __wrap_qmu_session_quiesce_host_callback_producers_deferred(qmu_session *s)
{
    g_assert_true(s == mock_session());
    g_assert_false(bql_locked());
    g_assert_cmpuint(++ctl.quiesce_calls, ==, 1);
    return QMU_OK;
}
void __wrap_qmu_session_rethrow_quiesce(qmu_session *s)
{
    g_assert_true(s == mock_session());
    g_assert_true(bql_locked());
    g_assert_cmpuint(ctl.quiesce_calls, ==, 1);
    g_assert_cmpuint(++ctl.rethrow_calls, ==, 1);
}

static void finalized(Object *obj, const char *name, void *opaque)
{
    g_assert_true(bql_locked());
    g_assert_cmpuint(ctl.finalized++, ==, 0);
}

static void *cpu_prepare(qmu_session *s,
        const struct qmu_ordinary_task_memory_handoff_v1 *h)
{
    g_assert_true(s == mock_session());
    g_assert_cmpuint(++ctl.prepare_calls, ==, 1);
    g_assert_cmpuint(h->version, ==, QMU_PRIVATE_ORDINARY_TASK_MEMORY_V1_VERSION);
    g_assert_cmpuint(h->struct_size, ==, sizeof(*h));
    g_assert_cmpuint(h->profile, ==, QMU_ORDINARY_TASK_MEMORY_LEGACY_CALLBACKS);
    g_assert_true(h->memory_ctx_identity == ctl.state);
    g_assert_true(h->facilities.user_ctx == ctl.state);
    g_assert_true(agfx_ordinary_task_memory_callbacks_complete(ctl.state,
                                                            &h->facilities));
    g_assert_true(h->acquired_context == ctl.state->ordinary_task_memory_context);
    AgfxEeeeNativeContext *ctx = h->acquired_context;
    g_assert_cmpuint(qatomic_read(&ctx->references), ==, 2);
    g_assert_cmpuint(h->facilities.eeee_task_host_page_size(h->facilities.user_ctx), >, 0);
    ctl.handoff = *h; /* Borrowed until the real nonnull prepare RETURN. */
    return !strcmp(ctl.mode, "prepare-null") ? NULL : &ctl;
}

static uint32_t cpu_install(void *receipt)
{
    g_assert_true(receipt == &ctl);
    g_assert_cmpuint(++ctl.install_calls, ==, 1);
    if (!strcmp(ctl.mode, "reject")) return QMU_ORDINARY_TASK_MEMORY_REJECTED;
    if (!strcmp(ctl.mode, "escape")) return QMU_ORDINARY_TASK_MEMORY_ESCAPE_RETAINED;
    if (!strcmp(ctl.mode, "unreturned")) return QMU_ORDINARY_TASK_MEMORY_UNRETURNED;
    if (!strcmp(ctl.mode, "unknown")) return 77;
    return QMU_ORDINARY_TASK_MEMORY_INSTALLED;
}

static bool cpu_dispose(void *receipt)
{
    g_assert_true(receipt == &ctl);
    g_assert_true(bql_locked());
    g_assert_cmpuint(++ctl.dispose_calls, ==, 1);
    g_assert_null(ctl.state->qmu_dev); /* Actual worker returned + store occurred. */
    g_assert_cmpuint(ctl.destroy_calls, ==, 1);
    AgfxEeeeNativeContext *ctx = ctl.handoff.acquired_context;
    g_assert_true(ctx->acquiring_stopped);
    g_assert_cmpuint(qatomic_read(&ctx->references), ==, 2);
    if (!strcmp(ctl.mode, "dispose-false")) {
        printf("FIXTURE_DISPOSE_FALSE_OWNER_HELD refs=2 destroy=1 dispose=1\n");
        fflush(stdout);
        return false;
    }
    ctl.handoff.release_context(ctl.handoff.acquired_context);
    ctl.lease_returned = true;
    return true;
}

static const struct qmu_private_ordinary_task_memory_v1 cpu_peer = {
    .version = QMU_PRIVATE_ORDINARY_TASK_MEMORY_V1_VERSION,
    .struct_size = sizeof(cpu_peer),
    .prepare = cpu_prepare, .install = cpu_install,
    .dispose_returned = cpu_dispose,
};

static bool v2_destroy(void *receipt, qmu_session *s)
{
    g_assert_null(receipt);
    g_assert_true(s == mock_session());
    g_assert_false(bql_locked());
    g_assert_cmpuint(++ctl.destroy_calls, ==, 1);
    if (!strcmp(ctl.mode, "normal") || !strcmp(ctl.mode, "dispose-false")) {
        g_assert_cmpuint(ctl.quiesce_calls, ==, 1);
        g_assert_cmpuint(ctl.rethrow_calls, ==, 1);
    }
    return true; /* Honest mock V2 peer RETURN; not an actual QMetal delete. */
}
static const struct qmu_private_ordinary_physical_capture_v2 v2_peer = {
    .version = QMU_PRIVATE_ORDINARY_PHYSICAL_CAPTURE_V2_VERSION,
    .struct_size = sizeof(v2_peer), .destroy_returned = v2_destroy,
};

static void drain_fixture(void)
{
    bql_unlock(); drain_call_rcu(); bql_lock();
}

static void start_idle_exit_roots(AppleGfxMLState *s)
{
    qemu_mutex_init(&s->mmio_job_mutex);
    qemu_mutex_init(&s->session_mutex);
    qemu_mutex_init(&s->bootstrap_present_mutex);
    qemu_cond_init(&s->bootstrap_present_cond);
    agfx_completion_queue_init(s);
    agfx_log_init(s);
    g_assert_true(memory_region_init_ram(&s->host_vram, OBJECT(s),
                                       "ordinary-fixture-vram", qemu_real_host_page_size(),
                                       &error_abort));
    qemu_sem_init(&s->mmio_sem, 0);
    qemu_thread_create(&s->mmio_worker, "fixture-real-agfx-io",
                      agfx_mmio_worker_thread, s, QEMU_THREAD_JOINABLE);
    qemu_thread_create(&s->bootstrap_present_worker, "fixture-real-agfx-present",
                      agfx_bootstrap_present_thread, s, QEMU_THREAD_JOINABLE);
}

static void run_mode(const char *mode)
{
    Error *err = NULL;
    AppleGfxMLState *s = APPLE_GFX_ML(object_new(TYPE_APPLE_GFX_ML));
    ctl = (OrdinaryControl){ .state = s, .mode = mode };
    object_property_add(OBJECT(s), "fixture-finalized", "bool", NULL, NULL,
                        finalized, NULL);
    ordinary_fixture_peer = &cpu_peer;
    g_assert_true(agfx_negotiate_ordinary_task_memory(s, &err));
    g_assert_null(err);
    if (!strcmp(mode, "absent")) {
        g_assert_null(s->ordinary_task_memory_peer);
        g_assert_cmpint(agfx_install_ordinary_task_memory(s, &err), ==,
                        AGFX_ORDINARY_INSTALLED);
        g_assert_null(s->ordinary_task_memory_context);
        g_assert_null(s->eeee_native_context);
        g_assert_null(s->eeee_provider_state);
        g_assert_cmpuint(ctl.prepare_calls, ==, 0);
    } else {
        g_assert_true(s->ordinary_task_memory_peer == &cpu_peer);
        s->qmu_dev = mock_session();
        s->ordinary_physical_peer = &v2_peer;
        s->ordinary_physical_context = agfx_ordinary_physical_context_create(s);
        enum AgfxOrdinaryInstallOutcome result =
            agfx_install_ordinary_task_memory(s, &err);
        if (!strcmp(mode, "prepare-null") || !strcmp(mode, "reject")) {
            g_assert_cmpint(result, ==, AGFX_ORDINARY_RETURNED_FAILURE);
            g_assert_nonnull(err); error_free(err); err = NULL;
            g_assert_cmpuint(ctl.install_calls, ==, !strcmp(mode, "reject"));
            /* Actual realize-failure tail leaves; caller-order source proof
             * remains separate. Normal mode below executes actual agfx_exit. */
            agfx_eeee_native_context_stop_acquiring(s->ordinary_task_memory_context);
            agfx_ordinary_physical_context_stop(s->ordinary_physical_context);
            agfx_destroy_ordinary_physical(s);
            agfx_return_ordinary_task_memory_after_destroy(s);
            agfx_ordinary_physical_context_retire(s->ordinary_physical_context);
            s->ordinary_physical_context = NULL;
            g_assert_cmpuint(ctl.dispose_calls, ==, !strcmp(mode, "reject"));
        } else if (result == AGFX_ORDINARY_RETAINED) {
            g_assert_null(err);
            AgfxEeeeNativeContext *ctx = s->ordinary_task_memory_context;
            g_assert_true(ctx->acquiring_stopped);
            g_assert_cmpuint(qatomic_read(&ctx->references), ==, 2);
            g_assert_true(s->qmu_dev == mock_session());
            g_assert_cmpuint(ctl.destroy_calls, ==, 0);
            g_assert_cmpuint(ctl.dispose_calls, ==, 0);
            printf("FIXTURE_INSTALL_OWNER_HELD result=%u refs=2 destroy=0 dispose=0\n",
                   s->ordinary_task_memory_install_result);
            fflush(stdout);
            agfx_ordinary_task_memory_retained("fixture_install",
                                              s->ordinary_task_memory_install_result);
        } else {
            g_assert_cmpint(result, ==, AGFX_ORDINARY_INSTALLED);
            start_idle_exit_roots(s);
            agfx_exit(PCI_DEVICE(s)); /* Actual production stop/quiesce/delete tail. */
            g_assert_cmpuint(ctl.quiesce_calls, ==, 1);
            g_assert_cmpuint(ctl.rethrow_calls, ==, 1);
            g_assert_cmpuint(ctl.destroy_calls, ==, 1);
            g_assert_cmpuint(ctl.dispose_calls, ==, 1);
            g_assert_true(ctl.lease_returned);
        }
        g_assert_null(s->ordinary_task_memory_context);
        g_assert_null(s->eeee_native_context);
        g_assert_null(s->eeee_provider_state);
        g_assert_null(s->qmu_dev);
    }
    g_assert_cmpuint(ctl.finalized, ==, 0);
    object_unref(OBJECT(s)); drain_fixture();
    g_assert_cmpuint(ctl.finalized, ==, 1);
    printf("FIXTURE_ORDINARY_TASK_MEMORY_RETURNED mode=%s prepare=%u install=%u destroy=%u dispose=%u QMetalDelete=MOCK GPU=0 VM=0\n",
           mode, ctl.prepare_calls, ctl.install_calls, ctl.destroy_calls, ctl.dispose_calls);
}

int main(int argc, char **argv)
{
    const char *modes[] = { "absent", "prepare-null", "reject", "normal",
                            "escape", "unreturned", "unknown", "dispose-false" };
    bool valid = false;
    g_assert_cmpint(argc, ==, 2);
    const char *mode = argv[1];
    for (size_t i = 0; i < ARRAY_SIZE(modes); ++i) {
        valid |= !strcmp(mode, modes[i]);
    }
    g_assert_true(valid);
    char options[][16] = { "-machine", "none", "-accel", "tcg",
                           "-nodefaults", "-display", "none", "-S" };
    char *qargv[] = { argv[0], options[0], options[1], options[2], options[3],
                     options[4], options[5], options[6], options[7], NULL };
    qemu_init(ARRAY_SIZE(qargv) - 1, qargv);
    run_mode(mode);
    qemu_cleanup(0);
    bql_unlock(); replay_mutex_unlock();
    return 0;
}
