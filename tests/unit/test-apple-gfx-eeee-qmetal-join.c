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

#define TYPE_JOIN_RAM_OWNER "test-eeee-qmetal-join-ram-owner"
#define JOIN_RAM_GPA UINT64_C(0x10000000)
#define JOIN_ALIAS_GPA UINT64_C(0x20000000)

typedef struct JoinRamOwner {
    Object parent_obj;
    MemoryRegion ram;
    MemoryRegion alias;
} JoinRamOwner;

static const TypeInfo join_owner_type = {
    .name = TYPE_JOIN_RAM_OWNER,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(JoinRamOwner),
};

int (*qemu_main)(void);

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
    status = g_test_run();
    qemu_cleanup(status);
    bql_unlock();
    replay_mutex_unlock();
    return status;
}
