/* Actual published packet/API types; no copied implementation or fake VA read. */
#define QMU_TEST_ACCESS 1
#include "apple-gfx-ml-connected-gamma-test-bridge.h"
#include "pvg/pvg_fifo.h"
#include "qmu/qmu_vulkan.h"

#include <cstddef>
#include <cstring>

static_assert(sizeof(PVGRootCmdHeader) == 12);
static_assert(offsetof(PVGRootCmdHeader, total_size) == 4);
static_assert(offsetof(PVGRootCmdHeader, stamp_value) == 8);
static_assert(sizeof(PVGRootCmdDefineTask) == 16);
static_assert(offsetof(PVGRootCmdDefineTask, task_size) == 4);
static_assert(offsetof(PVGRootCmdDefineTask, page_table_pfn) == 12);

extern "C" uint32_t agfx_connected_gamma_define_task(void *ring,
    uint32_t root_pfn)
{
    PVGRootCmdHeader header{};
    PVGRootCmdDefineTask task{};
    header.opcode = PVG_ROOT_CMD_DEFINE_TASK;
    header.total_size = sizeof(header) + sizeof(task);
    header.stamp_value = 1;
    task.task_id_encoded = 1; /* Actual task0/kernel encoding. */
    task.task_size = 4 * 1024 * 1024;
    task.page_table_pfn = root_pfn;
    std::memcpy(ring, &header, sizeof(header));
    std::memcpy(static_cast<unsigned char *>(ring) + sizeof(header),
        &task, sizeof(task));
    return header.total_size;
}

extern "C" int agfx_connected_gamma_task_state(qmu_session *session,
    int present, uint32_t root_pfn, uint64_t address_space_size, uint64_t *epoch)
{
    qmu_test_p0_task_snapshot snapshot{};
    if (qmu_test_get_p0_task_snapshot(session, 0, &snapshot) != QMU_OK ||
        snapshot.present != present) {
        return -1;
    }
    *epoch = snapshot.task_epoch;
    if (!present) {
        return 0;
    }
    /* The actual read-only public snapshot carries raw task_kind. Source:
     * qmu_p0_resource_terminal_internal.h TaskKind::local_kernel is zero.
     * Keep this bridge on published headers; do not copy a private layout. */
    return snapshot.task_id == 0 && snapshot.task_kind == 0 &&
        snapshot.task_epoch != 0 && snapshot.root_pfn == root_pfn &&
        snapshot.address_space_size == address_space_size ? 0 : -1;
}

extern "C" int agfx_connected_gamma_update(qmu_session *session,
    uint64_t table_va, uint64_t mapped_length, uint32_t entries, uint32_t sum)
{
    return qmu_vk_update_gamma_lut(qmu_session_get_vulkan(session), table_va,
        mapped_length, entries, sum);
}
