/* Link this companion ONLY for present-peer fixture. Excluding it leaves the
 * actual production weak getter genuinely unresolved, with the same test TU. */
#include "qemu/osdep.h"
#include "hw/display/qmu_ordinary_task_memory_v1.h"
extern const struct qmu_private_ordinary_task_memory_v1 *ordinary_fixture_peer;
const struct qmu_private_ordinary_task_memory_v1 *
qmu_get_private_ordinary_task_memory_v1(void)
{
    return ordinary_fixture_peer;
}
