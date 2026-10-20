/* Calls actual normally linked QMetal; no private state or copied TU.
 * QMU_TEST_ACCESS exposes only the already-existing read-only epoch getter
 * of the matched normal BUILD_TESTS DSO; no backend behavior switch is added.
 */
#define QMU_TEST_ACCESS 1
#include "apple-gfx-ml-credit-test-bridge.h"
#include "qmu/qmu_vulkan.h"

#include <cstdint>
#include <vector>

extern "C" uint32_t agfx_credit_prepare_texture(
    qmu_session *session, uint32_t width, uint32_t height)
{
    auto *vk = qmu_session_get_vulkan(session);
    if (!vk || qmu_vk_create_swapchain(vk, width, height, 2) != 1) {
        return 0;
    }
    // Public RT creation has data_va/data_size/task_id0: no fabricated guest
    // upload or unconfigured task/PT mapping is claimed. Pixel content of this
    // native image is not a visual witness; this case tests its record family.
    return qmu_vk_create_render_target(vk, width, height, 44);
}

extern "C" int agfx_credit_publish(
    qmu_session *session, enum AgfxCreditFamily family, uint32_t texture,
    uint32_t width, uint32_t height, uint32_t seed)
{
    auto *vk = qmu_session_get_vulkan(session);
    if (!vk) {
        return -1;
    }
    if (family == AGFX_CREDIT_IOSFC) {
        std::vector<uint32_t> pixels(static_cast<std::size_t>(width) * height, seed);
        return qmu_vk_iosfc_present_frame(vk, pixels.data(), width, height,
                                           width * 4);
    }
    if (!texture) {
        return -1;
    }
    // This documented generation-less helper is a synthetic source selector,
    // not a Type4/Transaction3/guest resource admission. All objects and the
    // capture/submit/terminal below are real normal-product objects/calls.
    qmu_vk_set_display_surface_binding(vk, 7, 0x7111, 0, 0x42475241,
                                         0, 0, width, height);
    qmu_vk_set_display_surface_texture_id(vk, texture);
    if (qmu_vk_set_display_backing_va(vk, 0x710000) != 0) {
        return -1;
    }
    qmu_vk_mark_pending_display_frame_renderable(vk);
    qmu_vk_signal_current_frame(vk);
    return 0;
}

extern "C" uint64_t agfx_credit_delivery_epoch(qmu_session *session)
{
    return qmu_vk_test_get_display_delivery_epoch(qmu_session_get_vulkan(session));
}
