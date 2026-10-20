/* Existing headers only: no copied product TU, fake translator or test hook. */
#include "apple-gfx-ml-imported-dirty-bridge.h"
#include "qmu/qmu_vulkan.h"
#include "ml_c_api_internal.h"

static std::weak_ptr<const ml::Buffer::ImportedGuestBacking> original_generation;

extern "C" int agfx_import_layout(qmu_session *session, AgfxImportLayout *out)
{
    auto *s = qmu_vk_get_session(qmu_session_get_vulkan(session));
    if (!s || !s->device) return -1;
    out->sparse = s->device->sparseBufferGranularity();
    VkPhysicalDeviceExternalMemoryHostPropertiesEXT external{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTERNAL_MEMORY_HOST_PROPERTIES_EXT};
    VkPhysicalDeviceProperties2 props{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};
    props.pNext = &external;
    vkGetPhysicalDeviceProperties2(s->device->gpu(), &props);
    out->host_alignment = external.minImportedHostPointerAlignment;
    VkPhysicalDeviceMemoryProperties memory{};
    vkGetPhysicalDeviceMemoryProperties(s->device->gpu(), &memory);
    out->memory_count = memory.memoryTypeCount;
    for (uint32_t i = 0; i < memory.memoryTypeCount; ++i)
        out->memory_flags[i] = memory.memoryTypes[i].propertyFlags;
    return out->sparse && out->host_alignment ? 0 : -1;
}

extern "C" int agfx_import_prepare(qmu_session *session, uint64_t va,
    uint64_t gpa, uint64_t size, AgfxImportIdentity *out)
{
    auto *vk = qmu_session_get_vulkan(session);
    auto *s = qmu_vk_get_session(vk);
    if (!s || s->active_cmd_buffer != VK_NULL_HANDLE ||
        qmu_define_task(session, 41, 0, 0) != QMU_OK ||
        qmu_map_memory(session, 41, va, gpa, size) != QMU_OK) return -1;
    const uint32_t id = qmu_vk_ensure_buffer(vk, 41, va, size,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
        VK_BUFFER_USAGE_TRANSFER_DST_BIT, ML_BUFFER_CLASS_SHARED);
    auto found = s->buffers.find(id);
    if (!id || found == s->buffers.end()) return -1;
    const auto &mb = found->second;
    if (!mb.buffer || !mb.buffer->isImported() || !mb.guest_backed_import ||
        !mb.direct_shared_alias || !mb.read_only_import || mb.import_stale ||
        mb.task_id != 41 || mb.data_va != va || mb.size_bytes != size ||
        mb.buffer_class != ML_BUFFER_CLASS_SHARED || !mb.runtime_generation)
        return -1; /* Copied fallback is a failure, never a SKIP. */
    auto generation = mb.buffer->importedGuestLease();
    if (!generation) return -1;
    original_generation = generation; /* weak, never extends the map lease */
    *out = {id, reinterpret_cast<uintptr_t>(mb.buffer.get()),
        (uint64_t)(uintptr_t)mb.buffer->handle(),
        reinterpret_cast<uintptr_t>(generation.get()), mb.runtime_generation};
    return 0;
}

extern "C" int agfx_import_fill(qmu_session *session, uint32_t id)
{
    auto *vk = qmu_session_get_vulkan(session);
    auto *s = qmu_vk_get_session(vk);
    if (!s || s->active_cmd_buffer != VK_NULL_HANDLE ||
        original_generation.expired()) return -1;
    qmu_vk_fill_buffer(vk, id, 0, 64, 0x12345678);
    return qmu_vk_has_fatal_error(vk) ? -1 : 0;
}

extern "C" int agfx_import_generation_expired(void)
{
    return original_generation.expired();
}
