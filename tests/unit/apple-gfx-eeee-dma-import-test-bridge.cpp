/* Native construction/custody only. No copied exporter, shader, CPU-alias
 * visibility assumption or production compatibility path. */
#include "apple-gfx-eeee-dma-import-test-bridge.h"
#include "ml_dma_buf_import_internal.h"
#include "ml/Device.h"
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fcntl.h>
#include <memory>

struct AgfxNativeDmaOwner {
    std::shared_ptr<const ml::Device> device;
    std::shared_ptr<ml::DmaBufImport> import;
    ml::DmaBufImport::Requirements geometry;
};
static void require(bool condition, const char *reason)
{
    if (!condition) {
        std::fprintf(stderr, "NATIVE_DMA_JOIN_FAIL: %s\n", reason);
        std::abort();
    }
}
extern "C" AgfxNativeDmaOwner *agfx_test_dma_prepare(uint64_t V, uint64_t L,
    AgfxNativeDmaRequirements *out)
{
    require(out != nullptr, "missing query output");
    *out = {};
    try {
        auto owner = std::make_unique<AgfxNativeDmaOwner>();
        owner->device = std::make_shared<ml::Device>();
        require(owner->device->hasDmaBufImport(), "actual Device lacks jointly enabled DMA import");
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(owner->device->gpu(), &properties);
        std::printf("NATIVE_DMA_DEVICE vendor=0x%04x device=0x%04x api=%u driver=%u name=%s\n",
            properties.vendorID, properties.deviceID, properties.apiVersion,
            properties.driverVersion, properties.deviceName);
        owner->import = ml::DmaBufImport::prepare(owner->device);
        require(bool(owner->import), "CPU preparation failed");
        require(owner->import->requirements(V, L, &owner->geometry), "actual external buffer requirements rejected");
        const auto &g = owner->geometry;
        require(g.visible_offset == V && g.visible_bytes == L && g.buffer_bytes == V+L,
            "original visible range changed");
        *out = {g.buffer_bytes, g.allocation_bytes, g.alignment,
                g.memory_type_bits, g.dedicated};
        return owner.release();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "NATIVE_DMA_JOIN_EXCEPTION prepare: %s\n", error.what());
        std::abort();
    }
}
extern "C" void agfx_test_dma_import_retire(AgfxNativeDmaOwner *raw, int fd,
    uint64_t A, uint64_t V, uint64_t L, void *source, void (*release)(void *))
{
    require(raw && source && release && fd >= 0, "invalid original owner relation");
    try {
        std::unique_ptr<AgfxNativeDmaOwner> owner(raw);
        const auto g = owner->geometry;
        require(owner->import->initialize({fd, A, V, L, source, release}), "actual DMA import/bind rejected");
        ml::DmaBufImport::Binding binding;
        require(owner->import->binding(&binding), "missing native binding");
        require(binding.device == owner->device && binding.parent && binding.backing && binding.memory,
            "actual Device/parent/backing/memory relation missing");
        require(binding.original_fd == fd && binding.source_owner == source &&
            binding.export_bytes == A && binding.visible_offset == V && binding.visible_bytes == L &&
            binding.buffer_bytes == V+L && binding.requirements_bytes == g.allocation_bytes &&
            binding.memory_bytes == g.allocation_bytes && binding.dedicated == g.dedicated,
            "A/V/L/B/R/S or same-buffer dedicated contract changed");
        require(fcntl(fd, F_GETFD) >= 0, "original export FD consumed instead of duplicate");
        std::printf("NATIVE_DMA_BOUND A=%" PRIu64 " V=%" PRIu64 " L=%" PRIu64
            " B=%" PRIu64 " R=%" PRIu64 " S=%" PRIu64 " type=%u flags=0x%x dedicated=%u\n",
            A, V, L, uint64_t(binding.buffer_bytes), uint64_t(binding.requirements_bytes),
            uint64_t(binding.memory_bytes), binding.memory_type_index, binding.memory_flags,
            unsigned(binding.dedicated));
        require(!owner->import->retire(), "retired with active binding leases");
        auto parent = binding.parent;
        auto backing = binding.backing;
        binding = {};
        require(!owner->import->retire(), "retired with retained parent/backing");
        parent.reset();
        require(!owner->import->retire(), "retired with physical backing lease");
        backing.reset();
        require(owner->import->retire(), "actual native/source retirement failed");
        require(owner->import->retire(), "completed retirement is not idempotent");
        errno = 0;
        require(fcntl(fd, F_GETFD) == -1 && errno == EBADF, "source release did not close original export FD");
        std::puts("NATIVE_DMA_RETIRED native-destroy-free-source-RETURN no-GPU-submit");
    } catch (const std::exception &error) {
        std::fprintf(stderr, "NATIVE_DMA_JOIN_EXCEPTION import/retire: %s\n", error.what());
        std::abort();
    }
}
