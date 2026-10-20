/* Actual typed QEMU capture -> compiled QMetal storage -> owning native view.
 * No shader submission/read or claim about original-alias cache visibility. */
#include "apple-gfx-efi-capture-test-bridge.h"
#include "unified/qmu_efi_source_storage_internal.h"
#include "unified/qmu_buffer2d_native_view.h"
#include "ml/Device.h"
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fcntl.h>
#include <memory>
#include <variant>

static void efi_require(bool condition, const char *reason)
{
    if (!condition) {
        std::fprintf(stderr, "EFI_TYPED_JOIN_FAIL: %s\n", reason);
        std::abort();
    }
}

extern "C" void agfx_test_efi_storage_run(
    const qmu_extended_callbacks *facilities, qmu_efi_capture_source_v2 capture,
    uint64_t task_va, uint64_t literal_length,
    const qmu_efi_capture_result_v2 *observed,
    void (*after_bound)(void *), void *after_bound_context)
{
    efi_require(facilities && capture && observed && after_bound,
                "missing actual source facilities");
    try {
        const auto device = std::make_shared<ml::Device>();
        efi_require(device->hasDmaBufImport(), "actual Device lacks DMA import");
        auto storage = qmu::EfiSourceStorage::prepare(device, *facilities, capture);
        efi_require(bool(storage), "actual storage preparation rejected");
        efi_require(storage->initialize(QMU_EFI_SOURCE_DMA_BUF, task_va,
            literal_length, 4, 2, 16), "actual typed construction rejected");
        auto view = storage->view();
        efi_require(view && view->native.is_dma(), "missing actual DMA view");
        const auto& binding = std::get<qmu::Buffer2DNativeStorage::Dma>(view->native.origin);
        const int original_fd = binding.original_fd;
        efi_require(binding.device == device && binding.parent && binding.backing && binding.memory,
                    "actual creating Device/parent/backing/memory missing");
        efi_require(observed->source_kind == QMU_EFI_SOURCE_DMA_BUF &&
            observed->payload.dma_buf.original_fd == original_fd &&
            observed->opaque_owner == binding.source_owner &&
            observed->allocation_bytes == binding.export_bytes &&
            observed->visible_offset == binding.visible_offset &&
            observed->visible_bytes == binding.visible_bytes &&
            binding.visible_bytes == literal_length &&
            binding.buffer_bytes == binding.visible_offset + literal_length &&
            binding.requirements_bytes >= binding.buffer_bytes &&
            binding.export_bytes >= binding.requirements_bytes &&
            binding.memory_bytes == binding.requirements_bytes,
            "typed source ownership or A/V/L/B/R/S changed");
        efi_require(view->descriptor.buffer == binding.parent->handle() &&
            view->descriptor.offset == 0 && view->descriptor.range == binding.buffer_bytes &&
            view->request.offset == binding.visible_offset &&
            view->request.width == 4 && view->request.height == 2 && view->request.row_bytes == 16,
            "view did not retain exact original buffer/range/geometry");
        efi_require(fcntl(original_fd, F_GETFD) >= 0, "original fd consumed");
        std::printf("EFI_TYPED_BOUND A=%" PRIu64 " V=%" PRIu64 " L=%" PRIu64
            " B=%" PRIu64 " R=%" PRIu64 " S=%" PRIu64 "\n",
            uint64_t(binding.export_bytes), uint64_t(binding.visible_offset),
            uint64_t(binding.visible_bytes), uint64_t(binding.buffer_bytes),
            uint64_t(binding.requirements_bytes), uint64_t(binding.memory_bytes));
        after_bound(after_bound_context);
        efi_require(!storage->retire(), "retired with actual view lease");
        efi_require(!storage->view(), "retirement still publishes a new view");
        auto copied_view = view;
        auto parent = binding.parent;
        auto backing = binding.backing;
        view.reset();
        efi_require(!storage->retire(), "retired with copied view lease");
        copied_view.reset();
        efi_require(!storage->retire(), "retired with retained parent/backing");
        parent.reset();
        efi_require(!storage->retire(), "retired with physical backing lease");
        backing.reset();
        efi_require(storage->retire(), "actual native/source retirement rejected");
        efi_require(storage->retire(), "completed retirement not idempotent");
        errno = 0;
        efi_require(fcntl(original_fd, F_GETFD) == -1 && errno == EBADF,
                    "actual original export fd not released");
        std::puts("EFI_TYPED_RETIRED native-free/source-RETURN no-GPU-submit");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "EFI_TYPED_JOIN_EXCEPTION: %s\n", error.what());
        std::abort();
    }
}
