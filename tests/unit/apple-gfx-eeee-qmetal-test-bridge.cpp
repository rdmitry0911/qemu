/* Invoke actual normally compiled QMetal facilities/provider symbols.
 * Test-only glue: no copied provider, controlled context or Session/GPU. */
#include "apple-gfx-eeee-qmetal-test-bridge.h"
#include "qmu_eeee_task_facilities_internal.h"
#include "qmu_eeee_owned_task_facilities_internal.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

using qmu::eeee::LegacyTaskFacilities;
using qmu::eeee::LegacyTaskOwner;
using qmu::eeee::LegacyTaskProvider;
using qmu::eeee::OwnedQemuTaskFacilities;
using qmu::eeee::OwnedTaskMemoryCallbacks;
using qmu::eeee::ProviderPhysicalRange;
using qmu::eeee::QemuTaskContextLease;
using qmu::eeee::QemuTaskFacilities;

/* Both wrappers share the original byte/MR/native CREATE/MAP/UNMAP oracle.
 * Only the new retirement case leaves its provider alive for worker release. */
static int legacy_native_window(
    LegacyTaskFacilities &facilities, uint64_t physical, uint64_t length,
    uint64_t offset, uint8_t expected, uint8_t written,
    LegacyTaskOwner *retained = nullptr, void **actual_region = nullptr,
    uint64_t *actual_region_offset = nullptr)
{
    constexpr uint64_t reservation = 0x1000000;
    const uint64_t page = facilities.host_page_size();
    if (!page || !length || offset >= reservation ||
        length > reservation - offset || physical % page || offset % page ||
        length % page || expected == written) {
        return 2;
    }
    auto provider = LegacyTaskProvider::create(facilities, reservation);
    if (!provider) {
        return 3;
    }
    const ProviderPhysicalRange range{physical, length};
    if (!provider->map(offset, &range, 1, false)) {
        return 4;
    }
    auto *alias = reinterpret_cast<uint8_t *>(provider->base_address() + offset);
    for (uint64_t index = 0; index < length; ++index) {
        if (alias[index] != expected) {
            return 5;
        }
    }

    // Acquire a separate actual canonical translation to prove shared byte
    // identity in both directions. This is a borrowed view, not a new pin.
    facilities.rcu_enter();
    facilities.task_lock();
    const auto translated = facilities.translate(physical, length, true);
    uint8_t *canonical = nullptr;
    if (translated.region && translated.length >= length &&
        facilities.direct(translated.region, true)) {
        auto *ram = static_cast<uint8_t *>(facilities.ram_pointer(translated.region));
        if (ram) {
            canonical = ram + translated.region_offset;
        }
    }
    if (actual_region) {
        *actual_region = translated.region;
        *actual_region_offset = translated.region_offset;
    }
    bool shared = canonical && canonical != alias;
    if (shared) {
        std::memset(alias, written, static_cast<size_t>(length));
        shared = std::memcmp(alias, canonical, static_cast<size_t>(length)) == 0;
        canonical[0] = expected;
        shared = shared && alias[0] == expected;
        canonical[0] = written;
        shared = shared && alias[0] == written;
    }
    facilities.task_unlock();
    facilities.rcu_leave();
    if (!shared) {
        return 6;
    }
    // This method internally takes the producer mutex; not under task_lock.
    provider->notify_host_write(offset, length);
    if (retained) {
        *retained = std::move(provider);
        return 0;
    }
    if (!provider->unmap(offset, length)) {
        return 7;
    }
    for (uint64_t index = 0; index < length; ++index) {
        if (alias[index] != 0 || canonical[index] != written) {
            return 8;
        }
    }
    provider.reset(); // Actual pins/allocation/task registry leave first.
    return 0;
}

extern "C" int agfx_test_qmetal_legacy_native_window(
    const qmu_extended_callbacks *callbacks, uint64_t physical,
    uint64_t length, uint64_t offset, uint8_t expected, uint8_t written)
{
    if (!callbacks || !QemuTaskFacilities::complete(*callbacks)) {
        return 1;
    }
    qmu_extended_callbacks copy = *callbacks;
    QemuTaskFacilities facilities(copy);
    copy = {};
    return legacy_native_window(facilities, physical, length, offset,
                                expected, written);
}

struct AgfxTestOwnedContext {
    /* Reverse destruction: provider/MR RETURN, descendant, original owner. */
    std::shared_ptr<OwnedQemuTaskFacilities> parent;
    OwnedTaskMemoryCallbacks descendant;
    LegacyTaskOwner provider;
};

extern "C" AgfxTestOwnedContext *agfx_test_owned_context_adopt(
    const qmu_extended_callbacks *callbacks, void *context,
    void (*release)(void *))
{
    if (!callbacks || !context || !release) {
        return nullptr;
    }
    auto owner = OwnedQemuTaskFacilities::adopt(
        QemuTaskContextLease(context, *release, *callbacks));
    if (!owner) {
        return nullptr;
    }
    auto result = std::make_unique<AgfxTestOwnedContext>();
    result->parent = owner;
    result->descendant = qmu::eeee::owned_task_memory_callbacks(owner);
    const auto &memory = result->descendant.callbacks;
    if (result->descendant.owner != owner ||
        memory.context != callbacks->user_ctx ||
        memory.map_gpa != callbacks->map_gpa ||
        memory.unmap_gpa != callbacks->unmap_gpa ||
        memory.read_memory != callbacks->read_memory ||
        memory.mark_mapped_memory_dirty != callbacks->mark_mapped_memory_dirty) {
        return nullptr;
    }
    return result.release();
}

extern "C" int agfx_test_owned_context_drop_parent(AgfxTestOwnedContext *owner)
{
    if (!owner || !owner->parent ||
        owner->parent != owner->descendant.owner) {
        return 1;
    }
    owner->parent.reset();
    return owner->descendant.owner.use_count() == 1 ? 0 : 2;
}

extern "C" int agfx_test_owned_context_native_window(
    AgfxTestOwnedContext *owner, uint64_t physical, uint64_t length,
    uint64_t offset, uint8_t expected, uint8_t written, int keep_alias,
    void **actual_region, uint64_t *actual_region_offset)
{
    if (!owner || owner->parent || !owner->descendant.owner ||
        owner->provider || !actual_region || !actual_region_offset) {
        return 1;
    }
    auto &facilities = owner->descendant.owner->facilities();
    const int result = legacy_native_window(facilities, physical, length,
        offset, expected, written, keep_alias ? &owner->provider : nullptr,
        actual_region, actual_region_offset);
    if (result) {
        return result;
    }
    /* The SAME descendant also carries actual common map/unmap callbacks.
     * read/dirty callbacks require SessionWork and are intentionally not
     * invoked by this context-only, unrealized QOM gate. */
    auto &memory = owner->descendant.callbacks;
    auto *mapped = static_cast<uint8_t *>(memory.map_gpa(
        memory.context, physical, static_cast<size_t>(length), 1));
    if (!mapped) {
        return 9;
    }
    bool exact = true;
    for (uint64_t index = 0; index < length; ++index) {
        exact = exact && mapped[index] == written;
    }
    memory.unmap_gpa(memory.context, mapped, static_cast<size_t>(length), 1);
    return exact ? 0 : 10;
}

extern "C" void agfx_test_owned_context_destroy(AgfxTestOwnedContext *owner)
{
    delete owner;
}
