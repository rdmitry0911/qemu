/* Invoke the actual normally compiled QMetal facilities/provider symbols.
 * No copied provider body, controlled callbacks or session factory is used. */
#include "apple-gfx-eeee-qmetal-test-bridge.h"
#include "qmu_eeee_task_facilities_internal.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

using qmu::eeee::LegacyTaskProvider;
using qmu::eeee::ProviderPhysicalRange;
using qmu::eeee::QemuTaskFacilities;

extern "C" int agfx_test_qmetal_legacy_native_window(
    const qmu_extended_callbacks *callbacks, uint64_t physical,
    uint64_t length, uint64_t offset, uint8_t expected, uint8_t written)
{
    constexpr uint64_t reservation = 0x1000000;
    if (!callbacks || !QemuTaskFacilities::complete(*callbacks)) {
        return 1;
    }
    qmu_extended_callbacks copy = *callbacks;
    QemuTaskFacilities facilities(copy);
    copy = {}; // Actual adapter owns its table copy, not the borrowed context.
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
    if (!provider->unmap(offset, length)) {
        return 7;
    }
    for (uint64_t index = 0; index < length; ++index) {
        if (alias[index] != 0 || canonical[index] != written) {
            return 8;
        }
    }
    provider.reset(); // All pins/allocation/task registry leave before context.
    return 0;
}
