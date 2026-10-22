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

/* Append to the existing normally linked bridge; no product implementation TU. */
#include "qmu_session_work_internal.h"
#include "qmu_eeee_map_first_leaf_observation_internal.h"
#include "qmu/qmu_fifo.h"
#include <array>
#include <cassert>
#include <cerrno>
#include <exception>
#include <sys/stat.h>
#include <unistd.h>

/* Already exported by current BUILD_TESTS mlapi; no new product compile flag. */
qmu::p0::ResourceTerminal *qmu_session_test_p0_terminal(qmu_session *);

struct AgfxTestMapMMIO {
    qmu_session *session;
    qmu::SessionWork work;
    uint32_t offset;
    uint64_t value, result{};
    bool read, returned{};
    std::exception_ptr escape;
    AgfxTestMapMMIO(qmu_session *s, uint32_t o, uint64_t v, bool r)
        : session(s), work(qmu::SessionWork::for_call(s)), offset(o), value(v), read(r) {}
};

extern "C" AgfxTestMapMMIO *agfx_test_map_mmio_new(
    qmu_session *s, uint32_t offset, uint64_t value, int read)
{
    try {
        auto call = std::make_unique<AgfxTestMapMMIO>(s, offset, value, read != 0);
        if (!call->work) return nullptr;
        return call.release(); // Original Work retained BEFORE C worker publication.
    } catch (...) { return nullptr; } // Host fixture refusal, not source status.
}
extern "C" qmu_session_work *agfx_test_map_mmio_work(AgfxTestMapMMIO *call)
{ return call->work.get(); }
extern "C" void agfx_test_map_mmio_worker(AgfxTestMapMMIO *call)
{
    try {
        qmu::SessionWorkScope scope(call->work);
        assert(qmu_session_current_host_work(call->session) == call->work.get());
        if (call->read) call->result = qmu_mmio_read(call->session, call->offset, 4);
        else qmu_mmio_write(call->session, call->offset, call->value, 4);
        call->returned = true;
    } catch (...) {
        call->escape = std::current_exception(); // No C frame is unwound.
    }
}
extern "C" int agfx_test_map_mmio_finish(AgfxTestMapMMIO *call, uint64_t *result)
{
    if (!call || !call->returned || call->escape) return 0;
    *result = call->result;
    delete call; // Only after worker JOIN and its real callback RETURNs.
    return 1;
}

/* Read the exact matched private Record type, not a separately invented ABI. */
extern "C" int agfx_test_map_task_snapshot(qmu_session *session, AgfxTestMapTask *out)
{
    try {
        const auto *terminal = qmu_session_test_p0_terminal(session);
        if (!terminal) return -1;
        const auto actual = terminal->task_snapshot(1);
        if (!actual.present || !actual.task.valid()) return -1;
        *out = {actual.task.epoch, actual.address_space_size, actual.heap_binding_epoch,
                actual.root_pfn, static_cast<uint32_t>(actual.kind)};
        return 0;
    } catch (...) { return -1; }
}
extern "C" int agfx_test_map_verify_records(
    int fd, const AgfxTestMapRead *reads, size_t count, const AgfxTestMapTask *tasks)
{
    namespace obs = qmu::eeee::map_first_leaf_observation;
#define MAP_RECORD_REQUIRE(x) do { if (!(x)) return __LINE__; } while (0)
    try {
        std::array<obs::Record, 3> records;
        struct stat st{};
        MAP_RECORD_REQUIRE(::fstat(fd, &st) == 0 &&
                           st.st_size == static_cast<off_t>(sizeof(records)));
        size_t have = 0;
        auto *bytes = reinterpret_cast<uint8_t *>(records.data());
        while (have != sizeof(records)) {
            const ssize_t n = ::pread(fd, bytes + have, sizeof(records) - have,
                                      static_cast<off_t>(have));
            if (n < 0 && errno == EINTR) continue;
            MAP_RECORD_REQUIRE(n > 0);
            have += static_cast<size_t>(n);
        }
        size_t cursor = 0;
        auto actual_return = [&](unsigned vector, uint64_t gpa, uint32_t size,
                                 int result, const uint8_t *data) {
            for (; cursor < count; ++cursor) {
                const auto &row = reads[cursor];
                if (row.vector != vector || row.gpa != gpa || row.bytes != size) continue;
                if (!row.entered || !row.returned || row.result != result) return false;
                if (!result && (row.prefix_bytes != size || std::memcmp(row.prefix, data, size)))
                    return false;
                ++cursor;
                return true;
            }
            return false;
        };
        for (unsigned v = 0; v != records.size(); ++v) {
            const auto &r = records[v];
            MAP_RECORD_REQUIRE(r.magic == obs::kMagic && r.version == obs::kVersion &&
                               r.record_bytes == sizeof(r));
            MAP_RECORD_REQUIRE(r.channel == 2 && r.opcode == QMU_GPU_MAP_MEMORY2 &&
                               r.stamp == 46 + v && r.command_offset == 32 * v);
            MAP_RECORD_REQUIRE(r.terminal_id && r.terminal_channel == 2 &&
                               r.terminal_stamp == r.stamp &&
                               r.terminal_command_offset == r.command_offset);
            MAP_RECORD_REQUIRE(r.clock_returned && !r.clock_result && r.monotonic_ns);
            MAP_RECORD_REQUIRE(r.declared_payload_bytes == 20 && r.captured_prefix_bytes == 20 &&
                               !r.trailing_unrecorded_bytes);
            MAP_RECORD_REQUIRE(r.requested_task == 1 && r.requested_offset == 0x2000 &&
                               r.requested_length == 0x1000);
            MAP_RECORD_REQUIRE(r.selected_present && r.selected_task == 1 && r.selected_epoch &&
                               r.address_space_bytes == 0x400000 && !r.binding_epoch);
            MAP_RECORD_REQUIRE(r.selected_epoch == tasks[v].epoch &&
                               r.root_header_pfn == tasks[v].root_pfn &&
                               r.original_task_kind == tasks[v].kind &&
                               r.binding_epoch == tasks[v].binding_epoch &&
                               r.address_space_bytes == tasks[v].address_space);
            MAP_RECORD_REQUIRE(r.root_header.entered && r.root_header.returned &&
                               r.root_header.bytes == 8 && !r.observer_exception &&
                               r.root_header.gpa == (uint64_t(r.root_header_pfn) << 12));
            MAP_RECORD_REQUIRE(actual_return(v, r.root_header.gpa, 8,
                               r.root_header.result, r.root_header.data.data()));
            if (v != 2) {
                MAP_RECORD_REQUIRE(r.root_header_pfn == 0x445 && r.pt_root_pfn == 0x470 &&
                                   r.depth == 1 && !r.root_header.result);
                MAP_RECORD_REQUIRE(r.status == obs::Status::first_visitor &&
                                   r.private_sentinel_caught && r.table_read_count == 1);
                const auto &t = r.table_reads[0];
                MAP_RECORD_REQUIRE(t.entered && t.returned && !t.result && t.ordinal == 1 &&
                                   t.bytes == 4 && t.gpa == 0x470008);
                MAP_RECORD_REQUIRE(actual_return(v, t.gpa, t.bytes, t.result, t.data.data()));
                const uint32_t expected_pfn = v ? 1 : 0x472;
                MAP_RECORD_REQUIRE(r.first_virtual_page == 0x2000 && r.first_pfn == expected_pfn &&
                                   r.first_writable == 1 && r.raw_leaf == (0x80000000U | expected_pfn));
                MAP_RECORD_REQUIRE(r.raw_leaf == obs::u32(t.data.data()));
            } else {
                MAP_RECORD_REQUIRE(r.root_header_pfn == 0xdeadbeef && r.root_header.result != 0 &&
                                   r.status == obs::Status::read_incomplete &&
                                   !r.table_read_count && !r.private_sentinel_caught);
            }
        }
        MAP_RECORD_REQUIRE(records[0].selected_epoch == records[1].selected_epoch &&
                           records[2].selected_epoch > records[1].selected_epoch);
        MAP_RECORD_REQUIRE(records[0].sequence == 1 && records[1].sequence == 2 &&
                           records[2].sequence == 1); // Actual DELETE/DEFINE; per-key limit.
        MAP_RECORD_REQUIRE(records[0].terminal_id != records[1].terminal_id &&
                           records[1].terminal_id != records[2].terminal_id &&
                           records[0].terminal_id != records[2].terminal_id);
        return 0;
    } catch (...) { return __LINE__; } // Host oracle failure, never a guest result.
#undef MAP_RECORD_REQUIRE
}
