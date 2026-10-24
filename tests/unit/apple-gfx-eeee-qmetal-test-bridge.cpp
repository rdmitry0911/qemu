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

/* Append after the existing real AgfxTestOwnedContext definition/functions.
 * No product TU is copied. All constructor/root/retirement calls below are
 * the candidate's normally compiled production implementation. */
#include "qmu_eeee_pending_native_local_task_internal.h"
#include "qmu_session_p0_internal.h"
#include <exception>
#include <optional>
#include <stdexcept>

namespace {
void pending_require(bool condition, const char *message)
{
    if (!condition) throw std::logic_error(message); // fixture oracle only
}
uint64_t pending_load64(const void *source)
{
    const auto *p=static_cast<const unsigned char *>(source);
    uint64_t value=0;
    for (unsigned n=0;n<8;++n) value|=uint64_t(p[n])<<(8*n);
    return value;
}
}
struct __attribute__((visibility("hidden"))) AgfxTestPendingParentCall {
    std::shared_ptr<OwnedQemuTaskFacilities> facilities;
    qmu_config config{};
    qmu_session *session{};
    qmu::eeee::TaskCreateRequest request;
    void *expected_root;
    uint32_t expected_page_table_root;
    std::shared_ptr<qmu::eeee::SourceStandardMtlReceiver> receiver;
    qmu::SessionWork transferred_work;
    std::optional<qmu_status> quiesce;
    std::exception_ptr worker_exception,main_exception;
    bool worker_returned{},session_creation_returned{},parent_completed{},destroy_returned{};
    AgfxTestPendingParentCall(std::shared_ptr<OwnedQemuTaskFacilities> source,
        uint32_t encoded,uint64_t length,uint32_t root,void *host,uint32_t pt)
        : facilities(std::move(source)),request{encoded,length,root,false},
          expected_root(host),expected_page_table_root(pt)
    { config.display_width=config.display_height=4; }
};
extern "C" AgfxTestPendingParentCall *agfx_test_pending_parent_new(
    AgfxTestOwnedContext *actual,uint32_t encoded,uint64_t length,uint32_t root,
    void *host,uint32_t pt)
{
    try {
        if (!actual || !actual->descendant.owner || !length || !root || !host || !pt)
            return nullptr; // test input refusal, never a source task outcome
        return new AgfxTestPendingParentCall(actual->descendant.owner,encoded,length,root,host,pt);
    } catch (...) { return nullptr; } // before any new Session/provider entry
}
extern "C" void agfx_test_pending_parent_worker_cpp(
    AgfxTestPendingParentCall *call,AgfxTestPendingParentAction action)
{
    // This entire extern-C entry catches C++ objects before returning through C.
    try {
        pending_require(call && !call->worker_exception && !call->main_exception,"caller custody");
        switch (action) {
        case AGFX_PENDING_CREATE_SESSION: {
            pending_require(!call->session && !call->session_creation_returned,"create once");
            const auto memory=qmu::eeee::owned_task_memory_callbacks(call->facilities);
            qmu_callbacks callbacks{};
            callbacks.user_ctx=memory.callbacks.context;
            callbacks.map_gpa=memory.callbacks.map_gpa;
            callbacks.unmap_gpa=memory.callbacks.unmap_gpa;
            callbacks.read_memory=memory.callbacks.read_memory;
            callbacks.mark_mapped_memory_dirty=memory.callbacks.mark_mapped_memory_dirty;
            call->session=qmu_create(&call->config,&callbacks);
            call->session_creation_returned=true;
            pending_require(call->session!=nullptr,"actual Session creation returned NULL");
            break;
        }
        case AGFX_PENDING_RUN_PARENT_PHASE: {
            using namespace qmu::eeee;
            pending_require(call->session && call->transferred_work && !call->parent_completed,
                "genuine invocation Work");
            qmu::SessionWorkScope invocation(call->transferred_work);
            call->receiver=make_standard_mtl_receiver(call->session,
                SourceStandardMtlQueryProfile::inherited_standard_feature_queries);
            pending_require(bool(call->receiver),"actual standard source receiver");
            auto reservation=qmu_session_p0_reserve_native_local_construction(
                call->session,call->request.encoded_task_id>>1);
            pending_require(reservation && reservation->owned_by_current_thread() &&
                qmu_session_p0_owns_native_local_construction(call->session,*reservation),
                "production same-thread canonical reservation");
            const auto generated_key=reservation->key(); // observed, never manufactured
            pending_require(generated_key.valid() &&
                !reservation->terminal().current_task(generated_key.task_id).valid(),
                "pending is not numerically published");
            auto record=std::make_shared<SourceNativeLocalTaskRecord>(*reservation,call->request,
                call->facilities,call->receiver,SourceNativeParentAllocation::plain_cpp_phase_only);
            std::weak_ptr<SourceNativeLocalTaskRecord> old_record=record;
            auto pending=PendingNativeLocalTask::adopt(call->session,std::move(reservation),
                std::move(record)); // sole actual record owner moves into source packet
            auto *actual=pending->record().get(); // borrow only; retire requires sole owner
            pending_require(actual && actual->key()==generated_key &&
                pending->phase()==PendingNativeLocalPhase::native_memory_ready &&
                actual->facilities()==call->facilities && actual->receiver()==call->receiver,
                "actual native CREATE/facilities/receiver phase");
            const auto *list=actual->native_legacy_list().get();
            std::weak_ptr<SourceNativeLegacyObjectList> old_list=actual->native_legacy_list();
            pending_require(list && list->borrowed_task()==actual &&
                list->task_length()==call->request.address_space_length &&
                !list->placement_supported() && actual->provider() &&
                actual->provider()->length()==call->request.address_space_length &&
                actual->provider()->base_address()!=0,"real nonnull parent/list/provider");
            {   // Actual mutex objects; all guards leave before native destruction.
                std::unique_lock<std::shared_mutex> operation(actual->operation_mutex());
                std::lock_guard<std::recursive_mutex> recursive(actual->recursive_mutex());
                std::lock_guard<std::mutex> prepare(actual->prepare_mutex());
                pending_require(actual->key()==generated_key,"same pending owner under its locks");
            }
            const SourceAuxiliaryAllocationService allocation{
                SourceAuxiliaryFailureProfile::core_foundation_throw};
            pending->construct_manager(false,allocation); // real anchored production wrapper
            auto *manager=actual->manager();
            pending_require(manager && manager->identity()==generated_key &&
                manager->borrowed_record()==actual && manager->borrowed_terminal()==&actual->terminal() &&
                manager->borrowed_source_mtl_receiver()==call->receiver.get() &&
                manager->supplied_native_legacy_list().get()==list && !manager->supplied_list(),
                "same source manager/native list, no model list");
            pending_require(pending->setup_task_root() &&
                pending->phase()==PendingNativeLocalPhase::root_ready &&
                actual->root_pointer()==call->expected_root &&
                actual->page_table_root_pfn()==call->expected_page_table_root &&
                actual->page_table_depth()==1 && actual->iterator() &&
                pending_load64(static_cast<unsigned char *>(actual->root_pointer())+0x14)==
                    call->request.address_space_length,"actual source root callback and metadata");
            pending_require(!actual->terminal().current_task(generated_key.task_id).valid(),
                "root-ready component still not numerically published");
            pending->retire_cpp_phase(); // SAME worker: root unmap/dirty RETURN, provider destroy, ACK
            pending.reset();
            pending_require(old_record.expired() && old_list.expired(),"last canonical owner RETURN");
            call->receiver.reset();
            call->parent_completed=true;
            break;
        }
        case AGFX_PENDING_QUIESCE_SESSION:
            pending_require(call->session && call->parent_completed && !call->transferred_work,
                "quiesce outside owned Work");
            qmu_session_begin_shutdown(call->session);
            call->quiesce=qmu_session_quiesce_host_callback_producers(call->session);
            pending_require(*call->quiesce==QMU_OK,"actual quiesce RETURN");
            break;
        case AGFX_PENDING_DESTROY_SESSION:
            pending_require(call->session && call->quiesce && *call->quiesce==QMU_OK &&
                !call->transferred_work,"destroy only after actual quiesce");
            qmu_destroy(call->session);
            call->session=nullptr; // only after actual destroy RETURN
            call->destroy_returned=true;
            break;
        default: throw std::logic_error("test action mismatch");
        }
    } catch (...) {
        if (call) call->worker_exception=std::current_exception();
        // Production anchor retains actual escaped graph; reservation guard
        // unwinds on THIS worker. Caller/facilities/Session are not destroyed.
    }
    if (call) call->worker_returned=true;
}
extern "C" AgfxTestPendingParentReturn agfx_test_pending_parent_main_cpp(
    AgfxTestPendingParentCall *call,AgfxTestPendingParentAction action,
    AgfxTestPendingParentDrive drive,void *context)
{
    try {
        pending_require(call && drive && !call->worker_exception && !call->main_exception,
            "main caller custody");
        call->worker_returned=false;
        if (action==AGFX_PENDING_RUN_PARENT_PHASE) {
            call->transferred_work=qmu::SessionWork::for_call(call->session);
            pending_require(bool(call->transferred_work),"accepted Work before worker publication");
        }
        drive(context,call,action); // returns only after normal C worker join
        pending_require(call->worker_returned,"actual C worker RETURN");
        call->transferred_work={};
        if (call->worker_exception) std::rethrow_exception(call->worker_exception);
        return AGFX_PENDING_CPP_RETURNED;
    } catch (...) {
        if (call) {
            call->transferred_work={};
            call->main_exception=std::current_exception(); // SAME original object/type
        }
        return AGFX_PENDING_CPP_ESCAPE_RETAINED; // host custody only, no SourceNS/result conversion
    }
}
extern "C" qmu_session *agfx_test_pending_parent_session(const AgfxTestPendingParentCall *call)
{ return call?call->session:nullptr; }
extern "C" int agfx_test_pending_parent_completed(const AgfxTestPendingParentCall *call)
{ return call && call->parent_completed; }
extern "C" int agfx_test_pending_parent_quiesced(const AgfxTestPendingParentCall *call)
{ return call && call->quiesce && *call->quiesce==QMU_OK; }
extern "C" int agfx_test_pending_parent_delete_returned(AgfxTestPendingParentCall *call)
{
    if (!call || call->session || !call->destroy_returned || !call->parent_completed ||
        call->receiver || call->transferred_work || call->worker_exception || call->main_exception)
        return 0;
    delete call; // last acquired full18 owner returns only after all native leaves
    return 1;
}
