/*
 * EEEE process-VM provider for Apple ParavirtualizedGraphics.
 *
 * Stock PGMappingTask owns persistent native virtual address ranges.  The
 * host adapter below translates that contract to QEMU's address-space and
 * MemoryRegion model without turning a host pointer into an identity.  A
 * retained MemoryRegion, shared RAM file descriptor, file offset, native VA
 * allocation and alias interval remain separate facts throughout the task
 * lifetime.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/rcu.h"
#include "qemu/thread.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/ramblock.h"

#include "apple-gfx-ml.h"
#include "apple-gfx-eeee-process-vm.h"
#include "qmu/qmetal_unified.h"

#include <sys/mman.h>

typedef struct AgfxEeeeAlias {
    MemoryRegion *region;
    uint64_t target;
    uint64_t region_offset;
    uint64_t length;
} AgfxEeeeAlias;

typedef struct AgfxEeeeAllocation {
    uint64_t address;
    uint64_t length;
    GPtrArray *aliases;
} AgfxEeeeAllocation;

struct AgfxEeeeProviderState {
    QemuMutex lock;
    /* address (pointing into AgfxEeeeAllocation) -> allocation */
    GHashTable *allocations;
    /* retained MemoryRegion identity -> reference count */
    GHashTable *regions;
    /* opaque QMetal LegacyTaskProvider pointer -> itself */
    GHashTable *published_tasks;
    bool destroying;
};

static bool agfx_eeee_range(uint64_t address, uint64_t length)
{
    return length != 0 && address <= UINT64_MAX - length;
}

static bool agfx_eeee_page_range(uint64_t address, uint64_t length)
{
    const uint64_t page_size = qemu_real_host_page_size();

    return page_size != 0 && agfx_eeee_range(address, length) &&
           (address % page_size) == 0 && (length % page_size) == 0;
}

static bool agfx_eeee_contains(uint64_t outer_address, uint64_t outer_length,
                               uint64_t inner_address, uint64_t inner_length)
{
    return agfx_eeee_range(outer_address, outer_length) &&
           agfx_eeee_range(inner_address, inner_length) &&
           inner_address >= outer_address &&
           inner_length <= outer_length - (inner_address - outer_address);
}

static bool agfx_eeee_intersects(uint64_t left_address, uint64_t left_length,
                                 uint64_t right_address, uint64_t right_length)
{
    return agfx_eeee_range(left_address, left_length) &&
           agfx_eeee_range(right_address, right_length) &&
           left_address < right_address + right_length &&
           right_address < left_address + left_length;
}

static void agfx_eeee_alias_destroy(gpointer data)
{
    g_free(data);
}

static void agfx_eeee_allocation_destroy(gpointer data)
{
    AgfxEeeeAllocation *allocation = data;

    if (!allocation) {
        return;
    }
    g_ptr_array_free(allocation->aliases, true);
    g_free(allocation);
}

AgfxEeeeProviderState *agfx_eeee_provider_state_create(void)
{
    AgfxEeeeProviderState *state = g_new0(AgfxEeeeProviderState, 1);

    qemu_mutex_init(&state->lock);
    state->allocations = g_hash_table_new_full(g_int64_hash, g_int64_equal,
                                               NULL, agfx_eeee_allocation_destroy);
    state->regions = g_hash_table_new(g_direct_hash, g_direct_equal);
    state->published_tasks = g_hash_table_new(g_direct_hash, g_direct_equal);
    return state;
}

bool agfx_eeee_provider_state_destroy(AgfxEeeeProviderState *state)
{
    bool empty;

    if (!state) {
        return true;
    }
    qemu_mutex_lock(&state->lock);
    state->destroying = true;
    empty = g_hash_table_size(state->allocations) == 0 &&
            g_hash_table_size(state->regions) == 0 &&
            g_hash_table_size(state->published_tasks) == 0;
    qemu_mutex_unlock(&state->lock);
    if (!empty) {
        return false;
    }
    g_hash_table_destroy(state->published_tasks);
    g_hash_table_destroy(state->regions);
    g_hash_table_destroy(state->allocations);
    qemu_mutex_destroy(&state->lock);
    g_free(state);
    return true;
}

static AgfxEeeeProviderState *agfx_eeee_state(void *ctx)
{
    AppleGfxMLState *s = ctx;

    if (!s || !s->eeee_provider_state) {
        /* This is an impossible partial-cutover invocation.  Returning an
         * ordinary provider failure here would let one ABI/callback subset
         * masquerade as the complete source owner graph. */
        abort();
    }
    return s->eeee_provider_state;
}

static AgfxEeeeAllocation *agfx_eeee_allocation_exact(
    AgfxEeeeProviderState *state, uint64_t address)
{
    return g_hash_table_lookup(state->allocations, &address);
}

static AgfxEeeeAllocation *agfx_eeee_allocation_containing(
    AgfxEeeeProviderState *state, uint64_t address, uint64_t length)
{
    GHashTableIter iter;
    gpointer key;
    gpointer value;

    g_hash_table_iter_init(&iter, state->allocations);
    while (g_hash_table_iter_next(&iter, &key, &value)) {
        AgfxEeeeAllocation *allocation = value;

        if (agfx_eeee_contains(allocation->address, allocation->length,
                               address, length)) {
            return allocation;
        }
    }
    return NULL;
}

/* A fresh-zero overwrite removes exactly the overwritten alias interval.  A
 * surviving prefix/suffix remains an alias to the same retained region; it
 * is not copied and does not obtain an additional MemoryRegion reference. */
static void agfx_eeee_remove_overwritten_aliases(AgfxEeeeAllocation *allocation,
                                                 uint64_t address, uint64_t length)
{
    const uint64_t end = address + length;
    guint index = 0;

    while (index < allocation->aliases->len) {
        AgfxEeeeAlias *alias = g_ptr_array_index(allocation->aliases, index);
        const uint64_t alias_end = alias->target + alias->length;

        if (!agfx_eeee_intersects(alias->target, alias->length, address, length)) {
            ++index;
            continue;
        }
        if (alias->target < address && alias_end > end) {
            AgfxEeeeAlias *suffix = g_new(AgfxEeeeAlias, 1);

            *suffix = *alias;
            suffix->target = end;
            suffix->region_offset += end - alias->target;
            suffix->length = alias_end - end;
            alias->length = address - alias->target;
            g_ptr_array_insert(allocation->aliases, index + 1, suffix);
            index += 2;
            continue;
        }
        if (alias->target < address) {
            alias->length = address - alias->target;
            ++index;
            continue;
        }
        if (alias_end > end) {
            alias->region_offset += end - alias->target;
            alias->target = end;
            alias->length = alias_end - end;
            ++index;
            continue;
        }
        g_ptr_array_remove_index(allocation->aliases, index);
    }
}

static int agfx_eeee_task_allocate(void *ctx, uint64_t *address,
                                   uint64_t length, uint32_t placement)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);
    void *requested = NULL;
    void *mapped;
    int flags = MAP_PRIVATE | MAP_ANONYMOUS;
    AgfxEeeeAllocation *allocation;

    if (!address || !agfx_eeee_page_range(0, length) || length > SIZE_MAX) {
        return 0;
    }
    switch (placement) {
    case QMU_EEEE_TASK_ALLOCATE_ANYWHERE:
        /* The QMetal caller deliberately leaves this input uninitialized.
         * Native ANYWHERE allocation must not inspect it. */
        break;
    case QMU_EEEE_TASK_ALLOCATE_FIXED_OVERWRITE:
        if (!agfx_eeee_page_range(*address, length)) {
            return 0;
        }
        requested = (void *)(uintptr_t)*address;
        flags |= MAP_FIXED;
        break;
    default:
        /* A new QMetal provider allocation mode needs a new reviewed ABI. */
        abort();
    }

    if (placement == QMU_EEEE_TASK_ALLOCATE_FIXED_OVERWRITE) {
        /* Do not let MAP_FIXED destroy an unrelated native mapping before
         * the exact task allocation has established ownership of the range. */
        qemu_mutex_lock(&state->lock);
        allocation = agfx_eeee_allocation_containing(state, *address, length);
        if (!allocation || state->destroying) {
            qemu_mutex_unlock(&state->lock);
            return 0;
        }
        mapped = mmap(requested, length, PROT_READ | PROT_WRITE, flags, -1, 0);
        if (mapped == MAP_FAILED || mapped != requested) {
            /* A successful MAP_FIXED is required to return its requested
             * address.  The other case is a platform contract violation. */
            if (mapped != MAP_FAILED) {
                abort();
            }
            qemu_mutex_unlock(&state->lock);
            return 0;
        }
        agfx_eeee_remove_overwritten_aliases(allocation, *address, length);
        qemu_mutex_unlock(&state->lock);
        return 1;
    }

    mapped = mmap(requested, length, PROT_READ | PROT_WRITE, flags, -1, 0);
    if (mapped == MAP_FAILED || (requested && mapped != requested)) {
        if (mapped != MAP_FAILED) {
            munmap(mapped, length);
        }
        return 0;
    }

    qemu_mutex_lock(&state->lock);
    if (state->destroying) {
        qemu_mutex_unlock(&state->lock);
        munmap(mapped, length);
        return 0;
    }
    allocation = g_new0(AgfxEeeeAllocation, 1);
    allocation->address = (uint64_t)(uintptr_t)mapped;
    allocation->length = length;
    allocation->aliases = g_ptr_array_new_with_free_func(agfx_eeee_alias_destroy);
    g_hash_table_insert(state->allocations, &allocation->address, allocation);
    qemu_mutex_unlock(&state->lock);
    *address = allocation->address;
    return 1;
}

static int agfx_eeee_task_remap_shared_fixed_overwrite(
    void *ctx, uint64_t *address, uint64_t length, uint64_t mask,
    uint64_t source, int *current_protection, int *maximum_protection)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);
    MemoryRegion *region;
    RAMBlock *ram_block;
    void *base;
    uint64_t region_offset;
    uint64_t file_offset;
    int fd;
    void *mapped;
    AgfxEeeeAllocation *allocation;
    AgfxEeeeAlias *alias;
    const uint64_t page_size = qemu_real_host_page_size();

    if (!address || !current_protection || !maximum_protection ||
        mask != page_size - 1 || !agfx_eeee_page_range(*address, length) ||
        !agfx_eeee_page_range(source, length)) {
        return 0;
    }
    /* QMetal holds RCU across translate/direct/RAM pointer acquisition and
     * this callback.  The pointer is a source-RAM location, never task VA. */
    region = memory_region_from_host((void *)(uintptr_t)source, &region_offset);
    if (!region || !region->ram_block) {
        return 0;
    }
    ram_block = region->ram_block;
    base = memory_region_get_ram_ptr(region);
    if (!base || source < (uint64_t)(uintptr_t)base ||
        region_offset != source - (uint64_t)(uintptr_t)base ||
        region_offset > qemu_ram_get_used_length(ram_block) ||
        length > qemu_ram_get_used_length(ram_block) - region_offset ||
        !qemu_ram_is_shared(ram_block)) {
        return 0;
    }
    fd = qemu_ram_get_fd(ram_block);
    file_offset = qemu_ram_get_fd_offset(ram_block);
    if (fd < 0 || file_offset > UINT64_MAX - region_offset ||
        !agfx_eeee_page_range(file_offset + region_offset, length)) {
        return 0;
    }

    /* QMetal's complete source provider calls remap under task_lock.  Taking
     * it again would turn the source single critical section into a recursive
     * QEMU lock acquisition. */
    allocation = agfx_eeee_allocation_containing(state, *address, length);
    if (!allocation || state->destroying ||
        GPOINTER_TO_UINT(g_hash_table_lookup(state->regions, region)) == 0) {
        return 0;
    }
    mapped = mmap((void *)(uintptr_t)*address, length, PROT_READ | PROT_WRITE,
                  MAP_SHARED | MAP_FIXED, fd, file_offset + region_offset);
    if (mapped == MAP_FAILED || mapped != (void *)(uintptr_t)*address) {
        if (mapped != MAP_FAILED) {
            munmap(mapped, length);
        }
        return 0;
    }
    agfx_eeee_remove_overwritten_aliases(allocation, *address, length);
    alias = g_new(AgfxEeeeAlias, 1);
    *alias = (AgfxEeeeAlias){
        .region = region,
        .target = *address,
        .region_offset = region_offset,
        .length = length,
    };
    g_ptr_array_add(allocation->aliases, alias);

    *current_protection = PROT_READ | PROT_WRITE;
    *maximum_protection = PROT_READ | PROT_WRITE;
    return 1;
}

static int agfx_eeee_task_protect(void *ctx, uint64_t address,
                                  uint64_t length, uint32_t protection)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);
    int native_protection;
    AgfxEeeeAllocation *allocation;

    if (!agfx_eeee_page_range(address, length)) {
        return 0;
    }
    switch (protection) {
    case QMU_EEEE_TASK_PROTECT_NONE:
        native_protection = PROT_NONE;
        break;
    case QMU_EEEE_TASK_PROTECT_READ_ONLY:
        native_protection = PROT_READ;
        break;
    case QMU_EEEE_TASK_PROTECT_READ_WRITE:
        native_protection = PROT_READ | PROT_WRITE;
        break;
    default:
        abort();
    }
    /* All source protect calls are made by QMetal under task_lock: initial
     * no-access, final map protection, and post-zero-overwrite no-access. */
    allocation = agfx_eeee_allocation_containing(state, address, length);
    if (!allocation || state->destroying ||
        mprotect((void *)(uintptr_t)address, length, native_protection) != 0) {
        return 0;
    }
    return 1;
}

static int agfx_eeee_task_deallocate(void *ctx, uint64_t address,
                                     uint64_t length)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);
    AgfxEeeeAllocation *allocation;

    if (!agfx_eeee_page_range(address, length)) {
        return -1;
    }
    qemu_mutex_lock(&state->lock);
    allocation = agfx_eeee_allocation_exact(state, address);
    if (!allocation || allocation->length != length || state->destroying ||
        munmap((void *)(uintptr_t)address, length) != 0) {
        qemu_mutex_unlock(&state->lock);
        return -1;
    }
    g_hash_table_remove(state->allocations, &address);
    qemu_mutex_unlock(&state->lock);
    return 0;
}

static uint64_t agfx_eeee_task_host_page_size(void *ctx)
{
    (void)agfx_eeee_state(ctx);
    return qemu_real_host_page_size();
}

static void agfx_eeee_task_rcu_enter(void *ctx)
{
    (void)agfx_eeee_state(ctx);
    rcu_read_lock();
}

static void agfx_eeee_task_rcu_leave(void *ctx)
{
    (void)agfx_eeee_state(ctx);
    rcu_read_unlock();
}

static void agfx_eeee_task_lock(void *ctx)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);

    qemu_mutex_lock(&state->lock);
}

static void agfx_eeee_task_unlock(void *ctx)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);

    qemu_mutex_unlock(&state->lock);
}

static int agfx_eeee_task_translate(void *ctx, uint64_t gpa, uint64_t length,
                                    int write, void **region,
                                    uint64_t *region_offset, uint64_t *covered)
{
    MemoryRegion *translated;
    hwaddr xlat;
    hwaddr xlat_length;

    (void)agfx_eeee_state(ctx);
    if (!region || !region_offset || !covered || length == 0 ||
        length > HWADDR_MAX) {
        return 0;
    }
    xlat = 0;
    xlat_length = length;
    translated = address_space_translate(&address_space_memory, gpa, &xlat,
                                         &xlat_length, write != 0,
                                         MEMTXATTRS_UNSPECIFIED);
    if (!translated || xlat_length == 0 || xlat_length > length) {
        return 0;
    }
    *region = translated;
    *region_offset = xlat;
    *covered = xlat_length;
    return 1;
}

static int agfx_eeee_task_direct(void *ctx, void *region, int write)
{
    (void)agfx_eeee_state(ctx);
    return region && memory_access_is_direct(region, write != 0,
                                              MEMTXATTRS_UNSPECIFIED);
}

static void *agfx_eeee_task_ram_pointer(void *ctx, void *region)
{
    (void)agfx_eeee_state(ctx);
    return region ? memory_region_get_ram_ptr(region) : NULL;
}

static void agfx_eeee_task_region_ref(void *ctx, void *opaque_region)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);
    MemoryRegion *region = opaque_region;
    guint count;

    if (!region) {
        abort();
    }
    memory_region_ref(region);
    /* map() calls this under task_lock immediately before its first remap.
     * This callback takes the actual QOM reference; the mapping remains owned
     * by QMetal until its provider deleter later invokes region_unref(). */
    count = GPOINTER_TO_UINT(g_hash_table_lookup(state->regions, region));
    if (count == G_MAXUINT || state->destroying) {
        memory_region_unref(region);
        abort();
    }
    g_hash_table_insert(state->regions, region, GUINT_TO_POINTER(count + 1));
}

static void agfx_eeee_task_region_unref(void *ctx, void *opaque_region)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);
    MemoryRegion *region = opaque_region;
    guint count;

    if (!region) {
        abort();
    }
    qemu_mutex_lock(&state->lock);
    count = GPOINTER_TO_UINT(g_hash_table_lookup(state->regions, region));
    if (count == 0) {
        qemu_mutex_unlock(&state->lock);
        abort();
    }
    if (count == 1) {
        g_hash_table_remove(state->regions, region);
    } else {
        g_hash_table_insert(state->regions, region, GUINT_TO_POINTER(count - 1));
    }
    qemu_mutex_unlock(&state->lock);
    memory_region_unref(region);
}

static void agfx_eeee_task_publish(void *ctx, void *task)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);

    if (!task) {
        abort();
    }
    /* QMetal calls this while its provider task lock is held.  It is not a
     * separate QEMU mutex acquisition. */
    if (state->destroying || g_hash_table_contains(state->published_tasks, task)) {
        abort();
    }
    g_hash_table_add(state->published_tasks, task);
}

static void agfx_eeee_task_remove(void *ctx, void *task)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);

    if (!task || !g_hash_table_remove(state->published_tasks, task)) {
        abort();
    }
}

static void agfx_eeee_task_host_write(void *ctx, uint64_t address,
                                      uint64_t length)
{
    AgfxEeeeProviderState *state = agfx_eeee_state(ctx);
    AgfxEeeeAllocation *allocation;
    guint index;

    if (!agfx_eeee_range(address, length)) {
        abort();
    }
    qemu_mutex_lock(&state->lock);
    allocation = agfx_eeee_allocation_containing(state, address, length);
    if (!allocation || state->destroying) {
        qemu_mutex_unlock(&state->lock);
        abort();
    }
    for (index = 0; index < allocation->aliases->len; ++index) {
        AgfxEeeeAlias *alias = g_ptr_array_index(allocation->aliases, index);
        const uint64_t begin = MAX(address, alias->target);
        const uint64_t end = MIN(address + length, alias->target + alias->length);

        if (begin < end) {
            memory_region_set_dirty(alias->region,
                                    alias->region_offset + begin - alias->target,
                                    end - begin);
        }
    }
    qemu_mutex_unlock(&state->lock);
}

static void agfx_eeee_task_fatal(void *ctx, uint32_t reason)
{
    (void)agfx_eeee_state(ctx);
    switch (reason) {
    case QMU_EEEE_TASK_FATAL_ALLOCATION:
    case QMU_EEEE_TASK_FATAL_REMAP:
    case QMU_EEEE_TASK_FATAL_UNMAP:
        error_report("apple-gfx-ml: EEEE process-VM provider fatal reason %u", reason);
        abort();
    default:
        /* A future enum cannot be coerced into an old provider fatal path. */
        abort();
    }
}

void agfx_eeee_provider_fill_callbacks(qmu_extended_callbacks *callbacks)
{
    if (!callbacks) {
        abort();
    }
    callbacks->eeee_task_allocate = agfx_eeee_task_allocate;
    callbacks->eeee_task_remap_shared_fixed_overwrite =
        agfx_eeee_task_remap_shared_fixed_overwrite;
    callbacks->eeee_task_protect = agfx_eeee_task_protect;
    callbacks->eeee_task_deallocate = agfx_eeee_task_deallocate;
    callbacks->eeee_task_host_page_size = agfx_eeee_task_host_page_size;
    callbacks->eeee_task_rcu_enter = agfx_eeee_task_rcu_enter;
    callbacks->eeee_task_rcu_leave = agfx_eeee_task_rcu_leave;
    callbacks->eeee_task_lock = agfx_eeee_task_lock;
    callbacks->eeee_task_unlock = agfx_eeee_task_unlock;
    callbacks->eeee_task_translate = agfx_eeee_task_translate;
    callbacks->eeee_task_direct = agfx_eeee_task_direct;
    callbacks->eeee_task_ram_pointer = agfx_eeee_task_ram_pointer;
    callbacks->eeee_task_region_ref = agfx_eeee_task_region_ref;
    callbacks->eeee_task_region_unref = agfx_eeee_task_region_unref;
    callbacks->eeee_task_publish = agfx_eeee_task_publish;
    callbacks->eeee_task_remove = agfx_eeee_task_remove;
    callbacks->eeee_task_host_write = agfx_eeee_task_host_write;
    callbacks->eeee_task_fatal = agfx_eeee_task_fatal;
}
