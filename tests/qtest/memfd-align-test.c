/*
 * memory-backend-memfd 'align' property.
 *
 * Accepted alignments place the memfd mapping at that alignment and leave the
 * seals as they were; the default (0) keeps the memfd page size; alignments
 * that are not a power of two or not a multiple of the page size are rejected
 * by the RAM block allocator, as for memory-backend-file; the property cannot
 * change after the backend exists.  The mapping and the seals are read from
 * the QEMU process through /proc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define MiB (1024 * 1024)
#define MEMFD_SEALS (F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL)

typedef struct MemfdMap {
    uint64_t start, end;
    uint64_t ino;
} MemfdMap;

/* mappings of a memory-backend-memfd file in the QEMU process */
static GArray *memfd_maps(QTestState *qts)
{
    g_autofree char *path = g_strdup_printf("/proc/%d/maps", qtest_pid(qts));
    g_autofree char *text = NULL;
    GArray *maps = g_array_new(false, false, sizeof(MemfdMap));
    g_auto(GStrv) lines = NULL;

    g_assert(g_file_get_contents(path, &text, NULL, NULL));
    lines = g_strsplit(text, "\n", -1);
    for (int i = 0; lines[i]; i++) {
        MemfdMap m;
        char perms[8], dev[16];
        unsigned long long off, ino;
        unsigned long long start, end;

        if (!strstr(lines[i], "/memfd:memory-backend-memfd")) {
            continue;
        }
        g_assert_cmpint(sscanf(lines[i], "%llx-%llx %7s %llx %15s %llu",
                               &start, &end, perms, &off, dev, &ino), ==, 6);
        m.start = start;
        m.end = end;
        m.ino = ino;
        g_array_append_val(maps, m);
    }
    return maps;
}

/* the one mapping in @after that is not in @before */
static MemfdMap new_map(GArray *before, GArray *after)
{
    MemfdMap found = { 0 };
    int n = 0;

    for (guint i = 0; i < after->len; i++) {
        MemfdMap a = g_array_index(after, MemfdMap, i);
        bool old = false;

        for (guint j = 0; j < before->len; j++) {
            old |= g_array_index(before, MemfdMap, j).start == a.start;
        }
        if (!old) {
            found = a;
            n++;
        }
    }
    g_assert_cmpint(n, ==, 1);
    return found;
}

/* F_GET_SEALS of the QEMU fd whose file has inode @ino */
static int memfd_seals(QTestState *qts, uint64_t ino)
{
    g_autofree char *dir = g_strdup_printf("/proc/%d/fd", qtest_pid(qts));
    GDir *d = g_dir_open(dir, 0, NULL);
    const char *e;
    int seals = -1;

    g_assert(d);
    while ((e = g_dir_read_name(d))) {
        g_autofree char *p = g_strdup_printf("%s/%s", dir, e);
        struct stat st;
        int fd;

        if (stat(p, &st) || st.st_ino != ino) {
            continue;
        }
        fd = open(p, O_RDONLY);
        g_assert_cmpint(fd, >=, 0);
        seals = fcntl(fd, F_GET_SEALS);
        close(fd);
        break;
    }
    g_dir_close(d);
    g_assert_cmpint(seals, >=, 0);
    return seals;
}

static int64_t qom_get_align(QTestState *qts, const char *id)
{
    g_autofree char *path = g_strdup_printf("/objects/%s", id);
    QDict *rsp = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments':"
                           " { 'path': %s, 'property': 'align' } }", path);
    int64_t v;

    g_assert(qdict_haskey(rsp, "return"));
    v = qdict_get_int(rsp, "return");
    qobject_unref(rsp);
    return v;
}

static MemfdMap add_memfd(QTestState *qts, const char *id, int64_t align)
{
    g_autoptr(GArray) before = memfd_maps(qts);
    g_autoptr(GArray) after = NULL;

    if (align < 0) {
        qtest_qmp_assert_success(qts, "{ 'execute': 'object-add', 'arguments':"
                                 " { 'qom-type': 'memory-backend-memfd',"
                                 " 'id': %s, 'size': %u } }", id, 8 * MiB);
    } else {
        qtest_qmp_assert_success(qts, "{ 'execute': 'object-add', 'arguments':"
                                 " { 'qom-type': 'memory-backend-memfd',"
                                 " 'id': %s, 'size': %u, 'align': %" PRId64
                                 " } }", id, 8 * MiB, align);
    }
    after = memfd_maps(qts);
    return new_map(before, after);
}

static void assert_add_fails(QTestState *qts, int64_t align, const char *why)
{
    g_autoptr(GArray) before = memfd_maps(qts);
    g_autoptr(GArray) after = NULL;
    QDict *rsp = qtest_qmp(qts, "{ 'execute': 'object-add', 'arguments':"
                           " { 'qom-type': 'memory-backend-memfd',"
                           " 'id': 'bad', 'size': %u, 'align': %" PRId64
                           " } }", 8 * MiB, align);
    const char *desc;

    g_assert(qdict_haskey(rsp, "error"));
    desc = qdict_get_str(qdict_get_qdict(rsp, "error"), "desc");
    g_test_message("align 0x%" PRIx64 ": %s", align, desc);
    g_assert(strstr(desc, why));
    qobject_unref(rsp);
    after = memfd_maps(qts);
    g_assert_cmpint(after->len, ==, before->len);
}

static void test_memfd_align(void)
{
    QTestState *qts = qtest_init("-machine none");
    const int64_t page = qemu_real_host_page_size();
    MemfdMap m;
    QDict *rsp;

    /* default: align reads 0, the memfd keeps its seals */
    m = add_memfd(qts, "m-default", -1);
    g_assert_cmpint(qom_get_align(qts, "m-default"), ==, 0);
    g_assert_cmpint(m.end - m.start, ==, 8 * MiB);
    g_assert_cmpint(memfd_seals(qts, m.ino), ==, MEMFD_SEALS);

    /* 2 MiB: base address 2 MiB-aligned, seals unchanged */
    m = add_memfd(qts, "m-2m", 2 * MiB);
    g_assert_cmpint(qom_get_align(qts, "m-2m"), ==, 2 * MiB);
    g_assert_cmpint(m.start % (2 * MiB), ==, 0);
    g_assert_cmpint(m.end - m.start, ==, 8 * MiB);
    g_assert_cmpint(memfd_seals(qts, m.ino), ==, MEMFD_SEALS);

    /* 4 MiB too, so the alignment is not a coincidence of the first */
    m = add_memfd(qts, "m-4m", 4 * MiB);
    g_assert_cmpint(m.start % (4 * MiB), ==, 0);

    /* rejected as for memory-backend-file (file_ram_alloc) */
    assert_add_fails(qts, 3 * MiB, "must be a power of two");
    assert_add_fails(qts, page / 2, "must be multiples of page size");

    /* no change after creation */
    rsp = qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments':"
                    " { 'path': '/objects/m-2m', 'property': 'align',"
                    " 'value': %u } }", 4 * MiB);
    g_assert(qdict_haskey(rsp, "error"));
    g_assert(strstr(qdict_get_str(qdict_get_qdict(rsp, "error"), "desc"),
                    "cannot change property 'align'"));
    qobject_unref(rsp);
    g_assert_cmpint(qom_get_align(qts, "m-2m"), ==, 2 * MiB);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/memfd/align", test_memfd_align);
    return g_test_run();
}
