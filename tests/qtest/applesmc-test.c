/*
 * QTests for the AppleSMC diagnostic PIO ring
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "libqtest.h"
#include "qobject/qdict.h"
#include "qobject/qjson.h"
#include "qobject/qlist.h"
#include "qobject/qstring.h"

#define APPLESMC_QOM_PATH "/machine/peripheral/smc0"
#define APPLESMC_DATA_PORT 0x300
#define APPLESMC_CMD_PORT  0x304
#define APPLESMC_RING_CAPACITY 1024

static QTestState *applesmc_start(bool ring_enabled)
{
    const char *ring_option = ring_enabled ? ",x-pio-ring-enabled=on" : "";

    return qtest_initf("-nodefaults -machine q35 -device isa-applesmc,"
                       "id=smc0,osk=0123456789abcdef0123456789abcdef"
                       "0123456789abcdef0123456789abcdef%s", ring_option);
}

static QDict *applesmc_ring_dump(QTestState *qts)
{
    QDict *response;
    QObject *parsed;
    QString *ring;
    Error *err = NULL;

    response = qtest_qmp(qts,
                         "{ 'execute': 'qom-get', 'arguments': {"
                         "'path': '" APPLESMC_QOM_PATH "', "
                         "'property': 'x-pio-ring' } }");
    g_assert(qdict_haskey(response, "return"));
    ring = qobject_to(QString, qdict_get(response, "return"));
    parsed = qobject_from_json(qstring_get_str(ring), &err);
    g_assert(!err);
    g_assert(parsed);
    g_assert_cmpint(qobject_type(parsed), ==, QTYPE_QDICT);
    qobject_unref(response);

    return qobject_to(QDict, parsed);
}

static QDict *applesmc_ring_record(const QList *records, size_t index)
{
    const QListEntry *entry = qlist_first(records);

    while (index--) {
        g_assert(entry);
        entry = qlist_next(entry);
    }
    g_assert(entry);
    return qobject_to(QDict, qlist_entry_obj(entry));
}

static void assert_record_schema(const QDict *record, uint64_t seq)
{
    static const char *const keys[] = {
        "seq", "cpu_index", "port", "direction", "value_valid", "value",
        "cmd", "status", "status_1e", "read_pos", "data_len", "data_pos",
        "key",
    };
    size_t i;

    for (i = 0; i < G_N_ELEMENTS(keys); i++) {
        g_assert(qdict_haskey(record, keys[i]));
    }
    g_assert_cmpuint(qdict_get_uint(record, "seq"), ==, seq);
    g_assert_cmpint(qdict_get_int(record, "cpu_index"), >=, -1);
}

static void applesmc_read_rev(QTestState *qts)
{
    static const uint8_t rev[] = { 0x01, 0x13, 0x0f, 0x00, 0x00, 0x03 };
    size_t i;

    qtest_outb(qts, APPLESMC_CMD_PORT, 0x10);
    qtest_outb(qts, APPLESMC_DATA_PORT, 'R');
    qtest_outb(qts, APPLESMC_DATA_PORT, 'E');
    qtest_outb(qts, APPLESMC_DATA_PORT, 'V');
    qtest_outb(qts, APPLESMC_DATA_PORT, ' ');
    qtest_outb(qts, APPLESMC_DATA_PORT, 0x00);
    for (i = 0; i < G_N_ELEMENTS(rev); i++) {
        g_assert_cmphex(qtest_inb(qts, APPLESMC_DATA_PORT), ==, rev[i]);
    }
}

static void test_ring_disabled(void)
{
    QTestState *qts = applesmc_start(false);
    QDict *dump = applesmc_ring_dump(qts);

    g_assert(!qdict_get_bool(dump, "enabled"));
    g_assert_cmpuint(qdict_get_uint(dump, "capacity"), ==,
                     APPLESMC_RING_CAPACITY);
    g_assert_cmpuint(qdict_get_uint(dump, "count"), ==, 0);
    g_assert_cmpuint(qdict_get_uint(dump, "next_seq"), ==, 0);
    g_assert_cmpuint(qlist_size(qdict_get_qlist(dump, "records")), ==, 0);
    qobject_unref(dump);

    applesmc_read_rev(qts);
    dump = applesmc_ring_dump(qts);
    g_assert_cmpuint(qdict_get_uint(dump, "count"), ==, 0);
    g_assert_cmpuint(qdict_get_uint(dump, "next_seq"), ==, 0);
    qobject_unref(dump);
    qtest_quit(qts);
}

static void test_ring_ordered_records_and_redaction(void)
{
    QTestState *qts = applesmc_start(true);
    QDict *dump;
    QList *records;
    QDict *record;
    size_t i;

    applesmc_read_rev(qts);
    dump = applesmc_ring_dump(qts);
    records = qdict_get_qlist(dump, "records");
    g_assert(qdict_get_bool(dump, "enabled"));
    g_assert_cmpuint(qdict_get_uint(dump, "count"), ==, 12);
    g_assert_cmpuint(qlist_size(records), ==, 12);
    for (i = 0; i < qlist_size(records); i++) {
        assert_record_schema(applesmc_ring_record(records, i), i);
    }

    record = applesmc_ring_record(records, 0);
    g_assert_cmpuint(qdict_get_uint(record, "port"), ==, 4);
    g_assert_cmpstr(qdict_get_str(record, "direction"), ==, "write");
    g_assert(qdict_get_bool(record, "value_valid"));
    g_assert_cmpuint(qdict_get_uint(record, "value"), ==, 0x10);
    g_assert_cmpuint(qdict_get_uint(record, "status"), ==, 0x0c);

    record = applesmc_ring_record(records, 5);
    g_assert_cmpuint(qdict_get_uint(record, "port"), ==, 0);
    g_assert_cmpstr(qdict_get_str(record, "direction"), ==, "write");
    g_assert_cmpuint(qdict_get_uint(record, "status"), ==, 0x05);
    g_assert_cmpuint(qdict_get_uint(record, "read_pos"), ==, 5);
    g_assert_cmpuint(qdict_get_uint(record, "data_len"), ==, 6);
    g_assert_cmpstr(qdict_get_str(record, "key"), ==, "52455620");

    record = applesmc_ring_record(records, 6);
    g_assert_cmpuint(qdict_get_uint(record, "port"), ==, 0);
    g_assert_cmpstr(qdict_get_str(record, "direction"), ==, "read");
    g_assert(!qdict_get_bool(record, "value_valid"));
    g_assert_cmpint(qobject_type(qdict_get(record, "value")), ==, QTYPE_QNULL);
    g_assert_cmpuint(qdict_get_uint(record, "data_pos"), ==, 1);

    record = applesmc_ring_record(records, 11);
    g_assert_cmpuint(qdict_get_uint(record, "status"), ==, 0);
    g_assert_cmpuint(qdict_get_uint(record, "data_pos"), ==, 6);
    qobject_unref(dump);
    qtest_quit(qts);
}

static void test_ring_interrupted_command_order(void)
{
    QTestState *qts = applesmc_start(true);
    QDict *dump;
    QList *records;
    QDict *record;

    qtest_outb(qts, APPLESMC_CMD_PORT, 0x10);
    qtest_outb(qts, APPLESMC_DATA_PORT, 'R');
    qtest_outb(qts, APPLESMC_CMD_PORT, 0x10);

    dump = applesmc_ring_dump(qts);
    records = qdict_get_qlist(dump, "records");
    g_assert_cmpuint(qlist_size(records), ==, 3);
    record = applesmc_ring_record(records, 2);
    assert_record_schema(record, 2);
    g_assert_cmpuint(qdict_get_uint(record, "port"), ==, 4);
    g_assert_cmpuint(qdict_get_uint(record, "status"), ==, 0x08);
    g_assert_cmpuint(qdict_get_uint(record, "status_1e"), ==, 0x80);
    g_assert_cmpuint(qdict_get_uint(record, "read_pos"), ==, 0);
    qobject_unref(dump);
    qtest_quit(qts);
}

static void test_ring_redacts_osk_data(void)
{
    QTestState *qts = applesmc_start(true);
    QDict *dump;
    QList *records;
    size_t i;

    qtest_outb(qts, APPLESMC_CMD_PORT, 0x10);
    qtest_outb(qts, APPLESMC_DATA_PORT, 'O');
    qtest_outb(qts, APPLESMC_DATA_PORT, 'S');
    qtest_outb(qts, APPLESMC_DATA_PORT, 'K');
    qtest_outb(qts, APPLESMC_DATA_PORT, '0');
    qtest_outb(qts, APPLESMC_DATA_PORT, 0x00);
    for (i = 0; i < 32; i++) {
        qtest_inb(qts, APPLESMC_DATA_PORT);
    }

    dump = applesmc_ring_dump(qts);
    records = qdict_get_qlist(dump, "records");
    g_assert_cmpuint(qlist_size(records), ==, 38);
    for (i = 6; i < qlist_size(records); i++) {
        QDict *record = applesmc_ring_record(records, i);

        g_assert_cmpuint(qdict_get_uint(record, "port"), ==, 0);
        g_assert_cmpstr(qdict_get_str(record, "direction"), ==, "read");
        g_assert(!qdict_get_bool(record, "value_valid"));
        g_assert_cmpint(qobject_type(qdict_get(record, "value")), ==,
                        QTYPE_QNULL);
    }
    qobject_unref(dump);
    qtest_quit(qts);
}

static void test_ring_persists_across_reset(void)
{
    QTestState *qts = applesmc_start(true);
    QDict *dump;

    qtest_outb(qts, APPLESMC_CMD_PORT, 0x10);
    qtest_system_reset(qts);

    dump = applesmc_ring_dump(qts);
    g_assert_cmpuint(qdict_get_uint(dump, "count"), ==, 1);
    g_assert_cmpuint(qdict_get_uint(dump, "next_seq"), ==, 1);
    g_assert_cmpuint(qdict_get_uint(
                         applesmc_ring_record(qdict_get_qlist(dump, "records"),
                                              0), "seq"), ==, 0);
    qobject_unref(dump);
    qtest_quit(qts);
}

static void test_ring_wraps_and_reports_loss(void)
{
    QTestState *qts = applesmc_start(true);
    QDict *dump;
    QList *records;
    uint32_t i;

    for (i = 0; i < APPLESMC_RING_CAPACITY + 1; i++) {
        qtest_outb(qts, APPLESMC_CMD_PORT, 0x10);
    }
    dump = applesmc_ring_dump(qts);
    records = qdict_get_qlist(dump, "records");
    g_assert_cmpuint(qdict_get_uint(dump, "count"), ==,
                     APPLESMC_RING_CAPACITY);
    g_assert_cmpuint(qdict_get_uint(dump, "dropped"), ==, 1);
    g_assert_cmpuint(qdict_get_uint(dump, "next_seq"), ==,
                     APPLESMC_RING_CAPACITY + 1);
    g_assert_cmpuint(qlist_size(records), ==, APPLESMC_RING_CAPACITY);
    g_assert_cmpuint(qdict_get_uint(applesmc_ring_record(records, 0), "seq"),
                     ==, 1);
    g_assert_cmpuint(qdict_get_uint(
                         applesmc_ring_record(records,
                                              APPLESMC_RING_CAPACITY - 1),
                         "seq"), ==, APPLESMC_RING_CAPACITY);
    qobject_unref(dump);
    qtest_quit(qts);
}

static void test_ring_properties_are_read_only_after_realize(void)
{
    QTestState *qts = applesmc_start(true);
    QDict *response;

    response = qtest_qmp(qts,
                         "{ 'execute': 'qom-set', 'arguments': {"
                         "'path': '" APPLESMC_QOM_PATH "', "
                         "'property': 'x-pio-ring', 'value': 'forbidden' } }");
    g_assert(qdict_haskey(response, "error"));
    qobject_unref(response);

    response = qtest_qmp(qts,
                         "{ 'execute': 'qom-set', 'arguments': {"
                         "'path': '" APPLESMC_QOM_PATH "', "
                         "'property': 'x-pio-ring-enabled', 'value': false } }");
    g_assert(qdict_haskey(response, "error"));
    qobject_unref(response);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/applesmc/ring-disabled", test_ring_disabled);
    qtest_add_func("/applesmc/ring-ordered-records-and-redaction",
                   test_ring_ordered_records_and_redaction);
    qtest_add_func("/applesmc/ring-interrupted-command-order",
                   test_ring_interrupted_command_order);
    qtest_add_func("/applesmc/ring-redacts-osk-data",
                   test_ring_redacts_osk_data);
    qtest_add_func("/applesmc/ring-persists-across-reset",
                   test_ring_persists_across_reset);
    qtest_add_func("/applesmc/ring-wraps-and-reports-loss",
                   test_ring_wraps_and_reports_loss);
    qtest_add_func("/applesmc/ring-properties-read-only-after-realize",
                   test_ring_properties_are_read_only_after_realize);

    return g_test_run();
}
