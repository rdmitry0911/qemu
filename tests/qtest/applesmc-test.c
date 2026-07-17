/*
 * QTests for AppleSMC GET_KEY_BY_INDEX.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "libqtest.h"

#define APPLESMC_DATA_PORT 0x300
#define APPLESMC_CMD_PORT  0x304
#define APPLESMC_ERR_PORT  0x31e

static QTestState *applesmc_start(void)
{
    return qtest_init("-nodefaults -machine q35 -device isa-applesmc,"
                      "osk=0123456789abcdef0123456789abcdef"
                      "0123456789abcdef0123456789abcdef");
}

static void applesmc_write_index(QTestState *qts, uint32_t index,
                                 uint8_t final_status)
{
    int shift;

    qtest_outb(qts, APPLESMC_CMD_PORT, 0x12);
    g_assert_cmphex(qtest_inb(qts, APPLESMC_CMD_PORT), ==, 0x0c);
    for (shift = 24; shift >= 0; shift -= 8) {
        qtest_outb(qts, APPLESMC_DATA_PORT, index >> shift);
        g_assert_cmphex(qtest_inb(qts, APPLESMC_CMD_PORT), ==,
                        shift == 0 ? final_status : 0x04);
    }
}

static void applesmc_assert_index_key(QTestState *qts, uint32_t index,
                                      const char expected[4])
{
    size_t i;

    applesmc_write_index(qts, index, 0x05);
    g_assert_cmphex(qtest_inb(qts, APPLESMC_ERR_PORT), ==, 0x00);
    for (i = 0; i < 4; i++) {
        g_assert_cmphex(qtest_inb(qts, APPLESMC_DATA_PORT), ==,
                        (uint8_t)expected[i]);
        g_assert_cmphex(qtest_inb(qts, APPLESMC_CMD_PORT), ==,
                        i == 3 ? 0x00 : 0x05);
    }
}

static void test_get_key_by_index_existing_order(void)
{
    QTestState *qts = applesmc_start();

    /* QLIST_INSERT_HEAD makes these the first and last current model keys. */
    applesmc_assert_index_key(qts, 0, "MSSD");
    applesmc_assert_index_key(qts, 5, "REV ");
    qtest_quit(qts);
}

static void test_get_key_by_index_big_endian_range_error(void)
{
    QTestState *qts = applesmc_start();

    /* A little-endian implementation would incorrectly select index one. */
    applesmc_write_index(qts, UINT32_C(0x01000000), 0x00);
    g_assert_cmphex(qtest_inb(qts, APPLESMC_ERR_PORT), ==, 0xb8);
    qtest_quit(qts);
}

static void test_get_key_by_index_out_of_range_recovery(void)
{
    QTestState *qts = applesmc_start();

    applesmc_write_index(qts, 6, 0x00);
    g_assert_cmphex(qtest_inb(qts, APPLESMC_ERR_PORT), ==, 0xb8);
    applesmc_assert_index_key(qts, 0, "MSSD");
    qtest_quit(qts);
}

static void test_get_key_by_index_collision(void)
{
    QTestState *qts = applesmc_start();

    qtest_outb(qts, APPLESMC_CMD_PORT, 0x12);
    qtest_outb(qts, APPLESMC_DATA_PORT, 0x00);
    g_assert_cmphex(qtest_inb(qts, APPLESMC_CMD_PORT), ==, 0x04);
    qtest_outb(qts, APPLESMC_CMD_PORT, 0x10);
    g_assert_cmphex(qtest_inb(qts, APPLESMC_CMD_PORT), ==, 0x08);
    g_assert_cmphex(qtest_inb(qts, APPLESMC_ERR_PORT), ==, 0x80);
    qtest_quit(qts);
}

static void test_read_command_unchanged(void)
{
    static const uint8_t rev[] = { 0x01, 0x13, 0x0f, 0x00, 0x00, 0x03 };
    QTestState *qts = applesmc_start();
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
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/applesmc/get-key-by-index/existing-order",
                   test_get_key_by_index_existing_order);
    qtest_add_func("/applesmc/get-key-by-index/big-endian-range-error",
                   test_get_key_by_index_big_endian_range_error);
    qtest_add_func("/applesmc/get-key-by-index/out-of-range-recovery",
                   test_get_key_by_index_out_of_range_recovery);
    qtest_add_func("/applesmc/get-key-by-index/collision",
                   test_get_key_by_index_collision);
    qtest_add_func("/applesmc/read-command/unchanged",
                   test_read_command_unchanged);

    return g_test_run();
}
