/*
 * Unit tests for the vfio-user client proxy (hw/vfio-user/proxy.c)
 *
 * The proxy under test is the production code: it is created with
 * vfio_user_connect_dev() against a real AF_UNIX listening socket owned by
 * the test, which plays the role of the vfio-user server.  The proxy's
 * receive handler runs on the real "vfio-user" IOThread.
 *
 * Contract VFIO-USER-PROXY-RECV-HDR-01 (server disconnect while the client
 * reads a message header):
 *  - io/channel.h: qio_channel_readv_full() returns -1 *with @errp set* on
 *    error, and 0 on EOF.
 *  - util/error.c: error_setv() requires *errp == NULL, so an error must not
 *    be set twice on the same @errp.
 *  - docs/interop/vfio-user.rst "Server Disconnection": the client must
 *    handle the server disconnecting (a restart or a crash) and reset the
 *    device rather than fail.
 * Expected: when the server closes the socket, the proxy reports one error
 * and moves to VFIO_PROXY_ERROR; it must not abort.  A server that closes
 * with unread client data produces ECONNRESET on Linux AF_UNIX stream
 * sockets; a server that drained the socket produces EOF.
 *
 * Device reset by the PCI layer (hw/vfio-user/pci.c) is not covered here.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <sys/ioctl.h>
#include <sys/un.h>

#include "hw/vfio-user/proxy.h"
#include "qapi/error.h"
#include "qemu/lockable.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"

#define STATE_WAIT_US   (10 * G_USEC_PER_SEC)
#define TRAP_TIMEOUT_US (30 * G_USEC_PER_SEC)

typedef struct {
    char *dir;
    char *path;
    int listen_fd;
    int server_fd;
    VFIODevice vbasedev;
    VFIOUserProxy *proxy;
} ProxyFixture;

static void unexpected_request(void *opaque, VFIOUserMsg *msg)
{
    g_assert_not_reached();
}

static void fixture_connect(ProxyFixture *f)
{
    struct sockaddr_un un = { .sun_family = AF_UNIX };
    SocketAddress addr = { .type = SOCKET_ADDRESS_TYPE_UNIX };
    GError *gerr = NULL;

    memset(f, 0, sizeof(*f));
    f->dir = g_dir_make_tmp("vfio-user-proxy-XXXXXX", &gerr);
    g_assert_no_error(gerr);
    f->path = g_build_filename(f->dir, "sock", NULL);
    g_assert_cmpuint(strlen(f->path), <, sizeof(un.sun_path));
    strcpy(un.sun_path, f->path);

    f->listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    g_assert_cmpint(f->listen_fd, >=, 0);
    g_assert_cmpint(bind(f->listen_fd, (struct sockaddr *)&un, sizeof(un)),
                    ==, 0);
    g_assert_cmpint(listen(f->listen_fd, 1), ==, 0);

    addr.u.q_unix.path = f->path;
    f->proxy = vfio_user_connect_dev(&addr, &error_abort);
    g_assert_nonnull(f->proxy);

    f->server_fd = accept(f->listen_fd, NULL, NULL);
    g_assert_cmpint(f->server_fd, >=, 0);

    f->vbasedev.proxy = f->proxy;
    vfio_user_set_handler(&f->vbasedev, unexpected_request, NULL);
}

static void fixture_teardown(ProxyFixture *f)
{
    vfio_user_disconnect(f->proxy);
    close(f->listen_fd);
    unlink(f->path);
    rmdir(f->dir);
    g_free(f->path);
    g_free(f->dir);
}

/* Client -> server request that expects a reply (like a DMA map). */
static void send_request(ProxyFixture *f)
{
    VFIOUserHdr *hdr = g_new0(VFIOUserHdr, 1);

    vfio_user_request_msg(hdr, VFIO_USER_DEVICE_RESET, sizeof(*hdr), 0);
    g_assert_true(vfio_user_send_nowait(f->proxy, hdr, NULL, 0,
                                        &error_abort));
}

static int server_unread_bytes(ProxyFixture *f)
{
    int n = -1;

    g_assert_cmpint(ioctl(f->server_fd, FIONREAD, &n), ==, 0);
    return n;
}

static enum proxy_state wait_proxy_left_connected(ProxyFixture *f)
{
    gint64 deadline = g_get_monotonic_time() + STATE_WAIT_US;
    enum proxy_state state;

    for (;;) {
        WITH_QEMU_LOCK_GUARD(&f->proxy->lock) {
            state = f->proxy->state;
        }
        if (state != VFIO_PROXY_CONNECTED ||
            g_get_monotonic_time() > deadline) {
            return state;
        }
        g_usleep(1000);
    }
}

/*
 * Server closes while the client's request is still unread in the
 * server's receive queue: the client's next header read fails with
 * ECONNRESET.
 */
static void run_server_close_unread(void)
{
    ProxyFixture f;

    fixture_connect(&f);
    send_request(&f);
    g_assert_cmpint(server_unread_bytes(&f), ==, sizeof(VFIOUserHdr));

    close(f.server_fd);

    g_assert_cmpint(wait_proxy_left_connected(&f), ==, VFIO_PROXY_ERROR);
    fixture_teardown(&f);
}

/* Control: server drains the request first, then closes -> EOF. */
static void run_server_close_drained(void)
{
    ProxyFixture f;
    VFIOUserHdr got;

    fixture_connect(&f);
    send_request(&f);
    g_assert_cmpint(read(f.server_fd, &got, sizeof(got)), ==, sizeof(got));
    g_assert_cmpuint(got.command, ==, VFIO_USER_DEVICE_RESET);
    g_assert_cmpint(server_unread_bytes(&f), ==, 0);

    close(f.server_fd);

    g_assert_cmpint(wait_proxy_left_connected(&f), ==, VFIO_PROXY_ERROR);
    fixture_teardown(&f);
}

static void test_server_close_unread(void)
{
    if (g_test_subprocess()) {
        run_server_close_unread();
        return;
    }
    g_test_trap_subprocess(NULL, TRAP_TIMEOUT_US, 0);
    g_test_trap_assert_passed();
    g_test_trap_assert_stderr(
        "*failed to read header: Unable to read from socket: "
        "Connection reset by peer*");
    g_test_trap_assert_stderr_unmatched("*EOF*");
}

static void test_server_close_drained(void)
{
    if (g_test_subprocess()) {
        run_server_close_drained();
        return;
    }
    g_test_trap_subprocess(NULL, TRAP_TIMEOUT_US, 0);
    g_test_trap_assert_passed();
    g_test_trap_assert_stderr("*failed to read header: EOF*");
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    module_call_init(MODULE_INIT_QOM);
    qemu_init_main_loop(&error_abort);

    g_test_add_func("/vfio-user/proxy/recv-hdr/server-close-unread",
                    test_server_close_unread);
    g_test_add_func("/vfio-user/proxy/recv-hdr/server-close-drained",
                    test_server_close_drained);

    return g_test_run();
}
