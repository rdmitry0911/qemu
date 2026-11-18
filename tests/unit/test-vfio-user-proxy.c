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
 * Contract VFIO-USER-PROXY-DISCONNECT-02 (requests in flight when the
 * server goes away), docs/interop/vfio-user.rst "Server Disconnection": the
 * client cannot tell a restart from a shutdown and resets the device; the
 * guest's error handling drives recovery.  "Request Retry and Response
 * Timeout": failing to send because the socket is disconnected is a
 * different error from a reply timeout.
 * Expected: once the client sees the disconnect, a request waiting for its
 * reply, a request queued but not sent, a vfio_user_wait_reqs() caller and
 * any new request fail at once with a disconnect error (ECONNRESET), not
 * after the reply timeout (x-msg-timeout, 5000 ms by default).
 *
 * Contract VFIO-USER-PROXY-RECONNECT-03, docs/interop/vfio-user.rst
 * "Client Disconnection": a server drops the client's memory regions and IRQ
 * fds; "when a client reconnects, it will re-establish IRQ and client memory
 * mappings".  Expected with reconnect enabled: the client redials the same
 * socket when the server listens again, negotiates VERSION first (the
 * protocol requires VERSION as the first message), then lets the device
 * layer set the device up again (restore callback), after which requests
 * round-trip on the new connection.  If the restore callback reports an
 * incompatible device while the connection is up, the client drops it and
 * stops redialing.  With reconnect disabled the client never redials.
 *
 * Device-level restore (DMA map replay, device reset, IRQ replay in
 * hw/vfio-user/pci.c) needs a PCI device and is covered by the live check.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/un.h>

#include "hw/vfio-user/proxy.h"
#include "qapi/error.h"
#include "qemu/lockable.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"

#define STATE_WAIT_US   (10 * G_USEC_PER_SEC)
#define TRAP_TIMEOUT_US (30 * G_USEC_PER_SEC)
#define REPLY_WAIT_MS   5000                    /* x-msg-timeout default */
#define PROMPT_US       (G_USEC_PER_SEC / 2)    /* << REPLY_WAIT_MS */
#define RECONNECT_MS    20
#define REDIAL_WAIT_MS  5000

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

/*
 * Server-side helpers (the test thread plays the vfio-user server).
 */
static bool read_full(int fd, void *buf, size_t len)
{
    char *p = buf;

    while (len > 0) {
        ssize_t n = read(fd, p, len);

        if (n <= 0) {
            return false;
        }
        p += n;
        len -= n;
    }
    return true;
}

/* Read one client message; returns its header, payload into @buf. */
static bool server_read_msg(int fd, VFIOUserHdr *hdr, void *buf, size_t len)
{
    if (!read_full(fd, hdr, sizeof(*hdr))) {
        return false;
    }
    g_assert_cmpuint(hdr->size, >=, sizeof(*hdr));
    g_assert_cmpuint(hdr->size - sizeof(*hdr), <=, len);
    return read_full(fd, buf, hdr->size - sizeof(*hdr));
}

static void server_reply(int fd, VFIOUserHdr *req, const void *data,
                         size_t len)
{
    g_autofree char *buf = g_malloc0(sizeof(*req) + len);
    VFIOUserHdr *rep = (VFIOUserHdr *)buf;

    *rep = *req;
    rep->flags = VFIO_USER_REPLY;
    rep->size = sizeof(*rep) + len;
    rep->error_reply = 0;
    memcpy(buf + sizeof(*rep), data, len);
    g_assert_cmpint(write(fd, buf, rep->size), ==, rep->size);
}

/* Serve the VERSION handshake a (re)connecting client must start with. */
static bool server_version(int fd)
{
    static const char caps[] = "{}";
    char payload[4096];
    VFIOUserHdr hdr;
    struct {
        uint16_t major, minor;
        char caps[sizeof(caps)];
    } QEMU_PACKED ver = { VFIO_USER_MAJOR_VER, VFIO_USER_MINOR_VER, "{}" };

    if (!server_read_msg(fd, &hdr, payload, sizeof(payload))) {
        return false;
    }
    g_assert_cmpuint(hdr.command, ==, VFIO_USER_VERSION);
    server_reply(fd, &hdr, &ver, sizeof(ver));
    return true;
}

static bool listener_has_client(ProxyFixture *f, int timeout_ms)
{
    struct pollfd pfd = { .fd = f->listen_fd, .events = POLLIN };

    return poll(&pfd, 1, timeout_ms) == 1;
}

/* Run the main loop (BHs, timers) until *@flag or @timeout_ms passes. */
static void run_main_loop_until(bool *flag, int timeout_ms)
{
    gint64 deadline = g_get_monotonic_time() + timeout_ms * 1000LL;

    while (!qatomic_read(flag) && g_get_monotonic_time() < deadline) {
        main_loop_wait(true);
        g_usleep(1000);
    }
}

static void run_main_loop_for(int ms)
{
    bool never = false;

    run_main_loop_until(&never, ms);
}

/*
 * A request failed because of the disconnect: either the proxy completed it
 * with ECONNRESET, or refused it with a disconnect error.  A reply timeout
 * ("timed out waiting for reply", ETIMEDOUT) is the wrong outcome.
 */
static void assert_disconnect_failure(bool ok, VFIOUserHdr *hdr, Error *err)
{
    if (ok) {
        g_assert_true(hdr->flags & VFIO_USER_ERROR);
        g_assert_cmpuint(hdr->error_reply, ==, ECONNRESET);
    } else {
        g_assert_nonnull(err);
        g_assert_null(strstr(error_get_pretty(err), "timed out"));
        g_assert_nonnull(strstr(error_get_pretty(err), "disconnected"));
        error_free(err);
    }
}

typedef struct {
    ProxyFixture *f;
    bool drain;         /* read the request before closing */
} CloseCtx;

/* The server dies while the client's request waits for its reply. */
static gpointer server_close_on_request(gpointer opaque)
{
    CloseCtx *c = opaque;
    gint64 deadline = g_get_monotonic_time() + STATE_WAIT_US;
    VFIOUserHdr hdr;

    if (c->drain) {
        g_assert_true(read_full(c->f->server_fd, &hdr, sizeof(hdr)));
    } else {
        while (server_unread_bytes(c->f) < (int)sizeof(hdr)) {
            g_assert_cmpint(g_get_monotonic_time(), <, deadline);
            g_usleep(1000);
        }
    }
    close(c->f->server_fd);
    return NULL;
}

static void run_wait_fails_promptly(bool drain)
{
    ProxyFixture f;
    CloseCtx c = { &f, drain };
    VFIOUserHdr hdr;
    Error *err = NULL;
    GThread *th;
    gint64 t0, elapsed;
    bool ok;

    fixture_connect(&f);
    f.proxy->wait_time = REPLY_WAIT_MS;
    vfio_user_request_msg(&hdr, VFIO_USER_DEVICE_RESET, sizeof(hdr), 0);

    th = g_thread_new("server", server_close_on_request, &c);
    t0 = g_get_monotonic_time();
    ok = vfio_user_send_wait(f.proxy, &hdr, NULL, 0, &err);
    elapsed = g_get_monotonic_time() - t0;
    g_thread_join(th);

    g_printerr("send_wait across server close (%s): ok=%d elapsed_us=%"
               PRId64 "\n", drain ? "drained" : "unread", ok, elapsed);
    g_assert_cmpint(elapsed, <, PROMPT_US);
    assert_disconnect_failure(ok, &hdr, err);
    g_assert_cmpint(wait_proxy_left_connected(&f), ==, VFIO_PROXY_ERROR);
    fixture_teardown(&f);
}

/* After the loss, a new request must not wait for a reply that cannot come */
static void run_send_after_loss(bool queued)
{
    ProxyFixture f;
    VFIOUserHdr hdr;
    Error *err = NULL;
    gint64 t0, elapsed;
    bool ok;

    fixture_connect(&f);
    f.proxy->wait_time = REPLY_WAIT_MS;
    if (queued) {
        f.proxy->flags |= VFIO_PROXY_FORCE_QUEUED;   /* x-send-queued=on */
    }
    close(f.server_fd);
    g_assert_cmpint(wait_proxy_left_connected(&f), ==, VFIO_PROXY_ERROR);

    vfio_user_request_msg(&hdr, VFIO_USER_DEVICE_RESET, sizeof(hdr), 0);
    t0 = g_get_monotonic_time();
    ok = vfio_user_send_wait(f.proxy, &hdr, NULL, 0, &err);
    elapsed = g_get_monotonic_time() - t0;

    g_printerr("send_wait after loss (%s): ok=%d elapsed_us=%" PRId64 "\n",
               queued ? "queued" : "inline", ok, elapsed);
    g_assert_cmpint(elapsed, <, PROMPT_US);
    g_assert_false(ok);
    error_free(err);
    fixture_teardown(&f);
}

/* A nowait request (DMA map in a transaction) was in flight at the loss. */
static void run_wait_reqs_after_loss(void)
{
    ProxyFixture f;
    VFIOUserHdr got;
    gint64 t0, elapsed;

    fixture_connect(&f);
    f.proxy->wait_time = REPLY_WAIT_MS;
    send_request(&f);
    g_assert_true(read_full(f.server_fd, &got, sizeof(got)));
    close(f.server_fd);
    g_assert_cmpint(wait_proxy_left_connected(&f), ==, VFIO_PROXY_ERROR);

    t0 = g_get_monotonic_time();
    vfio_user_wait_reqs(f.proxy);
    elapsed = g_get_monotonic_time() - t0;

    g_printerr("wait_reqs after loss: elapsed_us=%" PRId64 "\n", elapsed);
    g_assert_cmpint(elapsed, <, PROMPT_US);
    fixture_teardown(&f);
}

typedef struct {
    ProxyFixture *f;
    bool compatible;        /* restore callback result */
    int requests;           /* requests to serve after VERSION */
    bool accepted;          /* the client redialed */
    bool served;            /* VERSION and all requests served */
    bool saw_eof;           /* client dropped the new connection */
    bool redialed_again;    /* client dialed once more after dropping */
    bool restored;          /* restore callback ran */
    int restore_calls;
    bool restore_request_ok;
} ReconnectCtx;

static gpointer server_restart(gpointer opaque)
{
    ReconnectCtx *c = opaque;
    char payload[4096];
    VFIOUserHdr hdr;
    int fd, i;

    /* the server process dies ... */
    close(c->f->server_fd);
    c->f->server_fd = -1;

    /* ... and listens again on the same socket */
    if (!listener_has_client(c->f, REDIAL_WAIT_MS)) {
        return NULL;
    }
    fd = accept(c->f->listen_fd, NULL, NULL);
    g_assert_cmpint(fd, >=, 0);
    c->f->server_fd = fd;
    qatomic_set(&c->accepted, true);

    if (!server_version(fd)) {
        return NULL;
    }
    for (i = 0; i < c->requests; i++) {
        if (!server_read_msg(fd, &hdr, payload, sizeof(payload))) {
            return NULL;
        }
        server_reply(fd, &hdr, NULL, 0);
    }
    qatomic_set(&c->served, true);

    if (!c->compatible) {
        qatomic_set(&c->saw_eof, read(fd, payload, 1) == 0);
        qatomic_set(&c->redialed_again,
                    listener_has_client(c->f, 10 * RECONNECT_MS));
    }
    return NULL;
}

/* Device-layer stand-in: one request on the new connection, then verdict. */
static bool test_restore(void *opaque, Error **errp)
{
    ReconnectCtx *c = opaque;
    VFIOUserHdr hdr;

    c->restore_calls++;
    vfio_user_request_msg(&hdr, VFIO_USER_DEVICE_RESET, sizeof(hdr), 0);
    c->restore_request_ok =
        vfio_user_send_wait(c->f->proxy, &hdr, NULL, 0, &error_abort) &&
        !(hdr.flags & VFIO_USER_ERROR);
    qatomic_set(&c->restored, true);

    if (!c->compatible) {
        error_setg(errp, "test: incompatible device");
        return false;
    }
    return true;
}

static void run_reconnect(bool compatible)
{
    ProxyFixture f;
    ReconnectCtx c = { .f = &f, .compatible = compatible,
                       .requests = compatible ? 2 : 1 };
    VFIOUserHdr hdr;
    GThread *th;
    enum proxy_state state;

    fixture_connect(&f);
    f.proxy->wait_time = REPLY_WAIT_MS;
    vfio_user_set_reconnect(f.proxy, RECONNECT_MS, test_restore, &c);

    th = g_thread_new("server", server_restart, &c);
    run_main_loop_until(&c.restored, REDIAL_WAIT_MS + 1000);

    g_assert_true(qatomic_read(&c.accepted));
    g_assert_true(qatomic_read(&c.restored));
    g_assert_cmpint(c.restore_calls, ==, 1);
    g_assert_true(c.restore_request_ok);

    if (compatible) {
        /* the new connection carries requests */
        WITH_QEMU_LOCK_GUARD(&f.proxy->lock) {
            state = f.proxy->state;
        }
        g_assert_cmpint(state, ==, VFIO_PROXY_CONNECTED);
        vfio_user_request_msg(&hdr, VFIO_USER_DEVICE_RESET, sizeof(hdr), 0);
        g_assert_true(vfio_user_send_wait(f.proxy, &hdr, NULL, 0,
                                          &error_abort));
        g_assert_false(hdr.flags & VFIO_USER_ERROR);
        g_thread_join(th);
        g_assert_true(c.served);
    } else {
        g_thread_join(th);
        g_assert_true(c.served);
        g_assert_true(c.saw_eof);
        g_assert_false(c.redialed_again);
        run_main_loop_for(10 * RECONNECT_MS);
        g_assert_cmpint(c.restore_calls, ==, 1);
        g_assert_cmpint(wait_proxy_left_connected(&f), ==, VFIO_PROXY_ERROR);
    }
    fixture_teardown(&f);
}

/* Control: without reconnect the client never dials again. */
static void run_reconnect_disabled(void)
{
    ProxyFixture f;

    fixture_connect(&f);
    close(f.server_fd);
    g_assert_cmpint(wait_proxy_left_connected(&f), ==, VFIO_PROXY_ERROR);
    run_main_loop_for(10 * RECONNECT_MS);
    g_assert_false(listener_has_client(&f, 0));
    g_assert_cmpint(wait_proxy_left_connected(&f), ==, VFIO_PROXY_ERROR);
    fixture_teardown(&f);
}

#define SUBPROCESS_TEST(name, body)                     \
    static void name(void)                              \
    {                                                   \
        if (g_test_subprocess()) {                      \
            body;                                       \
            return;                                     \
        }                                               \
        g_test_trap_subprocess(NULL, TRAP_TIMEOUT_US,   \
                               G_TEST_SUBPROCESS_INHERIT_STDERR); \
        g_test_trap_assert_passed();                    \
    }

SUBPROCESS_TEST(test_wait_fails_unread, run_wait_fails_promptly(false))
SUBPROCESS_TEST(test_wait_fails_drained, run_wait_fails_promptly(true))
SUBPROCESS_TEST(test_send_after_loss_queued, run_send_after_loss(true))
SUBPROCESS_TEST(test_send_after_loss_inline, run_send_after_loss(false))
SUBPROCESS_TEST(test_wait_reqs_after_loss, run_wait_reqs_after_loss())
SUBPROCESS_TEST(test_reconnect, run_reconnect(true))
SUBPROCESS_TEST(test_reconnect_incompatible, run_reconnect(false))
SUBPROCESS_TEST(test_reconnect_disabled, run_reconnect_disabled())

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    /* as in QEMU (os_setup_signal_handling): a dead peer is EPIPE, not fatal */
    signal(SIGPIPE, SIG_IGN);
    module_call_init(MODULE_INIT_QOM);
    qemu_init_main_loop(&error_abort);

    g_test_add_func("/vfio-user/proxy/recv-hdr/server-close-unread",
                    test_server_close_unread);
    g_test_add_func("/vfio-user/proxy/recv-hdr/server-close-drained",
                    test_server_close_drained);
    g_test_add_func("/vfio-user/proxy/disconnect/wait-fails-unread",
                    test_wait_fails_unread);
    g_test_add_func("/vfio-user/proxy/disconnect/wait-fails-drained",
                    test_wait_fails_drained);
    g_test_add_func("/vfio-user/proxy/disconnect/send-after-loss-queued",
                    test_send_after_loss_queued);
    g_test_add_func("/vfio-user/proxy/disconnect/send-after-loss-inline",
                    test_send_after_loss_inline);
    g_test_add_func("/vfio-user/proxy/disconnect/wait-reqs-after-loss",
                    test_wait_reqs_after_loss);
    g_test_add_func("/vfio-user/proxy/reconnect/redial-restore",
                    test_reconnect);
    g_test_add_func("/vfio-user/proxy/reconnect/incompatible-stops",
                    test_reconnect_incompatible);
    g_test_add_func("/vfio-user/proxy/reconnect/disabled",
                    test_reconnect_disabled);

    return g_test_run();
}
