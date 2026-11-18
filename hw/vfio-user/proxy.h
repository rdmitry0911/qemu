#ifndef VFIO_USER_PROXY_H
#define VFIO_USER_PROXY_H

/*
 * vfio protocol over a UNIX socket.
 *
 * Copyright © 2018, 2021 Oracle and/or its affiliates.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "io/channel.h"
#include "io/channel-socket.h"

#include "qemu/queue.h"
#include "qemu/sockets.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
#include "hw/vfio/vfio-device.h"
#include "hw/vfio-user/protocol.h"

typedef struct {
    int send_fds;
    int recv_fds;
    int *fds;
} VFIOUserFDs;

enum msg_type {
    VFIO_MSG_NONE,
    VFIO_MSG_ASYNC,
    VFIO_MSG_WAIT,
    VFIO_MSG_NOWAIT,
    VFIO_MSG_REQ,
};

typedef struct VFIOUserMsg {
    QTAILQ_ENTRY(VFIOUserMsg) next;
    VFIOUserHdr *hdr;
    VFIOUserFDs *fds;
    uint32_t rsize;
    uint32_t id;
    QemuCond cv;
    bool complete;
    bool pending;
    enum msg_type type;
} VFIOUserMsg;


enum proxy_state {
    VFIO_PROXY_CONNECTED = 1,
    VFIO_PROXY_ERROR = 2,
    VFIO_PROXY_CLOSING = 3,
    VFIO_PROXY_CLOSED = 4,
};

typedef QTAILQ_HEAD(VFIOUserMsgQ, VFIOUserMsg) VFIOUserMsgQ;

typedef struct VFIOUserProxy {
    QLIST_ENTRY(VFIOUserProxy) next;
    char *sockname;
    struct QIOChannel *ioc;
    void (*request)(void *opaque, VFIOUserMsg *msg);
    void *req_arg;
    uint64_t max_xfer_size;
    uint64_t max_send_fds;
    uint64_t max_dma;
    uint64_t dma_pgsizes;
    uint64_t max_bitmap;
    uint64_t migr_pgsize;
    int flags;
    uint32_t wait_time;
    QemuCond close_cv;
    AioContext *ctx;
    QEMUBH *req_bh;
    bool async_ops;

    /*
     * Client reconnect (docs/interop/vfio-user.rst "Server Disconnection"):
     * when the server drops the socket the proxy fails everything in flight
     * and moves to VFIO_PROXY_ERROR.  If reconnect_ms is non-zero it then
     * redials the saved address every reconnect_ms on the main loop, redoes
     * the version handshake and calls restore_fn to set the device up again.
     * Only changed when BQL is held.
     */
    SocketAddress *sockaddr;    /* saved address to redial */
    uint32_t reconnect_ms;      /* 0: stay in VFIO_PROXY_ERROR */
    QEMUBH *lost_bh;            /* main loop: connection lost */
    QEMUTimer *reconnect_timer; /* main loop: next redial */
    bool (*restore_fn)(void *opaque, Error **errp);
    void *restore_opaque;
    bool reconnecting;          /* a DMA map replay is in progress */

    /*
     * above only changed when BQL is held
     * below are protected by per-proxy lock
     */
    QemuMutex lock;
    VFIOUserMsgQ free;
    VFIOUserMsgQ pending;
    VFIOUserMsgQ incoming;
    VFIOUserMsgQ outgoing;
    VFIOUserMsg *last_nowait;
    VFIOUserMsg *part_recv;
    size_t recv_left;
    VFIOUserWRMulti *wr_multi;
    int num_outgoing;
    enum proxy_state state;
} VFIOUserProxy;

/* VFIOProxy flags */
#define VFIO_PROXY_CLIENT        BIT(0)
#define VFIO_PROXY_FORCE_QUEUED  BIT(1)
#define VFIO_PROXY_NO_POST       BIT(2)
#define VFIO_PROXY_USE_MULTI     BIT(3)

/* coalescing high and low water marks for VFIOProxy num_outgoing */
#define VFIO_USER_OUT_HIGH       1024
#define VFIO_USER_OUT_LOW        128

typedef struct VFIODevice VFIODevice;

VFIOUserProxy *vfio_user_connect_dev(SocketAddress *addr, Error **errp);
void vfio_user_disconnect(VFIOUserProxy *proxy);
void vfio_user_set_handler(VFIODevice *vbasedev,
                           void (*handler)(void *opaque, VFIOUserMsg *msg),
                           void *reqarg);
bool vfio_user_validate_version(VFIOUserProxy *proxy, Error **errp);

/*
 * Enable (interval_ms > 0) or disable (0) client reconnect.  @restore_fn runs
 * on the main loop with BQL held after a successful redial and handshake; it
 * must set the device up on the new connection (DMA maps, reset, IRQs).  It
 * returns false with @errp set if that failed; if the connection is still up
 * at that point the device is treated as incompatible and reconnect stops.
 */
void vfio_user_set_reconnect(VFIOUserProxy *proxy, uint32_t interval_ms,
                             bool (*restore_fn)(void *opaque, Error **errp),
                             void *opaque);

VFIOUserFDs *vfio_user_getfds(int numfds);
void vfio_user_putfds(VFIOUserMsg *msg);

void vfio_user_disable_posted_writes(VFIOUserProxy *proxy);

void vfio_user_request_msg(VFIOUserHdr *hdr, uint16_t cmd,
                           uint32_t size, uint32_t flags);
void vfio_user_wait_reqs(VFIOUserProxy *proxy);
bool vfio_user_send_wait(VFIOUserProxy *proxy, VFIOUserHdr *hdr,
                         VFIOUserFDs *fds, int rsize, Error **errp);
bool vfio_user_send_nowait(VFIOUserProxy *proxy, VFIOUserHdr *hdr,
                           VFIOUserFDs *fds, int rsize, Error **errp);
bool vfio_user_send_async(VFIOUserProxy *proxy, VFIOUserHdr *hdr,
                          VFIOUserFDs *fds, Error **errp);

void vfio_user_send_reply(VFIOUserProxy *proxy, VFIOUserHdr *hdr, int size);
void vfio_user_send_error(VFIOUserProxy *proxy, VFIOUserHdr *hdr, int error);

void vfio_user_flush_multi(VFIOUserProxy *proxy);
void vfio_user_create_multi(VFIOUserProxy *proxy);
void vfio_user_add_multi(VFIOUserProxy *proxy, uint8_t index,
                         off_t offset, uint32_t count, void *data);

#endif /* VFIO_USER_PROXY_H */
