/*
 * vfio-user specific definitions.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_VFIO_USER_CONTAINER_H
#define HW_VFIO_USER_CONTAINER_H


#include "hw/vfio/vfio-container.h"
#include "hw/vfio-user/proxy.h"

/* MMU container sub-class for vfio-user. */
struct VFIOUserContainer {
    VFIOContainer parent_obj;

    VFIOUserProxy *proxy;
};

OBJECT_DECLARE_SIMPLE_TYPE(VFIOUserContainer, VFIO_IOMMU_USER);

/*
 * After a reconnect, the server's DMA table is empty.  Re-walk the guest
 * address space and re-send every DMA_MAP.  Runs on the main loop, BQL held.
 */
bool vfio_user_container_replay_maps(VFIOUserContainer *container, Error **errp);

#endif /* HW_VFIO_USER_CONTAINER_H */
