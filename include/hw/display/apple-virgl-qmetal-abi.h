/*
 * Narrow C ABI for the AppleVirgl QMetal display pump.
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qmu_vulkan.h is intentionally not included by C translation units: some
 * public declarations use C++ default arguments and nullptr.  Keep only the
 * callback-safe display-pump surface here.
 */

#ifndef HW_DISPLAY_APPLE_VIRGL_QMETAL_ABI_H
#define HW_DISPLAY_APPLE_VIRGL_QMETAL_ABI_H

struct qmu_vulkan_ctx;

void qmu_vk_consume_current_frame_signal(struct qmu_vulkan_ctx *ctx);
int qmu_vk_capture_display_frame_request(struct qmu_vulkan_ctx *ctx);
int qmu_vk_submit_captured_display_frame(struct qmu_vulkan_ctx *ctx);

#endif /* HW_DISPLAY_APPLE_VIRGL_QMETAL_ABI_H */
