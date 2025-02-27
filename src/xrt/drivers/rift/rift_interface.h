// Copyright 2025, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to Oculus Rift driver code.
 * @author Beyley Cardellion <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#pragma once

#include "xrt/xrt_device.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_prober.h"

#include "os/os_hid.h"

#include "util/u_device.h"
#include "util/u_logging.h"

#include <stdlib.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

enum rift_variant {
    RIFT_VARIANT_DK2,
};

#define OCULUS_VR_VID 0x2833

#define OCULUS_DK2_PID 0x0021

/*!
 * Probing function for Oculus Rift devices.
 *
 * @ingroup drv_rift
 * @see xrt_prober_found_func_t
 */
int
rift_found(struct xrt_prober *xp,
           struct xrt_prober_device **devices,
           size_t device_count,
           size_t index,
           cJSON *attached_data,
           struct xrt_device **out_xdev);

/*!
 * A rift HMD device.
 *
 * @implements xrt_device
 */
struct rift_hmd
{
	struct xrt_device base;

	struct xrt_pose pose;

	enum u_logging_level log_level;

	// has built-in mutex so thread safe
	struct m_relation_history *relation_hist;

    int64_t last_keepalive_time;
    enum rift_variant variant;
    struct os_hid_device *hid_dev;
};

struct rift_hmd *rift_hmd_create(struct os_hid_device *dev, enum rift_variant variant, char *device_name, char *serial_number);

/*!
 * @dir drivers/rift
 *
 * @brief @ref drv_rift files.
 */

#ifdef __cplusplus
}
#endif
