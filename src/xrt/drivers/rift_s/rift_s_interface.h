// Copyright 2020-2021, Collabora, Ltd.
// Copyright 2022 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to rift_s driver.
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_rift_s Oculus Rift S driver
 * @ingroup drv
 *
 * @brief Driver for the Oculus Rift S and touch controllers
 *
 */

#define OCULUS_VR_INC_VID 0x2833
#define OCULUS_RIFT_S_PID 0x0051

/*!
 * Probing function for Oculus Rift S HMD.
 *
 * @ingroup drv_rift_s
 * @see xrt_prober_found_function_t
 */
int
rift_s_found(struct xrt_prober *xp,
             struct xrt_prober_device **devices,
             size_t device_count,
             size_t index,
             cJSON *attached_data,
             struct xrt_device **out_xdev);

/*!
 * @dir drivers/rift_s
 *
 * @brief @ref drv_rift_s files.
 */

#ifdef __cplusplus
}
#endif
