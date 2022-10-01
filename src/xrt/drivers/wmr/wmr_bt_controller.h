// Copyright 2020-2021, N Madsen.
// Copyright 2020-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Driver interface for Bluetooth based WMR motion controllers.
 * Note: Only tested with HP Reverb (G1) controllers that are manually
 * paired to a non hmd-integrated, generic BT usb adapter.
 * @author Nis Madsen <nima_zero_one@protonmail.com>
 * @ingroup drv_wmr
 */
#pragma once

#include "os/os_threading.h"
#include "math/m_imu_3dof.h"
#include "util/u_logging.h"
#include "xrt/xrt_device.h"

#include "wmr_controller_protocol.h"
#include "wmr_config.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Indices in input list of each input.
 */
enum wmr_controller_input_id
{
	/* Common inputs */
	WMR_CONTROLLER_INPUT_ID_AIM_POSE = 0,
	WMR_CONTROLLER_INPUT_ID_GRIP_POSE,
	WMR_CONTROLLER_INPUT_ID_MENU_CLICK,
	WMR_CONTROLLER_INPUT_ID_WIN_CLICK,
	WMR_CONTROLLER_INPUT_ID_SQUEEZE_CLICK,
	WMR_CONTROLLER_INPUT_ID_TRIGGER_VALUE,
	WMR_CONTROLLER_INPUT_ID_THUMBSTICK_CLICK,
	WMR_CONTROLLER_INPUT_ID_THUMBSTICK,
	WMR_CONTROLLER_INPUT_ID_CLIFFHOUSE_CLICK,

	/* Original WMR controller specific */
	WMR_CONTROLLER_INPUT_ID_TRACKPAD_CLICK,
	WMR_CONTROLLER_INPUT_ID_TRACKPAD_TOUCH,
	WMR_CONTROLLER_INPUT_ID_TRACKPAD,

	/* Reverb G2 Oculus-touch style controller specific */
	WMR_CONTROLLER_INPUT_ID_A_CLICK,
	WMR_CONTROLLER_INPUT_ID_B_CLICK,
	WMR_CONTROLLER_INPUT_ID_X_CLICK,
	WMR_CONTROLLER_INPUT_ID_Y_CLICK,

	WMR_CONTROLLER_INPUT_ID_MAX,
};

/*!
 * Known controller variants
 */
enum wmr_controller_variant
{
	WMR_CONTROLLER_VARIANT_ORIGINAL,
	WMR_CONTROLLER_VARIANT_G2,
};

/*!
 * A Bluetooth connected WMR Controller device, representing just a single controller.
 *
 * @ingroup drv_wmr
 * @implements xrt_device
 */
struct wmr_bt_controller
{
	struct xrt_device base;

	bool standalone_device;

	enum u_logging_level log_level;
	struct os_hid_device *controller_hid;

	/* firmware configuration block */
	struct wmr_controller_config config;

	enum wmr_controller_variant variant;

	struct os_mutex lock;

	//! The last decoded package of IMU and button data
	struct wmr_controller_input input;
	//! Time of last IMU sample, in CPU time.
	uint64_t last_imu_timestamp_ns;
	//! Main fusion calculator.
	struct m_imu_3dof fusion;
	//! The last angular velocity from the IMU, for prediction.
	struct xrt_vec3 last_angular_velocity;

	/* Thread for direct Bluetooth connections,
	 * not for tunneled */
	struct os_thread_helper controller_thread;
};


struct xrt_device *
wmr_bt_controller_create(struct os_hid_device *controller_hid,
                         enum xrt_device_type controller_type,
                         uint16_t vid,
                         uint16_t pid,
                         enum u_logging_level log_level);

struct wmr_bt_controller *
wmr_controller_create_tunnelled(struct os_hid_device *controller_hid,
                                enum xrt_device_type controller_type,
                                uint16_t vid,
                                uint16_t pid,
                                enum u_logging_level log_level);

void
wmr_controller_handle_sensors_packet(struct wmr_bt_controller *d, uint64_t now_ns, unsigned char *buffer, int size);

#ifdef __cplusplus
}
#endif
