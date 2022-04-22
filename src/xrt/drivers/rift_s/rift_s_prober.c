// Copyright 2019, Collabora, Ltd.
// Copyright 2022, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Oculus Rift S prober code.
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */

#include <stdio.h>
#include <stdlib.h>
#include <wchar.h>

#include "os/os_hid.h"

#include "xrt/xrt_prober.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_trace_marker.h"

#include "rift_s_interface.h"
#include "rift_s.h"
#include "rift_s_hmd.h"

enum u_logging_level rift_s_log_level;

/* Interfaces for the various reports / HID controls */
#define RIFT_S_INTF_HMD 6
#define RIFT_S_INTF_STATUS 7
#define RIFT_S_INTF_CONTROLLERS 8

/*
 *
 * Defines & structs.
 *
 */

DEBUG_GET_ONCE_LOG_OPTION(rift_s_log, "RIFT_S_LOG", U_LOGGING_WARN)

int
rift_s_found(struct xrt_prober *xp,
             struct xrt_prober_device **devices,
             size_t device_count,
             size_t index,
             cJSON *attached_data,
             struct xrt_device **out_xdev)
{
	DRV_TRACE_MARKER();

	rift_s_log_level = debug_get_log_option_rift_s_log();
	struct xrt_prober_device *dev_hmd = devices[index];
	int num_devices = 0;

	struct os_hid_device *hid_hmd = NULL;
	int result = xrt_prober_open_hid_interface(xp, dev_hmd, RIFT_S_INTF_HMD, &hid_hmd);
	if (result != 0) {
		RIFT_S_ERROR("Failed to open Rift S HMD interface");
		return -1;
	}

	struct os_hid_device *hid_status = NULL;
	result = xrt_prober_open_hid_interface(xp, dev_hmd, RIFT_S_INTF_STATUS, &hid_status);
	if (result != 0) {
		os_hid_destroy(hid_hmd);
		RIFT_S_ERROR("Failed to open Rift S status interface");
		return -1;
	}

	struct os_hid_device *hid_controllers = NULL;
	result = xrt_prober_open_hid_interface(xp, dev_hmd, RIFT_S_INTF_CONTROLLERS, &hid_controllers);
	if (result != 0) {
		os_hid_destroy(hid_hmd);
		os_hid_destroy(hid_status);
		RIFT_S_ERROR("Failed to open Rift S controllers interface");
		return -1;
	}

	unsigned char hmd_serial_no[XRT_DEVICE_NAME_LEN];
	result = xrt_prober_get_string_descriptor(xp, dev_hmd, XRT_PROBER_STRING_SERIAL_NUMBER, hmd_serial_no,
	                                          XRT_DEVICE_NAME_LEN);
	if (result < 0) {
		RIFT_S_WARN("Could not read Rift S serial number from USB");
		snprintf((char *)hmd_serial_no, XRT_DEVICE_NAME_LEN, "Unknown");
	}

	struct rift_s_system *sys = rift_s_system_create(hmd_serial_no, hid_hmd, hid_status, hid_controllers);
	if (sys == NULL) {
		RIFT_S_ERROR("Failed to initialise Oculus Rift S driver");
		return -1;
	}

	out_xdev[num_devices++] = rift_s_system_get_hmd(sys);
	out_xdev[num_devices++] = rift_s_system_get_controller(sys, 0);
	out_xdev[num_devices++] = rift_s_system_get_controller(sys, 1);

	return num_devices;
}
