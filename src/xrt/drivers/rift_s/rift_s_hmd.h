/*
 * Copyright 2013, Fredrik Hultin.
 * Copyright 2013, Jakob Bornecrantz.
 * Copyright 2016 Philipp Zabel
 * Copyright 2019-2022 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

/*!
 * @file
 * @brief  Interface to the Oculus Rift S HMD driver code.
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */

#pragma once

#include "math/m_imu_3dof.h"

#include "os/os_threading.h"

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

/* Oculus Rift S Driver Internal Interface */
#ifndef RIFT_S_HMD_H
#define RIFT_S_HMD_H

#include "rift_s_protocol.h"
#include "rift_s_firmware.h"
#include "rift_s_controller.h"
#include "rift_s_radio.h"

#define MAX_TRACKED_DEVICES 2

#define HMD_HID 0
#define STATUS_HID 1
#define CONTROLLER_HID 2

/* Structure to track online devices and type */
struct rift_s_hmd_tracked_device
{
	uint64_t device_id;
	uint32_t device_type;
};

struct rift_s_hmd
{
	struct xrt_device base;
	struct xrt_reference ref;

	/* Packet processing thread */
	struct os_thread_helper oth;
	struct os_hid_device *handles[3];
	uint64_t last_keep_alive;

	/* 3DOF fusion */
	struct os_mutex mutex;
	uint32_t last_imu_timestamp32; /* 32-bit µS device timestamp */
	uint64_t last_imu_timestamp_ns;
	struct m_imu_3dof fusion;
	struct xrt_pose pose;
	struct xrt_vec3 raw_mag, raw_accel, raw_gyro;

	/* Auxilliary state */
	float temperature;
	bool display_on;

	/* Configuration / calibration info */
	rift_s_device_info_t device_info;
	rift_s_imu_config_t imu_config;
	rift_s_imu_calibration imu_calibration;

	/* state tracking for tracked devices on our radio link */
	int num_active_tracked_devices;
	struct rift_s_hmd_tracked_device tracked_device[MAX_TRACKED_DEVICES];

	/* Radio comms manager */
	rift_s_radio_state radio_state;

	/* Controller devices */
	struct rift_s_controller *controllers[MAX_TRACKED_DEVICES];
};

struct rift_s_hmd *
rift_s_hmd_create(struct os_hid_device *hid_hmd,
                  struct os_hid_device *hid_status,
                  struct os_hid_device *hid_controllers);

struct xrt_device *
rift_s_hmd_get_controller(struct rift_s_hmd *hmd_dev, int index);

void
rift_s_hmd_reference(struct rift_s_hmd **dst, struct rift_s_hmd *src);

#endif
