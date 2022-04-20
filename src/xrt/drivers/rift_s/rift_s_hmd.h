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
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "rift_s.h"
#include "rift_s_protocol.h"
#include "rift_s_firmware.h"

/* Oculus Rift S HMD Internal Interface */
#ifndef RIFT_S_HMD_H
#define RIFT_S_HMD_H

struct rift_s_hmd
{
	struct xrt_device base;

	struct rift_s_system *sys;

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
};

struct rift_s_hmd *
rift_s_hmd_create(struct rift_s_system *sys);
void
rift_s_hmd_handle_report(struct rift_s_hmd *hmd, rift_s_hmd_report_t *report);

#endif
