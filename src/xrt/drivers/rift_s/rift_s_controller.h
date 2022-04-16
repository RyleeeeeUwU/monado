/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#ifndef RIFT_S_CONTROLLER_H
#define RIFT_S_CONTROLLER_H

#include "math/m_imu_3dof.h"

#include "xrt/xrt_device.h"

#define MAX_LOG_SIZE 1024

typedef struct
{
	uint16_t accel_limit;
	uint16_t gyro_limit;
	uint16_t accel_hz;
	uint16_t gyro_hz;

	float accel_scale;
	float gyro_scale;
} rift_s_controller_config;

struct rift_s_controller
{
	struct xrt_device base;

	struct xrt_pose pose;

	/* The HMD this controller belongs to / receives reports from */
	struct rift_s_hmd *hmd;

	uint64_t device_id;
	uint32_t device_type;

	/* 0x04 = new log line
	 * 0x02 = parity bit, toggles each line when receiving log chars
	 * other bits, unknown */
	uint8_t log_flags;
	int log_bytes;
	uint8_t log[MAX_LOG_SIZE];

	bool imu_time_valid;
	uint32_t imu_timestamp32;
	uint64_t imu_timestamp;
	uint16_t imu_unknown_varying2;
	int16_t raw_accel[3];
	int16_t raw_gyro[3];

	/* 0x8, 0x0c 0x0d or 0xe block */
	uint8_t mask08;
	uint8_t buttons;
	uint8_t fingers;
	uint8_t mask0e;

	uint16_t trigger;
	uint16_t grip;

	int16_t joystick_x;
	int16_t joystick_y;

	uint8_t capsense_a_x;
	uint8_t capsense_b_y;
	uint8_t capsense_joystick;
	uint8_t capsense_trigger;

	uint8_t extra_bytes_len;
	uint8_t extra_bytes[48];

	bool reading_config;
	bool have_config;
	rift_s_controller_config config;

	bool reading_calibration;
	bool have_calibration;
	rift_s_controller_imu_calibration calibration;

	struct xrt_vec3 accel;
	struct xrt_vec3 gyro;
	struct xrt_vec3 mag;
	struct m_imu_3dof fusion;
};

struct rift_s_controller *
rift_s_controller_create(struct rift_s_hmd *hmd, enum xrt_device_type device_type);

void
rift_s_controller_update_configuration(struct rift_s_controller *ctrl);
bool
rift_s_controller_handle_report(struct rift_s_controller *ctrl, rift_s_controller_report_t *report);

#endif
