/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
/*!
 * @file
 * @brief  Oculus Rift S firmware parsing interface
 *
 * Functions for parsing JSON configuration from the HMD
 * and Touch Controller firmware.
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */
#ifndef __RTFT_S_FIRMWARE__
#define __RTFT_S_FIRMWARE__

#include "math/m_mathinclude.h"
#include "math/m_api.h"

#include "rift_s.h"

typedef enum
{
	RIFT_S_FIRMWARE_BLOCK_SERIAL_NUM = 0x0B,
	RIFT_S_FIRMWARE_BLOCK_THRESHOLD = 0xD,
	RIFT_S_FIRMWARE_BLOCK_IMU_CALIB = 0xE,
	RIFT_S_FIRMWARE_BLOCK_CAMERA_CALIB = 0xF,
	RIFT_S_FIRMWARE_BLOCK_DISPLAY_COLOR_CALIB = 0x10,
	RIFT_S_FIRMWARE_BLOCK_LENS_CALIB = 0x12
} rift_s_firmware_block;

typedef struct
{
	struct xrt_matrix_4x4 imu_to_device_transform;

	struct
	{
		struct xrt_matrix_3x3 rectification;
		struct xrt_vec3 offset;
	} gyro;

	struct
	{
		struct xrt_matrix_3x3 rectification;
		struct xrt_vec3 offset_at_0C;
		struct xrt_vec3 temp_coeff;
	} accel;
} rift_s_imu_calibration;

/* Rift S controller LED entry */
typedef struct
{
	// Relative position in metres
	struct xrt_vec3 pos;
	// Normal
	struct xrt_vec3 dir;

	// 85.0, 80.0, 0.0 in all entries so far
	struct xrt_vec3 angles;
} rift_s_led;

typedef struct
{
	int num_points;
	float points[4];
} rift_s_lensing_model;

typedef struct
{
	struct
	{
		struct xrt_matrix_3x3 rectification;
		struct xrt_vec3 offset;
	} gyro;

	struct
	{
		struct xrt_matrix_3x3 rectification;
		struct xrt_vec3 offset;
	} accel;

	struct xrt_vec3 imu_position;

	uint8_t num_leds;
	rift_s_led *leds;

	/* For some reason we have a separate calibration
	 * 4x4 matrix on top of the separate rectification
	 * and offset for gyro and accel
	 */
	struct xrt_matrix_4x4 gyro_calibration;
	struct xrt_matrix_4x4 accel_calibration;

	/* Lensing models */
	int num_lensing_models;
	rift_s_lensing_model *lensing_models;
} rift_s_controller_imu_calibration;

int
rift_s_parse_proximity_threshold(char *json, int *proximity_threshold);
int
rift_s_parse_imu_calibration(char *json, rift_s_imu_calibration *c);
int
rift_s_controller_parse_imu_calibration(char *json, rift_s_controller_imu_calibration *c);
void
rift_s_controller_free_imu_calibration(rift_s_controller_imu_calibration *c);

#endif
