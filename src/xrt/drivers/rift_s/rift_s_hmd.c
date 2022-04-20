/*
 * Copyright 2013, Fredrik Hultin.
 * Copyright 2013, Jakob Bornecrantz.
 * Copyright 2016 Philipp Zabel
 * Copyright 2019-2022 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 */
/*!
 * @file
 * @brief  Driver code for Oculus Rift S headsets
 *
 *
 * Ported from OpenHMD
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */

/* Oculus Rift S Driver - HID/USB Driver Implementation */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <assert.h>
#include <inttypes.h>

#include "math/m_api.h"
#include "math/m_vec3.h"

#include "os/os_time.h"

#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#include "xrt/xrt_device.h"

#include "rift_s_hmd.h"

#define DEG_TO_RAD(D) ((D)*M_PI / 180.)

static void
rift_s_update_inputs(struct xrt_device *xdev)
{
	// Empty, you should put code to update the attached input fields (if any)
}

static void
rift_s_get_tracked_pose(struct xrt_device *xdev,
                        enum xrt_input_name name,
                        uint64_t at_timestamp_ns,
                        struct xrt_space_relation *out_relation)
{
	struct rift_s_hmd *hmd = (struct rift_s_hmd *)(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		RIFT_S_ERROR("unknown input name");
		return;
	}

	// Estimate pose at timestamp at_timestamp_ns!
	math_quat_normalize(&hmd->pose.orientation);
	out_relation->pose = hmd->pose;
	out_relation->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
}

static void
rift_s_get_view_poses(struct xrt_device *xdev,
                      const struct xrt_vec3 *default_eye_relation,
                      uint64_t at_timestamp_ns,
                      uint32_t view_count,
                      struct xrt_space_relation *out_head_relation,
                      struct xrt_fov *out_fovs,
                      struct xrt_pose *out_poses)
{
	u_device_get_view_poses(xdev, default_eye_relation, at_timestamp_ns, view_count, out_head_relation, out_fovs,
	                        out_poses);
}

void
rift_s_hmd_handle_report(struct rift_s_hmd *hmd, rift_s_hmd_report_t *report)
{
	const uint32_t TICK_LEN_US = 1000000 / hmd->imu_config.imu_hz;
	uint32_t dt = TICK_LEN_US;

	if (hmd->last_imu_timestamp_ns != 0) {
		/* Avoid wrap-around on 32-bit device times */
		dt = report->timestamp - hmd->last_imu_timestamp32;
	} else {
		hmd->last_imu_timestamp_ns = report->timestamp;
	}
	hmd->last_imu_timestamp32 = report->timestamp;

	const float gyro_scale = 1.0 / hmd->imu_config.gyro_scale;
	const float accel_scale = MATH_GRAVITY_M_S2 / hmd->imu_config.accel_scale;
	const float temperature_scale = 1.0 / hmd->imu_config.temperature_scale;
	const float temperature_offset = hmd->imu_config.temperature_offset;

	for (int i = 0; i < 3; i++) {
		rift_s_hmd_imu_sample_t *s = report->samples + i;

		if (s->marker & 0x80)
			break; /* Sample (and remaining ones) are invalid */

		struct xrt_vec3 gyro, accel;

		gyro.x = DEG_TO_RAD(gyro_scale * s->gyro[0]);
		gyro.y = DEG_TO_RAD(gyro_scale * s->gyro[1]);
		gyro.z = DEG_TO_RAD(gyro_scale * s->gyro[2]);

		accel.x = accel_scale * s->accel[0];
		accel.y = accel_scale * s->accel[1];
		accel.z = accel_scale * s->accel[2];

		/* Apply correction offsets first, then rectify */
		accel = m_vec3_sub(accel, hmd->imu_calibration.accel.offset_at_0C);
		gyro = m_vec3_sub(gyro, hmd->imu_calibration.gyro.offset);
		math_matrix_3x3_transform_vec3(&hmd->imu_calibration.accel.rectification, &accel, &hmd->raw_accel);
		math_matrix_3x3_transform_vec3(&hmd->imu_calibration.gyro.rectification, &gyro, &hmd->raw_gyro);

		/* FIXME: This doesn't seem to produce the right numbers, but it's OK - we don't use it anyway */
		hmd->temperature = temperature_scale * (s->temperature - temperature_offset) + 25;

#if 0
		printf ("Sample %d dt %f accel %f %f %f gyro %f %f %f\n",
			i, dt_sec, hmd->raw_accel.x, hmd->raw_accel.y, hmd->raw_accel.z,
			hmd->raw_gyro.x, hmd->raw_gyro.y, hmd->raw_gyro.z);
#endif

		// Do 3DOF fusion
		m_imu_3dof_update(&hmd->fusion, hmd->last_imu_timestamp_ns, &hmd->raw_accel, &hmd->raw_gyro);

		hmd->last_imu_timestamp_ns += (uint64_t)dt * OS_NS_PER_USEC;
		dt = TICK_LEN_US;
	}
}

#if 0
static int
getf_hmd(ohmd_device *device, ohmd_float_value type, float *out)
{
	struct rift_s_hmd *hmd = dev_priv->hmd;

	switch (type) {
	case OHMD_DISTORTION_K: {
		for (int i = 0; i < 6; i++) {
			out[i] = 0.0; // hmd->display_info.distortion_k[i];
		}
		break;
	}

	case OHMD_ROTATION_QUAT: {
		*(quatf *)out = hmd->sensor_fusion.orient;
		break;
	}

	case OHMD_POSITION_VECTOR: out[0] = out[1] = out[2] = 0; break;

	case OHMD_CONTROLS_STATE: break;

	default:
		ohmd_set_error(hmd->ctx, "invalid type given to getf (%ud)", type);
		return -1;
		break;
	}

	return 0;
}
#endif

#if 0
static int
dump_fw_block(struct os_hid_device *handle, uint8_t block_id) {
	int res;
	char *data = NULL;
	int len;

	res = rift_s_read_firmware_block (handle, block_id, &data, &len);
	if (res	< 0)
			return res;

	free (data);
	return 0;
}
#endif

static int
read_hmd_calibration(struct rift_s_hmd *hmd, struct os_hid_device *hid_hmd)
{
	char *json = NULL;
	int json_len = 0;

	int ret = rift_s_read_firmware_block(hid_hmd, RIFT_S_FIRMWARE_BLOCK_IMU_CALIB, &json, &json_len);
	if (ret < 0)
		return ret;

	ret = rift_s_parse_imu_calibration(json, &hmd->imu_calibration);
	free(json);

	return ret;
}

static void
rift_s_hmd_destroy(struct xrt_device *xdev)
{
	struct rift_s_hmd *hmd = (struct rift_s_hmd *)(xdev);

	DRV_TRACE_MARKER();

	/* Remove this device from the system */
	rift_s_system_remove_hmd(hmd->sys);

	/* Drop the reference to the system */
	rift_s_system_reference(&hmd->sys, NULL);

	m_imu_3dof_close(&hmd->fusion);

	os_mutex_destroy(&hmd->mutex);

	u_device_free(&hmd->base);
}

struct rift_s_hmd *
rift_s_hmd_create(struct rift_s_system *sys)
{
	int ret;

	DRV_TRACE_MARKER();

	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct rift_s_hmd *hmd = U_DEVICE_ALLOCATE(struct rift_s_hmd, flags, 1, 0);
	if (hmd == NULL) {
		return NULL;
	}

	/* Take a reference to the rift_s_system */
	rift_s_system_reference(&hmd->sys, sys);

	hmd->base.tracking_origin = &sys->base;

	hmd->base.update_inputs = rift_s_update_inputs;
	hmd->base.get_tracked_pose = rift_s_get_tracked_pose;
	hmd->base.get_view_poses = rift_s_get_view_poses;
	hmd->base.destroy = rift_s_hmd_destroy;
	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->pose.orientation.w = 1.0f; // All other values set to zero by U_DEVICE_ALLOCATE (which calls U_CALLOC)

	m_imu_3dof_init(&hmd->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

	// Pose / state lock
	ret = os_mutex_init(&hmd->mutex);
	if (ret != 0) {
		RIFT_S_ERROR("Failed to init mutex!");
		goto cleanup;
	}

	// Print name.
	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "Oculus Rift S");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "FIXME S/N");

	// Setup input.
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;

	hmd->last_imu_timestamp_ns = 0;

	struct os_hid_device *hid_hmd = rift_s_system_hid_handle(hmd->sys);

	if (rift_s_read_device_info(hid_hmd, &hmd->device_info) < 0) {
		RIFT_S_ERROR("Failed to read Rift S device info");
		goto cleanup;
	}

	if (rift_s_get_report1(hid_hmd) < 0) {
		RIFT_S_ERROR("Failed to read Rift S Report 1");
		goto cleanup;
	}

	if (rift_s_read_imu_config(hid_hmd, &hmd->imu_config) < 0) {
		RIFT_S_ERROR("Failed to read IMU configuration block");
		goto cleanup;
	}

	if (read_hmd_calibration(hmd, hid_hmd) < 0)
		goto cleanup;

#if 0
	dump_fw_block(hid_hmd, 0xB);
	dump_fw_block(hid_hmd, 0xD);
	dump_fw_block(hid_hmd, 0xF);
	dump_fw_block(hid_hmd, 0x10);
	dump_fw_block(hid_hmd, 0x12);
#endif

	// Set up display details
	// refresh rate
	hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / 90.0f);

	const double hFOV = 90 * (M_PI / 180.0);
	const double vFOV = 96.73 * (M_PI / 180.0);
	// center of projection
	const double hCOP = 0.529;
	const double vCOP = 0.5;
	if (
	    /* right eye */
	    !math_compute_fovs(1, hCOP, hFOV, 1, vCOP, vFOV, &hmd->base.hmd->distortion.fov[1]) ||
	    /*
	     * left eye - same as right eye, except the horizontal center of projection is moved in the opposite
	     * direction now
	     */
	    !math_compute_fovs(1, 1.0 - hCOP, hFOV, 1, vCOP, vFOV, &hmd->base.hmd->distortion.fov[0])) {
		// If those failed, it means our math was impossible.
		RIFT_S_ERROR("Failed to setup basic device info");
		goto cleanup;
	}

	const int panel_w = 1080;
	const int panel_h = 1200;

	// Single "screen" (always the case)
	hmd->base.hmd->screens[0].w_pixels = panel_w * 2;
	hmd->base.hmd->screens[0].h_pixels = panel_h;

	// Left, Right
	for (uint8_t eye = 0; eye < 2; ++eye) {
		hmd->base.hmd->views[eye].display.w_pixels = panel_w;
		hmd->base.hmd->views[eye].display.h_pixels = panel_h;
		hmd->base.hmd->views[eye].viewport.y_pixels = 0;
		hmd->base.hmd->views[eye].viewport.w_pixels = panel_w;
		hmd->base.hmd->views[eye].viewport.h_pixels = panel_h;
		// if rotation is not identity, the dimensions can get more complex.
		hmd->base.hmd->views[eye].rot = u_device_rotation_ident;
	}
	// left eye starts at x=0, right eye starts at x=panel_width
	hmd->base.hmd->views[0].viewport.x_pixels = 0;
	hmd->base.hmd->views[1].viewport.x_pixels = panel_w;

	// Setup variable tracker: Optional but useful for debugging
	u_var_add_root(hmd, "Oculus Rift S", true);
	u_var_add_pose(hmd, &hmd->pose, "pose");
	u_var_add_log_level(hmd, &rift_s_log_level, "log_level");

	// Distortion information, fills in xdev->compute_distortion().
	u_distortion_mesh_set_none(&hmd->base);

	/* Set Opaque blend mode */
	hmd->base.hmd->blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = 1;

#if 0 // Render distortion etc
      // Set default device properties
	ohmd_set_default_device_properties(&hmd_dev->base.properties);

	/* FIXME: These defaults should be replaced from device configuration */
	hmd_dev->base.properties.hsize = 0.149760f;
	hmd_dev->base.properties.vsize = 0.093600f;
	hmd_dev->base.properties.lens_sep = 0.074f;
	hmd_dev->base.properties.lens_vpos = 0.046800f;
	hmd_dev->base.properties.fov = DEG_TO_RAD(105);

	/* FIXME: Incorrection distortion taken from the Rift CV1 for now */
#if 1
	ohmd_set_universal_distortion_k(&(hmd_dev->base.properties), 0.098f, .324f, -0.241f, 0.819f);
	ohmd_set_universal_aberration_k(&(hmd_dev->base.properties), 0.9952420f, 1.0f, 1.0008074f);
#else
	/* First pass at manual calibration */
	double scale = 1.31;
	double a = 0.049582 * scale, b = 0.221123 * scale, c = -0.174273 * scale;
	double d = 1.0 - (a + b + c);
	ohmd_set_universal_distortion_k(&(hmd_dev->base.properties), a, b, c, d);
	ohmd_set_universal_aberration_k(&(hmd_dev->base.properties), 0.99043452, 1.0, 1.0073939);
#endif

	hmd_dev->base.properties.hres = priv->device_info.h_resolution;
	hmd_dev->base.properties.vres = priv->device_info.v_resolution;

#if 0
	hmd_dev->base.properties.hsize = priv->device_info.h_screen_size;
	hmd_dev->base.properties.vsize = priv->device_info.v_screen_size;
	hmd_dev->base.properties.lens_sep = priv->device_info.lens_separation;
	hmd_dev->base.properties.lens_vpos = priv->device_info.v_center;
#endif
	hmd_dev->base.properties.ratio =
	    ((float)priv->device_info.h_resolution / (float)priv->device_info.v_resolution) / 2.0f;

	ohmd_calc_default_proj_matrices(&hmd_dev->base.properties);
#endif

	return hmd;

cleanup:
	rift_s_system_reference(&hmd->sys, NULL);
	return NULL;
}
