/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <inttypes.h>

#include "math/m_api.h"
#include "math/m_vec3.h"

#include "os/os_hid.h"

#include "util/u_device.h"
#include "util/u_trace_marker.h"

#include "rift_s.h"
#include "rift_s_hmd.h"
#include "rift_s_radio.h"
#include "rift_s_protocol.h"
#include "rift_s_controller.h"

/* Set to 1 to print controller states continuously */
#define DUMP_CONTROLLER_STATE 0

#define DEG_TO_RAD(D) ((D)*M_PI / 180.)

#if DUMP_CONTROLLER_STATE
static void
print_controller_state(struct rift_s_controller *ctrl)
{
	/* Dump the controller state if we see something unexpected / unknown, otherwise be quiet */
	if (ctrl->device_type == 0)
		return; // We don't know this device fully yet. Ignore it

	if (ctrl->extra_bytes_len == 0 && ctrl->mask08 == 0x50 && ctrl->mask0e == 0)
		return;

	printf("Controller %16lx type 0x%08x IMU ts %8u v2 %x accel %6d %6d %6d gyro %6d %6d %6d | ", ctrl->device_id,
	       ctrl->device_type, ctrl->imu_timestamp, ctrl->imu_unknown_varying2, ctrl->raw_accel[0],
	       ctrl->raw_accel[1], ctrl->raw_accel[2], ctrl->raw_gyro[0], ctrl->raw_gyro[1], ctrl->raw_gyro[2]);

	printf("unk %02x %02x buttons %02x fingers %02x | ", ctrl->mask08, ctrl->mask0e, ctrl->buttons, ctrl->fingers);
	printf("trigger %5d grip %5d |", ctrl->trigger, ctrl->grip);
	printf("joystick x %5d y %5d |", ctrl->joystick_x, ctrl->joystick_y);
	if (ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER) {
		printf("capsense x %u y %u joy %u trig %u | ", ctrl->capsense_a_x, ctrl->capsense_b_y,
		       ctrl->capsense_joystick, ctrl->capsense_trigger);
	} else if (ctrl->device_type == RIFT_S_DEVICE_RIGHT_CONTROLLER) {
		printf("capsense a %u b %u joy %u trig %u | ", ctrl->capsense_a_x, ctrl->capsense_b_y,
		       ctrl->capsense_joystick, ctrl->capsense_trigger);
	} else {
		printf("capsense ?? %u ?? %u ?? %u ?? %u | ", ctrl->capsense_a_x, ctrl->capsense_b_y,
		       ctrl->capsense_joystick, ctrl->capsense_trigger);
	}

	if (ctrl->extra_bytes_len) {
		printf(" | extra ");
		rift_s_hexdump_buffer(NULL, ctrl->extra_bytes, ctrl->extra_bytes_len);
	}
	printf("\n");
}
#endif

static void
handle_imu_update(struct rift_s_controller *ctrl,
                  uint32_t imu_timestamp,
                  const int16_t raw_accel[3],
                  const int16_t raw_gyro[3])
{
	if (ctrl->imu_time_valid) {
		uint32_t dt = imu_timestamp - ctrl->imu_timestamp32;
		ctrl->imu_timestamp += dt;
	} else {
		ctrl->imu_timestamp = imu_timestamp;
		ctrl->imu_time_valid = true;
	}

	ctrl->imu_timestamp32 = imu_timestamp;

	if (!ctrl->have_calibration || !ctrl->have_config)
		return; /* We need to finish reading the calibration or config blocks first */

	const float gyro_scale = ctrl->config.gyro_scale;
	const float accel_scale = MATH_GRAVITY_M_S2 * ctrl->config.accel_scale;

	struct xrt_vec3 gyro, accel;

	gyro.x = DEG_TO_RAD(gyro_scale * raw_gyro[0]);
	gyro.y = DEG_TO_RAD(gyro_scale * raw_gyro[1]);
	gyro.z = DEG_TO_RAD(gyro_scale * raw_gyro[2]);

	accel.x = accel_scale * raw_accel[0];
	accel.y = accel_scale * raw_accel[1];
	accel.z = accel_scale * raw_accel[2];

	/* Apply correction offsets first, then rectify */
	accel = m_vec3_sub(accel, ctrl->calibration.accel.offset);
	gyro = m_vec3_sub(gyro, ctrl->calibration.gyro.offset);

	math_matrix_3x3_transform_vec3(&ctrl->calibration.accel.rectification, &accel, &ctrl->accel);
	math_matrix_3x3_transform_vec3(&ctrl->calibration.gyro.rectification, &gyro, &ctrl->gyro);

	m_imu_3dof_update(&ctrl->fusion, ctrl->imu_timestamp, &ctrl->accel, &ctrl->gyro);

#if 0
	printf ("dt = %f raw accel %d %d %d gyro %d %d %d -> accel %f %f %f  gyro %f %f %f\n",
			dt_sec,
			raw_accel[0], raw_accel[1], raw_accel[2],
			raw_gyro[0], raw_gyro[1], raw_gyro[2],
			ctrl->accel.x, ctrl->accel.y, ctrl->accel.z,
			ctrl->gyro.x, ctrl->gyro.y, ctrl->gyro.z);
#endif
}

bool
rift_s_controller_handle_report(struct rift_s_controller *ctrl, rift_s_controller_report_t *report)
{
#if DUMP_CONTROLLER_STATE
	bool saw_imu_update = false;
#endif

	/* Collect state updates */
	ctrl->extra_bytes_len = 0;

	for (int i = 0; i < report->num_info; i++) {
		rift_s_controller_info_block_t *info = report->info + i;

		switch (info->block_id) {
		case RIFT_S_CTRL_MASK08: ctrl->mask08 = info->maskbyte.val; break;
		case RIFT_S_CTRL_BUTTONS: ctrl->buttons = info->maskbyte.val; break;
		case RIFT_S_CTRL_FINGERS: ctrl->fingers = info->maskbyte.val; break;
		case RIFT_S_CTRL_MASK0e: ctrl->mask0e = info->maskbyte.val; break;
		case RIFT_S_CTRL_TRIGGRIP: {
			ctrl->trigger = (uint16_t)(info->triggrip.vals[1] & 0x0f) << 8 | info->triggrip.vals[0];
			ctrl->grip =
			    (uint16_t)(info->triggrip.vals[1] & 0xf0) >> 4 | ((uint16_t)(info->triggrip.vals[2]) << 4);
			break;
		}
		case RIFT_S_CTRL_JOYSTICK:
			ctrl->joystick_x = info->joystick.val;
			ctrl->joystick_y = info->joystick.val >> 16;
			break;
		case RIFT_S_CTRL_CAPSENSE:
			ctrl->capsense_a_x = info->capsense.a_x;
			ctrl->capsense_b_y = info->capsense.b_y;
			ctrl->capsense_joystick = info->capsense.joystick;
			ctrl->capsense_trigger = info->capsense.trigger;
			break;
		case RIFT_S_CTRL_IMU: {
			int j;

#if DUMP_CONTROLLER_STATE
			/* print the state before updating the IMU timestamp a 2nd time */
			if (saw_imu_update)
				print_controller_state(ctrl);
			saw_imu_update = true;
#endif

			ctrl->imu_unknown_varying2 = info->imu.unknown_varying2;

			for (j = 0; j < 3; j++) {
				ctrl->raw_accel[j] = info->imu.accel[j];
				ctrl->raw_gyro[j] = info->imu.gyro[j];
			}
			handle_imu_update(ctrl, info->imu.timestamp, ctrl->raw_accel, ctrl->raw_gyro);
			break;
		}
		default:
			RIFT_S_WARN("Invalid controller info block with ID %02x from device %08" PRIx64
			            ". Please report it.\n",
			            info->block_id, ctrl->device_id);
			return false;
		}
	}

	if (report->extra_bytes_len > 0) {
		if (report->extra_bytes_len <= sizeof(ctrl->extra_bytes))
			memcpy(ctrl->extra_bytes, report->extra_bytes, report->extra_bytes_len);
		else {
			RIFT_S_WARN("Controller report from %16" PRIx64 " had too many extra bytes - %u (max %u)\n",
			            ctrl->device_id, report->extra_bytes_len,
			            (unsigned int)(sizeof(ctrl->extra_bytes)));
			return false;
		}
	}
	ctrl->extra_bytes_len = report->extra_bytes_len;

#if DUMP_CONTROLLER_STATE
	print_controller_state(ctrl);
#endif

	/* Finally, update and output the log */
	if (report->flags & 0x04) {
		/* New log line is starting, reset the counter */
		ctrl->log_bytes = 0;
	}

	if (ctrl->log_flags & 0x04 || (ctrl->log_flags & 0x02) != (report->flags & 0x02)) {
		/* New log bytes in this report, collect them */
		for (int i = 0; i < 3; i++) {
			uint8_t c = report->log[i];
			if (c != '\0') {
				if (ctrl->log_bytes == (MAX_LOG_SIZE - 1)) {
					/* Log line got too long... output it */
					ctrl->log[MAX_LOG_SIZE - 1] = '\0';
					RIFT_S_DEBUG("Controller: %s\n", ctrl->log);
					ctrl->log_bytes = 0;
				}
				ctrl->log[ctrl->log_bytes++] = c;
			} else if (ctrl->log_bytes > 0) {
				/* Found the end of the string */
				ctrl->log[ctrl->log_bytes] = '\0';
				printf("L	%s\n", ctrl->log);
				ctrl->log_bytes = 0;
			}
		}
	}
	ctrl->log_flags = report->flags;

	return true;
}

#define READ_LE16(b) (b)[0] | ((b)[1]) << 8
#define READ_LE32(b) (b)[0] | ((b)[1]) << 8 | ((b)[2]) << 16 | ((b)[3]) << 24
#define READ_LEFLOAT32(b) (*(float *)(b));

static void
ctrl_config_cb(bool success, uint8_t *response_bytes, int response_bytes_len, struct rift_s_controller *ctrl)
{
	ctrl->reading_config = false;
	if (!success) {
		RIFT_S_WARN("Failed to read controller config");
		return;
	}

	if (response_bytes_len < 5) {
		RIFT_S_WARN("Failed to read controller config - short result");
		return;
	}

	/* Response 0u32 0x10   00 7d a0 0f f4 01 f4 01 00 00 80 3a ff ff f9 3d
	 *   0x7d00 = 32000 0x0fa0 = 4000 0x01f4 = 500 0x01f4 = 500
	 *   0x3a800000 = 0.9765625e-03  = 1/1024
	 *   0x3df9ffff = 0.1220703      = 1/8192
	 */

	response_bytes_len = response_bytes[4];
	if (response_bytes_len < 16) {
		RIFT_S_ERROR("Failed to read controller config block - only got %d bytes\n", response_bytes_len);
		rift_s_hexdump_buffer("Controller Config", response_bytes, response_bytes_len);
		return;
	}
	response_bytes += 5;

	RIFT_S_INFO("Found new controller 0x%16" PRIx64 " type %08x\n", ctrl->device_id, ctrl->device_type);

	ctrl->config.accel_limit = READ_LE16(response_bytes + 0);
	ctrl->config.gyro_limit = READ_LE16(response_bytes + 2);
	ctrl->config.accel_hz = READ_LE16(response_bytes + 4);
	ctrl->config.gyro_hz = READ_LE16(response_bytes + 6);
	ctrl->config.accel_scale = READ_LEFLOAT32(response_bytes + 8);
	ctrl->config.gyro_scale = READ_LEFLOAT32(response_bytes + 12);

	ctrl->have_config = true;
}

static void
ctrl_json_cb(bool success, uint8_t *response_bytes, int response_bytes_len, struct rift_s_controller *ctrl)
{
	ctrl->reading_calibration = false;

	if (!success) {
		RIFT_S_WARN("Failed to read controller calibration block");
		return;
	}

	// RIFT_S_DEBUG ("Got Controller calibration:\n%s\n", response_bytes);

	if (rift_s_controller_parse_imu_calibration((char *)response_bytes, &ctrl->calibration) == 0) {
		ctrl->have_calibration = true;
	} else {
		RIFT_S_ERROR("Failed to parse controller configuration for controller 0x%16" PRIx64 "\n",
		             ctrl->device_id);
	}
}

#if 0 // FIXME
static void
init_touch_device(rift_s_controller_device *touch, int id)
{
	ohmd_device *ohmd_dev = &touch->base.base;

	touch->device_num = -1;

	ohmd_set_default_device_properties(&ohmd_dev->properties);

	ohmd_dev->properties.control_count = 8;

	if (id == 1) { // Right controller
		ohmd_dev->properties.controls_hints[0] = OHMD_BUTTON_A;
		ohmd_dev->properties.controls_hints[1] = OHMD_BUTTON_B;
		ohmd_dev->properties.controls_hints[2] = OHMD_HOME;         // Oculus button
		ohmd_dev->properties.controls_hints[3] = OHMD_ANALOG_PRESS; // stick button
	} else {
		ohmd_dev->properties.controls_hints[0] = OHMD_BUTTON_X;
		ohmd_dev->properties.controls_hints[1] = OHMD_BUTTON_Y;
		ohmd_dev->properties.controls_hints[2] = OHMD_MENU;
		ohmd_dev->properties.controls_hints[3] = OHMD_ANALOG_PRESS; // stick button
	}
	ohmd_dev->properties.controls_hints[4] = OHMD_TRIGGER;
	ohmd_dev->properties.controls_hints[5] = OHMD_SQUEEZE;
	ohmd_dev->properties.controls_hints[6] = OHMD_ANALOG_X;
	ohmd_dev->properties.controls_hints[7] = OHMD_ANALOG_Y;

	ohmd_dev->properties.controls_types[0] = OHMD_DIGITAL;
	ohmd_dev->properties.controls_types[1] = OHMD_DIGITAL;
	ohmd_dev->properties.controls_types[2] = OHMD_DIGITAL;
	ohmd_dev->properties.controls_types[3] = OHMD_DIGITAL;

	ohmd_dev->properties.controls_types[4] = OHMD_ANALOG;
	ohmd_dev->properties.controls_types[5] = OHMD_ANALOG;
	ohmd_dev->properties.controls_types[6] = OHMD_ANALOG;
	ohmd_dev->properties.controls_types[7] = OHMD_ANALOG;
}

static int
getf_touch_controller(ohmd_device *device, ohmd_float_value type, float *out)
{
	rift_s_device_priv *dev_priv = rift_s_device_priv_get(device);
	struct rift_s_hmd *hmd = dev_priv->sys;
	rift_s_controller_device *touch = (rift_s_controller_device *)(dev_priv);

	if (touch->device_num < 0)
		return -1; /* Device not online yet */

	struct rift_s_controller *ctrl = hmd->controllers + touch->device_num;

	switch (type) {
	case OHMD_ROTATION_QUAT: {
		*(quatf *)out = ctrl->imu_fusion.orient;
		break;
	}
	case OHMD_POSITION_VECTOR: out[0] = out[1] = out[2] = 0; break;
	case OHMD_DISTORTION_K: return -1;
	case OHMD_CONTROLS_STATE:
		out[0] = (ctrl->buttons & RIFT_S_BUTTON_A) != 0 ? 1.0 : 0.0;
		out[1] = (ctrl->buttons & RIFT_S_BUTTON_B) != 0 ? 1.0 : 0.0;
		out[2] = (ctrl->buttons & RIFT_S_BUTTON_OCULUS) != 0 ? 1.0 : 0.0;
		out[3] = (ctrl->buttons & RIFT_S_BUTTON_STICK) != 0 ? 1.0 : 0.0;

		out[4] = 1.0 - (float)(ctrl->trigger) / 4096.0;
		out[5] = 1.0 - (float)(ctrl->grip) / 4096.0;
		out[6] = (float)(ctrl->joystick_x) / 32768.0; /* FIXME: Scale this properly */
		out[7] = (float)(ctrl->joystick_y) / 32768.0; /* FIXME: Scale this properly */
		break;
	default: ohmd_set_error(hmd->ctx, "invalid type given to getf (%u)", type); return -1;
	}

	return 0;
}
#endif

static void
rift_s_controller_update_inputs(struct xrt_device *xdev)
{
	// Empty, you should put code to update the attached input fields (if any)
}

static void
rift_s_controller_get_tracked_pose(struct xrt_device *xdev,
                                   enum xrt_input_name name,
                                   uint64_t at_timestamp_ns,
                                   struct xrt_space_relation *out_relation)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		RIFT_S_ERROR("unknown input name");
		return;
	}

	// Estimate pose at timestamp at_timestamp_ns!
	math_quat_normalize(&ctrl->pose.orientation);
	out_relation->pose = ctrl->pose;
	out_relation->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
}

static void
rift_s_controller_get_view_poses(struct xrt_device *xdev,
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

static void
rift_s_controller_destroy(struct xrt_device *xdev)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);

	/* Tell the system this controller is going away */
	rift_s_system_remove_controller(ctrl->sys, ctrl);

	/* Release the HMD reference */
	rift_s_system_reference(&ctrl->sys, NULL);

	m_imu_3dof_close(&ctrl->fusion);

	u_device_free(&ctrl->base);
}

struct rift_s_controller *
rift_s_controller_create(struct rift_s_system *sys, enum xrt_device_type device_type)
{
	DRV_TRACE_MARKER();

	enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(U_DEVICE_ALLOC_TRACKING_NONE);

	struct rift_s_controller *ctrl = U_DEVICE_ALLOCATE(struct rift_s_controller, flags, 1, 0);
	if (ctrl == NULL) {
		return NULL;
	}

	/* Store a ref to the parent hmd, released in destroy */
	rift_s_system_reference(&ctrl->sys, sys);

	ctrl->base.update_inputs = rift_s_controller_update_inputs;
	ctrl->base.get_tracked_pose = rift_s_controller_get_tracked_pose;
	ctrl->base.get_view_poses = rift_s_controller_get_view_poses;
	ctrl->base.destroy = rift_s_controller_destroy;
	ctrl->base.name = XRT_DEVICE_TOUCH_CONTROLLER;
	ctrl->base.device_type = device_type;

	ctrl->pose.orientation.w = 1.0f; // All other values set to zero by U_DEVICE_ALLOCATE (which calls U_CALLOC)
	m_imu_3dof_init(&ctrl->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

	// Print name. FIXME: Set name to left/right. Set correct serial ID
	snprintf(ctrl->base.str, XRT_DEVICE_NAME_LEN, "Oculus Rift S Touch Controller");
	snprintf(ctrl->base.serial, XRT_DEVICE_NAME_LEN, "FIXME S/N");

	// Setup input. FIXME: Add all inputs and AIM+GRIP pose
	ctrl->base.inputs[0].name = XRT_INPUT_TOUCH_AIM_POSE;

	return ctrl;
}

void
rift_s_controller_update_configuration(struct rift_s_controller *ctrl)
{
	rift_s_radio_state *radio = rift_s_system_radio(ctrl->sys);

	if (!ctrl->have_config && !ctrl->reading_config) {
		const uint8_t config_req[] = {0x32, 0x20, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		rift_s_radio_queue_command(radio, ctrl->device_id, config_req, sizeof(config_req),
		                           (rift_s_radio_completion_fn)ctrl_config_cb, ctrl);
		ctrl->reading_config = true;
	}

	if (!ctrl->have_calibration && !ctrl->reading_calibration) {
		rift_s_radio_get_json_block(radio, ctrl->device_id, (rift_s_radio_completion_fn)ctrl_json_cb, ctrl);
		ctrl->reading_calibration = true;
	}
}
