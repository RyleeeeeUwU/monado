/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift S Driver - firmware JSON parsing functions */
#include <string.h>
#include <stdio.h>

#include "util/u_json.h"
#include "util/u_misc.h"

#include "rift_s_firmware.h"

#define JSON_INT(a, b, c) u_json_get_int(u_json_get(a, b), c)
#define JSON_FLOAT(a, b, c) u_json_get_float(u_json_get(a, b), c)
#define JSON_DOUBLE(a, b, c) u_json_get_double(u_json_get(a, b), c)
#define JSON_VEC3(a, b, c) u_json_get_vec3_array(u_json_get(a, b), c)
#define JSON_MATRIX_3X3_ARRAY(a, b, c) u_json_get_float_array(u_json_get(a, b), c.v, 9)
#define JSON_MATRIX_4x4_ARRAY(a, b, c) u_json_get_float_array(u_json_get(a, b), c.v, 16)

int
rift_s_parse_imu_calibration(char *json_string, rift_s_imu_calibration *c)
{
	const cJSON *obj, *version, *imu;
	float version_number = -1;

	cJSON *json_root = cJSON_Parse(json_string);
	if (!cJSON_IsObject(json_root)) {
		RIFT_S_ERROR("Could not parse JSON IMU calibration data.");
		cJSON_Delete(json_root);
		return -1;
	}

	obj = u_json_get(json_root, "FileFormat");
	if (!cJSON_IsObject(json_root)) {
		goto fail;
	}

	version = u_json_get(obj, "Version");
	char *version_str = cJSON_GetStringValue(version);
	if (version_str == NULL) {
		goto fail;
	}
	version_number = strtof(version_str, NULL);
	if (version_number != 1.0)
		goto fail;

	imu = u_json_get(json_root, "ImuCalibration");
	if (!cJSON_IsObject(imu)) {
		goto fail;
	}

	if (!JSON_MATRIX_4x4_ARRAY(imu, "DeviceFromImu", c->imu_to_device_transform))
		goto fail;

	obj = u_json_get(imu, "Gyroscope");
	if (!cJSON_IsObject(obj) || !JSON_MATRIX_3X3_ARRAY(obj, "RectificationMatrix", c->gyro.rectification)) {
		goto fail;
	}

	obj = u_json_get(obj, "Offset");
	if (!cJSON_IsObject(obj) || !JSON_VEC3(obj, "ConstantOffset", &c->gyro.offset)) {
		goto fail;
	}

	obj = u_json_get(imu, "Accelerometer");
	if (!cJSON_IsObject(obj) || !JSON_MATRIX_3X3_ARRAY(obj, "RectificationMatrix", c->accel.rectification)) {
		goto fail;
	}

	obj = u_json_get(obj, "Offset");
	if (!cJSON_IsObject(obj) || !JSON_VEC3(obj, "OffsetAtZeroDegC", &c->accel.offset_at_0C) ||
	    !JSON_VEC3(obj, "OffsetTemperatureCoefficient", &c->accel.temp_coeff)) {
		goto fail;
	}

	cJSON_Delete(json_root);
	return 0;

fail:
	RIFT_S_WARN("Unrecognised Rift S IMU Calibration JSON data. Version %f\n%s\n", version_number, json_string);
	cJSON_Delete(json_root);
	return -1;
}

static bool
json_read_led_point(const cJSON *led_model, rift_s_led *led, int n)
{
	const cJSON *array;
	double point[9];
	char name[32];

	snprintf(name, 32, "Point%d", n);
	array = u_json_get(led_model, name);
	if (!cJSON_IsArray(array) || cJSON_GetArraySize(array) != 9) {
		return false;
	}

	int j = 0;
	const cJSON *item = NULL;
	cJSON_ArrayForEach(item, array)
	{
		if (!cJSON_IsNumber(item)) {
			return false;
		}
		point[j++] = item->valuedouble;
	}

	led->pos.x = point[0];
	led->pos.y = point[1];
	led->pos.z = point[2];
	led->dir.x = point[3];
	led->dir.y = point[4];
	led->dir.z = point[5];
	led->angles.x = point[6];
	led->angles.y = point[7];
	led->angles.z = point[8];

	return true;
}

static bool
json_read_lensing_model(const cJSON *lensing_model, rift_s_lensing_model *model, int n)
{
	const cJSON *array;
	char name[32];

	snprintf(name, 32, "Model%d", n);
	array = u_json_get(lensing_model, name);
	if (!cJSON_IsArray(array) || cJSON_GetArraySize(array) != 5) {
		return false;
	}

	model->num_points = cJSON_GetArrayItem(array, 0)->valueint;

	for (int j = 0; j < 4; j++) {
		const cJSON *item = cJSON_GetArrayItem(array, j + 1);
		if (!cJSON_IsNumber(item)) {
			return false;
		}

		model->points[j] = item->valuedouble;
	}

	return true;
}

int
rift_s_controller_parse_imu_calibration(char *json_string, rift_s_controller_imu_calibration *c)
{
	const cJSON *obj, *version, *leds;
	const cJSON *item = NULL;
	int i;

	cJSON *json_root = cJSON_Parse(json_string);
	if (!cJSON_IsObject(json_root)) {
		RIFT_S_ERROR("Could not parse JSON Controller IMU calibration data.");
		cJSON_Delete(json_root);
		return -1;
	}

	obj = u_json_get(json_root, "TrackedObject");
	if (!cJSON_IsObject(obj)) {
		goto fail;
	}

	version = u_json_get(obj, "FlsVersion");
	char *version_str = cJSON_GetStringValue(version);
	if (version_str == NULL || strcmp(version_str, "1.0.10")) {
		RIFT_S_ERROR("Controller calibration version number has changed - got %s", version_str);
		goto fail;
	}

	if (!JSON_VEC3(obj, "ImuPosition", &c->imu_position))
		goto fail;

	if (!JSON_MATRIX_4x4_ARRAY(obj, "AccCalibration", c->accel_calibration))
		goto fail;

	if (!JSON_MATRIX_4x4_ARRAY(obj, "GyroCalibration", c->gyro_calibration))
		goto fail;

	/* LED positions */
	leds = u_json_get(obj, "ModelPoints");
	if (!cJSON_IsObject(leds)) {
		goto fail;
	}

	/* FIXME: Array foreach */
	c->num_leds = cJSON_GetArraySize(leds);
	c->leds = calloc(c->num_leds, sizeof(rift_s_led));
	i = 0;
	cJSON_ArrayForEach(item, leds)
	{
		i++;
		if (!json_read_led_point(leds, c->leds + i, i))
			goto fail;
	}

	/* LED lensing models */
	leds = u_json_get(obj, "Lensing");
	if (!cJSON_IsObject(leds)) {
		goto fail;
	}

	c->num_lensing_models = cJSON_GetArraySize(leds);
	c->lensing_models = calloc(c->num_lensing_models, sizeof(rift_s_lensing_model));
	i = 0;
	cJSON_ArrayForEach(item, leds)
	{
		if (!json_read_lensing_model(leds, c->lensing_models + i, i))
			goto fail;
	}

	if (!JSON_MATRIX_3X3_ARRAY(json_root, "gyro_m", c->gyro.rectification) ||
	    !JSON_VEC3(json_root, "gyro_b", &c->gyro.offset) ||
	    !JSON_MATRIX_3X3_ARRAY(json_root, "acc_m", c->gyro.rectification) ||
	    !JSON_VEC3(json_root, "acc_b", &c->gyro.offset)) {
		goto fail;
	}

	cJSON_Delete(json_root);
	return 0;

fail:
	RIFT_S_WARN("Unrecognised Rift S Controller Calibration JSON data.\n%s\n", json_string);
	rift_s_controller_free_imu_calibration(c);
	cJSON_Delete(json_root);
	return -1;
}

void
rift_s_controller_free_imu_calibration(rift_s_controller_imu_calibration *c)
{
	if (c->lensing_models) {
		free(c->lensing_models);
		c->lensing_models = NULL;
	}

	if (c->leds) {
		free(c->leds);
		c->leds = NULL;
	}
}
