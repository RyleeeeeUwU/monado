#include "rift_sensor.h"

int
rift_find_sensors(struct xrt_prober *xp,
                  struct xrt_prober_device **in_devices,
                  size_t device_count,
                  struct rift_sensor **out_sensors,
                  size_t *out_found_sensors)
{
	int result;
	size_t found_sensors = 0;

	for (size_t i = 0; i < device_count; i++) {
		struct xrt_prober_device *device = in_devices[i];

		if (device->vendor_id != OCULUS_VR_VID)
			continue;

		switch (device->product_id) {
		case OCULUS_DK2_SENSOR_PID: {
			found_sensors++;
			break;
		}
		default: continue;
		}
	}

	if (found_sensors == 0) {
		*out_found_sensors = 0;
		return 0;
	}

	struct rift_sensor *sensors = calloc(found_sensors, sizeof(struct rift_sensor));
	found_sensors = 0;
	for (size_t i = 0; i < device_count; i++) {
		struct xrt_prober_device *device = in_devices[i];

		if (device->vendor_id != OCULUS_VR_VID)
			continue;

		switch (device->product_id) {
		case OCULUS_DK2_SENSOR_PID: {
			struct rift_sensor sensor;

			result = xrt_prober_open_video_device(xp, device, &sensor.frame_context, &sensor.frame_server);
			if (result < 0)
				goto cleanup;
            
			// result = xrt_prober_open_hid_interface(xp, device, 1, &sensor.hid_dev);
			// if (result < 0)
			// 	goto cleanup;

			sensors[found_sensors++] = sensor;

			break;
		}
		default: continue;
		}
	}

	*out_found_sensors = found_sensors;
	*out_sensors = sensors;
	return 0;

cleanup:
    //TODO: properly close sensors that *may* have been already loaded
	free(sensors);
	return result;
}