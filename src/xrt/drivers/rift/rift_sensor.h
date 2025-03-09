#pragma once

#include "rift_interface.h"

#include "xrt/xrt_frame.h"
#include "xrt/xrt_frameserver.h"
#include "util/u_sink.h"

struct rift_sensor
{
	struct xrt_frame_context frame_context;
	struct xrt_fs *frame_server;
	// struct os_hid_device *hid_dev;
};

int
rift_find_sensors(struct xrt_prober *xp,
                  struct xrt_prober_device **in_devices,
                  size_t device_count,
                  struct rift_sensor **out_sensors,
                  size_t *out_found_sensors);