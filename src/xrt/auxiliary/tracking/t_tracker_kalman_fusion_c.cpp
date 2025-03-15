#include "tracking/t_tracker_psvr2_fusion.hpp"
#include "xrt/xrt_tracking.h"
#include <cstdlib>
#include "psvr2_fusion.h"

using xrt::auxiliary::tracking::PSVR2FusionInterface;

struct PSVR2FusionInterfaceWrapper
{
	std::unique_ptr<PSVR2FusionInterface> fusion;

	PSVR2FusionInterfaceWrapper() : fusion(PSVR2FusionInterface::create()) {}

	~PSVR2FusionInterfaceWrapper() {}
};

extern "C" {

struct PSVR2FusionInterfaceWrapper *
psvr2_fusion_create(void)
{
	return new PSVR2FusionInterfaceWrapper;
}

void
psvr2_fusion_destroy(PSVR2FusionInterfaceWrapper *wrapper)
{
	delete wrapper;
}

void
psvr2_fusion_process_imu_data(PSVR2FusionInterfaceWrapper *wrapper,
                              struct xrt_imu_sample *sample,
                              struct xrt_vec3 *orientation_variance_optional)
{
	wrapper->fusion->process_imu_data(sample, orientation_variance_optional);
}

void
psvr2_fusion_process_slam_pose(PSVR2FusionInterfaceWrapper *wrapper,
                               struct xrt_pose_sample *sample,
                               struct xrt_vec3 *position_variance_optional,
                               struct xrt_vec3 *orientation_variance_optional,
                               float residual_limit)
{
	wrapper->fusion->process_slam_pose(sample, position_variance_optional, orientation_variance_optional,
	                                   residual_limit);
}

void
psvr2_fusion_get_prediction(struct PSVR2FusionInterfaceWrapper *wrapper,
                            timepoint_ns timestamp_ns,
                            struct xrt_space_relation *out_relation)
{
	wrapper->fusion->get_prediction(timestamp_ns, out_relation);
}
}
