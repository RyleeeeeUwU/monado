#include "xrt/xrt_tracking.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

struct PSVR2FusionInterfaceWrapper;

struct PSVR2FusionInterfaceWrapper *
psvr2_fusion_create(void);

void
psvr2_fusion_destroy(struct PSVR2FusionInterfaceWrapper *wrapper);

void
psvr2_fusion_process_imu_data(struct PSVR2FusionInterfaceWrapper *wrapper,
                              struct xrt_imu_sample *sample,
                              struct xrt_vec3 *orientation_variance_optional);

void
psvr2_fusion_process_slam_pose(struct PSVR2FusionInterfaceWrapper *wrapper,
                               struct xrt_pose_sample *sample,
                               struct xrt_vec3 *position_variance_optional,
                               struct xrt_vec3 *orientation_variance_optional,
                               float residual_limit);

void
psvr2_fusion_get_prediction(struct PSVR2FusionInterfaceWrapper *wrapper,
                            timepoint_ns timestamp_ns,
                            struct xrt_space_relation *out_relation);

#ifdef __cplusplus
}
#endif // __cplusplus
