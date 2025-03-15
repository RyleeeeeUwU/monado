// Copyright 2024, Joel Valenciano
// Copyright 2025, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS VR2 tracker code that is expensive to compile.
 *
 * @author Joel Valenciano <joelv1907@gmail.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_tracking
 */

#include "tracking/t_tracker_kalman_fusion.hpp"
#include "flexkalman/AugmentedProcessModel.h"
#include "flexkalman/AugmentedState.h"
#include "flexkalman/ConstantProcess.h"
#include "flexkalman/PureVectorState.h"
#include "tracking/t_fusion.hpp"
#include "tracking/t_imu_fusion.hpp"

#include "math/m_eigen_interop.hpp"

#include "util/u_misc.h"

#include <cstdio>
#include <flexkalman/AbsolutePositionMeasurement.h>
#include "flexkalman/AbsoluteOrientationMeasurement.h"
#include <flexkalman/AccelerometerMeasurement.h>
#include <iostream>
#include "flexkalman/FlexibleKalmanFilter.h"
#include "flexkalman/FlexibleUnscentedCorrect.h"
#include "flexkalman/PoseSeparatelyDampedConstantVelocity.h"
#include "flexkalman/PoseState.h"


namespace xrt::auxiliary::tracking {

using namespace xrt::auxiliary::math;

//! Anonymous namespace to hide implementation names
namespace {
	using State = flexkalman::pose_externalized_rotation::State;
	using BiasState = flexkalman::PureVectorState<3>;
	using CombinedState = flexkalman::AugmentedState<State, BiasState>;
	using ProcessModelA = flexkalman::PoseSeparatelyDampedConstantVelocityProcessModel<State>;
	using ProcessModelB = flexkalman::ConstantProcess<BiasState>;
	using CombinedProcessModel = flexkalman::AugmentedProcessModel<ProcessModelA, ProcessModelB>;
	using AbsolutePositionMeasurement = flexkalman::AbsolutePositionEKFMeasurement<State>;
	using AbsoluteOrientationMeasurement = flexkalman::AbsoluteOrientationEKFMeasurement<State>;
	using AccelerometerMeasurement = flexkalman::AccelerometerMeasurement<State>;

	struct TrackingInfo
	{
		bool valid{false};
		bool tracked{false};
	};
	class KalmanFusion : public KalmanFusionInterface
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		void
		clear_position_tracked_flag() override;

		void
		process_imu_data(const struct xrt_imu_sample *sample,
		                 const struct xrt_vec3 *accel_variance_optional,
		                 const struct xrt_vec3 *gyro_variance_optional) override;
		void
		process_slam_pose(const struct xrt_pose_sample *sample,
		                  const struct xrt_vec3 *position_variance_optional,
		                  const struct xrt_vec3 *orientation_variance_optional,
		                  const float residual_limit) override;

		void
		get_prediction(const timepoint_ns when_ns,
		               struct xrt_space_relation *out_relation) override;

	private:
		void
		reset_filter();
		void
		reset_filter_and_imu();

		State filter_state;
		BiasState bias_state{0, 0, 0};
		CombinedState combined_state{filter_state, bias_state};

		ProcessModelA main_process_model;
		ProcessModelB bias_process_model;
		CombinedProcessModel combined_process_model{main_process_model, bias_process_model};

		SimpleIMUFusion imu;

		timepoint_ns filter_time_ns{0};
		bool tracked{false};
		TrackingInfo orientation_state;
		TrackingInfo position_state;
	};



	void
	KalmanFusion::clear_position_tracked_flag()
	{
		position_state.tracked = false;
	}

	void
	KalmanFusion::reset_filter()
	{
		filter_state = State{};
		tracked = false;
		position_state = TrackingInfo{};
	}
	void
	KalmanFusion::reset_filter_and_imu()
	{
		reset_filter();
		orientation_state = TrackingInfo{};
		imu = SimpleIMUFusion{};
	}

	void
	KalmanFusion::process_imu_data(const struct xrt_imu_sample *sample,
	                               const struct xrt_vec3 *accel_variance_optional,
	                               const struct xrt_vec3 *gyro_variance_optional)
	{
		Eigen::Vector3d accel_variance = Eigen::Vector3d::Constant(0.01);
		Eigen::Vector3d gyro_variance = Eigen::Vector3d::Constant(0.01);
		if (accel_variance_optional) {
			accel_variance = map_vec3(*accel_variance_optional).cast<double>();
		}
		if (gyro_variance_optional) {
			gyro_variance = map_vec3(*gyro_variance_optional).cast<double>();
		}

		auto accel = map_vec3_f64(sample->accel_m_s2);
		auto gyro = map_vec3_f64(sample->gyro_rad_secs);
		imu.handleAccel(accel, sample->timestamp_ns);
		imu.handleGyro(gyro, sample->timestamp_ns);
		imu.postCorrect();

		//! @todo use better measurements instead of the preceding "simple
		//! fusion"
		if (filter_time_ns != 0 && filter_time_ns != sample->timestamp_ns) {
			float dt = time_ns_to_s(sample->timestamp_ns - filter_time_ns);
			assert(dt > 0);
			flexkalman::predict(combined_state, combined_process_model, dt);
		}

		filter_time_ns = sample->timestamp_ns;

		auto accel_residual = imu.getCorrectedWorldAccel(accel);
		auto accel_measurement = AccelerometerMeasurement{accel_residual, accel_variance};
		auto gyro_measurement = BiasedGyroMeasurement{gyro, gyro_variance};

		if (flexkalman::correctUnscented(combined_state, gyro_measurement) &&
		    flexkalman::correctUnscented(filter_state, accel_measurement)) {
			orientation_state.tracked = true;
			orientation_state.valid = true;
		} else {
			U_LOG_E(
			    "Got non-finite something when filtering IMU - "
			    "resetting filter and IMU fusion!");
			reset_filter_and_imu();
		}

		// 7200 deg/sec
		constexpr double max_rad_per_sec = 20 * double(EIGEN_PI) * 2;
		if (filter_state.angularVelocity().squaredNorm() > max_rad_per_sec * max_rad_per_sec) {
			U_LOG_E(
			    "Got excessive angular velocity when filtering "
			    "IMU - resetting filter and IMU fusion!");
			reset_filter_and_imu();
		}
	}

	void
	KalmanFusion::process_slam_pose(const struct xrt_pose_sample *sample,
	                                const struct xrt_vec3 *position_variance_optional,
	                                const struct xrt_vec3 *orientation_variance_optional,
	                                float residual_limit)
	{
		Eigen::Vector3f pos = map_vec3(sample->pose.position);
		Eigen::Quaternionf orient = map_quat(sample->pose.orientation);
		Eigen::Vector3d position_variance{1.e-4, 1.e-4, 4.e-4};
		Eigen::Vector3d orientation_variance{1.e-4, 1.e-4, 4.e-4};

		if (position_variance_optional) {
			position_variance = map_vec3(*position_variance_optional).cast<double>();
		}
		if (orientation_variance_optional) {
			orientation_variance = map_vec3(*orientation_variance_optional).cast<double>();
		}

		auto pos_measurement = AbsolutePositionMeasurement{pos.cast<double>(), position_variance};
		auto orient_measurement = AbsoluteOrientationMeasurement{orient.cast<double>(), orientation_variance};

		double pos_resid = pos_measurement.getResidual(filter_state).norm();
		double orient_resid = orient_measurement.getResidual(filter_state).norm();

		if (pos_resid > residual_limit) {
			// Residual arbitrarily "too large"
			U_LOG_W(
			    "position measurement residual is %f, resetting "
			    "filter state",
			    pos_resid);
			reset_filter();
			return;
		}
		if (orient_resid > residual_limit) {
			// Residual arbitrarily "too large"
			U_LOG_W(
			    "orientation measurement residual is %f, resetting "
			    "filter state",
			    orient_resid);
			reset_filter();
			return;
		}
		if (flexkalman::correctUnscented(filter_state, orient_measurement) &&
		    flexkalman::correctUnscented(filter_state, pos_measurement)) {
			if (!tracked) {
				tracked = true;
				position_state.valid = true;
				position_state.tracked = true;
			}
		} else {
			U_LOG_W(
			    "Got non-finite something when filtering "
			    "tracker - resetting filter!");
			reset_filter();
		}
	}

	void
	KalmanFusion::get_prediction(timepoint_ns when_ns, struct xrt_space_relation *out_relation)
	{
		if (out_relation == NULL) {
			return;
		}
		// Clear to identity values
		U_ZERO(out_relation);
		out_relation->pose.orientation.w = 1;
		if (!tracked || filter_time_ns == 0) {
			return;
		}
		float dt = time_ns_to_s(when_ns - filter_time_ns);
		auto predicted_state = flexkalman::getPrediction(filter_state, main_process_model, dt);

		map_vec3(out_relation->pose.position) = predicted_state.position().cast<float>();
		map_quat(out_relation->pose.orientation) = predicted_state.getQuaternion().cast<float>();
		map_vec3(out_relation->linear_velocity) = predicted_state.velocity().cast<float>();
		map_vec3(out_relation->angular_velocity) = predicted_state.angularVelocity().cast<float>();

		uint64_t flags = 0;
		if (position_state.valid) {
			flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT;
			flags |= XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
			if (position_state.tracked) {
				flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
			}
		}
		if (orientation_state.valid) {
			flags |= XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
			flags |= XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
			if (orientation_state.tracked) {
				flags |= XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
			}
		}
		out_relation->relation_flags = (xrt_space_relation_flags)flags;
	}
} // namespace


std::unique_ptr<KalmanFusionInterface>
KalmanFusionInterface::create()
{
	auto ret = std::make_unique<KalmanFusion>();
	return ret;
}
} // namespace xrt::auxiliary::tracking
