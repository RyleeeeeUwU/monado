// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS Move tracker code that is expensive to compile.
 *
 * Typically built as a part of t_kalman.cpp to reduce incremental build times.
 *
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_tracking
 */

#include "BaseTypes.h"
#include "flexkalman/FlexibleKalmanBase.h"

namespace flexkalman {
template <typename State>
class AccelerometerMeasurement
    : public flexkalman::MeasurementBase<AccelerometerMeasurement<State>> {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;

    AccelerometerMeasurement(types::Vector<3> const &accel,
                             types::Vector<3> const &variance)
        : accel_(accel), covariance_(variance.asDiagonal()) {}

    MeasurementSquareMatrix const &getCovariance(State const & /*s*/)

    {
        return covariance_;
    }

    MeasurementVector predictMeasurement(State const &s) const {
        return s.acceleration();
    }

    MeasurementVector getResidual(MeasurementVector const &predictedMeasurement,
                                  State const &s) const {
        return accel_ - predictedMeasurement;
    }

    MeasurementVector getResidual(State const &s) const {
        return getResidual(predictMeasurement(s), s);
    }

  private:
    MeasurementVector accel_;
    MeasurementSquareMatrix covariance_;
};
} // namespace flexkalman
