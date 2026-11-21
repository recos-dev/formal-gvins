/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <formal_gvins/msckf/state.hpp>
#include <formal_gvins/camera/radtan.hpp>

namespace formal_gvins::msckf
{

using camera::Radtan;

template <typename Scalar> struct CameraDomain
{
    Scalar minimum_depth;
    Scalar maximum_depth;
    Scalar maximum_normalized_radius;
};

template <typename Linalg> struct CameraProjection
{
    linalg::Vector<Linalg, 2U> pixel;
    linalg::Matrix<Linalg, 2U, 3U> jacobian;
};

template <typename Linalg> struct CameraObservation
{
    linalg::Vector<Linalg, 2U> residual;
    linalg::Matrix<Linalg, 2U, 6U> H_pose;
    linalg::Matrix<Linalg, 2U, 3U> H_feature;
};

/** Forward projection and ordinary point derivative. All domains are explicit;
 * caller qualifies calibration/radius for the camera. Failure preserves output.
 * Raw image dimensions/calibration identity belong to the acquisition owner.
 */
template <typename Linalg>
[[nodiscard]] Status
project(linalg::Vector<Linalg, 3U> const & point, Radtan<typename Linalg::value_type> const & camera,
        CameraDomain<typename Linalg::value_type> const & domain, CameraProjection<Linalg> & output)
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    std::array<value_type, 11> const parameters{camera.fx,
                                                camera.fy,
                                                camera.cx,
                                                camera.cy,
                                                camera.k1,
                                                camera.k2,
                                                camera.p1,
                                                camera.p2,
                                                domain.minimum_depth,
                                                domain.maximum_depth,
                                                domain.maximum_normalized_radius};
    for (value_type value : parameters)
    {
        if (!math_type::is_finite(value))
        {
            return Status::non_finite_input;
        }
    }
    if (!(camera.fx > 0) || !(camera.fy > 0) || !(domain.minimum_depth > 0) ||
        domain.maximum_depth < domain.minimum_depth || !(domain.maximum_normalized_radius > 0))
    {
        return Status::domain_error;
    }
    if (!linalg::all_finite(point))
    {
        return Status::non_finite_input;
    }
    if (point(2) < domain.minimum_depth || point(2) > domain.maximum_depth)
    {
        return Status::domain_error;
    }
    value_type const x = point(0) / point(2);
    value_type const y = point(1) / point(2);
    value_type const rho = x * x + y * y;
    value_type const radius_squared = domain.maximum_normalized_radius * domain.maximum_normalized_radius;
    if (!math_type::is_finite(rho) || !math_type::is_finite(radius_squared))
    {
        return Status::non_finite_result;
    }
    if (rho > radius_squared)
    {
        return Status::domain_error;
    }
    auto const d = camera::distort(camera, x, y);
    CameraProjection<Linalg> candidate;
    candidate.pixel(0) = camera.fx * d.x + camera.cx;
    candidate.pixel(1) = camera.fy * d.y + camera.cy;
    linalg::Matrix<Linalg, 2U, 2U> JD;
    JD(0, 0) = camera.fx * d.j00;
    JD(0, 1) = camera.fx * d.j01;
    JD(1, 0) = camera.fy * d.j01;
    JD(1, 1) = camera.fy * d.j11;
    linalg::Matrix<Linalg, 2U, 3U> Jn;
    Jn(0, 0) = value_type{1} / point(2);
    Jn(1, 1) = value_type{1} / point(2);
    Jn(0, 2) = -x / point(2);
    Jn(1, 2) = -y / point(2);
    candidate.jacobian = JD * Jn;
    if (!linalg::all_finite(candidate.pixel) || !linalg::all_finite(candidate.jacobian))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** M-VISUAL/M-FEJ: residual at current pose; all geometric derivatives at first.
 * q_BC rotates camera to body; p_BC is camera origin in body coordinates.
 * The supplied feature is one common temporary global point for this candidate.
 * Returns both H_pose and H_feature; this is NOT yet a feature-eliminated constraint.
 */
template <typename Linalg>
[[nodiscard]] Status
camera_observation(Pose<Linalg> const & current, Pose<Linalg> const & first,
                   linalg::Vector<Linalg, 3U> const & feature_W, so3::UnitQuaternion<Linalg> const & q_BC,
                   linalg::Vector<Linalg, 3U> const & p_BC, linalg::Vector<Linalg, 2U> const & measurement,
                   Radtan<typename Linalg::value_type> const & camera,
                   CameraDomain<typename Linalg::value_type> const & domain,
                   Tolerances<typename Linalg::value_type> const & tolerance, CameraObservation<Linalg> & output)
{
    if (!valid_tolerances<Linalg>(tolerance))
    {
        return Status::domain_error;
    }
    Status status = validate_pose(current, tolerance.quaternion_squared_norm);
    if (!succeeded(status))
    {
        return status;
    }
    status = validate_pose(first, tolerance.quaternion_squared_norm);
    if (!succeeded(status))
    {
        return status;
    }
    status = validate_pose(Pose<Linalg>{p_BC, q_BC}, tolerance.quaternion_squared_norm);
    if (!succeeded(status))
    {
        return status;
    }
    if (!linalg::all_finite(feature_W) || !linalg::all_finite(measurement))
    {
        return Status::non_finite_input;
    }
    auto const R_CB = linalg::transpose(so3::to_rotation_matrix(q_BC));
    auto const current_R_BW = linalg::transpose(so3::to_rotation_matrix(current.q_WB));
    auto const first_R_BW = linalg::transpose(so3::to_rotation_matrix(first.q_WB));
    auto const s = current_R_BW * (feature_W - current.p_WB);
    auto const s0 = first_R_BW * (feature_W - first.p_WB);
    auto const point_C = R_CB * (s - p_BC);
    auto const first_point_C = R_CB * (s0 - p_BC);
    if (!linalg::all_finite(point_C) || !linalg::all_finite(first_point_C))
    {
        return Status::non_finite_result;
    }
    CameraProjection<Linalg> prediction, linearization;
    status = project(point_C, camera, domain, prediction);
    if (!succeeded(status))
    {
        return status;
    }
    status = project(first_point_C, camera, domain, linearization);
    if (!succeeded(status))
    {
        return status;
    }
    CameraObservation<Linalg> candidate;
    candidate.residual = measurement - prediction.pixel;
    auto const J = linearization.jacobian * R_CB;
    candidate.H_feature = J * first_R_BW;
    candidate.H_pose.template set_block<0, 0>(-candidate.H_feature);
    candidate.H_pose.template set_block<0, 3>(J * so3::hat(s0));
    if (!linalg::all_finite(candidate.residual) || !linalg::all_finite(candidate.H_pose) ||
        !linalg::all_finite(candidate.H_feature))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_gvins::msckf */
