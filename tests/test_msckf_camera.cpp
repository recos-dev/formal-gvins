/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/msckf/camera.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"
#include <limits>

template <typename Linalg>
bool same_projection(formal_gvins::msckf::CameraProjection<Linalg> const & a,
                     formal_gvins::msckf::CameraProjection<Linalg> const & b)
{
    return formal_gvins::linalg::max_abs(a.pixel - b.pixel) == 0 &&
           formal_gvins::linalg::max_abs(a.jacobian - b.jacobian) == 0;
}

template <typename Linalg>
bool same_observation(formal_gvins::msckf::CameraObservation<Linalg> const & a,
                      formal_gvins::msckf::CameraObservation<Linalg> const & b)
{
    return formal_gvins::linalg::max_abs(a.residual - b.residual) == 0 &&
           formal_gvins::linalg::max_abs(a.H_pose - b.H_pose) == 0 &&
           formal_gvins::linalg::max_abs(a.H_feature - b.H_feature) == 0;
}

int main()
{
    using namespace formal_gvins;
    using L = test::Backend<double>;
    using Vector = linalg::Vector<L, 3>;
    test::configure_backend_test();
    test::Context t;
    msckf::Radtan<double> camera{400, 450, 320, 240, 0, 0, 0, 0};
    msckf::CameraDomain<double> domain{.1, 60, 1.5};
    msckf::Tolerances<double> tol{1e-12, 1e-12};
    msckf::CameraProjection<L> projection;
    auto point = Vector::from_row_major({1., 2., 4.});
    t.expect(succeeded(msckf::project(point, camera, domain, projection)), "camera", "pinhole limit");
    t.expect(projection.pixel(0) == 420 && projection.pixel(1) == 465 && projection.jacobian(0, 0) == 100 &&
                 projection.jacobian(0, 2) == -25,
             "camera", "pixel and derivative units");
    auto saved = projection;
    point(2) = -1;
    t.expect(msckf::project(point, camera, domain, projection) == Status::domain_error &&
                 same_projection(projection, saved),
             "camera", "behind-camera atomicity");
    point = Vector::from_row_major({0., 0., .1});
    t.expect(succeeded(msckf::project(point, camera, domain, projection)), "camera", "minimum depth equality");
    point(2) = 60;
    t.expect(succeeded(msckf::project(point, camera, domain, projection)), "camera", "maximum depth equality");
    camera.fx = 0;
    t.expect(msckf::project(point, camera, domain, projection) == Status::domain_error, "camera", "bad intrinsics");
    camera.fx = 400;
    camera.k1 = -.2;
    camera.k2 = .03;
    camera.p1 = .001;
    camera.p2 = -.002;
    msckf::Pose<L> current, first;
    current.p_WB(0) = .1;
    point = Vector::from_row_major({.5, .3, 4.});
    Vector lever;
    lever(0) = .05;
    linalg::Vector<L, 2> measured;
    measured(0) = 370;
    measured(1) = 270;
    msckf::CameraObservation<L> observation;
    t.expect(succeeded(msckf::camera_observation(current, first, point, so3::UnitQuaternion<L>{}, lever, measured,
                                                 camera, domain, tol, observation)),
             "camera", "current residual and frozen derivative");
    auto old = observation;
    current.p_WB(0) = .2;
    t.expect(succeeded(msckf::camera_observation(current, first, point, so3::UnitQuaternion<L>{}, lever, measured,
                                                 camera, domain, tol, observation)),
             "camera", "updated current pose");
    t.expect(observation.residual(0) != old.residual(0) && linalg::max_abs(observation.H_pose - old.H_pose) == 0 &&
                 linalg::max_abs(observation.H_feature - old.H_feature) == 0,
             "camera", "FEJ derivative remains frozen");
    old = observation;
    first.p_WB(2) = 5;
    t.expect(msckf::camera_observation(current, first, point, so3::UnitQuaternion<L>{}, lever, measured, camera, domain,
                                       tol, observation) == Status::domain_error &&
                 same_observation(observation, old),
             "camera", "invalid frozen projection atomicity");
    point = Vector::from_row_major({7., 0., 4.});
    t.expect(msckf::project(point, camera, domain, projection) == Status::domain_error, "camera", "radius limit");
    point = Vector::from_row_major({4., 4., 4.});
    camera.k1 = std::numeric_limits<double>::max();
    saved = projection;
    t.expect(msckf::project(point, camera, domain, projection) == Status::non_finite_result &&
                 same_projection(projection, saved),
             "camera", "finite-input distortion overflow");
    camera.k1 = -.2;
    current.p_WB(0) = -std::numeric_limits<double>::max();
    point(0) = std::numeric_limits<double>::max();
    t.expect(msckf::camera_observation(current, first, point, so3::UnitQuaternion<L>{}, lever, measured, camera, domain,
                                       tol, observation) == Status::non_finite_result &&
                 same_observation(observation, old),
             "camera", "transformed-point overflow");
    point(0) = std::numeric_limits<double>::quiet_NaN();
    t.expect(msckf::project(point, camera, domain, projection) == Status::non_finite_input, "camera",
             "nonfinite geometry");
    return t.failures() == 0 ? 0 : 1;
}
