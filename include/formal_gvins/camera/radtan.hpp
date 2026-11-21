/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <formal_gvins/status.hpp>
#include <array>
#include <cstddef>

namespace formal_gvins::camera
{

/** Zero-skew pinhole intrinsics and normalized radial-tangential distortion. */
template <typename Scalar> struct Radtan
{
    Scalar fx = 0, fy = 0, cx = 0, cy = 0;
    Scalar k1 = 0, k2 = 0, p1 = 0, p2 = 0;
};

template <typename Scalar> struct Bearing
{
    Scalar x = 0, y = 0; // Homogeneous ray [x,y,1], not a unit vector.
};

template <typename Scalar> struct Distortion
{
    Scalar x, y;
    Scalar j00, j01, j11; // Symmetric derivative with respect to normalized x,y.
};

/** Unchecked algebra shared by forward projection and inverse iteration. */
template <typename Scalar> Distortion<Scalar> distort(Radtan<Scalar> const & camera, Scalar x, Scalar y)
{
    Scalar const r2 = x * x + y * y;
    Scalar const radial = Scalar{1} + camera.k1 * r2 + camera.k2 * r2 * r2;
    Scalar const eta = camera.k1 + Scalar{2} * camera.k2 * r2;
    return {x * radial + Scalar{2} * camera.p1 * x * y + camera.p2 * (r2 + Scalar{2} * x * x),
            y * radial + camera.p1 * (r2 + Scalar{2} * y * y) + Scalar{2} * camera.p2 * x * y,
            radial + Scalar{2} * x * x * eta + Scalar{2} * camera.p1 * y + Scalar{6} * camera.p2 * x,
            Scalar{2} * x * y * eta + Scalar{2} * camera.p1 * x + Scalar{2} * camera.p2 * y,
            radial + Scalar{2} * y * y * eta + Scalar{6} * camera.p1 * y + Scalar{2} * camera.p2 * x};
}

template <typename Scalar> struct InverseDomain
{
    Scalar radius = Scalar{2};
    Scalar residual_px = Scalar{1e-8};
    Scalar maximum_step = Scalar{0.5};
    Scalar minimum_slope = Scalar{1e-4};
    std::size_t iterations = 20; // At most 32 evaluations, including the final residual check.
};

/** Minimum of 1 + a*t + b*t^2 over [0,end]. */
template <typename Scalar> Scalar minimum_radial(Scalar a, Scalar b, Scalar end)
{
    Scalar const endpoint = Scalar{1} + a * end + b * end * end;
    Scalar result = endpoint < Scalar{1} ? endpoint : Scalar{1};
    if (b > Scalar{0} && a < Scalar{0})
    {
        Scalar const t = -a / (Scalar{2} * b);
        if (t < end)
        {
            Scalar const interior = Scalar{1} + a * t + b * t * t;
            result = interior < result ? interior : result;
        }
    }
    return result;
}

/** Conservative positive-Jacobian test on a disk. In exact arithmetic a
 * positive lower bound implies injectivity there. This numerical test is not
 * an interval certificate; deployments still qualify their calibration domain.
 */
template <typename Math>
Status validate_inverse(Radtan<typename Math::value_type> const & camera,
                        InverseDomain<typename Math::value_type> const & domain)
{
    using Scalar = typename Math::value_type;
    std::array<Scalar, 12> const values{camera.fx,     camera.fy,          camera.cx,           camera.cy,
                                        camera.k1,     camera.k2,          camera.p1,           camera.p2,
                                        domain.radius, domain.residual_px, domain.maximum_step, domain.minimum_slope};
    for (Scalar value : values)
    {
        if (!Math::is_finite(value))
        {
            return Status::non_finite_input;
        }
    }
    if (camera.fx <= 0 || camera.fy <= 0 || domain.radius <= 0 || domain.residual_px <= 0 || domain.maximum_step <= 0 ||
        domain.minimum_slope <= 0 || domain.iterations == 0 || domain.iterations > 32)
    {
        return Status::domain_error;
    }
    Scalar const end = domain.radius * domain.radius;
    if (!Math::is_finite(Scalar{3} * camera.k1 * end) || !Math::is_finite(Scalar{5} * camera.k2 * end * end))
    {
        return Status::non_finite_result;
    }
    Scalar const tangential = minimum_radial(camera.k1, camera.k2, end);
    Scalar const radial = minimum_radial(Scalar{3} * camera.k1, Scalar{5} * camera.k2, end);
    Scalar const perturbation = Scalar{8} * (Math::absolute(camera.p1) + Math::absolute(camera.p2)) * domain.radius;
    if (!Math::is_finite(end) || !Math::is_finite(tangential) || !Math::is_finite(radial) ||
        !Math::is_finite(perturbation))
    {
        return Status::non_finite_result;
    }
    return tangential - perturbation > domain.minimum_slope && radial - perturbation > domain.minimum_slope
               ? Status::success
               : Status::ill_conditioned;
}

/** Bounded Newton inverse. Caller checks raw image bounds and calibration identity.
 * No clamping or last-iterate success. Any failure preserves output.
 */
template <typename Math>
Status unproject(typename Math::value_type u, typename Math::value_type v,
                 Radtan<typename Math::value_type> const & camera,
                 InverseDomain<typename Math::value_type> const & domain, Bearing<typename Math::value_type> & output)
{
    using Scalar = typename Math::value_type;
    Status const status = validate_inverse<Math>(camera, domain);
    if (!succeeded(status))
    {
        return status;
    }
    if (!Math::is_finite(u) || !Math::is_finite(v))
    {
        return Status::non_finite_input;
    }
    Scalar const xd = (u - camera.cx) / camera.fx;
    Scalar const yd = (v - camera.cy) / camera.fy;
    Bearing<Scalar> candidate{xd, yd};
    for (std::size_t iteration = 0; iteration < 32; ++iteration)
    {
        if (iteration >= domain.iterations)
        {
            break;
        }
        Scalar const radius_squared = candidate.x * candidate.x + candidate.y * candidate.y;
        if (!Math::is_finite(radius_squared))
        {
            return Status::non_finite_result;
        }
        if (radius_squared > domain.radius * domain.radius)
        {
            return Status::out_of_range;
        }
        auto const d = distort(camera, candidate.x, candidate.y);
        Scalar const ex = d.x - xd, ey = d.y - yd;
        Scalar const det = d.j00 * d.j11 - d.j01 * d.j01;
        if (!Math::is_finite(ex) || !Math::is_finite(ey) || !Math::is_finite(det))
        {
            return Status::non_finite_result;
        }
        if (d.j00 <= domain.minimum_slope || d.j11 <= domain.minimum_slope ||
            det <= domain.minimum_slope * domain.minimum_slope)
        {
            return Status::ill_conditioned;
        }
        if (Math::absolute(camera.fx * ex) <= domain.residual_px &&
            Math::absolute(camera.fy * ey) <= domain.residual_px)
        {
            output = candidate;
            return Status::success;
        }
        Scalar const dx = (d.j11 * ex - d.j01 * ey) / det;
        Scalar const dy = (d.j00 * ey - d.j01 * ex) / det;
        if (!Math::is_finite(dx) || !Math::is_finite(dy))
        {
            return Status::non_finite_result;
        }
        if (Math::absolute(dx) > domain.maximum_step || Math::absolute(dy) > domain.maximum_step)
        {
            return Status::out_of_range;
        }
        candidate.x -= dx;
        candidate.y -= dy;
    }
    return Status::ill_conditioned;
}

} /* end namespace formal_gvins::camera */
