/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/epipolar.hpp>
#include <algorithm>
#include <cmath>

namespace formal_gvins::vision
{
bool normalize_epipolar(cv::Matx33d & E)
{
    double scale = 0;
    for (double value : E.val)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
        scale = std::max(scale, std::abs(value));
    }
    if (scale == 0)
    {
        return false;
    }
    auto candidate = E;
    double sum = 0;
    for (double & value : candidate.val)
    {
        value /= scale;
        sum += value * value;
    }
    double const norm = std::sqrt(sum);
    for (double & value : candidate.val)
    {
        value /= norm;
    }
    E = candidate;
    return true;
}

bool epipolar_inlier(cv::Matx33d const & E, cv::Point2d a, cv::Point2d b, double threshold, double minimum_denominator)
{
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) || !std::isfinite(b.y) ||
        !std::isfinite(threshold) || threshold <= 0 || !std::isfinite(minimum_denominator) || minimum_denominator <= 0)
    {
        return false;
    }
    cv::Vec3d const x(a.x, a.y, 1), y(b.x, b.y, 1);
    auto const line_b = E * x, line_a = E.t() * y;
    double const residual = y.dot(line_b);
    double const denominator =
        line_b[0] * line_b[0] + line_b[1] * line_b[1] + line_a[0] * line_a[0] + line_a[1] * line_a[1];
    if (!std::isfinite(denominator) || denominator <= minimum_denominator)
    {
        return false;
    }
    double const error = residual * residual / denominator;
    return std::isfinite(error) && std::isfinite(threshold * threshold) && error <= threshold * threshold;
}
} /* end namespace formal_gvins::vision */
