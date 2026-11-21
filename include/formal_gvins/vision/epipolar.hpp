/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <opencv2/core.hpp>

namespace formal_gvins::vision
{
/** Normalize a finite nonzero model to unit Frobenius norm. Failure preserves E. */
[[nodiscard]] bool normalize_epipolar(cv::Matx33d & E);

/** Normalized-coordinate Sampson gate. E must have unit Frobenius norm.
 * Undefined/ill-conditioned residuals reject; this is not a depth/pose test.
 */
[[nodiscard]] bool epipolar_inlier(cv::Matx33d const & E, cv::Point2d a, cv::Point2d b, double threshold,
                                   double minimum_denominator);
} /* end namespace formal_gvins::vision */
