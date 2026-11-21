/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/orb_tracker.hpp>
#include <formal_gvins/vision/epipolar.hpp>
#include <formal_gvins/scalar/backend/standard.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <opencv2/calib3d.hpp>

namespace formal_gvins::vision
{
namespace
{
cv::Vec3d ray(cv::Point2d point)
{
    cv::Vec3d const d(point.x, point.y, 1);
    return d / cv::norm(d);
}

bool fit_rotation(std::vector<cv::Point2d> const & a, std::vector<cv::Point2d> const & b,
                  std::span<std::size_t const> indices, cv::Matx33d & R)
{
    cv::Matx33d covariance = cv::Matx33d::zeros();
    for (auto i : indices)
    {
        covariance += ray(b[i]) * ray(a[i]).t();
    }
    cv::Mat singular, U, Vt;
    cv::SVD::compute(covariance, singular, U, Vt);
    if (singular.at<double>(1) <= 1e-6 * singular.at<double>(0))
    {
        return false;
    }
    cv::Mat sign = cv::Mat::eye(3, 3, CV_64F);
    sign.at<double>(2, 2) = cv::determinant(U * Vt) < 0 ? -1 : 1;
    cv::Mat const matrix = U * sign * Vt;
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            R(row, col) = matrix.at<double>(row, col);
        }
    }
    return true;
}

std::vector<std::size_t> rotation_support(cv::Matx33d const & R, std::vector<cv::Point2d> const & a,
                                          std::vector<cv::Point2d> const & b,
                                          std::vector<std::size_t> const & candidates, double threshold)
{
    std::vector<std::size_t> accepted;
    for (auto i : candidates)
    {
        auto const d = R * cv::Vec3d(a[i].x, a[i].y, 1);
        if (d[2] > 1e-12 && cv::norm(cv::Point2d(d[0] / d[2], d[1] / d[2]) - b[i]) <= threshold)
        {
            accepted.push_back(i);
        }
    }
    return accepted;
}

bool screen_rotation(std::vector<cv::Point2d> const & a, std::vector<cv::Point2d> const & b,
                     OrbTrackingOptions const & options, std::vector<std::size_t> & accepted)
{
    // E is still required. A dominant rotation provides an additional constraint,
    // never a fallback accepting candidates from a failed epipolar model.
    if (accepted.size() < 8)
    {
        return false;
    }
    cv::RNG random(0);
    std::vector<std::size_t> best;
    for (int iteration = 0; iteration < options.rotation_iterations; ++iteration)
    {
        std::array<std::size_t, 3> sample;
        for (int j = 0; j < 3; ++j)
        {
            sample[static_cast<std::size_t>(j)] =
                accepted[static_cast<std::size_t>(random.uniform(0, static_cast<int>(accepted.size())))];
        }
        if (sample[0] == sample[1] || sample[0] == sample[2] || sample[1] == sample[2])
        {
            continue;
        }
        cv::Matx33d R;
        if (!fit_rotation(a, b, sample, R))
        {
            continue;
        }
        auto support = rotation_support(R, a, b, accepted, options.epipolar_threshold);
        if (support.size() > best.size())
        {
            best = std::move(support);
            if (best.size() == accepted.size())
            {
                break;
            }
        }
    }
    if (best.size() < 8 || best.size() < options.rotation_support_fraction * accepted.size())
    {
        return false;
    }
    cv::Matx33d R;
    if (!fit_rotation(a, b, best, R))
    {
        return false;
    }
    auto refined = rotation_support(R, a, b, accepted, options.epipolar_threshold);
    if (refined.size() < 8 || refined.size() < options.rotation_support_fraction * accepted.size())
    {
        return false;
    }
    accepted = std::move(refined);
    return true;
}

bool spread(std::vector<cv::Point2d> const & points)
{
    cv::Point2d mean;
    for (auto const & p : points)
    {
        mean += p;
    }
    mean *= 1.0 / static_cast<double>(points.size());
    double xx = 0, xy = 0, yy = 0;
    for (auto const & p : points)
    {
        auto const d = p - mean;
        xx += d.x * d.x;
        xy += d.x * d.y;
        yy += d.y * d.y;
    }
    // Reject repeated/collinear samples without imposing a minimum parallax.
    double const trace = xx + yy;
    return std::isfinite(trace) && trace > 1e-12 && xx * yy - xy * xy > 1e-8 * trace * trace;
}

bool ratio_candidates(std::vector<std::vector<cv::DMatch>> const & neighbors, std::size_t query_count,
                      std::size_t train_count, float ratio, std::vector<cv::DMatch> & best, std::size_t & passed)
{
    if (neighbors.size() > query_count)
    {
        return false;
    }
    std::vector<bool> seen(query_count, false);
    best.resize(query_count);
    for (auto const & pair : neighbors)
    {
        if (pair.empty())
        {
            continue;
        }
        int const query = pair[0].queryIdx;
        if (query < 0 || static_cast<std::size_t>(query) >= query_count || seen[static_cast<std::size_t>(query)] ||
            pair.size() > 2)
        {
            return false;
        }
        seen[static_cast<std::size_t>(query)] = true;
        for (auto const & match : pair)
        {
            if (match.queryIdx != query || match.trainIdx < 0 ||
                static_cast<std::size_t>(match.trainIdx) >= train_count || !std::isfinite(match.distance) ||
                match.distance < 0 || match.distance > 256)
            {
                return false;
            }
        }
        if (pair.size() < 2)
        {
            continue;
        }
        auto const & first = pair[0];
        auto const & second = pair[1];
        if (first.trainIdx == second.trainIdx || second.distance < first.distance)
        {
            return false;
        }
        if (first.distance < ratio * second.distance)
        {
            best[static_cast<std::size_t>(query)] = first;
            ++passed;
        }
    }
    return true;
}

} /* end anonymous namespace */

VisionStatus reciprocal_matches(std::vector<std::vector<cv::DMatch>> const & forward,
                                std::vector<std::vector<cv::DMatch>> const & reverse, std::size_t query_count,
                                std::size_t train_count, float ratio, DescriptorMatches & output)
{
    if (query_count > 10000 || train_count > 10000 || !std::isfinite(ratio) || ratio <= 0 || ratio >= 1)
    {
        return VisionStatus::invalid_input;
    }
    DescriptorMatches candidate;
    std::vector<cv::DMatch> a, b;
    if (!ratio_candidates(forward, query_count, train_count, ratio, a, candidate.forward_ratio_passed) ||
        !ratio_candidates(reverse, train_count, query_count, ratio, b, candidate.reverse_ratio_passed))
    {
        return VisionStatus::invalid_input;
    }
    for (auto const & match : a)
    {
        if (match.trainIdx >= 0 && b[static_cast<std::size_t>(match.trainIdx)].trainIdx == match.queryIdx)
        {
            candidate.matches.push_back(match);
        }
    }
    output = std::move(candidate);
    return VisionStatus::success;
}

VisionStatus verify_matches(OrbFeatures const & previous, OrbFeatures const & current,
                            std::vector<std::vector<cv::DMatch>> const & neighbors,
                            std::vector<std::vector<cv::DMatch>> const & reverse, OrbTrackingOptions const & options,
                            OrbMatchResult & output)
{
    if (!options.valid())
    {
        return VisionStatus::invalid_input;
    }
    OrbMatchResult candidate;
    auto const status = reciprocal_matches(neighbors, reverse, previous.keypoints.size(), current.keypoints.size(),
                                           options.ratio, candidate.descriptors);
    if (status != VisionStatus::success)
    {
        return status;
    }
    std::vector<cv::DMatch> unique;
    std::vector<cv::Point2d> points_previous, points_current;
    for (auto const & match : candidate.descriptors.matches)
    {
        auto const query = static_cast<std::size_t>(match.queryIdx);
        auto const train = static_cast<std::size_t>(match.trainIdx);
        auto const p = previous.keypoints[query].pt;
        auto const q = current.keypoints[train].pt;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(q.x) || !std::isfinite(q.y))
        {
            return VisionStatus::invalid_input;
        }
        camera::Bearing<double> a, b;
        if (!succeeded(camera::unproject<scalar::StandardMath<double>>(p.x, p.y, options.camera, options.inverse, a)) ||
            !succeeded(camera::unproject<scalar::StandardMath<double>>(q.x, q.y, options.camera, options.inverse, b)))
        {
            ++candidate.inverse_rejected;
            continue;
        }
        unique.push_back(match);
        points_previous.emplace_back(a.x, a.y);
        points_current.emplace_back(b.x, b.y);
    }
    try
    {
        // Known intrinsics constrain the model to E = [t]_x R. Require more
        // candidates than the five-point minimal sample before confirming tracks.
        if (unique.size() >= 8U && spread(points_previous) && spread(points_current))
        {
            cv::Mat mask;
            cv::Mat const E =
                cv::findEssentialMat(points_previous, points_current, cv::Matx33d::eye(), cv::RANSAC,
                                     options.confidence, options.epipolar_threshold, options.ransac_iterations, mask);
            cv::Matx33d normalized;
            if (E.rows == 3 && E.cols == 3 && E.type() == CV_64F && mask.total() == unique.size())
            {
                for (int row = 0; row < 3; ++row)
                {
                    for (int column = 0; column < 3; ++column)
                    {
                        normalized(row, column) = E.at<double>(row, column);
                    }
                }
                if (normalize_epipolar(normalized))
                {
                    std::vector<std::size_t> accepted;
                    for (std::size_t index = 0; index < unique.size(); ++index)
                    {
                        if (mask.ptr<unsigned char>()[index] != 0 &&
                            epipolar_inlier(normalized, points_previous[index], points_current[index],
                                            options.epipolar_threshold, options.minimum_epipolar_denominator))
                        {
                            accepted.push_back(index);
                        }
                    }
                    candidate.rotation_checked = screen_rotation(points_previous, points_current, options, accepted);
                    for (auto index : accepted)
                    {
                        candidate.inliers.push_back(unique[index]);
                    }
                    candidate.geometry_valid = candidate.inliers.size() >= 8U;
                    if (!candidate.geometry_valid)
                    {
                        candidate.inliers.clear();
                    }
                }
            }
        }
        output = std::move(candidate);
        return VisionStatus::success;
    }
    catch (cv::Exception const &)
    {
        return VisionStatus::backend_failure;
    }
}

} /* end namespace formal_gvins::vision */
