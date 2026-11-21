/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <formal_gvins/vision/tracker.hpp>
#include <cstddef>
#include <span>
#include <vector>

namespace formal_gvins::vision
{
struct SelectionOptions
{
    int capacity = 400, columns = 8, rows = 6;
    double minimum_distance = 8; // Raw-image pixels, including across cells and octaves.
    [[nodiscard]] bool valid() const;
};

struct SelectionCandidate
{
    double u = 0, v = 0, response = 0;
    std::uint64_t track_id = 0, age = 0; // Both zero for a new feature.
};

struct SelectionStats
{
    std::size_t invalid_rejected = 0, quota_rejected = 0, spacing_rejected = 0;
};

struct SelectionResult
{
    std::vector<std::size_t> indices;
    SelectionStats stats;
};

/** Continuing tracks first (age then ID), new candidates by response then index.
 * Quotas and global spacing apply to both. Invalid pixels are excluded; malformed
 * configuration/identity fails atomically. Input image bounds are [0,width)x[0,height).
 */
[[nodiscard]] VisionStatus select_features(std::span<SelectionCandidate const> candidates, int width, int height,
                                           SelectionOptions const & options, SelectionResult & output);
} /* end namespace formal_gvins::vision */
