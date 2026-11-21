/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/selection.hpp>
#include <algorithm>
#include <cmath>

namespace formal_gvins::vision
{
bool SelectionOptions::valid() const
{
    return capacity > 0 && capacity <= 10000 && columns > 0 && columns <= 64 && rows > 0 && rows <= 64 &&
           capacity >= rows * columns && std::isfinite(minimum_distance) && minimum_distance > 0 &&
           minimum_distance <= 16384;
}

VisionStatus select_features(std::span<SelectionCandidate const> candidates, int width, int height,
                             SelectionOptions const & options, SelectionResult & output)
{
    if (!options.valid() || candidates.size() > 10000 || width <= 0 || height <= 0 || width > 16384 || height > 16384)
    {
        return VisionStatus::invalid_input;
    }
    SelectionResult result;
    std::vector<std::size_t> order;
    std::vector<std::uint64_t> ids;
    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
        auto const & c = candidates[i];
        if ((c.track_id == 0) != (c.age == 0))
        {
            return VisionStatus::invalid_input;
        }
        if (c.track_id != 0)
        {
            ids.push_back(c.track_id);
        }
        if (!std::isfinite(c.u) || !std::isfinite(c.v) || !std::isfinite(c.response) || c.u < 0 || c.v < 0 ||
            c.u >= width || c.v >= height)
        {
            ++result.stats.invalid_rejected;
            continue;
        }
        order.push_back(i);
    }
    std::sort(ids.begin(), ids.end());
    if (std::adjacent_find(ids.begin(), ids.end()) != ids.end())
    {
        return VisionStatus::invalid_input;
    }
    std::sort(order.begin(), order.end(),
              [&](std::size_t a, std::size_t b)
              {
                  auto const & x = candidates[a];
                  auto const & y = candidates[b];
                  if ((x.track_id != 0) != (y.track_id != 0))
                  {
                      return x.track_id != 0;
                  }
                  if (x.age != y.age)
                  {
                      return x.age > y.age;
                  }
                  if (x.track_id != y.track_id)
                  {
                      return x.track_id < y.track_id;
                  }
                  if (x.response != y.response)
                  {
                      return x.response > y.response;
                  }
                  return a < b;
              });
    int const cells = options.columns * options.rows;
    std::vector<int> occupancy(static_cast<std::size_t>(cells), 0);
    double const minimum_squared = options.minimum_distance * options.minimum_distance;
    for (auto index : order)
    {
        auto const & point = candidates[index];
        int const cell = static_cast<int>(point.v * options.rows / height) * options.columns +
                         static_cast<int>(point.u * options.columns / width);
        int const quota = options.capacity / cells + (cell < options.capacity % cells ? 1 : 0);
        if (occupancy[static_cast<std::size_t>(cell)] >= quota)
        {
            ++result.stats.quota_rejected;
            continue;
        }
        bool separated = true;
        for (auto kept : result.indices)
        {
            auto const & old = candidates[kept];
            double const dx = point.u - old.u, dy = point.v - old.v;
            if (dx * dx + dy * dy < minimum_squared)
            {
                separated = false;
                break;
            }
        }
        if (!separated)
        {
            ++result.stats.spacing_rejected;
            continue;
        }
        ++occupancy[static_cast<std::size_t>(cell)];
        result.indices.push_back(index);
    }
    output = std::move(result);
    return VisionStatus::success;
}
} /* end namespace formal_gvins::vision */
