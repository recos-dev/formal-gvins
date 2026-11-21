/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>

namespace formal_gvins::vision
{
enum class FrontendStage
{
    detection,
    localization,
    description,
    unprojection,
    matching,
    geometry,
    selection,
    publication,
    count
};
inline constexpr std::array<char const *, 8> frontend_stage_names{
    "detection", "localization", "description", "unprojection", "matching", "geometry", "selection", "publication"};
struct StageMetrics
{
    bool executed = false;
    double milliseconds = 0;
    // Linux process high-water RSS sampled at stage exit, not memory owned by the stage.
    // Includes warmup/library allocations. Zero means unavailable; GPU VRAM is not measured.
    std::uint64_t process_peak_kib = 0;
};
using FrontendMetrics = std::array<StageMetrics, static_cast<std::size_t>(FrontendStage::count)>;
} /* end namespace formal_gvins::vision */
