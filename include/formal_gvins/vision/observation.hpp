/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cstdint>

namespace formal_gvins::vision
{

/** Shared metadata for one image's observations; time is acquisition time in ns. */
struct ImageIdentity
{
    std::uint64_t session_id = 0;
    std::uint32_t camera_id = 0;
    std::uint64_t image_id = 0;
    std::uint64_t time_ns = 0;
    std::uint64_t calibration_id = 0;
    std::uint64_t noise_model_id = 0;
};

enum class PixelDomain
{
    unspecified,
    raw,
    undistorted,
};

/** Pixel coordinates, interpreted using the enclosing batch's domain and image bindings. */
struct FeatureObservation
{
    std::uint64_t track_id = 0;
    double u = 0;
    double v = 0;
    double x = 0, y = 0; // Derived normalized ray [x,y,1]; same calibration, not another measurement.
};

} /* end namespace formal_gvins::vision */
