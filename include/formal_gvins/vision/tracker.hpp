/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <formal_gvins/vision/observation.hpp>
#include <vector>

namespace formal_gvins::vision
{

enum class VisionStatus
{
    success,
    invalid_input,
    backend_unavailable,
    backend_failure,
};

enum class Execution
{
    cpu,
    cuda,
};

/**
 * Owning host-side observation batch, independent of the tracking method.
 * Success replaces the complete active set for this camera image; a failed
 * process call preserves the batch and tracker history. An absent previous ID
 * is retired only after a successful frame or an explicit reset.
 * Numerical consumers receive the records, not this dynamically sized container.
 */
struct TrackedImage
{
    ImageIdentity identity;
    PixelDomain pixel_domain = PixelDomain::unspecified;
    std::vector<FeatureObservation> observations;
};

} /* end namespace formal_gvins::vision */
