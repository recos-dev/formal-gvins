/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <formal_gvins/vision/metrics.hpp>
#include <chrono>

namespace formal_gvins::vision
{
/** Host diagnostics only; a null destination performs no clock or OS calls. */
class StageTimer
{
public:
    StageTimer(FrontendMetrics * destination, FrontendStage stage);
    ~StageTimer();
    StageTimer(StageTimer const &) = delete;
    StageTimer & operator=(StageTimer const &) = delete;
    void next(FrontendStage stage);
    void finish();

private:
    FrontendMetrics * m_destination;
    StageMetrics * m_stage = nullptr;
    std::chrono::steady_clock::time_point m_start;
};
} /* end namespace formal_gvins::vision */
