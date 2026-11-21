/* SPDX-License-Identifier: Apache-2.0 */
#include "timing.hpp"
#if defined(__linux__)
#include <sys/resource.h>
#endif

namespace formal_gvins::vision
{
StageTimer::StageTimer(FrontendMetrics * destination, FrontendStage stage) : m_destination(destination) { next(stage); }
StageTimer::~StageTimer() { finish(); }
void StageTimer::next(FrontendStage stage)
{
    finish();
    if (m_destination != nullptr)
    {
        m_stage = &(*m_destination)[static_cast<std::size_t>(stage)];
        m_start = std::chrono::steady_clock::now();
    }
}
void StageTimer::finish()
{
    if (m_stage == nullptr)
    {
        return;
    }
    m_stage->executed = true;
    m_stage->milliseconds =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_start).count();
#if defined(__linux__)
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) == 0 && usage.ru_maxrss > 0)
    {
        m_stage->process_peak_kib = static_cast<std::uint64_t>(usage.ru_maxrss);
    }
#endif
    m_stage = nullptr;
}
} /* end namespace formal_gvins::vision */
