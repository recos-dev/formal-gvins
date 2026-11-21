/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/orb_tracker.hpp>
#include "test_support.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
bool inject_failure = false;
int intercepted_calls = 0;
} // namespace

// GNU/Clang ELF test seam for the exact overload in the pinned OpenCV 4.13.
// Only this test target wraps the dependency symbol; production has no hook.
extern "C" cv::Mat real_essential(
    cv::InputArray, cv::InputArray, cv::InputArray, int, double, double, int,
    cv::OutputArray) asm("__real__ZN2cv16findEssentialMatERKNS_11_InputArrayES2_S2_iddiRKNS_12_OutputArrayE");
extern "C" cv::Mat wrapped_essential(
    cv::InputArray a, cv::InputArray b, cv::InputArray K, int method, double probability, double threshold,
    int iterations,
    cv::OutputArray mask) asm("__wrap__ZN2cv16findEssentialMatERKNS_11_InputArrayES2_S2_iddiRKNS_12_OutputArrayE");
extern "C" cv::Mat wrapped_essential(cv::InputArray a, cv::InputArray b, cv::InputArray K, int method,
                                     double probability, double threshold, int iterations, cv::OutputArray mask)
{
    ++intercepted_calls;
    if (inject_failure)
    {
        CV_Error(cv::Error::StsError, "Injected geometry backend failure");
    }
    return real_essential(a, b, K, method, probability, threshold, iterations, mask);
}

bool same(formal_gvins::vision::TrackedImage const & a, formal_gvins::vision::TrackedImage const & b)
{
    if (a.identity.image_id != b.identity.image_id || a.identity.time_ns != b.identity.time_ns ||
        a.observations.size() != b.observations.size() || a.events.size() != b.events.size())
    {
        return false;
    }
    for (std::size_t i = 0; i < a.observations.size(); ++i)
    {
        auto const & x = a.observations[i];
        auto const & y = b.observations[i];
        if (x.track_id != y.track_id || x.u != y.u || x.v != y.v || x.x != y.x || x.y != y.y)
        {
            return false;
        }
    }
    for (std::size_t i = 0; i < a.events.size(); ++i)
    {
        if (a.events[i].track_id != b.events[i].track_id || a.events[i].change != b.events[i].change)
        {
            return false;
        }
    }
    return true;
}

int main()
{
    using namespace formal_gvins::vision;
    formal_gvins::test::Context test;
    cv::setNumThreads(1);
    OrbTrackingOptions options;
    options.camera = {400, 400, 376, 240};
    OrbTracker tracker(options), control(options);
    cv::Mat image(480, 752, CV_8UC1);
    cv::RNG(39).fill(image, cv::RNG::UNIFORM, 0, 256);
    cv::GaussianBlur(image, image, {5, 5}, .8);
    ImageIdentity identity{1, 0, 1, 100, 1, 1};
    TrackedImage output, reference;
    OrbTrackingDiagnostics diagnostics;
    test.expect(tracker.process(identity, image, {}, output, &diagnostics) == VisionStatus::success &&
                    control.process(identity, image, {}, reference) == VisionStatus::success,
                "failure", "initial frame");
    auto const saved = output;
    auto const saved_ages = diagnostics.ages;
    auto const saved_descriptor = diagnostics.features.descriptors.clone();
    ++identity.image_id;
    identity.time_ns += 50;
    inject_failure = true;
    test.expect(tracker.process(identity, image, {}, output, &diagnostics) == VisionStatus::backend_failure &&
                    intercepted_calls > 0,
                "failure", "dependency exception reached in production geometry path");
    test.expect(same(output, saved) && diagnostics.ages == saved_ages &&
                    cv::norm(diagnostics.features.descriptors, saved_descriptor, cv::NORM_INF) == 0,
                "failure", "both caller outputs remain unchanged");
    inject_failure = false;
    cv::setRNGSeed(0);
    test.expect(tracker.process(identity, image, {}, output, &diagnostics) == VisionStatus::success, "failure",
                "same frame may be retried");
    cv::setRNGSeed(0);
    test.expect(control.process(identity, image, {}, reference) == VisionStatus::success && same(output, reference),
                "failure", "failure did not consume IDs, frame time or history");
    auto retained = output;
    auto invalid = identity;
    ++invalid.image_id;
    --invalid.time_ns;
    test.expect(tracker.process(invalid, image, {}, output) == VisionStatus::invalid_input && same(output, retained),
                "failure", "backwards time");
    invalid = identity;
    ++invalid.image_id;
    invalid.time_ns += 50;
    ++invalid.noise_model_id;
    test.expect(tracker.process(invalid, image, {}, output) == VisionStatus::invalid_input && same(output, retained),
                "failure", "noise binding switch");
    invalid = identity;
    ++invalid.image_id;
    invalid.time_ns += 50;
    test.expect(tracker.process(invalid, cv::Mat::zeros(400, 600, CV_8UC1), {}, output) ==
                        VisionStatus::invalid_input &&
                    same(output, retained),
                "failure", "image geometry switch");
    test.expect(tracker.process(invalid, {}, {}, output) == VisionStatus::invalid_input && same(output, retained),
                "failure", "missing image is not a featureless image");
    return test.failures() == 0 ? 0 : 1;
}
