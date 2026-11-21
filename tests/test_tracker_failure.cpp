/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/stereo_tracker.hpp>
#include "test_support.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
bool inject_failure = false;
int intercepted_calls = 0;
int fail_essential_call = 0;
int match_calls = 0, fail_match_call = 0;
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
    if (inject_failure || intercepted_calls == fail_essential_call)
    {
        CV_Error(cv::Error::StsError, "Injected geometry backend failure");
    }
    return real_essential(a, b, K, method, probability, threshold, iterations, mask);
}

// Dependency-level injection only in this executable, including the stereo-only phase.
extern "C" void real_knn(cv::DescriptorMatcher const *, cv::InputArray, cv::InputArray,
                         std::vector<std::vector<cv::DMatch>> &, int, cv::InputArray,
                         bool) asm("__real__ZNK2cv17DescriptorMatcher8knnMatchERKNS_11_InputArrayES3_RSt6vectorIS4_INS_"
                                   "6DMatchESaIS5_EESaIS7_EEiS3_b");
extern "C" void wrapped_knn(cv::DescriptorMatcher const *, cv::InputArray, cv::InputArray,
                            std::vector<std::vector<cv::DMatch>> &, int, cv::InputArray,
                            bool) asm("__wrap__ZNK2cv17DescriptorMatcher8knnMatchERKNS_11_InputArrayES3_RSt6vectorIS4_"
                                      "INS_6DMatchESaIS5_EESaIS7_EEiS3_b");
extern "C" void wrapped_knn(cv::DescriptorMatcher const * self, cv::InputArray a, cv::InputArray b,
                            std::vector<std::vector<cv::DMatch>> & matches, int k, cv::InputArray mask, bool compact)
{
    ++match_calls;
    if (match_calls == fail_match_call)
    {
        CV_Error(cv::Error::StsError, "Injected stereo matching failure");
    }
    real_knn(self, a, b, matches, k, mask, compact);
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

void stereo_failures(formal_gvins::test::Context & test, cv::Mat const & image)
{
    using namespace formal_gvins::vision;
    StereoTrackingOptions options;
    for (auto & camera : options.cameras)
    {
        camera.camera = {400, 400, 376, 240};
    }
    options.rig.t_RL = {-.1, 0, 0};
    StereoOrbTracker tracker(options), control(options);
    std::array<cv::Mat, 2> images{image, image};
    std::array<ImageIdentity, 2> ids{ImageIdentity{1, 0, 1, 100, 1, 1}, ImageIdentity{1, 1, 1, 100, 2, 1}};
    std::array<TrackedImage, 2> output, reference;
    StereoTrackingDiagnostics details;
    test.expect(tracker.process(ids, images, {}, output, &details) == VisionStatus::success &&
                    control.process(ids, images, {}, reference) == VisionStatus::success,
                "stereo failure", "initial pair");
    auto const saved = output;
    for (auto & id : ids)
    {
        ++id.image_id;
        id.time_ns += 100;
    }
    fail_essential_call = intercepted_calls + 2; // Left preparation succeeds; right temporal geometry throws.
    test.expect(tracker.process(ids, images, {}, output, &details) == VisionStatus::backend_failure &&
                    intercepted_calls == fail_essential_call && same(output[0], saved[0]) && same(output[1], saved[1]),
                "stereo failure", "second camera backend failure commits neither history");
    fail_essential_call = 0;
    cv::setRNGSeed(11);
    test.expect(tracker.process(ids, images, {}, output, &details) == VisionStatus::success, "stereo failure", "retry");
    cv::setRNGSeed(11);
    test.expect(control.process(ids, images, {}, reference) == VisionStatus::success && same(output[0], reference[0]) &&
                    same(output[1], reference[1]),
                "stereo failure", "retry matches unmodified two-camera control");
    for (auto & id : ids)
    {
        ++id.image_id;
        id.time_ns += 100;
    }
    fail_match_call = match_calls + 5; // Two directions for each temporal camera, then first stereo direction.
    test.expect(tracker.process(ids, images, {}, output, &details) == VisionStatus::success &&
                    match_calls == fail_match_call && details.association == StereoStatus::backend_failure &&
                    details.matches.inliers.empty() && details.cameras[0].matches.inliers.size() > 20 &&
                    details.cameras[1].matches.inliers.size() > 20,
                "stereo failure", "stereo-only backend exception preserves valid mono continuations");
    fail_match_call = 0;
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
    stereo_failures(test, image);
    return test.failures() == 0 ? 0 : 1;
}
