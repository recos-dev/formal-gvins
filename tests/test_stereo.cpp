/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/stereo_tracker.hpp>
#include <formal_gvins/scalar/backend/standard.hpp>
#include "test_support.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <opencv2/imgproc.hpp>

namespace
{
using namespace formal_gvins::vision;
using formal_gvins::test::Context;

void geometry(Context & test)
{
    StereoRig rig;
    double const angle = .12;
    rig.R_RL = {std::cos(angle), 0, std::sin(angle), 0, 1, 0, -std::sin(angle), 0, std::cos(angle)};
    rig.t_RL = {-.18, .025, .01};
    rig.epipolar_threshold = 1e-5;
    std::array<formal_gvins::camera::Radtan<double>, 2> cameras{
        formal_gvins::camera::Radtan<double>{410, 420, 320, 240, -.05, .005, .0001, -.0002},
        formal_gvins::camera::Radtan<double>{430, 405, 315, 245, -.06, .008, -.0002, .0001}};
    std::array<std::vector<FeatureObservation>, 2> views;
    std::vector<std::vector<cv::DMatch>> forward;
    for (int i = 0; i < 40; ++i)
    {
        cv::Vec3d const p{(i % 8 - 3.5) * .2, (i / 8 - 2.) * .22, 2.5 + .2 * (i % 3)};
        std::array<cv::Vec3d, 2> points{p, rig.R_RL * p + rig.t_RL};
        for (std::size_t c = 0; c < 2; ++c)
        {
            auto const & k = cameras[c];
            double const x = points[c][0] / points[c][2], y = points[c][1] / points[c][2];
            double const r2 = x * x + y * y, radial = 1 + k.k1 * r2 + k.k2 * r2 * r2;
            double const u = k.fx * (x * radial + 2 * k.p1 * x * y + k.p2 * (r2 + 2 * x * x)) + k.cx;
            double const v = k.fy * (y * radial + k.p1 * (r2 + 2 * y * y) + 2 * k.p2 * x * y) + k.cy;
            formal_gvins::camera::Bearing<double> bearing;
            test.expect(
                formal_gvins::succeeded(
                    formal_gvins::camera::unproject<formal_gvins::scalar::StandardMath<double>>(u, v, k, {}, bearing)),
                "stereo", "raw distorted pixel unprojects in each distinct camera");
            views[c].push_back({0, u, v, bearing.x, bearing.y});
        }
        if (i >= 32)
        {
            views[1].back().y += .2; // Known incorrect correspondence, not an epipolar-derived label.
        }
        forward.push_back({cv::DMatch(i, i, 10), cv::DMatch(i, (i + 1) % 40, 100)});
    }
    StereoMatchResult matches;
    test.expect(verify_stereo(views[0], views[1], forward, forward, rig, .75F, matches) == VisionStatus::success &&
                    matches.inliers.size() == 32 && matches.geometry_rejected == 8,
                "stereo", "known nonparallel rig accepts true and rejects false pairs");
    rig.R_RL = rig.R_RL.t();
    test.expect(verify_stereo(views[0], views[1], forward, forward, rig, .75F, matches) == VisionStatus::success &&
                    matches.inliers.size() < 4,
                "stereo", "reversing rotation disagrees with known scene");
    rig.t_RL = {0, 0, 0};
    auto const saved = matches.geometry_rejected;
    test.expect(verify_stereo(views[0], views[1], forward, forward, rig, .75F, matches) ==
                        VisionStatus::invalid_input &&
                    matches.geometry_rejected == saved,
                "stereo", "zero baseline fails atomically");
    cv::Matx33d E = cv::Matx33d::eye();
    rig.t_RL = {-.1, 0, 0};
    rig.R_RL = cv::Matx33d::eye();
    rig.R_RL(0, 0) = -1;
    test.expect(!rig.essential(E) && E == cv::Matx33d::eye(), "stereo", "reflection is not a rotation");
    rig.R_RL = cv::Matx33d::eye();
    rig.t_RL = {0, 0, 1};
    std::vector<FeatureObservation> epipoles(2);
    std::vector<std::vector<cv::DMatch>> neighbors{{{0, 0, 0}, {0, 1, 100}}, {{1, 1, 0}, {1, 0, 100}}};
    test.expect(verify_stereo(epipoles, epipoles, neighbors, neighbors, rig, .75F, matches) == VisionStatus::success &&
                    matches.inliers.empty(),
                "stereo", "undefined Sampson denominator is not a perfect match");
    test.expect(verify_stereo({}, {}, {}, {}, rig, .75F, matches) == VisionStatus::success && matches.inliers.empty(),
                "stereo", "empty or occluded camera supplies no links");
}

bool same(TrackedImage const & a, TrackedImage const & b)
{
    return a.identity.image_id == b.identity.image_id && a.identity.time_ns == b.identity.time_ns &&
           std::equal(a.observations.begin(), a.observations.end(), b.observations.begin(), b.observations.end(),
                      [](auto const & x, auto const & y)
                      { return x.track_id == y.track_id && x.u == y.u && x.v == y.v; }) &&
           std::equal(a.events.begin(), a.events.end(), b.events.begin(), b.events.end(),
                      [](auto const & x, auto const & y) { return x.track_id == y.track_id && x.change == y.change; });
}

void integration(Context & test, Execution execution)
{
    StereoTrackingOptions options;
    for (auto & camera : options.cameras)
    {
        camera.camera = {400, 400, 376, 240};
        camera.orb.execution = execution;
    }
    options.rig.t_RL = {-.12, 0, 0};
    StereoOrbTracker tracker(options), control(options);
    std::array<cv::Mat, 2> images;
    images[0] = cv::Mat(480, 752, CV_8UC1);
    cv::RNG(39).fill(images[0], cv::RNG::UNIFORM, 0, 256);
    cv::GaussianBlur(images[0], images[0], {5, 5}, .8);
    cv::warpAffine(images[0], images[1], cv::Matx23d(1, 0, -12, 0, 1, 0), images[0].size());
    std::array<ImageIdentity, 2> ids{ImageIdentity{1, 0, 1, 100, 1, 1}, ImageIdentity{1, 1, 1, 100, 2, 1}};
    std::array<TrackedImage, 2> output, reference;
    StereoTrackingDiagnostics details;
    auto process = [&](StereoOrbTracker & owner, std::array<TrackedImage, 2> & result, StereoTrackingDiagnostics * info)
    {
        cv::setRNGSeed(71);
        return owner.process(ids, images, {}, result, info);
    };
    test.expect(process(tracker, output, &details) == VisionStatus::success && details.matches.inliers.size() > 20,
                "stereo", "calibrated planar translation shares observations");
    // Exact control comparison applies to CPU. CUDA FAST/culling can choose different
    // candidate sets on repeated calls; atomic failure snapshots below remain exact on both.
    test.expect(process(control, reference, nullptr) == VisionStatus::success &&
                    (execution == Execution::cuda || (same(output[0], reference[0]) && same(output[1], reference[1]))),
                "stereo", "optional diagnostics do not change either camera");
    auto const first = output;
    for (auto const & m : details.matches.inliers)
    {
        test.expect(output[0].observations[static_cast<std::size_t>(m.queryIdx)].track_id ==
                        output[1].observations[static_cast<std::size_t>(m.trainIdx)].track_id,
                    "stereo", "verified partners have a shared ID");
    }
    for (auto & id : ids)
    {
        ++id.image_id;
        id.time_ns += 100;
    }
    auto const saved_image = images[1];
    images[1] = cv::Mat{};
    auto const saved_count = details.matches.inliers.size();
    test.expect(process(tracker, output, &details) == VisionStatus::invalid_input && same(output[0], first[0]) &&
                    same(output[1], first[1]) && details.matches.inliers.size() == saved_count,
                "stereo", "right input failure commits neither side");
    images[1] = saved_image;
    test.expect(process(tracker, output, &details) == VisionStatus::success &&
                    process(control, reference, nullptr) == VisionStatus::success &&
                    (execution == Execution::cuda || (same(output[0], reference[0]) && same(output[1], reference[1]))),
                "stereo", "retry has unchanged history and ID counter");
    for (std::size_t c = 0; c < 2; ++c)
    {
        std::set<std::uint64_t> seen;
        for (auto const & o : output[c].observations)
        {
            test.expect(seen.insert(o.track_id).second, "stereo", "one ID per camera image");
        }
    }
    // A deliberately unsynchronized first pair establishes independent temporal IDs.
    StereoOrbTracker conflict(options);
    ids[1].time_ns += 1;
    test.expect(process(conflict, output, &details) == VisionStatus::success &&
                    details.association == StereoStatus::unsynchronized,
                "stereo", "out-of-bound skew retains mono output without pretending simultaneous geometry");
    for (auto & id : ids)
    {
        ++id.image_id;
        id.time_ns = 500;
    }
    test.expect(process(conflict, output, &details) == VisionStatus::success && details.identity_conflicts > 20,
                "stereo", "established different IDs are never merged by a new stereo link");
    test.expect(details.cameras[0].matches.inliers.size() > 20 && details.cameras[1].matches.inliers.size() > 20,
                "stereo", "conflicting stereo links do not destroy temporal continuations");
    // Featureless is a valid observation, unlike a missing image.
    for (auto & id : ids)
    {
        ++id.image_id;
        id.time_ns += 100;
    }
    images[1] = cv::Mat(images[0].size(), CV_8UC1, cv::Scalar(0));
    test.expect(process(tracker, output, &details) == VisionStatus::success && output[1].observations.empty() &&
                    !output[0].observations.empty() && details.association == StereoStatus::no_matches,
                "stereo", "occlusion preserves the independently tracked camera");
    images[1] = saved_image;
    for (auto & id : ids)
    {
        ++id.image_id;
        id.time_ns += 100;
    }
    test.expect(process(tracker, output, &details) == VisionStatus::success && details.matches.inliers.size() > 20,
                "stereo", "returning camera reacquires global IDs from its live partner");
    for (auto const & link : details.matches.inliers)
    {
        auto const r = static_cast<std::size_t>(link.trainIdx);
        test.expect(details.cameras[1].ages[r] == 1 && output[1].events[r].change == TrackChange::started, "stereo",
                    "reacquired camera segment starts at age one");
    }
    tracker.reset();
    images[1] = saved_image;
    for (auto & id : ids)
    {
        ++id.image_id;
        id.time_ns += 100;
    }
    test.expect(process(tracker, output, &details) == VisionStatus::success, "stereo",
                "pair reset supports new acquisition");
    std::set<std::uint64_t> old;
    for (auto const & camera : first)
    {
        for (auto const & o : camera.observations)
        {
            old.insert(o.track_id);
        }
    }
    for (auto const & camera : output)
    {
        for (auto const & o : camera.observations)
        {
            test.expect(!old.contains(o.track_id), "stereo", "reset never recycles pair IDs");
        }
    }
}
} // namespace
int main(int argc, char ** argv)
{
    Context test;
    cv::setNumThreads(1);
    geometry(test);
    integration(test, argc == 2 && std::string_view(argv[1]) == "--cuda" ? Execution::cuda : Execution::cpu);
    return test.failures() == 0 ? 0 : 1;
}
