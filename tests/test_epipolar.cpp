/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/epipolar.hpp>
#include <formal_gvins/vision/orb_tracker.hpp>
#include "test_support.hpp"
#include <limits>

int main()
{
    using namespace formal_gvins::vision;
    formal_gvins::test::Context test;
    cv::setNumThreads(1);
    cv::Matx33d E(0, 0, 0, 0, 0, -1, 0, 1, 0);
    test.expect(normalize_epipolar(E), "Sampson", "unit model");
    test.expect(epipolar_inlier(E, {0, 0}, {.4, 0}, .001, 1e-12), "Sampson", "horizontal epipolar displacement");
    test.expect(!epipolar_inlier(E, {0, 0}, {.4, .01}, .001, 1e-12), "Sampson", "off-line point rejected");
    test.expect(!epipolar_inlier(cv::Matx33d::zeros(), {0, 0}, {0, 0}, .001, 1e-12), "Sampson",
                "undefined residual rejected");
    auto scaled = E * 1e300;
    test.expect(normalize_epipolar(scaled) && cv::norm(scaled - E) < 1e-15, "Sampson", "overflow-safe model scaling");
    scaled(0, 0) = std::numeric_limits<double>::infinity();
    test.expect(!normalize_epipolar(scaled), "Sampson", "nonfinite model rejected");
    OrbTrackingOptions options;
    options.camera = {400, 450, 320, 240, -.2, .04, .001, -.002};
    options.epipolar_threshold = 0.0003;
    for (int scenario = 0; scenario < 5; ++scenario)
    {
        for (int seed = 0; seed < 8; ++seed)
        {
            cv::RNG random(seed + 17);
            cv::setRNGSeed(seed);
            OrbFeatures a, b;
            std::vector<std::vector<cv::DMatch>> neighbors;
            for (int i = 0; i < 80; ++i)
            {
                double const x = random.uniform(-1.5, 1.5), y = random.uniform(-1., 1.);
                double const z = scenario == 3 ? 5 : random.uniform(3., 8.);
                double const angle = scenario == 1 ? .2 : .03;
                double const t = scenario == 0 || scenario == 1 ? 0 : scenario == 2 ? 1e-5 : .3;
                double const c = scenario == 0 ? 1 : std::cos(angle), s = scenario == 0 ? 0 : std::sin(angle);
                auto const p = formal_gvins::camera::distort(options.camera, x / z, y / z);
                auto q = formal_gvins::camera::distort(options.camera, (c * x + s * z + t) / (-s * x + c * z),
                                                       y / (-s * x + c * z));
                if (i >= 64)
                {
                    q.x = random.uniform(-.4, .4);
                    q.y = random.uniform(-.3, .3);
                }
                a.keypoints.emplace_back(cv::Point2f(options.camera.fx * p.x + options.camera.cx,
                                                     options.camera.fy * p.y + options.camera.cy),
                                         1);
                b.keypoints.emplace_back(cv::Point2f(options.camera.fx * q.x + options.camera.cx,
                                                     options.camera.fy * q.y + options.camera.cy),
                                         1);
                neighbors.push_back({cv::DMatch(i, i, 10), cv::DMatch(i, (i + 1) % 80, 100)});
            }
            OrbMatchResult matches;
            test.expect(verify_matches(a, b, neighbors, neighbors, options, matches) == VisionStatus::success &&
                            matches.geometry_valid,
                        "temporal", "static, rotation, small baseline, planar and general scenes");
            if (scenario < 3)
            {
                test.expect(matches.rotation_checked, "temporal", "rotation-consistent support receives extra screen");
            }
            std::size_t correct = 0, wrong = 0;
            for (auto const & m : matches.inliers)
            {
                if (m.queryIdx < 64)
                {
                    ++correct;
                }
                else
                {
                    ++wrong;
                }
            }
            if (correct < 60 || wrong > 2)
            {
                std::cerr << "scenario=" << scenario << " seed=" << seed << " correct=" << correct << " wrong=" << wrong
                          << '\n';
            }
            test.expect(correct >= 60 && wrong <= 2, "temporal", "known correspondences and distractors");
            if (scenario == 1)
            {
                test.expect(wrong == 0, "temporal",
                            "supplementary rotation screen rejects distractors in pure-rotation fixtures");
            }
            for (auto & key : a.keypoints)
            {
                key.pt = {320, 240};
            }
            test.expect(verify_matches(a, b, neighbors, neighbors, options, matches) == VisionStatus::success &&
                            !matches.geometry_valid && matches.inliers.empty(),
                        "temporal", "repeated coordinates fail without all-inlier fallback");
        }
    }
    return test.failures() == 0 ? 0 : 1;
}
