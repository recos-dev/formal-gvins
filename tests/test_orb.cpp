/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/vision/orb.hpp>
#include "test_support.hpp"

int main(int argc, char ** argv)
{
    using namespace formal_gvins::vision;
    formal_gvins::test::Context test;
    cv::setNumThreads(1);
    cv::Mat image(480, 752, CV_8UC1);
    cv::RNG random(17);
    random.fill(image, cv::RNG::UNIFORM, 0, 256);
    OrbOptions options;
    options.execution = argc == 2 && std::string_view(argv[1]) == "--cuda" ? Execution::cuda : Execution::cpu;
    OrbExtractor extractor(options);
    OrbFeatures features;
    test.expect(extractor.extract(image, {}, features) == VisionStatus::success, "ORB", "extract");
    test.expect(!features.keypoints.empty() && features.keypoints.size() <= 1000U, "ORB", "bounded features");
    test.expect(features.descriptors.rows == static_cast<int>(features.keypoints.size()) &&
                    features.descriptors.cols == 32 && features.descriptors.type() == CV_8UC1,
                "ORB", "paired binary descriptors");
    auto const count = features.keypoints.size();
    std::vector<std::vector<cv::DMatch>> matches;
    test.expect(extractor.match(features, features, matches) == VisionStatus::success && matches.size() == count, "ORB",
                "two-nearest matching");
    for (auto const & neighbors : matches)
    {
        test.expect(neighbors.size() == 2U && neighbors[0].distance == 0.0F, "ORB", "Hamming self match");
    }
    test.expect(extractor.extract({}, {}, features) == VisionStatus::invalid_input &&
                    features.keypoints.size() == count,
                "ORB", "invalid input preserves output");
    test.expect(extractor.extract(image, cv::Mat::zeros(5, 5, CV_8UC1), features) == VisionStatus::invalid_input, "ORB",
                "mask dimensions checked");
    test.expect(extractor.extract(cv::Mat::zeros(480, 752, CV_8UC3), {}, features) == VisionStatus::invalid_input,
                "ORB", "color image rejected");
    OrbFeatures known;
    known.keypoints.resize(3);
    known.descriptors = cv::Mat(3, 32, CV_8UC1);
    known.descriptors.row(0).setTo(0);
    known.descriptors.row(1).setTo(255);
    known.descriptors.row(2).setTo(15);
    test.expect(extractor.match(known, known, matches) == VisionStatus::success, "ORB", "known descriptors");
    for (auto const & neighbors : matches)
    {
        test.expect(neighbors.size() == 2U && neighbors[0].distance == 0.0F && neighbors[1].distance == 128.0F, "ORB",
                    "binary Hamming distance (not L2)");
    }
    OrbFeatures one;
    one.keypoints.resize(1);
    one.descriptors = known.descriptors.row(0).clone();
    test.expect(extractor.match(known, one, matches) == VisionStatus::success && matches.empty(), "ORB",
                "insufficient second neighbor");
    cv::Mat const blank = cv::Mat::zeros(image.size(), CV_8UC1);
    test.expect(extractor.extract(blank, {}, features) == VisionStatus::success && features.keypoints.empty(), "ORB",
                "blank image produces empty observations");
    test.expect(extractor.extract(image, blank, features) == VisionStatus::success && features.keypoints.empty(), "ORB",
                "mask excludes all features");
#if !defined(FORMAL_GVINS_FRONTEND_CUDA)
    options.execution = Execution::cuda;
    OrbExtractor unavailable(options);
    test.expect(unavailable.extract(image, {}, features) == VisionStatus::backend_unavailable, "ORB",
                "CPU build does not silently replace requested CUDA execution");
#endif
    return test.failures() == 0 ? 0 : 1;
}
