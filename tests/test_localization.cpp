/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/orb.hpp>
#include "test_support.hpp"
#include <opencv2/imgproc.hpp>

cv::Mat corner(double angle, int contrast, double blur, cv::Point2f center)
{
    cv::Mat image(180, 180, CV_8UC1);
    for (int y = 0; y < image.rows; ++y)
    {
        for (int x = 0; x < image.cols; ++x)
        {
            double value = 0;
            for (int sy = 0; sy < 8; ++sy)
            {
                for (int sx = 0; sx < 8; ++sx)
                {
                    double const dx = x + (sx + .5) / 8 - .5 - center.x, dy = y + (sy + .5) / 8 - .5 - center.y;
                    double const a = std::cos(angle) * dx + std::sin(angle) * dy;
                    double const b = -std::sin(angle) * dx + std::cos(angle) * dy;
                    value += a * b > 0 ? 128 + contrast : 128 - contrast;
                }
            }
            image.at<unsigned char>(y, x) = cv::saturate_cast<unsigned char>(value / 64);
        }
    }
    cv::GaussianBlur(image, image, {7, 7}, blur);
    return image;
}

int main()
{
    using namespace formal_gvins::vision;
    formal_gvins::test::Context test;
    cv::setNumThreads(1);
    OrbOptions options;
    cv::Point2f const truth(80.25F, 81.625F);
    std::vector<cv::KeyPoint> detected{cv::KeyPoint({80, 82}, 31)}, output;
    LocalizationStats stats;
    double squared = 0, worst = 0;
    int count = 0;
    for (double angle : {0.0, .3, .7, 1.2})
    {
        for (int contrast : {40, 120})
        {
            for (double blur : {.7, 1.2})
            {
                auto const image = corner(angle, contrast, blur, truth);
                test.expect(localize_orb(image, {}, detected, options, output, stats) == VisionStatus::success &&
                                output.size() == 1,
                            "localization", "known corner admitted");
                if (output.size() == 1)
                {
                    double const error = cv::norm(output[0].pt - truth);
                    squared += error * error;
                    worst = std::max(worst, error);
                    ++count;
                    test.expect(error < .15, "localization", "absolute corner error below 0.15 raw pixels");
                }
            }
        }
    }
    std::cout << "Known-corner cases=" << count << " RMSE_px=" << std::sqrt(squared / count) << " max_px=" << worst
              << '\n';
    auto const image = corner(.3, 120, .7, truth);
    auto coarse = detected;
    coarse[0].octave = 1;
    coarse[0].size = 31 * options.scale_factor;
    test.expect(localize_orb(image, {}, coarse, options, output, stats) == VisionStatus::success &&
                    output.size() == 1 && cv::norm(output[0].pt - truth) < .3,
                "localization", "octave conversion retains raw coordinates");
    cv::Mat mask(image.size(), CV_8UC1, cv::Scalar(255));
    mask.at<unsigned char>(80, 65) = 0;
    test.expect(localize_orb(image, mask, detected, options, output, stats) == VisionStatus::success &&
                    output.empty() && stats.support_rejected == 1,
                "localization", "mask covers full support, not only center");
    test.expect(localize_orb(cv::Mat(image.size(), CV_8UC1, cv::Scalar(255)), {}, detected, options, output, stats) ==
                        VisionStatus::success &&
                    output.empty() && stats.quality_rejected == 1,
                "localization", "flat saturated patch rejected");
    cv::Mat edge = cv::Mat::zeros(image.size(), CV_8UC1);
    edge.colRange(80, 180).setTo(255);
    test.expect(localize_orb(edge, {}, detected, options, output, stats) == VisionStatus::success && output.empty(),
                "localization", "edge without corner rejected");
    coarse = detected;
    coarse[0].pt = {83, 82};
    test.expect(localize_orb(image, {}, coarse, options, output, stats) == VisionStatus::success && output.empty() &&
                    stats.displacement_rejected == 1,
                "localization", "excessive raw displacement rejected");
    coarse = detected;
    coarse[0].pt = {10, 10};
    test.expect(localize_orb(image, {}, coarse, options, output, stats) == VisionStatus::success && output.empty() &&
                    stats.support_rejected == 1,
                "localization", "border rejected");
    options.localization.iterations = 1;
    test.expect(localize_orb(image, {}, detected, options, output, stats) == VisionStatus::success, "localization",
                "bounded invocation without convergence claim");
    auto saved = output;
    auto saved_stats = stats;
    options.localization.iterations = 0;
    test.expect(localize_orb(image, {}, detected, options, output, stats) == VisionStatus::invalid_input &&
                    output.size() == saved.size() && stats.detected == saved_stats.detected,
                "localization", "invalid options preserve outputs");
    return test.failures() == 0 ? 0 : 1;
}
