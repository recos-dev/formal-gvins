/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/dataset/euroc.hpp>
#include "test_support.hpp"
#include <chrono>
#include <fstream>
#include <opencv2/imgcodecs.hpp>

namespace
{
using formal_gvins::dataset::EurocSequence;

void write(std::filesystem::path const & path, std::string const & text)
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path) << text;
}

bool rejects(std::filesystem::path const & path)
{
    try
    {
        EurocSequence::load(path);
        return false;
    }
    catch (std::exception const &)
    {
        return true;
    }
}

void fixture(std::filesystem::path const & root)
{
    std::string const camera =
        "camera_model: pinhole\ndistortion_model: radial-tangential\n"
        "resolution: [80, 64]\nintrinsics: [50.0, 50.0, 40.0, 32.0]\n"
        "distortion_coefficients: [-0.2, 0.04, 0.001, -0.002]\n"
        "T_BS:\n  rows: 4\n  cols: 4\n"
        "  data: [0.0, -1.0, 0.0, 0.12, 1.0, 0.0, 0.0, -0.03, 0.0, 0.0, 1.0, 0.02, 0.0, 0.0, 0.0, 1.0]\n";
    for (std::string const name : {"cam0", "cam1"})
    {
        write(root / name / "sensor.yaml", camera);
        write(root / name / "data.csv", "# timestamp, filename\n1400000000000000001,first.png\n");
        std::filesystem::create_directories(root / name / "data");
        cv::Mat image(64, 80, CV_8UC1);
        cv::RNG(17).fill(image, cv::RNG::UNIFORM, 0, 256);
        cv::imwrite((root / name / "data/first.png").string(), image);
    }
    write(root / "imu0/sensor.yaml",
          "rate_hz: 200\ngyroscope_noise_density: 0.0001\naccelerometer_noise_density: 0.002\n"
          "gyroscope_random_walk: 0.00001\naccelerometer_random_walk: 0.0001\n"
          "T_BS:\n  rows: 4\n  cols: 4\n"
          "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n");
    write(root / "imu0/data.csv", "1400000000000000000,1,2,3,4,5,6\n1400000000000000001,1,2,3,4,5,6\n");
    write(root / "state_groundtruth_estimate0/data.csv",
          "1400000000000000000,1,2,3,0.5,0.5,0.5,0.5,4,5,6,7,8,9,10,11,12\n");
    write(root / "state_groundtruth_estimate0/sensor.yaml",
          "sensor_type: visual-inertial\nT_BS:\n  rows: 4\n  cols: 4\n"
          "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n");
}
} /* end anonymous namespace */

int main(int argc, char ** argv)
{
    formal_gvins::test::Context test;
    if (argc == 3 && std::string_view(argv[1]) == "--dataset")
    {
        try
        {
            auto const sequence = EurocSequence::load(argv[2]);
            for (std::size_t camera = 0; camera < 2; ++camera)
            {
                sequence.read_image(camera, 0);
                sequence.read_image(camera, sequence.images(camera).size() - 1);
                std::cout << "cam" << camera << ": " << sequence.images(camera).size() << " images\n";
            }
            std::cout << "IMU: " << sequence.imu().size() << "; GT: " << sequence.ground_truth().size() << '\n';
            return 0;
        }
        catch (std::exception const & error)
        {
            std::cerr << error.what() << '\n';
            return 1;
        }
    }
    auto const root =
        std::filesystem::temp_directory_path() /
        ("formal-gvins-euroc-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fixture(root);
    try
    {
        auto const sequence = EurocSequence::load(root);
        test.expect(sequence.imu()[1].time_ns - sequence.imu()[0].time_ns == 1, "EuRoC", "nanosecond precision");
        test.expect(sequence.imu()[0].angular_rate[2] == 3 && sequence.imu()[0].specific_force[2] == 6, "EuRoC",
                    "IMU column order");
        auto const & truth = sequence.ground_truth()[0];
        test.expect(truth.q_RB[0] == 0.5 && truth.velocity_R[0] == 4 && truth.gyro_bias[0] == 7 &&
                        truth.accel_bias[0] == 10,
                    "EuRoC", "GT Hamilton order and state columns");
        cv::Vec4d const p_B = sequence.camera(0).T_BC * cv::Vec4d(1, 0, 0, 1);
        test.expect(cv::norm(p_B - cv::Vec4d(0.12, 0.97, 0.02, 1)) < 1e-12, "EuRoC", "T_BC direction and lever arm");
        auto const raw = cv::imread((root / "cam0/data/first.png").string(), cv::IMREAD_UNCHANGED);
        test.expect(cv::norm(raw, sequence.read_image(0, 0), cv::NORM_INF) == 0, "EuRoC",
                    "nonzero-distortion pixels remain raw");
        write(root / "imu0/data.csv", "1400000000000000001,1,2,3,4,5,6\n1400000000000000001,1,2,3,4,5,6\n");
        test.expect(rejects(root), "EuRoC", "duplicate timestamp");
        write(root / "imu0/data.csv", "1400000000000000001,1,2,3,4,5,nan\n");
        test.expect(rejects(root), "EuRoC", "non-finite sample");
        fixture(root);
        write(root / "cam0/data.csv", "1400000000000000001,../escape.png\n");
        test.expect(rejects(root), "EuRoC", "path traversal");
        fixture(root);
        write(root / "state_groundtruth_estimate0/data.csv",
              "1400000000000000000,1,2,3,0,0,0,0,4,5,6,7,8,9,10,11,12\n");
        test.expect(rejects(root), "EuRoC", "invalid quaternion");
        write(root / "state_groundtruth_estimate0/data.csv",
              "1400000000000000000,1,2,3,1.00004,0,0,0,4,5,6,7,8,9,10,11,12\n");
        test.expect(EurocSequence::load(root).ground_truth()[0].q_RB[0] == 1.00004, "EuRoC",
                    "raw rounded quaternion is not silently normalized");
        write(root / "state_groundtruth_estimate0/sensor.yaml",
              "sensor_type: visual-inertial\nT_BS:\n  rows: 4\n  cols: 4\n"
              "  data: [1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n");
        test.expect(rejects(root), "EuRoC", "unsupported ground-truth/body offset");
    }
    catch (std::exception const & error)
    {
        test.expect(false, "EuRoC", error.what());
    }
    std::filesystem::remove_all(root);
    return test.failures() == 0 ? 0 : 1;
}
