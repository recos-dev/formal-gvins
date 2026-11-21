/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/dataset/euroc.hpp>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace formal_gvins::dataset
{
namespace
{

std::string_view trim(std::string_view text)
{
    auto const first = text.find_first_not_of(" \t\r");
    if (first == std::string_view::npos)
    {
        return {};
    }
    return text.substr(first, text.find_last_not_of(" \t\r") - first + 1U);
}

template <typename Value> Value number(std::string_view text)
{
    text = trim(text);
    Value value{};
    auto const result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || !std::isfinite(value))
    {
        throw std::runtime_error("Invalid CSV number: " + std::string(text));
    }
    return value;
}

class CsvReader
{
public:
    CsvReader(std::filesystem::path const & path, std::size_t columns) : m_input(path), m_columns(columns)
    {
        if (!m_input)
        {
            throw std::runtime_error("Cannot open " + path.string());
        }
    }

    bool next()
    {
        while (std::getline(m_input, m_line))
        {
            std::string_view line = trim(m_line);
            if (line.empty() || line.front() == '#')
            {
                continue;
            }
            m_fields.clear();
            for (;;)
            {
                auto const comma = line.find(',');
                m_fields.push_back(trim(line.substr(0, comma)));
                if (comma == std::string_view::npos)
                {
                    break;
                }
                line.remove_prefix(comma + 1U);
            }
            if (m_fields.size() != m_columns)
            {
                throw std::runtime_error("Unexpected CSV column count");
            }
            auto const current = number<std::uint64_t>(m_fields[0]);
            if (m_seen && current <= m_time)
            {
                throw std::runtime_error("CSV timestamps must be strictly increasing");
            }
            m_seen = true;
            m_time = current;
            return true;
        }
        if (!m_input.eof())
        {
            throw std::runtime_error("CSV read failed");
        }
        return false;
    }

    std::uint64_t time() const { return m_time; }
    std::string_view field(std::size_t index) const { return m_fields.at(index); }
    cv::Vec3d vector(std::size_t first) const
    {
        return {number<double>(field(first)), number<double>(field(first + 1U)), number<double>(field(first + 2U))};
    }

private:
    std::ifstream m_input;
    std::size_t m_columns;
    std::string m_line;
    std::vector<std::string_view> m_fields;
    std::uint64_t m_time = 0;
    bool m_seen = false;
};

cv::FileStorage yaml(std::filesystem::path const & path)
{
    std::ifstream input(path);
    if (!input)
    {
        throw std::runtime_error("Cannot open " + path.string());
    }
    std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    if (text.rfind("%YAML", 0) != 0)
    {
        text = "%YAML:1.0\n---\n" + text;
    }
    return cv::FileStorage(text, cv::FileStorage::READ | cv::FileStorage::MEMORY);
}

std::vector<double> coefficients(cv::FileNode const & node, std::size_t count)
{
    std::vector<double> values;
    node >> values;
    if (values.size() != count)
    {
        throw std::runtime_error("Invalid calibration coefficient count");
    }
    for (double value : values)
    {
        if (!std::isfinite(value))
        {
            throw std::runtime_error("Non-finite calibration");
        }
    }
    return values;
}

cv::Matx44d transform(cv::FileNode const & node)
{
    if (static_cast<int>(node["rows"]) != 4 || static_cast<int>(node["cols"]) != 4)
    {
        throw std::runtime_error("Expected 4x4 T_BS");
    }
    auto const values = coefficients(node["data"], 16);
    cv::Matx44d T;
    std::copy(values.begin(), values.end(), T.val);
    cv::Matx33d const R = T.get_minor<3, 3>(0, 0);
    if (cv::norm(R.t() * R - cv::Matx33d::eye()) > 1e-6 || std::abs(cv::determinant(R) - 1.0) > 1e-6 || T(3, 0) != 0 ||
        T(3, 1) != 0 || T(3, 2) != 0 || T(3, 3) != 1)
    {
        throw std::runtime_error("T_BS is not a rigid transform");
    }
    return T;
}

double positive(cv::FileStorage const & config, char const * name)
{
    double const value = static_cast<double>(config[name]);
    if (!std::isfinite(value) || value <= 0)
    {
        throw std::runtime_error(std::string("Invalid calibration field: ") + name);
    }
    return value;
}

CameraCalibration read_camera(std::filesystem::path const & path)
{
    auto const config = yaml(path);
    if (static_cast<std::string>(config["camera_model"]) != "pinhole" ||
        static_cast<std::string>(config["distortion_model"]) != "radial-tangential")
    {
        throw std::runtime_error("Expected EuRoC pinhole/radial-tangential calibration");
    }
    CameraCalibration result;
    auto const resolution = coefficients(config["resolution"], 2);
    for (double value : resolution)
    {
        if (value < 1 || value > 16384 || std::floor(value) != value)
        {
            throw std::runtime_error("Invalid image resolution");
        }
    }
    result.resolution = {static_cast<int>(resolution[0]), static_cast<int>(resolution[1])};
    auto const intrinsics = coefficients(config["intrinsics"], 4);
    if (intrinsics[0] <= 0 || intrinsics[1] <= 0)
    {
        throw std::runtime_error("Invalid focal length");
    }
    auto const distortion = coefficients(config["distortion_coefficients"], 4);
    result.intrinsics = {intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
                         distortion[0], distortion[1], distortion[2], distortion[3]};
    result.T_BC = transform(config["T_BS"]);
    result.valid_mask = cv::Mat(result.resolution, CV_8UC1, cv::Scalar(255));
    return result;
}

} /* end anonymous namespace */

EurocSequence EurocSequence::load(std::filesystem::path const & mav0)
{
    EurocSequence result;
    auto const imu_config = yaml(mav0 / "imu0/sensor.yaml");
    cv::Matx44d const T_BI = transform(imu_config["T_BS"]);
    // The state_groundtruth_estimate0 CSV refers to the dataset body frame.
    // EuRoC's reviewed profile makes that frame coincide with the IMU.
    if (cv::norm(T_BI - cv::Matx44d::eye()) > 1e-9)
    {
        throw std::runtime_error("Nonidentity IMU/body extrinsic needs an explicit ground-truth conversion");
    }
    auto const truth_config = yaml(mav0 / "state_groundtruth_estimate0/sensor.yaml");
    if (static_cast<std::string>(truth_config["sensor_type"]) != "visual-inertial" ||
        cv::norm(transform(truth_config["T_BS"]) - cv::Matx44d::eye()) > 1e-9)
    {
        throw std::runtime_error("Expected processed ground truth in the IMU/body frame");
    }
    auto & noise = result.m_imu_calibration;
    noise.rate_hz = positive(imu_config, "rate_hz");
    noise.gyro_noise_density = positive(imu_config, "gyroscope_noise_density");
    noise.accel_noise_density = positive(imu_config, "accelerometer_noise_density");
    noise.gyro_random_walk = positive(imu_config, "gyroscope_random_walk");
    noise.accel_random_walk = positive(imu_config, "accelerometer_random_walk");
    for (std::size_t index = 0; index < 2; ++index)
    {
        auto const directory = mav0 / ("cam" + std::to_string(index));
        result.m_cameras[index] = read_camera(directory / "sensor.yaml");
        CsvReader rows(directory / "data.csv", 2);
        while (rows.next())
        {
            std::filesystem::path const name(rows.field(1));
            if (name.empty() || name != name.filename() || name == "." || name == "..")
            {
                throw std::runtime_error("Image filename must stay inside the camera data directory");
            }
            result.m_images[index].push_back({rows.time(), directory / "data" / name});
        }
        if (result.m_images[index].empty())
        {
            throw std::runtime_error("Empty camera stream");
        }
    }
    CsvReader imu_rows(mav0 / "imu0/data.csv", 7);
    while (imu_rows.next())
    {
        result.m_imu.push_back({imu_rows.time(), imu_rows.vector(1), imu_rows.vector(4)});
    }
    CsvReader truth_rows(mav0 / "state_groundtruth_estimate0/data.csv", 17);
    while (truth_rows.next())
    {
        GroundTruth truth;
        truth.time_ns = truth_rows.time();
        truth.position_R = truth_rows.vector(1);
        truth.q_RB = {number<double>(truth_rows.field(4)), number<double>(truth_rows.field(5)),
                      number<double>(truth_rows.field(6)), number<double>(truth_rows.field(7))};
        // Preserve publisher coefficients. A later scoring adapter owns any
        // reviewed normalization; the CSV is not an exact unit-quaternion source.
        double const quaternion_norm = cv::norm(truth.q_RB);
        if (!std::isfinite(quaternion_norm) || quaternion_norm == 0)
        {
            throw std::runtime_error("Invalid ground-truth quaternion coefficients");
        }
        truth.velocity_R = truth_rows.vector(8);
        truth.gyro_bias = truth_rows.vector(11);
        truth.accel_bias = truth_rows.vector(14);
        result.m_ground_truth.push_back(truth);
    }
    if (result.m_imu.empty() || result.m_ground_truth.empty())
    {
        throw std::runtime_error("Empty IMU or ground-truth stream");
    }
    return result;
}

cv::Mat EurocSequence::read_image(std::size_t camera_index, std::size_t image_index) const
{
    auto const & calibration = camera(camera_index);
    auto const & sample = images(camera_index).at(image_index);
    cv::Mat const raw = cv::imread(sample.path.string(), cv::IMREAD_UNCHANGED);
    if (raw.empty() || raw.type() != CV_8UC1 || raw.size() != calibration.resolution)
    {
        throw std::runtime_error("Invalid image: " + sample.path.string());
    }
    return raw;
}

} /* end namespace formal_gvins::dataset */
