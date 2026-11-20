/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <vector>
#include <opencv2/core.hpp>

namespace formal_gvins::dataset
{

struct ImageSample
{
    std::uint64_t time_ns = 0;
    std::filesystem::path path;
};

struct ImuSample
{
    std::uint64_t time_ns = 0;
    cv::Vec3d angular_rate;   // rad/s in the calibrated IMU frame
    cv::Vec3d specific_force; // m/s^2, including gravity response
};

struct GroundTruth
{
    std::uint64_t time_ns = 0;
    cv::Vec3d position_R;
    cv::Vec4d q_RB; // Raw Hamilton [w,x,y,z], B to R; not silently normalized
    cv::Vec3d velocity_R;
    cv::Vec3d gyro_bias;
    cv::Vec3d accel_bias;
};

struct CameraCalibration
{
    cv::Size resolution;
    cv::Matx33d K;
    cv::Vec4d distortion; // radtan [k1,k2,p1,p2]
    cv::Matx44d T_BC;     // Camera optical frame to calibrated IMU frame
    cv::Mat map_x, map_y, valid_mask;
};

struct ImuCalibration
{
    double rate_hz = 0;
    double gyro_noise_density = 0;
    double accel_noise_density = 0;
    double gyro_random_walk = 0;
    double accel_random_walk = 0;
};

/**
 * Host-only EuRoC ASL/CSV reader. Loading throws on malformed/inconsistent input.
 * Timestamps remain integer nanoseconds. No GT interpolation or frame alignment
 * is implicit. Metadata is loaded once; images are read on demand.
 */
class EurocSequence
{
public:
    static EurocSequence load(std::filesystem::path const & mav0);
    CameraCalibration const & camera(std::size_t index) const { return m_cameras.at(index); }
    std::vector<ImageSample> const & images(std::size_t index) const { return m_images.at(index); }
    std::vector<ImuSample> const & imu() const { return m_imu; }
    ImuCalibration const & imu_calibration() const { return m_imu_calibration; }
    std::vector<GroundTruth> const & ground_truth() const { return m_ground_truth; }

    /** Undistorted grayscale pixels with unchanged K and optical axes. */
    cv::Mat read_image(std::size_t camera_index, std::size_t image_index) const;

private:
    std::array<CameraCalibration, 2> m_cameras;
    std::array<std::vector<ImageSample>, 2> m_images;
    std::vector<ImuSample> m_imu;
    ImuCalibration m_imu_calibration;
    std::vector<GroundTruth> m_ground_truth;
};

} /* end namespace formal_gvins::dataset */
