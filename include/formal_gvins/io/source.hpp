/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 */
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <variant>

namespace formal_gvins::io
{

struct SensorStream
{
    std::uint32_t id = 0U;
    std::string topic;
    std::string schema;
    std::string kind;
};

struct ImuMeasurement
{
    std::uint64_t time_ns = 0U;
    std::array<double, 3U> angular_velocity{};    // rad/s, original sensor frame
    std::array<double, 3U> linear_acceleration{}; // m/s^2, original sensor frame
};

struct GnssPvt
{
    std::uint32_t week = 0U;
    double tow = 0.0;                      // Native GNSS time, seconds; not a converted camera clock.
    double latitude = 0.0;                 // degrees
    double longitude = 0.0;                // degrees
    double altitude = 0.0;                 // ellipsoid height, m
    double height_msl = 0.0;               // m
    std::array<double, 3U> velocity_ned{}; // m/s
    double horizontal_accuracy = 0.0;      // m, receiver report, not covariance
    double vertical_accuracy = 0.0;        // m
    double velocity_accuracy = 0.0;        // m/s
    double position_dop = 0.0;
    std::uint8_t fix_type = 0U;
    std::uint8_t carrier_solution = 0U;
    std::uint8_t satellites = 0U;
    bool valid_fix = false;
    bool differential_solution = false;
};

struct ImageMeasurement
{
    std::uint64_t time_ns = 0U;
    std::uint32_t width = 0U;
    std::uint32_t height = 0U;
    std::uint32_t step = 0U;
    std::string encoding;
    bool big_endian = false;
    std::vector<std::uint8_t> data; // Original bytes, including row padding.
};

/** A stream without a common typed measurement. Its native fields remain
 * available through the source-specific record accessor. */
struct AuxiliaryMeasurement
{
};

using SensorMeasurement = std::variant<AuxiliaryMeasurement, ImuMeasurement, ImageMeasurement, GnssPvt>;

/** Owning host event. Known measurements are decoded once by the source.
 * source_record is optional source-specific provenance/auxiliary data; its
 * encoding and accessor belong to the adapter, not the numerical consumer. */
struct SensorEvent
{
    std::uint64_t sequence = 0U;
    std::uint64_t record_time_ns = 0U;
    std::uint32_t stream_id = 0U;
    SensorMeasurement measurement;
    std::string source_record;
};

/** Single-pass, single-owner host source. False means EOF; errors throw.
 * next() preserves its output on EOF/error. Source errors are terminal.
 * Live sources can supply typed events without a serialized source_record. */
class SensorSource
{
public:
    virtual ~SensorSource() = default;
    virtual std::uint64_t start_time_ns() const = 0;
    virtual std::vector<SensorStream> const & streams() const = 0;
    virtual bool next(SensorEvent & output) = 0;
};

} // namespace formal_gvins::io
