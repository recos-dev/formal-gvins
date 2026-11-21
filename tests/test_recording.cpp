/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 */
#include <formal_gvins/dataset/recording.hpp>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <type_traits>

namespace
{
using Json = nlohmann::json;
using formal_gvins::dataset::RecordingSource;
using formal_gvins::io::SensorEvent;
static_assert(std::is_nothrow_move_assignable_v<SensorEvent>);
static_assert(std::is_nothrow_swappable_v<SensorEvent>);
constexpr std::uint64_t epoch = 1606691893000000001ULL;

struct Fixture
{
    std::filesystem::path path;
    Json manifest;
    std::vector<Json> events;

    explicit Fixture(std::filesystem::path directory) : path(std::move(directory))
    {
        std::filesystem::create_directories(path);
        Json header = {{"stamp", {{"sec", 12}, {"nanosec", 345}}}};
        events = {{{"sequence", 0},
                   {"stream_id", 4},
                   {"record_time_ns", epoch},
                   {"payload",
                    {{"header", header},
                     {"width", 2},
                     {"height", 1},
                     {"step", 3},
                     {"encoding", "mono8"},
                     {"is_bigendian", 0},
                     {"data", {{"offset", 0}, {"size", 3}}}}}},
                  {{"sequence", 1},
                   {"stream_id", 8},
                   {"record_time_ns", epoch},
                   {"payload",
                    {{"header", header},
                     {"angular_velocity", {{"x", 1}, {"y", 2}, {"z", 3}}},
                     {"linear_acceleration", {{"x", 4}, {"y", 5}, {"z", {{"$float", "nan"}}}}}}}},
                  {{"sequence", 2},
                   {"stream_id", 9},
                   {"record_time_ns", epoch + 1U},
                   {"payload", {{"satellite_id", std::numeric_limits<std::uint64_t>::max()}}}}};
        manifest = {
            {"format", "formal-gvins.sensor-recording"},
            {"version", 1},
            {"time_basis", "recorded_arrival"},
            {"start_time_ns", epoch},
            {"end_time_ns", epoch + 1U},
            {"event_count", 3},
            {"max_record_bytes", 4194304},
            {"max_image_bytes", 16777216},
            {"streams",
             Json::array({{{"id", 4}, {"topic", "/cam"}, {"schema", "sensor_msgs/msg/Image"}, {"kind", "image"}},
                          {{"id", 8}, {"topic", "/imu"}, {"schema", "sensor_msgs/msg/Imu"}, {"kind", "imu"}},
                          {{"id", 9}, {"topic", "/raw"}, {"schema", "custom/msg/Raw"}, {"kind", "other"}}})},
            {"stream_counts", {{"4", 1}, {"8", 1}, {"9", 1}}}};
    }

    void write()
    {
        {
            std::ofstream file(path / "events.jsonl");
            for (auto const & event : events)
            {
                file << event.dump() << '\n';
            }
        }
        {
            std::ofstream file(path / "images.bin", std::ios::binary);
            file << "abx";
        }
        for (auto const * name : {"events.jsonl", "images.bin"})
        {
            manifest["files"][name]["bytes"] = std::filesystem::file_size(path / name);
        }
        std::ofstream(path / "manifest.json") << manifest.dump();
    }
};

void expect_failure(std::filesystem::path const & path)
{
    bool failed = false;
    try
    {
        RecordingSource source(path);
        SensorEvent event;
        while (source.next(event))
        {
        }
    }
    catch (std::exception const &)
    {
        failed = true;
    }
    assert(failed);
}
void test_pvt(std::filesystem::path const & path)
{
    Fixture fixture(path);
    fixture.manifest["streams"][1]["schema"] = "gnss_comm/msg/GnssPVTSolnMsg";
    fixture.manifest["streams"][1]["kind"] = "gnss_pvt";
    fixture.events[1]["payload"] = Json::parse(R"({"time":{"week":2134,"tow":83911.4},
        "fix_type":3,"valid_fix":false,"diff_soln":true,"carr_soln":2,"num_sv":28,
        "latitude":22.3,"longitude":114.2,"altitude":11.0,"height_msl":12.0,
        "h_acc":0.1,"v_acc":0.2,"p_dop":1.2,"vel_n":1,"vel_e":2,"vel_d":3,"vel_acc":0.05})");
    fixture.write();
    RecordingSource source(path);
    SensorEvent event;
    assert(source.next(event) && source.next(event));
    auto const & pvt = std::get<formal_gvins::io::GnssPvt>(event.measurement);
    assert(pvt.week == 2134U && pvt.tow == 83911.4);
    assert(pvt.latitude == 22.3 && pvt.longitude == 114.2);
    assert(pvt.altitude == 11.0 && pvt.height_msl == 12.0);
    assert((pvt.velocity_ned == std::array<double, 3U>{1., 2., 3.}));
    assert(!pvt.valid_fix && pvt.differential_solution && pvt.carrier_solution == 2U && pvt.satellites == 28U);
    assert(pvt.horizontal_accuracy == 0.1 && pvt.vertical_accuracy == 0.2 && pvt.velocity_accuracy == 0.05);
    assert(formal_gvins::dataset::recording_fields(event) == fixture.events[1]["payload"]);
    // Typed access is independent of retained provenance: there is no second decode.
    event.source_record.clear();
    assert(std::get<formal_gvins::io::GnssPvt>(event.measurement).latitude == 22.3);
    fixture.events[1]["payload"]["num_sv"] = 256U;
    fixture.write();
    expect_failure(path);
}
} // namespace

int main(int argc, char ** argv)
{
    assert(argc == 2);
    std::filesystem::path const path(argv[1]);
    test_pvt(path);
    Fixture fixture(path);
    fixture.write();
    RecordingSource source(path);
    assert(source.start_time_ns() == epoch);
    SensorEvent event;
    assert(source.next(event));
    auto const & image = std::get<formal_gvins::io::ImageMeasurement>(event.measurement);
    assert(image.time_ns == 12000000345ULL && image.step == 3U);
    assert(image.data == std::vector<std::uint8_t>({'a', 'b', 'x'}));
    assert(source.next(event));
    auto const & imu = std::get<formal_gvins::io::ImuMeasurement>(event.measurement);
    assert(imu.angular_velocity[1] == 2.0 && std::isnan(imu.linear_acceleration[2]));
    assert(source.next(event));
    assert(formal_gvins::dataset::recording_fields(event).at("satellite_id").get<std::uint64_t>() ==
           std::numeric_limits<std::uint64_t>::max());
    assert(std::holds_alternative<formal_gvins::io::AuxiliaryMeasurement>(event.measurement));
    auto const saved = event.source_record;
    assert(!source.next(event) && !source.next(event) && event.source_record == saved);

    enum class Corruption
    {
        sequence,
        reversed_time,
        image_size,
        image_offset,
        stream_id,
        count,
        missing_event
    };
    for (auto const failure :
         {Corruption::sequence, Corruption::reversed_time, Corruption::image_size, Corruption::image_offset,
          Corruption::stream_id, Corruption::count, Corruption::missing_event})
    {
        Fixture bad(path);
        switch (failure)
        {
        case Corruption::sequence:
            bad.events[1]["sequence"] = 2;
            break;
        case Corruption::reversed_time:
            bad.events[1]["record_time_ns"] = epoch - 1U;
            break;
        case Corruption::image_size:
            bad.events[0]["payload"]["data"]["size"] = 16777217;
            break;
        case Corruption::image_offset:
            bad.events[0]["payload"]["data"]["offset"] = 1;
            break;
        case Corruption::stream_id:
            bad.events[1]["stream_id"] = -1;
            break;
        case Corruption::count:
            bad.manifest["stream_counts"]["8"] = 2;
            break;
        case Corruption::missing_event:
            bad.events.pop_back();
            break;
        }
        bad.write();
        expect_failure(path);
    }
    fixture.write();
    RecordingSource failing(path);
    // Files are immutable by contract; simulate an I/O truncation to check atomic failure.
    std::ofstream(path / "events.jsonl", std::ios::trunc) << "{broken";
    bool failed = false;
    try
    {
        failing.next(event);
    }
    catch (std::exception const &)
    {
        failed = true;
    }
    assert(failed && event.source_record == saved);
    failed = false;
    try
    {
        failing.next(event);
    }
    catch (std::exception const &)
    {
        failed = true;
    }
    assert(failed);
    std::cout << "Recording source: pass\n";
}
