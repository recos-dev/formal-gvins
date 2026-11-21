/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 */
#include <formal_gvins/dataset/recording.hpp>
#include <formal_gvins/io/replay.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

int main(int argc, char ** argv)
{
    assert(argc == 2);
    constexpr std::uint64_t epoch = 1606691893000000001ULL;
    constexpr std::array<std::int64_t, 4U> offsets{0, 0, 1000000, 1000000};
    std::array<unsigned, 4U> types_seen{};
    formal_gvins::io::SensorEvent retained_image;
    {
        formal_gvins::dataset::RecordingSource source(argv[1]);
        assert(source.streams().size() == 4U && source.start_time_ns() == epoch);
        formal_gvins::io::Replay replay(source, {2U, std::chrono::nanoseconds{0}});
        replay.start();
        formal_gvins::io::Publication publication;
        std::size_t count = 0U;
        while (replay.next(publication))
        {
            auto & event = publication.event;
            assert(count < offsets.size() && event.sequence == count);
            assert(event.record_time_ns == epoch + static_cast<std::uint64_t>(offsets[count]));
            assert(publication.scheduled.count() == offsets[count]);
            assert(publication.elapsed >= publication.scheduled);
            assert(publication.lateness == publication.elapsed - publication.scheduled);
            // Access every native payload through the dataset layer, independently
            // of the common typed measurement consumed below.
            auto const fields = formal_gvins::dataset::recording_fields(event);
            if (auto const * imu = std::get_if<formal_gvins::io::ImuMeasurement>(&event.measurement))
            {
                ++types_seen[0];
                assert(imu->time_ns == 12000000345ULL);
                assert((imu->angular_velocity == std::array<double, 3U>{1., 2., 3.}));
                assert(imu->linear_acceleration[0] == 4. && imu->linear_acceleration[1] == 5.);
                assert(std::isnan(imu->linear_acceleration[2]));
                assert(fields.at("orientation").at("w") == 1.0);
            }
            else if (auto const * image = std::get_if<formal_gvins::io::ImageMeasurement>(&event.measurement))
            {
                ++types_seen[1];
                assert(image->time_ns == 12000000345ULL);
                assert(image->width == 2U && image->height == 2U && image->step == 3U);
                assert(image->encoding == "mono8" && !image->big_endian);
                assert(image->data == std::vector<std::uint8_t>({1, 2, 99, 3, 4, 88}));
                assert(fields.at("header").at("frame_id") == "imu");
                retained_image = std::move(event);
            }
            else if (auto const * pvt = std::get_if<formal_gvins::io::GnssPvt>(&event.measurement))
            {
                ++types_seen[2];
                assert(pvt->week == 2134U && pvt->tow == 83911.4);
                assert(pvt->latitude == 22.3 && pvt->longitude == 114.2);
                assert(pvt->altitude == 11. && pvt->height_msl == 12.);
                assert((pvt->velocity_ned == std::array<double, 3U>{1., 2., 3.}));
                assert(pvt->fix_type == 3U && !pvt->valid_fix && pvt->differential_solution);
                assert(pvt->carrier_solution == 2U && pvt->satellites == 28U);
                assert(pvt->horizontal_accuracy == .1 && pvt->vertical_accuracy == .2);
                assert(pvt->position_dop == 1.2 && pvt->velocity_accuracy == .05);
            }
            else
            {
                ++types_seen[3];
                assert(std::holds_alternative<formal_gvins::io::AuxiliaryMeasurement>(event.measurement));
                assert(fields.at("id").get<std::uint64_t>() == std::numeric_limits<std::uint64_t>::max());
                assert(fields.at("values").at(0).at("$float") == "+inf");
                assert(fields.at("values").at(1).at("$float") == "-inf");
            }
            ++count;
        }
        assert(count == 4U && (types_seen == std::array<unsigned, 4U>{1U, 1U, 1U, 1U}));
    }
    // The event owns image and provenance even after source/replay destruction.
    assert(std::get<formal_gvins::io::ImageMeasurement>(retained_image.measurement).data.back() == 88U);
    assert(formal_gvins::dataset::recording_fields(retained_image).at("width") == 2U);
    std::cout << "Bag conversion -> typed source -> local publication: pass\n";
}
