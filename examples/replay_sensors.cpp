/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 */
#include <formal_gvins/dataset/recording.hpp>
#include <formal_gvins/io/replay.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>

int main(int argc, char ** argv)
{
    try
    {
        if (argc < 2 || argc > 4)
        {
            throw std::runtime_error("Usage: replay_sensors RECORDING [--inspect | --timing FILE.csv]");
        }
        bool inspect = false;
        std::ofstream timing;
        if (argc == 3 && std::string(argv[2]) == "--inspect")
        {
            inspect = true;
        }
        else if (argc == 4 && std::string(argv[2]) == "--timing")
        {
            // Refuse to overwrite an existing timing report.
            if (std::filesystem::exists(argv[3]))
            {
                throw std::runtime_error("Timing output already exists");
            }
            timing.open(argv[3]);
            if (!timing)
            {
                throw std::runtime_error("Cannot open timing output");
            }
            timing << "sequence,stream_id,record_time_ns,scheduled_ns,elapsed_ns,lateness_ns\n";
        }
        else if (argc != 2)
        {
            throw std::runtime_error("Invalid replay arguments");
        }

        formal_gvins::dataset::RecordingSource source(argv[1]);
        std::map<std::uint32_t, std::uint64_t> counts;
        std::chrono::nanoseconds maximum_lateness{0};
        if (inspect)
        {
            formal_gvins::io::SensorEvent event;
            while (source.next(event))
            {
                ++counts[event.stream_id];
            }
        }
        else
        {
            formal_gvins::io::Replay replay(source);
            replay.start();
            std::cout << "Local replay started at 1x recorded arrival time\n" << std::flush;
            formal_gvins::io::Publication publication;
            while (replay.next(publication))
            {
                // This is the local publication boundary. An application may forward
                // the owning event to its input queues here; no estimator is run.
                auto const & event = publication.event;
                ++counts[event.stream_id];
                maximum_lateness = std::max(maximum_lateness, publication.lateness);
                if (timing.is_open())
                {
                    timing << event.sequence << ',' << event.stream_id << ',' << event.record_time_ns << ','
                           << publication.scheduled.count() << ',' << publication.elapsed.count() << ','
                           << publication.lateness.count() << '\n';
                    if (!timing)
                    {
                        throw std::runtime_error("Timing output failed");
                    }
                }
            }
            std::cout << "Maximum delivery lateness: " << maximum_lateness.count() << " ns\n";
        }
        for (auto const & stream : source.streams())
        {
            std::cout << stream.topic << ": " << counts[stream.id] << " (" << stream.kind << ")\n";
        }
        if (timing.is_open())
        {
            timing.close();
            if (!timing)
            {
                throw std::runtime_error("Timing output failed on close");
            }
        }
        return 0;
    }
    catch (std::exception const & error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
