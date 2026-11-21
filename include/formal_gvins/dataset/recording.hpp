/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 */
#pragma once
#include <formal_gvins/io/source.hpp>
#include <filesystem>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>

namespace formal_gvins::dataset
{
/** Streaming reader for convert_hkust.py output. Owns file handles and returns
 * owning events. Malformed/truncated input is a terminal error, never EOF.
 * Files must remain immutable during use; hashes in manifest identify evidence,
 * but this reader does not cryptographically authenticate payloads. */
class RecordingSource final : public io::SensorSource
{
public:
    explicit RecordingSource(std::filesystem::path const & directory);
    std::uint64_t start_time_ns() const override;
    std::vector<io::SensorStream> const & streams() const override;
    bool next(io::SensorEvent & output) override;
    nlohmann::json const & manifest() const;

private:
    struct StreamProgress
    {
        std::uint64_t expected = 0U;
        std::uint64_t read = 0U;
    };

    void load_manifest(std::filesystem::path const & directory);
    void open_data(std::filesystem::path const & directory);
    io::SensorEvent read_event(std::string line);
    io::ImageMeasurement read_image(nlohmann::json const & fields);
    void check_complete() const;

    nlohmann::json m_manifest;
    std::vector<io::SensorStream> m_streams;
    std::map<std::uint32_t, std::size_t> m_indices;
    std::vector<StreamProgress> m_progress;
    std::uint64_t m_start_time = 0U;
    std::uint64_t m_end_time = 0U;
    std::uint64_t m_event_count = 0U;
    std::ifstream m_events;
    std::ifstream m_images;
    std::uint64_t m_image_size = 0U;
    std::uint64_t m_next_image = 0U;
    std::uint64_t m_sequence = 0U;
    std::uint64_t m_last_time = 0U;
    bool m_failed = false;
};

/** Decode the complete original payload on demand, including fields outside the
 * typed measurement. Requires an event produced by RecordingSource; throws if
 * its retained record is absent/malformed. Does not affect source state. */
nlohmann::json recording_fields(io::SensorEvent const & event);

} // namespace formal_gvins::dataset
