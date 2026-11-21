/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 */
#include <formal_gvins/dataset/recording.hpp>
#include <limits>
#include <stdexcept>
#include <utility>

namespace formal_gvins::dataset
{
namespace
{
constexpr std::size_t max_record_bytes = 4U * 1024U * 1024U;
constexpr std::size_t max_image_bytes = 16U * 1024U * 1024U;

std::uint64_t unsigned_value(nlohmann::json const & value, std::uint64_t maximum)
{
    if (!value.is_number_unsigned() || value.get<std::uint64_t>() > maximum)
    {
        throw std::runtime_error("Expected an unsigned integer in range");
    }
    return value.get<std::uint64_t>();
}

double number(nlohmann::json const & value)
{
    if (value.is_number())
    {
        return value.get<double>();
    }
    if (value.is_object() && value.size() == 1U && value.contains("$float"))
    {
        auto const tag = value.at("$float").get<std::string>();
        if (tag == "nan")
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        if (tag == "+inf")
        {
            return std::numeric_limits<double>::infinity();
        }
        if (tag == "-inf")
        {
            return -std::numeric_limits<double>::infinity();
        }
    }
    throw std::runtime_error("Expected a floating-point field");
}

std::uint64_t header_time(nlohmann::json const & fields)
{
    auto const & stamp = fields.at("header").at("stamp");
    auto const seconds = unsigned_value(stamp.at("sec"), 4294967295ULL);
    auto const nanos = unsigned_value(stamp.at("nanosec"), 999999999U);
    return seconds * 1000000000ULL + nanos;
}

std::array<double, 3U> vector3(nlohmann::json const & fields)
{
    return {number(fields.at("x")), number(fields.at("y")), number(fields.at("z"))};
}

std::uint64_t unsigned_field(nlohmann::json const & object, char const * key)
{
    return unsigned_value(object.at(key), std::numeric_limits<std::uint64_t>::max());
}

bool read_line(std::istream & input, std::string & line)
{
    line.clear();
    char value = 0;
    while (input.get(value))
    {
        if (value == '\n')
        {
            return true;
        }
        if (line.size() == max_record_bytes)
        {
            throw std::runtime_error("Oversized recording record");
        }
        line.push_back(value);
    }
    if (input.bad() || !line.empty())
    {
        throw std::runtime_error("Truncated recording record");
    }
    return false;
}

io::ImuMeasurement decode_imu(nlohmann::json const & fields)
{
    return {header_time(fields), vector3(fields.at("angular_velocity")), vector3(fields.at("linear_acceleration"))};
}

io::GnssPvt decode_pvt(nlohmann::json const & fields)
{
    io::GnssPvt result;
    result.week = static_cast<std::uint32_t>(unsigned_value(fields.at("time").at("week"), 4294967295ULL));
    result.tow = number(fields.at("time").at("tow"));
    result.latitude = number(fields.at("latitude"));
    result.longitude = number(fields.at("longitude"));
    result.altitude = number(fields.at("altitude"));
    result.height_msl = number(fields.at("height_msl"));
    result.velocity_ned = {number(fields.at("vel_n")), number(fields.at("vel_e")), number(fields.at("vel_d"))};
    result.horizontal_accuracy = number(fields.at("h_acc"));
    result.vertical_accuracy = number(fields.at("v_acc"));
    result.velocity_accuracy = number(fields.at("vel_acc"));
    result.position_dop = number(fields.at("p_dop"));
    result.fix_type = static_cast<std::uint8_t>(unsigned_value(fields.at("fix_type"), 255U));
    result.carrier_solution = static_cast<std::uint8_t>(unsigned_value(fields.at("carr_soln"), 255U));
    result.satellites = static_cast<std::uint8_t>(unsigned_value(fields.at("num_sv"), 255U));
    result.valid_fix = fields.at("valid_fix").get<bool>();
    result.differential_solution = fields.at("diff_soln").get<bool>();
    return result;
}

} // namespace

RecordingSource::RecordingSource(std::filesystem::path const & directory)
{
    load_manifest(directory);
    open_data(directory);
}

void RecordingSource::load_manifest(std::filesystem::path const & directory)
{
    auto const path = directory / "manifest.json";
    if (std::filesystem::file_size(path) > max_record_bytes)
    {
        throw std::runtime_error("Oversized recording manifest");
    }
    std::ifstream manifest_file(path);
    m_manifest = nlohmann::json::parse(manifest_file);
    m_start_time = unsigned_field(m_manifest, "start_time_ns");
    m_end_time = unsigned_field(m_manifest, "end_time_ns");
    m_event_count = unsigned_field(m_manifest, "event_count");
    if (m_manifest.at("format") != "formal-gvins.sensor-recording" || unsigned_field(m_manifest, "version") != 1U ||
        m_manifest.at("time_basis") != "recorded_arrival" || m_event_count == 0U || m_start_time > m_end_time ||
        unsigned_field(m_manifest, "max_record_bytes") > max_record_bytes ||
        unsigned_field(m_manifest, "max_image_bytes") > max_image_bytes)
    {
        throw std::runtime_error("Unsupported recording manifest");
    }
    auto const & counts = m_manifest.at("stream_counts");
    for (auto const & item : m_manifest.at("streams"))
    {
        auto const id = unsigned_field(item, "id");
        if (id > std::numeric_limits<std::uint32_t>::max() || m_streams.size() >= 256U ||
            !m_indices.emplace(static_cast<std::uint32_t>(id), m_streams.size()).second)
        {
            throw std::runtime_error("Invalid or duplicate stream ID");
        }
        m_streams.push_back({static_cast<std::uint32_t>(id), item.at("topic").get<std::string>(),
                             item.at("schema").get<std::string>(), item.at("kind").get<std::string>()});
        auto const key = std::to_string(id);
        m_progress.push_back({counts.contains(key) ? unsigned_field(counts, key.c_str()) : 0U, 0U});
    }
}

void RecordingSource::open_data(std::filesystem::path const & directory)
{
    for (auto const * name : {"events.jsonl", "images.bin"})
    {
        if (std::filesystem::file_size(directory / name) != unsigned_field(m_manifest.at("files").at(name), "bytes"))
        {
            throw std::runtime_error("Recording file size mismatch");
        }
    }
    m_image_size = std::filesystem::file_size(directory / "images.bin");
    m_events.open(directory / "events.jsonl", std::ios::binary);
    m_images.open(directory / "images.bin", std::ios::binary);
    if (!m_events || !m_images)
    {
        throw std::runtime_error("Cannot open recording files");
    }
}

std::uint64_t RecordingSource::start_time_ns() const { return m_start_time; }
std::vector<io::SensorStream> const & RecordingSource::streams() const { return m_streams; }
nlohmann::json const & RecordingSource::manifest() const { return m_manifest; }

io::ImageMeasurement RecordingSource::read_image(nlohmann::json const & fields)
{
    auto const & data = fields.at("data");
    auto const offset = unsigned_field(data, "offset");
    auto const size = unsigned_field(data, "size");
    if (offset != m_next_image || size > max_image_bytes || offset > m_image_size || size > m_image_size - offset)
    {
        throw std::runtime_error("Invalid image extent");
    }
    io::ImageMeasurement image;
    image.data.resize(static_cast<std::size_t>(size));
    m_images.read(reinterpret_cast<char *>(image.data.data()), static_cast<std::streamsize>(size));
    if (!m_images)
    {
        throw std::runtime_error("Truncated image buffer");
    }
    image.time_ns = header_time(fields);
    image.width = static_cast<std::uint32_t>(unsigned_value(fields.at("width"), 4294967295ULL));
    image.height = static_cast<std::uint32_t>(unsigned_value(fields.at("height"), 4294967295ULL));
    image.step = static_cast<std::uint32_t>(unsigned_value(fields.at("step"), 4294967295ULL));
    image.encoding = fields.at("encoding").get<std::string>();
    image.big_endian = unsigned_value(fields.at("is_bigendian"), 1U) != 0U;
    if (image.width == 0U || image.height == 0U || image.step == 0U ||
        static_cast<std::uint64_t>(image.height) * image.step != image.data.size())
    {
        throw std::runtime_error("Invalid image layout");
    }
    m_next_image += size;
    return image;
}

io::SensorEvent RecordingSource::read_event(std::string line)
{
    auto const record = nlohmann::json::parse(line);
    io::SensorEvent event;
    event.sequence = unsigned_field(record, "sequence");
    event.record_time_ns = unsigned_field(record, "record_time_ns");
    auto const id = unsigned_value(record.at("stream_id"), std::numeric_limits<std::uint32_t>::max());
    event.stream_id = static_cast<std::uint32_t>(id);
    auto const index = m_indices.at(event.stream_id);
    if (event.sequence != m_sequence || m_sequence >= m_event_count ||
        (m_sequence == 0U && event.record_time_ns != m_start_time) || event.record_time_ns < m_last_time ||
        event.record_time_ns > m_end_time)
    {
        throw std::runtime_error("Invalid recording order");
    }
    auto const & fields = record.at("payload");
    if (!fields.is_object())
    {
        throw std::runtime_error("Expected message fields");
    }
    auto const & schema = m_streams[index].schema;
    if (schema == "sensor_msgs/msg/Image")
    {
        event.measurement = read_image(fields);
    }
    else if (schema == "sensor_msgs/msg/Imu")
    {
        event.measurement = decode_imu(fields);
    }
    else if (schema == "gnss_comm/msg/GnssPVTSolnMsg")
    {
        event.measurement = decode_pvt(fields);
    }
    event.source_record = std::move(line);
    ++m_progress[index].read;
    ++m_sequence;
    m_last_time = event.record_time_ns;
    return event;
}

void RecordingSource::check_complete() const
{
    if (m_sequence != m_event_count || m_last_time != m_end_time || m_next_image != m_image_size)
    {
        throw std::runtime_error("Incomplete recording");
    }
    for (auto const & progress : m_progress)
    {
        if (progress.read != progress.expected)
        {
            throw std::runtime_error("Stream count mismatch");
        }
    }
}

bool RecordingSource::next(io::SensorEvent & output)
{
    if (m_failed)
    {
        throw std::runtime_error("Recording source has failed");
    }
    try
    {
        std::string line;
        if (!read_line(m_events, line))
        {
            check_complete();
            return false;
        }
        output = read_event(std::move(line));
        return true;
    }
    catch (...)
    {
        m_failed = true;
        throw;
    }
}

nlohmann::json recording_fields(io::SensorEvent const & event)
{
    auto record = nlohmann::json::parse(event.source_record);
    return std::move(record.at("payload"));
}
} // namespace formal_gvins::dataset
