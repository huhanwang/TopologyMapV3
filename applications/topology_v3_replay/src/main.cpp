#include <sqlite3.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/perception/fusion/fused_bev_road_geometry.pb.h"
#include "onboard/proto/perception/types/vision_lane_type.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "topology_v3/topology_pipeline.h"

namespace {

using Json = nlohmann::json;
namespace fs = std::filesystem;

struct FrameRange {
    std::string type = "frame";
    std::int64_t start = -1;
    std::int64_t end = -1;
};

struct ReplayConfig {
    fs::path db_path;
    std::string main_topic = "AutoFusedBevRoadGeometry";
    std::string time_type = "raw_timestamp";
    FrameRange range;
    std::string sync_mode = "previous_or_equal";
    std::vector<std::string> topics;
    fs::path output_dir;
    int frame_dir_width = 10;
    std::set<std::string> write_files;
    std::set<std::string> debug_layers;
};

struct TopicIndexItem {
    std::int64_t rowid = 0;
    std::int64_t frame_id = 0;
    std::int64_t raw_timestamp_us = 0;
    std::int64_t local_timestamp_us = 0;
};

struct TopicFrameData {
    TopicIndexItem index;
    std::vector<std::uint8_t> raw_data;
};

Json readJson(const fs::path& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot read config: " + path.string());
    Json value;
    stream >> value;
    return value;
}

fs::path resolvePath(const fs::path& base, const std::string& value) {
    fs::path path(value);
    return path.is_absolute() ? path : base / path;
}

std::set<std::string> readStringSet(const Json& object, const char* key) {
    std::set<std::string> result;
    for (const auto& item : object.value(key, Json::array())) {
        result.insert(item.get<std::string>());
    }
    return result;
}

ReplayConfig readConfig(const fs::path& path) {
    const Json j = readJson(path);
    const fs::path base = path.parent_path();
    ReplayConfig cfg;
    const auto replay = j.value("replay", Json::object());
    const auto source = replay.value("source", Json::object());
    if (source.value("type", std::string{"db"}) != "db") {
        throw std::runtime_error("TopologyMapV3 replay currently expects replay.source.type=db");
    }
    cfg.db_path = resolvePath(base, source.at("path").get<std::string>());

    const auto main_axis = replay.value("main_axis", Json::object());
    cfg.main_topic = main_axis.value("topic", cfg.main_topic);
    cfg.time_type = main_axis.value("time_type", cfg.time_type);
    const auto range = main_axis.value("range", Json::object());
    cfg.range.type = range.value("type", cfg.range.type);
    cfg.range.start = range.value("start", std::int64_t{-1});
    cfg.range.end = range.value("end", std::int64_t{-1});
    cfg.sync_mode = replay.value("sync", Json::object()).value("mode", cfg.sync_mode);
    cfg.topics = replay.value("topics", std::vector<std::string>{cfg.main_topic});
    if (std::find(cfg.topics.begin(), cfg.topics.end(), cfg.main_topic) == cfg.topics.end()) {
        cfg.topics.insert(cfg.topics.begin(), cfg.main_topic);
    }

    const auto output = j.value("output", Json::object());
    cfg.output_dir = resolvePath(base, output.at("dir").get<std::string>());
    cfg.frame_dir_width = output.value("frame_dir_width", cfg.frame_dir_width);
    cfg.write_files = readStringSet(output, "write_files");
    if (cfg.write_files.empty()) cfg.write_files = {"frame", "sync", "topology_v3_debug"};

    const auto debug = j.value("debug", Json::object());
    cfg.debug_layers = readStringSet(debug, "layers");
    return cfg;
}

class SqliteDb {
public:
    explicit SqliteDb(const fs::path& path) {
        if (sqlite3_open_v2(path.string().c_str(), &db_, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
            std::string err = db_ ? sqlite3_errmsg(db_) : "unknown sqlite error";
            if (db_) sqlite3_close(db_);
            db_ = nullptr;
            throw std::runtime_error("failed to open db: " + path.string() + " err=" + err);
        }
    }

    ~SqliteDb() {
        if (db_) sqlite3_close(db_);
    }

    sqlite3* get() const { return db_; }

private:
    sqlite3* db_ = nullptr;
};

std::vector<TopicIndexItem> loadTopicIndex(sqlite3* db, const std::string& topic) {
    if (topic.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos) {
        throw std::runtime_error("invalid topic table name: " + topic);
    }
    const std::string sql =
        "SELECT rowid, id, raw_timestamp, local_timestamp FROM " + topic +
        " ORDER BY raw_timestamp ASC";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error(std::string("prepare topic index failed: ") + sqlite3_errmsg(db));
    }

    std::vector<TopicIndexItem> result;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        result.push_back({
            sqlite3_column_int64(stmt, 0),
            sqlite3_column_int64(stmt, 1),
            sqlite3_column_int64(stmt, 2),
            sqlite3_column_int64(stmt, 3)});
    }
    sqlite3_finalize(stmt);
    return result;
}

TopicFrameData loadTopicFrameData(
    sqlite3* db,
    const std::string& topic,
    const TopicIndexItem& item) {
    if (topic.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos) {
        throw std::runtime_error("invalid topic table name: " + topic);
    }
    const std::string sql = "SELECT raw_data FROM " + topic + " WHERE rowid = ?";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error(std::string("prepare topic raw load failed: ") + sqlite3_errmsg(db));
    }
    sqlite3_bind_int64(stmt, 1, item.rowid);

    TopicFrameData data;
    data.index = item;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        const void* blob = sqlite3_column_blob(stmt, 0);
        const int bytes = sqlite3_column_bytes(stmt, 0);
        if (blob != nullptr && bytes > 0) {
            const auto* begin = static_cast<const std::uint8_t*>(blob);
            data.raw_data.assign(begin, begin + bytes);
        }
    }
    sqlite3_finalize(stmt);
    return data;
}

template <typename ProtoT>
bool parseProto(const TopicFrameData& data, ProtoT* proto, std::string* error) {
    if (proto == nullptr) return false;
    if (data.raw_data.empty()) {
        if (error) *error = "empty raw_data";
        return false;
    }
    if (!proto->ParseFromArray(data.raw_data.data(), static_cast<int>(data.raw_data.size()))) {
        if (error) *error = "ParseFromArray failed";
        return false;
    }
    return true;
}

double polylineLength(const std::vector<topology_map::topology_v3::BoundaryPoint2d>& points) {
    double length = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        length += std::hypot(points[i].x_m - points[i - 1].x_m,
                             points[i].y_m - points[i - 1].y_m);
    }
    return length;
}

template <typename LaneInfo>
std::vector<topology_map::topology_v3::BoundaryPoint2d> lanePoints(const LaneInfo& lane) {
    std::vector<topology_map::topology_v3::BoundaryPoint2d> points;
    for (const auto& section : lane.lanes()) {
        for (const auto& point : section.bev_points()) {
            points.push_back({point.x(), point.y()});
        }
    }
    return points;
}

template <typename LaneInfo>
std::string laneSourceType(const LaneInfo& lane) {
    using snoah::CURB;
    using snoah::CURB_BARRIER;
    using snoah::CURB_HORIZONTAL;
    using snoah::CURB_MOVABLE;
    using snoah::STOP_LINE;
    if (lane.lane_position_index() == LaneInfo::ROAD_EDGE_LEFT ||
        lane.lane_position_index() == LaneInfo::ROAD_EDGE_RIGHT) {
        return "road_edge";
    }
    bool has_curb = false;
    bool has_stopline = false;
    for (const auto& section : lane.lanes()) {
        const auto type = section.bev_lane_type();
        has_curb = has_curb || type == CURB || type == CURB_BARRIER ||
            type == CURB_HORIZONTAL || type == CURB_MOVABLE;
        has_stopline = has_stopline || type == STOP_LINE;
    }
    if (has_curb) return "curb";
    if (has_stopline) return "stopline";
    return "lane_line";
}

template <typename LaneList>
void appendVisualLines(
    const LaneList& lanes,
    const std::string& source,
    std::int64_t frame_id,
    std::vector<topology_map::topology_v3::VisualBoundaryLineInput>* out) {
    using LaneInfo = snoah::FusedBevRoadGeometryProto::ParameterizedLaneInfo;
    for (int i = 0; i < lanes.size(); ++i) {
        const auto& lane = lanes.Get(i);
        auto points = lanePoints(lane);
        if (points.size() < 2) continue;
        topology_map::topology_v3::VisualBoundaryLineInput line;
        line.source = source;
        line.lane_id = lane.lane_id();
        line.lane_position = LaneInfo::PositionIndex_Name(lane.lane_position_index());
        line.source_type = laneSourceType(lane);
        line.coeffs = {lane.c0(), lane.c1(), lane.c2(), lane.c3()};
        line.points = std::move(points);
        line.confidence = 1.0;
        line.id = "vision:auto_fused:" + std::to_string(frame_id) + ":" +
                  source + ":" + std::to_string(line.lane_id) + ":" + std::to_string(i);
        out->push_back(std::move(line));
    }
}

std::vector<topology_map::topology_v3::VisualBoundaryLineInput> convertVisualLines(
    std::int64_t frame_id,
    const snoah::FusedBevRoadGeometryProto& proto) {
    std::vector<topology_map::topology_v3::VisualBoundaryLineInput> lines;
    appendVisualLines(proto.smooth_bev_lanes(), "smooth_bev_lanes", frame_id, &lines);
    appendVisualLines(proto.vehicle_bev_lanes(), "vehicle_bev_lanes", frame_id, &lines);
    return lines;
}

topology_map::topology_v3::SmoothPoseInput convertSmoothPose(
    const snoah::FusedBevRoadGeometryProto& proto) {
    topology_map::topology_v3::SmoothPoseInput out;
    if (proto.has_vehicle_pose() && proto.vehicle_pose().has_x() &&
        proto.vehicle_pose().has_y() && proto.vehicle_pose().has_yaw()) {
        const auto& pose = proto.vehicle_pose();
        out.valid = true;
        out.x_m = pose.x();
        out.y_m = pose.y();
        out.yaw_rad = pose.yaw();
    }
    return out;
}

topology_map::topology_v3::GnssInput convertGnss(const snoah::GnssRawReadingProto& proto) {
    topology_map::topology_v3::GnssInput out;
    out.valid = proto.has_latitude() && proto.has_longitude();
    out.latitude = proto.has_latitude() ? proto.latitude() : 0.0;
    out.longitude = proto.has_longitude() ? proto.longitude() : 0.0;
    out.altitude = proto.has_altitude() ? proto.altitude() : 0.0;
    out.yaw_rad = proto.has_yaw() ? proto.yaw() : 0.0;
    return out;
}

topology_map::topology_v3::NavigationRouteInput convertRoute(const snoah::SDRouteProto& proto) {
    topology_map::topology_v3::NavigationRouteInput out;
    out.available = proto.navigation_segments_size() > 0;
    out.route_id = proto.has_route_id() ? proto.route_id() : 0;
    for (int i = 0; i < proto.navigation_segments_size(); ++i) {
        const auto& segment = proto.navigation_segments(i);
        topology_map::topology_v3::NavigationRouteSegmentInput converted;
        if (segment.has_instruction()) converted.instruction = segment.instruction();
        if (segment.has_crossing_name()) converted.crossing_name = segment.crossing_name();
        if (segment.has_exit_direction_info()) converted.exit_direction_info = segment.exit_direction_info();
        if (segment.has_exit_name()) converted.exit_name = segment.exit_name();
        for (int p = 0; p < segment.points_size(); ++p) {
            const auto& point = segment.points(p);
            converted.points.push_back({point.latitude(), point.longitude(), point.altitude()});
        }
        out.segments.push_back(std::move(converted));
    }
    return out;
}

std::int64_t axisTime(const TopicIndexItem& item, const std::string& time_type) {
    if (time_type == "local_timestamp") return item.local_timestamp_us;
    return item.raw_timestamp_us;
}

std::vector<TopicIndexItem> selectMainFrames(
    const std::vector<TopicIndexItem>& index,
    const ReplayConfig& cfg) {
    std::vector<TopicIndexItem> result;
    for (const auto& item : index) {
        if (cfg.range.type == "time_us") {
            const auto t = axisTime(item, cfg.time_type);
            if ((cfg.range.start < 0 || t >= cfg.range.start) &&
                (cfg.range.end < 0 || t <= cfg.range.end)) {
                result.push_back(item);
            }
        } else {
            if ((cfg.range.start < 0 || item.frame_id >= cfg.range.start) &&
                (cfg.range.end < 0 || item.frame_id <= cfg.range.end)) {
                result.push_back(item);
            }
        }
    }
    return result;
}

const TopicIndexItem* findPreviousOrEqual(
    const std::vector<TopicIndexItem>& index,
    std::int64_t target_time_us,
    const std::string& time_type) {
    const TopicIndexItem* best = nullptr;
    for (const auto& item : index) {
        const auto t = axisTime(item, time_type);
        if (t > target_time_us) break;
        best = &item;
    }
    return best;
}

std::string paddedFrameDirName(std::int64_t frame_id, int width) {
    std::ostringstream stream;
    stream << std::setw(width) << std::setfill('0') << frame_id;
    return stream.str();
}

void writeTextFile(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream stream(path);
    if (!stream) throw std::runtime_error("Cannot write file: " + path.string());
    stream << text;
}

bool shouldWrite(const ReplayConfig& cfg, const std::string& name) {
    return cfg.write_files.count(name) != 0;
}

Json syncEntryToJson(const topology_map::topology_v3::TopicSyncEntry& entry) {
    return {
        {"topic", entry.topic},
        {"rowid", entry.rowid},
        {"frame_id", entry.frame_id},
        {"raw_timestamp_us", entry.raw_timestamp_us},
        {"local_timestamp_us", entry.local_timestamp_us},
        {"reason", entry.reason}};
}

Json frameToJson(const topology_map::topology_v3::ReplayFrameInput& frame) {
    Json sync = Json::array();
    for (const auto& entry : frame.sync) sync.push_back(syncEntryToJson(entry));
    return {
        {"index", frame.index},
        {"main_topic", frame.main_topic},
        {"main_frame_id", frame.frame_id},
        {"main_time_us", frame.timestamp_us},
        {"sync", std::move(sync)}};
}

Json visualLaneLinesToJson(const topology_map::topology_v3::ReplayFrameInput& input) {
    Json lines = Json::array();
    const bool has_smooth_pose = input.smooth_pose && input.smooth_pose->valid;
    const double c = has_smooth_pose ? std::cos(input.smooth_pose->yaw_rad) : 1.0;
    const double s = has_smooth_pose ? std::sin(input.smooth_pose->yaw_rad) : 0.0;
    for (const auto& line : input.visual_boundary_lines) {
        if (line.source != "vehicle_bev_lanes") continue;
        Json points = Json::array();
        for (const auto& point : line.points) {
            Json out = {
                {"x_vcs_m", point.x_m},
                {"y_vcs_m", point.y_m}};
            if (has_smooth_pose) {
                out["x_smooth_m"] = input.smooth_pose->x_m + point.x_m * c - point.y_m * s;
                out["y_smooth_m"] = input.smooth_pose->y_m + point.x_m * s + point.y_m * c;
            }
            points.push_back(std::move(out));
        }
        lines.push_back({
            {"id", line.id},
            {"lane_id", line.lane_id},
            {"source", line.source},
            {"lane_position", line.lane_position},
            {"source_type", line.source_type},
            {"confidence", line.confidence},
            {"points", std::move(points)}});
    }
    return lines;
}

Json outputToJson(const topology_map::topology_v3::ReplayFrameInput& input,
                  const topology_map::topology_v3::TopologyFrameOutput& output,
                  const ReplayConfig& cfg) {
    Json layers = Json::array();
    for (const auto& layer : output.debug_layers) {
        if (!cfg.debug_layers.empty() && cfg.debug_layers.count(layer.name) == 0) continue;
        layers.push_back({
            {"name", layer.name},
            {"stage", layer.stage},
            {"generated", layer.generated},
            {"messages", layer.messages}});
    }
    const auto visualReferenceToJson = [](const auto& result) {
        Json points = Json::array();
        for (const auto& point : result.points) {
            points.push_back({{"s", point.s_m}, {"x", point.x_m}, {"y", point.y_m}});
        }
        Json coeffs = Json::array();
        for (double coeff : result.center_coeffs) coeffs.push_back(coeff);
        return Json{
            {"schema_version", "topology-map-v3.visual-reference.v1"},
            {"ok", result.ok},
            {"error", result.error},
            {"selected_source", result.selected_source},
            {"method", result.method},
            {"left_line_id", result.left_line_id},
            {"right_line_id", result.right_line_id},
            {"confidence", result.confidence},
            {"s_range_m", {result.s_begin_m, result.s_end_m}},
            {"center_coeffs", std::move(coeffs)},
            {"points", std::move(points)},
            {"debug", {
                {"input_line_count", result.input_line_count},
                {"selected_source_line_count", result.selected_source_line_count},
                {"left_lane_position", result.left_lane_position},
                {"right_lane_position", result.right_lane_position},
                {"left_source_type", result.left_source_type},
                {"right_source_type", result.right_source_type},
                {"left_line_x_span_m", result.left_line_x_span_m},
                {"right_line_x_span_m", result.right_line_x_span_m},
                {"left_line_length_m", result.left_line_length_m},
                {"right_line_length_m", result.right_line_length_m},
                {"point_count", result.points.size()}}}};
    };
    const auto navigationReferenceToJson = [](const auto& result) {
        Json points = Json::array();
        for (const auto& point : result.vcs_points) {
            points.push_back({point.x_m, point.y_m, 0.0});
        }
        return Json{
            {"schema_version", "topology-map-v3.navigation-reference.v1"},
            {"ok", result.ok},
            {"error", result.error},
            {"backward_length_m", result.backward_length_m},
            {"forward_length_m", result.forward_length_m},
            {"lateral_error_m", result.lateral_error_m},
            {"heading_error_rad", result.heading_error_rad},
            {"stop_reason_forward", result.stop_reason_forward},
            {"stop_reason_backward", result.stop_reason_backward},
            {"points", std::move(points)}};
    };
    const auto fusedReferenceToJson = [](const auto& result) {
        Json points = Json::array();
        for (const auto& point : result.points) {
            points.push_back({
                {"s", point.s_m},
                {"x", point.x_m},
                {"y", point.y_m},
                {"heading_rad", point.heading_rad},
                {"curvature_m_inv", point.curvature_m_inv},
                {"source", point.source}});
        }
        return Json{
            {"schema_version", "topology-map-v3.fused-reference.v1"},
            {"ok", result.ok},
            {"error", result.error},
            {"method", result.method},
            {"confidence", result.confidence},
            {"used_navigation", result.used_navigation},
            {"lateral_offset_m", result.lateral_offset_m},
            {"heading_error_rad", result.heading_error_rad},
            {"overlap_length_m", result.overlap_length_m},
            {"visual_end_x_m", result.visual_end_x_m},
            {"fused_start_x_m", result.fused_start_x_m},
            {"fused_end_x_m", result.fused_end_x_m},
            {"points", std::move(points)}};
    };
    const auto rawBoundaryEvidenceToJson = [](const auto& result) {
        Json boundaries = Json::array();
        std::size_t sample_count = 0;
        for (const auto& boundary : result.boundaries) {
            Json samples = Json::array();
            for (const auto& sample : boundary.samples) {
                Json point = {
                    {"id", sample.id},
                    {"s_m", sample.s_m},
                    {"l_m", sample.l_m},
                    {"x_vcs_m", sample.x_vcs_m},
                    {"y_vcs_m", sample.y_vcs_m},
                    {"confidence", sample.confidence},
                    {"source_line_id", sample.source_line_id},
                    {"track_line_id", sample.track_line_id},
                    {"semantic_type", sample.semantic_type}};
                if (std::isfinite(sample.x_smooth_m) && std::isfinite(sample.y_smooth_m)) {
                    point["x_smooth_m"] = sample.x_smooth_m;
                    point["y_smooth_m"] = sample.y_smooth_m;
                }
                samples.push_back(std::move(point));
            }
            sample_count += boundary.samples.size();
            boundaries.push_back({
                {"observation_id", boundary.observation_id},
                {"debug_label", boundary.debug_label},
                {"semantic_type", boundary.semantic_type},
                {"quality", boundary.quality},
                {"source_identity_ids", boundary.source_identity_ids},
                {"sample_count", boundary.samples.size()},
                {"samples", std::move(samples)}});
        }
        return Json{
            {"schema_version", "topology-map-v3.raw-boundary-evidence.v1"},
            {"ok", result.ok},
            {"error", result.error},
            {"boundary_count", result.boundaries.size()},
            {"sample_count", sample_count},
            {"boundaries", std::move(boundaries)}};
    };
    const auto rawVisualPreprocessToJson = [](const auto& result) {
        Json boundaries = Json::array();
        for (const auto& boundary : result.boundaries) {
            Json points = Json::array();
            for (const auto& point : boundary.points) {
                points.push_back({{"x_vcs_m", point.x_m}, {"y_vcs_m", point.y_m}});
            }
            boundaries.push_back({
                {"raw_ft_id", boundary.raw_ft_id},
                {"debug_label", boundary.debug_label},
                {"source", boundary.source},
                {"track_line_id", boundary.track_line_id},
                {"source_line_ids", boundary.source_line_ids},
                {"lane_id", boundary.lane_id},
                {"lane_position", boundary.lane_position},
                {"semantic_type", boundary.semantic_type},
                {"confidence", boundary.confidence},
                {"point_count", boundary.points.size()},
                {"rejected", boundary.rejected},
                {"rejection_reason", boundary.rejection_reason},
                {"points", std::move(points)}});
        }
        return Json{
            {"schema_version", "topology-map-v3.raw-visual-preprocess.v1"},
            {"ok", result.ok},
            {"error", result.error},
            {"input_line_count", result.input_line_count},
            {"boundary_count", result.boundaries.size()},
            {"merged_track_count", result.merged_track_count},
            {"hard_rejected_count", result.hard_rejected_count},
            {"boundaries", std::move(boundaries)}};
    };
    const auto frenetSliceIntersectionsToJson = [](const auto& result) {
        Json slices = Json::array();
        for (const auto& slice : result.slices) {
            Json nodes = Json::array();
            for (const auto& node : slice.nodes) {
                Json node_json = {
                    {"node_id", node.node_id},
                    {"raw_ft_id", node.raw_ft_id},
                    {"debug_label", node.debug_label},
                    {"slice_index", node.slice_index},
                    {"s_m", node.s_m},
                    {"l_m", node.l_m},
                    {"x_vcs_m", node.x_vcs_m},
                    {"y_vcs_m", node.y_vcs_m},
                    {"confidence", node.confidence},
                    {"semantic_type", node.semantic_type},
                    {"source_line_ids", node.source_line_ids}};
                if (std::isfinite(node.x_smooth_m) && std::isfinite(node.y_smooth_m)) {
                    node_json["x_smooth_m"] = node.x_smooth_m;
                    node_json["y_smooth_m"] = node.y_smooth_m;
                }
                nodes.push_back(std::move(node_json));
            }
            slices.push_back({
                {"slice_index", slice.slice_index},
                {"s_m", slice.s_m},
                {"origin_x_vcs_m", slice.origin_x_vcs_m},
                {"origin_y_vcs_m", slice.origin_y_vcs_m},
                {"normal_x", slice.normal_x},
                {"normal_y", slice.normal_y},
                {"node_count", slice.nodes.size()},
                {"nodes", std::move(nodes)}});
        }
        return Json{
            {"schema_version", "topology-map-v3.frenet-slice-intersections.v1"},
            {"ok", result.ok},
            {"error", result.error},
            {"slice_count", result.slices.size()},
            {"node_count", result.node_count},
            {"slices", std::move(slices)}};
    };
    const auto rawFtFilterToJson = [](const auto& result) {
        Json decisions = Json::array();
        for (const auto& decision : result.decisions) {
            decisions.push_back({
                {"raw_ft_id", decision.raw_ft_id},
                {"debug_label", decision.debug_label},
                {"state", decision.state},
                {"reason", decision.reason},
                {"direct_topology_candidate", decision.direct_topology_candidate},
                {"passive_boundary", decision.passive_boundary},
                {"sample_count", decision.sample_count},
                {"support_length_m", decision.support_length_m}});
        }
        return Json{
            {"schema_version", "topology-map-v3.raw-ft-filter.v1"},
            {"ok", result.ok},
            {"error", result.error},
            {"decision_count", result.decisions.size()},
            {"kept_count", result.kept_count},
            {"pending_count", result.pending_count},
            {"suppressed_count", result.suppressed_count},
            {"passive_boundary_count", result.passive_boundary_count},
            {"decisions", std::move(decisions)}};
    };
    const auto frenetSliceGraphToJson = [](const auto& result) {
        Json nodes = Json::array();
        for (const auto& node : result.nodes) {
            nodes.push_back({
                {"node_id", node.node_id},
                {"raw_ft_id", node.raw_ft_id},
                {"final_ft_id", node.final_ft_id},
                {"debug_label", node.debug_label},
                {"slice_index", node.slice_index},
                {"s_m", node.s_m},
                {"l_m", node.l_m},
                {"state", node.state},
                {"provenance", node.provenance},
                {"reason", node.reason},
                {"semantic_type", node.semantic_type},
                {"reconstruction_support_node_id", node.reconstruction_support_node_id},
                {"reconstruction_support_is_left", node.reconstruction_support_is_left},
                {"reconstruction_width_m", node.reconstruction_width_m}});
        }
        Json lon_links = Json::array();
        for (const auto& link : result.lon_links) {
            lon_links.push_back({
                {"link_id", link.link_id},
                {"from_node_id", link.from_node_id},
                {"to_node_id", link.to_node_id},
                {"kind", link.kind},
                {"active", link.active},
                {"score", link.score},
                {"reason", link.reason}});
        }
        Json lat_links = Json::array();
        for (const auto& link : result.lat_links) {
            lat_links.push_back({
                {"link_id", link.link_id},
                {"right_node_id", link.right_node_id},
                {"left_node_id", link.left_node_id},
                {"slice_index", link.slice_index},
                {"s_m", link.s_m},
                {"width_m", link.width_m},
                {"active", link.active},
                {"reason", link.reason}});
        }
        Json slice_ribbons = Json::array();
        for (const auto& ribbon : result.slice_ribbons) {
            slice_ribbons.push_back({
                {"ribbon_id", ribbon.ribbon_id},
                {"slice_index", ribbon.slice_index},
                {"right_node_id", ribbon.right_node_id},
                {"left_node_id", ribbon.left_node_id},
                {"s_m", ribbon.s_m},
                {"width_m", ribbon.width_m},
                {"center_l_m", ribbon.center_l_m}});
        }
        return Json{
            {"schema_version", "topology-map-v3.frenet-slice-graph.v1"},
            {"ok", result.ok},
            {"error", result.error},
            {"node_count", result.nodes.size()},
            {"observed_node_count", result.observed_node_count},
            {"inferred_node_count", result.inferred_node_count},
            {"lon_link_count", result.lon_links.size()},
            {"observed_lon_link_count", result.observed_lon_link_count},
            {"inferred_lon_link_count", result.inferred_lon_link_count},
            {"near_topology_link_count", result.near_topology_link_count},
            {"lat_link_count", result.lat_links.size()},
            {"slice_ribbon_count", result.slice_ribbons.size()},
            {"nodes", std::move(nodes)},
            {"lon_links", std::move(lon_links)},
            {"lat_links", std::move(lat_links)},
            {"slice_ribbons", std::move(slice_ribbons)}};
    };
    const auto boundaryColor = [](const std::string& semantic_type) {
        if (semantic_type == "curb") return "#eb5757";
        if (semantic_type == "road_edge") return "#f2994a";
        if (semantic_type == "stopline") return "#56ccf2";
        return "#dfe6e9";
    };
    const auto rawFtStateColor = [](const std::string& state) {
        if (state == "kept") return "#27ae60";
        if (state == "pending") return "#f2c94c";
        if (state == "suppressed") return "#ff3b30";
        if (state == "passive_boundary") return "#f2994a";
        return "#828b94";
    };
    const auto visualReferenceLayer = [](const auto& result) {
        Json layer = {{"id", "visual_reference"}, {"name", "Visual reference"},
                      {"visible", true}, {"items", Json::array()}};
        if (!result.ok || result.points.size() < 2) return layer;
        Json points = Json::array();
        for (const auto& point : result.points) points.push_back({point.x_m, point.y_m, 0.0});
        layer["items"].push_back({
            {"type", "polyline"},
            {"id", "visual_reference_center"},
            {"name", "Visual reference center"},
            {"points", std::move(points)},
            {"style", {{"color", "#00b894"}, {"width", 0.18}, {"dash", false}}},
            {"properties", {
                {"source", "visual_reference_module"},
                {"selected_source", result.selected_source},
                {"method", result.method},
                {"left_line_id", result.left_line_id},
                {"right_line_id", result.right_line_id}}}});
        return layer;
    };
    const auto navigationReferenceLayer = [](const auto& result) {
        Json layer = {{"id", "navigation_reference"}, {"name", "Navigation reference"},
                      {"visible", true}, {"items", Json::array()}};
        if (!result.ok || result.vcs_points.size() < 2) return layer;
        Json points = Json::array();
        for (const auto& point : result.vcs_points) points.push_back({point.x_m, point.y_m, 0.0});
        layer["items"].push_back({
            {"type", "polyline"},
            {"id", "navigation_reference_center"},
            {"name", "Navigation reference"},
            {"points", std::move(points)},
            {"style", {{"color", "#2d9cdb"}, {"width", 0.16}, {"dash", false}}},
            {"properties", {
                {"source", "navigation_reference_module"},
                {"forward_length_m", result.forward_length_m},
                {"backward_length_m", result.backward_length_m},
                {"lateral_error_m", result.lateral_error_m},
                {"heading_error_rad", result.heading_error_rad},
                {"stop_reason_forward", result.stop_reason_forward},
                {"stop_reason_backward", result.stop_reason_backward}}}});
        return layer;
    };
    const auto fusedReferenceLayer = [](const auto& result) {
        Json layer = {{"id", "fused_reference"}, {"name", "Fused reference"},
                      {"visible", true}, {"modes", {"vcs"}}, {"items", Json::array()}};
        if (!result.ok || result.points.size() < 2) return layer;
        Json points = Json::array();
        for (const auto& point : result.points) points.push_back({point.x_m, point.y_m, 0.0});
        layer["items"].push_back({
            {"type", "polyline"},
            {"id", "fused_reference_center"},
            {"name", "Fused reference"},
            {"points", std::move(points)},
            {"style", {{"color", "#f2c94c"}, {"width", 0.2}, {"dash", false}}},
            {"properties", {
                {"source", "fused_reference_builder"},
                {"method", result.method},
                {"confidence", result.confidence},
                {"used_navigation", result.used_navigation},
                {"lateral_offset_m", result.lateral_offset_m},
                {"heading_error_rad", result.heading_error_rad},
                {"overlap_length_m", result.overlap_length_m}}}});
        return layer;
    };
    const auto rawBoundaryEvidenceLayer = [&boundaryColor](const auto& result) {
        Json layer = {
            {"id", "raw_boundary_evidence"},
            {"name", "Raw boundary evidence"},
            {"visible", true},
            {"modes", {"frenet"}},
            {"items", Json::array()}};
        if (!result.ok) return layer;
        for (const auto& boundary : result.boundaries) {
            if (boundary.samples.size() < 2) continue;
            Json points = Json::array();
            for (const auto& sample : boundary.samples) {
                Json point = {
                    {"s_m", sample.s_m},
                    {"l_m", sample.l_m},
                    {"x_vcs_m", sample.x_vcs_m},
                    {"y_vcs_m", sample.y_vcs_m}};
                if (std::isfinite(sample.x_smooth_m) && std::isfinite(sample.y_smooth_m)) {
                    point["x_smooth_m"] = sample.x_smooth_m;
                    point["y_smooth_m"] = sample.y_smooth_m;
                }
                points.push_back(std::move(point));
            }
            layer["items"].push_back({
                {"type", "polyline"},
                {"id", "raw_boundary_evidence_" + std::to_string(boundary.observation_id)},
                {"name", boundary.debug_label.empty() ? std::to_string(boundary.observation_id)
                                                       : boundary.debug_label},
                {"points", std::move(points)},
                {"style", {
                    {"color", boundaryColor(boundary.semantic_type)},
                    {"width", 0.08},
                    {"dash", false}}},
                {"properties", {
                    {"source", "raw_boundary_evidence_builder"},
                    {"semantic_type", boundary.semantic_type},
                    {"quality", boundary.quality},
                    {"sample_count", boundary.samples.size()}}}});
        }
        return layer;
    };
    const auto rawVisualPreprocessLayer = [&boundaryColor](const auto& result) {
        Json layer = {
            {"id", "raw_visual_preprocess"},
            {"name", "Raw visual preprocess"},
            {"visible", false},
            {"modes", {"vcs"}},
            {"items", Json::array()}};
        if (!result.ok) return layer;
        for (const auto& boundary : result.boundaries) {
            if (boundary.points.size() < 2) continue;
            Json points = Json::array();
            for (const auto& point : boundary.points) {
                points.push_back({{"x_vcs_m", point.x_m}, {"y_vcs_m", point.y_m}});
            }
            const std::string color = boundary.rejected ? "#ff3b30" : boundaryColor(boundary.semantic_type);
            layer["items"].push_back({
                {"type", "polyline"},
                {"id", "raw_visual_preprocess_" + std::to_string(boundary.raw_ft_id) + "_" +
                         boundary.debug_label},
                {"name", boundary.debug_label},
                {"points", std::move(points)},
                {"style", {
                    {"color", color},
                    {"width", boundary.rejected ? 0.12 : 0.08},
                    {"dash", boundary.rejected}}},
                {"properties", {
                    {"source", "raw_visual_boundary_preprocessor"},
                    {"track_line_id", boundary.track_line_id},
                    {"semantic_type", boundary.semantic_type},
                    {"rejected", boundary.rejected},
                    {"rejection_reason", boundary.rejection_reason},
                    {"source_line_ids", boundary.source_line_ids}}}});
        }
        return layer;
    };
    const auto frenetSliceIntersectionsLayer = [&boundaryColor](const auto& result) {
        Json layer = {
            {"id", "frenet_slice_intersections"},
            {"name", "Frenet slice intersections"},
            {"visible", false},
            {"modes", {"frenet"}},
            {"items", Json::array()}};
        if (!result.ok) return layer;

        for (const auto& slice : result.slices) {
            if (slice.nodes.empty()) continue;
            layer["items"].push_back({
                {"type", "polyline"},
                {"id", "frenet_slice_normal_" + std::to_string(slice.slice_index)},
                {"name", "S" + std::to_string(slice.slice_index)},
                {"points", Json::array({
                    {{"s_m", slice.s_m}, {"l_m", -40.0}},
                    {{"s_m", slice.s_m}, {"l_m", 40.0}}})},
                {"style", {{"color", "#3a444d"}, {"width", 0.025}, {"dash", true}}},
                {"properties", {
                    {"source", "frenet_slice_intersection_builder"},
                    {"slice_index", slice.slice_index},
                    {"node_count", slice.nodes.size()}}}});
        }

        std::map<std::uint64_t, Json> point_sets;
        std::map<std::uint64_t, std::string> labels;
        std::map<std::uint64_t, std::string> semantic_types;
        for (const auto& slice : result.slices) {
            for (const auto& node : slice.nodes) {
                if (point_sets.find(node.raw_ft_id) == point_sets.end()) {
                    point_sets[node.raw_ft_id] = Json::array();
                }
                point_sets[node.raw_ft_id].push_back({{"s_m", node.s_m}, {"l_m", node.l_m}});
                labels[node.raw_ft_id] = node.debug_label;
                semantic_types[node.raw_ft_id] = node.semantic_type;
            }
        }
        for (auto& [raw_ft_id, points] : point_sets) {
            if (points.empty()) continue;
            const auto semantic_it = semantic_types.find(raw_ft_id);
            const std::string semantic =
                semantic_it == semantic_types.end() ? "" : semantic_it->second;
            Json intersection_points = points;
            layer["items"].push_back({
                {"type", "polyline"},
                {"id", "frenet_slice_nodes_" + std::to_string(raw_ft_id)},
                {"name", labels[raw_ft_id]},
                {"points", std::move(points)},
                {"style", {
                    {"color", boundaryColor(semantic)},
                    {"width", 0.10},
                    {"dash", false}}},
                {"properties", {
                    {"source", "frenet_slice_intersection_builder"},
                    {"raw_ft_id", raw_ft_id},
                    {"semantic_type", semantic}}}});
            layer["items"].push_back({
                {"type", "points"},
                {"id", "frenet_slice_points_" + std::to_string(raw_ft_id)},
                {"name", labels[raw_ft_id] + " intersections"},
                {"points", std::move(intersection_points)},
                {"style", {
                    {"color", boundaryColor(semantic)},
                    {"radius_px", 3.2}}},
                {"properties", {
                    {"source", "frenet_slice_intersection_builder"},
                    {"raw_ft_id", raw_ft_id},
                    {"semantic_type", semantic}}}});
        }
        return layer;
    };
    const auto rawFtFilterLayer = [&rawFtStateColor](
                                      const auto& result,
                                      const auto& intersections) {
        Json layer = {
            {"id", "raw_ft_filter"},
            {"name", "Raw FT filter"},
            {"visible", false},
            {"modes", {"frenet"}},
            {"items", Json::array()}};
        if (!result.ok || !intersections.ok) return layer;

        std::map<std::uint64_t, Json> point_sets;
        for (const auto& slice : intersections.slices) {
            for (const auto& node : slice.nodes) {
                if (point_sets.find(node.raw_ft_id) == point_sets.end()) {
                    point_sets[node.raw_ft_id] = Json::array();
                }
                point_sets[node.raw_ft_id].push_back({{"s_m", node.s_m}, {"l_m", node.l_m}});
            }
        }

        for (const auto& decision : result.decisions) {
            auto it = point_sets.find(decision.raw_ft_id);
            if (it == point_sets.end() || it->second.empty()) continue;
            Json points = it->second;
            const std::string color = rawFtStateColor(decision.state);
            layer["items"].push_back({
                {"type", "polyline"},
                {"id", "raw_ft_filter_" + std::to_string(decision.raw_ft_id)},
                {"name", decision.debug_label + " " + decision.state},
                {"points", std::move(points)},
                {"style", {
                    {"color", color},
                    {"width", decision.state == "suppressed" ? 0.13 : 0.10},
                    {"dash", decision.state != "kept"}}},
                {"properties", {
                    {"source", "raw_ft_filter_builder"},
                    {"raw_ft_id", decision.raw_ft_id},
                    {"state", decision.state},
                    {"reason", decision.reason},
                    {"sample_count", decision.sample_count},
                    {"support_length_m", decision.support_length_m}}}});
        }
        return layer;
    };
    const auto frenetSliceGraphLayer = [](
                                            const auto& result) {
        Json layer = {
            {"id", "frenet_slice_graph"},
            {"name", "Frenet slice graph"},
            {"visible", false},
            {"modes", {"frenet"}},
            {"items", Json::array()}};
        if (!result.ok) return layer;

        std::map<std::uint64_t, const topology_map::topology_v3::FrenetSliceGraphNode*> nodes;
        for (const auto& node : result.nodes) nodes[node.node_id] = &node;
        const auto nodeLabel = [&](std::uint64_t node_id) {
            const auto it = nodes.find(node_id);
            if (it == nodes.end()) return std::string{};
            return it->second->debug_label + "@" + std::to_string(it->second->slice_index);
        };
        const auto nodePoint = [&](std::uint64_t node_id) {
            const auto it = nodes.find(node_id);
            if (it == nodes.end()) return Json{};
            return Json{{"s_m", it->second->s_m}, {"l_m", it->second->l_m}};
        };
        const auto linkColor = [](const std::string& kind) {
            if (kind == "observed") return "#2d9cdb";
            if (kind == "inferred") return "#f2c94c";
            if (kind == "near_topology") return "#eb5757";
            return "#9aa4ad";
        };
        for (const auto& link : result.lon_links) {
            if (!link.active) continue;
            const auto from_it = nodes.find(link.from_node_id);
            const auto to_it = nodes.find(link.to_node_id);
            if (from_it == nodes.end() || to_it == nodes.end()) continue;
            layer["items"].push_back({
                {"type", "polyline"},
                {"id", "frenet_slice_graph_lon_" + std::to_string(link.link_id)},
                {"name", nodeLabel(link.from_node_id) + " -> " +
                         nodeLabel(link.to_node_id) + " " + link.kind},
                {"points", Json::array({nodePoint(link.from_node_id), nodePoint(link.to_node_id)})},
                {"style", {
                    {"color", linkColor(link.kind)},
                    {"width", link.kind == "near_topology" ? 0.16 : 0.11},
                    {"dash", link.kind != "observed"}}},
                {"properties", {
                    {"source", "frenet_slice_graph_builder"},
                    {"kind", link.kind},
                    {"reason", link.reason},
                    {"from_node_id", link.from_node_id},
                    {"to_node_id", link.to_node_id}}}});
        }
        Json inferred_points = Json::array();
        for (const auto& node : result.nodes) {
            if (node.state != "inferred") continue;
            Json point = {{"s_m", node.s_m}, {"l_m", node.l_m}};
            point["name"] = node.debug_label + " inferred";
            point["support_node_id"] = node.reconstruction_support_node_id;
            point["width_m"] = node.reconstruction_width_m;
            inferred_points.push_back(std::move(point));
        }
        if (!inferred_points.empty()) {
            layer["items"].push_back({
                {"type", "points"},
                {"id", "frenet_slice_graph_inferred_nodes"},
                {"name", "Inferred nodes"},
                {"points", std::move(inferred_points)},
                {"style", {{"color", "#f2c94c"}, {"radius_px", 4.0}}},
                {"properties", {{"source", "frenet_slice_graph_builder"}}}});
        }
        return layer;
    };
    return {
        {"frame_id", input.frame_id},
        {"timestamp_us", input.timestamp_us},
        {"input_summary", {
            {"visual_boundary_line_count", input.visual_boundary_lines.size()},
            {"has_smooth_pose", input.smooth_pose.has_value() && input.smooth_pose->valid},
            {"has_gnss", input.gnss.has_value() && input.gnss->valid},
            {"has_navigation_route", input.navigation_route.has_value() && input.navigation_route->available}}},
        {"visual_lane_lines", visualLaneLinesToJson(input)},
        {"visual_reference", visualReferenceToJson(output.visual_reference)},
        {"navigation_reference", navigationReferenceToJson(output.navigation_reference)},
        {"fused_reference", fusedReferenceToJson(output.fused_reference)},
        {"raw_visual_preprocess", rawVisualPreprocessToJson(output.raw_visual_preprocess)},
        {"raw_boundary_evidence", rawBoundaryEvidenceToJson(output.raw_boundary_evidence)},
        {"frenet_slice_intersections",
         frenetSliceIntersectionsToJson(output.frenet_slice_intersections)},
        {"raw_ft_filter", rawFtFilterToJson(output.raw_ft_filter)},
        {"frenet_slice_graph", frenetSliceGraphToJson(output.frenet_slice_graph)},
        {"viz_layers", {visualReferenceLayer(output.visual_reference),
                        navigationReferenceLayer(output.navigation_reference),
                        fusedReferenceLayer(output.fused_reference),
                        rawVisualPreprocessLayer(output.raw_visual_preprocess),
                        rawBoundaryEvidenceLayer(output.raw_boundary_evidence),
                        frenetSliceIntersectionsLayer(output.frenet_slice_intersections),
                        rawFtFilterLayer(output.raw_ft_filter,
                                         output.frenet_slice_intersections),
                        frenetSliceGraphLayer(output.frenet_slice_graph)}},
        {"debug_layers", std::move(layers)},
        {"diagnostics", output.diagnostics}};
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: topology_v3_replay <config.json>\n";
        return 2;
    }

    try {
        const fs::path config_path = argv[1];
        const ReplayConfig cfg = readConfig(config_path);

        std::cout << "[topology_v3_replay] db=" << cfg.db_path << "\n";
        SqliteDb db(cfg.db_path);

        std::map<std::string, std::vector<TopicIndexItem>> indices;
        for (const auto& topic : cfg.topics) {
            indices[topic] = loadTopicIndex(db.get(), topic);
            std::cout << "[topology_v3_replay] topic=" << topic
                      << " rows=" << indices[topic].size() << "\n";
        }

        const auto main_it = indices.find(cfg.main_topic);
        if (main_it == indices.end() || main_it->second.empty()) {
            throw std::runtime_error("main topic has no rows: " + cfg.main_topic);
        }
        const auto main_frames = selectMainFrames(main_it->second, cfg);
        std::cout << "[topology_v3_replay] selected main frames="
                  << main_frames.size() << "\n";

        fs::create_directories(cfg.output_dir / "frames");
        topology_map::topology_v3::TopologyPipeline pipeline;
        Json dataset = {
            {"topology_version", "v3"},
            {"config", config_path.string()},
            {"frames", Json::array()}};

        for (std::size_t i = 0; i < main_frames.size(); ++i) {
            const auto& main = main_frames[i];
            topology_map::topology_v3::ReplayFrameInput input;
            input.index = i;
            input.frame_id = main.frame_id;
            input.timestamp_us = axisTime(main, cfg.time_type);
            input.main_topic = cfg.main_topic;

            for (const auto& topic : cfg.topics) {
                const auto index_it = indices.find(topic);
                if (index_it == indices.end()) continue;
                const TopicIndexItem* selected = nullptr;
                std::string reason;
                if (topic == cfg.main_topic) {
                    selected = &main;
                    reason = "main_axis";
                } else if (cfg.sync_mode == "previous_or_equal") {
                    selected = findPreviousOrEqual(index_it->second, input.timestamp_us, cfg.time_type);
                    reason = selected ? "previous_or_equal" : "no_previous_or_equal_frame";
                } else {
                    throw std::runtime_error("unsupported sync mode: " + cfg.sync_mode);
                }
                topology_map::topology_v3::TopicSyncEntry entry;
                entry.topic = topic;
                entry.reason = reason;
                if (selected != nullptr) {
                    entry.rowid = selected->rowid;
                    entry.frame_id = selected->frame_id;
                    entry.raw_timestamp_us = selected->raw_timestamp_us;
                    entry.local_timestamp_us = selected->local_timestamp_us;
                }
                input.sync.push_back(std::move(entry));
                if (selected == nullptr) continue;

                const auto topic_data = loadTopicFrameData(db.get(), topic, *selected);
                std::string parse_error;
                if (topic == "AutoFusedBevRoadGeometry") {
                    snoah::FusedBevRoadGeometryProto proto;
                    if (parseProto(topic_data, &proto, &parse_error)) {
                        input.visual_boundary_lines = convertVisualLines(input.frame_id, proto);
                        input.smooth_pose = convertSmoothPose(proto);
                    } else {
                        std::cerr << "[topology_v3_replay] frame=" << input.frame_id
                                  << " topic=" << topic << " parse_error=" << parse_error << "\n";
                    }
                } else if (topic == "AutoSensorGnss") {
                    snoah::GnssRawReadingProto proto;
                    if (parseProto(topic_data, &proto, &parse_error)) {
                        input.gnss = convertGnss(proto);
                    } else {
                        std::cerr << "[topology_v3_replay] frame=" << input.frame_id
                                  << " topic=" << topic << " parse_error=" << parse_error << "\n";
                    }
                } else if (topic == "AutoSDRoute") {
                    snoah::SDRouteProto proto;
                    if (parseProto(topic_data, &proto, &parse_error)) {
                        input.navigation_route = convertRoute(proto);
                    } else {
                        std::cerr << "[topology_v3_replay] frame=" << input.frame_id
                                  << " topic=" << topic << " parse_error=" << parse_error << "\n";
                    }
                }
            }

            const auto output = pipeline.update(input);
            const auto rel_dir = fs::path("frames") / paddedFrameDirName(input.frame_id, cfg.frame_dir_width);
            const auto frame_dir = cfg.output_dir / rel_dir;
            fs::create_directories(frame_dir);

            Json files = Json::object();
            if (shouldWrite(cfg, "frame")) {
                writeTextFile(frame_dir / "frame.json", frameToJson(input).dump(2) + "\n");
                files["frame"] = (rel_dir / "frame.json").string();
            }
            if (shouldWrite(cfg, "sync")) {
                Json sync = Json::array();
                for (const auto& entry : input.sync) sync.push_back(syncEntryToJson(entry));
                writeTextFile(frame_dir / "sync.json", sync.dump(2) + "\n");
                files["sync"] = (rel_dir / "sync.json").string();
            }
            if (shouldWrite(cfg, "topology_v3_debug")) {
                writeTextFile(frame_dir / "topology_v3_debug.json",
                              outputToJson(input, output, cfg).dump(2) + "\n");
                files["topology_v3_debug"] = (rel_dir / "topology_v3_debug.json").string();
            }

            dataset["frames"].push_back({
                {"dir", rel_dir.string()},
                {"main_frame_id", input.frame_id},
                {"main_time_us", input.timestamp_us},
                {"files", std::move(files)}});
        }

        writeTextFile(cfg.output_dir / "dataset.js",
                      "window.REPLAY_DATASET = " + dataset.dump(2) + ";\n");
        std::cout << "[topology_v3_replay] wrote " << main_frames.size()
                  << " frames to " << cfg.output_dir << "\n";
    } catch (const std::exception& error) {
        std::cerr << "topology_v3_replay failed: " << error.what() << "\n";
        return 1;
    }

    return 0;
}
