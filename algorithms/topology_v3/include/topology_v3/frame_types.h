#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace topology_map::topology_v3 {

struct Vec2d {
    double x_m = 0.0;
    double y_m = 0.0;
};

struct TimedPose {
    std::int64_t timestamp_us = 0;
    Vec2d smooth_position;
    double heading_rad = 0.0;
    double quality = 0.0;
};

struct TopicSyncEntry {
    std::string topic;
    std::int64_t rowid = 0;
    std::int64_t frame_id = 0;
    std::int64_t raw_timestamp_us = 0;
    std::int64_t local_timestamp_us = 0;
    std::string reason;
};

struct BoundaryPoint2d {
    double x_m = 0.0;
    double y_m = 0.0;
};

struct VisualBoundaryLineInput {
    std::string id;
    std::string source;
    int lane_id = 0;
    std::string lane_position;
    std::string source_type = "lane_line";
    std::vector<double> coeffs;
    std::vector<BoundaryPoint2d> points;
    double confidence = 1.0;
};

struct GnssInput {
    bool valid = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double yaw_rad = 0.0;
};

struct SmoothPoseInput {
    bool valid = false;
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
};

struct NavigationRoutePointInput {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
};

struct NavigationRouteSegmentInput {
    std::string instruction;
    std::string crossing_name;
    std::string exit_direction_info;
    std::string exit_name;
    std::vector<NavigationRoutePointInput> points;
};

struct NavigationRouteInput {
    bool available = false;
    std::uint64_t route_id = 0;
    std::vector<NavigationRouteSegmentInput> segments;
};

struct ReferencePoint {
    double s_m = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
};

struct VisualReferenceOutput {
    bool ok = false;
    std::string error;
    std::string selected_source;
    std::string method;
    std::string left_line_id;
    std::string right_line_id;
    double confidence = 0.0;
    double s_begin_m = 0.0;
    double s_end_m = 0.0;
    std::vector<double> center_coeffs;
    std::vector<ReferencePoint> points;
    int input_line_count = 0;
    int selected_source_line_count = 0;
    std::string left_lane_position;
    std::string right_lane_position;
    std::string left_source_type;
    std::string right_source_type;
    double left_line_x_span_m = 0.0;
    double right_line_x_span_m = 0.0;
    double left_line_length_m = 0.0;
    double right_line_length_m = 0.0;
};

struct NavigationReferenceOutput {
    bool ok = false;
    std::string error;
    std::vector<BoundaryPoint2d> vcs_points;
    double backward_length_m = 0.0;
    double forward_length_m = 0.0;
    double lateral_error_m = 0.0;
    double heading_error_rad = 0.0;
    std::string stop_reason_forward;
    std::string stop_reason_backward;
};

struct FusedReferencePoint {
    double s_m = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
    double heading_rad = 0.0;
    double curvature_m_inv = 0.0;
    std::string source;
};

struct FusedReferenceOutput {
    bool ok = false;
    std::string error;
    std::string method;
    std::vector<FusedReferencePoint> points;
    double confidence = 0.0;
    double lateral_offset_m = 0.0;
    double heading_error_rad = 0.0;
    double overlap_length_m = 0.0;
    double visual_end_x_m = 0.0;
    double fused_start_x_m = 0.0;
    double fused_end_x_m = 0.0;
    bool used_navigation = false;
};

struct FrenetSamplePoint {
    std::uint64_t id = 0;
    double s_m = 0.0;
    double l_m = 0.0;
    double x_vcs_m = 0.0;
    double y_vcs_m = 0.0;
    double x_smooth_m = 0.0;
    double y_smooth_m = 0.0;
    double confidence = 0.0;
    std::string source_line_id;
    std::string track_line_id;
    std::string semantic_type;
};

struct RawBoundaryEvidence {
    std::uint64_t observation_id = 0;
    std::vector<FrenetSamplePoint> samples;
    std::vector<std::string> source_identity_ids;
    std::string semantic_type;
    double quality = 0.0;
};

struct RawBoundaryEvidenceOutput {
    bool ok = false;
    std::string error;
    std::vector<RawBoundaryEvidence> boundaries;
};

struct RawRibbonRelation {
    std::uint64_t relation_id = 0;
    std::uint64_t right_observation_id = 0;
    std::uint64_t left_observation_id = 0;
    double s_begin_m = 0.0;
    double s_end_m = 0.0;
    std::string profile_type;
    bool propagation_eligible = false;
    std::vector<std::string> rejection_reasons;
};

struct RawRibbonGraphOutput {
    bool ok = false;
    std::string error;
    std::vector<RawRibbonRelation> lateral_relations;
};

struct LonLinkRepairStop {
    std::uint64_t observation_id = 0;
    std::string direction;
    std::string reason;
    double s_m = 0.0;
    std::uint64_t contact_observation_id = 0;
};

struct LonLinkRepairOutput {
    bool ok = false;
    std::string error;
    std::vector<RawBoundaryEvidence> repaired_boundaries;
    std::vector<LonLinkRepairStop> repair_stops;
};

struct KeySliceNode {
    std::uint64_t node_id = 0;
    std::uint64_t observation_id = 0;
    double s_m = 0.0;
    double l_m = 0.0;
};

struct KeySliceLink {
    std::uint64_t from_node_id = 0;
    std::uint64_t to_node_id = 0;
    std::string source;
};

struct KeySliceGraphOutput {
    bool ok = false;
    std::string error;
    std::vector<double> key_slices_m;
    std::vector<KeySliceNode> nodes;
    std::vector<KeySliceLink> links;
};

struct JunctionCandidate {
    std::uint64_t candidate_id = 0;
    std::string type;
    std::vector<std::uint64_t> node_ids;
    double confidence = 0.0;
    std::vector<std::string> evidence;
};

struct JunctionEvidenceOutput {
    bool ok = false;
    std::string error;
    std::vector<JunctionCandidate> candidates;
    std::vector<JunctionCandidate> rejected;
};

struct ResegmentedBoundaryOutput {
    bool ok = false;
    std::string error;
    std::vector<RawBoundaryEvidence> boundaries;
};

struct SingleFramePropagationOutput {
    bool ok = false;
    std::string error;
    std::vector<RawRibbonRelation> propagation_relations;
};

struct PersistentBoundarySegment {
    std::uint64_t track_id = 0;
    std::vector<RawBoundaryEvidence> supporting_observations;
    std::string lifecycle_state;
    double confidence = 0.0;
};

struct PersistentLaneRelation {
    std::uint64_t relation_id = 0;
    std::uint64_t right_track_id = 0;
    std::uint64_t left_track_id = 0;
    std::string lifecycle_state;
    double confidence = 0.0;
};

struct PersistentJunction {
    std::uint64_t junction_id = 0;
    std::string type;
    std::vector<std::uint64_t> boundary_track_ids;
    std::string lifecycle_state;
    double confidence = 0.0;
};

struct SmoothTopologyOutput {
    bool ok = false;
    std::string error;
    std::vector<PersistentBoundarySegment> boundaries;
    std::vector<PersistentLaneRelation> lane_relations;
    std::vector<PersistentJunction> junctions;
    std::vector<std::string> association_trace;
};

struct ReplayFrameInput {
    std::size_t index = 0;
    std::int64_t frame_id = 0;
    std::int64_t timestamp_us = 0;
    std::string main_topic;
    std::vector<TopicSyncEntry> sync;
    std::vector<VisualBoundaryLineInput> visual_boundary_lines;
    std::optional<SmoothPoseInput> smooth_pose;
    std::optional<GnssInput> gnss;
    std::optional<NavigationRouteInput> navigation_route;
};

struct DebugLayer {
    std::string name;
    std::string stage;
    bool generated = false;
    std::vector<std::string> messages;
};

struct TopologyFrameOutput {
    std::int64_t frame_id = 0;
    VisualReferenceOutput visual_reference;
    NavigationReferenceOutput navigation_reference;
    FusedReferenceOutput fused_reference;
    RawBoundaryEvidenceOutput raw_boundary_evidence;
    RawRibbonGraphOutput raw_ribbon_graph;
    LonLinkRepairOutput lonlink_repair;
    KeySliceGraphOutput key_slice_graph;
    JunctionEvidenceOutput junction_evidence;
    ResegmentedBoundaryOutput resegmented_boundaries;
    SingleFramePropagationOutput single_frame_propagation;
    SmoothTopologyOutput smooth_topology;
    std::vector<DebugLayer> debug_layers;
    std::vector<std::string> diagnostics;
};

}  // namespace topology_map::topology_v3
