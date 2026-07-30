#pragma once

#include <cstddef>
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

struct PreprocessedVisualBoundary {
    std::uint64_t raw_ft_id = 0;
    std::string debug_label;
    std::string source;
    std::string track_line_id;
    std::vector<std::string> source_line_ids;
    int lane_id = 0;
    std::string lane_position;
    std::string semantic_type;
    std::vector<double> coeffs;
    std::vector<BoundaryPoint2d> points;
    double confidence = 1.0;
    bool rejected = false;
    std::string rejection_reason;
};

struct RawVisualBoundaryPreprocessOutput {
    bool ok = false;
    std::string error;
    std::vector<PreprocessedVisualBoundary> boundaries;
    int input_line_count = 0;
    int merged_track_count = 0;
    int hard_rejected_count = 0;
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
    std::string debug_label;
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

struct FrenetSliceIntersectionNode {
    std::uint64_t node_id = 0;
    std::uint64_t raw_ft_id = 0;
    std::string debug_label;
    int slice_index = -1;
    double s_m = 0.0;
    double l_m = 0.0;
    double x_vcs_m = 0.0;
    double y_vcs_m = 0.0;
    double x_smooth_m = 0.0;
    double y_smooth_m = 0.0;
    double confidence = 0.0;
    std::string semantic_type;
    std::vector<std::string> source_line_ids;
};

struct FrenetSliceIntersection {
    int slice_index = -1;
    double s_m = 0.0;
    double origin_x_vcs_m = 0.0;
    double origin_y_vcs_m = 0.0;
    double normal_x = 0.0;
    double normal_y = 1.0;
    std::vector<FrenetSliceIntersectionNode> nodes;
};

struct FrenetSliceIntersectionOutput {
    bool ok = false;
    std::string error;
    std::vector<FrenetSliceIntersection> slices;
    int node_count = 0;
};

struct RawFtUseDecision {
    std::uint64_t raw_ft_id = 0;
    std::string debug_label;
    std::string state;  // kept, pending, suppressed, passive_boundary
    std::string reason;
    bool direct_topology_candidate = false;
    bool passive_boundary = false;
    int sample_count = 0;
    double support_length_m = 0.0;
};

struct RawFtFilterOutput {
    bool ok = false;
    std::string error;
    std::vector<RawFtUseDecision> decisions;
    int kept_count = 0;
    int pending_count = 0;
    int suppressed_count = 0;
    int passive_boundary_count = 0;
};

struct RawFtAssociationCandidate {
    std::uint64_t from_raw_ft_id = 0;
    std::uint64_t to_raw_ft_id = 0;
    std::uint64_t shared_anchor_raw_ft_id = 0;
    bool anchor_is_left = false;
    double gap_m = 0.0;
    double endpoint_l_delta_m = 0.0;
    double width_delta_m = 0.0;
    double score = 0.0;
    std::string classification;
    std::vector<std::string> reasons;
};

struct RawFtAssociationOutput {
    bool ok = false;
    std::string error;
    std::vector<RawFtAssociationCandidate> candidates;
    int ready_continuation_count = 0;
    int ambiguous_count = 0;
};

struct FrenetSliceGraphNode {
    std::uint64_t node_id = 0;
    std::uint64_t raw_ft_id = 0;
    std::uint64_t final_ft_id = 0;
    std::string debug_label;
    int slice_index = -1;
    double s_m = 0.0;
    double l_m = 0.0;
    std::string state;  // observed, inferred, suppressed, merged
    std::string provenance;
    std::string reason;
    std::string semantic_type;
    std::uint64_t reconstruction_support_node_id = 0;
    bool reconstruction_support_is_left = false;
    double reconstruction_width_m = 0.0;
};

struct FrenetSliceGraphLonLink {
    std::uint64_t link_id = 0;
    std::uint64_t from_node_id = 0;
    std::uint64_t to_node_id = 0;
    std::string kind;  // observed, inferred, continuation, near_topology
    bool active = true;
    double score = 0.0;
    std::string reason;
};

struct FrenetSliceGraphLatLink {
    std::uint64_t link_id = 0;
    std::uint64_t right_node_id = 0;
    std::uint64_t left_node_id = 0;
    int slice_index = -1;
    double s_m = 0.0;
    double width_m = 0.0;
    bool active = true;
    std::string reason;
};

struct SliceRibbonCell {
    std::uint64_t ribbon_id = 0;
    int slice_index = -1;
    std::uint64_t right_node_id = 0;
    std::uint64_t left_node_id = 0;
    double s_m = 0.0;
    double width_m = 0.0;
    double center_l_m = 0.0;
};

struct RibbonTransitionEvidence {
    std::uint64_t transition_id = 0;
    std::uint64_t from_ribbon_id = 0;
    std::uint64_t to_ribbon_id = 0;
    std::string kind;  // continuation, split_candidate, merge_candidate, unknown
    double score = 0.0;
    std::vector<std::string> reasons;
};

struct FrenetSliceGraphOutput {
    bool ok = false;
    std::string error;
    std::vector<FrenetSliceGraphNode> nodes;
    std::vector<FrenetSliceGraphLonLink> lon_links;
    std::vector<FrenetSliceGraphLatLink> lat_links;
    std::vector<SliceRibbonCell> slice_ribbons;
    std::vector<RibbonTransitionEvidence> transitions;
    int observed_node_count = 0;
    int inferred_node_count = 0;
    int observed_lon_link_count = 0;
    int inferred_lon_link_count = 0;
    int near_topology_link_count = 0;
};

struct RawRibbonRelation {
    std::uint64_t relation_id = 0;
    std::uint64_t right_observation_id = 0;
    std::uint64_t left_observation_id = 0;
    std::string right_debug_label;
    std::string left_debug_label;
    double s_begin_m = 0.0;
    double s_end_m = 0.0;
    std::string profile_type;
    double width_begin_m = 0.0;
    double width_end_m = 0.0;
    double width_min_m = 0.0;
    double width_max_m = 0.0;
    double width_median_m = 0.0;
    double width_mad_m = 0.0;
    double max_local_width_delta_m = 0.0;
    double monotonic_ratio = 0.0;
    double stable_ratio = 0.0;
    double valid_length_m = 0.0;
    int sample_count = 0;
    int valid_sample_count = 0;
    std::vector<double> sample_s_m;
    std::vector<double> sample_right_l_m;
    std::vector<double> sample_left_l_m;
    std::vector<double> sample_width_m;
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
    RawVisualBoundaryPreprocessOutput raw_visual_preprocess;
    RawBoundaryEvidenceOutput raw_boundary_evidence;
    FrenetSliceIntersectionOutput frenet_slice_intersections;
    RawFtFilterOutput raw_ft_filter;
    RawFtAssociationOutput raw_ft_association;
    FrenetSliceGraphOutput frenet_slice_graph;
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
