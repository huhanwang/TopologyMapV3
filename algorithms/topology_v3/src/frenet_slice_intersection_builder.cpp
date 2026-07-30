#include "topology_v3/stages/frenet_observation/frenet_slice_intersection_builder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <utility>

#include "topology_v3/geometry/frenet_projector.h"

namespace topology_map::topology_v3 {
namespace {

struct ProjectedInputPoint {
    double s_m = 0.0;
    double l_m = 0.0;
    double x_vcs_m = 0.0;
    double y_vcs_m = 0.0;
};

std::uint64_t stableId(const std::string& value) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const unsigned char c : value) {
        hash ^= static_cast<std::uint64_t>(c);
        hash *= 1099511628211ULL;
    }
    return hash;
}

BoundaryPoint2d vcsToSmooth(const BoundaryPoint2d& point, const SmoothPoseInput& pose) {
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    return {pose.x_m + c * point.x_m - s * point.y_m,
            pose.y_m + s * point.x_m + c * point.y_m};
}

bool between(double value, double a, double b) {
    constexpr double kEps = 1e-6;
    return value >= std::min(a, b) - kEps && value <= std::max(a, b) + kEps;
}

FrenetSliceIntersectionNode interpolateNode(
    const PreprocessedVisualBoundary& boundary,
    const ProjectedInputPoint& a,
    const ProjectedInputPoint& b,
    int slice_index,
    double slice_s_m,
    const SmoothPoseInput* smooth_pose) {
    const double ds = b.s_m - a.s_m;
    const double t = std::abs(ds) < 1e-9 ? 0.0 : (slice_s_m - a.s_m) / ds;
    const double clamped_t = std::max(0.0, std::min(1.0, t));
    const double l_m = a.l_m + (b.l_m - a.l_m) * clamped_t;
    const double x_vcs_m = a.x_vcs_m + (b.x_vcs_m - a.x_vcs_m) * clamped_t;
    const double y_vcs_m = a.y_vcs_m + (b.y_vcs_m - a.y_vcs_m) * clamped_t;

    FrenetSliceIntersectionNode node;
    node.raw_ft_id = boundary.raw_ft_id;
    node.debug_label = boundary.debug_label;
    node.slice_index = slice_index;
    node.s_m = slice_s_m;
    node.l_m = l_m;
    node.x_vcs_m = x_vcs_m;
    node.y_vcs_m = y_vcs_m;
    if (smooth_pose && smooth_pose->valid) {
        const auto smooth = vcsToSmooth({x_vcs_m, y_vcs_m}, *smooth_pose);
        node.x_smooth_m = smooth.x_m;
        node.y_smooth_m = smooth.y_m;
    } else {
        node.x_smooth_m = std::numeric_limits<double>::quiet_NaN();
        node.y_smooth_m = std::numeric_limits<double>::quiet_NaN();
    }
    node.confidence = boundary.confidence;
    node.semantic_type = boundary.semantic_type;
    node.source_line_ids = boundary.source_line_ids;
    node.node_id = stableId(boundary.track_line_id + ":" + std::to_string(slice_index));
    return node;
}

}  // namespace

FrenetSliceIntersectionOutput FrenetSliceIntersectionBuilder::build(
    const RawVisualBoundaryPreprocessOutput& preprocessed,
    const FusedReferenceOutput& fused_reference,
    const SmoothPoseInput* smooth_pose) const {
    FrenetSliceIntersectionOutput output;
    if (!preprocessed.ok) {
        output.error = preprocessed.error.empty() ? "invalid_raw_visual_preprocess"
                                                 : preprocessed.error;
        return output;
    }
    if (!fused_reference.ok || fused_reference.points.size() < 2) {
        output.error = "invalid_fused_reference";
        return output;
    }

    const FrenetProjector projector(fused_reference);
    if (!projector.ok()) {
        output.error = projector.error();
        return output;
    }

    const auto slice_values = projector.sampleSValues(cfg_.slice_step_m);
    output.slices.reserve(slice_values.size());
    for (std::size_t i = 0; i < slice_values.size(); ++i) {
        FrenetPose pose;
        if (!projector.poseAt(slice_values[i], &pose)) continue;
        FrenetSliceIntersection slice;
        slice.slice_index = static_cast<int>(i);
        slice.s_m = slice_values[i];
        slice.origin_x_vcs_m = pose.x_m;
        slice.origin_y_vcs_m = pose.y_m;
        slice.normal_x = pose.normal_x;
        slice.normal_y = pose.normal_y;
        output.slices.push_back(std::move(slice));
    }

    for (const auto& boundary : preprocessed.boundaries) {
        if (boundary.rejected || boundary.points.size() < 2) continue;

        std::vector<ProjectedInputPoint> projected;
        projected.reserve(boundary.points.size());
        for (const auto& point : boundary.points) {
            FrenetProjection projection;
            if (!projector.project(point.x_m, point.y_m, &projection)) continue;
            if (std::abs(projection.l_m) > cfg_.max_abs_l_m) continue;
            projected.push_back({projection.s_m, projection.l_m, point.x_m, point.y_m});
        }
        if (projected.size() < 2) continue;
        std::sort(projected.begin(), projected.end(),
                  [](const auto& a, const auto& b) { return a.s_m < b.s_m; });

        std::set<int> emitted_slices;
        for (std::size_t segment_index = 1; segment_index < projected.size(); ++segment_index) {
            const auto& a = projected[segment_index - 1];
            const auto& b = projected[segment_index];
            if (std::abs(b.s_m - a.s_m) < 1e-6) continue;
            for (auto& slice : output.slices) {
                if (!between(slice.s_m, a.s_m, b.s_m)) continue;
                if (!emitted_slices.insert(slice.slice_index).second) continue;
                slice.nodes.push_back(interpolateNode(boundary, a, b, slice.slice_index,
                                                      slice.s_m, smooth_pose));
                ++output.node_count;
            }
        }
    }

    for (auto& slice : output.slices) {
        std::sort(slice.nodes.begin(), slice.nodes.end(),
                  [](const auto& a, const auto& b) { return a.l_m < b.l_m; });
    }

    output.ok = true;
    if (output.node_count == 0) output.error = "no_frenet_slice_intersections";
    return output;
}

}  // namespace topology_map::topology_v3
