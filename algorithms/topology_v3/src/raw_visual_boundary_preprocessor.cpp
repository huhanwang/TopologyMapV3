#include "topology_v3/stages/frenet_observation/raw_visual_boundary_preprocessor.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <utility>

namespace topology_map::topology_v3 {
namespace {

std::uint64_t stableId(const std::string& value) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const unsigned char c : value) {
        hash ^= static_cast<std::uint64_t>(c);
        hash *= 1099511628211ULL;
    }
    return hash;
}

double polylineLength(const std::vector<BoundaryPoint2d>& points) {
    double length = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const double dx = points[i].x_m - points[i - 1].x_m;
        const double dy = points[i].y_m - points[i - 1].y_m;
        length += std::hypot(dx, dy);
    }
    return length;
}

bool finitePoint(const BoundaryPoint2d& point) {
    return std::isfinite(point.x_m) && std::isfinite(point.y_m);
}

bool boundarySemantic(const std::string& type) {
    return type == "lane_line" || type == "road_edge" || type == "curb";
}

bool hasPreferredSource(const ReplayFrameInput& input, const std::string& source) {
    if (source.empty()) return false;
    return std::any_of(input.visual_boundary_lines.begin(), input.visual_boundary_lines.end(),
                       [&source](const auto& line) { return line.source == source; });
}

std::string trackKey(const VisualBoundaryLineInput& line) {
    return line.source + ":" + std::to_string(line.lane_id) + ":" + line.source_type;
}

PreprocessedVisualBoundary makeBoundary(const VisualBoundaryLineInput& line) {
    PreprocessedVisualBoundary boundary;
    boundary.raw_ft_id = stableId(trackKey(line));
    boundary.source = line.source;
    boundary.track_line_id = trackKey(line);
    boundary.source_line_ids.push_back(line.id);
    boundary.lane_id = line.lane_id;
    boundary.lane_position = line.lane_position;
    boundary.semantic_type = line.source_type;
    boundary.coeffs = line.coeffs;
    boundary.points = line.points;
    boundary.confidence = line.confidence;
    return boundary;
}

void appendLine(PreprocessedVisualBoundary* boundary, const VisualBoundaryLineInput& line) {
    boundary->source_line_ids.push_back(line.id);
    boundary->points.insert(boundary->points.end(), line.points.begin(), line.points.end());
    boundary->confidence = std::max(boundary->confidence, line.confidence);
    if (boundary->coeffs.empty()) boundary->coeffs = line.coeffs;
}

void reject(PreprocessedVisualBoundary* boundary, const std::string& reason) {
    boundary->rejected = true;
    boundary->rejection_reason = reason;
}

}  // namespace

RawVisualBoundaryPreprocessOutput RawVisualBoundaryPreprocessor::build(
    const ReplayFrameInput& input) const {
    RawVisualBoundaryPreprocessOutput output;
    output.input_line_count = static_cast<int>(input.visual_boundary_lines.size());

    const bool use_preferred_source = hasPreferredSource(input, cfg_.preferred_source);
    std::map<std::uint64_t, std::size_t> boundary_index_by_track;
    for (const auto& line : input.visual_boundary_lines) {
        if (use_preferred_source && line.source != cfg_.preferred_source) continue;

        const auto raw_ft_id = stableId(trackKey(line));
        const auto it = boundary_index_by_track.find(raw_ft_id);
        if (it == boundary_index_by_track.end()) {
            boundary_index_by_track[raw_ft_id] = output.boundaries.size();
            output.boundaries.push_back(makeBoundary(line));
        } else {
            appendLine(&output.boundaries[it->second], line);
        }
    }

    for (std::size_t boundary_index = 0; boundary_index < output.boundaries.size();
         ++boundary_index) {
        auto& boundary = output.boundaries[boundary_index];
        boundary.debug_label = "R" + std::to_string(boundary_index);
        std::sort(boundary.points.begin(), boundary.points.end(),
                  [](const auto& a, const auto& b) { return a.x_m < b.x_m; });
        const double length_m = polylineLength(boundary.points);

        if (!boundarySemantic(boundary.semantic_type)) {
            reject(&boundary, "unsupported_semantic_type");
        } else if (boundary.points.size() < cfg_.min_input_point_count) {
            reject(&boundary, "too_few_input_points");
        } else if (!std::all_of(boundary.points.begin(), boundary.points.end(), finitePoint)) {
            reject(&boundary, "non_finite_input_point");
        } else if (length_m < cfg_.min_hard_keep_length_m) {
            reject(&boundary, "too_short_hard_floor");
        } else if (std::any_of(boundary.points.begin(), boundary.points.end(),
                               [this](const auto& point) {
                                   return std::abs(point.y_m) > cfg_.max_abs_y_vcs_m;
                               })) {
            reject(&boundary, "outside_vcs_lateral_range");
        }
        if (boundary.rejected) ++output.hard_rejected_count;
    }

    output.merged_track_count = static_cast<int>(output.boundaries.size());
    output.ok = true;
    if (output.boundaries.empty()) output.error = "no_visual_boundaries_after_source_selection";
    return output;
}

}  // namespace topology_map::topology_v3
