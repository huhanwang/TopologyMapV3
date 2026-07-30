#include "topology_v3/stages/raw_boundary_evidence_builder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include "topology_v3/geometry/frenet_projector.h"

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

BoundaryPoint2d vcsToSmooth(const BoundaryPoint2d& point, const SmoothPoseInput& pose) {
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    return {pose.x_m + c * point.x_m - s * point.y_m,
            pose.y_m + s * point.x_m + c * point.y_m};
}

bool sampleOrderIsIncreasing(const RawBoundaryEvidence& boundary) {
    return boundary.samples.size() < 2 ||
           boundary.samples.front().s_m <= boundary.samples.back().s_m;
}

bool hasPreferredSource(const ReplayFrameInput& input, const std::string& source) {
    if (source.empty()) return false;
    return std::any_of(input.visual_boundary_lines.begin(), input.visual_boundary_lines.end(),
                       [&source](const auto& line) { return line.source == source; });
}

}  // namespace

RawBoundaryEvidenceOutput RawBoundaryEvidenceBuilder::build(
    const ReplayFrameInput& input,
    const FusedReferenceOutput& fused_reference) const {
    RawBoundaryEvidenceOutput output;
    if (!fused_reference.ok || fused_reference.points.size() < 2) {
        output.error = "invalid_fused_reference";
        return output;
    }

    const FrenetProjector projector(fused_reference);
    if (!projector.ok()) {
        output.error = projector.error();
        return output;
    }

    const bool use_preferred_source = hasPreferredSource(input, cfg_.preferred_source);
    std::size_t evidence_index = 0;
    for (std::size_t line_index = 0; line_index < input.visual_boundary_lines.size(); ++line_index) {
        const auto& line = input.visual_boundary_lines[line_index];
        if (use_preferred_source && line.source != cfg_.preferred_source) continue;
        if (line.points.size() < cfg_.min_sample_count) continue;

        RawBoundaryEvidence boundary;
        boundary.observation_id = stableId(line.id);
        boundary.debug_label = "E" + std::to_string(evidence_index);
        boundary.source_identity_ids.push_back(line.id);
        if (!line.source.empty()) boundary.source_identity_ids.push_back(line.source);
        boundary.semantic_type = line.source_type;
        boundary.quality = line.confidence;
        boundary.samples.reserve(line.points.size());

        std::size_t source_index = 0;
        for (const auto& raw_point : line.points) {
            FrenetProjection projection;
            if (!projector.project(raw_point.x_m, raw_point.y_m, &projection)) {
                ++source_index;
                continue;
            }
            if (std::abs(projection.l_m) > cfg_.max_abs_l_m) {
                ++source_index;
                continue;
            }

            FrenetSamplePoint sample;
            sample.id = stableId(line.id + ":" + std::to_string(source_index));
            sample.s_m = projection.s_m;
            sample.l_m = projection.l_m;
            sample.x_vcs_m = raw_point.x_m;
            sample.y_vcs_m = raw_point.y_m;
            if (input.smooth_pose.has_value() && input.smooth_pose->valid) {
                const auto smooth = vcsToSmooth(raw_point, *input.smooth_pose);
                sample.x_smooth_m = smooth.x_m;
                sample.y_smooth_m = smooth.y_m;
            } else {
                sample.x_smooth_m = std::numeric_limits<double>::quiet_NaN();
                sample.y_smooth_m = std::numeric_limits<double>::quiet_NaN();
            }
            sample.confidence = line.confidence;
            sample.source_line_id = line.id;
            sample.track_line_id = line.id;
            sample.semantic_type = line.source_type;
            boundary.samples.push_back(std::move(sample));
            ++source_index;
        }

        if (boundary.samples.size() < cfg_.min_sample_count) continue;
        if (!sampleOrderIsIncreasing(boundary)) {
            std::reverse(boundary.samples.begin(), boundary.samples.end());
        }
        output.boundaries.push_back(std::move(boundary));
        ++evidence_index;
    }

    output.ok = true;
    if (output.boundaries.empty()) output.error = "no_projected_raw_boundary_evidence";
    return output;
}

}  // namespace topology_map::topology_v3
