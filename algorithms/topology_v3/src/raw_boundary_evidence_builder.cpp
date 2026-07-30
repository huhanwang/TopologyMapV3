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

bool hasPreferredSource(const ReplayFrameInput& input, const std::string& source) {
    if (source.empty()) return false;
    return std::any_of(input.visual_boundary_lines.begin(), input.visual_boundary_lines.end(),
                       [&source](const auto& line) { return line.source == source; });
}

RawBoundaryEvidence makeBoundaryEvidence(
    const PreprocessedVisualBoundary& boundary,
    const FrenetProjector& projector,
    const SmoothPoseInput* smooth_pose,
    const RawBoundaryEvidenceBuilder::Config& cfg,
    std::size_t evidence_index) {
    RawBoundaryEvidence evidence;
    evidence.observation_id = boundary.raw_ft_id;
    evidence.debug_label = "E" + std::to_string(evidence_index);
    evidence.source_identity_ids = boundary.source_line_ids;
    if (!boundary.source.empty()) evidence.source_identity_ids.push_back(boundary.source);
    if (!boundary.track_line_id.empty()) evidence.source_identity_ids.push_back(boundary.track_line_id);
    evidence.semantic_type = boundary.semantic_type;
    evidence.quality = boundary.confidence;
    evidence.samples.reserve(boundary.points.size());

    std::size_t source_index = 0;
    for (const auto& raw_point : boundary.points) {
        FrenetProjection projection;
        if (!projector.project(raw_point.x_m, raw_point.y_m, &projection)) {
            ++source_index;
            continue;
        }
        if (std::abs(projection.l_m) > cfg.max_abs_l_m) {
            ++source_index;
            continue;
        }

        FrenetSamplePoint sample;
        sample.id = stableId(boundary.track_line_id + ":" + std::to_string(source_index));
        sample.s_m = projection.s_m;
        sample.l_m = projection.l_m;
        sample.x_vcs_m = raw_point.x_m;
        sample.y_vcs_m = raw_point.y_m;
        if (smooth_pose && smooth_pose->valid) {
            const auto smooth = vcsToSmooth(raw_point, *smooth_pose);
            sample.x_smooth_m = smooth.x_m;
            sample.y_smooth_m = smooth.y_m;
        } else {
            sample.x_smooth_m = std::numeric_limits<double>::quiet_NaN();
            sample.y_smooth_m = std::numeric_limits<double>::quiet_NaN();
        }
        sample.confidence = boundary.confidence;
        sample.source_line_id =
            boundary.source_line_ids.empty() ? boundary.track_line_id : boundary.source_line_ids.front();
        sample.track_line_id = boundary.track_line_id;
        sample.semantic_type = boundary.semantic_type;
        evidence.samples.push_back(std::move(sample));
        ++source_index;
    }

    std::sort(evidence.samples.begin(), evidence.samples.end(),
              [](const auto& a, const auto& b) { return a.s_m < b.s_m; });
    return evidence;
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
        std::sort(boundary.samples.begin(), boundary.samples.end(),
                  [](const auto& a, const auto& b) { return a.s_m < b.s_m; });
        output.boundaries.push_back(std::move(boundary));
        ++evidence_index;
    }

    output.ok = true;
    if (output.boundaries.empty()) output.error = "no_projected_raw_boundary_evidence";
    return output;
}

RawBoundaryEvidenceOutput RawBoundaryEvidenceBuilder::build(
    const RawVisualBoundaryPreprocessOutput& preprocessed,
    const FusedReferenceOutput& fused_reference,
    const SmoothPoseInput* smooth_pose) const {
    RawBoundaryEvidenceOutput output;
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

    std::size_t evidence_index = 0;
    for (const auto& boundary : preprocessed.boundaries) {
        if (boundary.rejected) continue;
        auto evidence = makeBoundaryEvidence(boundary, projector, smooth_pose, cfg_, evidence_index);
        if (evidence.samples.size() < cfg_.min_sample_count) continue;
        output.boundaries.push_back(std::move(evidence));
        ++evidence_index;
    }

    output.ok = true;
    if (output.boundaries.empty()) output.error = "no_projected_raw_boundary_evidence";
    return output;
}

}  // namespace topology_map::topology_v3
