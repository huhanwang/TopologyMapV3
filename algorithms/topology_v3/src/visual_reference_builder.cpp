#include "topology_v3/stages/visual_reference_builder.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <tuple>

namespace topology_map::topology_v3 {
namespace {

struct BoundaryStats {
    const VisualBoundaryLineInput* line = nullptr;
    std::pair<double, double> s_range{0.0, 0.0};
    std::pair<double, double> d_range{0.0, 0.0};
    double length_m = 0.0;
};

struct VisualCandidate {
    const BoundaryStats* left = nullptr;
    const BoundaryStats* right = nullptr;
    std::pair<double, double> s_range{0.0, 0.0};
    std::vector<double> center_coeffs;
    double confidence = 0.0;
    std::string method;
};

double distance2d(double ax, double ay, double bx, double by) {
    return std::hypot(ax - bx, ay - by);
}

double polyValue(const std::vector<double>& coeffs, double x) {
    double total = 0.0;
    double power = 1.0;
    for (double coeff : coeffs) {
        total += coeff * power;
        power *= x;
    }
    return total;
}

double polylineLength(const std::vector<BoundaryPoint2d>& points) {
    double length = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        length += distance2d(points[i - 1].x_m, points[i - 1].y_m,
                             points[i].x_m, points[i].y_m);
    }
    return length;
}

std::vector<double> meanCoeffs(const std::vector<double>& left,
                               const std::vector<double>& right) {
    const std::size_t n = std::max(left.size(), right.size());
    std::vector<double> out(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        out[i] = 0.5 * ((i < left.size() ? left[i] : 0.0) +
                        (i < right.size() ? right[i] : 0.0));
    }
    return out;
}

BoundaryStats buildStats(const VisualBoundaryLineInput& line) {
    BoundaryStats stats;
    stats.line = &line;
    if (line.points.empty()) return stats;
    double min_x = line.points.front().x_m;
    double max_x = line.points.front().x_m;
    double min_y = line.points.front().y_m;
    double max_y = line.points.front().y_m;
    for (const auto& point : line.points) {
        min_x = std::min(min_x, point.x_m);
        max_x = std::max(max_x, point.x_m);
        min_y = std::min(min_y, point.y_m);
        max_y = std::max(max_y, point.y_m);
    }
    stats.s_range = {min_x, max_x};
    stats.d_range = {min_y, max_y};
    stats.length_m = polylineLength(line.points);
    return stats;
}

bool isLongitudinalBoundary(
    const BoundaryStats& stats,
    const VisualReferenceBuilder::Config& cfg) {
    if (stats.line == nullptr) return false;
    const double x_span = stats.s_range.second - stats.s_range.first;
    if (x_span < cfg.min_line_x_span_m) return false;
    if (std::max(std::abs(stats.d_range.first), std::abs(stats.d_range.second)) >
        cfg.max_abs_d_m) {
        return false;
    }
    if (stats.line->source_type != "lane_line") return false;
    return !stats.line->coeffs.empty();
}

std::string selectSource(
    const std::vector<BoundaryStats>& stats,
    const VisualReferenceBuilder::Config& cfg) {
    for (const auto& source : cfg.preferred_sources) {
        int count = 0;
        for (const auto& item : stats) {
            if (item.line && item.line->source == source &&
                isLongitudinalBoundary(item, cfg)) {
                ++count;
            }
        }
        if (count >= 2) return source;
    }
    return cfg.preferred_sources.empty() ? "smooth_bev_lanes" : cfg.preferred_sources.front();
}

const BoundaryStats* bestHostLine(const std::vector<const BoundaryStats*>& lines,
                                  const char* position) {
    const BoundaryStats* best = nullptr;
    for (const auto* line : lines) {
        if (!line || !line->line || line->line->lane_position != position) continue;
        if (!best || std::tie(line->length_m, line->line->confidence) >
                         std::tie(best->length_m, best->line->confidence)) {
            best = line;
        }
    }
    return best;
}

std::pair<double, double> extendedRange(
    double start,
    double end,
    const VisualReferenceBuilder::Config& cfg) {
    return {start - cfg.max_reference_extension_m,
            std::min(cfg.max_length_m, end + cfg.max_reference_extension_m)};
}

std::optional<VisualCandidate> pairReference(
    const BoundaryStats& left,
    const BoundaryStats& right,
    const VisualReferenceBuilder::Config& cfg) {
    const double overlap_start = std::max(left.s_range.first, right.s_range.first);
    const double overlap_end = std::min(left.s_range.second, right.s_range.second);
    if (overlap_end - overlap_start < cfg.min_reference_length_m) return std::nullopt;
    VisualCandidate candidate;
    candidate.left = &left;
    candidate.right = &right;
    candidate.s_range = extendedRange(overlap_start, overlap_end, cfg);
    candidate.center_coeffs = meanCoeffs(left.line->coeffs, right.line->coeffs);
    candidate.confidence = std::min(left.line->confidence, right.line->confidence);
    candidate.method = "host_pair";
    return candidate;
}

std::optional<VisualCandidate> singleLineReference(
    const BoundaryStats& line,
    const VisualReferenceBuilder::Config& cfg,
    const std::string& method) {
    if (line.s_range.second - line.s_range.first < cfg.min_reference_length_m) {
        return std::nullopt;
    }
    VisualCandidate candidate;
    candidate.left = &line;
    candidate.right = &line;
    candidate.s_range = extendedRange(line.s_range.first, line.s_range.second, cfg);
    candidate.center_coeffs = line.line->coeffs;
    candidate.confidence = line.line->confidence;
    candidate.method = method;
    return candidate;
}

std::optional<VisualCandidate> selectReference(
    const std::vector<BoundaryStats>& source_stats,
    const VisualReferenceBuilder::Config& cfg) {
    std::vector<const BoundaryStats*> candidates;
    for (const auto& line : source_stats) {
        if (isLongitudinalBoundary(line, cfg)) candidates.push_back(&line);
    }
    const auto* host_left = bestHostLine(candidates, "HOST_LEFT");
    const auto* host_right = bestHostLine(candidates, "HOST_RIGHT");
    if (host_left && host_right) {
        auto candidate = pairReference(*host_left, *host_right, cfg);
        if (candidate) return candidate;
    }
    if (host_left) return singleLineReference(*host_left, cfg, "host_left_single");
    if (host_right) return singleLineReference(*host_right, cfg, "host_right_single");
    if (candidates.empty()) return std::nullopt;
    const auto* longest = *std::max_element(
        candidates.begin(), candidates.end(), [](const auto* a, const auto* b) {
            return std::make_tuple(a->length_m, a->line->points.size()) <
                   std::make_tuple(b->length_m, b->line->points.size());
        });
    return singleLineReference(*longest, cfg, "longest_lane_line");
}

std::vector<ReferencePoint> sampleReferencePoints(
    const VisualCandidate& ref,
    double spacing_m) {
    std::vector<ReferencePoint> points;
    const double start = ref.s_range.first;
    const double end = ref.s_range.second;
    if (end <= start) {
        points.push_back({start, start, polyValue(ref.center_coeffs, start)});
        return points;
    }
    const int count = static_cast<int>(std::floor((end - start) / std::max(1e-6, spacing_m)));
    for (int i = 0; i <= count; ++i) {
        const double s = start + i * spacing_m;
        points.push_back({s, s, polyValue(ref.center_coeffs, s)});
    }
    if (std::abs(points.back().s_m - end) > 1e-6) {
        points.push_back({end, end, polyValue(ref.center_coeffs, end)});
    }
    return points;
}

}  // namespace

VisualReferenceOutput VisualReferenceBuilder::build(const ReplayFrameInput& input) const {
    VisualReferenceOutput result;
    std::vector<BoundaryStats> stats;
    stats.reserve(input.visual_boundary_lines.size());
    for (const auto& line : input.visual_boundary_lines) stats.push_back(buildStats(line));
    const auto selected_source = selectSource(stats, cfg_);
    std::vector<BoundaryStats> source_stats;
    for (const auto& item : stats) {
        if (item.line && item.line->source == selected_source) source_stats.push_back(item);
    }
    const auto reference = selectReference(source_stats, cfg_);
    result.selected_source = selected_source;
    result.input_line_count = static_cast<int>(stats.size());
    result.selected_source_line_count = static_cast<int>(source_stats.size());
    if (!reference) {
        result.error = "no_visual_reference_candidate";
        return result;
    }
    result.ok = true;
    result.method = reference->method;
    result.left_line_id = reference->left && reference->left->line ? reference->left->line->id : "";
    result.right_line_id = reference->right && reference->right->line ? reference->right->line->id : "";
    result.confidence = reference->confidence;
    result.s_begin_m = reference->s_range.first;
    result.s_end_m = reference->s_range.second;
    result.center_coeffs = reference->center_coeffs;
    result.points = sampleReferencePoints(*reference, cfg_.slice_spacing_m);
    result.left_lane_position = reference->left && reference->left->line ? reference->left->line->lane_position : "";
    result.right_lane_position = reference->right && reference->right->line ? reference->right->line->lane_position : "";
    result.left_source_type = reference->left && reference->left->line ? reference->left->line->source_type : "";
    result.right_source_type = reference->right && reference->right->line ? reference->right->line->source_type : "";
    result.left_line_x_span_m = reference->left ? reference->left->s_range.second - reference->left->s_range.first : 0.0;
    result.right_line_x_span_m = reference->right ? reference->right->s_range.second - reference->right->s_range.first : 0.0;
    result.left_line_length_m = reference->left ? reference->left->length_m : 0.0;
    result.right_line_length_m = reference->right ? reference->right->length_m : 0.0;
    return result;
}

}  // namespace topology_map::topology_v3
