#include "topology_v3/stages/frenet_observation/raw_ft_filter_builder.h"

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace topology_map::topology_v3 {
namespace {

struct RawFtStats {
    std::uint64_t raw_ft_id = 0;
    std::string debug_label;
    std::string semantic_type;
    int sample_count = 0;
    double min_s_m = 0.0;
    double max_s_m = 0.0;
};

bool passiveSemantic(const std::string& semantic_type) {
    return semantic_type == "curb" || semantic_type == "road_edge";
}

void updateStats(RawFtStats* stats, const FrenetSliceIntersectionNode& node) {
    if (!stats) return;
    if (stats->sample_count == 0) {
        stats->raw_ft_id = node.raw_ft_id;
        stats->debug_label = node.debug_label;
        stats->semantic_type = node.semantic_type;
        stats->min_s_m = node.s_m;
        stats->max_s_m = node.s_m;
    } else {
        stats->min_s_m = std::min(stats->min_s_m, node.s_m);
        stats->max_s_m = std::max(stats->max_s_m, node.s_m);
    }
    ++stats->sample_count;
}

}  // namespace

RawFtFilterOutput RawFtFilterBuilder::build(
    const FrenetSliceIntersectionOutput& intersections) const {
    RawFtFilterOutput output;
    if (!intersections.ok) {
        output.error = intersections.error.empty() ? "invalid_frenet_slice_intersections"
                                                  : intersections.error;
        return output;
    }

    std::map<std::uint64_t, RawFtStats> stats_by_ft;
    for (const auto& slice : intersections.slices) {
        for (const auto& node : slice.nodes) {
            updateStats(&stats_by_ft[node.raw_ft_id], node);
        }
    }

    output.decisions.reserve(stats_by_ft.size());
    for (const auto& [raw_ft_id, stats] : stats_by_ft) {
        RawFtUseDecision decision;
        decision.raw_ft_id = raw_ft_id;
        decision.debug_label = stats.debug_label;
        decision.sample_count = stats.sample_count;
        decision.support_length_m = stats.max_s_m - stats.min_s_m;

        if (decision.sample_count < cfg_.min_sample_count) {
            decision.state = "suppressed";
            decision.reason = "too_few_slice_nodes";
            ++output.suppressed_count;
        } else if (decision.support_length_m < cfg_.min_support_length_m) {
            decision.state = "suppressed";
            decision.reason = "too_short_slice_support";
            ++output.suppressed_count;
        } else if (passiveSemantic(stats.semantic_type)) {
            decision.state = "passive_boundary";
            decision.reason = "passive_boundary_semantic";
            decision.passive_boundary = true;
            ++output.passive_boundary_count;
        } else {
            decision.state = "kept";
            decision.reason = "lane_line_direct_candidate";
            decision.direct_topology_candidate = true;
            ++output.kept_count;
        }

        output.decisions.push_back(std::move(decision));
    }

    output.ok = true;
    if (output.decisions.empty()) output.error = "no_raw_ft_filter_decisions";
    return output;
}

}  // namespace topology_map::topology_v3
