#include "topology_v3/stages/frenet_observation/raw_ft_filter_builder.h"

#include <algorithm>
#include <cmath>
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

struct FtNodeSeries {
    std::uint64_t raw_ft_id = 0;
    std::string semantic_type;
    std::map<int, double> l_by_slice;
};

bool passiveSemantic(const std::string& semantic_type) {
    return semantic_type == "curb" || semantic_type == "road_edge";
}

RawFtUseDecision* findDecision(
    std::map<std::uint64_t, RawFtUseDecision>* decisions,
    std::uint64_t raw_ft_id) {
    if (!decisions) return nullptr;
    auto it = decisions->find(raw_ft_id);
    return it == decisions->end() ? nullptr : &it->second;
}

bool isActiveLaneLineDecision(const RawFtUseDecision& decision, const RawFtStats& stats) {
    return stats.semantic_type == "lane_line" && decision.state == "kept";
}

bool isCloseDuplicateShorter(
    const FtNodeSeries& shorter,
    const FtNodeSeries& longer,
    const RawFtUseDecision& shorter_decision,
    const RawFtUseDecision& longer_decision,
    const RawFtFilterBuilder::Config& cfg) {
    if (shorter.semantic_type != "lane_line" || longer.semantic_type != "lane_line") return false;
    if (shorter_decision.support_length_m <= 0.0) return false;
    if (longer_decision.support_length_m <
        shorter_decision.support_length_m * cfg.min_close_duplicate_length_ratio) {
        return false;
    }

    std::vector<double> l_deltas;
    for (const auto& [slice_index, shorter_l] : shorter.l_by_slice) {
        const auto it = longer.l_by_slice.find(slice_index);
        if (it == longer.l_by_slice.end()) continue;
        l_deltas.push_back(std::abs(shorter_l - it->second));
    }
    if (static_cast<int>(l_deltas.size()) < cfg.min_close_duplicate_overlap_count) {
        return false;
    }
    const double short_overlap_ratio =
        static_cast<double>(l_deltas.size()) /
        static_cast<double>(std::max<std::size_t>(1, shorter.l_by_slice.size()));
    if (short_overlap_ratio < cfg.min_close_duplicate_short_overlap_ratio) return false;

    std::sort(l_deltas.begin(), l_deltas.end());
    const double median = l_deltas[l_deltas.size() / 2];
    const double max_delta = l_deltas.back();
    return median <= cfg.max_close_duplicate_median_l_delta_m &&
           max_delta <= cfg.max_close_duplicate_l_delta_m;
}

double clamp01(double value) {
    if (value < 0.0) return 0.0;
    if (value > 1.0) return 1.0;
    return value;
}

double rampDownScore(double value, double full_score_value, double zero_score_value) {
    if (zero_score_value <= full_score_value) return value <= full_score_value ? 1.0 : 0.0;
    return clamp01((zero_score_value - value) / (zero_score_value - full_score_value));
}

bool findNarrowestAdjacentLaneWidth(
    std::uint64_t raw_ft_id,
    const FrenetSliceIntersectionOutput& intersections,
    const std::map<std::uint64_t, RawFtUseDecision>& decisions,
    const RawFtFilterBuilder::Config& cfg,
    double* narrowest_median_width_m) {
    if (narrowest_median_width_m) *narrowest_median_width_m = 0.0;
    std::map<std::uint64_t, std::vector<double>> widths_by_neighbor;
    for (const auto& slice : intersections.slices) {
        std::vector<const FrenetSliceIntersectionNode*> active_lane_nodes;
        for (const auto& node : slice.nodes) {
            const auto decision_it = decisions.find(node.raw_ft_id);
            if (decision_it == decisions.end()) continue;
            if (decision_it->second.state != "kept") continue;
            if (node.semantic_type != "lane_line") continue;
            active_lane_nodes.push_back(&node);
        }
        std::sort(active_lane_nodes.begin(), active_lane_nodes.end(), [](const auto* a, const auto* b) {
            if (a->l_m != b->l_m) return a->l_m < b->l_m;
            return a->raw_ft_id < b->raw_ft_id;
        });

        for (std::size_t i = 0; i < active_lane_nodes.size(); ++i) {
            if (active_lane_nodes[i]->raw_ft_id != raw_ft_id) continue;
            for (std::size_t neighbor_index : {i == 0 ? i : i - 1, i + 1}) {
                if (neighbor_index == i || neighbor_index >= active_lane_nodes.size()) continue;
                const auto* neighbor = active_lane_nodes[neighbor_index];
                widths_by_neighbor[neighbor->raw_ft_id].push_back(
                    std::abs(active_lane_nodes[i]->l_m - neighbor->l_m));
            }
            break;
        }
    }

    bool found = false;
    double best_median = 0.0;
    for (auto& [_, widths] : widths_by_neighbor) {
        if (static_cast<int>(widths.size()) < cfg.min_far_short_narrow_overlap_count) continue;
        std::sort(widths.begin(), widths.end());
        const double median = widths[widths.size() / 2];
        if (!found || median < best_median) {
            best_median = median;
            found = true;
        }
    }
    if (found && narrowest_median_width_m) *narrowest_median_width_m = best_median;
    return found;
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
    std::map<std::uint64_t, FtNodeSeries> nodes_by_ft;
    for (const auto& slice : intersections.slices) {
        for (const auto& node : slice.nodes) {
            updateStats(&stats_by_ft[node.raw_ft_id], node);
            auto& series = nodes_by_ft[node.raw_ft_id];
            series.raw_ft_id = node.raw_ft_id;
            series.semantic_type = node.semantic_type;
            series.l_by_slice[node.slice_index] = node.l_m;
        }
    }

    std::map<std::uint64_t, RawFtUseDecision> decisions_by_ft;
    for (const auto& [raw_ft_id, stats] : stats_by_ft) {
        RawFtUseDecision decision;
        decision.raw_ft_id = raw_ft_id;
        decision.debug_label = stats.debug_label;
        decision.sample_count = stats.sample_count;
        decision.support_length_m = stats.max_s_m - stats.min_s_m;

        if (passiveSemantic(stats.semantic_type)) {
            decision.state = "passive_boundary";
            decision.reason = "passive_boundary_semantic";
            decision.passive_boundary = true;
        } else if (decision.sample_count < cfg_.min_sample_count) {
            decision.state = "suppressed";
            decision.reason = "too_few_slice_nodes";
        } else if (decision.support_length_m < cfg_.min_support_length_m) {
            decision.state = "suppressed";
            decision.reason = "too_short_slice_support";
        } else {
            decision.state = "kept";
            decision.reason = "lane_line_direct_candidate";
            decision.direct_topology_candidate = true;
        }

        decisions_by_ft[raw_ft_id] = std::move(decision);
    }

    for (const auto& [short_id, short_series] : nodes_by_ft) {
        auto* short_decision = findDecision(&decisions_by_ft, short_id);
        if (!short_decision) continue;
        const auto short_stats_it = stats_by_ft.find(short_id);
        if (short_stats_it == stats_by_ft.end()) continue;
        if (!isActiveLaneLineDecision(*short_decision, short_stats_it->second)) continue;

        for (const auto& [long_id, long_series] : nodes_by_ft) {
            if (short_id == long_id) continue;
            auto* long_decision = findDecision(&decisions_by_ft, long_id);
            if (!long_decision) continue;
            const auto long_stats_it = stats_by_ft.find(long_id);
            if (long_stats_it == stats_by_ft.end()) continue;
            if (!isActiveLaneLineDecision(*long_decision, long_stats_it->second)) continue;
            if (!isCloseDuplicateShorter(short_series, long_series, *short_decision,
                                         *long_decision, cfg_)) {
                continue;
            }

            short_decision->state = "suppressed";
            short_decision->reason = "close_duplicate_shorter_raw_ft";
            short_decision->direct_topology_candidate = false;
            break;
        }
    }

    for (const auto& [raw_ft_id, stats] : stats_by_ft) {
        auto* decision = findDecision(&decisions_by_ft, raw_ft_id);
        if (!decision) continue;
        if (!isActiveLaneLineDecision(*decision, stats)) continue;
        if (stats.min_s_m < cfg_.far_short_start_s_m) continue;

        const double short_score = rampDownScore(decision->support_length_m,
                                                 cfg_.far_short_full_score_length_m,
                                                 cfg_.far_short_zero_score_length_m);
        const bool very_short =
            decision->support_length_m <= cfg_.far_short_full_score_length_m;

        double narrowest_width_m = 0.0;
        const bool has_adjacent_lane = findNarrowestAdjacentLaneWidth(
            raw_ft_id, intersections, decisions_by_ft, cfg_, &narrowest_width_m);
        if (!very_short && !has_adjacent_lane) {
            continue;
        }
        const double narrow_score = has_adjacent_lane
            ? rampDownScore(narrowest_width_m, cfg_.far_narrow_full_score_width_m,
                            cfg_.far_narrow_zero_score_width_m)
            : 0.0;
        const double noise_score = 0.5 * short_score + 0.5 * narrow_score;
        if (!very_short && noise_score < cfg_.far_short_noise_score_threshold) continue;

        decision->state = "suppressed";
        decision->reason = "far_short_noise_raw_ft";
        decision->direct_topology_candidate = false;
    }

    output.decisions.reserve(decisions_by_ft.size());
    for (auto& [_, decision] : decisions_by_ft) {
        if (decision.state == "kept") {
            ++output.kept_count;
        } else if (decision.state == "pending") {
            ++output.pending_count;
        } else if (decision.state == "passive_boundary") {
            ++output.passive_boundary_count;
        } else if (decision.state == "suppressed") {
            ++output.suppressed_count;
        }
        output.decisions.push_back(std::move(decision));
    }

    output.ok = true;
    if (output.decisions.empty()) output.error = "no_raw_ft_filter_decisions";
    return output;
}

}  // namespace topology_map::topology_v3
