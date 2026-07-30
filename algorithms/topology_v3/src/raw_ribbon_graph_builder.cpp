#include "topology_v3/stages/raw_ribbon_graph_builder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace topology_map::topology_v3 {
namespace {

struct BoundaryView {
    const RawBoundaryEvidence* boundary = nullptr;
    double mean_l_m = 0.0;
    double s_begin_m = 0.0;
    double s_end_m = 0.0;
};

std::uint64_t mixIds(std::uint64_t right_id, std::uint64_t left_id) {
    std::uint64_t hash = 1469598103934665603ULL;
    const auto mix = [&hash](std::uint64_t value) {
        for (int i = 0; i < 8; ++i) {
            hash ^= static_cast<unsigned char>((value >> (i * 8)) & 0xffU);
            hash *= 1099511628211ULL;
        }
    };
    mix(right_id);
    mix(left_id);
    return hash;
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 0) {
        return 0.5 * (values[middle - 1] + values[middle]);
    }
    return values[middle];
}

void addReason(RawRibbonRelation* relation, std::string reason) {
    if (!relation) return;
    if (std::find(relation->rejection_reasons.begin(),
                  relation->rejection_reasons.end(),
                  reason) == relation->rejection_reasons.end()) {
        relation->rejection_reasons.push_back(std::move(reason));
    }
}

bool sampleAtS(const RawBoundaryEvidence& boundary, double s_m, double* l_m) {
    if (!l_m || boundary.samples.size() < 2) return false;
    const auto& samples = boundary.samples;
    if (s_m < samples.front().s_m - 1e-6 || s_m > samples.back().s_m + 1e-6) {
        return false;
    }
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const auto& before = samples[i - 1];
        const auto& after = samples[i];
        if (s_m < before.s_m - 1e-6 || s_m > after.s_m + 1e-6) continue;
        const double span = after.s_m - before.s_m;
        if (span <= 1e-6) continue;
        const double ratio = (s_m - before.s_m) / span;
        *l_m = before.l_m + ratio * (after.l_m - before.l_m);
        return true;
    }
    return false;
}

BoundaryView makeBoundaryView(const RawBoundaryEvidence& boundary) {
    BoundaryView view;
    view.boundary = &boundary;
    view.s_begin_m = boundary.samples.front().s_m;
    view.s_end_m = boundary.samples.back().s_m;
    double total_l = 0.0;
    for (const auto& sample : boundary.samples) total_l += sample.l_m;
    view.mean_l_m = total_l / static_cast<double>(boundary.samples.size());
    return view;
}

void computeStats(RawRibbonRelation* relation, const RawRibbonGraphBuilder::Config& cfg) {
    if (!relation || relation->sample_width_m.empty()) return;

    relation->sample_count = static_cast<int>(relation->sample_width_m.size());
    relation->width_begin_m = relation->sample_width_m.front();
    relation->width_end_m = relation->sample_width_m.back();
    relation->width_min_m = *std::min_element(relation->sample_width_m.begin(),
                                              relation->sample_width_m.end());
    relation->width_max_m = *std::max_element(relation->sample_width_m.begin(),
                                              relation->sample_width_m.end());
    relation->width_median_m = median(relation->sample_width_m);

    std::vector<double> deviations;
    deviations.reserve(relation->sample_width_m.size());
    for (const double width : relation->sample_width_m) {
        deviations.push_back(std::abs(width - relation->width_median_m));
    }
    relation->width_mad_m = median(std::move(deviations));

    int stable_steps = 0;
    int directional_steps = 0;
    int nonzero_steps = 0;
    const double total_delta = relation->width_end_m - relation->width_begin_m;
    const double direction = total_delta >= 0.0 ? 1.0 : -1.0;
    for (std::size_t i = 1; i < relation->sample_width_m.size(); ++i) {
        const double local_delta = relation->sample_width_m[i] - relation->sample_width_m[i - 1];
        relation->max_local_width_delta_m =
            std::max(relation->max_local_width_delta_m, std::abs(local_delta));
        if (std::abs(local_delta) <= cfg.stable_local_width_delta_m) ++stable_steps;
        if (std::abs(local_delta) <= cfg.monotonic_width_epsilon_m) continue;
        ++nonzero_steps;
        if (local_delta * direction > 0.0) ++directional_steps;
    }
    const int step_count = std::max(1, relation->sample_count - 1);
    relation->stable_ratio = static_cast<double>(stable_steps) / static_cast<double>(step_count);
    relation->monotonic_ratio = nonzero_steps == 0
        ? 1.0
        : static_cast<double>(directional_steps) / static_cast<double>(nonzero_steps);
}

std::string classifyProfile(const RawRibbonRelation& relation,
                            const RawRibbonGraphBuilder::Config& cfg) {
    if (relation.sample_width_m.empty()) return "invalid";
    if (relation.valid_sample_count == 0) {
        if (relation.width_max_m < cfg.min_valid_width_m) return "too_narrow";
        if (relation.width_min_m > cfg.max_valid_width_m) return "too_wide";
        return "invalid";
    }
    const double delta = relation.width_end_m - relation.width_begin_m;
    if (std::abs(delta) <= cfg.stable_width_delta_m &&
        relation.stable_ratio >= cfg.min_stable_ratio) {
        return "stable";
    }
    return delta > 0.0 ? "opening" : "closing";
}

RawRibbonRelation buildRelation(const BoundaryView& right,
                                const BoundaryView& left,
                                const RawRibbonGraphBuilder::Config& cfg) {
    RawRibbonRelation relation;
    relation.right_observation_id = right.boundary->observation_id;
    relation.left_observation_id = left.boundary->observation_id;
    relation.right_debug_label = right.boundary->debug_label;
    relation.left_debug_label = left.boundary->debug_label;
    relation.relation_id = mixIds(relation.right_observation_id, relation.left_observation_id);
    relation.s_begin_m = std::max(right.s_begin_m, left.s_begin_m);
    relation.s_end_m = std::min(right.s_end_m, left.s_end_m);

    if (relation.s_end_m - relation.s_begin_m < cfg.min_overlap_m) {
        relation.profile_type = "invalid";
        addReason(&relation, "insufficient_overlap");
        return relation;
    }
    if (left.mean_l_m - right.mean_l_m > cfg.max_pair_lateral_gap_m) {
        addReason(&relation, "mean_lateral_gap_too_large");
    }

    const double step = std::max(1e-6, cfg.sample_step_m);
    for (double s_m = relation.s_begin_m; s_m <= relation.s_end_m + 1e-6; s_m += step) {
        double right_l = 0.0;
        double left_l = 0.0;
        if (!sampleAtS(*right.boundary, s_m, &right_l) ||
            !sampleAtS(*left.boundary, s_m, &left_l)) {
            continue;
        }
        const double width = left_l - right_l;
        relation.sample_s_m.push_back(s_m);
        relation.sample_right_l_m.push_back(right_l);
        relation.sample_left_l_m.push_back(left_l);
        relation.sample_width_m.push_back(width);
        if (width >= cfg.min_valid_width_m && width <= cfg.max_valid_width_m) {
            ++relation.valid_sample_count;
        }
    }

    computeStats(&relation, cfg);
    if (relation.sample_count == 0) {
        relation.profile_type = "invalid";
        addReason(&relation, "no_overlap_samples");
        return relation;
    }

    const double valid_ratio =
        static_cast<double>(relation.valid_sample_count) / static_cast<double>(relation.sample_count);
    relation.valid_length_m = valid_ratio * (relation.s_end_m - relation.s_begin_m);
    relation.profile_type = classifyProfile(relation, cfg);

    if (relation.width_min_m < cfg.min_valid_width_m) addReason(&relation, "width_too_narrow");
    if (relation.width_max_m > cfg.max_valid_width_m) addReason(&relation, "width_too_wide");
    if (valid_ratio < cfg.min_valid_width_ratio) addReason(&relation, "low_valid_width_ratio");
    if (relation.profile_type == "stable" &&
        relation.max_local_width_delta_m > cfg.stable_local_width_delta_m) {
        addReason(&relation, "unstable_local_width_delta");
    }

    relation.propagation_eligible =
        relation.rejection_reasons.empty() &&
        (relation.profile_type == "stable" ||
         relation.profile_type == "opening" ||
         relation.profile_type == "closing");
    return relation;
}

}  // namespace

RawRibbonGraphOutput RawRibbonGraphBuilder::build(
    const RawBoundaryEvidenceOutput& raw_evidence) const {
    RawRibbonGraphOutput output;
    if (!raw_evidence.ok) {
        output.error = raw_evidence.error.empty() ? "invalid_raw_boundary_evidence"
                                                 : raw_evidence.error;
        return output;
    }

    std::vector<BoundaryView> boundaries;
    boundaries.reserve(raw_evidence.boundaries.size());
    for (const auto& boundary : raw_evidence.boundaries) {
        if (boundary.samples.size() < 2) continue;
        boundaries.push_back(makeBoundaryView(boundary));
    }
    std::sort(boundaries.begin(), boundaries.end(),
              [](const auto& a, const auto& b) { return a.mean_l_m < b.mean_l_m; });

    for (std::size_t i = 0; i + 1 < boundaries.size(); ++i) {
        output.lateral_relations.push_back(
            buildRelation(boundaries[i], boundaries[i + 1], cfg_));
    }

    output.ok = true;
    if (output.lateral_relations.empty()) output.error = "no_raw_ribbon_relations";
    return output;
}

}  // namespace topology_map::topology_v3
