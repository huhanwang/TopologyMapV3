#include "topology_v3/stages/frenet_observation/raw_ft_association_builder.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <vector>

namespace topology_map::topology_v3 {
namespace {

struct DecisionInfo {
    std::string state;
    bool passive_boundary = false;
};

struct TrackSamples {
    std::uint64_t raw_ft_id = 0;
    std::string debug_label;
    std::string semantic_type;
    std::map<int, FrenetSliceIntersectionNode> by_slice;
};

struct EndpointEvidence {
    std::uint64_t anchor_raw_ft_id = 0;
    bool anchor_is_left = false;
    double width_m = 0.0;
};

bool laneLine(const std::string& semantic_type) {
    return semantic_type == "lane_line";
}

std::map<std::uint64_t, DecisionInfo> buildDecisionMap(const RawFtFilterOutput& filter) {
    std::map<std::uint64_t, DecisionInfo> result;
    for (const auto& decision : filter.decisions) {
        result[decision.raw_ft_id] = {decision.state, decision.passive_boundary};
    }
    return result;
}

bool usableAssociationTrack(std::uint64_t raw_ft_id,
                            const std::map<std::uint64_t, DecisionInfo>& decisions) {
    const auto it = decisions.find(raw_ft_id);
    return it != decisions.end() && it->second.state == "kept" && !it->second.passive_boundary;
}

std::map<std::uint64_t, TrackSamples> buildTracks(
    const FrenetSliceIntersectionOutput& intersections,
    const std::map<std::uint64_t, DecisionInfo>& decisions) {
    std::map<std::uint64_t, TrackSamples> tracks;
    for (const auto& slice : intersections.slices) {
        for (const auto& node : slice.nodes) {
            if (!usableAssociationTrack(node.raw_ft_id, decisions)) continue;
            if (!laneLine(node.semantic_type)) continue;
            auto& track = tracks[node.raw_ft_id];
            track.raw_ft_id = node.raw_ft_id;
            track.debug_label = node.debug_label;
            track.semantic_type = node.semantic_type;
            track.by_slice[node.slice_index] = node;
        }
    }
    return tracks;
}

std::vector<const FrenetSliceIntersectionNode*> orderedUsableNodesAtSlice(
    const FrenetSliceIntersection& slice,
    const std::map<std::uint64_t, DecisionInfo>& decisions) {
    std::vector<const FrenetSliceIntersectionNode*> result;
    for (const auto& node : slice.nodes) {
        if (!usableAssociationTrack(node.raw_ft_id, decisions)) continue;
        if (!laneLine(node.semantic_type)) continue;
        result.push_back(&node);
    }
    std::sort(result.begin(), result.end(), [](const auto* a, const auto* b) {
        if (a->l_m != b->l_m) return a->l_m < b->l_m;
        return a->raw_ft_id < b->raw_ft_id;
    });
    return result;
}

std::vector<EndpointEvidence> endpointEvidence(
    const TrackSamples& track,
    bool tail,
    const FrenetSliceIntersectionOutput& intersections,
    const std::map<std::uint64_t, DecisionInfo>& decisions) {
    std::vector<EndpointEvidence> result;
    if (track.by_slice.empty()) return result;
    const int slice_index = tail ? track.by_slice.rbegin()->first : track.by_slice.begin()->first;
    if (slice_index < 0 || slice_index >= static_cast<int>(intersections.slices.size())) {
        return result;
    }
    const auto& endpoint = tail ? track.by_slice.rbegin()->second : track.by_slice.begin()->second;
    const auto ordered = orderedUsableNodesAtSlice(intersections.slices[static_cast<std::size_t>(slice_index)],
                                                  decisions);
    for (std::size_t i = 0; i < ordered.size(); ++i) {
        if (ordered[i]->raw_ft_id != track.raw_ft_id) continue;
        if (i > 0) {
            const auto* right = ordered[i - 1];
            result.push_back({right->raw_ft_id, false, std::abs(endpoint.l_m - right->l_m)});
        }
        if (i + 1 < ordered.size()) {
            const auto* left = ordered[i + 1];
            result.push_back({left->raw_ft_id, true, std::abs(left->l_m - endpoint.l_m)});
        }
        break;
    }
    return result;
}

bool hasEndpointEvidence(const TrackSamples& track,
                         std::uint64_t anchor_raw_ft_id,
                         bool anchor_is_left,
                         const FrenetSliceIntersectionOutput& intersections,
                         const std::map<std::uint64_t, DecisionInfo>& decisions) {
    for (const bool tail : {false, true}) {
        for (const auto& evidence : endpointEvidence(track, tail, intersections, decisions)) {
            if (evidence.anchor_raw_ft_id == anchor_raw_ft_id &&
                evidence.anchor_is_left == anchor_is_left) {
                return true;
            }
        }
    }
    return false;
}

}  // namespace

RawFtAssociationOutput RawFtAssociationBuilder::build(
    const FrenetSliceIntersectionOutput& intersections,
    const RawFtFilterOutput& filter) const {
    RawFtAssociationOutput output;
    if (!intersections.ok || !filter.ok) {
        output.error = !intersections.ok ? intersections.error : filter.error;
        if (output.error.empty()) output.error = "invalid_raw_ft_association_input";
        return output;
    }

    const auto decisions = buildDecisionMap(filter);
    const auto tracks = buildTracks(intersections, decisions);
    std::vector<RawFtAssociationCandidate> candidates;

    for (const auto& [from_id, from] : tracks) {
        if (from.by_slice.empty()) continue;
        const auto& tail = from.by_slice.rbegin()->second;
        const auto tail_evidence = endpointEvidence(from, true, intersections, decisions);
        if (tail_evidence.empty()) continue;
        for (const auto& [to_id, to] : tracks) {
            if (from_id == to_id || to.by_slice.empty()) continue;
            const auto& head = to.by_slice.begin()->second;
            const double gap = head.s_m - tail.s_m;
            if (gap < -cfg_.max_association_near_gap_m || gap > cfg_.max_association_gap_m) {
                continue;
            }
            const double endpoint_delta = std::abs(head.l_m - tail.l_m);
            if (endpoint_delta > cfg_.max_endpoint_l_delta_m) continue;
            const auto head_evidence = endpointEvidence(to, false, intersections, decisions);
            for (const auto& from_evidence : tail_evidence) {
                for (const auto& to_evidence : head_evidence) {
                    if (from_evidence.anchor_raw_ft_id != to_evidence.anchor_raw_ft_id ||
                        from_evidence.anchor_is_left != to_evidence.anchor_is_left) {
                        continue;
                    }
                    const double width_delta =
                        std::abs(from_evidence.width_m - to_evidence.width_m);
                    if (width_delta > cfg_.max_width_delta_m) continue;
                    RawFtAssociationCandidate candidate;
                    candidate.from_raw_ft_id = from_id;
                    candidate.to_raw_ft_id = to_id;
                    candidate.shared_anchor_raw_ft_id = from_evidence.anchor_raw_ft_id;
                    candidate.anchor_is_left = from_evidence.anchor_is_left;
                    candidate.gap_m = gap;
                    candidate.endpoint_l_delta_m = endpoint_delta;
                    candidate.width_delta_m = width_delta;
                    candidate.score = std::max(0.0, gap) + 2.0 * endpoint_delta +
                                      3.0 * width_delta;
                    candidate.classification =
                        std::abs(gap) <= cfg_.max_association_near_gap_m
                            ? "near_observation"
                            : "continuation_candidate";
                    candidate.reasons.push_back("shared_endpoint_ribbon_anchor");
                    candidates.push_back(std::move(candidate));
                }
            }
        }
    }

    for (auto& candidate : candidates) {
        if (candidate.classification != "continuation_candidate") continue;
        const auto from_it = tracks.find(candidate.from_raw_ft_id);
        const auto to_it = tracks.find(candidate.to_raw_ft_id);
        if (from_it == tracks.end() || to_it == tracks.end()) continue;
        const auto& from_tail = from_it->second.by_slice.rbegin()->second;
        const auto& to_head = to_it->second.by_slice.begin()->second;
        for (const auto& [intermediate_id, intermediate] : tracks) {
            if (intermediate_id == candidate.from_raw_ft_id ||
                intermediate_id == candidate.to_raw_ft_id ||
                intermediate.by_slice.empty()) {
                continue;
            }
            if (!hasEndpointEvidence(intermediate, candidate.shared_anchor_raw_ft_id,
                                     candidate.anchor_is_left, intersections, decisions)) {
                continue;
            }
            const double start_s = intermediate.by_slice.begin()->second.s_m;
            const double end_s = intermediate.by_slice.rbegin()->second.s_m;
            if (end_s <= from_tail.s_m + cfg_.max_association_near_gap_m ||
                start_s >= to_head.s_m - cfg_.max_association_near_gap_m) {
                continue;
            }
            candidate.classification = "blocked_by_intermediate_fragment";
            candidate.reasons.push_back("blocked_by_intermediate_fragment");
            break;
        }
    }

    std::map<std::uint64_t, int> outgoing_count;
    std::map<std::uint64_t, int> incoming_count;
    for (const auto& candidate : candidates) {
        if (candidate.classification != "continuation_candidate") continue;
        ++outgoing_count[candidate.from_raw_ft_id];
        ++incoming_count[candidate.to_raw_ft_id];
    }
    for (auto& candidate : candidates) {
        if (candidate.classification != "continuation_candidate") continue;
        if (outgoing_count[candidate.from_raw_ft_id] == 1 &&
            incoming_count[candidate.to_raw_ft_id] == 1) {
            candidate.classification = "ready_continuation";
            candidate.reasons.push_back("unique_incoming_outgoing");
            ++output.ready_continuation_count;
        } else {
            candidate.classification = "ambiguous_branch";
            candidate.reasons.push_back("ambiguous_incoming_outgoing");
            ++output.ambiguous_count;
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        if (a.classification != b.classification) return a.classification < b.classification;
        return a.score < b.score;
    });
    output.candidates = std::move(candidates);
    output.ok = true;
    return output;
}

}  // namespace topology_map::topology_v3
