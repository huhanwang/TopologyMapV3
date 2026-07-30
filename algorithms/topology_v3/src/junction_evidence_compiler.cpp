#include "topology_v3/stages/junction_evidence_compiler.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>

namespace topology_map::topology_v3 {
namespace {

bool activeNode(const FrenetSliceGraphNode& node) {
    return node.state == "observed" || node.state == "inferred";
}

std::vector<std::uint64_t> sortedIds(std::set<std::uint64_t> ids) {
    return {ids.begin(), ids.end()};
}

std::string classifyJunction(std::size_t incoming_count, std::size_t outgoing_count) {
    if (incoming_count == 1 && outgoing_count > 1) return "split";
    if (incoming_count > 1 && outgoing_count == 1) return "merge";
    if (incoming_count == 1 && outgoing_count == 1) return "near_contact";
    return "complex";
}

}  // namespace

JunctionEvidenceOutput JunctionEvidenceCompiler::build(
    const FrenetSliceGraphOutput& slice_graph) const {
    JunctionEvidenceOutput output;
    if (!slice_graph.ok) {
        output.error = slice_graph.error.empty() ? "invalid_slice_graph" : slice_graph.error;
        return output;
    }

    std::map<std::uint64_t, const FrenetSliceGraphNode*> node_by_id;
    for (const auto& node : slice_graph.nodes) {
        if (activeNode(node)) node_by_id[node.node_id] = &node;
    }

    std::vector<const FrenetSliceGraphLonLink*> near_links;
    for (const auto& link : slice_graph.lon_links) {
        if (link.active && link.kind == "near_topology") near_links.push_back(&link);
    }

    std::vector<std::set<std::size_t>> components;
    std::set<std::size_t> visited;
    for (std::size_t seed = 0; seed < near_links.size(); ++seed) {
        if (!visited.insert(seed).second) continue;
        std::set<std::size_t> component;
        std::queue<std::size_t> pending;
        pending.push(seed);
        while (!pending.empty()) {
            const std::size_t index = pending.front();
            pending.pop();
            component.insert(index);
            const auto* relation = near_links[index];
            const auto* from = node_by_id[relation->from_node_id];
            const auto* to = node_by_id[relation->to_node_id];
            if (!from || !to) continue;
            const int relation_min_slice = std::min(from->slice_index, to->slice_index);
            const int relation_max_slice = std::max(from->slice_index, to->slice_index);
            const std::set<std::uint64_t> relation_final_ids = {
                from->final_ft_id,
                to->final_ft_id};
            for (std::size_t other_index = 0; other_index < near_links.size(); ++other_index) {
                if (visited.count(other_index) != 0) continue;
                const auto* other = near_links[other_index];
                const auto* other_from = node_by_id[other->from_node_id];
                const auto* other_to = node_by_id[other->to_node_id];
                if (!other_from || !other_to) continue;
                const int other_min_slice = std::min(other_from->slice_index, other_to->slice_index);
                const int other_max_slice = std::max(other_from->slice_index, other_to->slice_index);
                const bool local = other_min_slice <= relation_max_slice + 1 &&
                                   relation_min_slice <= other_max_slice + 1;
                const bool shares_final_ft =
                    relation_final_ids.count(other_from->final_ft_id) != 0 ||
                    relation_final_ids.count(other_to->final_ft_id) != 0;
                if (local && shares_final_ft && visited.insert(other_index).second) {
                    pending.push(other_index);
                }
            }
        }
        components.push_back(std::move(component));
    }

    std::uint64_t next_candidate_id = 1;
    for (const auto& component : components) {
        std::set<std::uint64_t> node_ids;
        std::set<std::uint64_t> link_ids;
        std::set<std::uint64_t> final_ft_ids;
        std::map<std::uint64_t, std::uint64_t> incoming_node_by_ft;
        std::map<std::uint64_t, std::uint64_t> outgoing_node_by_ft;
        std::map<int, std::vector<double>> lateral_by_slice;
        std::vector<std::string> evidence;

        for (const std::size_t index : component) {
            const auto* link = near_links[index];
            const auto* from = node_by_id[link->from_node_id];
            const auto* to = node_by_id[link->to_node_id];
            if (!from || !to) continue;
            node_ids.insert(from->node_id);
            node_ids.insert(to->node_id);
            link_ids.insert(link->link_id);
            final_ft_ids.insert(from->final_ft_id);
            final_ft_ids.insert(to->final_ft_id);
            lateral_by_slice[from->slice_index].push_back(from->l_m);
            lateral_by_slice[to->slice_index].push_back(to->l_m);
            evidence.push_back(link->reason);

            const bool road_forward = from->slice_index <= to->slice_index;
            const auto* incoming = road_forward ? from : to;
            const auto* outgoing = road_forward ? to : from;
            incoming_node_by_ft[incoming->final_ft_id] = incoming->node_id;
            outgoing_node_by_ft[outgoing->final_ft_id] = outgoing->node_id;
        }

        if (node_ids.size() < 2) continue;

        int best_slice = -1;
        double best_spread = 1e100;
        double best_mean_l = 0.0;
        for (const auto& [slice_index, laterals] : lateral_by_slice) {
            if (laterals.size() < 2) continue;
            const auto [min_it, max_it] = std::minmax_element(laterals.begin(), laterals.end());
            const double spread = *max_it - *min_it;
            if (spread < best_spread) {
                best_spread = spread;
                best_slice = slice_index;
                best_mean_l = 0.0;
                for (double l : laterals) best_mean_l += l;
                best_mean_l /= static_cast<double>(laterals.size());
            }
        }
        if (best_slice < 0) {
            for (std::uint64_t node_id : node_ids) {
                const auto* node = node_by_id[node_id];
                if (!node) continue;
                lateral_by_slice[node->slice_index].push_back(node->l_m);
            }
            if (!lateral_by_slice.empty()) {
                best_slice = lateral_by_slice.begin()->first;
                best_mean_l = lateral_by_slice.begin()->second.front();
            }
        }

        double s_m = 0.0;
        int s_count = 0;
        for (std::uint64_t node_id : node_ids) {
            const auto* node = node_by_id[node_id];
            if (!node || node->slice_index != best_slice) continue;
            s_m += node->s_m;
            ++s_count;
        }
        if (s_count > 0) s_m /= static_cast<double>(s_count);
        else {
            for (std::uint64_t node_id : node_ids) {
                const auto* node = node_by_id[node_id];
                if (!node) continue;
                s_m = node->s_m;
                break;
            }
        }

        std::set<std::uint64_t> incoming_ids;
        std::set<std::uint64_t> outgoing_ids;
        for (const auto& [_, node_id] : incoming_node_by_ft) incoming_ids.insert(node_id);
        for (const auto& [_, node_id] : outgoing_node_by_ft) outgoing_ids.insert(node_id);

        JunctionCandidate candidate;
        candidate.candidate_id = next_candidate_id++;
        candidate.type = classifyJunction(incoming_ids.size(), outgoing_ids.size());
        candidate.slice_index = best_slice;
        candidate.s_m = s_m;
        candidate.l_m = best_mean_l;
        candidate.incoming_node_ids = sortedIds(std::move(incoming_ids));
        candidate.outgoing_node_ids = sortedIds(std::move(outgoing_ids));
        candidate.node_ids = sortedIds(std::move(node_ids));
        candidate.lon_link_ids = sortedIds(std::move(link_ids));
        candidate.final_ft_ids = sortedIds(std::move(final_ft_ids));
        candidate.confidence = std::clamp(1.0 - best_spread / cfg_.max_contact_distance_m,
                                          cfg_.min_candidate_confidence,
                                          1.0);
        std::sort(evidence.begin(), evidence.end());
        evidence.erase(std::unique(evidence.begin(), evidence.end()), evidence.end());
        candidate.evidence = std::move(evidence);
        output.candidates.push_back(std::move(candidate));
    }

    output.ok = true;
    return output;
}

}  // namespace topology_map::topology_v3
