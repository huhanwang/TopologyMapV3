#include "topology_v3/stages/boundary_junction_graph_builder.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

namespace topology_map::topology_v3 {
namespace {

bool activeNode(const FrenetSliceGraphNode& node) {
    return node.state == "observed" || node.state == "inferred";
}

bool activeLink(const FrenetSliceGraphLonLink& link) {
    return link.active &&
           (link.kind == "observed" || link.kind == "inferred" ||
            link.kind == "near_topology");
}

BoundaryJunctionGraphSample makeSample(const FrenetSliceGraphNode& node) {
    BoundaryJunctionGraphSample sample;
    sample.node_id = node.node_id;
    sample.raw_ft_id = node.raw_ft_id;
    sample.final_ft_id = node.final_ft_id;
    sample.source_debug_label = node.debug_label;
    sample.slice_index = node.slice_index;
    sample.s_m = node.s_m;
    sample.l_m = node.l_m;
    sample.state = node.state;
    sample.provenance = node.provenance;
    sample.semantic_type = node.semantic_type;
    return sample;
}

template <typename T>
std::vector<T> sortedUnique(std::vector<T> values) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
    return values;
}

BoundaryJunctionBoundary makeBoundary(
    std::uint32_t boundary_id,
    const std::vector<std::uint64_t>& chain,
    const std::map<std::uint64_t, const FrenetSliceGraphNode*>& node_by_id,
    const std::map<std::uint64_t, int>& incoming_count,
    const std::map<std::uint64_t, int>& outgoing_count,
    const std::set<std::uint64_t>& junction_node_ids) {
    BoundaryJunctionBoundary boundary;
    boundary.boundary_id = boundary_id;
    boundary.debug_label = "B" + std::to_string(boundary_id);
    boundary.node_ids = chain;

    for (std::uint64_t node_id : chain) {
        const auto node_it = node_by_id.find(node_id);
        if (node_it == node_by_id.end()) continue;
        const auto& node = *node_it->second;
        boundary.samples.push_back(makeSample(node));
        boundary.raw_ft_ids.push_back(node.raw_ft_id);
        boundary.final_ft_ids.push_back(node.final_ft_id);
        if (!node.debug_label.empty()) boundary.source_debug_labels.push_back(node.debug_label);
        if (node.state == "observed") ++boundary.observed_sample_count;
        if (node.state == "inferred") ++boundary.inferred_sample_count;
    }

    std::sort(boundary.samples.begin(), boundary.samples.end(),
              [](const auto& a, const auto& b) {
                  if (a.s_m != b.s_m) return a.s_m < b.s_m;
                  return a.l_m < b.l_m;
              });
    if (!boundary.samples.empty()) {
        boundary.s_begin_m = boundary.samples.front().s_m;
        boundary.s_end_m = boundary.samples.back().s_m;
        for (std::size_t i = 1; i < boundary.samples.size(); ++i) {
            const auto& a = boundary.samples[i - 1];
            const auto& b = boundary.samples[i];
            boundary.length_m += std::hypot(b.s_m - a.s_m, b.l_m - a.l_m);
        }
    }
    boundary.raw_ft_ids = sortedUnique(std::move(boundary.raw_ft_ids));
    boundary.final_ft_ids = sortedUnique(std::move(boundary.final_ft_ids));
    boundary.source_debug_labels = sortedUnique(std::move(boundary.source_debug_labels));

    if (!chain.empty()) {
        const auto in_it = incoming_count.find(chain.front());
        const auto out_it = outgoing_count.find(chain.front());
        const int front_in = in_it == incoming_count.end() ? 0 : in_it->second;
        const int front_out = out_it == outgoing_count.end() ? 0 : out_it->second;
        boundary.starts_at_junction =
            junction_node_ids.count(chain.front()) != 0 || front_in > 1 || front_out > 1;
        const auto end_in_it = incoming_count.find(chain.back());
        const auto end_out_it = outgoing_count.find(chain.back());
        const int back_in = end_in_it == incoming_count.end() ? 0 : end_in_it->second;
        const int back_out = end_out_it == outgoing_count.end() ? 0 : end_out_it->second;
        boundary.ends_at_junction =
            junction_node_ids.count(chain.back()) != 0 || back_in > 1 || back_out > 1;
    }
    return boundary;
}

}  // namespace

BoundaryJunctionGraphOutput BoundaryJunctionGraphBuilder::build(
    const FrenetSliceGraphOutput& slice_graph,
    const JunctionEvidenceOutput& junction_evidence) const {
    BoundaryJunctionGraphOutput output;
    if (!slice_graph.ok) {
        output.error = slice_graph.error.empty() ? "invalid_boundary_junction_graph_input"
                                                : slice_graph.error;
        return output;
    }

    std::map<std::uint64_t, const FrenetSliceGraphNode*> node_by_id;
    for (const auto& node : slice_graph.nodes) {
        if (activeNode(node)) node_by_id[node.node_id] = &node;
    }
    std::set<std::uint64_t> junction_node_ids;
    for (const auto& candidate : junction_evidence.candidates) {
        junction_node_ids.insert(candidate.node_ids.begin(), candidate.node_ids.end());
    }

    std::map<std::uint64_t, std::vector<std::uint64_t>> outgoing;
    std::map<std::uint64_t, int> incoming_count;
    std::map<std::uint64_t, int> outgoing_count;
    std::set<std::pair<std::uint64_t, std::uint64_t>> all_links;
    for (const auto& link : slice_graph.lon_links) {
        if (!activeLink(link)) continue;
        if (node_by_id.count(link.from_node_id) == 0 ||
            node_by_id.count(link.to_node_id) == 0) {
            continue;
        }
        outgoing[link.from_node_id].push_back(link.to_node_id);
        ++incoming_count[link.to_node_id];
        ++outgoing_count[link.from_node_id];
        all_links.insert({link.from_node_id, link.to_node_id});
    }
    for (auto& [_, next_nodes] : outgoing) {
        std::sort(next_nodes.begin(), next_nodes.end(), [&](auto a, auto b) {
            const auto* na = node_by_id[a];
            const auto* nb = node_by_id[b];
            if (na->slice_index != nb->slice_index) return na->slice_index < nb->slice_index;
            if (na->l_m != nb->l_m) return na->l_m < nb->l_m;
            return a < b;
        });
    }

    std::set<std::pair<std::uint64_t, std::uint64_t>> consumed_links;
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> boundaries_by_node;
    const auto appendBoundary = [&](std::vector<std::uint64_t> chain) {
        if (chain.empty()) return;
        const auto id = static_cast<std::uint32_t>(output.boundaries.size());
        auto boundary = makeBoundary(id, chain, node_by_id, incoming_count, outgoing_count,
                                     junction_node_ids);
        for (std::uint64_t node_id : boundary.node_ids) {
            boundaries_by_node[node_id].push_back(id);
        }
        output.boundaries.push_back(std::move(boundary));
    };

    for (const auto& [node_id, node] : node_by_id) {
        const int in = incoming_count[node_id];
        const int out = outgoing_count[node_id];
        if (in == 1 && out == 1) continue;
        const auto out_it = outgoing.find(node_id);
        if (out_it == outgoing.end()) continue;
        for (std::uint64_t first_to : out_it->second) {
            if (!consumed_links.insert({node_id, first_to}).second) continue;
            std::vector<std::uint64_t> chain = {node_id, first_to};
            std::uint64_t current = first_to;
            while (incoming_count[current] == 1 && outgoing_count[current] == 1) {
                const auto next_it = outgoing.find(current);
                if (next_it == outgoing.end() || next_it->second.empty()) break;
                const std::uint64_t next = next_it->second.front();
                if (!consumed_links.insert({current, next}).second) break;
                chain.push_back(next);
                current = next;
            }
            appendBoundary(std::move(chain));
        }
    }

    for (const auto& link : all_links) {
        if (consumed_links.count(link) != 0) continue;
        appendBoundary({link.first, link.second});
    }
    for (const auto& [node_id, _] : node_by_id) {
        if (boundaries_by_node.count(node_id) == 0) appendBoundary({node_id});
    }

    std::uint64_t relation_id = 1;
    for (const auto& candidate : junction_evidence.candidates) {
        BoundaryJunctionRelation relation;
        relation.relation_id = relation_id++;
        relation.junction_candidate_id = candidate.candidate_id;
        relation.type = candidate.type;
        relation.s_m = candidate.s_m;
        relation.l_m = candidate.l_m;
        relation.node_ids = candidate.node_ids;
        relation.final_ft_ids = candidate.final_ft_ids;
        relation.confidence = candidate.confidence;
        relation.evidence = candidate.evidence;

        const auto addBoundaryAtNode = [&](std::uint64_t node_id, bool incoming) {
            const auto it = boundaries_by_node.find(node_id);
            if (it == boundaries_by_node.end()) return;
            for (std::uint32_t boundary_id : it->second) {
                const auto& boundary = output.boundaries[boundary_id];
                if (boundary.node_ids.empty()) continue;
                const bool at_front = boundary.node_ids.front() == node_id;
                const bool at_back = boundary.node_ids.back() == node_id;
                if (incoming && at_back) {
                    relation.incoming_boundary_ids.push_back(boundary_id);
                }
                if (!incoming && at_front) {
                    relation.outgoing_boundary_ids.push_back(boundary_id);
                }
            }
        };
        for (std::uint64_t node_id : candidate.incoming_node_ids) {
            addBoundaryAtNode(node_id, true);
        }
        for (std::uint64_t node_id : candidate.outgoing_node_ids) {
            addBoundaryAtNode(node_id, false);
        }
        for (std::uint64_t node_id : candidate.node_ids) {
            const auto it = boundaries_by_node.find(node_id);
            if (it == boundaries_by_node.end()) continue;
            for (std::uint32_t boundary_id : it->second) {
                const auto& boundary = output.boundaries[boundary_id];
                if (boundary.node_ids.empty()) continue;
                if (boundary.node_ids.back() == node_id &&
                    boundary.s_end_m <= candidate.s_m + 1.0) {
                    relation.incoming_boundary_ids.push_back(boundary_id);
                }
                if (boundary.node_ids.front() == node_id &&
                    boundary.s_begin_m >= candidate.s_m - 1.0) {
                    relation.outgoing_boundary_ids.push_back(boundary_id);
                }
            }
        }
        relation.incoming_boundary_ids = sortedUnique(std::move(relation.incoming_boundary_ids));
        relation.outgoing_boundary_ids = sortedUnique(std::move(relation.outgoing_boundary_ids));
        output.junctions.push_back(std::move(relation));
    }

    output.ok = true;
    if (output.boundaries.empty()) output.error = "empty_boundary_junction_graph";
    return output;
}

}  // namespace topology_map::topology_v3
