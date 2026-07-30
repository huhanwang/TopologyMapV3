#include "topology_v3/stages/frenet_observation/frenet_slice_graph_builder.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace topology_map::topology_v3 {
namespace {

struct DecisionInfo {
    std::string state;
    std::string reason;
    bool direct_topology_candidate = false;
    bool passive_boundary = false;
    int sample_count = 0;
    double support_length_m = 0.0;
};

bool laneLine(const std::string& semantic_type) {
    return semantic_type == "lane_line";
}

bool passiveBoundary(const std::string& semantic_type) {
    return semantic_type == "curb" || semantic_type == "road_edge";
}

bool activeNodeState(const FrenetSliceGraphNode& node) {
    return node.state == "observed" || node.state == "inferred";
}

bool supportedLonKind(const std::string& kind) {
    return kind == "observed" || kind == "inferred";
}

bool validLaneWidth(double width_m, const FrenetSliceGraphBuilder::Config& cfg) {
    return std::isfinite(width_m) &&
           width_m >= cfg.min_stable_ribbon_width_m &&
           width_m <= cfg.max_stable_ribbon_width_m;
}

bool validPassiveRepairWidth(double width_m, const FrenetSliceGraphBuilder::Config& cfg) {
    return std::isfinite(width_m) &&
           width_m >= cfg.min_passive_repair_width_m &&
           width_m <= cfg.max_stable_ribbon_width_m;
}

bool supportAllowedForNode(const FrenetSliceGraphNode& node,
                           double width_m,
                           const FrenetSliceGraphBuilder::Config& cfg) {
    const double abs_width = std::abs(width_m);
    if (laneLine(node.semantic_type)) {
        return validLaneWidth(abs_width, cfg) || abs_width <= cfg.max_corridor_support_width_m;
    }
    if (passiveBoundary(node.semantic_type)) {
        return validPassiveRepairWidth(abs_width, cfg);
    }
    return false;
}

bool repairableBoundary(const FrenetSliceGraphNode& node,
                        const std::map<std::uint64_t, DecisionInfo>& decisions,
                        const FrenetSliceGraphBuilder::Config& cfg) {
    const auto it = decisions.find(node.raw_ft_id);
    if (it == decisions.end()) return false;
    if (laneLine(node.semantic_type)) return it->second.state == "kept";
    if (!passiveBoundary(node.semantic_type)) return false;
    return it->second.passive_boundary &&
           it->second.support_length_m >= cfg.min_passive_support_length_m;
}

std::map<std::uint64_t, DecisionInfo> buildDecisionMap(const RawFtFilterOutput& filter) {
    std::map<std::uint64_t, DecisionInfo> result;
    for (const auto& decision : filter.decisions) {
        result[decision.raw_ft_id] = {
            decision.state,
            decision.reason,
            decision.direct_topology_candidate,
            decision.passive_boundary,
            decision.sample_count,
            decision.support_length_m};
    }
    return result;
}

std::map<std::uint64_t, std::map<int, FrenetSliceIntersectionNode>> buildKeptNodes(
    const FrenetSliceIntersectionOutput& intersections,
    const std::map<std::uint64_t, DecisionInfo>& decisions) {
    std::map<std::uint64_t, std::map<int, FrenetSliceIntersectionNode>> result;
    for (const auto& slice : intersections.slices) {
        for (const auto& node : slice.nodes) {
            const auto decision_it = decisions.find(node.raw_ft_id);
            if (decision_it == decisions.end()) continue;
            const auto& decision = decision_it->second;
            const bool keep = decision.state == "kept" || decision.passive_boundary;
            if (!keep) continue;
            result[node.raw_ft_id][node.slice_index] = node;
        }
    }
    return result;
}

std::map<std::uint64_t, std::pair<int, int>> buildRawFtSliceRanges(
    const std::map<std::uint64_t, std::map<int, FrenetSliceIntersectionNode>>& kept_nodes) {
    std::map<std::uint64_t, std::pair<int, int>> result;
    for (const auto& [raw_ft_id, by_slice] : kept_nodes) {
        if (by_slice.empty()) continue;
        result[raw_ft_id] = {by_slice.begin()->first, by_slice.rbegin()->first};
    }
    return result;
}

std::map<std::uint64_t, std::uint64_t> buildFinalFtMap(
    const std::map<std::uint64_t, std::map<int, FrenetSliceIntersectionNode>>& kept_nodes,
    const RawFtAssociationOutput& associations) {
    std::map<std::uint64_t, std::uint64_t> parent;
    for (const auto& [raw_ft_id, _] : kept_nodes) parent[raw_ft_id] = raw_ft_id;
    const auto find_root = [&](const auto& self, std::uint64_t id) -> std::uint64_t {
        const auto it = parent.find(id);
        if (it == parent.end() || it->second == id) return id;
        return self(self, it->second);
    };
    const auto unite = [&](std::uint64_t first, std::uint64_t second) {
        if (parent.count(first) == 0 || parent.count(second) == 0) return;
        const std::uint64_t first_root = find_root(find_root, first);
        const std::uint64_t second_root = find_root(find_root, second);
        if (first_root == second_root) return;
        if (first_root < second_root) parent[second_root] = first_root;
        else parent[first_root] = second_root;
    };
    for (const auto& candidate : associations.candidates) {
        if (candidate.classification == "ready_continuation") {
            unite(candidate.from_raw_ft_id, candidate.to_raw_ft_id);
        }
    }
    std::map<std::uint64_t, std::uint64_t> result;
    for (const auto& [raw_ft_id, _] : kept_nodes) {
        result[raw_ft_id] = find_root(find_root, raw_ft_id);
    }
    return result;
}

class GraphWork {
public:
    explicit GraphWork(FrenetSliceGraphOutput* output) : output_(output) {}

    std::uint64_t addObservedNode(const FrenetSliceIntersectionNode& source,
                                  std::uint64_t final_ft_id) {
        FrenetSliceGraphNode node;
        node.node_id = next_node_id_++;
        node.raw_ft_id = source.raw_ft_id;
        node.final_ft_id = final_ft_id;
        node.debug_label = source.debug_label;
        node.slice_index = source.slice_index;
        node.s_m = source.s_m;
        node.l_m = source.l_m;
        node.state = "observed";
        node.provenance = "raw_boundary";
        node.reason = "raw_ft_filter_retained";
        node.semantic_type = source.semantic_type;
        output_->nodes.push_back(node);
        node_by_ft_slice_[{node.final_ft_id, node.slice_index}] = node.node_id;
        ++output_->observed_node_count;
        return node.node_id;
    }

    std::uint64_t addAssociationConnectorNode(const FrenetSliceGraphNode& from,
                                              const FrenetSliceGraphNode& to,
                                              int target_slice,
                                              double target_s,
                                              double target_l) {
        FrenetSliceGraphNode node;
        node.node_id = next_node_id_++;
        node.raw_ft_id = from.raw_ft_id;
        node.final_ft_id = from.final_ft_id;
        node.debug_label = from.debug_label;
        node.slice_index = target_slice;
        node.s_m = target_s;
        node.l_m = target_l;
        node.state = "inferred";
        node.provenance = "raw_ft_association";
        node.reason = "raw_ft_association_connector";
        node.semantic_type = from.semantic_type.empty() ? to.semantic_type : from.semantic_type;
        output_->nodes.push_back(node);
        node_by_ft_slice_[{node.final_ft_id, node.slice_index}] = node.node_id;
        ++output_->inferred_node_count;
        return node.node_id;
    }

    std::uint64_t addInferredNode(const FrenetSliceGraphNode& endpoint,
                                  int target_slice,
                                  double target_s,
                                  double target_l,
                                  const FrenetSliceGraphNode& support_next,
                                  double signed_width_m,
                                  const std::string& reason) {
        FrenetSliceGraphNode node;
        node.node_id = next_node_id_++;
        node.raw_ft_id = endpoint.raw_ft_id;
        node.final_ft_id = endpoint.final_ft_id;
        node.debug_label = endpoint.debug_label;
        node.slice_index = target_slice;
        node.s_m = target_s;
        node.l_m = target_l;
        node.state = "inferred";
        node.provenance = "ribbon_reconstruction";
        node.reason = reason;
        node.semantic_type = endpoint.semantic_type;
        node.reconstruction_support_node_id = support_next.node_id;
        node.reconstruction_support_is_left = support_next.l_m > target_l;
        node.reconstruction_width_m = signed_width_m;
        output_->nodes.push_back(node);
        node_by_ft_slice_[{node.final_ft_id, node.slice_index}] = node.node_id;
        ++output_->inferred_node_count;
        return node.node_id;
    }

    void addLonLink(std::uint64_t from,
                    std::uint64_t to,
                    const std::string& kind,
                    double score,
                    const std::string& reason) {
        if (from == to || hasLonLink(from, to)) return;
        FrenetSliceGraphLonLink link;
        link.link_id = next_link_id_++;
        link.from_node_id = from;
        link.to_node_id = to;
        link.kind = kind;
        link.score = score;
        link.reason = reason;
        output_->lon_links.push_back(link);
        if (kind == "observed") ++output_->observed_lon_link_count;
        else if (kind == "inferred") ++output_->inferred_lon_link_count;
        else if (kind == "near_topology") ++output_->near_topology_link_count;
    }

    void rebuildLateralLinks() {
        output_->lat_links.clear();
        output_->slice_ribbons.clear();
        std::map<int, std::vector<std::uint64_t>> by_slice;
        for (const auto& node : output_->nodes) {
            if (activeNodeState(node)) by_slice[node.slice_index].push_back(node.node_id);
        }
        std::uint64_t lat_id = 0;
        std::uint64_t ribbon_id = 0;
        for (auto& [slice_index, ids] : by_slice) {
            std::sort(ids.begin(), ids.end(), [&](std::uint64_t a, std::uint64_t b) {
                return node(a)->l_m < node(b)->l_m;
            });
            for (std::size_t i = 1; i < ids.size(); ++i) {
                const auto* right = node(ids[i - 1]);
                const auto* left = node(ids[i]);
                if (!right || !left) continue;
                const double width = left->l_m - right->l_m;
                FrenetSliceGraphLatLink lat;
                lat.link_id = lat_id++;
                lat.right_node_id = right->node_id;
                lat.left_node_id = left->node_id;
                lat.slice_index = slice_index;
                lat.s_m = right->s_m;
                lat.width_m = width;
                lat.reason = "adjacent_active_nodes";
                output_->lat_links.push_back(lat);
                output_->slice_ribbons.push_back({
                    ribbon_id++,
                    slice_index,
                    right->node_id,
                    left->node_id,
                    right->s_m,
                    width,
                    0.5 * (right->l_m + left->l_m)});
            }
        }
    }

    const FrenetSliceGraphNode* node(std::uint64_t node_id) const {
        if (node_id == 0 || node_id > output_->nodes.size()) return nullptr;
        const auto& node_ref = output_->nodes[static_cast<std::size_t>(node_id - 1)];
        return node_ref.node_id == node_id ? &node_ref : nullptr;
    }

    const FrenetSliceGraphNode* nodeByFtSlice(std::uint64_t final_ft_id, int slice_index) const {
        const auto it = node_by_ft_slice_.find({final_ft_id, slice_index});
        return it == node_by_ft_slice_.end() ? nullptr : node(it->second);
    }

    bool hasLonLink(std::uint64_t from, std::uint64_t to) const {
        return std::any_of(output_->lon_links.begin(), output_->lon_links.end(), [&](const auto& link) {
            return link.active && link.from_node_id == from && link.to_node_id == to;
        });
    }

    bool hasDirectionalLonLink(std::uint64_t node_id, bool forward) const {
        return std::any_of(output_->lon_links.begin(), output_->lon_links.end(), [&](const auto& link) {
            return link.active && supportedLonKind(link.kind) &&
                   (forward ? link.from_node_id == node_id : link.to_node_id == node_id);
        });
    }

    const FrenetSliceGraphNode* linkedNeighbor(std::uint64_t node_id, bool forward) const {
        const FrenetSliceGraphNode* best = nullptr;
        for (const auto& link : output_->lon_links) {
            if (!link.active || !supportedLonKind(link.kind) ||
                !(forward ? link.from_node_id == node_id : link.to_node_id == node_id)) {
                continue;
            }
            const auto* candidate = node(forward ? link.to_node_id : link.from_node_id);
            if (!candidate) continue;
            if (!best || (forward ? candidate->s_m < best->s_m : candidate->s_m > best->s_m)) {
                best = candidate;
            }
        }
        return best;
    }

    const FrenetSliceGraphNode* sameFinalFtNext(
        const FrenetSliceGraphNode& node,
        const FrenetSliceGraphNode& current_support,
        bool forward) const {
        return nodeByFtSlice(current_support.final_ft_id,
                             node.slice_index + (forward ? 1 : -1));
    }

    bool hasActiveLatLink(std::uint64_t first_node_id, std::uint64_t second_node_id) const {
        return std::any_of(output_->lat_links.begin(), output_->lat_links.end(), [&](const auto& link) {
            return link.active &&
                   ((link.right_node_id == first_node_id && link.left_node_id == second_node_id) ||
                    (link.left_node_id == first_node_id && link.right_node_id == second_node_id));
        });
    }

    std::vector<std::uint64_t> adjacentNodeIds(std::uint64_t node_id) const {
        std::vector<std::uint64_t> result;
        for (const auto& link : output_->lat_links) {
            if (!link.active) continue;
            if (link.right_node_id == node_id) result.push_back(link.left_node_id);
            else if (link.left_node_id == node_id) result.push_back(link.right_node_id);
        }
        return result;
    }

    std::vector<std::uint64_t> activeNodesAtSlice(int slice_index) const {
        std::vector<std::uint64_t> result;
        for (const auto& node_ref : output_->nodes) {
            if (node_ref.slice_index == slice_index && activeNodeState(node_ref)) {
                result.push_back(node_ref.node_id);
            }
        }
        std::sort(result.begin(), result.end(), [&](std::uint64_t a, std::uint64_t b) {
            return node(a)->l_m < node(b)->l_m;
        });
        return result;
    }

private:
    FrenetSliceGraphOutput* output_ = nullptr;
    std::uint64_t next_node_id_ = 1;
    std::uint64_t next_link_id_ = 1;
    std::map<std::pair<std::uint64_t, int>, std::uint64_t> node_by_ft_slice_;
};

std::optional<double> predictBySelfTrend(const GraphWork& graph,
                                         const FrenetSliceGraphNode& node,
                                         bool forward,
                                         double target_s_m) {
    const auto* opposite = graph.linkedNeighbor(node.node_id, !forward);
    if (!opposite) return std::nullopt;
    const double ds = node.s_m - opposite->s_m;
    if (std::abs(ds) < 1e-6) return std::nullopt;
    const double slope = (node.l_m - opposite->l_m) / ds;
    const double predicted_l = node.l_m + slope * (target_s_m - node.s_m);
    return std::isfinite(predicted_l) ? std::optional<double>(predicted_l) : std::nullopt;
}

struct Support {
    const FrenetSliceGraphNode* current = nullptr;
    const FrenetSliceGraphNode* next = nullptr;
    double width_m = 0.0;
    bool support_is_left = false;
    bool junction_probe = false;
};

bool passiveAdjacentLaneLineClear(const GraphWork& graph,
                                  const FrenetSliceGraphNode& node,
                                  const FrenetSliceGraphBuilder::Config& cfg) {
    if (!passiveBoundary(node.semantic_type)) return true;
    for (std::uint64_t adjacent_id : graph.adjacentNodeIds(node.node_id)) {
        const auto* adjacent = graph.node(adjacent_id);
        if (!adjacent || !laneLine(adjacent->semantic_type)) continue;
        if (std::abs(adjacent->l_m - node.l_m) < cfg.min_passive_repair_width_m) {
            return false;
        }
    }
    return true;
}

std::vector<Support> adjacentSupports(const GraphWork& graph,
                                      const FrenetSliceGraphNode& node,
                                      bool forward,
                                      const FrenetSliceGraphBuilder::Config& cfg) {
    std::vector<Support> result;
    if (!passiveAdjacentLaneLineClear(graph, node, cfg)) return result;
    if (node.reconstruction_support_node_id != 0) {
        const auto* inherited = graph.node(node.reconstruction_support_node_id);
        if (inherited && inherited->slice_index == node.slice_index &&
            graph.hasActiveLatLink(node.node_id, inherited->node_id)) {
            const auto* inherited_next = graph.sameFinalFtNext(node, *inherited, forward);
            if (inherited_next) {
                const double width = std::abs(node.l_m - inherited->l_m);
                if (supportAllowedForNode(node, width, cfg)) {
                    result.push_back({
                        inherited,
                        inherited_next,
                        node.l_m - inherited->l_m,
                        inherited->l_m > node.l_m,
                        width <= cfg.max_junction_probe_width_m});
                    return result;
                }
            }
        }
    }
    for (std::uint64_t candidate_id : graph.adjacentNodeIds(node.node_id)) {
        const auto* candidate = graph.node(candidate_id);
        if (!candidate || candidate->node_id == node.node_id) continue;
        if (!laneLine(candidate->semantic_type)) continue;
        const auto* next = graph.linkedNeighbor(candidate->node_id, forward);
        if (!next) continue;
        const double width = std::abs(node.l_m - candidate->l_m);
        if (width > cfg.max_corridor_support_width_m) continue;
        if (!supportAllowedForNode(node, width, cfg)) continue;
        result.push_back({candidate, next, node.l_m - candidate->l_m,
                          candidate->l_m > node.l_m,
                          width <= cfg.max_junction_probe_width_m});
    }
    std::sort(result.begin(), result.end(), [&](const auto& a, const auto& b) {
        if (a.support_is_left != b.support_is_left) return a.support_is_left > b.support_is_left;
        const bool a_stable = validLaneWidth(std::abs(a.width_m), cfg);
        const bool b_stable = validLaneWidth(std::abs(b.width_m), cfg);
        if (a_stable != b_stable) return a_stable > b_stable;
        return std::abs(a.width_m) < std::abs(b.width_m);
    });
    return result;
}

std::vector<std::uint64_t> nearNodes(const GraphWork& graph,
                                     const FrenetSliceGraphNode& node,
                                     int target_slice,
                                     double predicted_l,
                                     const FrenetSliceGraphBuilder::Config& cfg) {
    std::vector<std::uint64_t> result;
    if (passiveBoundary(node.semantic_type)) return result;
    for (std::uint64_t candidate_id : graph.activeNodesAtSlice(target_slice)) {
        const auto* candidate = graph.node(candidate_id);
        if (!candidate || candidate->final_ft_id == node.final_ft_id) continue;
        if (!laneLine(candidate->semantic_type)) continue;
        if (std::abs(candidate->l_m - predicted_l) <= cfg.near_node_distance_m) {
            result.push_back(candidate_id);
        }
    }
    return result;
}

bool passiveRepairEndpointDirection(
    const FrenetSliceGraphNode& node,
    bool forward,
    const std::map<std::uint64_t, std::pair<int, int>>& raw_ft_slice_ranges) {
    if (!passiveBoundary(node.semantic_type)) return true;
    const auto it = raw_ft_slice_ranges.find(node.raw_ft_id);
    if (it == raw_ft_slice_ranges.end()) return false;
    if (node.state == "inferred" && node.provenance == "ribbon_reconstruction") {
        return forward ? node.slice_index >= it->second.second
                       : node.slice_index <= it->second.first;
    }
    return forward ? node.slice_index == it->second.second
                   : node.slice_index == it->second.first;
}

}  // namespace

FrenetSliceGraphOutput FrenetSliceGraphBuilder::build(
    const FrenetSliceIntersectionOutput& intersections,
    const RawFtFilterOutput& filter,
    const RawFtAssociationOutput& associations) const {
    FrenetSliceGraphOutput output;
    if (!intersections.ok || !filter.ok) {
        output.error = !intersections.ok ? intersections.error : filter.error;
        if (output.error.empty()) output.error = "invalid_frenet_slice_graph_input";
        return output;
    }

    const auto decisions = buildDecisionMap(filter);
    const auto kept_nodes = buildKeptNodes(intersections, decisions);
    const auto raw_ft_slice_ranges = buildRawFtSliceRanges(kept_nodes);
    const auto final_ft_by_raw_ft = buildFinalFtMap(kept_nodes, associations);
    GraphWork graph(&output);

    for (const auto& [raw_ft_id, by_slice] : kept_nodes) {
        std::uint64_t previous_node = 0;
        int previous_slice = -1;
        const auto final_it = final_ft_by_raw_ft.find(raw_ft_id);
        const std::uint64_t final_ft_id =
            final_it == final_ft_by_raw_ft.end() ? raw_ft_id : final_it->second;
        for (const auto& [slice_index, source] : by_slice) {
            const std::uint64_t node_id = graph.addObservedNode(source, final_ft_id);
            if (previous_node != 0 && slice_index == previous_slice + 1) {
                graph.addLonLink(previous_node, node_id, "observed", source.confidence,
                                 "final_ft_consecutive_samples");
            }
            previous_node = node_id;
            previous_slice = slice_index;
        }
    }
    for (const auto& candidate : associations.candidates) {
        if (candidate.classification != "ready_continuation") continue;
        const auto from_range = raw_ft_slice_ranges.find(candidate.from_raw_ft_id);
        const auto to_range = raw_ft_slice_ranges.find(candidate.to_raw_ft_id);
        const auto final_it = final_ft_by_raw_ft.find(candidate.from_raw_ft_id);
        if (from_range == raw_ft_slice_ranges.end() ||
            to_range == raw_ft_slice_ranges.end() ||
            final_it == final_ft_by_raw_ft.end()) {
            continue;
        }
        const int from_slice = from_range->second.second;
        const int to_slice = to_range->second.first;
        if (to_slice <= from_slice) continue;
        const auto* from_node = graph.nodeByFtSlice(final_it->second, from_slice);
        const auto* to_node = graph.nodeByFtSlice(final_it->second, to_slice);
        if (!from_node || !to_node) continue;
        std::uint64_t previous_node_id = from_node->node_id;
        for (int slice = from_slice + 1; slice < to_slice; ++slice) {
            if (const auto* existing = graph.nodeByFtSlice(final_it->second, slice)) {
                graph.addLonLink(previous_node_id, existing->node_id, "inferred", 0.9,
                                 "raw_ft_association_existing_final_ft_node");
                previous_node_id = existing->node_id;
                continue;
            }
            const double s = intersections.slices[static_cast<std::size_t>(slice)].s_m;
            const double ratio = (s - from_node->s_m) / (to_node->s_m - from_node->s_m);
            const double l = from_node->l_m + ratio * (to_node->l_m - from_node->l_m);
            const std::uint64_t connector_id =
                graph.addAssociationConnectorNode(*from_node, *to_node, slice, s, l);
            graph.addLonLink(previous_node_id, connector_id, "inferred", 0.9,
                             "raw_ft_association_connector");
            previous_node_id = connector_id;
        }
        graph.addLonLink(previous_node_id, to_node->node_id, "inferred", 0.9,
                         "raw_ft_association_connector");
    }
    graph.rebuildLateralLinks();

    bool changed = true;
    int iteration = 0;
    while (changed && iteration < cfg_.max_repair_iterations) {
        changed = false;
        ++iteration;
        std::vector<std::pair<std::uint64_t, bool>> frontier;
        for (const auto& node : output.nodes) {
            if (!activeNodeState(node) || !repairableBoundary(node, decisions, cfg_)) continue;
            if (!graph.hasDirectionalLonLink(node.node_id, true) &&
                passiveRepairEndpointDirection(node, true, raw_ft_slice_ranges)) {
                frontier.push_back({node.node_id, true});
            }
            if (!graph.hasDirectionalLonLink(node.node_id, false) &&
                passiveRepairEndpointDirection(node, false, raw_ft_slice_ranges)) {
                frontier.push_back({node.node_id, false});
            }
        }

        for (const auto& [node_id, forward] : frontier) {
            const auto* node = graph.node(node_id);
            if (!node || graph.hasDirectionalLonLink(node_id, forward)) continue;
            const int target_slice = node->slice_index + (forward ? 1 : -1);
            if (target_slice < 0 || target_slice >= static_cast<int>(intersections.slices.size())) continue;

            if (const auto* existing = graph.nodeByFtSlice(node->final_ft_id, target_slice)) {
                graph.addLonLink(forward ? node_id : existing->node_id,
                                 forward ? existing->node_id : node_id,
                                 "inferred", 0.9,
                                 "lonlink_repair_existing_final_ft_node");
                changed = true;
                continue;
            }

            const double target_s = intersections.slices[static_cast<std::size_t>(target_slice)].s_m;
            const auto self_prediction = predictBySelfTrend(graph, *node, forward, target_s);
            if (self_prediction) {
                const auto self_near_nodes = nearNodes(graph, *node, target_slice,
                                                       *self_prediction, cfg_);
                if (!self_near_nodes.empty()) {
                    for (std::uint64_t near_id : self_near_nodes) {
                        graph.addLonLink(forward ? node_id : near_id,
                                         forward ? near_id : node_id,
                                         "near_topology", 0.8,
                                         "lonlink_repair_self_trend_near_topology_stop");
                    }
                    continue;
                }
            }

            const auto supports = adjacentSupports(graph, *node, forward, cfg_);
            if (supports.empty()) continue;
            const auto& support = supports.front();
            const double predicted_l = support.next->l_m + support.width_m;
            const auto near_nodes = nearNodes(graph, *node, target_slice, predicted_l, cfg_);
            if (!near_nodes.empty()) {
                for (std::uint64_t near_id : near_nodes) {
                    graph.addLonLink(forward ? node_id : near_id,
                                     forward ? near_id : node_id,
                                     "near_topology", 0.8,
                                     "lonlink_repair_near_topology_stop");
                }
                continue;
            }
            if (!supportAllowedForNode(*node, support.width_m, cfg_)) continue;

            const std::uint64_t inferred_id = graph.addInferredNode(
                *node, target_slice, target_s, predicted_l, *support.next, support.width_m,
                "lonlink_repair_lateral_support");
            graph.addLonLink(forward ? node_id : inferred_id,
                             forward ? inferred_id : node_id,
                             "inferred", 0.8,
                             "lonlink_repair_lateral_support");
            changed = true;
        }
        if (changed) graph.rebuildLateralLinks();
    }

    output.ok = true;
    if (output.nodes.empty()) output.error = "empty_frenet_slice_graph";
    return output;
}

}  // namespace topology_map::topology_v3
