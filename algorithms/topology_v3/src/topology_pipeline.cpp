#include "topology_v3/topology_pipeline.h"

namespace topology_map::topology_v3 {

TopologyFrameOutput TopologyPipeline::update(const ReplayFrameInput& input) {
    TopologyFrameOutput output;
    output.frame_id = input.frame_id;
    output.visual_reference = visual_reference_builder_.build(input);
    output.navigation_reference =
        navigation_reference_builder_.build(input, output.visual_reference);
    output.fused_reference =
        fused_reference_builder_.build(output.visual_reference, output.navigation_reference);
    output.raw_visual_preprocess = raw_visual_boundary_preprocessor_.build(input);
    output.raw_boundary_evidence =
        raw_boundary_evidence_builder_.build(output.raw_visual_preprocess, output.fused_reference,
                                             input.smooth_pose ? &*input.smooth_pose : nullptr);
    output.frenet_slice_intersections =
        frenet_slice_intersection_builder_.build(output.raw_visual_preprocess,
                                                 output.fused_reference,
                                                 input.smooth_pose ? &*input.smooth_pose : nullptr);
    output.raw_ft_filter = raw_ft_filter_builder_.build(output.frenet_slice_intersections);
    output.raw_ft_association =
        raw_ft_association_builder_.build(output.frenet_slice_intersections,
                                          output.raw_ft_filter);
    output.frenet_slice_graph =
        frenet_slice_graph_builder_.build(output.frenet_slice_intersections,
                                          output.raw_ft_filter,
                                          output.raw_ft_association);
    output.junction_evidence =
        junction_evidence_compiler_.build(output.frenet_slice_graph);
    output.boundary_junction_graph =
        boundary_junction_graph_builder_.build(output.frenet_slice_graph,
                                               output.junction_evidence);

    output.debug_layers.push_back({
        "visual_reference",
        "visual_references",
        output.visual_reference.ok,
        {output.visual_reference.ok ? "visual reference generated"
                                    : output.visual_reference.error}});
    output.debug_layers.push_back({
        "navigation_reference",
        "navigation_references",
        output.navigation_reference.ok,
        {output.navigation_reference.ok ? "navigation reference generated"
                                        : output.navigation_reference.error}});
    output.debug_layers.push_back({
        "fused_reference",
        "fused_reference",
        output.fused_reference.ok,
        {output.fused_reference.ok ? "fused reference generated"
                                   : output.fused_reference.error}});
    output.debug_layers.push_back({
        "raw_visual_preprocess",
        "raw_visual_preprocess",
        output.raw_visual_preprocess.ok,
        {output.raw_visual_preprocess.ok ? "raw visual preprocess generated"
                                         : output.raw_visual_preprocess.error}});
    output.debug_layers.push_back({
        "raw_boundary_evidence",
        "raw_boundary_evidence",
        output.raw_boundary_evidence.ok,
        {output.raw_boundary_evidence.ok ? "raw boundary evidence generated"
                                         : output.raw_boundary_evidence.error}});
    output.debug_layers.push_back({
        "frenet_slice_intersections",
        "frenet_slice_intersections",
        output.frenet_slice_intersections.ok,
        {output.frenet_slice_intersections.ok ? "frenet slice intersections generated"
                                              : output.frenet_slice_intersections.error}});
    output.debug_layers.push_back({
        "raw_ft_filter",
        "raw_ft_filter",
        output.raw_ft_filter.ok,
        {output.raw_ft_filter.ok ? "raw FT filter generated"
                                 : output.raw_ft_filter.error}});
    output.debug_layers.push_back({
        "raw_ft_association",
        "raw_ft_association",
        output.raw_ft_association.ok,
        {output.raw_ft_association.ok ? "raw FT association generated"
                                      : output.raw_ft_association.error}});
    output.debug_layers.push_back({
        "frenet_slice_graph",
        "frenet_slice_graph",
        output.frenet_slice_graph.ok,
        {output.frenet_slice_graph.ok ? "frenet slice graph generated"
                                      : output.frenet_slice_graph.error}});
    output.debug_layers.push_back({
        "junction_evidence",
        "junction_evidence",
        output.junction_evidence.ok,
        {output.junction_evidence.ok ? "junction evidence generated"
                                     : output.junction_evidence.error}});
    output.debug_layers.push_back({
        "boundary_junction_graph",
        "boundary_junction_graph",
        output.boundary_junction_graph.ok,
        {output.boundary_junction_graph.ok ? "boundary junction graph generated"
                                           : output.boundary_junction_graph.error}});
    return output;
}

void TopologyPipeline::reset() {}

}  // namespace topology_map::topology_v3
