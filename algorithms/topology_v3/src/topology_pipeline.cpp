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
    output.raw_ribbon_graph =
        raw_ribbon_graph_builder_.build(output.raw_boundary_evidence);

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
        "raw_ribbon_graph",
        "raw_ribbon_graph",
        output.raw_ribbon_graph.ok,
        {output.raw_ribbon_graph.ok ? "raw ribbon graph generated"
                                    : output.raw_ribbon_graph.error}});
    return output;
}

void TopologyPipeline::reset() {}

}  // namespace topology_map::topology_v3
