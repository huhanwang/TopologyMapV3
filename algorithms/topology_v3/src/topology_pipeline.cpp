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
    output.raw_boundary_evidence =
        raw_boundary_evidence_builder_.build(input, output.fused_reference);

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
        "raw_boundary_evidence",
        "raw_boundary_evidence",
        output.raw_boundary_evidence.ok,
        {output.raw_boundary_evidence.ok ? "raw boundary evidence generated"
                                         : output.raw_boundary_evidence.error}});
    return output;
}

void TopologyPipeline::reset() {}

}  // namespace topology_map::topology_v3
