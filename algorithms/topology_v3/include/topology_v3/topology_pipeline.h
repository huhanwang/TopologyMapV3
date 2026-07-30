#pragma once

#include "topology_v3/frame_types.h"
#include "topology_v3/stages/boundary_resegmenter.h"
#include "topology_v3/stages/fused_reference_builder.h"
#include "topology_v3/stages/frenet_observation/frenet_slice_graph_builder.h"
#include "topology_v3/stages/frenet_observation/frenet_slice_intersection_builder.h"
#include "topology_v3/stages/frenet_observation/raw_ft_association_builder.h"
#include "topology_v3/stages/frenet_observation/raw_ft_filter_builder.h"
#include "topology_v3/stages/frenet_observation/raw_visual_boundary_preprocessor.h"
#include "topology_v3/stages/frenet_observation/ribbon_profile_compiler.h"
#include "topology_v3/stages/junction_evidence_compiler.h"
#include "topology_v3/stages/key_slice_graph_builder.h"
#include "topology_v3/stages/lonlink_repairer.h"
#include "topology_v3/stages/navigation_reference_builder.h"
#include "topology_v3/stages/raw_boundary_evidence_builder.h"
#include "topology_v3/stages/single_frame_propagator.h"
#include "topology_v3/stages/smooth_topology/smooth_topology_tracker.h"
#include "topology_v3/stages/visual_reference_builder.h"

namespace topology_map::topology_v3 {

class TopologyPipeline {
public:
    TopologyFrameOutput update(const ReplayFrameInput& input);
    void reset();

private:
    VisualReferenceBuilder visual_reference_builder_;
    NavigationReferenceBuilder navigation_reference_builder_;
    FusedReferenceBuilder fused_reference_builder_;
    RawVisualBoundaryPreprocessor raw_visual_boundary_preprocessor_;
    RawBoundaryEvidenceBuilder raw_boundary_evidence_builder_;
    FrenetSliceIntersectionBuilder frenet_slice_intersection_builder_;
    RawFtFilterBuilder raw_ft_filter_builder_;
    RawFtAssociationBuilder raw_ft_association_builder_;
    FrenetSliceGraphBuilder frenet_slice_graph_builder_;
    RibbonProfileCompiler ribbon_profile_compiler_;
    LonLinkRepairer lonlink_repairer_;
    KeySliceGraphBuilder key_slice_graph_builder_;
    JunctionEvidenceCompiler junction_evidence_compiler_;
    BoundaryResegmenter boundary_resegmenter_;
    SingleFramePropagator single_frame_propagator_;
    StatefulSmoothTopologyTracker smooth_topology_tracker_;
};

}  // namespace topology_map::topology_v3
