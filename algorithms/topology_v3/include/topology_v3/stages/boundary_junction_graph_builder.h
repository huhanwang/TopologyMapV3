#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class BoundaryJunctionGraphBuilder {
public:
    BoundaryJunctionGraphOutput build(
        const FrenetSliceGraphOutput& slice_graph,
        const JunctionEvidenceOutput& junction_evidence) const;
};

}  // namespace topology_map::topology_v3
