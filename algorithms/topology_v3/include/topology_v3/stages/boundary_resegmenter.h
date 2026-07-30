#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class BoundaryResegmenter {
public:
    ResegmentedBoundaryOutput apply(
        const LonLinkRepairOutput& lonlink_repair,
        const JunctionEvidenceOutput& junction_evidence) const;
};

}  // namespace topology_map::topology_v3
