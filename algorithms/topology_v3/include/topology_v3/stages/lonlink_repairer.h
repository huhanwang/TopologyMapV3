#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class LonLinkRepairer {
public:
    struct Config {
        double max_missing_s_gap_m = 25.0;
        double near_contact_lateral_m = 0.5;
        double max_repair_step_m = 2.0;
    };

    LonLinkRepairOutput repair(
        const RawBoundaryEvidenceOutput& raw_evidence,
        const RawRibbonGraphOutput& raw_ribbon_graph,
        const FusedReferenceOutput& fused_reference) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
