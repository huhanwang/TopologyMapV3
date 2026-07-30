#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class RawRibbonGraphBuilder {
public:
    struct Config {
        double max_pair_lateral_gap_m = 8.0;
        double min_overlap_m = 8.0;
        double sample_step_m = 2.0;
        double min_valid_width_m = 0.3;
        double max_valid_width_m = 6.5;
        double stable_width_delta_m = 0.45;
        double stable_local_width_delta_m = 0.35;
        double monotonic_width_epsilon_m = 0.05;
        double min_valid_width_ratio = 0.6;
        double min_stable_ratio = 0.7;
    };

    RawRibbonGraphOutput build(const RawBoundaryEvidenceOutput& raw_evidence) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
