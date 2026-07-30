#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class RibbonProfileCompiler {
public:
    struct Config {
        double min_valid_width_m = 0.3;
        double max_valid_width_m = 6.5;
        double stable_width_delta_m = 0.45;
        double stable_local_width_delta_m = 0.35;
        double monotonic_width_epsilon_m = 0.05;
    };

    RawRibbonGraphOutput build(const FrenetSliceGraphOutput& slice_graph) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
