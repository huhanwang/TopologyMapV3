#pragma once

#include <cstddef>
#include <string>

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class RawVisualBoundaryPreprocessor {
public:
    struct Config {
        std::string preferred_source = "vehicle_bev_lanes";
        std::size_t min_input_point_count = 2;
        double min_hard_keep_length_m = 6.0;
        double max_abs_y_vcs_m = 50.0;
        bool keep_passive_semantic_boundaries = true;
    };

    RawVisualBoundaryPreprocessOutput build(const ReplayFrameInput& input) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
