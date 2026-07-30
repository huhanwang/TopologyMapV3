#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class NavigationReferenceBuilder {
public:
    struct Config {
        double front_x_m = 250.0;
        double rear_x_m = -20.0;
        double max_search_forward_m = 360.0;
        double max_search_backward_m = 120.0;
        double min_forward_m = 80.0;
        double max_lateral_error_m = 12.0;
        double max_heading_error_rad = 0.45;
        double max_local_turn_rad = 0.52;
        double max_total_heading_change_rad = 1.57;
    };

    NavigationReferenceOutput build(
        const ReplayFrameInput& input,
        const VisualReferenceOutput& visual_reference) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
