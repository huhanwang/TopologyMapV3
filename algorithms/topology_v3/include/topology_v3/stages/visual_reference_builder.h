#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class VisualReferenceBuilder {
public:
    struct Config {
        std::vector<std::string> preferred_sources{"smooth_bev_lanes", "vehicle_bev_lanes"};
        double slice_spacing_m = 2.0;
        double min_line_x_span_m = 20.0;
        double min_reference_length_m = 20.0;
        double max_reference_extension_m = 5.0;
        double max_length_m = 120.0;
        double max_abs_d_m = 35.0;
        double pair_width_sample_step_m = 5.0;
        double min_pair_width_m = 2.4;
        double max_pair_width_m = 6.2;
        double max_pair_width_span_m = 2.6;
        double max_pair_invalid_width_ratio = 0.2;
    };

    VisualReferenceOutput build(const ReplayFrameInput& input) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
