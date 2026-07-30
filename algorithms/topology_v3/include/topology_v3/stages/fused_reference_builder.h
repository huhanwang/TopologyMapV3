#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class FusedReferenceBuilder {
public:
    struct Config {
        double sample_step_m = 2.0;
        double min_visual_length_m = 15.0;
        double min_navigation_overlap_m = 20.0;
        double max_lateral_offset_m = 12.0;
        double max_heading_error_rad = 0.35;
        double max_extension_m = 150.0;
        double max_heading_delta_from_visual_rad = 0.9;
        double max_heading_step_rad = 0.03;
        double navigation_trend_ramp_m = 55.0;
    };

    FusedReferenceOutput build(
        const VisualReferenceOutput& visual_reference,
        const NavigationReferenceOutput& navigation_reference,
        const FusedReferenceOutput* previous_reference = nullptr) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
