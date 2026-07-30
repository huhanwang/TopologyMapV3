#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class FrenetSliceIntersectionBuilder {
public:
    struct Config {
        double slice_step_m = 2.0;
        double max_abs_l_m = 35.0;
        double normal_half_length_m = 40.0;
    };

    FrenetSliceIntersectionOutput build(
        const RawVisualBoundaryPreprocessOutput& preprocessed,
        const FusedReferenceOutput& fused_reference,
        const SmoothPoseInput* smooth_pose) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
