#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class RawBoundaryEvidenceBuilder {
public:
    struct Config {
        double sample_spacing_m = 2.0;
        double max_abs_l_m = 35.0;
        double half_length_m = 40.0;
    };

    RawBoundaryEvidenceOutput build(
        const ReplayFrameInput& input,
        const FusedReferenceOutput& fused_reference) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
