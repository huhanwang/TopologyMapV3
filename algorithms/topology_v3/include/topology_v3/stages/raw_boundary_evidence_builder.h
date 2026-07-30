#pragma once

#include <cstddef>
#include <string>

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class RawBoundaryEvidenceBuilder {
public:
    struct Config {
        std::string preferred_source = "vehicle_bev_lanes";
        double max_abs_l_m = 35.0;
        std::size_t min_sample_count = 2;
    };

    RawBoundaryEvidenceOutput build(
        const ReplayFrameInput& input,
        const FusedReferenceOutput& fused_reference) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
