#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class KeySliceGraphBuilder {
public:
    struct Config {
        double key_slice_spacing_m = 10.0;
        double min_link_length_m = 2.0;
    };

    KeySliceGraphOutput build(
        const RawBoundaryEvidenceOutput& raw_evidence,
        const LonLinkRepairOutput& lonlink_repair) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
