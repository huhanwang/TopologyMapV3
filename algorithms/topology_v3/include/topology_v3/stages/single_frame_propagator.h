#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class SingleFramePropagator {
public:
    struct Config {
        double min_relation_quality = 0.5;
    };

    SingleFramePropagationOutput build(
        const ResegmentedBoundaryOutput& boundaries,
        const RawRibbonGraphOutput& ribbon_profiles) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
