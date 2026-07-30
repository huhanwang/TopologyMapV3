#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class JunctionEvidenceCompiler {
public:
    struct Config {
        double min_candidate_confidence = 0.5;
        double max_contact_distance_m = 0.5;
    };

    JunctionEvidenceOutput build(const KeySliceGraphOutput& key_slice_graph) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
