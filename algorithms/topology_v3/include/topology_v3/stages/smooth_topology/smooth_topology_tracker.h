#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class StatefulSmoothTopologyTracker {
public:
    struct Config {
        int max_missed_frames = 3;
        double min_boundary_confirm_score = 0.7;
        double min_relation_commit_score = 0.7;
        double min_junction_commit_score = 0.7;
    };

    SmoothTopologyOutput update(
        const FrenetSliceGraphOutput& slice_graph,
        const JunctionEvidenceOutput& junction_evidence);
    void reset();

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
