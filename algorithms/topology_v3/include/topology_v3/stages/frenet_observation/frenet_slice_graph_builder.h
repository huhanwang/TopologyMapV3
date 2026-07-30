#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class FrenetSliceGraphBuilder {
public:
    struct Config {
        double max_crossing_short_ft_length_m = 30.0;
        double crossing_contact_distance_m = 0.8;
        double max_unconfirmed_short_final_ft_length_m = 20.0;
        int min_unconfirmed_short_final_ft_samples = 4;
        double max_unconfirmed_narrow_ribbon_width_m = 1.8;
        double max_replacement_ribbon_width_m = 5.5;
        double near_node_distance_m = 0.5;
        double min_stable_ribbon_width_m = 2.7;
        double max_junction_probe_width_m = 0.8;
        double max_stable_ribbon_width_m = 4.8;
        double max_corridor_support_width_m = 7.0;
        double max_support_width_span_m = 0.75;
        double min_passive_support_length_m = 20.0;
        double min_passive_repair_width_m = 2.7;
        int max_repair_iterations = 80;
    };

    FrenetSliceGraphOutput build(
        const FrenetSliceIntersectionOutput& intersections,
        const RawFtFilterOutput& filter,
        const RawFtAssociationOutput& associations) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
