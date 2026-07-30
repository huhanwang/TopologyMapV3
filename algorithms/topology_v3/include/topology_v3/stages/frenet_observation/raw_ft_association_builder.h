#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class RawFtAssociationBuilder {
public:
    struct Config {
        double max_association_gap_m = 50.0;
        double max_association_near_gap_m = 0.5;
        double max_endpoint_l_delta_m = 2.0;
        double max_width_delta_m = 1.2;
        double max_endpoint_touch_gap_m = 0.5;
        double max_endpoint_touch_l_delta_m = 0.5;
    };

    RawFtAssociationOutput build(
        const FrenetSliceIntersectionOutput& intersections,
        const RawFtFilterOutput& filter) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
