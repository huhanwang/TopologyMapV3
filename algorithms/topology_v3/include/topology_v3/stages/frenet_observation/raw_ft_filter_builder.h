#pragma once

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

class RawFtFilterBuilder {
public:
    struct Config {
        int min_sample_count = 3;
        double min_support_length_m = 6.0;
        double short_support_length_m = 16.0;
        double min_stable_ribbon_width_m = 2.7;
        double max_stable_ribbon_width_m = 4.8;
        int min_stable_ribbon_sample_count = 3;
        double semantic_attachment_distance_m = 1.0;
        int min_semantic_attachment_samples = 4;
        double min_semantic_attachment_length_m = 8.0;
        int min_close_duplicate_overlap_count = 3;
        double min_close_duplicate_short_overlap_ratio = 0.7;
        double max_close_duplicate_median_l_delta_m = 0.35;
        double max_close_duplicate_l_delta_m = 0.6;
        double min_close_duplicate_length_ratio = 1.5;
        double far_short_start_s_m = 100.0;
        int min_far_short_narrow_overlap_count = 3;
        double far_short_full_score_length_m = 8.0;
        double far_short_zero_score_length_m = 20.0;
        double far_narrow_full_score_width_m = 1.0;
        double far_narrow_zero_score_width_m = 2.2;
        double far_short_noise_score_threshold = 0.65;
    };

    RawFtFilterOutput build(const FrenetSliceIntersectionOutput& intersections) const;

private:
    Config cfg_;
};

}  // namespace topology_map::topology_v3
