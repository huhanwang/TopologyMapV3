#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "topology_v3/frame_types.h"

namespace topology_map::topology_v3 {

struct FrenetPose {
    double s_m = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
    double tangent_x = 1.0;
    double tangent_y = 0.0;
    double normal_x = 0.0;
    double normal_y = 1.0;
    double heading_rad = 0.0;
    double curvature_m_inv = 0.0;
};

struct FrenetProjection {
    double s_m = 0.0;
    double l_m = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
    double distance_m = 0.0;
};

class FrenetProjector {
public:
    FrenetProjector();
    explicit FrenetProjector(const FusedReferenceOutput& reference);

    void reset(const FusedReferenceOutput& reference);

    bool ok() const { return ok_; }
    const std::string& error() const { return error_; }

    bool poseAt(double s_m, FrenetPose* pose) const;
    bool project(double x_m, double y_m, FrenetProjection* projection) const;
    BoundaryPoint2d unproject(double s_m, double l_m) const;
    std::vector<double> sampleSValues(double spacing_m) const;

private:
    const FusedReferencePoint* pointBeforeOrAt(double s_m, std::size_t* index) const;

    bool ok_ = false;
    std::string error_;
    std::vector<FusedReferencePoint> points_;
};

}  // namespace topology_map::topology_v3
