#include "topology_v3/geometry/frenet_projector.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace topology_map::topology_v3 {
namespace {

double clampValue(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

}  // namespace

FrenetProjector::FrenetProjector() = default;

FrenetProjector::FrenetProjector(const FusedReferenceOutput& reference) {
    reset(reference);
}

void FrenetProjector::reset(const FusedReferenceOutput& reference) {
    ok_ = false;
    error_.clear();
    points_.clear();
    if (!reference.ok || reference.points.size() < 2) {
        error_ = "invalid_fused_reference";
        return;
    }
    points_ = reference.points;
    std::sort(points_.begin(), points_.end(), [](const auto& a, const auto& b) {
        return a.s_m < b.s_m;
    });
    ok_ = true;
}

const FusedReferencePoint* FrenetProjector::pointBeforeOrAt(
    double s_m,
    std::size_t* index) const {
    if (!ok_ || points_.size() < 2) return nullptr;
    if (s_m <= points_.front().s_m) {
        if (index) *index = 0;
        return &points_.front();
    }
    if (s_m >= points_.back().s_m) {
        if (index) *index = points_.size() - 2;
        return &points_[points_.size() - 2];
    }
    auto it = std::upper_bound(
        points_.begin(), points_.end(), s_m,
        [](double value, const auto& point) { return value < point.s_m; });
    const std::size_t i = static_cast<std::size_t>(std::distance(points_.begin(), it));
    if (index) *index = i - 1;
    return &points_[i - 1];
}

bool FrenetProjector::poseAt(double s_m, FrenetPose* pose) const {
    if (!pose) return false;
    std::size_t index = 0;
    const auto* left = pointBeforeOrAt(s_m, &index);
    if (!left || index + 1 >= points_.size()) return false;
    const auto& right = points_[index + 1];
    const double span = right.s_m - left->s_m;
    const double ratio =
        std::abs(span) < 1e-9 ? 0.0 : clampValue((s_m - left->s_m) / span, 0.0, 1.0);
    const double x = left->x_m + (right.x_m - left->x_m) * ratio;
    const double y = left->y_m + (right.y_m - left->y_m) * ratio;
    const double dx = right.x_m - left->x_m;
    const double dy = right.y_m - left->y_m;
    const double length = std::hypot(dx, dy);

    pose->s_m = s_m;
    pose->x_m = x;
    pose->y_m = y;
    pose->curvature_m_inv =
        left->curvature_m_inv + (right.curvature_m_inv - left->curvature_m_inv) * ratio;
    if (length < 1e-9) {
        pose->tangent_x = 1.0;
        pose->tangent_y = 0.0;
        pose->normal_x = 0.0;
        pose->normal_y = 1.0;
        pose->heading_rad = left->heading_rad;
        return true;
    }
    pose->tangent_x = dx / length;
    pose->tangent_y = dy / length;
    pose->normal_x = -pose->tangent_y;
    pose->normal_y = pose->tangent_x;
    pose->heading_rad = std::atan2(pose->tangent_y, pose->tangent_x);
    return true;
}

BoundaryPoint2d FrenetProjector::unproject(double s_m, double l_m) const {
    FrenetPose pose;
    BoundaryPoint2d point;
    if (!poseAt(s_m, &pose)) return point;
    point.x_m = pose.x_m + pose.normal_x * l_m;
    point.y_m = pose.y_m + pose.normal_y * l_m;
    return point;
}

bool FrenetProjector::project(double x_m, double y_m, FrenetProjection* projection) const {
    if (!projection || !ok_ || points_.size() < 2) return false;
    double best_dist2 = std::numeric_limits<double>::infinity();
    FrenetProjection best;
    for (std::size_t i = 1; i < points_.size(); ++i) {
        const auto& a = points_[i - 1];
        const auto& b = points_[i];
        const double dx = b.x_m - a.x_m;
        const double dy = b.y_m - a.y_m;
        const double len2 = dx * dx + dy * dy;
        if (len2 < 1e-12) continue;
        const double t = clampValue(((x_m - a.x_m) * dx + (y_m - a.y_m) * dy) / len2, 0.0, 1.0);
        const double px = a.x_m + dx * t;
        const double py = a.y_m + dy * t;
        const double ex = x_m - px;
        const double ey = y_m - py;
        const double dist2 = ex * ex + ey * ey;
        if (dist2 >= best_dist2) continue;
        const double len = std::sqrt(len2);
        best_dist2 = dist2;
        best.s_m = a.s_m + (b.s_m - a.s_m) * t;
        best.l_m = ex * (-dy / len) + ey * (dx / len);
        best.x_m = x_m;
        best.y_m = y_m;
        best.distance_m = std::sqrt(dist2);
    }
    if (!std::isfinite(best_dist2)) return false;
    *projection = best;
    return true;
}

std::vector<double> FrenetProjector::sampleSValues(double spacing_m) const {
    std::vector<double> values;
    if (!ok_ || points_.empty()) return values;
    const double start = points_.front().s_m;
    const double end = points_.back().s_m;
    const double step = std::max(1e-6, spacing_m);
    if (end <= start) {
        values.push_back(start);
        return values;
    }
    const int count = static_cast<int>(std::floor((end - start) / step));
    values.reserve(static_cast<std::size_t>(count + 2));
    for (int i = 0; i <= count; ++i) {
        values.push_back(start + static_cast<double>(i) * step);
    }
    if (values.empty() || std::abs(values.back() - end) > 1e-6) {
        values.push_back(end);
    }
    return values;
}

}  // namespace topology_map::topology_v3
