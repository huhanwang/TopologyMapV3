#include "topology_v3/stages/navigation_reference_builder.h"

#include <algorithm>
#include <cmath>

namespace topology_map::topology_v3 {
namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kMinSegmentVectorLengthM = 1e-3;

struct RoutePoint {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
    double s_m = 0.0;
};

struct RouteProjection {
    bool valid = false;
    std::size_t segment_start_index = 0;
    double s_m = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
    double heading_rad = 0.0;
    double lateral_error_m = 0.0;
    double score = 0.0;
};

double distance2d(double ax, double ay, double bx, double by) {
    return std::hypot(ax - bx, ay - by);
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

RoutePoint geoToRoutePoint(
    const NavigationRoutePointInput& input,
    double origin_latitude,
    double origin_longitude) {
    RoutePoint point;
    point.latitude = input.latitude;
    point.longitude = input.longitude;
    point.altitude = input.altitude;
    point.x_m = (input.longitude - origin_longitude) * std::cos(origin_latitude) * kEarthRadiusM;
    point.y_m = (input.latitude - origin_latitude) * kEarthRadiusM;
    return point;
}

std::vector<RoutePoint> buildGlobalRoute(const NavigationRouteInput& route) {
    std::vector<RoutePoint> global;
    if (!route.available || route.segments.empty()) return global;
    bool has_origin = false;
    double origin_lat = 0.0;
    double origin_lon = 0.0;
    for (const auto& segment : route.segments) {
        if (!segment.points.empty()) {
            origin_lat = segment.points.front().latitude;
            origin_lon = segment.points.front().longitude;
            has_origin = true;
            break;
        }
    }
    if (!has_origin) return global;
    for (const auto& segment : route.segments) {
        for (const auto& input_point : segment.points) {
            RoutePoint point = geoToRoutePoint(input_point, origin_lat, origin_lon);
            if (!global.empty() &&
                distance2d(global.back().x_m, global.back().y_m, point.x_m, point.y_m) < 0.05) {
                continue;
            }
            point.s_m = global.empty()
                ? 0.0
                : global.back().s_m +
                      distance2d(global.back().x_m, global.back().y_m, point.x_m, point.y_m);
            global.push_back(point);
        }
    }
    return global;
}

bool gnssToRouteXY(
    const GnssInput& gnss,
    const std::vector<RoutePoint>& route,
    double* x_m,
    double* y_m) {
    if (!gnss.valid || route.empty() || x_m == nullptr || y_m == nullptr) return false;
    const auto& origin = route.front();
    *x_m = (gnss.longitude - origin.longitude) * std::cos(origin.latitude) * kEarthRadiusM;
    *y_m = (gnss.latitude - origin.latitude) * kEarthRadiusM;
    return true;
}

bool routeXYToVcs(
    const GnssInput& gnss,
    const std::vector<RoutePoint>& route,
    double x_m,
    double y_m,
    BoundaryPoint2d* vcs) {
    double ego_x = 0.0;
    double ego_y = 0.0;
    if (vcs == nullptr || !gnssToRouteXY(gnss, route, &ego_x, &ego_y)) return false;
    const double dx = x_m - ego_x;
    const double dy = y_m - ego_y;
    const double c = std::cos(gnss.yaw_rad);
    const double s = std::sin(gnss.yaw_rad);
    vcs->x_m = dx * c + dy * s;
    vcs->y_m = -dx * s + dy * c;
    return true;
}

bool vcsToRouteXY(
    const GnssInput& gnss,
    const std::vector<RoutePoint>& route,
    const ReferencePoint& vcs,
    double* x_m,
    double* y_m) {
    double ego_x = 0.0;
    double ego_y = 0.0;
    if (x_m == nullptr || y_m == nullptr || !gnssToRouteXY(gnss, route, &ego_x, &ego_y)) {
        return false;
    }
    const double c = std::cos(gnss.yaw_rad);
    const double s = std::sin(gnss.yaw_rad);
    *x_m = ego_x + vcs.x_m * c - vcs.y_m * s;
    *y_m = ego_y + vcs.x_m * s + vcs.y_m * c;
    return true;
}

RouteProjection projectToGlobalRoute(const std::vector<RoutePoint>& route, double x_m, double y_m) {
    RouteProjection best;
    for (std::size_t i = 1; i < route.size(); ++i) {
        const auto& a = route[i - 1];
        const auto& b = route[i];
        const double vx = b.x_m - a.x_m;
        const double vy = b.y_m - a.y_m;
        const double len2 = vx * vx + vy * vy;
        if (len2 < kMinSegmentVectorLengthM) continue;
        const double t = clamp01(((x_m - a.x_m) * vx + (y_m - a.y_m) * vy) / len2);
        const double px = a.x_m + t * vx;
        const double py = a.y_m + t * vy;
        const double lateral = distance2d(x_m, y_m, px, py);
        const double score = lateral;
        if (best.valid && score >= best.score) continue;
        best.valid = true;
        best.segment_start_index = i - 1;
        best.s_m = a.s_m + std::sqrt(len2) * t;
        best.x_m = px;
        best.y_m = py;
        best.heading_rad = std::atan2(vy, vx);
        best.lateral_error_m = lateral;
        best.score = score;
    }
    return best;
}

}  // namespace

NavigationReferenceOutput NavigationReferenceBuilder::build(
    const ReplayFrameInput& input,
    const VisualReferenceOutput& visual_reference) const {
    NavigationReferenceOutput result;
    if (!input.navigation_route || !input.navigation_route->available) {
        result.error = "route_not_ready";
        return result;
    }
    if (!input.gnss || !input.gnss->valid) {
        result.error = "invalid_gnss";
        return result;
    }
    if (!visual_reference.ok || visual_reference.points.size() < 2) {
        result.error = "invalid_visual_reference";
        return result;
    }
    const auto route = buildGlobalRoute(*input.navigation_route);
    if (route.size() < 2) {
        result.error = "route_not_ready";
        return result;
    }
    const auto& visual_mid = visual_reference.points[visual_reference.points.size() / 2];
    double mid_route_x = 0.0;
    double mid_route_y = 0.0;
    if (!vcsToRouteXY(*input.gnss, route, visual_mid, &mid_route_x, &mid_route_y)) {
        result.error = "invalid_gnss";
        return result;
    }
    const auto projection = projectToGlobalRoute(route, mid_route_x, mid_route_y);
    if (!projection.valid) {
        result.error = "no_global_route_projection";
        return result;
    }

    const auto& visual_front = visual_reference.points.back();
    const auto& visual_back = visual_reference.points.front();
    const double visual_heading_vcs =
        std::atan2(visual_front.y_m - visual_back.y_m, visual_front.x_m - visual_back.x_m);
    const double route_heading_vcs = projection.heading_rad - input.gnss->yaw_rad;
    result.heading_error_rad = std::abs(normalizeAngle(route_heading_vcs - visual_heading_vcs));
    result.lateral_error_m = projection.lateral_error_m;
    if (result.lateral_error_m > cfg_.max_lateral_error_m) {
        result.error = "navigation_reference_lateral_mismatch";
        return result;
    }
    if (result.heading_error_rad > cfg_.max_heading_error_rad) {
        result.error = "navigation_reference_heading_mismatch";
        return result;
    }

    BoundaryPoint2d anchor_vcs;
    if (!routeXYToVcs(*input.gnss, route, projection.x_m, projection.y_m, &anchor_vcs)) {
        result.error = "invalid_gnss";
        return result;
    }

    std::vector<RoutePoint> forward_route_points;
    RoutePoint anchor;
    anchor.x_m = projection.x_m;
    anchor.y_m = projection.y_m;
    anchor.s_m = projection.s_m;
    forward_route_points.push_back(anchor);
    std::size_t forward_idx = projection.segment_start_index + 1;
    double last_heading = projection.heading_rad;
    double last_x = projection.x_m;
    double last_y = projection.y_m;
    while (forward_idx < route.size()) {
        const auto& point = route[forward_idx];
        const double search_len = point.s_m - projection.s_m;
        BoundaryPoint2d point_vcs;
        if (!routeXYToVcs(*input.gnss, route, point.x_m, point.y_m, &point_vcs)) {
            result.stop_reason_forward = "invalid_gnss";
            break;
        }
        if (point_vcs.x_m >= cfg_.front_x_m) {
            result.forward_length_m = cfg_.front_x_m;
            result.stop_reason_forward = "front_x";
            forward_route_points.push_back(point);
            break;
        }
        if (search_len > cfg_.max_search_forward_m) {
            result.stop_reason_forward = "max_forward_search";
            break;
        }
        const double heading = std::atan2(point.y_m - last_y, point.x_m - last_x);
        if (std::abs(normalizeAngle(heading - last_heading)) > cfg_.max_local_turn_rad) {
            result.stop_reason_forward = "local_turn";
            break;
        }
        if (std::abs(normalizeAngle(heading - projection.heading_rad)) >
            cfg_.max_total_heading_change_rad) {
            result.stop_reason_forward = "total_heading_change";
            break;
        }
        forward_route_points.push_back(point);
        result.forward_length_m = std::max(result.forward_length_m, point_vcs.x_m);
        last_heading = heading;
        last_x = point.x_m;
        last_y = point.y_m;
        ++forward_idx;
    }
    if (result.stop_reason_forward.empty()) result.stop_reason_forward = "route_end";

    std::vector<RoutePoint> backward_route_points;
    if (projection.segment_start_index < route.size()) {
        std::size_t backward_idx = projection.segment_start_index;
        last_heading = projection.heading_rad;
        last_x = projection.x_m;
        last_y = projection.y_m;
        while (true) {
            const auto& point = route[backward_idx];
            const double search_len = projection.s_m - point.s_m;
            BoundaryPoint2d point_vcs;
            if (!routeXYToVcs(*input.gnss, route, point.x_m, point.y_m, &point_vcs)) {
                result.stop_reason_backward = "invalid_gnss";
                break;
            }
            if (point_vcs.x_m <= cfg_.rear_x_m) {
                result.backward_length_m = std::abs(cfg_.rear_x_m);
                result.stop_reason_backward = "rear_x";
                backward_route_points.push_back(point);
                break;
            }
            if (search_len > cfg_.max_search_backward_m) {
                result.stop_reason_backward = "max_backward_search";
                break;
            }
            const double heading = std::atan2(last_y - point.y_m, last_x - point.x_m);
            if (std::abs(normalizeAngle(heading - last_heading)) > cfg_.max_local_turn_rad) {
                result.stop_reason_backward = "local_turn";
                break;
            }
            if (std::abs(normalizeAngle(heading - projection.heading_rad)) >
                cfg_.max_total_heading_change_rad) {
                result.stop_reason_backward = "total_heading_change";
                break;
            }
            backward_route_points.push_back(point);
            result.backward_length_m = std::max(result.backward_length_m, std::max(0.0, -point_vcs.x_m));
            last_heading = heading;
            last_x = point.x_m;
            last_y = point.y_m;
            if (backward_idx == 0) {
                result.stop_reason_backward = "route_start";
                break;
            }
            --backward_idx;
        }
    }
    if (result.stop_reason_backward.empty()) result.stop_reason_backward = "route_start";
    if (result.forward_length_m < cfg_.min_forward_m) {
        result.error = "navigation_reference_too_short";
        return result;
    }

    for (auto it = backward_route_points.rbegin(); it != backward_route_points.rend(); ++it) {
        BoundaryPoint2d vcs;
        if (routeXYToVcs(*input.gnss, route, it->x_m, it->y_m, &vcs)) {
            result.vcs_points.push_back(vcs);
        }
    }
    result.vcs_points.push_back(anchor_vcs);
    for (std::size_t i = 1; i < forward_route_points.size(); ++i) {
        BoundaryPoint2d vcs;
        if (routeXYToVcs(*input.gnss, route, forward_route_points[i].x_m,
                         forward_route_points[i].y_m, &vcs)) {
            result.vcs_points.push_back(vcs);
        }
    }
    if (result.vcs_points.size() < 2) {
        result.error = "navigation_reference_empty_after_projection";
        return result;
    }
    result.ok = true;
    return result;
}

}  // namespace topology_map::topology_v3
