#include "topology_v3/stages/fused_reference_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace topology_map::topology_v3 {
namespace {

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    if (values.size() % 2 == 1) return values[mid];
    return 0.5 * (values[mid - 1] + values[mid]);
}

double pointDistance(const BoundaryPoint2d& a, const BoundaryPoint2d& b) {
    return std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
}

bool interpolateYByX(const std::vector<BoundaryPoint2d>& points, double x, double* y) {
    if (!y || points.size() < 2) return false;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const auto& a = points[i - 1];
        const auto& b = points[i];
        const double min_x = std::min(a.x_m, b.x_m);
        const double max_x = std::max(a.x_m, b.x_m);
        if (x < min_x - 1e-6 || x > max_x + 1e-6) continue;
        const double denom = b.x_m - a.x_m;
        const double t = std::abs(denom) < 1e-6 ? 0.0 : (x - a.x_m) / denom;
        *y = lerp(a.y_m, b.y_m, clamp(t, 0.0, 1.0));
        return true;
    }
    return false;
}

double headingAtX(const std::vector<ReferencePoint>& points, double x) {
    if (points.size() < 2) return 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const auto& a = points[i - 1];
        const auto& b = points[i];
        const double min_x = std::min(a.x_m, b.x_m);
        const double max_x = std::max(a.x_m, b.x_m);
        if (x >= min_x - 1e-6 && x <= max_x + 1e-6) {
            return std::atan2(b.y_m - a.y_m, b.x_m - a.x_m);
        }
    }
    const auto& a = points[points.size() - 2];
    const auto& b = points.back();
    return std::atan2(b.y_m - a.y_m, b.x_m - a.x_m);
}

double curvatureFromThreePoints(
    const FusedReferencePoint& a,
    const FusedReferencePoint& b,
    const FusedReferencePoint& c) {
    const double ab = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    const double bc = std::hypot(c.x_m - b.x_m, c.y_m - b.y_m);
    const double ca = std::hypot(a.x_m - c.x_m, a.y_m - c.y_m);
    const double denom = ab * bc * ca;
    if (denom < 1e-6) return 0.0;
    const double cross = (b.x_m - a.x_m) * (c.y_m - a.y_m) -
                         (b.y_m - a.y_m) * (c.x_m - a.x_m);
    return 2.0 * cross / denom;
}

double polyValue(const std::vector<double>& coeffs, double x) {
    double y = 0.0;
    double xp = 1.0;
    for (double coeff : coeffs) {
        y += coeff * xp;
        xp *= x;
    }
    return y;
}

bool visualYAtX(const VisualReferenceOutput& visual, double x, double* y) {
    if (y == nullptr) return false;
    if (!visual.center_coeffs.empty()) {
        *y = polyValue(visual.center_coeffs, x);
        return true;
    }
    std::vector<BoundaryPoint2d> points;
    points.reserve(visual.points.size());
    for (const auto& point : visual.points) points.push_back({point.x_m, point.y_m});
    return interpolateYByX(points, x, y);
}

std::vector<double> cumulativeDistances(const std::vector<BoundaryPoint2d>& points) {
    std::vector<double> cumulative;
    cumulative.reserve(points.size());
    cumulative.push_back(0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        cumulative.push_back(cumulative.back() + pointDistance(points[i - 1], points[i]));
    }
    return cumulative;
}

double smoothedHeadingAtIndex(const std::vector<BoundaryPoint2d>& points, std::size_t index) {
    if (points.size() < 2) return 0.0;
    const std::size_t left = index == 0 ? 0 : index - 1;
    const std::size_t right = std::min(points.size() - 1, index + 1);
    if (left == right) return 0.0;
    return std::atan2(points[right].y_m - points[left].y_m,
                      points[right].x_m - points[left].x_m);
}

std::vector<double> navigationHeadingProfile(const std::vector<BoundaryPoint2d>& nav_points) {
    std::vector<double> headings;
    headings.reserve(nav_points.size());
    if (nav_points.size() < 2) return headings;
    for (std::size_t i = 0; i < nav_points.size(); ++i) {
        const double heading = smoothedHeadingAtIndex(nav_points, i);
        if (headings.empty()) {
            headings.push_back(heading);
        } else {
            headings.push_back(headings.back() + normalizeAngle(heading - headings.back()));
        }
    }
    if (headings.size() < 5) return headings;
    std::vector<double> smoothed;
    smoothed.reserve(headings.size());
    for (std::size_t i = 0; i < headings.size(); ++i) {
        const std::size_t begin = i < 2 ? 0 : i - 2;
        const std::size_t end = std::min(headings.size(), i + 3);
        double sum = 0.0;
        for (std::size_t j = begin; j < end; ++j) sum += headings[j];
        smoothed.push_back(sum / static_cast<double>(end - begin));
    }
    return smoothed;
}

double headingAtDistance(
    const std::vector<double>& cumulative,
    const std::vector<double>& headings,
    double query_s) {
    if (cumulative.empty() || headings.empty()) return 0.0;
    if (query_s <= cumulative.front()) return headings.front();
    for (std::size_t i = 1; i < cumulative.size(); ++i) {
        if (query_s > cumulative[i]) continue;
        const double ds = cumulative[i] - cumulative[i - 1];
        const double t = ds < 1e-6 ? 0.0 : (query_s - cumulative[i - 1]) / ds;
        return lerp(headings[i - 1], headings[i], clamp(t, 0.0, 1.0));
    }
    return headings.back();
}

double projectXToNavigationS(
    const std::vector<BoundaryPoint2d>& points,
    const std::vector<double>& cumulative,
    double query_x) {
    if (points.size() < 2 || cumulative.size() != points.size()) return 0.0;
    double best_s = cumulative.front();
    double best_abs_dx = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < points.size(); ++i) {
        const auto& a = points[i - 1];
        const auto& b = points[i];
        const double min_x = std::min(a.x_m, b.x_m);
        const double max_x = std::max(a.x_m, b.x_m);
        if (query_x >= min_x - 1e-6 && query_x <= max_x + 1e-6) {
            const double dx = b.x_m - a.x_m;
            const double t = std::abs(dx) < 1e-6 ? 0.0 : (query_x - a.x_m) / dx;
            return lerp(cumulative[i - 1], cumulative[i], clamp(t, 0.0, 1.0));
        }
        const double left_abs_dx = std::abs(query_x - a.x_m);
        if (left_abs_dx < best_abs_dx) {
            best_abs_dx = left_abs_dx;
            best_s = cumulative[i - 1];
        }
        const double right_abs_dx = std::abs(query_x - b.x_m);
        if (right_abs_dx < best_abs_dx) {
            best_abs_dx = right_abs_dx;
            best_s = cumulative[i];
        }
    }
    return best_s;
}

bool appendVisualSamples(
    const VisualReferenceOutput& visual,
    double visual_start_x,
    double visual_end_x,
    double sample_step_m,
    std::vector<FusedReferencePoint>* points) {
    if (!points || sample_step_m <= 0.0) return false;
    points->clear();
    double s = 0.0;
    double prev_x = visual_start_x;
    double prev_y = 0.0;
    bool has_prev = false;
    for (double x = visual_start_x; x <= visual_end_x + 1e-6; x += sample_step_m) {
        const double sample_x = std::min(x, visual_end_x);
        double y = 0.0;
        if (!visualYAtX(visual, sample_x, &y)) continue;
        if (has_prev) s += std::hypot(sample_x - prev_x, y - prev_y);
        points->push_back({s, sample_x, y, 0.0, 0.0, "visual"});
        prev_x = sample_x;
        prev_y = y;
        has_prev = true;
        if (sample_x >= visual_end_x) break;
    }
    return points->size() >= 2;
}

void updateGeometry(std::vector<FusedReferencePoint>* points) {
    if (!points || points->size() < 2) return;
    for (std::size_t i = 0; i < points->size(); ++i) {
        const std::size_t left = i == 0 ? 0 : i - 1;
        const std::size_t right = std::min(points->size() - 1, i + 1);
        const auto& a = (*points)[left];
        const auto& b = (*points)[right];
        (*points)[i].heading_rad = std::atan2(b.y_m - a.y_m, b.x_m - a.x_m);
        if (i > 0 && i + 1 < points->size()) {
            (*points)[i].curvature_m_inv =
                curvatureFromThreePoints((*points)[i - 1], (*points)[i], (*points)[i + 1]);
        }
    }
}

}  // namespace

FusedReferenceOutput FusedReferenceBuilder::build(
    const VisualReferenceOutput& visual,
    const NavigationReferenceOutput& navigation,
    const FusedReferenceOutput* previous_reference) const {
    (void)previous_reference;
    FusedReferenceOutput result;
    if (!visual.ok || visual.points.size() < 2) {
        result.error = "invalid_visual_reference";
        return result;
    }

    const double visual_start_x = visual.points.front().x_m;
    const double visual_end_x = visual.points.back().x_m;
    const double visual_length = visual_end_x - visual_start_x;
    result.visual_end_x_m = visual_end_x;
    auto build_visual_only = [&](const std::string& reason) {
        result.ok = true;
        result.method = "visual_only";
        result.error = reason;
        result.confidence = visual.confidence;
        result.used_navigation = false;
        result.fused_start_x_m = visual_start_x;
        result.fused_end_x_m = visual_end_x;
        appendVisualSamples(visual, visual_start_x, visual_end_x, cfg_.sample_step_m, &result.points);
        updateGeometry(&result.points);
    };

    if (visual_length < cfg_.min_visual_length_m) {
        result.error = "visual_reference_too_short";
        return result;
    }
    if (!navigation.ok || navigation.vcs_points.size() < 2) {
        build_visual_only(navigation.error.empty() ? "navigation_reference_unavailable" : navigation.error);
        return result;
    }

    const auto& nav_points = navigation.vcs_points;
    const double nav_start_x = nav_points.front().x_m;
    const double nav_end_x = nav_points.back().x_m;
    const double overlap_start = std::max(visual_start_x, nav_start_x);
    const double overlap_end = std::min(visual_end_x, nav_end_x);
    const double overlap = overlap_end - overlap_start;
    result.overlap_length_m = std::max(0.0, overlap);
    result.heading_error_rad = navigation.heading_error_rad;
    if (overlap < cfg_.min_navigation_overlap_m) {
        build_visual_only("navigation_visual_overlap_too_short");
        return result;
    }
    if (navigation.heading_error_rad > cfg_.max_heading_error_rad) {
        build_visual_only("navigation_heading_mismatch");
        return result;
    }

    std::vector<double> offsets;
    for (double x = overlap_start; x <= overlap_end + 1e-6; x += cfg_.sample_step_m) {
        double visual_y = 0.0;
        double nav_y = 0.0;
        if (visualYAtX(visual, x, &visual_y) && interpolateYByX(nav_points, x, &nav_y)) {
            offsets.push_back(visual_y - nav_y);
        }
    }
    if (offsets.size() < 3) {
        build_visual_only("navigation_alignment_insufficient_samples");
        return result;
    }
    result.lateral_offset_m = median(offsets);
    if (std::abs(result.lateral_offset_m) > cfg_.max_lateral_offset_m) {
        build_visual_only("navigation_lateral_offset_too_large");
        return result;
    }

    std::vector<FusedReferencePoint> fused_points;
    if (!appendVisualSamples(visual, visual_start_x, visual_end_x, cfg_.sample_step_m, &fused_points)) {
        result.error = "fused_reference_empty";
        return result;
    }

    const auto nav_cumulative = cumulativeDistances(nav_points);
    const auto nav_headings = navigationHeadingProfile(nav_points);
    if (nav_cumulative.size() < 2 || nav_headings.size() != nav_cumulative.size()) {
        build_visual_only("navigation_heading_profile_unavailable");
        return result;
    }

    const double nav_length = nav_cumulative.back();
    const double nav_anchor_s = projectXToNavigationS(nav_points, nav_cumulative, visual_end_x);
    const double remaining_nav_length = std::max(0.0, nav_length - nav_anchor_s);
    const double extension_length = clamp(
        std::min(navigation.forward_length_m, remaining_nav_length), 0.0, cfg_.max_extension_m);
    if (extension_length <= cfg_.sample_step_m) {
        build_visual_only("navigation_extension_too_short");
        return result;
    }

    const FusedReferencePoint visual_end = fused_points.back();
    const double visual_heading = headingAtX(visual.points, visual_end.x_m);
    const double candidate_start_heading = headingAtDistance(nav_cumulative, nav_headings, nav_anchor_s);
    double previous_heading = visual_heading;
    double x = visual_end.x_m;
    double y = visual_end.y_m;
    double s = visual_end.s_m;
    double traveled = 0.0;
    while (traveled < extension_length - 1e-6) {
        const double ds = std::min(cfg_.sample_step_m, extension_length - traveled);
        const double query_s = nav_anchor_s + traveled + ds;
        const double candidate_heading = headingAtDistance(nav_cumulative, nav_headings, query_s);
        const double relative_heading = normalizeAngle(candidate_heading - candidate_start_heading);
        const double trend_weight = clamp(
            (traveled + ds) / std::max(1e-6, cfg_.navigation_trend_ramp_m), 0.0, 1.0);
        const double target_heading = visual_heading + clamp(
            relative_heading * trend_weight,
            -cfg_.max_heading_delta_from_visual_rad,
            cfg_.max_heading_delta_from_visual_rad);
        const double next_heading = previous_heading + clamp(
            normalizeAngle(target_heading - previous_heading),
            -cfg_.max_heading_step_rad,
            cfg_.max_heading_step_rad);
        const double mid_heading = previous_heading +
            0.5 * normalizeAngle(next_heading - previous_heading);
        x += std::cos(mid_heading) * ds;
        y += std::sin(mid_heading) * ds;
        s += ds;
        traveled += ds;
        previous_heading = normalizeAngle(next_heading);
        fused_points.push_back({
            s, x, y, previous_heading, 0.0,
            "visual_endpoint_integrated_navigation_trend"
        });
    }

    result.ok = true;
    result.method = "visual_anchor_navigation_trend_extension_v1";
    result.confidence = std::min(1.0, 0.6 * visual.confidence + 0.4);
    result.used_navigation = true;
    result.fused_start_x_m = visual_start_x;
    result.fused_end_x_m = fused_points.empty() ? visual_end_x : fused_points.back().x_m;
    result.points = std::move(fused_points);
    if (result.points.size() < 2) {
        result = {};
        result.error = "fused_reference_empty";
        return result;
    }
    updateGeometry(&result.points);
    return result;
}

}  // namespace topology_map::topology_v3
