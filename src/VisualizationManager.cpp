#include "ouster_obstacle_avoidance/VisualizationManager.hpp"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <algorithm>

namespace ouster_obstacle_avoidance {

// ─────────────────────────────────────────────────────────────────────────────
//  Construction & publisher setup
// ─────────────────────────────────────────────────────────────────────────────

VisualizationManager::VisualizationManager(ros::NodeHandle& nh,
                                           const Params& params)
    : params_(params)
{
    pub_cloud_        = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1);
    pub_markers_      = nh.advertise<visualization_msgs::MarkerArray>("markers", 1);
    pub_scan_         = nh.advertise<sensor_msgs::LaserScan>("aggregated_scan", 1);
    pub_state_marker_ = nh.advertise<visualization_msgs::Marker>("state_marker", 1);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main publish — called once per frame
// ─────────────────────────────────────────────────────────────────────────────

void VisualizationManager::publish(
    const std::vector<ObstaclePoint>& points,
    const std::vector<SectorData>&    sectors,
    const std::vector<float>&         aggregated_scan,
    const PushResult&                 result,
    AvoidanceState                    state,
    const ros::Time&                  stamp)
{
    publishObstacleCloud(points, stamp);
    publishMarkers(sectors, result, state, stamp);
    publishAggregatedScan(aggregated_scan, stamp);
    publishStateMarker(state, stamp);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Colour ramp
//  Distance → RGBA: far=dark grey → close=bright red
// ─────────────────────────────────────────────────────────────────────────────

std_msgs::ColorRGBA VisualizationManager::distanceToColor(float d) const
{
    std_msgs::ColorRGBA c;
    c.a = 1.0f;

    if (d >= params_.viz_max_dist) { c.a = 0.0f; return c; }

    // t = 0 at viz_max_dist (far/safe), t = 1 at d_stop (critical)
    const float t = std::clamp(
        1.0f - (d - params_.d_stop) / (params_.viz_max_dist - params_.d_stop),
        0.0f, 1.0f);

    if (t < 0.30f) {
        // Dark grey → deep blue
        const float s = t / 0.30f;
        c.r = 0.15f * (1.0f - s);
        c.g = 0.15f * (1.0f - s);
        c.b = 0.15f + 0.55f * s;
    } else if (t < 0.55f) {
        // Deep blue → cyan
        const float s = (t - 0.30f) / 0.25f;
        c.r = 0.0f;
        c.g = 0.6f * s;
        c.b = 0.70f + 0.30f * (1.0f - s);
    } else if (t < 0.75f) {
        // Cyan → yellow
        const float s = (t - 0.55f) / 0.20f;
        c.r = s;
        c.g = 0.6f + 0.4f * (1.0f - s) * s;
        c.b = 1.0f - s;
    } else if (t < 0.90f) {
        // Yellow → orange
        const float s = (t - 0.75f) / 0.15f;
        c.r = 1.0f;
        c.g = 1.0f - 0.65f * s;
        c.b = 0.0f;
    } else {
        // Orange → bright red
        const float s = (t - 0.90f) / 0.10f;
        c.r = 1.0f;
        c.g = 0.35f * (1.0f - s);
        c.b = 0.0f;
    }

    // Fade in opacity: barely visible when far, fully opaque when close
    c.a = 0.30f + 0.70f * t;
    return c;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Obstacle cloud — XYZRGB PointCloud2
// ─────────────────────────────────────────────────────────────────────────────

void VisualizationManager::publishObstacleCloud(
    const std::vector<ObstaclePoint>& points,
    const ros::Time& stamp)
{
    if (pub_cloud_.getNumSubscribers() == 0) return;

    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp    = stamp;
    cloud.header.frame_id = params_.frame_id;
    cloud.height          = 1;
    cloud.width           = static_cast<uint32_t>(points.size());
    cloud.is_dense        = false;
    cloud.is_bigendian    = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float>   iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>   iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>   iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud, "rgb");

    for (const auto& pt : points)
    {
        const auto col = distanceToColor(pt.horiz_dist);

        *iter_x = pt.x;
        *iter_y = pt.y;
        *iter_z = pt.z;

        // Pack RGB into the single float field (rviz convention: B G R padding)
        iter_rgb[0] = static_cast<uint8_t>(col.b * 255.0f);
        iter_rgb[1] = static_cast<uint8_t>(col.g * 255.0f);
        iter_rgb[2] = static_cast<uint8_t>(col.r * 255.0f);
        iter_rgb[3] = static_cast<uint8_t>(col.a * 255.0f);

        ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
    }

    pub_cloud_.publish(cloud);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Aggregated LaserScan (column-wise min)
// ─────────────────────────────────────────────────────────────────────────────

void VisualizationManager::publishAggregatedScan(
    const std::vector<float>& scan,
    const ros::Time& stamp)
{
    if (pub_scan_.getNumSubscribers() == 0) return;
    if (scan.empty()) return;

    sensor_msgs::LaserScan msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = params_.frame_id;

    const int   n    = static_cast<int>(scan.size());
    const float two_pi = 2.0f * static_cast<float>(M_PI);

    msg.angle_min       =  0.0f;
    msg.angle_max       =  two_pi;
    msg.angle_increment =  two_pi / static_cast<float>(n);
    msg.time_increment  =  0.0f;
    msg.scan_time       =  0.1f;
    msg.range_min       =  params_.scan_min_range;
    msg.range_max       =  params_.scan_max_range;

    msg.ranges.assign(scan.begin(), scan.end());

    pub_scan_.publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  MarkerArray: safety rings, sector arcs, push arrow
// ─────────────────────────────────────────────────────────────────────────────

void VisualizationManager::publishMarkers(
    const std::vector<SectorData>& sectors,
    const PushResult& result,
    AvoidanceState state,
    const ros::Time& stamp)
{
    if (pub_markers_.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray arr;
    int id = 0;

    // ── Safety rings (warn + stop distance) ──────────────────────────────────
    {
        std_msgs::ColorRGBA warn_color; warn_color.r=1.0f; warn_color.g=0.8f;
                                        warn_color.b=0.0f; warn_color.a=0.5f;
        std_msgs::ColorRGBA stop_color; stop_color.r=1.0f; stop_color.g=0.0f;
                                        stop_color.b=0.0f; stop_color.a=0.6f;
        arr.markers.push_back(makeSafetyRing(params_.ring_warn_radius,
                                             warn_color, id++, stamp));
        arr.markers.push_back(makeSafetyRing(params_.ring_stop_radius,
                                             stop_color, id++, stamp));
    }

    // ── Per-sector arcs coloured by d_min ────────────────────────────────────
    for (const auto& s : sectors)
        arr.markers.push_back(makeSectorArc(s, id++, stamp));

    // ── Push arrow (only when actively avoiding) ─────────────────────────────
    if (state == AvoidanceState::AVOIDING)
        arr.markers.push_back(makePushArrow(result, stamp));
    else
    {
        // Publish a DELETE marker so the arrow disappears when state changes
        visualization_msgs::Marker del;
        del.header.stamp    = stamp;
        del.header.frame_id = params_.frame_id;
        del.action = visualization_msgs::Marker::DELETE;
        del.id     = 9999;
        del.ns     = "push_arrow";
        arr.markers.push_back(del);
    }

    pub_markers_.publish(arr);
}

// ─────────────────────────────────────────────────────────────────────────────
//  State halo — sphere above robot, green/red
// ─────────────────────────────────────────────────────────────────────────────

void VisualizationManager::publishStateMarker(
    AvoidanceState state, const ros::Time& stamp)
{
    if (pub_state_marker_.getNumSubscribers() == 0) return;

    visualization_msgs::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = params_.frame_id;
    m.ns     = "avoidance_state";
    m.id     = 0;
    m.type   = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = params_.halo_height;
    m.pose.orientation.w = 1.0;

    m.scale.x = m.scale.y = m.scale.z = params_.halo_radius * 2.0f;

    switch (state)
    {
        case AvoidanceState::NOMINAL:
            m.color.r = 0.0f; m.color.g = 0.9f;
            m.color.b = 0.2f; m.color.a = 0.5f;
            break;
        case AvoidanceState::AVOIDING:
            m.color.r = 1.0f; m.color.g = 0.1f;
            m.color.b = 0.0f; m.color.a = 0.8f;
            break;
        case AvoidanceState::COASTING:
            m.color.r = 1.0f; m.color.g = 0.6f;
            m.color.b = 0.0f; m.color.a = 0.6f;
            break;
    }

    pub_state_marker_.publish(m);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Marker helpers
// ─────────────────────────────────────────────────────────────────────────────

visualization_msgs::Marker VisualizationManager::makeSafetyRing(
    float radius,
    const std_msgs::ColorRGBA& color,
    int id,
    const ros::Time& stamp) const
{
    visualization_msgs::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = params_.frame_id;
    m.ns     = "safety_rings";
    m.id     = id;
    m.type   = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.02f;   // line width
    m.color   = color;
    m.pose.orientation.w = 1.0;

    // 64-point circle
    constexpr int kRingPts = 64;
    for (int i = 0; i <= kRingPts; ++i)
    {
        const float angle = 2.0f * static_cast<float>(M_PI)
                          * static_cast<float>(i) / static_cast<float>(kRingPts);
        geometry_msgs::Point p;
        p.x = radius * std::cos(angle);
        p.y = radius * std::sin(angle);
        p.z = params_.sector_arc_height;
        m.points.push_back(p);
    }

    return m;
}

visualization_msgs::Marker VisualizationManager::makePushArrow(
    const PushResult& result,
    const ros::Time& stamp) const
{
    visualization_msgs::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = params_.frame_id;
    m.ns     = "push_arrow";
    m.id     = 9999;
    m.type   = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;

    // Start at robot origin, end along push vector
    geometry_msgs::Point start, end;
    start.x = 0.0; start.y = 0.0; start.z = params_.sector_arc_height + 0.1f;
    end.x = result.push_vector.x() * params_.arrow_length_scale;
    end.y = result.push_vector.y() * params_.arrow_length_scale;
    end.z = start.z;

    m.points = {start, end};
    m.scale.x = params_.arrow_shaft_diameter;
    m.scale.y = params_.arrow_head_diameter;
    m.scale.z = params_.arrow_head_diameter;

    m.color.r = 1.0f; m.color.g = 1.0f;
    m.color.b = 1.0f; m.color.a = 0.9f;
    m.pose.orientation.w = 1.0;

    return m;
}

visualization_msgs::Marker VisualizationManager::makeSectorArc(
    const SectorData& sector,
    int id,
    const ros::Time& stamp) const
{
    visualization_msgs::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = params_.frame_id;
    m.ns     = "sector_arcs";
    m.id     = id;
    m.type   = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = params_.sector_arc_width;
    m.pose.orientation.w = 1.0;

    // Colour by sector's minimum distance
    m.color = distanceToColor(sector.d_min);

    if (!sector.occupied || m.color.a < 0.05f) {
        m.action = visualization_msgs::Marker::DELETE;
        return m;  // empty sector
    }

    // Draw a short arc at d_min within the sector angular range
    const float sector_width = 2.0f * static_cast<float>(M_PI)
                             / static_cast<float>(params_.n_sectors);
    const float half_w = sector_width * 0.5f;
    const float angle0 = sector.angle - half_w;
    const float angle1 = sector.angle + half_w;

    constexpr int kArcPts = 8;
    for (int i = 0; i <= kArcPts; ++i)
    {
        const float t = static_cast<float>(i) / static_cast<float>(kArcPts);
        const float a = angle0 + t * (angle1 - angle0);
        geometry_msgs::Point p;
        p.x = sector.d_min * std::cos(a);
        p.y = sector.d_min * std::sin(a);
        p.z = params_.sector_arc_height;
        m.points.push_back(p);
    }

    return m;
}

} // namespace ouster_obstacle_avoidance
