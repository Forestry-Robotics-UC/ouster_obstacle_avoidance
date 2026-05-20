#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include "ouster_obstacle_avoidance/Types.hpp"

namespace ouster_obstacle_avoidance {

/**
 * VisualizationManager
 *
 * Owns all RViz publishers. Called once per frame after detection and
 * field computation. Nothing here affects behaviour — pure visualisation.
 *
 * Published topics:
 *   ~/obstacle_cloud   PointCloud2   obstacle points coloured by horiz distance
 *   ~/markers          MarkerArray   sector arcs, push arrow, safety rings
 *   ~/aggregated_scan  LaserScan     column-wise min scan (viz only)
 *   ~/state_marker     Marker        robot halo coloured by state
 */
class VisualizationManager {
public:

    struct Params {
        std::string frame_id        = "os_sensor";  // LiDAR frame

        // Distance ramp for point cloud colouring
        // Points beyond viz_max_dist are not published
        float d_stop                = 0.25f;
        float d_warn                = 0.80f;
        float viz_max_dist          = 2.00f;

        // Sector arc markers
        float sector_arc_height     = 0.05f;   // [m] above ground, for flat arcs
        float sector_arc_width      = 0.03f;   // [m] line width

        // Safety ring marker radii
        float ring_warn_radius      = 0.80f;
        float ring_stop_radius      = 0.25f;

        // State halo marker (sphere above robot)
        float halo_height           = 0.50f;   // [m] above LiDAR origin
        float halo_radius           = 0.20f;

        // Push arrow marker
        float arrow_shaft_diameter  = 0.05f;
        float arrow_head_diameter   = 0.10f;
        float arrow_length_scale    = 1.00f;   // scale push vector magnitude

        // Aggregated scan params (mirrors ObstacleDetector scan range)
        float scan_min_range        = 0.10f;
        float scan_max_range        = 2.00f;

        int   n_sectors             = 36;
    };

    VisualizationManager(ros::NodeHandle& nh, const Params& params);

    /**
     * publish — call once per frame with current detection results and state.
     */
    void publish(const std::vector<ObstaclePoint>& points,
                 const std::vector<SectorData>&    sectors,
                 const std::vector<float>&         aggregated_scan,
                 const PushResult&                 result,
                 AvoidanceState                    state,
                 const ros::Time&                  stamp);

private:
    Params         params_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_scan_;
    ros::Publisher pub_state_marker_;

    // Distance → RGBA colour ramp:
    //   > viz_max_dist : invisible
    //   d_warn..viz_max : dark grey → deep blue
    //   d_stop..d_warn  : blue → yellow → orange
    //   < d_stop        : bright red
    std_msgs::ColorRGBA distanceToColor(float horiz_dist) const;

    void publishObstacleCloud(const std::vector<ObstaclePoint>& points,
                              const ros::Time& stamp);

    void publishMarkers(const std::vector<SectorData>& sectors,
                        const PushResult& result,
                        AvoidanceState state,
                        const ros::Time& stamp);

    void publishAggregatedScan(const std::vector<float>& scan,
                               const ros::Time& stamp);

    void publishStateMarker(AvoidanceState state,
                            const ros::Time& stamp);

    // Helpers for building specific marker types
    visualization_msgs::Marker makeSafetyRing(float radius,
                                              const std_msgs::ColorRGBA& color,
                                              int id,
                                              const ros::Time& stamp) const;

    visualization_msgs::Marker makePushArrow(const PushResult& result,
                                             const ros::Time& stamp) const;

    visualization_msgs::Marker makeSectorArc(const SectorData& sector,
                                             int id,
                                             const ros::Time& stamp) const;
};

} // namespace ouster_obstacle_avoidance
