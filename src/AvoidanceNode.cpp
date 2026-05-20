#include "ouster_obstacle_avoidance/AvoidanceNode.hpp"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <json/json.h>
#include <cmath>

namespace ouster_obstacle_avoidance {

// ─────────────────────────────────────────────────────────────────────────────
//  Construction
// ─────────────────────────────────────────────────────────────────────────────

AvoidanceNode::AvoidanceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_)
{
    // Load config
    ObstacleDetector::Params     det_params;
    PotentialField::Params       field_params;
    VisualizationManager::Params viz_params;
    loadParams(det_params, field_params, viz_params);

    // Init components
    detector_ = std::make_unique<ObstacleDetector>(det_params);
    field_    = std::make_unique<PotentialField>(field_params);
    viz_      = std::make_unique<VisualizationManager>(pnh_, viz_params);

    // ── ROS setup ────────────────────────────────────────────────────────────

    std::string cmd_vel_topic;
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic, "avoidance/cmd_vel");
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

    std::string points_topic;
    pnh_.param<std::string>("points_topic", points_topic, "/ouster/points");
    
    sub_points_ = nh_.subscribe(points_topic, sync_queue_size_, &AvoidanceNode::pointsCallback, this);

    std::string goal_topic;
    pnh_.param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
    sub_goal_ = nh_.subscribe(goal_topic, 1, &AvoidanceNode::goalCallback, this);

    // Timer for zero-vel coast pulse
    coast_timer_ = nh_.createTimer(ros::Duration(coast_duration_),
                                   &AvoidanceNode::coastTimerCallback,
                                   this,
                                   true,   // oneshot
                                   false); // autostart off

    ROS_INFO("[AvoidanceNode] Initialised. Listening to pointclouds on %s, goal on %s.", points_topic.c_str(), goal_topic.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Goal Callback
// ─────────────────────────────────────────────────────────────────────────────

void AvoidanceNode::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    last_goal_ = *msg;
    has_goal_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main points callback — runs every LiDAR frame
// ─────────────────────────────────────────────────────────────────────────────

void AvoidanceNode::pointsCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // ── Detection ─────────────────────────────────────────────────────────────
    std::vector<ObstaclePoint> points;
    std::vector<SectorData>    sectors;
    detector_->process(*cloud_msg, points, sectors);

    // ── Optional Goal Transformation ──────────────────────────────────────────
    std::optional<Eigen::Vector2f> local_goal;
    if (has_goal_) {
        try {
            geometry_msgs::PoseStamped target_time_goal = last_goal_;

            target_time_goal.header.stamp = ros::Time(0); // Use latest available transform

            geometry_msgs::PoseStamped transformed_goal;
            tf_buffer_.transform(target_time_goal, transformed_goal, cloud_msg->header.frame_id, ros::Duration(0.1));
            
            local_goal = Eigen::Vector2f(
                transformed_goal.pose.position.x,
                transformed_goal.pose.position.y
            );
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(2.0, "[AvoidanceNode] Could not transform goal: %s", ex.what());
        }
    }

    // ── Potential field ───────────────────────────────────────────────────────
    const PushResult result = field_->compute(sectors, local_goal);

    // ── State machine ─────────────────────────────────────────────────────────
    updateState(result);

    // ── cmd_vel output ────────────────────────────────────────────────────────
    if (state_ == AvoidanceState::AVOIDING)
        pub_cmd_vel_.publish(result.cmd_vel);
    // COASTING: zero twist published by coastTimerCallback
    // NOMINAL:  silent — twist_mux falls through to the controller

    // ── Visualisation ─────────────────────────────────────────────────────────
    viz_->publish(points, sectors,
                  detector_->getAggregatedScan(),
                  result, state_, cloud_msg->header.stamp);
}

// ─────────────────────────────────────────────────────────────────────────────
//  State machine
// ─────────────────────────────────────────────────────────────────────────────

void AvoidanceNode::updateState(const PushResult& result)
{
    switch (state_)
    {
        case AvoidanceState::NOMINAL:
            if (result.frontal_clearance < 1.0f || result.trapped)
            {
                state_ = AvoidanceState::AVOIDING;
                ROS_INFO_THROTTLE(2.0, "[AvoidanceNode] → AVOIDING");
            }
            break;

        case AvoidanceState::AVOIDING:
            // Exit when the field has fully relaxed (urgency back to zero)
            if (result.urgency < 0.01f && !result.trapped)
            {
                state_ = AvoidanceState::COASTING;
                ROS_INFO_THROTTLE(2.0, "[AvoidanceNode] → COASTING");

                // Publish zero vel for coast_duration, then go silent
                coast_timer_ = nh_.createTimer(
                    ros::Duration(coast_duration_),
                    &AvoidanceNode::coastTimerCallback, this,
                    /*oneshot=*/true);

                // Publish the zero twist immediately
                pub_cmd_vel_.publish(geometry_msgs::Twist{});
            }
            break;

        case AvoidanceState::COASTING:
            // If a new obstacle appears during coast, snap back to AVOIDING
            if (result.frontal_clearance < 1.0f || result.trapped)
            {
                coast_timer_.stop();
                state_ = AvoidanceState::AVOIDING;
                ROS_INFO_THROTTLE(2.0, "[AvoidanceNode] Coast interrupted → AVOIDING");
            }
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Coasting timer — fires once after coast_duration
// ─────────────────────────────────────────────────────────────────────────────

void AvoidanceNode::coastTimerCallback(const ros::TimerEvent&)
{
    state_ = AvoidanceState::NOMINAL;
    ROS_INFO_THROTTLE(2.0, "[AvoidanceNode] → NOMINAL (controller resumed)");
    // From here the node publishes nothing; twist_mux hands back to controller.
}

// ─────────────────────────────────────────────────────────────────────────────
//  Parameter loading
// ─────────────────────────────────────────────────────────────────────────────

void AvoidanceNode::loadParams(
    ObstacleDetector::Params&    dp,
    PotentialField::Params&      fp,
    VisualizationManager::Params& vp)
{
    // ── Detector ──────────────────────────────────────────────────────────────
    pnh_.param("detector/n_rows",             dp.n_rows,             128);
    pnh_.param("detector/n_cols",             dp.n_cols,             1024);
    pnh_.param("detector/max_horiz_dist",     dp.max_horiz_dist,     1.00f);
    pnh_.param("detector/min_z_lidar",        dp.min_z_lidar,        -0.10f);
    pnh_.param("detector/max_z_lidar",        dp.max_z_lidar,        0.15f);
    pnh_.param("detector/range_scale",        dp.range_scale,        0.001f);
    pnh_.param("detector/n_sectors",          dp.n_sectors,          36);
    pnh_.param("detector/scan_row_min",       dp.scan_row_min,       40);
    pnh_.param("detector/scan_row_max",       dp.scan_row_max,       88);
    pnh_.param("detector/nearir_veg_threshold",dp.nearir_veg_threshold, 800.0f);
    pnh_.param("detector/use_nearir_filter",  dp.use_nearir_filter,  false);

    // ── Potential field ───────────────────────────────────────────────────────
    pnh_.param("field/d_warn",                fp.d_warn,             0.80f);
    pnh_.param("field/d_stop",                fp.d_stop,             0.25f);
    pnh_.param("field/d_max",                 fp.d_max,              1.00f);
    pnh_.param("field/k_rep",                 fp.k_rep,              1.00f);
    pnh_.param("field/k_att",                 fp.k_att,              1.00f);
    pnh_.param("field/v_max",                 fp.v_max,              0.40f);
    pnh_.param("field/omega_max",             fp.omega_max,          0.80f);
    pnh_.param("field/k_ang",                 fp.k_ang,              1.20f);
    pnh_.param("field/k_boost",               fp.k_boost,            0.60f);
    pnh_.param("field/k_escape",              fp.k_escape,           0.40f);
    pnh_.param("field/k_front_weight",        fp.k_front_weight,     1.50f);
    pnh_.param("field/front_half_angle",      fp.front_half_angle,   1.047f);
    pnh_.param("field/k_omega_speed_scale",   fp.k_omega_speed_scale,0.50f);
    pnh_.param("field/f_cancel_threshold",    fp.f_cancel_threshold, 0.10f);
    pnh_.param("field/trapped_sector_ratio",  fp.trapped_sector_ratio,0.75f);

    // ── Visualization ─────────────────────────────────────────────────────────
    pnh_.param<std::string>("viz/frame_id",   vp.frame_id,           "os_sensor");
    
    // Default viz configs to field configs to ensure consistency
    pnh_.param("viz/viz_max_dist",            vp.viz_max_dist,       fp.d_max);
    pnh_.param("viz/ring_warn_radius",        vp.ring_warn_radius,   fp.d_warn);
    pnh_.param("viz/ring_stop_radius",        vp.ring_stop_radius,   fp.d_stop);
    
    pnh_.param("viz/halo_height",             vp.halo_height,        0.50f);
    pnh_.param("viz/halo_radius",             vp.halo_radius,        0.20f);
    pnh_.param("viz/sector_arc_height",       vp.sector_arc_height,  0.05f);
    pnh_.param("viz/sector_arc_width",        vp.sector_arc_width,   0.03f);
    pnh_.param("viz/arrow_shaft_diameter",    vp.arrow_shaft_diameter,0.05f);
    pnh_.param("viz/arrow_head_diameter",     vp.arrow_head_diameter, 0.10f);
    pnh_.param("viz/arrow_length_scale",      vp.arrow_length_scale,  1.00f);
    pnh_.param("viz/scan_min_range",          vp.scan_min_range,      0.10f);
    pnh_.param("viz/scan_max_range",          vp.scan_max_range,      2.00f);

    pnh_.param("clear_dist",             clear_dist_,       fp.d_max + 0.1f);
    pnh_.param("coast_duration",         coast_duration_,   0.50);
    pnh_.param("sync_queue_size",        sync_queue_size_,  10);

    // Overwrite the detector max_horiz_dist to match viz or field logic
    // This establishes the consistency the user asked for:
    dp.max_horiz_dist = fp.d_max;

    ROS_INFO("[AvoidanceNode] Parameters loaded.");
}

} // namespace ouster_obstacle_avoidance
