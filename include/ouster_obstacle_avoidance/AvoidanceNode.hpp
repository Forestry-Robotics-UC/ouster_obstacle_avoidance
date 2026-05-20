#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>

#include "ouster_obstacle_avoidance/Types.hpp"
#include "ouster_obstacle_avoidance/ObstacleDetector.hpp"
#include "ouster_obstacle_avoidance/PotentialField.hpp"
#include "ouster_obstacle_avoidance/VisualizationManager.hpp"

namespace ouster_obstacle_avoidance {

class AvoidanceNode {
public:
    explicit AvoidanceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void coastTimerCallback(const ros::TimerEvent&);
    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    void loadParams(ObstacleDetector::Params& det_params,
                    PotentialField::Params&   field_params,
                    VisualizationManager::Params& viz_params);

    void updateState(const PushResult& result);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher  pub_cmd_vel_;
    ros::Subscriber sub_points_;
    ros::Subscriber sub_goal_;

    std::unique_ptr<ObstacleDetector>     detector_;
    std::unique_ptr<PotentialField>       field_;
    std::unique_ptr<VisualizationManager> viz_;

    AvoidanceState state_           = AvoidanceState::NOMINAL;
    ros::Timer     coast_timer_;
    bool           geometry_ready_  = false;

    float  clear_dist_       = 1.10f;
    double coast_duration_   = 0.50;
    int    sync_queue_size_  = 10;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    geometry_msgs::PoseStamped last_goal_;
    bool has_goal_ = false;
};

} // namespace ouster_obstacle_avoidance
