#include <ros/ros.h>
#include "ouster_obstacle_avoidance/AvoidanceNode.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ouster_obstacle_avoidance_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");   // private namespace — params live under node name

    ouster_obstacle_avoidance::AvoidanceNode node(nh, pnh);

    ros::spin();
    return 0;
}
