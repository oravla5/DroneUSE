#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <mission_control/droneuse_m100.h>
#include <mission_control/droneuse_gimbal.h>
#include <geometry_msgs/PointStamped.h>


using namespace std;

droneuse_gimbal*    gimbal;
droneuse_m100*      m100;

void apriltag_subscriber_callback(geometry_msgs::PointStamped apriltag_position_msg)
{
    gimbal->set_target_orientation(apriltag_position_msg);
    ROS_INFO("TAG RECEIVED");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    gimbal  = new droneuse_gimbal(nh);
    m100    = new droneuse_m100(nh);
    ros::Subscriber apriltag_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/tag_position",10, apriltag_subscriber_callback);
    if(m100->get_sdk_control())
    {
        ROS_INFO("PERMISSION ACQUIRED");
        ros::spin();
    }

    return 0;
}

