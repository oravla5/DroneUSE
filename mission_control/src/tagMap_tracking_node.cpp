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

void landing_point_subscriber_callback(geometry_msgs::PointStamped landing_point_msg)
{
    gimbal->set_target(landing_point_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tagMap_tracking_node");
    ros::NodeHandle nh;
    gimbal  = new droneuse_gimbal(nh);
    m100    = new droneuse_m100(nh);
    ros::Subscriber langing_point_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/landing_platform_position",10, landing_point_subscriber_callback);
    if(m100->get_sdk_control())
    {
        ros::spin();
    }

    return 0;
}

