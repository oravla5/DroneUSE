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

bool follow_apriltag_enabled = false;

void apriltag_subscriber_callback(geometry_msgs::PointStamped apriltag_position_msg)
{
    gimbal->set_target(apriltag_position);
    m100->set_target_orientation(apriltag_position);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;

    //TODO put gimbal class inside m100;
    gimbal  = new droneuse_gimbal(nh);
    m100    = new droneuse_m100(nh);

    ros::Subscriber apriltag_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/tag_position",10, apriltag_subscriber_callback);

    while(m100->get_sdk_control() != true);
    m100->takeoff();
    ros::Duration(8).sleep();

    ros::Time time_start = ros::Time::now();
    while(time_start < ros::Time::now() + ros::Duration(20)
    {
        ros::spinOnce();
    }

    m100->land();
    m100->disarm();

    return 0;
}

