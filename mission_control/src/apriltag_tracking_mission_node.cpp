#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <mission_control/droneuse_m100.h>
#include <mission_control/droneuse_gimbal.h>
#include <geometry_msgs/PointStamped.h>


using namespace std;

//States nextState(const States &current);
//void performTask(const States &current);
//DJIDrone* drone;
//dji_sdk::LocalPosition target_position;
//dji_sdk::LocalPosition home_position;
//float dist2goal = 0.0;

//droneuse_m100* m100;
droneuse_gimbal* gimbal;
void apriltag_subscriber_callback(geometry_msgs::PointStamped apriltag_position_msg)
{
    gimbal->set_target_orientation(apriltag_position_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    gimbal = new droneuse_gimbal(nh);
    ros::Subscriber apriltag_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/tag_position",10, apriltag_subscriber_callback);
    ros::spin();
    return 0;
}

