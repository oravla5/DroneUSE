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
    gimbal->set_target(apriltag_position_msg);
    m100->set_target_orientation(apriltag_position_msg);
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
		std::cout<< "Getting SDK Contro \n";
    while(m100->arm() != true);
		std::cout<< "Arming \n";
    m100->takeoff();
		std::cout<< "Takeoff \n";
    ros::Duration(10).sleep();

    ros::Time time_start = ros::Time::now();
    while(time_start > ros::Time::now() - ros::Duration(20))
    {
	std::cout<< "Following AprilTags \n";
        ros::spinOnce();
    }

    m100->land();
		std::cout<< "Landing \n";
    m100->disarm();
		std::cout<< "Disarming \n";
    while(m100->release_sdk_control() != true);
		std::cout<< "Release \n";

    return 0;
}

