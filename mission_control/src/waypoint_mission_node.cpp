#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk.h>
#include <std_msgs/UInt8.h>
#include <mission_control/droneuse_m100.h>
#include <mission_control/droneuse_gimbal.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>


using namespace std;
using namespace DJI::onboardSDK;

droneuse_gimbal*    gimbal;
droneuse_m100*      m100;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    ros::Rate rate(15);

    gimbal  = new droneuse_gimbal(nh);
    m100    = new droneuse_m100(nh);

	// Wait for take off
	while(m100->get_flight_status() != 3)
		//TODO callbacks shoudlnt have to be updated in this node, services??
		ros::spinOnce();

	// Get SDK control
	while(m100->get_sdk_control() != true)
		ros::spinOnce();

	// Ascend
	geometry_msgs::PointStamped takeoff_waypoint;
	takeoff_waypoint.point.x = 0.0;
	takeoff_waypoint.point.y = 0.0;
	takeoff_waypoint.point.z = 2.0;
	takeoff_waypoint.header.frame_id = "/world";
	takeoff_waypoint.header.stamp = ros::Time::now();

	m100->set_target_position(takeoff_waypoint);

	// Wait to ascend
	//TODO points without stamp
	bool ascending = true;
	while(ascending)
	{
		takeoff_waypoint.header.stamp = ros::Time::now();
		if(m100->distance_to_position(takeoff_waypoint) < 0.5)
			ascending = false;
		ros::spinOnce();
	}


	// Go to first waypoint
	geometry_msgs::PointStamped waypoint1_waypoint;
	waypoint1_waypoint.point.x = 2.0;
	waypoint1_waypoint.point.y = 0.0;
	waypoint1_waypoint.point.z = 2.0;
	waypoint1_waypoint.header.frame_id = "/world";
	waypoint1_waypoint.header.stamp = ros::Time::now();

	m100->set_target_position(waypoint1_waypoint);

	// Wait to ascend
	//TODO points without stamp
	bool flying = true;
	while(flying)
	{
		waypoint1_waypoint.header.stamp = ros::Time::now();
		if(m100->distance_to_position(waypoint1_waypoint) < 0.5)
			flying = false;
		ros::spinOnce();
	}


	return 0;
}
