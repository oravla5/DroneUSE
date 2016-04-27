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

	geometry_msgs::PointStamped home_waypoint;
	home_waypoint.point.x = 0.0;
	home_waypoint.point.y = 0.0;
	home_waypoint.point.z = 2.0;
	home_waypoint.header.frame_id = "/world";

	geometry_msgs::PointStamped waypoint1_waypoint;
	waypoint1_waypoint.point.x = 3.5;
	waypoint1_waypoint.point.y = 0.0;
	waypoint1_waypoint.point.z = 2.0;
	waypoint1_waypoint.header.frame_id = "/world";

	geometry_msgs::PointStamped land_waypoint;
	land_waypoint.point.x = 0.0;
	land_waypoint.point.y = 0.0;
	land_waypoint.point.z = 0.0;
	land_waypoint.header.frame_id = "/world";
	// Wait for take off
	while(m100->get_flight_status() != 3)
    {
		//TODO callbacks shoudlnt have to be updated in this node, services??
		ros::spinOnce();
        rate.sleep();
    }
	// Get SDK control
	while(m100->get_sdk_control() != true)
    {
		ros::spinOnce();
        rate.sleep();
    }
	
    // Ascend
	home_waypoint.header.stamp = ros::Time::now();
	m100->set_target_position(home_waypoint);

	// Wait to ascend
	//TODO points without stamp
	bool ascending = true;
	while(ascending)
	{
		ros::spinOnce();
		home_waypoint.header.stamp = ros::Time::now();
		if(m100->distance_to_position(home_waypoint) < 0.2)
			ascending = false;
        rate.sleep();
	}


	// Go to first waypoint
	waypoint1_waypoint.header.stamp = ros::Time::now();
    m100->set_target_position(waypoint1_waypoint);
    //m100->set_target_orientation(waypoint1_waypoint);
    

	//TODO points without stamp
	bool flying = true;
	while(flying)
	{
		ros::spinOnce();
		waypoint1_waypoint.header.stamp = ros::Time::now();
		if(m100->distance_to_position(waypoint1_waypoint) < 0.2)
			flying = false;
        rate.sleep();
	}

    // Return to home
	home_waypoint.header.stamp = ros::Time::now();
	m100->set_target_position(home_waypoint);
    //m100->set_target_orientation(home_waypoint);
	flying = true;
	while(flying)
	{
		ros::spinOnce();
		home_waypoint.header.stamp = ros::Time::now();
		if(m100->distance_to_position(home_waypoint) < 0.2)
			flying = false;
        rate.sleep();
	}


    // Wait to descend
    bool descending = true;
    land_waypoint.header.stamp = ros::Time::now();
	m100->set_target_position(land_waypoint);
    while(descending)
    {
        ros::spinOnce();
		home_waypoint.header.stamp = ros::Time::now();
		if(m100->distance_to_position(land_waypoint) < 0.5)
			descending = false;
        rate.sleep();

    }
    
    //Land
    m100->land();
    m100->disarm();

    while(m100->release_sdk_control() != true)
    {
		ros::spinOnce();
        rate.sleep();
    }

	return 0;
}
