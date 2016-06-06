#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <dji_sdk/dji_sdk.h>
#include <tf/transform_listener.h>


using namespace std;

bool landing = false;
geometry_msgs::PointStamped tag_position;

void local_position_callback(dji_sdk::LocalPosition msg)
{
    if(!landing && abs(msg.z) > 3.0)
    {
	ROS_INFO("Getting Posiiton");
        tag_position.header.frame_id = "/world";
        tag_position.header.stamp = ros::Time::now();
        tag_position.point.x = msg.x;
        tag_position.point.y = msg.y;
        tag_position.point.z = 0.0;
        landing = true;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_position_generator_node");
    ros::NodeHandle nh;
    int node_rate = 5;
    bool landing = false;
    ros::Rate rate(node_rate);
    
    ros::Publisher tag_positionPub = nh.advertise<geometry_msgs::PointStamped>("droneuse/landing_tag_position",1);
    ros::Subscriber local_position_subscriber = nh.subscribe<dji_sdk::LocalPosition>("dji_sdk/local_position",10,local_position_callback);
    double dt = 1.0/node_rate;
    
    double vel_x = 0.5;
    double vel_y = 0.0;
    double vel_z = 0.0;
    
	while(ros::ok())
	{
		if (landing)
		{
			ROS_INFO("Publishing...");
			tag_position.header.frame_id = "/world";
			tag_position.header.stamp = ros::Time::now();
			tag_position.point.x = tag_position.point.x + dt*vel_x;
			tag_position.point.y = tag_position.point.y + dt*vel_y;
			tag_position.point.z = tag_position.point.z + dt*vel_z;
			tag_positionPub.publish(tag_position);
		}
		rate.sleep();
		ros::spinOnce();
	}

    return 0;
}
