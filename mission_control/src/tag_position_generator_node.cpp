#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <dji_sdk/dji_sdk.h>
#include <tf/transform_listener.h>
#include <mission_control/droneuse_m100.h>


using namespace std;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_position_generator_node");
    ros::NodeHandle nh;
    int node_rate = 5;
    bool landing = false;
    ros::Rate rate(node_rate);
    
    ros::Publisher tag_positionPub = nh.advertise<geometry_msgs::PointStamped>("droneuse/landing_tag_position",1);
    double dt = 1.0/node_rate;
    
    droneuse_m100*      m100 = new droneuse_m100(nh);
    geometry_msgs::PointStamped tag_position;
    
    double vel_x = 1.0;
    double vel_y = 0.0;
    double vel_z = 0.0;
    
	while(ros::ok())
	{
		ROS_INFO("Loop cycle... landing flag = %d",landing);
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

        	if (!landing && m100->get_local_position().z>3.0)
        	{
            		landing = true;
            		tag_position.header.frame_id = "/world";
			tag_position.header.stamp = ros::Time::now();
			tag_position.point.x = m100->get_local_position().x + 1.0;
			tag_position.point.y = m100->get_local_position().y;
			tag_position.point.z = 0.5;
			tag_positionPub.publish(tag_position);

        	}
		rate.sleep();
		ros::spinOnce();
	}

    return 0;
}
