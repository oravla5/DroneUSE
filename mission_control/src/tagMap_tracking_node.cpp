#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <mission_control/droneuse_m100.h>
#include <mission_control/droneuse_gimbal.h>
#include <geometry_msgs/PointStamped.h>


using namespace std;

ros::Duration       no_target_time;
ros::Time		last_target_time;
droneuse_gimbal*    gimbal;
droneuse_m100*      m100;
std::vector<geometry_msgs::PointStamped> gimbal_target;

void landing_point_subscriber_callback(geometry_msgs::PointStamped landing_point_msg)
{
	gimbal_target.clear();
	gimbal_target.push_back(landing_point_msg);
//	gimbal->set_target(landing_point_msg);
//	std::cout << "target x " << landing_point_msg.point.x << std::endl;
//	std::cout << "target y " << landing_point_msg.point.y << std::endl;
//	std::cout << "target z " << landing_point_msg.point.z << std::endl;
	last_target_time = landing_point_msg.header.stamp;
//	std::cout << "time difference is: " << ros::Time::now() - last_target_time << std::endl;
}

int main(int argc, char **argv)
{
    	ros::init(argc, argv, "tagMap_tracking_node");
    	ros::NodeHandle nh;
	geometry_msgs::PointStamped default_point;	
	default_point.header.frame_id = "/body_frame";
	default_point.point.x = 10.0;
	default_point.point.y = 0.0;
	default_point.point.z = 2.0;
	ros::Rate rate(10);
	no_target_time = ros::Duration(2.0);
	last_target_time = ros::Time::now() + no_target_time;
    	gimbal  = new droneuse_gimbal(nh);
    	m100    = new droneuse_m100(nh);
    	ros::Subscriber langing_point_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/landing_platform_position",10, landing_point_subscriber_callback);
    	if(m100->get_sdk_control())
    	{
		while(ros::ok())
		{
			if((ros::Time::now() - last_target_time) > no_target_time)
			{
				default_point.header.stamp = ros::Time::now();
				gimbal->set_target(default_point);
			}
			else if(!gimbal_target.empty())
			{
				gimbal->set_target(gimbal_target[0]);
				gimbal_target.clear();
			}
			ros::spinOnce();
			rate.sleep();

		}	
    	}

    return 0;
}

