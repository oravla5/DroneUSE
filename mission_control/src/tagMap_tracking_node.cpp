#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <mission_control/droneuse_m100.h>
#include <mission_control/droneuse_gimbal.h>
#include <geometry_msgs/PointStamped.h>


using namespace std;
using namespace geometry_msgs;

ros::Duration		no_target_time;
ros::Time		last_target_time;
int 			control_rate = 10; 		
droneuse_gimbal*    	gimbal;
droneuse_m100*      	m100;
PointStamped 		default_gimbal_target;
vector<PointStamped> 	m100_target;
vector<PointStamped> 	gimbal_target;

// Functions
void 	gimbal_target_update();
void 	landing_point_subscriber_callback(PointStamped landing_point_msg);

// Mission States booleans
bool 			MISSION_ON 	= false;
bool			FLYING 		= false;
bool			GROUND 		= false;
bool 			TAKEOFF 	= false;
bool 			CHASE 		= false;
bool 			LANDING 	= false;


int main(int argc, char **argv)
{
    	ros::init(argc, argv, "tagMap_tracking_node");
    	ros::NodeHandle nh;
    	ros::Subscriber langing_point_subscriber = nh.subscribe< PointStamped >("droneuse/landing_platform_position",10, landing_point_subscriber_callback);
	// Default Gimbal Orientation
	default_gimbal_target.header.frame_id = "/body_frame";
	default_gimbal_target.point.x = 10.0;
	default_gimbal_target.point.y = 0.0;
	default_gimbal_target.point.z = 2.0;

	ros::Rate rate(control_rate);
	no_target_time = ros::Duration(2.0);
	last_target_time = ros::Time::now() + no_target_time;
    	gimbal  = new droneuse_gimbal(nh);
    	m100    = new droneuse_m100(nh);
	
	// Hold until taking off 
	// flight_status == 3 means drone has taken off
	while(m100->get_flight_status() != 3)
	{
		ros::spinOnce();
		rate.sleep();
	}
	FLYING = true;

	// Get drone control and start the mission
	if(m100->get_sdk_control())
	{
		MISSION_ON = true;
		while(ros::ok() && MISSION_ON)
		{	
			MISSION_ON = true;
			while(ros::ok() && MISSION_ON)
			{
				gimbal_target_update();
			}	
			ros::spinOnce();
			rate.sleep();
		}
	}

    return 0;
}

void landing_point_subscriber_callback(geometry_msgs::PointStamped landing_point_msg)
{
	m100_target.clear();
	m100_target.push_back(landing_point_msg);
	gimbal_target.clear();
	gimbal_target.push_back(landing_point_msg);
//	gimbal->set_target(landing_point_msg);
//	std::cout << "target x " << landing_point_msg.point.x << std::endl;
//	std::cout << "target y " << landing_point_msg.point.y << std::endl;
//	std::cout << "target z " << landing_point_msg.point.z << std::endl;
	last_target_time = landing_point_msg.header.stamp;
//	std::cout << "time difference is: " << ros::Time::now() - last_target_time << std::endl;
}

void gimbal_target_update()
{
	if((ros::Time::now() - last_target_time) > no_target_time)
	{
		default_gimbal_target.header.stamp = ros::Time::now();
		gimbal->set_target(default_gimbal_target);
	}
	else if(!gimbal_target.empty())
	{
		gimbal->set_target(gimbal_target[0]);
		gimbal_target.clear();
	}
}

void m100_target_update()
{
	if( ((ros::Time::now() - last_target_time) > no_target_time) && FLYING )
	{
		default_gimbal_target.header.stamp = ros::Time::now();
		m100->set_target_position(default_gimbal_target);
	}
	else if(!m100_target.empty())
	{
		m100->set_target_position(m100_target[0]);
		m100_target.clear();
	}
}
