#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <mission_control/droneuse_m100.h>
#include <mission_control/droneuse_gimbal.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace geometry_msgs;

ros::Duration		no_target_time;
ros::Time		last_target_time;
int 			control_rate = 10; 		
droneuse_gimbal*    	gimbal;
droneuse_m100*      	m100;
PointStamped 		default_gimbal_target;
PointStamped 		default_m100_target;
vector<PointStamped> 	m100_target;
vector<PointStamped> 	gimbal_target;
vector<PointStamped> 	waypoint;
tf::TransformListener* 	tf_listener;
// Functions
void 			gimbal_target_update();
void 			m100_target_update();
void 			landing_point_subscriber_callback(PointStamped landing_point_msg);
vector<PointStamped> 	generateWP();

// Mission States booleans
bool 			MISSION_ON 	= false;
bool 			TAKEOFF 	= false;
bool 			CHASE 		= false;
bool 			LANDING 	= false;


int main(int argc, char **argv)
{
    	ros::init(argc, argv, "tagMap_tracking_node");
    	ros::NodeHandle nh;
    	//ros::Subscriber langing_point_subscriber = nh.subscribe< PointStamped >("droneuse/landing_platform_position",1, landing_point_subscriber_callback);
	// Default Gimbal Orientation
	default_gimbal_target.header.frame_id = "/body_frame";
	default_gimbal_target.point.x = 6.0;
	default_gimbal_target.point.y = 0.0;
	default_gimbal_target.point.z = 6.0;

	default_m100_target.header.frame_id = "/body_frame";
	default_m100_target.point.x = 0.1;
	default_m100_target.point.y = 0.0;
	default_m100_target.point.z = 0.0;
	ros::Rate rate(control_rate);
	no_target_time = ros::Duration(2.0);
	last_target_time = ros::Time::now() + no_target_time;
    	gimbal  = new droneuse_gimbal(nh);
    	m100    = new droneuse_m100(nh);
	tf_listener = new tf::TransformListener;	

	waypoint = generateWP();


	ros::Duration(5.0).sleep();
std::cout << "frame wp 1" << waypoint[0].header.frame_id << endl;
	std::cout << "x " << waypoint[0].point.x << endl;
	std::cout << "y " << waypoint[0].point.y << endl;
	std::cout << "z " << waypoint[0].point.z << endl;
std::cout << "frame wp 2 " << waypoint[1].header.frame_id << endl;
	std::cout << "x " << waypoint[1].point.x << endl;
	std::cout << "y " << waypoint[1].point.y << endl;
	std::cout << "z " << waypoint[1].point.z << endl;

	// Hold until getting frames transformations
	bool init = false;
	while(!init)
	{
		try{
			ros::Time time_now = ros::Time::now();
			for(size_t j=0; j<waypoint.size(); j++)
			{
				waypoint[j].header.stamp = time_now;	
				tf_listener->waitForTransform("/world", waypoint[j].header.frame_id, waypoint[j].header.stamp, ros::Duration(2.0/control_rate));
				tf_listener->transformPoint("/world", waypoint[j].header.stamp, waypoint[j], waypoint[j].header.frame_id, waypoint[j]);
			}
			init = true;
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			init = false;
		}
		ros::spinOnce();
		rate.sleep();
	}

std::cout << "frame wp 1" << waypoint[0].header.frame_id << endl;
	std::cout << "x " << waypoint[0].point.x << endl;
	std::cout << "y " << waypoint[0].point.y << endl;
	std::cout << "z " << waypoint[0].point.z << endl;
std::cout << "frame wp 2 " << waypoint[1].header.frame_id << endl;
	std::cout << "x " << waypoint[1].point.x << endl;
	std::cout << "y " << waypoint[1].point.y << endl;
	std::cout << "z " << waypoint[1].point.z << endl;

	// Hold until taking off 
	// flight_status == 3 means drone has taken off
	while(m100->get_flight_status() != 3)
	{
		gimbal_target_update();
		ros::spinOnce();
		rate.sleep();
		std::cout << " waiting to take off" << std::endl;
	}

	// Get drone control and start the mission
std::cout << "getting control" << std::endl;
	if(m100->get_sdk_control())
	{
		MISSION_ON = true;
		TAKEOFF = true;
		while(ros::ok() && MISSION_ON)
		{	
			rate.sleep();
			ros::spinOnce();
			gimbal_target_update();
			m100_target_update();
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

std::vector<PointStamped> generateWP()
{
	std::vector<PointStamped> wp_list;
	PointStamped wp;
	wp.header.frame_id = "/body_frame";
	wp.header.stamp = ros::Time::now();
	// wp 1
	wp.point.x = 0.1;
	wp.point.y = 0.1;
	wp.point.z = -2.0;
	wp_list.push_back(wp);
	// wp 2
	wp.point.x = 2.0;
	wp.point.y = 0.1;
	wp.point.z = -2.0;
	wp_list.push_back(wp);
	return wp_list;
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
	if(CHASE)
	{
		if( (ros::Time::now() - last_target_time) > no_target_time )
		{
			cout << "hovering" << endl;
			m100->hover();
		}
		else if(!m100_target.empty())
		{
			if(m100->distance_to_position(m100_target[0]) < 0)
			{
				CHASE = false;
				LANDING = true;
			}
			else
			{
				m100_target[0].point.z = m100_target[0].point.z - 1.5;
				m100->set_target_position(m100_target[0]);

//				if( fabs(m100_target[0].point.x) < 0.1 && fabs(m100_target[0].point.y) < 0.1 )
//					m100->set_target_orientation(m100_target[0]);
				m100_target.clear();
			}
		}
	}

	if(LANDING)
	{
		if( (ros::Time::now() - last_target_time) > no_target_time )
		{
			m100->hover();
			CHASE = true;
			LANDING = false;
		}
		else if(!m100_target.empty())
		{
			if(m100->distance_to_position(m100_target[0]) < 0.2)
			{
				m100->land();	
				m100->disarm();
				MISSION_ON = false;
				CHASE = false;
				LANDING = false;
			}
			else
			{
				m100_target[0].point.z = m100_target[0].point.z - 0.15;
				m100->set_target_position(m100_target[0]);
				m100_target.clear();
			}
		}
	}

	if(TAKEOFF)
	{
		if( !waypoint.empty() )
		{
			if(m100->distance_to_position(waypoint[0]) > 0.4)
			{
				waypoint[0].header.stamp = ros::Time::now();
				m100->set_target_position(waypoint[0]);
				
				std::cout << "waypoint x " << waypoint[0].point.x << endl;
				std::cout << "waypoint y " << waypoint[0].point.y << endl;
				std::cout << "waypoint z " << waypoint[0].point.z << endl;
			}
			else
			{
				waypoint.erase(waypoint.begin());
			}
		}
		else 
		{
			m100->hover();		
			CHASE = true;
			TAKEOFF = false; 
		}
		
	}
 }
