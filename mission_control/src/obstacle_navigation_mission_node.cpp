// LANDING NODE
#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk.h>
#include <std_msgs/UInt8.h>
#include <mission_control/droneuse_m100.h>
#include <mission_control/droneuse_gimbal.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace std;
using namespace DJI::onboardSDK;

typedef trajectory_msgs::MultiDOFJointTrajectory Trajectory;

/// State machine of the followme mission
//void performTask(const States &current);
droneuse_gimbal*    gimbal;
droneuse_m100*      m100;

ros::ServiceClient m100_attitude_control_service; 
dji_sdk::AttitudeControl m100_control_command;

const int control_rate = 20;

Trajectory local_path;
void local_planning_path(const Trajectory::ConstPtr& local_path_msg)
{
	local_path = *local_path_msg;
	return;
}

bool go_next_waypoint()
{
	bool wp_exists = true;
	geometry_msgs::PointStamped next_wp;

	if(local_path.points.empty())
	{
		cout << "No hay trayectoria local" << endl;
		wp_exists = false;
	}	
	else
	{
		next_wp.header = local_path.header;		
		for(int i=0; i<local_path.points.size(); i++)
		{
			next_wp.point = local_path.points[i].transforms[0].translation;	
			if(m100->distance_to_point(next_wp) > 0.4)
				break;
		}	
		m100->set_target_position(next_wp);
		//m100->set_target_rotation(local_path.points[i]);
	}
	return wp_exists;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    ros::Rate rate(control_rate);
    gimbal  = new droneuse_gimbal(nh);
    m100    = new droneuse_m100(nh);

     m100_attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");

    ros::Subscriber apriltag_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/tag_position",10, apriltag_subscriber_callback);
    
    // Wait for take off
	while(m100->get_flight_status() != 3)
    	{
        	rate.sleep();
    	}

    while(m100->get_sdk_control() != true)
    {
        rate.sleep();
    }

	//hover for 3 seconds
	m100->hover();
       	ros::Duration(3.0).sleep();
	
	//Get trajectory
	ros::spinOnce();

	//Go to the next waypoint
	while(ros::ok() && go_next_waypoint())
	{
		rate.sleep();
		ros::spinOnce();
	}

    m100->land();
    m100->disarm();

   return 0;
}    
