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

int 			control_rate = 10; 		
droneuse_m100*      	m100;
vector<PointStamped> 	waypoint;
tf::TransformListener* 	tf_listener;
vector<PointStamped> 	generateWP();


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tagMap_tracking_node");
    ros::NodeHandle nh;
	// Default Gimbal Orientation

	ros::Rate rate(control_rate);
    m100    = new droneuse_m100(nh);
	tf_listener = new tf::TransformListener;	

	waypoint = generateWP();


	ros::Duration(5.0).sleep();

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

	// Hold until taking off 
    m100->wait_to_takeoff();
//    if(!m100->follow_waypoint(waypoint))
//        m100->hover(3.0);
	m100->hover(2.0);
	m100->custom_land();
    //if(!m100->perform_landing_maneuver())
    //    m100->hover(3.0);

    return 0;
}


std::vector<PointStamped> generateWP()
{
	std::vector<PointStamped> wp_list;
	PointStamped wp;
	wp.header.frame_id = "/body_frame";
	wp.header.stamp = ros::Time::now();
	// wp 1
	wp.point.x = 0.0;
	wp.point.y = 0.0;
	wp.point.z = -3.0;
	wp_list.push_back(wp);
	// wp 2
	wp.point.x = 3.0;
	wp.point.y = 0.0;
	wp.point.z = -3.0;
	wp_list.push_back(wp);
	// wp 3
	wp.point.x = 3.0;
	wp.point.y = 3.0;
	wp.point.z = -3.0;
	wp_list.push_back(wp);
	// wp 4
	wp.point.x = 0.0;
	wp.point.y = 3.0;
	wp.point.z = -3.0;
	wp_list.push_back(wp);
	// wp 5
	wp.point.x = 0.0;
	wp.point.y = 0.0;
	wp.point.z = -3.0;
	wp_list.push_back(wp);
	return wp_list;
}
