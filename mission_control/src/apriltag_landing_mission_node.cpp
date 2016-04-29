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

using namespace std;
using namespace DJI::onboardSDK;

/// State machine of the followme mission
//void performTask(const States &current);
droneuse_gimbal*    gimbal;
droneuse_m100*      m100;
bool follow_april_flag = false;
ros::ServiceClient m100_attitude_control_service; 
dji_sdk::AttitudeControl m100_control_command;
unsigned char control_flag;
ros::Time last_apriltag_time;
geometry_msgs::PointStamped last_apriltag_position;
tf::TransformListener* tf_listener;
uint8_t flight_status;

bool april_detection = false;

int control_rate = 20;

void apriltag_subscriber_callback(geometry_msgs::PointStamped apriltag_position_msg)
{
	last_apriltag_time = apriltag_position_msg.header.stamp;
	gimbal->set_target(apriltag_position_msg);
    april_detection = true;
    try
    {
	    tf_listener->waitForTransform("/ground_frame",apriltag_position_msg.header.frame_id, apriltag_position_msg.header.stamp, ros::Duration(2.0/15));
	    tf_listener->transformPoint("/ground_frame", apriltag_position_msg.header.stamp, apriltag_position_msg,apriltag_position_msg.header.frame_id ,last_apriltag_position);	

    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s ERROR EN APRILTAG", ex.what());
    }
}

void flight_status_subscriber_callback(std_msgs::UInt8 flight_status_msg)
{
    flight_status = flight_status_msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    ros::Rate rate(control_rate);
    gimbal  = new droneuse_gimbal(nh);
    m100    = new droneuse_m100(nh);
    tf_listener = new tf::TransformListener;
    
    geometry_msgs::PointStamped home_waypoint;
    home_waypoint.header.frame_id = "/world";
    home_waypoint.header.stamp = ros::Time::now();
    home_waypoint.point.x = 0.0;
    home_waypoint.point.y = 0.0;
    home_waypoint.point.z = 4.0;

    geometry_msgs::PointStamped default_gimbal;
    default_gimbal.header.frame_id = "/body_frame";
    default_gimbal.point.x = 2.0;
    default_gimbal.point.z = 10.0;
    default_gimbal.point.y = 0.0;

    
	//geometry_msgs::PointStamped waypoint1_waypoint;
	//waypoint1_waypoint.point.x = 2.0;
	//waypoint1_waypoint.point.y = 0.0;
	//waypoint1_waypoint.point.z = 2.0;
	//waypoint1_waypoint.header.frame_id = "/world";
    
    // Position where the apriltag is setted 
   
     //m100_attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");

    ros::Subscriber apriltag_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/tag_position",10, apriltag_subscriber_callback);
    ros::Subscriber flight_status_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/flight_status", 10, flight_status_subscriber_callback);
    // Wait for take off
	while(flight_status != 3)
    {
		//TODO callbacks shoudlnt have to be updated in this node, services??
		ros::spinOnce();
        rate.sleep();
    }

    while(m100->get_sdk_control() != true)
    {
		ros::spinOnce();
        rate.sleep();
    }

    default_gimbal.header.stamp = ros::Time::now();
    gimbal->set_target(default_gimbal);

	home_waypoint.header.stamp = ros::Time::now();
	m100->set_target_position(home_waypoint);
	
    bool ascending = true;
	while(!april_detection)
	{
		ros::spinOnce();
		rate.sleep();
	}
    
    float dist2goal; 
    float altitude2goal;

    do{
	ros::spinOnce();
        dist2goal = sqrt(last_apriltag_position.point.x*last_apriltag_position.point.x + last_apriltag_position.point.y*last_apriltag_position.point.y);
        if(last_apriltag_position.point.z != 0)
            altitude2goal = last_apriltag_position.point.z;
        last_apriltag_position.point.z = 0;
        m100->set_target_position(last_apriltag_position);

        rate.sleep();
    }while(dist2goal >0.5);

    do{
		ros::spinOnce();
        altitude2goal = last_apriltag_position.point.z;
        m100->set_target_position(last_apriltag_position);

        rate.sleep();
    }while(altitude2goal > 0.5);

    m100->land();

    m100->disarm();

    m100->release_sdk_control();

    return 0;
}    

    

/*
    while(ros::ok() && currentState != FINISHED)
    {
      /// Execute state machine.
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void performTask(const States &current)
{
   switch(current)
   {
        case INIT_MISSION : 
            break;

        case TAKEOFF :
            if(!(flight_status == 3))
                m100->takeoff();
            else
            {
                m100->set_target_position(takeoff_goal);
                body_origin.header.stamp = time_now;
                tf_listener->waitForTransform("/world", "/body_frame", time_now , ros::Duration(2.0/control_rate));
                tf_listener->transformPoint("/world", time_now, body_origin, "/body_frame", local_position);
                height = abs(local_position.point.z);
            }
            break;

        case SEARCH :
            if(!april_detection || ((ros::Time::now() - last_apriltag_time) > 1))
            {
                target_gimbal_orientation.frame_id = "/body_frame";
                target_gimbal.header.stamp = ros::Time::now();
                target_gimbal.point.x = 1.0;
                target_gimbal.point.z = 1.0;
                target_gimbal.point.x = 0.0;
                gimbal->set_target(target_gimbal);

                target_position.header.frame_id = "/world";
                target_position.header.stamp = ros::Time::now();
                target_position.point.x = 3.0;
                target_position.point.y = 0.0;
                target_position.point.z = -2.0;
                m100->set_target_position(target_position);
                m100->set_target_orientation(target_position);
                april_detection = false;
            }
            else
            {
                target_gimbal_orientation.
                tf_listener->waitForTransform("/world", last_apriltag_position.header.frame_id, last_apriltag_time , ros::Duration(2.0/control_rate));
                tf_listener->transformPoint("/world", last_apriltag_time, last_apriltag_position, "/world", target_position);
                target_position.point.z = target_position.point.z - 2.0;
                m100->set_target_position(target_position);
                m100->set_target_orientation(target_position);
            }

            tf_listener->waitForTransform("/body_frame", target_position.header.frame_id, target_position.header.stamp , ros::Duration(2.0/control_rate));
            tf_listener->transformPoint("/body_frame", target_position.header.stamp, , "/world", target_position);


            dist2goal = sqrt((target_position.point.x))
            break;

        case LAND :
            if(follow_april_flag)
                follow_april_flag =  false;
            m100->land();
            break;

        case FINISHED :
            m100->disarm();

      }
   }

*/
