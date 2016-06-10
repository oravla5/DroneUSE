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
bool land_flag = false;

bool april_detection = false;
bool chase = false;
bool approach = false;

int control_rate = 20;

void landing_tag_subscriber_callback(geometry_msgs::PointStamped landing_tag_position_msg)
{
    if(follow_april_flag)
    {
        gimbal->set_target(landing_tag_position_msg);

        try{
            tf_listener->waitForTransform("/ground_frame",landing_tag_position_msg.header.frame_id, landing_tag_position_msg.header.stamp, ros::Duration(2.0/15));
            tf_listener->transformPoint("/ground_frame", landing_tag_position_msg.header.stamp, landing_tag_position_msg,landing_tag_position_msg.header.frame_id ,landing_tag_position_msg);	
            geometry_msgs::PointStamped target_position;
            tf::Vector3 target_position_vector;
            tf::Vector3 apriltag_position_vector (	landing_tag_position_msg.point.x,
                                        landing_tag_position_msg.point.y,
                                        landing_tag_position_msg.point.z);

            //target_position_vector = apriltag_position_vector - apriltag_position_vector.normalized()*2.5;
            target_position_vector = apriltag_position_vector;

            if(chase)
            {
                target_position.header.frame_id = landing_tag_position_msg.header.frame_id;
                target_position.header.stamp = landing_tag_position_msg.header.stamp;
                target_position.point.x = (float) landing_tag_position_msg.point.x;
                target_position.point.y = (float) landing_tag_position_msg.point.y;
                target_position.point.z = (float) landing_tag_position_msg.point.z - 2.5; 

                if(landing_tag_position_msg.point.x > m100->get_local_position().x)
                {
                    m100->set_target_position(target_position);
                    //m100->set_target_position(landing_tag_position_msg);

                    if(m100->distance_to_position(target_position) < 0.5)
                    //if(m100->distance_to_position(landing_tag_position_msg) < 2.5)
                    {
                        chase = false;
                        approach = true;
                    }
                }
            }
            if(approach)
            {
                target_position.header.frame_id = landing_tag_position_msg.header.frame_id;
                target_position.header.stamp = landing_tag_position_msg.header.stamp;
                target_position.point.x = (float) landing_tag_position_msg.point.x;
                target_position.point.y = (float) landing_tag_position_msg.point.y;
                target_position.point.z = (float) landing_tag_position_msg.point.z - 0.5; 
                m100->set_target_position(target_position);
                //m100->set_target_position(landing_tag_position_msg);

                if(landing_tag_position_msg.point.x > m100->get_local_position().x)
                {
                    if(m100->distance_to_position(target_position) < 0.2)
                    //if(m100->distance_to_position(landing_tag_position_msg) < 0.2)
                    {
                        approach = false;
                        land_flag = true;
                    }
                }
            }
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moving_april_landing_mission_node");
    ros::NodeHandle nh;
    ros::Rate rate(control_rate);
    gimbal  = new droneuse_gimbal(nh);
    m100    = new droneuse_m100(nh);
    tf_listener = new tf::TransformListener;
    
    geometry_msgs::PointStamped default_gimbal;
    default_gimbal.header.frame_id = "/body_frame";
    default_gimbal.point.x = 2.0;
    default_gimbal.point.y = 0.0;
    default_gimbal.point.z = 10.0;

	geometry_msgs::PointStamped waypoint1_waypoint;
	waypoint1_waypoint.point.x = 0.0;
	waypoint1_waypoint.point.y = 0.0;
	waypoint1_waypoint.point.z = 5.0;
	waypoint1_waypoint.header.frame_id = "/world";
    
    // Position where the apriltag is setted 
   
     m100_attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");

    ros::Subscriber apriltag_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/landing_tag_position",10, landing_tag_subscriber_callback);
    // Wait for take off
	while(m100->get_flight_status() != 3)
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

	waypoint1_waypoint.header.stamp = ros::Time::now();
	m100->set_target_position(waypoint1_waypoint);
	
    bool ascending = true;
follow_april_flag = true;
	while(ascending)
	{
		ros::spinOnce();
		waypoint1_waypoint.header.stamp = ros::Time::now();
		if(m100->distance_to_position(waypoint1_waypoint) < 0.5)
        {
			ascending = false;
            chase = true;
        }
        rate.sleep();
	}
    while(land_flag == false)
    {
        ros::spinOnce();
        rate.sleep();
    }
    m100->land();
    m100->disarm();
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
