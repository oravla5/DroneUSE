// LANDING NODE
#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk.h>
#include <std_msgs/UInt8.h>
#include <mission_control/droneuse_m100.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace DJI::onboardSDK;

/// State machine of the followme mission
//void performTask(const States &current);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_response_identification_mission_node");
    ros::NodeHandle nh;
    ros::Rate rate(control_rate);
    m100    = new droneuse_m100(nh);
    tf_listener = new tf::TransformListener;
    
	geometry_msgs::PointStamped waypoint1_waypoint;
	waypoint1_waypoint.point.x = 0.0;
	waypoint1_waypoint.point.y = 0.0;
	waypoint1_waypoint.point.z = 5.0;
	waypoint1_waypoint.header.frame_id = "/world";
    
    ros::Publisher referencePub = nh.advertise<dji_sdk::Velocity>("droneuse/reference_velocity",1);
    // Position where the apriltag is setted 
   
    sleep(3);
    while(m100->get_sdk_control() != true)
    {
		ros::spinOnce();
        rate.sleep();
    }

    while(m100->arm() != true)
    {
		ros::spinOnce();
        rate.sleep();
    }

    while(m100->takeoff() != true)
    {
	ros::spinOnce();
        rate.sleep();
    }


    unsigned char ctrl_flag =   DJI::onboardSDK::Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                DJI::onboardSDK::Flight::VerticalLogic::VERTICAL_VELOCITY |
                                DJI::onboardSDK::Flight::YawLogic::YAW_PALSTANCE |
                                DJI::onboardSDK::Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                DJI::onboardSDK::Flight::SmoothMode::SMOOTH_ENABLE;

    dji_sdk::Velocity reference_velocity_msg;
    reference_velocity_msg.vx = 0.0;
    reference_velocity_msg.vy = 0.0;
    reference_velocity_msg.vz = 1.0;

    ros::Duration hold_time(20.0);
    ros::Time time_init = ros::Time::now(); 
    ros::Time currentTime = ros::Time::now();
    ros::Duration duration = currentTime - time_init;
    while (duration < hold_time)
    {
        ros::spinOnce();
	if(m100->attitude_control(ctrl_flag, reference_velocity_msg.vx, reference_velocity_msg.vy, reference_velocity_msg.vz, 0.0))
        {
		ros::Time currentTime = ros::Time::now();
		duration = currentTime - time_init;
		reference_velocity_msg.header.stamp = currentTime;
		referencePub.publish(reference_velocity_msg);
	}
        rate.sleep();
        
    }
    
    reference_velocity_msg.vx = 0.0;
    reference_velocity_msg.vy = 0.0;
    reference_velocity_msg.vz = 0.0;

    time_init = ros::Time::now(); 
    currentTime = ros::Time::now();
    duration = currentTime - time_init;
    while (duration < hold_time)
    {
        ros::spinOnce();
	if(m100->attitude_control(ctrl_flag, reference_velocity_msg.vx, reference_velocity_msg.vy, reference_velocity_msg.vz, 0.0))
        {
		ros::Time currentTime = ros::Time::now();
		duration = currentTime - time_init;
		reference_velocity_msg.header.stamp = currentTime;
		referencePub.publish(reference_velocity_msg);
	}
        rate.sleep();
    }

    reference_velocity_msg.vx = 1.0;
    reference_velocity_msg.vy = 0.0;
    reference_velocity_msg.vz = 0.0;

    time_init = ros::Time::now(); 
    currentTime = ros::Time::now();
    duration = currentTime - time_init;
    while (duration < hold_time)
    {
        ros::spinOnce();
	if(m100->attitude_control(ctrl_flag, reference_velocity_msg.vx, reference_velocity_msg.vy, reference_velocity_msg.vz, 0.0))
        {
		ros::Time currentTime = ros::Time::now();
		duration = currentTime - time_init;
		reference_velocity_msg.header.stamp = currentTime;
		referencePub.publish(reference_velocity_msg);
	}
        rate.sleep();
    }
    reference_velocity_msg.vx = 0.0;
    reference_velocity_msg.vy = 0.0;
    reference_velocity_msg.vz = 0.0;

    time_init = ros::Time::now(); 
    currentTime = ros::Time::now();
    duration = currentTime - time_init;
    while (duration < hold_time)
    {
        ros::spinOnce();
	if(m100->attitude_control(ctrl_flag, reference_velocity_msg.vx, reference_velocity_msg.vy, reference_velocity_msg.vz, 0.0))
        {
		ros::Time currentTime = ros::Time::now();
		duration = currentTime - time_init;
		reference_velocity_msg.header.stamp = currentTime;
		referencePub.publish(reference_velocity_msg);
	}
        rate.sleep();
    }

    reference_velocity_msg.vx = 0.0;
    reference_velocity_msg.vy = 1.0;
    reference_velocity_msg.vz = 0.0;

    time_init = ros::Time::now(); 
    currentTime = ros::Time::now();
    duration = currentTime - time_init;
    while (duration < hold_time)
    {
        ros::spinOnce();
	if(m100->attitude_control(ctrl_flag, reference_velocity_msg.vx, reference_velocity_msg.vy, reference_velocity_msg.vz, 0.0))
        {
		ros::Time currentTime = ros::Time::now();
		duration = currentTime - time_init;
		reference_velocity_msg.header.stamp = currentTime;
		referencePub.publish(reference_velocity_msg);
	}
        rate.sleep();
    }

    reference_velocity_msg.vx = 0.0;
    reference_velocity_msg.vy = 0.0;
    reference_velocity_msg.vz = 0.0;

    time_init = ros::Time::now(); 
    currentTime = ros::Time::now();
    duration = currentTime - time_init;
    while (duration < hold_time)
    {
        ros::spinOnce();
	if(m100->attitude_control(ctrl_flag, reference_velocity_msg.vx, reference_velocity_msg.vy, reference_velocity_msg.vz, 0.0))
        {
		ros::Time currentTime = ros::Time::now();
		duration = currentTime - time_init;
		reference_velocity_msg.header.stamp = currentTime;
		referencePub.publish(reference_velocity_msg);
	}
        rate.sleep();
    }

    reference_velocity_msg.vx = 0.0;
    reference_velocity_msg.vy = 0.0;
    reference_velocity_msg.vz = 1.0;

    time_init = ros::Time::now(); 
    currentTime = ros::Time::now();
    duration = currentTime - time_init;
    while (duration < hold_time)
    {
        ros::spinOnce();
	if(m100->attitude_control(ctrl_flag, reference_velocity_msg.vx, reference_velocity_msg.vy, reference_velocity_msg.vz, 0.0))
        {
		ros::Time currentTime = ros::Time::now();
		duration = currentTime - time_init;
		reference_velocity_msg.header.stamp = currentTime;
		referencePub.publish(reference_velocity_msg);
	}
        rate.sleep();
    }

    reference_velocity_msg.vx = 0.0;
    reference_velocity_msg.vy = 0.0;
    reference_velocity_msg.vz = 0.0;

    time_init = ros::Time::now(); 
    currentTime = ros::Time::now();
    duration = currentTime - time_init;
    while (duration < hold_time)
    {
        ros::spinOnce();
	if(m100->attitude_control(ctrl_flag, reference_velocity_msg.vx, reference_velocity_msg.vy, reference_velocity_msg.vz, 0.0))
        {
		ros::Time currentTime = ros::Time::now();
		duration = currentTime - time_init;
		reference_velocity_msg.header.stamp = currentTime;
		referencePub.publish(reference_velocity_msg);
	}
        rate.sleep();
    }

    m100->land();
    m100->disarm();
    return 0;
}    
