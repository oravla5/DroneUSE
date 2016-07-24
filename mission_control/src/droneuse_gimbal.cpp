#include "mission_control/droneuse_gimbal.h"
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#define C_PI (double) 3.141592653589793

void droneuse_gimbal::gimbal_attitude_subscriber_callback(const dji_sdk::Gimbal gimbal_attitude)
{
    this->gimbal_attitude = gimbal_attitude;
}

void droneuse_gimbal::disable_gimbal_controller()
{
    std_msgs::UInt8 gimbal_control_msgs;
    gimbal_control_msgs.data = false;

    gimbal_control_state_publisher.publish(gimbal_control_msgs);
}

droneuse_gimbal::droneuse_gimbal(ros::NodeHandle& nh)
{
    gimbal_speed_control_service = nh.serviceClient<dji_sdk::GimbalSpeedControl>("dji_sdk/gimbal_speed_control");
    
    gimbal_attitude_subscriber = nh.subscribe<dji_sdk::Gimbal>("dji_sdk/gimbal", 10, &droneuse_gimbal::gimbal_attitude_subscriber_callback, this);

   gimbal_target_publisher = nh.advertise<geometry_msgs::PointStamped>("droneuse/gimbal_target", 1);
   //control type: true=orientation, false=target
   gimbal_control_type_publisher = nh.advertise<std_msgs::Bool>("droneuse/gimbal_control_type", 1);
   gimbal_control_state_publisher = nh.advertise<std_msgs::UInt8>("droneuse/gimbal_control_state", 10);
}

void droneuse_gimbal::set_target(geometry_msgs::PointStamped target_orientation)
{
    gimbal_target_publisher.publish(target_orientation);
    //control type: true=orientation, false=target
    std_msgs::Bool control_type;
    control_type.data = false;
    gimbal_control_type_publisher.publish(control_type);
}

bool droneuse_gimbal::set_pitch(float pitch)
{
	if( (pitch < 120.0) && (pitch > -30.0) )
	{
		geometry_msgs::PointStamped target_orientation;
		target_orientation.header.frame_id = "body_frame";
		target_orientation.header.stamp = ros::Time::now();
		target_orientation.point.x 	= 1.0;
		target_orientation.point.y 	= 0.0;
		target_orientation.point.z 	= tan(pitch*C_PI/180.0); 
		gimbal_target_publisher.publish(target_orientation);
		//control type: true=orientation, false=target
		std_msgs::Bool control_type;
		control_type.data = true;
		gimbal_control_type_publisher.publish(control_type);
		return true;
	}
	else
		return false;
}

bool droneuse_gimbal::set_yaw(float yaw)
{
	if(fabs(yaw) < 100.0)
	{
		geometry_msgs::PointStamped target_orientation;
		target_orientation.header.frame_id = "body_frame";
		target_orientation.header.stamp = ros::Time::now();
		target_orientation.point.x 	= 1.0;
		target_orientation.point.y 	= tan(yaw*C_PI/180.0); 
		target_orientation.point.z 	= 0.0;
		gimbal_target_publisher.publish(target_orientation);
		//control type: true=orientation, false=target
		std_msgs::Bool control_type;
		control_type.data = true;
		gimbal_control_type_publisher.publish(control_type);
		return true;
	}
	else
		return false;
}

dji_sdk::Gimbal droneuse_gimbal::get_gimbal()
{
    return gimbal_attitude;
}

float droneuse_gimbal::get_yaw()
{
    return gimbal_attitude.yaw;
}

float droneuse_gimbal::get_pitch()
{
    return gimbal_attitude.pitch;
}

float droneuse_gimbal::get_roll()
{
    return gimbal_attitude.roll;
}
