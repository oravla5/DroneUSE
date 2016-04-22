#include "droneuse_m100.h"
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include "pid.h"
#define C_PI (double) 3.141592653589793


void droneuse_m100::m100_position_subscriber_callback(const dji_sdk::LocalPosition m100_local_position)
{
    this->m100_local_position = local_position;
}

void droneuse_m100::refresh_m100_target_local_position(float x, float y, float z)
{
    LocalPosition::m100_target_local_position;
    m100_target_local_position.x = x;
    m100_target_local_position.y = y;
    m100_target_local_position.z = z;

    m100_target_local_position_publisher.publish(m100_target_local_position);
}

void droneuse_m100::refresh_m100_target_local_position(float x, float y, float z, float yaw)
{
    LocalPosition::m100_target_local_position;
    m100_target_local_position.x = x;
    m100_target_local_position.y = y;
    m100_target_local_position.z = z;
    m100_target_local_position.yaw = yaw;

    m100_target_local_position_publisher.publish(m100_target_local_position);
}

void droneuse_m100::enable_m100_controller()
{
    std_msgs::UInt8 m100_control_msgs;
    m100_control.data = true;

    m100_control_mode_publisher.publish(m100_control_msgs);
}

void droneuse_m100::disable_m100_velocity_control()
{
    std_msgs::UInt8 m100_control_msgs;
    m100_control.data = false;

    m100_control_mode_publisher.publish(m100_control_msgs);
}

droneuse_gimbal::droneuse_gimbal(ros::NodeHandle& nh)
{
    gimbal_attitude_subscriber = nh.subscriber<dji_sdk::Gimbal>("dji_sdk/local_position", 10, &droneuse_gimbal::gimbal_attitude_subscriber_callback, this);

   m100_target_local_position_publisher = nh.advertise<dji_sdk::LocalPosition>("droneuse/m100_target_position", 10);
   m100_control_mode_publisher = nh.advertise<std_msgs::UInt8>("droneuse/m100_controller_state", 10);

}

bool droneuse_gimbal::refresh_gimbal_target_orientation(unsigned char ctrl_flag, float x, float y, float z, float yaw)
{
    // Control Flag Structure: http://download.dji-innovations.com/downloads/dev/OnboardSDK/Onboard_API_introduction_version_1.0.1_en.pdf

        dji_sdk::AttitudeControl attitude_control;
		attitude_control.request.flag = ctrl_flag;
		attitude_control.request.x = x;
		attitude_control.request.y = y;
		attitude_control.request.z = z;
		attitude_control.request.yaw = yaw;

		return attitude_control_service.call(attitude_control) && attitude_control.response.result;
}

dji_sdk::Gimbal get_gimbal()
{
    return gimbal_attitude;
}

float get_yaw()
{
    return gimbal_attitude.yaw;
}

float get_pitch()
{
    return gimbal_attitude.pitch;
}

float get_roll()
{
    return gimbal_attitude.roll;
}
