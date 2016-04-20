#include "gimbalController.h"
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include "pid.h"

void gimbalController::gimbal_subscriber_callback(const dji_sdk::Gimbal gimbal)
{
    this->gimbal = gimbal;
}

void gimbalController::gimbal_target_subscriber_callback(const dji_sdk::Gimbal gimbal_target)
{
    this->gimbal_target = gimbal_target;
}

void gimbalController::m100_local_position_subscriber_callback(const dji_sdk::LocalPosition m100_local_position)
{
    this->m100_local_position = m100_local_position;
}
gimbalController::gimbalController(ros::NodeHandle& nh, int control_rate)
{
    this->control_rate = control_rate;  // Hz

    gimbal_subscriber = nh.subscribe<dji_sdk::Gimbal>("dji_sdk/gimbal", 10, &gimbalController::gimbal_subscriber_callback, this);

    gimbal_target_subscriber = nh.subscribe<dji_sdk::Gimbal>("droneuse/gimbal_target", 10, &gimbalController::gimbal_target_subscriber_callback, this);

    m100_local_position_subscriber = nh.subscribe<dji_sdk::LocalPosition>("dji_sdk/local_position", 10, &gimbalController::m100_local_position_subscriber_callback, this);

    gimbal_speed_control_service = nh.serviceClient<dji_sdk::GimbalSpeedControl>("dji_sdk/gimbal_speed_control");

    // PID initialization
    gimbal_pitch_rate_pid = new PID(gimbal_pitch_maxRate, -gimbal_pitch_maxRate, gimbal_pitchRate_Kp, gimbal_pitchRate_Kd, gimbal_pitchRate_Ki);

    gimbal_yaw_rate_pid = new PID(gimbal_yaw_maxRate, -gimbal_yaw_maxRate, gimbal_yawRate_Kp, gimbal_yawRate_Kd, gimbal_yawRate_Ki);
}


bool gimbalController::gimbal_rate_based_orientation_controller()
{
    double pitch_rate = gimbal_pitch_rate_pid->calculate(1/control_rate, (double) gimbal_attitude.pitch, (double) gimbal_target.pitch);

double yaw_rate = gimbal_yaw_rate_pid->calculate(1/control_rate, (double) gimbal_attitude.yaw, (double) gimbal_target.yaw);

    // dji_sdk Service Call
    dji_sdk::GimbalSpeedControl gimbal_speed_control;
    gimbal_speed_control.request.roll_rate = 0;
    gimbal_speed_control.request.pitch_rate = (int) pitch_rate;
    gimbal_speed_control.request.yaw_rate = (int) yaw_rate;

	return gimbal_speed_control_service.call(gimbal_speed_control) && gimbal_speed_control.response.result;
}

void set_rpy_to_body(int roll, int pitch, int yaw)
{
    
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
