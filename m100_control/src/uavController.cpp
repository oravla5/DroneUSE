#include "uavController.h"
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include "pid.h"
#define C_PI (double) 3.141592653589793

void uavController::m100_position_subscriber_callback(const dji_sdk::LocalPosition local_position)
{
    this->m100_local_position = local_position;
}

void uavController::m100_target_position_subscriber_callback(const dji_sdk::LocalPosition target_local_position)
{
    this->m100_target_local_position = target_local_position;
}

uavController::uavController(ros::NodeHandle& nh, int control_rate)
{
    this->control_rate = control_rate; // Hz

    m100_position_subscriber = nh.subscribe<dji_sdk::LocalPosition>("dji_sdk/local_position", 10, &uavController::m100_position_subscriber_callback, this);

    m100_target_position_subscriber = nh.subscribe<dji_sdk::LocalPosition>("droneuse/local_position_target", 10, &uavController::m100_target_position_subscriber_callback, this);

    m100_attitude_control_service = serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");

    // PID initialization

    m100_vel_x_pid = new PID(x_maxVelocity, -x_maxVelocity, x_velocity_Kp, x_velocity_Kd, x_velocity_Ki);
    m100_vel_y_pid = new PID(y_mayVelocity, -y_mayVelocity, y_velocity_Kp, y_velocity_Kd, y_velocity_Ki);
    m100_vel_z_pid = new PID(z_mazVelocity, -z_mazVelocity, z_velocity_Kp, z_velocity_Kd, z_velocity_Ki);
}

bool uavController::m100_velocity_based_position_controller()
{
   // REFERENCE = GROUND_FRAME
    unsigned char ctrl_flag =   Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                Flight::VerticalLogic::VERTICAL_VELOCITY |
                                Flight::YawLogic::YAW_ANGLE |
                                Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
                                Flight::SmoothMode::SMOOTH_ENABLE;

    double velocity_x   = m100_velocity_x_pid->calculate(1/control_rate, m100_local_position.x, m100_target_local_position.y);
    double velocity_y   = m100_velocity_y_pid->calculate(1/control_rate, m100_local_position.y, m100_target_local_position.y);
    double velocity_z   = m100_velocity_z_pid->calculate(1/control_rate, m100_local_position.z, m100_target_local_position.y);
    double vel_mod      = sqrt(velocity_x*velocity_x + velocity_y*velocity_y + velocity_z*velocity_z);

    velocity_x          = velocity_x/vel_mod*vel_max;
    velocity_y          = velocity_y/vel_mod*vel_max;
    velocity_z          = velocity_z/vel_mod*vel_max;

    float yaw           = atan2(velocity_y, velocity_x)*C_PI/180;

    dji_sdk::AttitudeControl m100_control_command;
    m100_control_command.request.flag   = ctrl_flag;
    m100_control_command.request.x      = (float) velocity_x;
    m100_control_command.request.y      = (float) velocity_y;
    m100_control_command.request.z      = (float) velocity_z;
    m100_control_command.request.yaw    = yaw;

    return m100_attitude_control_service.call(m100_control_command) && m100_attitude_control_service.response.result;
                                 
}

bool uavController::flag_based_motion_control(unsigned char ctrl_flag, float x, float y, float z, float yaw)
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

dji_sdk::LocalPosition get_local_position()
{
    return m100_local_position;
}

float get_local_X()
{
    return m100_local_position.x;
}

float get_local_Y()
{
    return m100_local_position.y;
}

float get_local_Z()
{
    return m100_local_position.z;
}
