#include "m100_control/m100Controller.h"
#include <ros/ros.h>
#include <dji_sdk/dji_sdk.h>
#include <math.h>
#include <stdio.h>
#include "m100_control/pid.h"
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#define C_PI (double) 3.141592653589793

void m100Controller::m100_target_local_position_subscriber_callback(const geometry_msgs::PointStamped target_position)
{
    this->m100_target_position = target_position;
}

void m100Controller::m100_control_state_subscriber_callback(const std_msgs::UInt8 control_state)
{
    this->control_state = control_state.data;
}

m100Controller::m100Controller(ros::NodeHandle& nh, int control_rate)
{
    this->control_rate = control_rate; // Hz

    m100_local_position_subscriber = nh.subscribe<dji_sdk::LocalPosition>("dji_sdk/local_position", 10, &m100Controller::m100_local_position_subscriber_callback, this);

    m100_target_local_position_subscriber = nh.subscribe<m100_control::TargetLocalPosition>("droneuse/local_position_target", 10, &m100Controller::m100_target_local_position_subscriber_callback, this);

    m100_control_state_subscriber = nh.subscribe<std_msgs::UInt8>("droneuse/m100_controller_state",10, &m100Controller::m100_control_state_subscriber_callback, this);

    m100_attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");

    // PID initialization

    m100_velocity_x_pid     = new PID(x_maxVelocity, -x_maxVelocity, x_velocity_Kp, x_velocity_Kd, x_velocity_Ki);
    m100_velocity_y_pid     = new PID(y_maxVelocity, -y_maxVelocity, y_velocity_Kp, y_velocity_Kd, y_velocity_Ki);
    m100_velocity_z_pid     = new PID(z_maxVelocity, -z_maxVelocity, z_velocity_Kp, z_velocity_Kd, z_velocity_Ki);
    m100_yaw_rate_pid       = new PID(yaw_maxRate, -yaw_maxRate, yaw_rate_Kp, yaw_rate_Kd, yaw_rate_Ki);
}

bool m100Controller::m100_controller_update()
{
    if(control_enable)
    {
        unsigned char ctrl_flag =   DJI::onboardSDK::Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                    DJI::onboardSDK::Flight::VerticalLogic::VERTICAL_VELOCITY |
                                    DJI::onboardSDK::Flight::YawLogic::YAW_RATE |
                                    DJI::onboardSDK::Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                    DJI::onboardSDK::Flight::SmoothMode::SMOOTH_ENABLE;

        geometry_msgs::PointStamped m100_target_position_transformed;
        tf_listener->tranformPoint("/body",m100_target_position, m100_target_position_transformed);


        double target_yaw = atan2(y,x)*180/C_PI; 
        
        double velocity_x   = m100_velocity_x_pid->calculate(1/control_rate, 0.0, m100_target_position_transformed.x);
        double velocity_y   = m100_velocity_y_pid->calculate(1/control_rate, 0.0, m100_target_position_transformed.y);
        double velocity_z   = m100_velocity_z_pid->calculate(1/control_rate, 0.0, m100_target_position_transformed.z);

        geometry_msgs::PointStamped m100_target_orientation_transformed;
        tf_listener->transformPoint("/body", m100_target_orientation, m100_target_orientation_transformed);
        
        double x            = m100_target_orientation_transformed.point.x;
        double y            = m100_target_orientation_transformed.point.y;

        double yaw_rate     = m100_yaw_rate_pid->calculate(1/control_rate, 0.0, target_yaw);
        
           // double vel_mod      = sqrt(velocity_x*velocity_x + velocity_y*velocity_y + velocity_z*velocity_z);

        //velocity_x          = velocity_x/vel_mod*max_velocity;
        //velocity_y          = velocity_y/vel_mod*max_velocity;
        //velocity_z          = velocity_z/vel_mod*max_velocity;

        dji_sdk::AttitudeControl m100_control_command;
        m100_control_command.request.flag   = ctrl_flag;
        m100_control_command.request.x      = (float) velocity_x;
        m100_control_command.request.y      = (float) velocity_y;
        m100_control_command.request.z      = (float) velocity_z;
        m100_control_command.request.yaw    = (float) yaw_rate;

        return m100_attitude_control_service.call(m100_control_command) && m100_control_command.response.result;
    }                   
}

