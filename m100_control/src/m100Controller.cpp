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

void m100Controller::m100_target_position_subscriber_callback(const geometry_msgs::PointStamped target_position)
{
    this->m100_target_position = target_position;
    this->position_control_enable = true;
}

void m100Controller::m100_position_control_state_subscriber_callback(const std_msgs::UInt8 control_state)
{
    this->position_control_enable = control_state.data;
}

void m100Controller::m100_orientation_control_state_subscriber_callback(const std_msgs::UInt8 control_state)
{
    this->orientation_control_enable = control_state.data;
}

void m100Controller::m100_target_orientation_subscriber_callback(const geometry_msgs::PointStamped target_orientation)
{
    this->m100_target_orientation = target_orientation;
    this->orientation_control_enable = true;
}

m100Controller::m100Controller(ros::NodeHandle& nh, int control_rate)
{
    this->control_rate = control_rate; // Hz

    tf_listener = new tf::TransformListener;

    m100_attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");

    m100_target_position_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/m100_target_position", 10, &m100Controller::m100_target_position_subscriber_callback, this);
    m100_target_orientation_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/m100_target_orientation", 10, &m100Controller::m100_target_orientation_subscriber_callback, this);
    m100_position_control_state_subscriber = nh.subscribe<std_msgs::UInt8>("droneuse/m100_position_control_state",10, &m100Controller::m100_position_control_state_subscriber_callback, this);
    m100_orientation_control_state_subscriber = nh.subscribe<std_msgs::UInt8>("droneuse/m100_orientation_control_state",10, &m100Controller::m100_orientation_control_state_subscriber_callback, this);

    m100_velocity_x_pid     = new PID(x_maxVelocity, -x_maxVelocity, x_velocity_Kp, x_velocity_Kd, x_velocity_Ki);
    m100_velocity_y_pid     = new PID(y_maxVelocity, -y_maxVelocity, y_velocity_Kp, y_velocity_Kd, y_velocity_Ki);
    m100_velocity_z_pid     = new PID(z_maxVelocity, -z_maxVelocity, z_velocity_Kp, z_velocity_Kd, z_velocity_Ki);
    m100_yaw_rate_pid       = new PID(yaw_maxRate, -yaw_maxRate, yaw_rate_Kp, yaw_rate_Kd, yaw_rate_Ki);

}

bool m100Controller::m100_controller_update()
{
    double velocity_x = 0;   
    double velocity_y = 0;  
    double velocity_z = 0; 
    double yaw_rate   = 0;

    if(position_control_enable)
    {
        try
        {
            ros::Time time_now = ros::Time::now();
            tf_listener->waitForTransform("/body_frame", "/world", time_now , ros::Duration(2.0/control_rate));
            tf_listener->transformPoint("/body_frame", time_now, m100_target_position, "/world", m100_target_position);

// TODO: check the need for a minus sign on velocity y
            velocity_x   = m100_velocity_x_pid->calculate(1.0/control_rate, m100_target_position.point.x, 0.0);
            velocity_y   = -m100_velocity_y_pid->calculate(1.0/control_rate, m100_target_position.point.y, 0.0);
            velocity_z   = -m100_velocity_z_pid->calculate(1.0/control_rate, m100_target_position.point.z, 0.0);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return 0;
        }
    }                   
    if(orientation_control_enable)
    {
        try
        {
            ros::Time time_now = ros::Time::now();
            tf_listener->waitForTransform("/body_frame", "/world", time_now , ros::Duration(2.0/control_rate));
            tf_listener->transformPoint("/body_frame", time_now, m100_target_orientation, "/world", m100_target_orientation);

            double x            = m100_target_orientation.point.x;
            double y            = m100_target_orientation.point.y;
            double target_yaw   = atan2(y,x)*180/C_PI;

            yaw_rate     = m100_yaw_rate_pid->calculate(1.0/control_rate, target_yaw, 0.0);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return 0;
        }

    }

    if(position_control_enable || orientation_control_enable)
    {
        m100_control_command.request.flag   = ctrl_flag;
        m100_control_command.request.x      = (float) velocity_x;
        m100_control_command.request.y      = (float) velocity_y;
        m100_control_command.request.z      = (float) velocity_z;
        m100_control_command.request.yaw    = (float) yaw_rate;
        return m100_attitude_control_service.call(m100_control_command) && m100_control_command.response.result;
    }
    else
        return 0;
}
