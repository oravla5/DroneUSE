#include "m100_control/gimbalController.h"
#include <ros/ros.h>
#include <dji_sdk/dji_sdk.h>
#include <math.h>
#include <stdio.h>
#include "m100_control/pid.h"
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#define C_PI (double) 3.141592653589793

void gimbalController::gimbal_target_subscriber_callback(geometry_msgs::PointStamped gimbal_target)
{
    this->gimbal_target = gimbal_target;
    this->control_enable = true;
}

void gimbalController::gimbal_control_type_subscriber_callback(std_msgs::Bool control_type)
{
//	std::cout << "Control type received : " << control_type << std::endl;
    this->control_type = control_type.data;
}

void gimbalController::gimbal_control_state_subscriber_callback(std_msgs::UInt8 control_state)
{
    this->control_enable = control_state.data;
}

gimbalController::gimbalController(ros::NodeHandle& nh, int control_rate)
{
    this->control_rate = control_rate;  // Hz

    tf_listener = new tf::TransformListener;

    gimbal_speed_control_service        = nh.serviceClient<dji_sdk::GimbalSpeedControl>("dji_sdk/gimbal_speed_control");

    gimbal_target_subscriber            = nh.subscribe<geometry_msgs::PointStamped>("droneuse/gimbal_target", 1, &gimbalController::gimbal_target_subscriber_callback, this);
    gimbal_control_state_subscriber     = nh.subscribe<std_msgs::UInt8>("droneuse/gimbal_control_state", 3, &gimbalController::gimbal_control_state_subscriber_callback, this);
    gimbal_control_type_subscriber     = nh.subscribe<std_msgs::Bool>("droneuse/gimbal_control_type", 1, &gimbalController::gimbal_control_type_subscriber_callback, this);

    gimbal_pitch_rate_pid = new PID(gimbal_pitch_maxRate, -gimbal_pitch_maxRate, gimbal_pitchRate_Kp, gimbal_pitchRate_Kd, gimbal_pitchRate_Ki);
    gimbal_yaw_rate_pid = new PID(gimbal_yaw_maxRate, -gimbal_yaw_maxRate, gimbal_yawRate_Kp, gimbal_yawRate_Kd, gimbal_yawRate_Ki);
}

bool gimbalController::gimbal_controller_update()
{
    if(control_enable)
    {
        try
        {
            ros::Time time_now = ros::Time::now();
		// control_type: orientation=true, target=false
		if(control_type)
		{
			gimbal_target.header.stamp = ros::Time::now();
		}
	    geometry_msgs::PointStamped transformed_target;
            tf_listener->waitForTransform(gimbal_target.header.frame_id, "/gimbal", time_now, ros::Duration(2.0/control_rate));
		// Commented code is original one
            //tf_listener->waitForTransform("/gimbal", "/world", time_now, ros::Duration(2.0/control_rate));
            //tf_listener->transformPoint("/gimbal", time_now, gimbal_target, "/world", gimbal_target);
            tf_listener->transformPoint("/gimbal", time_now, gimbal_target, "/world", transformed_target);
            double x         = transformed_target.point.x;
            double y         = transformed_target.point.y;
            double z         = transformed_target.point.z;
            double r_proj    = sqrt(x*x + y*y);

            double target_pitch = atan2(-z,r_proj)*180/C_PI; 
            double target_yaw = 0.0; 
//            double target_yaw = atan2(y,x)*180/C_PI; 

            double pitch_rate = gimbal_pitch_rate_pid->calculate(1.0/control_rate, target_pitch, 0.0);
            double yaw_rate = gimbal_yaw_rate_pid->calculate(1.0/control_rate, target_yaw, 0.0);

            dji_sdk::GimbalSpeedControl gimbal_speed_control;
            gimbal_speed_control.request.roll_rate = 0;
            gimbal_speed_control.request.pitch_rate = (int) pitch_rate;
            gimbal_speed_control.request.yaw_rate = (int) yaw_rate;

            return gimbal_speed_control_service.call(gimbal_speed_control) && gimbal_speed_control.response.result;
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return 0;
        }
    }
}

