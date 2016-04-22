#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H
#include <ros/ros.h>
#include <dji_sdk/dji_sdk.h>
#include "pid.h"
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

class gimbalController
{
    private:
        tf::TransformListener* tf_listener;

        // Service
        ros::ServiceClient gimbal_speed_control_service;

        // Subscriber
        ros::Subscriber gimbal_target_subscriber;
        ros::Subscriber gimbal_control_state_subscriber;

        void gimbal_target_subscriber_callback(geometry_msgs::PointStamped gimbal_target);
        void gimbal_control_state_subscriber_callback(std_msgs::UInt8 control_state);

        geometry_msgs::PointStamped gimbal_attitude_target;

        bool control_enable = false;

        // Gimbal Mechanical Specs: http://wiki.dji.com/en/index.php/Matrice_100-DJI_Zenmuse_X3_Gimbal_with_Camera
        double gimbal_yaw_maxRate       = 400.0;
        double gimbal_pitch_maxRate     = 400.0;

        // Gimbal pitch rate PID parameters
        double gimbal_pitchRate_Kp      = 20.0;
        double gimbal_pitchRate_Kd      = 0.0;
        double gimbal_pitchRate_Ki      = 0.0;

        // Gimbal yaw rate PID parameters
        double gimbal_yawRate_Kp        = 20.0;
        double gimbal_yawRate_Kd        = 0.0;
        double gimbal_yawRate_Ki        = 0.0;

        PID* gimbal_pitch_rate_pid;
        PID* gimbal_yaw_rate_pid;

        int control_rate;

    public:
        gimbalController(ros::NodeHandle& nh, int control_rate);
        bool gimbal_controller_update();


};

#endif
