#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H
#include <dji_sdk/dji_sdk.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "pid.h"

class gimbalController
{
    private:
        // Service
        ros::ServiceClient gimbal_speed_control_service;

        // Subscriber
        ros::Subscriber gimbal_subscriber;
        ros::Subscriber gimbal_target_subscriber;
        ros::Subscriber m100_local_position_subscriber;

        void gimbal_subscriber_callback(const dji_sdk::Gimbal gimbal);
        void gimbal_target_subscriber_callback(const dji_sdk::Gimbal gimbal_target);
        void m100_local_position_subscriber_callback(const dji_sdk::LocalPosition m100_local_position);

        dji_sdk::Gimbal gimbal_attitude;
        dji_sdk::Gimbal gimbal_target;
        dji_sdk::LocalPosition m100_local_positon;

        int gimbal_roll_offset      = 0;
        int gimbal_pitch_offset     = 0;
        int gimbal_yaw_offset       = 0;

        // Gimbal Mechanical Specs: http://wiki.dji.com/en/index.php/Matrice_100-DJI_Zenmuse_X3_Gimbal_with_Camera
        double gimbal_yaw_maxRate       = 1800.0;
        double gimbal_pitch_maxRate     = 1200.0;

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

        typedef gimbalBehaviour
        {
            FPV,
            HOT_POINT,
            APRILTAG_TRACK,
            ATTITUDE
        } struct;

    public:

        bool gimbal_rate_based_orientation_cotroller();
        void set_rpy_to_body(int roll, int pitch, int yaw);

        dji_sdk::Gimbal     get_gimbal();
        float               get_yaw();
        float               get_pitch();
        float               get_roll();

};

#endif
