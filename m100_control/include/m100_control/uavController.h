#ifndef UAV_CONTROLLER_H
#define UAV_CONTROLLER_H
#include <dji_sdk/dji_sdk.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

class uavController
{
    private:
        // Service
        ros::ServiceClient m100_attitude_control_service;

        // Subscriber
        ros::Subscriber m100_subscriber;
        ros::Subscriber m100_target_subscriber;

        void m100_local_position_subscriber_callback(const dji_sdk::LocalPosition m100_local_position);
        void m100_target_local_position_subscriber_callback(const dji_sdk::LocalPosition m100_target_local_position);

        dji_sdk::LocalPosition m100_local_position;
        dji_sdk::LocalPosition m100_target_local_position;

        // M100 Specs http://wiki.dji.com/en/index.php/Matrice_100
        double x_maxVelocity    = 1.0;
        double y_maxVelocity    = 1.0;
        double z_maxVelocity    = 1.0;

        double x_velocity_Kp    = 1.0;
        double x_velocity_Kd    = 0.0;
        double x_velocity_Ki    = 0.0;
        
        double y_velocity_Kp    = 1.0;
        double y_velocity_Kd    = 0.0;
        double y_velocity_Ki    = 0.0;
        
        double z_velocity_Kp    = 1.0;
        double z_velocity_Kd    = 0.0;
        double z_velocity_Ki    = 0.0;

        double vel_max          = 1.0;

        PID* m100_velocity_x_pid;
        PID* m100_velocity_y_pid;
        PID* m100_velocity_z_pid;

        int control_rate;

    public:

        bool velocity_based_position_controller();
        bool flag_based_motion_control(unsigned char ctrl_flag, float x, float z, float yaw);

        dji_sdk::LocalPosition  get_local_position();
        float                   get_local_X();
        float                   get_local_Y();
        float                   get_local_Y();
};
#endif
