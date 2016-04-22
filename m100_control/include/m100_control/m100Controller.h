#ifndef M100_CONTROLLER_H 
#define M100_CONTROLLER_H 
#include <ros/ros.h> 
#include <dji_sdk/dji_sdk.h>
#include "pid.h"
#include <std_msgs/UInt8.h>
#include <m100_control/TargetLocalPosition.h>
#include <tf/transform_listener.h>

class m100Controller 
{ 
    private: 
        tf::TransformListener* tf_listener;
        // Service 
        ros::ServiceClient m100_attitude_control_service; 
        // Subscriber
        ros::Subscriber m100_target_local_position_subscriber; 
        ros::Subscriber m100_control_state_subscriber;
        
        // Subscriber Callback
        void m100_target_local_position_subscriber_callback(const m100_control::TargetLocalPosition m100_target_local_position);
        void m100_control_state_subscriber_callback(const std_msgs::UInt8 control_state);
        
        geometry_msgs::PointStamped m100_target_position;
        geometry_msgs::PointStamped m100_target_orientation;
        bool control_enable = false;
        
        // M100 Specs http://wiki.dji.com/en/index.php/Matrice_100 
        
        // PID controller parameters
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
        
        double max_velocity     = 1.0;

        double yaw_maxRate      = 1.0;

        double yaw_rate_Kp      = 1.0;
        double yaw_rate_Kd      = 0.0;
        double yaw_rate_Ki      = 0.0;
        
        PID* m100_velocity_x_pid;
        PID* m100_velocity_y_pid;
        PID* m100_velocity_z_pid;
        PID* m100_yaw_rate_pid;
        
        int control_rate;
    
    public: 
        m100Controller(ros::NodeHandle& nh, int control_rate);
        bool m100_controller_update();
        
}; 

#endif

