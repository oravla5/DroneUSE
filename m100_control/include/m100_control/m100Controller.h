#ifndef M100_CONTROLLER_H 
#define M100_CONTROLLER_H 
#include <ros/ros.h> 
#include <dji_sdk/dji_sdk.h>
#include "pid.h"
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

class m100Controller 
{ 
    private: 
        tf::TransformListener* tf_listener;
        // Service 
        ros::ServiceClient m100_attitude_control_service; 

        // Subscriber
        ros::Subscriber m100_target_position_subscriber; 
        ros::Subscriber m100_target_orientation_subscriber;
        ros::Subscriber m100_position_control_state_subscriber;
        ros::Subscriber m100_orientation_control_state_subscriber;
        
        // Subscriber Callback
        void m100_target_position_subscriber_callback(const geometry_msgs::PointStamped m100_target_position);
        void m100_target_orientation_subscriber_callback(const geometry_msgs::PointStamped m100_target_orientation);
        void m100_position_control_state_subscriber_callback(const std_msgs::UInt8 control_state);
        void m100_orientation_control_state_subscriber_callback(const std_msgs::UInt8 control_state);
        
        geometry_msgs::PointStamped m100_target_position;
        geometry_msgs::PointStamped m100_target_orientation;
        bool position_control_enable = false;
        bool orientation_control_enable = false;

        // M100 Specs http://wiki.dji.com/en/index.php/Matrice_100 
        unsigned char ctrl_flag =   DJI::onboardSDK::Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                    DJI::onboardSDK::Flight::VerticalLogic::VERTICAL_VELOCITY |
                                    DJI::onboardSDK::Flight::YawLogic::YAW_PALSTANCE |
                                    DJI::onboardSDK::Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                    DJI::onboardSDK::Flight::SmoothMode::SMOOTH_ENABLE;
        dji_sdk::AttitudeControl m100_control_command;

        // PID controller parameters
        double x_maxVelocity    = 0.3;
        double y_maxVelocity    = 0.3;
        double z_maxVelocity    = 0.3;
        
        double x_velocity_Kp    = 0.5;
        double x_velocity_Kd    = 0.05;
        double x_velocity_Ki    = 0.0;
        
        double y_velocity_Kp    = 0.5;
        double y_velocity_Kd    = 0.05;
        double y_velocity_Ki    = 0.0;
        
        double z_velocity_Kp    = 0.5;
        double z_velocity_Kd    = 0.05;
        double z_velocity_Ki    = 0.0;
        
        //double max_velocity     = 1.0;

        double yaw_maxRate      = 10.0;

        double yaw_rate_Kp      = 1.0;
        double yaw_rate_Kd      = 0.05;
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

