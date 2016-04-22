#ifndef DRONEUSE_GIMBAL_H 
#define DRONEUSE_GIMBAL_H 
#include <ros/ros.h> 
#include <dji_sdk/dji_sdk.h>
#include <geometry_msgs/PointStamped.h>

class droneuse_gimbal
{
    private:
        // Service Suscription
        ros::ServiceClient gimbal_speed_control_service;

        // Topic Subscriber
        ros::Subscriber gimbal_attitude_subscriber;
        
        // Topic Publisher
        ros::Publisher gimbal_target_publisher;
        ros::Publisher gimbal_control_state_publisher;

        // Subscriber Callback
        void gimbal_attitude_subscriber_callback();
        
    public:
        void refresh_gimbal_target_orientation(float roll, float pitch, float yaw);
        void enable_gimbal_controller();
        void disable_gimbal_controller();

        dji_sdk::Gimbal     get_gimbal();
        float               get_yaw();
        float               get_pitch();
        float               get_roll();
};
