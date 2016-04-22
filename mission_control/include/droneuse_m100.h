#ifndef DRONEUSE_M100_H 
#define DRONEUSE_M100_H 
#include <dji_sdk/dji_sdk.h>
#include <ros/ros.h> 
#include <geometry_msgs/PointStamped.h>

class droneuse_m100
{
    private:
        // Service Suscription
        ros::ServiceClient m100_attitude_control_service;

        // Topic Subscriber
        ros::Subscriber m100_local_position_subscriber;
        
        // Topic Publisher
        ros::Publisher m100_target_local_position_publisher;
        ros::Publisher m100_control_mode_publisher;
        
        // Subscriber Callback
        void m100_local_position_subscriber_callback(const dji_sdk::LocalPosition m100_local_position);
        
        dji_sdk::LocalPosition m100_local_position;
        dji_sdk::LocalPosition m100_target_local_position;

    public:
        void refresh_m100_target_local_position(float x, float y, float z, float yaw);
        void enable_m100_velocity_control();
        void disable_m100_velocity_control();
        bool attitude_control(unsigned char ctrl_flag, float x, float y, float z, float yaw);

        dji_sdk::LocalPosition  get_local_position();
        float                   get_local_X();
        float                   get_local_Y();
        float                   get_local_Y();
};
