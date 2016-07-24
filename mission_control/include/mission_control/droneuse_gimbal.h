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
        ros::Publisher gimbal_control_type_publisher;
        ros::Publisher gimbal_control_state_publisher;

        // Subscriber Callback
        void gimbal_attitude_subscriber_callback(const dji_sdk::Gimbal gimbal);

        dji_sdk::Gimbal gimbal_attitude;
        geometry_msgs::PointStamped gimbal_target_attitude;
        
    public:
        droneuse_gimbal(ros::NodeHandle& nh);
        void set_target(geometry_msgs::PointStamped target_orientation);
	bool set_pitch(float pitch);	// Angle given in degrees
	bool set_yaw(float yaw);	// Angle given in degrees
        void disable_gimbal_controller();

        dji_sdk::Gimbal     get_gimbal();
        float               get_yaw();
        float               get_pitch();
        float               get_roll();
};

#endif
