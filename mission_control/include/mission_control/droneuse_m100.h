#ifndef DRONEUSE_M100_H 
#define DRONEUSE_M100_H 
#include <ros/ros.h> 
#include <dji_sdk/dji_sdk.h>
#include <geometry_msgs/PointStamped.h>

class droneuse_m100
{
    private:
        ros::ServiceClient m100_attitude_control_service;
        ros::ServiceClient m100_task_control_service;
        ros::ServiceClient m100_arm_control_service;
        ros::ServiceClient m100_sdk_permission_control_service;

        ros::Subscriber m100_local_position_subscriber;
        
        ros::Publisher m100_target_position_publisher;
        ros::Publisher m100_target_orientation_publisher;
        ros::Publisher m100_position_control_state_publisher;
        ros::Publisher m100_orientation_control_state_publisher;
        
        void m100_local_position_subscriber_callback(const dji_sdk::LocalPosition m100_local_position);
        
        dji_sdk::LocalPosition      m100_local_position;
        geometry_msgs::PointStamped m100_target_position;

    public:
        droneuse_m100(ros::NodeHandle& nh);
        
        void set_target_position(geometry_msgs::PointStamped target_position);
        void set_target_orientation(geometry_msgs::PointStamped target_orientation);
        void disable_m100_position_control();
        void disable_m100_orientation_control();

        bool attitude_control(unsigned char ctrl_flag, float x, float y, float z, float yaw);

        bool get_sdk_control();
        bool release_sdk_control();
        bool takeoff();
        bool land();
        bool arm();
        bool disarm();

        dji_sdk::LocalPosition  get_local_position();
};

#endif
