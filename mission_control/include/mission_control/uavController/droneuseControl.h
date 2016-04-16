#ifndef DRONEUSE_CONTROL_H
#define DRONEUSE_CONTROL_H
#include <dji_sdk/dji_sdk.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

class droneuseControl
{
private:
    
    ros::ServiceClient gimbal_angle_control_service;
    ros::ServiceClient gimbal_speed_control_service;
    
    ros::Subscriber gimbal_subscriber;
    void gimbal_subscriber_callback(const dji_sdk::Gimbal gimbal);
    
    dji_sdk::Gimbal gimbal;

    int maxRate;
    int K_p;

public:
    bool gimbal_rateBased_orientation_controller(const float x, const float y, const float z);
   
};
#endif
