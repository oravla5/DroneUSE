#include "mission_control/droneuse_m100.h"
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <std_msgs/UInt8.h>
#define C_PI (double) 3.141592653589793

void droneuse_m100::m100_local_position_subscriber_callback(const dji_sdk::LocalPosition m100_local_position)
{
    this->m100_local_position = m100_local_position;
}

void droneuse_m100::set_target_position(geometry_msgs::PointStamped target_position)
{
    m100_target_position_publisher.publish(target_position);
}

void droneuse_m100::set_target_orientation(geometry_msgs::PointStamped target_orientation)
{
    m100_target_orientation_publisher.publish(target_orientation);
}

void droneuse_m100::disable_m100_position_control()
{
    std_msgs::UInt8 m100_control_msgs;
    m100_control_msgs.data = false;

    m100_position_control_state_publisher.publish(m100_control_msgs);
}

void droneuse_m100::disable_m100_orientation_control()
{
    std_msgs::UInt8 m100_control_msgs;
    m100_control_msgs.data = false;

    m100_orientation_control_state_publisher.publish(m100_control_msgs);
}

droneuse_m100::droneuse_m100(ros::NodeHandle& nh)
{
    m100_attitude_control_service       = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");
    m100_task_control_service           = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    m100_arm_control_service            = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
    m100_sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("dji_sdk/sdk_permission_control");
    m100_send_data_service 		= nh.serviceClient<dji_sdk::SendDataToRemoteDevice>("dji_sdk/send_data_to_remote_device");
    

    m100_local_position_subscriber      = nh.subscribe<dji_sdk::LocalPosition>("dji_sdk/local_position", 10, &droneuse_m100::m100_local_position_subscriber_callback, this);

   m100_target_position_publisher           = nh.advertise<geometry_msgs::PointStamped>("droneuse/m100_target_position", 10);
   m100_target_orientation_publisher        = nh.advertise<geometry_msgs::PointStamped>("droneuse/m100_target_orientation", 10);
   m100_position_control_state_publisher    = nh.advertise<std_msgs::UInt8>("droneuse/m100_position_control_state", 10);
   m100_orientation_control_state_publisher = nh.advertise<std_msgs::UInt8>("droneuse/m100_orientation_control_state", 10);

}

bool droneuse_m100::sendData(unsigned char data)
{
	dji_sdk::SendDataToRemoteDevice send_msg;

	std::vector<unsigned char> send_data;
	send_data.push_back(data);

	send_msg.request.data = send_data;
	
	return m100_send_data_service.call(send_msg) && send_msg.response.result; 
}

bool droneuse_m100::attitude_control(unsigned char ctrl_flag, float x, float y, float z, float yaw)
{
    // Control Flag Structure: http://download.dji-innovations.com/downloads/dev/OnboardSDK/Onboard_API_introduction_version_1.0.1_en.pdf

    dji_sdk::AttitudeControl attitude_control;
    attitude_control.request.flag = ctrl_flag;
    attitude_control.request.x = x;
    attitude_control.request.y = y;
    attitude_control.request.z = z;
    attitude_control.request.yaw = yaw;

    return m100_attitude_control_service.call(attitude_control) && attitude_control.response.result;
}
bool droneuse_m100::get_sdk_control()
{

    dji_sdk::SDKPermissionControl sdk_permission_control;
    sdk_permission_control.request.control_enable = 1;
    
    return m100_sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result;
}
bool droneuse_m100::release_sdk_control()
{
    dji_sdk::SDKPermissionControl sdk_permission_control;
    sdk_permission_control.request.control_enable = 0;
    
    std::cout << m100_sdk_permission_control_service.call(sdk_permission_control) << std::endl;
	std::cout << sdk_permission_control.response.result << std::endl;

    return m100_sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result;
}
bool droneuse_m100::arm()
{
    dji_sdk::DroneArmControl drone_arm_control;
    drone_arm_control.request.arm = 1;
    return m100_arm_control_service.call(drone_arm_control) && drone_arm_control.response.result;
}
bool droneuse_m100::disarm()
{
    dji_sdk::DroneArmControl drone_arm_control;
    drone_arm_control.request.arm = 0;
    return m100_arm_control_service.call(drone_arm_control) && drone_arm_control.response.result;
}
bool droneuse_m100::takeoff()
{
    dji_sdk::DroneTaskControl drone_task_control;
    drone_task_control.request.task = 4;
    return m100_task_control_service.call(drone_task_control) && drone_task_control.response.result;
}

bool droneuse_m100::land()
{
    dji_sdk::DroneTaskControl drone_task_control;
    drone_task_control.request.task = 6;
    return m100_task_control_service.call(drone_task_control) && drone_task_control.response.result;
}

dji_sdk::LocalPosition droneuse_m100::get_local_position()
{
    return m100_local_position;
}
