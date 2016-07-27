#include "mission_control/droneuse_m100.h"
#include "mission_control/droneuse_gimbal.h"
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_listener.h>
#include <string>
#define C_PI (double) 3.141592653589793

droneuse_m100::droneuse_m100(ros::NodeHandle& nh)
{
    m100_attitude_control_service       = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");
    m100_task_control_service           = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    m100_arm_control_service            = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
    m100_sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("dji_sdk/sdk_permission_control");
    m100_send_data_service 		= nh.serviceClient<dji_sdk::SendDataToRemoteDevice>("dji_sdk/send_data_to_remote_device");

    m100_local_position_subscriber          = nh.subscribe<dji_sdk::LocalPosition>("dji_sdk/local_position", 10, &droneuse_m100::m100_local_position_subscriber_callback, this);
    m100_flight_status_subscriber 	        = nh.subscribe<std_msgs::UInt8>("dji_sdk/flight_status", 10, &droneuse_m100::m100_flight_status_subscriber_callback, this);
    landing_platform_position_subscriber    = nh.subscribe<geometry_msgs::PointStamped>("droneuse/landing_platform_position", 1, &droneuse_m100::landing_platform_position_subscriber_callback, this);

   m100_target_position_publisher           = nh.advertise<geometry_msgs::PointStamped>("droneuse/m100_target_position", 10);
   m100_target_orientation_publisher        = nh.advertise<geometry_msgs::PointStamped>("droneuse/m100_target_orientation", 10);
   m100_position_control_state_publisher    = nh.advertise<std_msgs::UInt8>("droneuse/m100_position_control_state", 10);
   m100_orientation_control_state_publisher = nh.advertise<std_msgs::UInt8>("droneuse/m100_orientation_control_state", 10);

   tf_listener = new tf::TransformListener;

   land_init_time = ros::Time::now();
   last_platform_detection_time = ros::Time::now();

   sdk_control = false;

   gimbal = new droneuse_gimbal(nh);

   // It's defined a loop rate of 20Hz
   loop_rate = 20;

   apriltagMap_landing_flag = 0;	// 0: No landing, 1: Standard Landing, 2: Custom Landing
   ros::param::get("mission_control/tagMap_tracking_node/land_flag", apriltagMap_landing_flag);
   apriltagMap_chase_height = 2.0;
   ros::param::get("mission_control/tagMap_tracking_node/chase_height", apriltagMap_chase_height);
   apriltagMap_approach_height = 1.5;
   ros::param::get("mission_control/tagMap_tracking_node/approach_height", apriltagMap_approach_height);
   apriltagMap_landing_height = 0.3;
   ros::param::get("mission_control/tagMap_tracking_node/landing_height", apriltagMap_landing_height);
}

void droneuse_m100::m100_local_position_subscriber_callback(const dji_sdk::LocalPosition m100_local_position)
{
    this->local_position = m100_local_position;
}

void droneuse_m100::m100_flight_status_subscriber_callback(const std_msgs::UInt8 flight_status_msg)
{
    this->flight_status = flight_status_msg.data;
}

void droneuse_m100::landing_platform_position_subscriber_callback(const geometry_msgs::PointStamped landing_platform_position)
{
    this->landing_platform_position = landing_platform_position;
    last_platform_detection_time = landing_platform_position.header.stamp;
    gimbal->set_target(landing_platform_position);
}

void droneuse_m100::set_target_position(const geometry_msgs::PointStamped target_position)
{
    if(!sdk_control)
        get_sdk_control();

    m100_target_position_publisher.publish(target_position);
}

void droneuse_m100::set_target_orientation(const geometry_msgs::PointStamped target_orientation)
{
    if(!sdk_control)
        get_sdk_control();

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
    if(!sdk_control)
        get_sdk_control();

    // Control Flag Structure: http://download.dji-innovations.com/downloads/dev/OnboardSDK/Onboard_API_introduction_version_1.0.1_en.pdf

    dji_sdk::AttitudeControl attitude_control;
    attitude_control.request.flag = ctrl_flag;
    attitude_control.request.x = x;
    attitude_control.request.y = y;
    attitude_control.request.z = z;
    attitude_control.request.yaw = yaw;

    return m100_attitude_control_service.call(attitude_control) && attitude_control.response.result;
}

bool droneuse_m100::hover()
{
	if(!sdk_control)
		get_sdk_control();

    // Control Flag Structure: http://download.dji-innovations.com/downloads/dev/OnboardSDK/Onboard_API_introduction_version_1.0.1_en.pdfww
	disable_m100_position_control();
	unsigned char hover_ctrl_flag =   DJI::onboardSDK::Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                    DJI::onboardSDK::Flight::VerticalLogic::VERTICAL_VELOCITY |
                                    DJI::onboardSDK::Flight::YawLogic::YAW_PALSTANCE |
                                    DJI::onboardSDK::Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                    DJI::onboardSDK::Flight::SmoothMode::SMOOTH_ENABLE;

	dji_sdk::AttitudeControl m100_control_command;
	m100_control_command.request.flag	= hover_ctrl_flag;
	m100_control_command.request.x      	= 0.0;
	m100_control_command.request.y      	= 0.0;
	m100_control_command.request.z      	= 0.0;
	m100_control_command.request.yaw    	= 0.0;
	return m100_attitude_control_service.call(m100_control_command) && m100_control_command.response.result;
}

bool droneuse_m100::hover(float height)
{
    if(!sdk_control)
    {
        get_sdk_control();
	ros::Duration(2.0).sleep();
    }
    // Control Flag Structure: http://download.dji-innovations.com/downloads/dev/OnboardSDK/Onboard_API_introduction_version_1.0.1_en.pdf
	disable_m100_position_control();
	unsigned char hover_ctrl_flag = DJI::onboardSDK::Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                    DJI::onboardSDK::Flight::VerticalLogic::VERTICAL_POSITION |
                                    DJI::onboardSDK::Flight::YawLogic::YAW_PALSTANCE |
                                    DJI::onboardSDK::Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                    DJI::onboardSDK::Flight::SmoothMode::SMOOTH_ENABLE;

	dji_sdk::AttitudeControl m100_control_command;
	m100_control_command.request.flag       = hover_ctrl_flag;
	m100_control_command.request.x      	= 0.0;
	m100_control_command.request.y      	= 0.0;
	m100_control_command.request.z      	= height;
	m100_control_command.request.yaw	= 0.0;

	ros::Rate rate(loop_rate);
	while(fabs(get_local_position().z - height) > 0.2)
	{
		m100_attitude_control_service.call(m100_control_command);
		ros::spinOnce();
		rate.sleep();
	}
	return true;
  
}

bool droneuse_m100::get_sdk_control()
{
    dji_sdk::SDKPermissionControl sdk_permission_control;
    sdk_permission_control.request.control_enable = 1;

    sdk_control = (m100_sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result);
    return sdk_control;
}

bool droneuse_m100::release_sdk_control()
{
    dji_sdk::SDKPermissionControl sdk_permission_control;
    sdk_permission_control.request.control_enable = 0;
    
    std::cout << m100_sdk_permission_control_service.call(sdk_permission_control) << std::endl;
	std::cout << sdk_permission_control.response.result << std::endl;

    // If sdk control is released, then sdk_control should be false
    sdk_control = !(m100_sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result);
    return !sdk_control;
}

bool droneuse_m100::arm()
{
    if(!sdk_control)
        get_sdk_control();

    dji_sdk::DroneArmControl drone_arm_control;
    drone_arm_control.request.arm = 1;
    return m100_arm_control_service.call(drone_arm_control) && drone_arm_control.response.result;
}

bool droneuse_m100::disarm()
{
    if(!sdk_control)
        get_sdk_control();

    dji_sdk::DroneArmControl drone_arm_control;
    drone_arm_control.request.arm = 0;
    return m100_arm_control_service.call(drone_arm_control) && drone_arm_control.response.result;
}

bool droneuse_m100::takeoff()
{
    if(!sdk_control)
        get_sdk_control();
    
    dji_sdk::DroneTaskControl drone_task_control;
    drone_task_control.request.task = 4;
    return m100_task_control_service.call(drone_task_control) && drone_task_control.response.result;
}

bool droneuse_m100::wait_to_takeoff()
{
    ros::Rate rate(loop_rate);
	// flight_status == 3 means drone has taken off
	while(get_flight_status() != 3)
	{
		ros::spinOnce();
		rate.sleep();
	}
    return true;
}

bool droneuse_m100::land()
{
    if(!sdk_control)
        get_sdk_control();
    
    dji_sdk::DroneTaskControl drone_task_control;
    drone_task_control.request.task = 6;
    return m100_task_control_service.call(drone_task_control) && drone_task_control.response.result;
}

bool droneuse_m100::custom_land()
{
	if(!sdk_control)
		get_sdk_control();

	ros::Rate rate(loop_rate);
    
    unsigned char land_ctrl_flag = DJI::onboardSDK::Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                    DJI::onboardSDK::Flight::VerticalLogic::VERTICAL_THRUST |
                                    DJI::onboardSDK::Flight::YawLogic::YAW_PALSTANCE |
                                    DJI::onboardSDK::Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                    DJI::onboardSDK::Flight::SmoothMode::SMOOTH_ENABLE;

	dji_sdk::AttitudeControl m100_control_command;
	m100_control_command.request.flag	= land_ctrl_flag;
	m100_control_command.request.x      	= 0.0;
	m100_control_command.request.y      	= 0.0;
	m100_control_command.request.z      	= 10.0;
	m100_control_command.request.yaw    	= 0.0;

	ros::Time init_descent = ros::Time::now();
	while(ros::Duration(ros::Time::now() - init_descent) < ros::Duration(2.0))
	{
		m100_attitude_control_service.call(m100_control_command);
		ros::spinOnce();
		rate.sleep();
	}
	land();
	disarm();
	return release_sdk_control();
	
}

float droneuse_m100::distance_to_position(geometry_msgs::PointStamped position)
{
    ros::Time time_now = ros::Time::now();
    try
    {
        tf_listener->waitForTransform("/body_frame", position.header.frame_id, time_now, ros::Duration(0.1));
	geometry_msgs::PointStamped position_transformed;
        tf_listener->transformPoint("/body_frame", time_now, position, position.header.frame_id, position_transformed);
        return (float) sqrt(position_transformed.point.x*position_transformed.point.x + position_transformed.point.y*position_transformed.point.y + position_transformed.point.z*position_transformed.point.z);
    }
    catch(tf::TransformException ex)
    {
            ROS_ERROR("%s", ex.what());
            return 0;
    }
}

float droneuse_m100::distance_to_position(geometry_msgs::Point point, std::string frame_id)
{
    //TODO Create unestamped data type
    ros::Time time_now = ros::Time::now();
    try
    {
        geometry_msgs::PointStamped position;
        position.point.x = point.x;
        position.point.y = point.y;
        position.point.z = point.z;
        position.header.frame_id = frame_id;
        position.header.stamp = time_now;
        tf_listener->waitForTransform("/body_frame", position.header.frame_id, time_now, ros::Duration(0.1));
        tf_listener->transformPoint("/body_frame", time_now, position, "/world", position);
        return (float) sqrt(position.point.x*position.point.x + position.point.y*position.point.y + position.point.z*position.point.z);
    }
    catch(tf::TransformException ex)
    {
            ROS_ERROR("%s", ex.what());
            return 0;
    }
}

float droneuse_m100::horizontal_distance_to_position(geometry_msgs::PointStamped position)
{
    ros::Time time_now = ros::Time::now();
    try
    {
        tf_listener->waitForTransform("/body_frame", position.header.frame_id, time_now, ros::Duration(0.1));
	geometry_msgs::PointStamped position_transformed;
        tf_listener->transformPoint("/body_frame", time_now, position, position.header.frame_id, position_transformed);
        return (float) sqrt(position_transformed.point.x*position_transformed.point.x + position_transformed.point.y*position_transformed.point.y);
    }
    catch(tf::TransformException ex)
    {
            ROS_ERROR("%s", ex.what());
            return 0;
    }
}

float droneuse_m100::vertical_distance_to_position(geometry_msgs::PointStamped position)
{
    ros::Time time_now = ros::Time::now();
    try
    {
        tf_listener->waitForTransform("/body_frame", position.header.frame_id, time_now, ros::Duration(0.1));
        geometry_msgs::PointStamped position_transformed;
        tf_listener->transformPoint("/body_frame", time_now, position, position.header.frame_id, position_transformed);
        return (float) fabs(position_transformed.point.z);
    }
    catch(tf::TransformException ex)
    {
            ROS_ERROR("%s", ex.what());
            return 0;
    }
}

dji_sdk::LocalPosition droneuse_m100::get_local_position()
{
    return local_position;
}

uint8_t droneuse_m100::get_flight_status()
{
	return flight_status;
}

bool droneuse_m100::follow_waypoint(std::vector<geometry_msgs::PointStamped> waypoint_list)
{
    if(!sdk_control)
        get_sdk_control();
    
    ros::Rate rate(loop_rate);
    if(waypoint_list.size() > 0)
    {
        for(size_t k = 0; k < waypoint_list.size(); k++)
        {   
            waypoint_list[k].header.stamp = ros::Time::now();
            set_target_position(waypoint_list[k]);
            while(distance_to_position(waypoint_list[k]) > 0.4)
            {
                // Loop until target is reached
		ros::spinOnce();
		rate.sleep();
            }
        }
    }
    else
        return false;
    return true;
}

bool droneuse_m100::perform_landing_maneuver()
{
    if(!sdk_control)
        get_sdk_control();
    
    bool land_flag = false;
    ros::Time init_time = ros::Time::now();
    ros::Rate rate(loop_rate);
    
    // Perform the maneuver until the drone has succesfully landed, it has elapsed more than 30 seconds or no landing_platform has been detected for a while
    while( !land_flag && (ros::Time::now() - init_time) < ros::Duration(100) )
    {
	if((ros::Time::now() - last_platform_detection_time) > ros::Duration(3.0) )
	{
		hover();
		gimbal->set_pitch(45.0);
	}
	else
	{
		if(chase())
		{
			if(approach())
			{
				if(apriltagMap_landing_flag == 0)
					land_flag = false;
				else if (apriltagMap_landing_flag == 1)
					land_flag = land();
				else if(apriltagMap_landing_flag == 2)
					land_flag = landing();
				else
					land_flag = true;
			}
		}
	}

	ros::spinOnce();
	rate.sleep();
    }
    return land_flag;
}

// Returns true if this stage is completed
bool droneuse_m100::chase()
{
    bool chase_flag = false;
    if( (horizontal_distance_to_position(landing_platform_position) < 3.0) && (vertical_distance_to_position(landing_platform_position) < apriltagMap_chase_height*1.25) )
        chase_flag = true;
    else
    {
	std::cout << "chasing..." << std::endl;

        geometry_msgs::PointStamped commanded_target;
	commanded_target.header.stamp = landing_platform_position.header.stamp;
	commanded_target.header.frame_id = landing_platform_position.header.frame_id;
        commanded_target.point.x = landing_platform_position.point.x;
        commanded_target.point.y = landing_platform_position.point.y;
        commanded_target.point.z = (landing_platform_position.point.z - apriltagMap_chase_height);

        set_target_position(commanded_target);
    }
    return chase_flag;
}

// Returns true if this stage is completed
bool droneuse_m100::approach()
{
    bool approach_flag = false;
    if( (horizontal_distance_to_position(landing_platform_position) < 1.5) && (vertical_distance_to_position(landing_platform_position) < apriltagMap_approach_height*1.25) && (apriltagMap_landing_flag == 1 || apriltagMap_landing_flag == 2) )
        approach_flag = true; 
    else
    {
	std::cout << "approaching..." << std::endl;

        geometry_msgs::PointStamped commanded_target;
	commanded_target.header.stamp = landing_platform_position.header.stamp;
	commanded_target.header.frame_id = landing_platform_position.header.frame_id;
        commanded_target.point.x = landing_platform_position.point.x;
        commanded_target.point.y = landing_platform_position.point.y;
        commanded_target.point.z = (landing_platform_position.point.z - apriltagMap_approach_height);

        set_target_position(commanded_target);
    }
    return approach_flag;
}

// Returns true if landing is performed successfully
bool droneuse_m100::landing()
{
    bool landing_flag = false;
    geometry_msgs::PointStamped commanded_target;
    commanded_target.header.stamp = landing_platform_position.header.stamp;
    commanded_target.header.frame_id = landing_platform_position.header.frame_id;
    commanded_target.point.x = landing_platform_position.point.x;
    commanded_target.point.y = landing_platform_position.point.y;
    commanded_target.point.z = (landing_platform_position.point.z - apriltagMap_landing_height);

    set_target_position(commanded_target);

    if( (horizontal_distance_to_position(landing_platform_position) < 0.3) && (vertical_distance_to_position(landing_platform_position) < apriltagMap_landing_height*1.5) )
    {
        if( ros::Duration(ros::Time::now() - land_init_time) > ros::Duration(10.0) )
            land_init_time = ros::Time::now();
        else if( ros::Duration(ros::Time::now() - land_init_time) > ros::Duration(1.0) )
            landing_flag = custom_land();
    }
    return landing_flag;
}

