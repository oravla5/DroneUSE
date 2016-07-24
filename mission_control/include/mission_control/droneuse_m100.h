#ifndef DRONEUSE_M100_H 
#define DRONEUSE_M100_H 

#include <cstdlib>
#include <stdio.h>
#include <ros/ros.h> 
#include <mission_control/droneuse_gimbal.h>
#include <dji_sdk/dji_sdk.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt8.h>
#include <string>

class droneuse_m100
{
    private:
        // Service Clients
        ros::ServiceClient m100_attitude_control_service;
        ros::ServiceClient m100_task_control_service;
        ros::ServiceClient m100_arm_control_service;
        ros::ServiceClient m100_sdk_permission_control_service;
        ros::ServiceClient m100_send_data_service;

        // Subscribers
        ros::Subscriber m100_local_position_subscriber;
        ros::Subscriber m100_flight_status_subscriber;
        ros::Subscriber landing_platform_position_subscriber;

        // Publishers
        ros::Publisher m100_target_position_publisher;
        ros::Publisher m100_target_orientation_publisher;
        ros::Publisher m100_position_control_state_publisher;
        ros::Publisher m100_orientation_control_state_publisher;
        
        
        dji_sdk::LocalPosition      local_position;
        uint8_t                     flight_status;
        geometry_msgs::PointStamped landing_platform_position;
        bool                        sdk_control;

        tf::TransformListener* tf_listener;
        ros::Time land_init_time;
        ros::Time last_platform_detection_time;
        int loop_rate;
	int apriltagMap_landing_flag;
	double apriltagMap_landing_height;
	double apriltagMap_approach_height;
	double apriltagMap_chase_height;
    public:
        droneuse_m100(ros::NodeHandle& nh);

        // SDK control over m100
        bool get_sdk_control();
        bool release_sdk_control();

        // Direct Position and Orientation Control Commands
        void set_target_position(geometry_msgs::PointStamped target_position);
        void set_target_orientation(geometry_msgs::PointStamped target_orientation);
        void disable_m100_position_control();
        void disable_m100_orientation_control();
        bool attitude_control(unsigned char ctrl_flag, float x, float y, float z, float yaw);
        
        // Position Control via waypoints
        bool follow_waypoint(std::vector<geometry_msgs::PointStamped> waypoint_list);      // Perform the navigation through the points determined by the waypoint list 
        bool follow_waypoint(std::vector<geometry_msgs::PoseStamped> waypoint_list);       //Perform the navigation through the poses determined by the waypoint list

        // Taking Off and Landing commands
        bool takeoff();
        bool custom_takeoff();
        bool wait_to_takeoff();
        bool land();
        bool custom_land();

        // Activation and Deactivatiion of the engines
        bool arm();
        bool disarm();

        // Complex Landing Maneuver
        bool perform_landing_maneuver();

    	bool hover();
	bool hover(float height);

        // M100 Status commands
        float distance_to_position(geometry_msgs::PointStamped position);
        float distance_to_position(geometry_msgs::Point point, std::string frame_id);
        float horizontal_distance_to_position(geometry_msgs::PointStamped position);
        float vertical_distance_to_position(geometry_msgs::PointStamped position);
        uint8_t get_flight_status();
        dji_sdk::LocalPosition  get_local_position();

        bool sendData(unsigned char data);

	droneuse_gimbal *gimbal;

    private:
        // Subscribers Callbacks
        void m100_local_position_subscriber_callback(const dji_sdk::LocalPosition m100_local_position);
        void m100_flight_status_subscriber_callback(const std_msgs::UInt8 flight_status_msg);
        void landing_platform_position_subscriber_callback(const geometry_msgs::PointStamped landing_platform_position);

        // Landing Maneuver Stages
        bool chase();       // Initial approaching maneuver to the landing platform once it has been detected
        bool approach();    // Descension maneuver to the landing platform once the vehicle is close enough
        bool landing();     // Final part of the landing maneuver when throttle is cut off

};

#endif
