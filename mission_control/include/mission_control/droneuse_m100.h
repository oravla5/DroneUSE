#ifndef DRONEUSE_M100_H 
#define DRONEUSE_M100_H 
#include <ros/ros.h> 
#include <dji_sdk/dji_sdk.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt8.h>
#include <string>

class droneuse_m100
{
    private:
        ros::ServiceClient m100_attitude_control_service;
        ros::ServiceClient m100_task_control_service;
        ros::ServiceClient m100_arm_control_service;
        ros::ServiceClient m100_sdk_permission_control_service;
	ros::ServiceClient m100_send_data_service;

        ros::Subscriber m100_local_position_subscriber;
        ros::Subscriber m100_flight_status_subscriber;

        ros::Publisher m100_target_position_publisher;
        ros::Publisher m100_target_orientation_publisher;
        ros::Publisher m100_position_control_state_publisher;
        ros::Publisher m100_orientation_control_state_publisher;
        
        void m100_local_position_subscriber_callback(const dji_sdk::LocalPosition m100_local_position);
	void m100_flight_status_subscriber_callback(std_msgs::UInt8 flight_status_msg);
        
        dji_sdk::LocalPosition      m100_local_position;
        geometry_msgs::PointStamped m100_target_position;
	uint8_t m100_flight_status;

        tf::TransformListener* tf_listener;

    public:
        droneuse_m100(ros::NodeHandle& nh);
        
        void set_target_position(geometry_msgs::PointStamped target_position);
	bool hover();
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
        float distance_to_position(geometry_msgs::PointStamped position);
	uint8_t get_flight_status();
        float distance_to_position(geometry_msgs::Point point, std::string frame_id);

	bool sendData(unsigned char data);

        dji_sdk::LocalPosition  get_local_position();
};

#endif
