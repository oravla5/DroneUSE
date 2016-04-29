#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk.h>
#include <std_msgs/UInt8.h>
#include <mission_control/droneuse_m100.h>
#include <mission_control/droneuse_gimbal.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace DJI::onboardSDK;

/// State machine of the followme mission
typedef enum
{
    INIT_MISSION,
    TAKEOFF,
    FOLLOW_APRIL,
    LAND,
    FINISHED
} States;

States nextState(const States &current);
void performTask(const States &current);
droneuse_gimbal*    gimbal;
droneuse_m100*      m100;
ros::Time time_start;
bool follow_april_flag = false;
uint8_t flight_status;
ros::ServiceClient m100_attitude_control_service; 
dji_sdk::AttitudeControl m100_control_command;
unsigned char control_flag;
tf::TransformListener* tf_listener;
ros::Time last_april_time;
geometry_msgs::PointStamped default_gimbal;

void received_data_callback(dji_sdk::TransparentTransmissionData msg)
{
	cout << "msg " << (int)msg.data[0] << endl;
}

void apriltag_subscriber_callback(geometry_msgs::PointStamped apriltag_position_msg)
{
	if(m100->sendData(1))
		cout << "Successs \n";

        gimbal->set_target(apriltag_position_msg);
    last_april_time = ros::Time::now();
    if(follow_april_flag)
    {
        m100->set_target_orientation(apriltag_position_msg);

	try{
	tf_listener->waitForTransform("/ground_frame",apriltag_position_msg.header.frame_id, apriltag_position_msg.header.stamp, ros::Duration(2.0/15));
	tf_listener->transformPoint("/ground_frame", apriltag_position_msg.header.stamp, apriltag_position_msg,apriltag_position_msg.header.frame_id ,apriltag_position_msg);	
        geometry_msgs::PointStamped target_position;
	tf::Vector3 target_position_vector;
        tf::Vector3 apriltag_position_vector (	apriltag_position_msg.point.x,
                        			apriltag_position_msg.point.y,
                        			apriltag_position_msg.point.z);

        target_position_vector = apriltag_position_vector - apriltag_position_vector.normalized()*2.0;

        target_position.header.frame_id = apriltag_position_msg.header.frame_id;
        target_position.header.stamp = apriltag_position_msg.header.stamp;
        target_position.point.x = (float)target_position_vector.x();
        target_position.point.y = (float)target_position_vector.y();
        target_position.point.z = (float) apriltag_position_msg.point.z - 0.25; 
        m100->set_target_position(target_position);
}
catch(tf::TransformException ex)
{
ROS_ERROR("%s", ex.what());
}
}
    }


void flight_status_subscriber_callback(std_msgs::UInt8 flight_status_msg)
{
    flight_status = flight_status_msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    ros::Rate rate(15);
    States currentState = INIT_MISSION;
    gimbal  = new droneuse_gimbal(nh);
    m100    = new droneuse_m100(nh);
    tf_listener = new tf::TransformListener;
    
     m100_attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");

    ros::Subscriber apriltag_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/tag_position",10, apriltag_subscriber_callback);
    
    ros::Subscriber flight_status_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/flight_status", 10, flight_status_subscriber_callback);

	ros::Subscriber received_data_subscriber = nh.subscribe<dji_sdk::TransparentTransmissionData>("dji_sdk/data_received_from_remote_device", 10, received_data_callback);	

    last_april_time = ros::Time::now();
    
    default_gimbal.header.frame_id  = "/body_frame";
    default_gimbal.point.x          = 10.0;
    default_gimbal.point.y          = 0.0;
    default_gimbal.point.z          = 0.0;

    m100->set_target_orientation(default_gimbal);

    while(ros::ok() && currentState != FINISHED)
    {
      /// Execute state machine.
        ros::spinOnce();
        performTask(currentState);
        currentState = nextState(currentState);
        if ((ros::Time::now() - last_april_time) > ros::Duration(3.0))
        {
            default_gimbal.header.stamp = ros::Time::now();
            gimbal->set_target(default_gimbal);
        }
	//std::cout << "flight status " << int(flight_status) << "\n";
        rate.sleep();
    }
    return 0;
}


States nextState(const States &current)
{
    switch(current)
    {
        case INIT_MISSION :
            if(flight_status == 3)
            {
                ROS_INFO("TAKEOFF");
                if(m100->get_sdk_control())
                    printf("\n Control Acquired \n");
                time_start = ros::Time::now();
                return TAKEOFF;
            }
            else
            {
                return INIT_MISSION;
            }
            break;
        case TAKEOFF :
            if((ros::Time::now() - time_start) < ros::Duration(0.3))
            {
                return TAKEOFF;
            }
            else
            {
                ROS_INFO("FOLLOW_APRIL");
                time_start = ros::Time::now();
                return FOLLOW_APRIL;
            }
            break;

        case FOLLOW_APRIL :
            if(!(flight_status == 4))
            {
                return FOLLOW_APRIL;
            }
            else
            {
                ROS_INFO("LAND");
                time_start = ros::Time::now();
                return LAND;
            }
            break;


        case LAND :
            if((ros::Time::now() - time_start) < ros::Duration(10))
            {
                ROS_INFO("LAND");
                return LAND;
            }
            else
            {
                if(m100->disarm())
                    printf("\n Disarmed \n");
                if(m100->release_sdk_control())
                    printf("\n Control Released \n");
                return FINISHED;
            }
            break;

        case FINISHED :
            return FINISHED;

        default :
            return FINISHED;
    }
}


void performTask(const States &current)
{
   switch(current)
   {
        case INIT_MISSION : 
            break;

        case TAKEOFF :
            m100->takeoff();
            break;

        case FOLLOW_APRIL :
            if(!follow_april_flag)
                follow_april_flag = true;

            break;

        case LAND :
            if(follow_april_flag)
                follow_april_flag =  false;
            m100->land();
            break;

        case FINISHED :
            m100->disarm();

      }
   }


