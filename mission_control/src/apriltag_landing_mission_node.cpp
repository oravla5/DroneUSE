// LANDING NODE
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
    SEARCH,
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
ros::Time last_apriltag_time;

tf::TransformListener* tf_listener;
geometry_msgs::Pointstamped last_apriltag_postion;
geometry_msgs::PointStamped takeoff_goal;
geometry_msgs::PointStamped body_origin;
geometry_msgs::PointStamped local_position;

bool april_detection = false;

ros::Time time_now;
int control_rate = 10;

void apriltag_subscriber_callback(geometry_msgs::PointStamped apriltag_position_msg)
{
    last_apriltag_position = apriltag_position_msg;
    last_apriltag_time = last_apriltag_position.header.stamp;
    aril_detection = true;
}

void flight_status_subscriber_callback(std_msgs::UInt8 flight_status_msg)
{
    flight_status = flight_status_msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    ros::Rate rate(control_rate);
    States currentState = INIT_MISSION;
    gimbal  = new droneuse_gimbal(nh);
    m100    = new droneuse_m100(nh);
    tf_listener = new tf::TransformListener;
    
    geometry_msgs::PointStamped body_origin;
    geometry_msgs::PointStamped takeoff_goal;

    // Take Off goal point
    takeoff_goal.header.frame_id = "/world";
    takeoff_goal.header.stamp = ros::Time::now();
    takeoff_goal.point.x = 0.0;
    takeoff_goal.point.y = 0.0;
    takeoff_goal.point.z = -2.0;
    // Body frame
    body_origin.header.frame_id = "/body_frame";
    body_origin.header.stamp = ros::Time::now();
    body_origin.point.x = 0.0;
    body_origin.point.y = 0.0;
    body_origin.point.z = 0.0;

    target_gimbal_orientation.frame_id = "/body_frame";
    target_gimbal.header.stamp = ros::Time::now();
    target_gimbal.point.x = 1.0;
    target_gimbal.point.z = 1.0;
    target_gimbal.point.x = 0.0;
    
    // Position where the apriltag is setted 
   
     m100_attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");

    ros::Subscriber apriltag_subscriber = nh.subscribe<geometry_msgs::PointStamped>("droneuse/tag_position",10, apriltag_subscriber_callback);
    
    ros::Subscriber flight_status_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/flight_status", 10, flight_status_subscriber_callback);

    
    while(ros::ok() && currentState != FINISHED)
    {
      /// Execute state machine.
        ros::spinOnce();
        time_now = ros::Time::now();
        performTask(currentState);
        currentState = nextState(currentState);
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
                return TAKEOFF;
            }
            else
            {
                return INIT_MISSION;
            }
            break;
        case TAKEOFF :
            if(height < 1.9)
            {
                return TAKEOFF;
            }
            else
            {
                ROS_INFO("SEARCH");
                return SEARCH;
            }
            break;

        case SEARCH :
            if(!(flight_status == 4))
            {
                return SEARCH;
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
            if(!(flight_status == 3))
                m100->takeoff();
            else
            {
                m100->set_target_position(takeoff_goal);
                body_origin.header.stamp = time_now;
                tf_listener->waitForTransform("/world", "/body_frame", time_now , ros::Duration(2.0/control_rate));
                tf_listener->transformPoint("/world", time_now, body_origin, "/body_frame", local_position);
                height = abs(local_position.point.z);
            }
            break;

        case SEARCH :
            if(!april_detection || ((ros::Time::now() - last_apriltag_time) > 1))
            {
                target_gimbal_orientation.frame_id = "/body_frame";
                target_gimbal.header.stamp = ros::Time::now();
                target_gimbal.point.x = 1.0;
                target_gimbal.point.z = 1.0;
                target_gimbal.point.x = 0.0;
                gimbal->set_target(target_gimbal);

                target_position.header.frame_id = "/world";
                target_position.header.stamp = ros::Time::now();
                target_position.point.x = 3.0;
                target_position.point.y = 0.0;
                target_position.point.z = -2.0;
                m100->set_target_position(target_position);
                m100->set_target_orientation(target_position);
                april_detection = false;
            }
            else
            {
                target_gimbal_orientation.
                tf_listener->waitForTransform("/world", last_apriltag_position.header.frame_id, last_apriltag_time , ros::Duration(2.0/control_rate));
                tf_listener->transformPoint("/world", last_apriltag_time, last_apriltag_position, "/world", target_position);
                target_position.point.z = target_position.point.z - 2.0;
                m100->set_target_position(target_position);
                m100->set_target_orientation(target_position);
            }

            tf_listener->waitForTransform("/body_frame", target_position.header.frame_id, target_position.header.stamp , ros::Duration(2.0/control_rate));
            tf_listener->transformPoint("/body_frame", target_position.header.stamp, , "/world", target_position);


            dist2goal = sqrt((target_position.point.x))
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


