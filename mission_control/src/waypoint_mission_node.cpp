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
    WAYPOINT_NAV,
    RETURN_HOME,
    LAND,
    FINISHED
} States;

States nextState(const States &current);
void performTask(const States &current);
droneuse_gimbal*    gimbal;
droneuse_m100*      m100;
uint8_t flight_status;
ros::ServiceClient m100_attitude_control_service; 
dji_sdk::AttitudeControl m100_control_command;
geometry_msgs::PointStamped home_point;
geometry_msgs::PointStamped target_point;
geometry_msgs::PointStamped land_point;
float dist2goal;


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
    m100_attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");

    ros::Subscriber flight_status_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/flight_status", 10, flight_status_subscriber_callback);

    home_point.point.x = 0.0;
    home_point.point.y = 0.0;
    home_point.point.z = 2.0;
    home_point.header.frame_id = "/world";
    home_point.header.stamp = ros::Time::now();
    dist2goal = m100->distance_to_position(home_point);

    target_point.point.x = 3.0;
    target_point.point.y = 3.0;
    target_point.point.z = 2.0;
    target_point.header.frame_id = "/world";
    target_point.header.stamp = ros::Time::now();

    land_point.point.x = 0.0;
    land_point.point.y = 0.0;
    land_point.point.z = 0.0;
    land_point.header.frame_id = "/world";
    land_point.header.stamp = ros::Time::now();

    dist2goal = m100->distance_to_position(home_point);
    while(ros::ok() && currentState != FINISHED)
    {
      /// Execute state machine.
        ros::spinOnce();
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
            if(dist2goal > 0.5)
            {
                return TAKEOFF;
            }
            else
            {
                ROS_INFO("FOLLOW_APRIL");
                return WAYPOINT_NAV;
            }
            break;

        case WAYPOINT_NAV :
            if(dist2goal > 0.5)
            {
                return WAYPOINT_NAV;
            }
            else
            {
                ROS_INFO("LAND");
                return RETURN_HOME;
            }
                break;

        case RETURN_HOME :
            if(dist2goal > 0.5)
            {
                return RETURN_HOME;
            }
            else
            {
                ROS_INFO("LAND");
                return LAND;
            }
            break;


        case LAND :
            if((flight_status == 3))
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
                dist2goal = m100->distance_to_position(home_point);
                m100->set_target_position(home_point);
            }
            break;

        case WAYPOINT_NAV :
            dist2goal = m100->distance_to_position(target_point);
            m100->set_target_position(target_point);
            break;

        case RETURN_HOME :
            dist2goal = m100->distance_to_position(target_point);
            m100->set_target_position(home_point);
            break;

        case LAND :
            dist2goal = m100->distance_to_position(target_point);
            if(dist2goal > 0.3)
                m100->set_target_position(land_point);
            else
                m100->land();
            break;

        case FINISHED :
            break;

      }
 }


