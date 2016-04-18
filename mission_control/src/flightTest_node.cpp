#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <dji_sdk/dji_drone.h>


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
DJIDrone* drone;
dji_sdk::LocalPosition target_position;
dji_sdk::LocalPosition home_position;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    States currentState = INIT_MISSION;
    drone = new DJIDrone(nh);
    target_position.x = 3;
    target_position.y = 0;
    target_position.z = 3;
    while(ros::ok())
    {
      /// Execute state machine.
        ros::spinOnce();
        performTask(currentState);
        currentState = nextState(currentState);
        rate.sleep();
    }
/*
    drone->drone_arm();
    cout << "Taking off\n";
    sleep(8);
    cout << "Ascend\n";
    drone->local_position_navigation_send_request(0,0,flying_height);
    sleep(3);
    cout << "Go Waypoint\n";
    drone->local_position_navigation_send_request(50,30,flying_height);
    sleep(8);
    cout << "Go Home\n";
    drone->gohome();
    sleep(8);
    cout << "Land\n";
    drone->landing();
    sleep(8);
    cout << "Disarm\n";
    drone->drone_disarm();    
    
*/
   //ros::shutdown();
    return 0;
   //return EXIT_SUCCESS;
}


States nextState(const States &current)
{
    switch(current)
    {
        case INIT_MISSION:
        {
            if(drone->flight_status == 3)
                return TAKEOFF;
            else
                return INIT_MISSION;
        }
        case TAKEOFF:
        {
            if(drone->local_position.z < 1)
                return TAKEOFF;
            else
                return WAYPOINT_NAV;
            break;
        }
        case WAYPOINT_NAV:
        {
            float dist2goal = sqrt((drone->local_position.x - target_position.x)*(drone->local_position.x - target_position.x) + (drone->local_position.y - target_position.y)*(drone->local_position.y - target_position.y) + (drone->local_position.z - target_position.z)*(drone->local_position.z - target_position.z));

            if(abs(dist2goal) < 0.5)
                return RETURN_HOME;
            else
                return WAYPOINT_NAV;
            break;
        }
        case RETURN_HOME:
        {
            float dist2goal = sqrt((drone->local_position.x - home_position.x)*(drone->local_position.x - home_position.x) + (drone->local_position.y - home_position.y)*(drone->local_position.y - home_position.y) + (drone->local_position.z - home_position.z)*(drone->local_position.z - home_position.z));

            if(abs(dist2goal) < 0.5)
                return LAND;
            else
                return RETURN_HOME;
            break;
        }
        case LAND:
        {
            if(drone->local_position.z < 0.1)
                return FINISHED;
            else
                return LAND;
            break;
        }
        case FINISHED:
        {
            return FINISHED;
            break;
        }
        default:
        {
            return FINISHED;
            break;
        }
    }
}



void performTask(const States &current)
{
   switch(current)
   {
        case INIT_MISSION: 
        {
            ROS_INFO("INIT_MISSION");
            home_position.x = drone->local_position.x;
            home_position.y = drone->local_position.y;
            break;
        }
        case TAKEOFF:
        {
            if(drone->request_sdk_permission_control())
                printf("\n Permission Control Acquired \n");
            ROS_INFO("TAKEOFF");
            drone->takeoff();
            break;
        }
        case WAYPOINT_NAV:
        {
            home_position.z = drone->local_position.z;
            cout << "Navigating... ";
            drone->local_position_navigation_send_request(target_position.x,target_position.y,target_position.z);
            cout << "Waypoint Reached\n";
        }
        case RETURN_HOME:
        {
            cout << "Returning home... ";
            drone->local_position_navigation_send_request(home_position.x,home_position.y,home_position.z);
            cout << "Home point reached\n";
        }
        case LAND:
        {
            cout << "Landing... ";
            drone->landing();
            cout << "Ground\n";
            break;
        }
        case FINISHED:
        {
            drone->drone_disarm();
            cout << "Disarmed\n";
        }
        default:
      {
         /// Nothing to do

         break;
      }
   }
}


