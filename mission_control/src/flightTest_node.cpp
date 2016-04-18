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
float dist2goal = 0.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    States currentState = INIT_MISSION;
    drone = new DJIDrone(nh);
    target_position.x = drone->local_position.x + 10;
    target_position.y = 0;
    target_position.z = 5;
    while(ros::ok())
    {
      /// Execute state machine.
        ros::spinOnce();
        performTask(currentState);
        currentState = nextState(currentState);
        rate.sleep();
    }
    return 0;
}


States nextState(const States &current)
{
    switch(current)
    {
        case INIT_MISSION :
            if(drone->flight_status == 3)
            {
                ROS_INFO("TAKEOFF");
                if(drone->request_sdk_permission_control())
                    printf("\n Permission Control Acquired \n");
                return TAKEOFF;
            }
            else
            {
                return INIT_MISSION;
            }
            break;
        case TAKEOFF :
            if(drone->local_position.z < 1)
            {
                return TAKEOFF;
            }
            else
            {
                ROS_INFO("WAYPOINT_NAV");
                return WAYPOINT_NAV;
            }
            break;

        case WAYPOINT_NAV :
            dist2goal = sqrt((drone->local_position.x - target_position.x)*(drone->local_position.x - target_position.x) + (drone->local_position.y - target_position.y)*(drone->local_position.y - target_position.y) + (drone->local_position.z - target_position.z)*(drone->local_position.z - target_position.z));

            if(abs(dist2goal) < 0.5)
            {
                ROS_INFO("RETURN_HOME");
                return RETURN_HOME;
            }
            else
            {
                return WAYPOINT_NAV;
            }
            break;

        case RETURN_HOME :
            dist2goal = sqrt((drone->local_position.x - home_position.x)*(drone->local_position.x - home_position.x) + (drone->local_position.y - home_position.y)*(drone->local_position.y - home_position.y) + (drone->local_position.z - home_position.z)*(drone->local_position.z - home_position.z));

            //cout << dist2goal << "\n";
            if(abs(dist2goal) < 0.5)
            {
                ROS_INFO("LAND");
                return LAND;
            }
            else
            {
                return RETURN_HOME;
            }
            break;

        case LAND :
            if(drone->local_position.z < 0.1)
            {
                ROS_INFO("FINISHED");
                return FINISHED;
            }
            else
            {
                return LAND;
            }
            break;

        case FINISHED :
            return INIT_MISSION;

        default :
            return FINISHED;
    }
}


void performTask(const States &current)
{
   switch(current)
   {
        case INIT_MISSION : 
            home_position.x = drone->local_position.x;
            home_position.y = drone->local_position.y;
            break;

        case TAKEOFF :
            drone->takeoff();
            break;

        case WAYPOINT_NAV :
            home_position.z = drone->local_position.z;
            drone->local_position_navigation_send_request(target_position.x,target_position.y,target_position.z,0);
            break;

        case RETURN_HOME :
            drone->local_position_navigation_send_request(home_position.x,home_position.y,home_position.z,0);
            break;

        case LAND :
            drone->landing();
            break;

        case FINISHED :
            drone->drone_disarm();

      }
   }


