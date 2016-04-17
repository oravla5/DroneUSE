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
States currentState = INIT_MISSION;
bool init_flag = false;
dji_sdk::MissionWaypoint home_wp;
dji_sdk::MissionWaypointTask waypoint_task;
float flying_height = 7.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    ros::Rate rate(20);

    drone = new DJIDrone(nh);

    while(ros::ok())
    {
      /// Execute state machine.
        ros::spinOnce();
        performTask(currentState);
        currentState = nextState(currentState);
        rate.sleep();
    }

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
            if(init_flag)
                return TAKEOFF;
            else
                return INIT_MISSION;
        }
        case TAKEOFF:
        {
            return WAYPOINT_NAV;
            break;
        }
        case WAYPOINT_NAV:
        {
            return RETURN_HOME;
            break;
        }
        case RETURN_HOME:
        {
            return LAND;
            break;
        }
        case LAND:
        {
            return FINISHED;
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
            if(!init_flag)
            {
                char keyboard_input;
                cout << "\n Input i to start mission: ";
                cin >> keyboard_input;
                cout << "\n";
                if(keyboard_input == 'i')
                    init_flag = true;
            }

            home_wp.latitude = drone->global_position.latitude;
            home_wp.longitude = drone->global_position.longitude;
            home_wp.altitude = flying_height;
            home_wp.damping_distance = 0;
            home_wp.target_yaw = 0;
            home_wp.target_gimbal_pitch = 0;
            home_wp.turn_mode = 0;
            home_wp.has_action = 0;
            break;
        }
        case TAKEOFF:
        {
            drone->drone_arm();
            drone->takeoff();
            drone->local_position_navigation_send_request(0,0,flying_height);
            break;
        }
        case WAYPOINT_NAV:
        {
            drone->local_position_navigation_send_request(50,30,flying_height);
        }
        case RETURN_HOME:
        {
            waypoint_task.velocity_range = 10;
            waypoint_task.idle_velocity = 3;
            waypoint_task.action_on_finish = 0;
            waypoint_task.mission_exec_times = 1;
            waypoint_task.yaw_mode = 4;
            waypoint_task.trace_mode = 0;
            waypoint_task.action_on_rc_lost = 0;
            waypoint_task.gimbal_pitch_mode = 0;
            waypoint_task.mission_waypoint.push_back(home_wp);
            drone->mission_waypoint_upload(waypoint_task);
            drone->mission_start();
        }
        case LAND:
        {
            drone->landing();
            break;
        }
        case FINISHED:
        {
            drone->drone_disarm();
        }
        default:
      {
         /// Nothing to do

         break;
      }
   }
}


