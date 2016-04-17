#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <dji_sdk/dji_drone.h>


using namespace std;
using namespace DJI::onboardSDK;

/// State machine of the followme mission
typedef enum
{
    INIT_MISSION,
    GROUND,
    TAKEOFF,
    WAYPOINT,
    RETURN_HOME,
    LAND,
    FINISHED
} States;

void uavControlReferencesLoop(const ros::TimerEvent &te);
bool getWayPointsFromFile(const string &filename, GoToWayPointsGoal &wp_goal_ugv);
States nextState(const States &current);
void perfromTask(const States &current);
DJIDrone* drone;
ros::Publisher             uavControlReferencesPub;
GoToWayPointsGoal          wpGoal;
States                     currentState = INIT_MISSION;
int keyboard_input;

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    
    drone = new DJIDrone(nh);

    while(ros::ok())
    {
      /// Execute state machine.
        if(currentState == INIT_MISSION)
        {
            cout << "\n Input i to start mission: "
            cin >> keyboard_input;
            cout << "\n";
        }
        performTask(currentState);
        currentState = nextState(currentState);
        usleep(50000);
    }

   ros::shutdown();

   return EXIT_SUCCESS;
}


States nextState(const States &current)
{
    switch(current)
    {
        case INIT_MISSION:
        {
            if(keyboard_input == 'i')
                return TAKEOFF;
            else
                return INIT_MISSION;
        }
        case TAKEOFF:
        {
            if(drone->global_position > flying_height)
            {
                return WAYPOINT_NAV;
            }
            else
            {
                return TAKEOFF;
            }
            break;
        }
        case WAYPOINT_NAV
        {
            return RETURN_HOME;
        }
        case RETURN_HOME
        {
            return LAND;
        }
        case LAND:
        {
            tf::Vector3 uavPosition, landPosition;

            uavPosition.setValue(uavController->getLastUAVState().ual_state.dynamic_state.position.x,
                                  uavController->getLastUAVState().ual_state.dynamic_state.position.y,
                                  uavController->getLastUAVState().ual_state.dynamic_state.position.z);

            landPosition.setValue(safetyPos.c_reference_rw.position.x,
                                   safetyPos.c_reference_rw.position.y,
                                   safetyPos.c_reference_rw.position.z);

            if((landPosition - uavPosition).length() < 0.1)
            {
                return LAND;
            }
            break;
        }
        case LAND:
        {
            ROS_INFO("[MissionControllerNode] Mission Finished");

            return FINISHED;
        }
        case FINISHED:
        default:
        {
            return FINISHED;
            break;
        }
    }
}

void perfromTask(const States &current)
{
   switch(current)
   {
      case UAV_TAKEOFF:
      {

         safetyPos.c_reference_rw.cruise      = 0.25;
         safetyPos.c_reference_rw.position.x  = uavController->getLastUAVState().
                                                     ual_state.dynamic_state.position.x;
         safetyPos.c_reference_rw.position.y  = uavController->getLastUAVState().
                                                     ual_state.dynamic_state.position.y;
         safetyPos.c_reference_rw.position.z  = FLYING_HEIGHT;

         ROS_INFO("[MissionControllerNode] UAV Safety Position: [%f;%f;%f]",
                  safetyPos.c_reference_rw.position.x,
                  safetyPos.c_reference_rw.position.y,
                  safetyPos.c_reference_rw.position.z);

         if(safetyPos.c_reference_rw.position.x != 0 ||
            safetyPos.c_reference_rw.position.y != 0)
         {
            uavController->takeOff();
         }

         break;
      }
      case UAV_LAND:
      {
         uavController->land();

         break;
      }
      case WAITFOR_UAV_SAFETY_POSITION:
      case FINISHED:
      default:
      {
         /// Nothing to do

         break;
      }
   }
}


void uavControlReferencesLoop(const ros::TimerEvent &te)
{
   if(followUGV)
   {
      /// Send following position
      followingPos.header.seq++;
      followingPos.header.stamp              = ros::Time::now();

      /// [Ejercicio] Ajustar followingPos para seguir al UGV:
      followingPos.c_reference_rw.position.x = ugvController->getLastUGVState().ual_state.dynamic_state.position.x;
      followingPos.c_reference_rw.position.y = ugvController->getLastUGVState().ual_state.dynamic_state.position.y;
      followingPos.c_reference_rw.position.z = 1.5;
      followingPos.c_reference_rw.cruise = 0.3;
      /// Altura fija: 1.5 metros
      /// Velocidad fija: 0.3 metros / segundo

      /// [Ejercicio] Publicar tópico: followingPos
      uavControlReferencesPub.publish(followingPos);
      ///
   }
   else
   {
      /// Send safety position
      safetyPos.header.seq++;
      safetyPos.header.stamp = ros::Time::now();

      /// [Ejercicio] Publicar tópico: safetyPos
      uavControlReferencesPub.publish(safetyPos);
   }
}
