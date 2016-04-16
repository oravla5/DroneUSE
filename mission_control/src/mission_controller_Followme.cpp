#include <fstream>

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <dji_sdk/dji_drone.h>


using namespace std;

/// State machine of the followme mission
typedef enum
{
   UAV_TAKEOFF,
   UAV_WAYPOINT,
   UAV_RETURN_HOME,
   UAV_LAND,
   FINISHED
} States;

void uavControlReferencesLoop(const ros::TimerEvent &te);
bool getWayPointsFromFile(const string &filename, GoToWayPointsGoal &wp_goal_ugv);
States nextState(const States &current);
void perfromTask(const States &current);

UGVControl                 *ugvController = NULL;
UAVControl                 *uavController = NULL;
bool                       followUGV = false;
ros::Publisher             uavControlReferencesPub;
ControlReferenceRwStamped  safetyPos;
ControlReferenceRwStamped  followingPos;
GoToWayPointsGoal          wpGoal;
States                     currentState = UAV_TAKEOFF;

int main(int argc, char** argv)
{
   if(argc < 3)
   {
      ROS_ERROR("[MissionControllerNode] This program has two input parameter.\n"
                "The first input parameter is the number of the UAV and UGV.\n"
                "The second input parameter is the path of the waypoints file.");
      return EXIT_FAILURE;
   }

   const string vehicleId = argv[1];

   if(!getWayPointsFromFile(string(argv[2]), wpGoal))
   {
      ROS_ERROR_STREAM("[MissionControllerNode] Cannot get waypoints from file: " << argv[2]);
      return EXIT_FAILURE;
   }

   ros::init(argc, argv, "MissionControllerNode");

   ros::NodeHandle nh;

   /// [Ejercicio] Crear un publicar para el tópico "/ual_1/control_references_rw"
   /// (Tipo msg: ControlReferenceRwStamped)
   uavControlReferencesPub = nh.advertise<ControlReferenceRwStamped>("/ual_1/control_references_rw",1);

   ros::Timer timerCR = nh.createTimer(ros::Duration(1.0 / 50.0), &uavControlReferencesLoop);

   ros::AsyncSpinner spinner(2);
   spinner.start();

   ugvController = new UGVControl(nh, vehicleId);
   uavController = new UAVControl(nh, vehicleId);

   uavController->establishInitialPosition();
   ugvController->establishInitialPosition();

   while(ros::ok())
   {
      /// Execute state machine.
      perfromTask(currentState);
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
      case UAV_TAKEOFF:
      {
         if(safetyPos.c_reference_rw.position.x != 0 ||
            safetyPos.c_reference_rw.position.y != 0)
         {
            return UGV_SEND_WAYPOINTS;
         }
         else
         {
            return UAV_TAKEOFF;
         }
         break;
      }
      case UGV_SEND_WAYPOINTS:
      {
         return WAITFOR_UAV_SAFETY_POSITION;
         break;
      }
      case WAITFOR_UAV_SAFETY_POSITION:
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
            return UAV_LAND;
         }
         else
         {
            return WAITFOR_UAV_SAFETY_POSITION;
         }

         break;
      }
      case UAV_LAND:
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
         followUGV = false;

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
      case UGV_SEND_WAYPOINTS:
      {
         followUGV = true;
         ugvController->goToWps(wpGoal);
         followUGV = false;

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


bool getWayPointsFromFile(const string &filename, GoToWayPointsGoal &wpGoalUgv)
{
   bool result = false;
   ifstream file(filename.c_str());

   if(file.is_open())
   {
      wpGoalUgv.way_points.clear();
      wpGoalUgv.size = 0;

      WayPointWithCruiseStamped wp;

      while(file.good())
      {
         file >> wp.way_point.x;
         file >> wp.way_point.y;
         file >> wp.way_point.z;

         if(!file.good())
         {
            break;
         }

         file >> wp.way_point.cruise;

         wp.header.frame_id = "mission_ctrl_FOLLOWME";

         wpGoalUgv.way_points.push_back(wp);

         ROS_INFO("[MissionControllerNode] WayPoint added to UGV: [%f;%f;%f]",
                  wp.way_point.x, wp.way_point.y, wp.way_point.z);
      }

      wpGoalUgv.size = wpGoalUgv.way_points.size();

      result = true;
   }

   return result;
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
