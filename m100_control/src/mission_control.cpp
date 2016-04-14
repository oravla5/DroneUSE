//------------------------------------------------------------------------
// mission_control.cpp
// Created on April the 2nd, 2016 by Alvaro Fernandez
// This node performs the management of the entire Search & Rescue Mission
//------------------------------------------------------------------------
#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>

// States machine definition
typedef enum
{
    WAITING,                    // Pre-Mission and Post-Mission state
    TAKE_OFF,                   // The mission starts and the UAV leaves the ground
    NAVIGATE_TO_SAR_ZONE,       // The UAV moves to the Search & Rescue Zone
    SAR_ZONE_EXPLORATION,       // The UAV explores of the Search & Rescue zone
    NAVIGATE_TO_LANDING_ZONE,   // The UAV returns to the landing zone
    LAND                        // The UAV lands and swithes off motors
} MissionState;


typedef enum
{
    GROUND
    WAYPOINTS,
    FOLLOW_ME,
    FIX_POSITION,
    MANUAL_CONTROL
} FlightMode;


typedef enum
{
    OFF,
    EXPLORATION,
    POINT_OF_INTEREST
} CameraMode;


typedef enum
{
    OFF,
    ON
} GuidanceMode;


MissionState                nextState(const MissionState &current);
void                        performTask(const MissionState &current);

MissionState                currentMissionMode = WAITING;
FlightMode                  currentFlightMode = GROUND;
CameraMode                  currentCameraMode = OFF;
GuidanceMode                currentGuidanceMode = OFF;

DJI::onboardSDK::DJIDrone   *drone;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MissionControlNode");

    ros::NodeHandle nh;
    DJI::onboardSDK::DJIDrone *drone = new DJIDrone(nh);
    

    while(ros::ok())
    {
        performTask(currentState);
        currentState = nextState(currentState);
    }

    ros::shutdown();

    return EXIT_SUCCESS;
}

MissionStates nextState(const MissionStates &current)
{
    switch(current)
    {
        case WAITING:
        {
            return TAKE_OFF; 
        }
        case TAKE_OFF: 
        {
            return NAVIGATE_TO_SAR_ZONE; 
        }    
        case NAVIGATE_TO_SAR_ZONE: 
        {
            return SAR_ZONE_EXPLORATION; 
        }
        case SAR_ZONE_EXPLORATION:
        {
            return NAVIGATE_TO_LANDING_ZONE; 
        }
        case NAVIGATE_TO_LANDING_ZONE:
        {    
            return LAND;
        }
        case LAND:
        {
            return WAITING;
        }
    }
}

void performTask(const MissionStates &current)
{
    switch(current)
    {
        case WAITING:
        {
        }
        case TAKE_OFF: 
        {
            drone->takeoff();
            currentFlightMode = FIX_POSITION;
        }    
        case NAVIGATE_TO_SAR_ZONE: 
        {
        }
        case SAR_ZONE_EXPLORATION:
        {
        }
        case NAVIGATE_TO_LANDING_ZONE:
        {    
        }
        case LAND:
        {
        }
    }
}
