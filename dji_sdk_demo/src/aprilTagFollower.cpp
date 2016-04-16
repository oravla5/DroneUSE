#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <pid.h>
#include <Eigen/Dense>
#define C_PI (double) 3.141592653589793

using namespace DJI::onboardSDK;
DJIDrone*   drone;
//PID*        pidController;
float       pitch;
float       yaw;

/*
void targetPosition_callback(const geometry_msgs::PointStamped& geom_msgs)
{

	
    int phi = drone->gimbal.pitch;
    int lam = drone->gimbal.yaw;

    Eigen::Matrix3d mat_rot;
    mat_rot << cos(lam)*cos(phi),  -sin(lam),  -cos(lam)*sin(phi),
               cos(phi)*sin(lam),  cos(lam),   -sin(lam)*sin(phi),
               sin(phi),           0,          cos(phi);
    
    float   x       = geom_msgs.point.x;
    float   y       = geom_msgs.point.y;
    float   z       = geom_msgs.point.z;

    Eigen::Vector3d pos_rel(x,y,z);
    Eigen::Vector3d pos_global;
    pos_global = mat_rot*pos_rel;

    x = pos_global(0);
    y = pos_global(1);
    z = pos_global(2);

    float   r_proy  = sqrt(x*x +  y*y);
    pitch   = atan2(z,r_proy)*180/C_PI*10;
    yaw     = atan2(y,x)*180/C_PI*10;
    printf("Pitch = %f\n Yaw = %f\n ------------------\n", pitch, yaw);
}*/

int main(int argc, char **argv)
{
    pitch   = 0.0;
    yaw     = 0.0;
    ros::init(argc, argv, "aprilTagFollower");
    ROS_INFO("droneuse_camera_control");
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);
    //pidController = new PID(200.0,-200.0,10.0,0.0,0.0);
    if(drone->request_sdk_permission_control())
        printf("\n Permission Control Acquired\n");
//    ros::Subscriber targetPosition = nh.subscribe("droneuse/tag_position", 10, targetPosition_callback);
    if(drone->gimbal_angle_control(200,0,0,20,1))
        printf("\nGimbal Position Refreshed.\n");
    sleep(2);
    while(nh.ok())
    {
	ros::spinOnce();


        /*
        float phi = drone->gimbal.pitch;
        float lam = drone->gimbal.yaw;
        int pitch_rate    = pidController->calculate(1.0, pitch, phi);
        int yaw_rate      = pidController->calculate(1.0, yaw, lam);
        if(drone->gimbal_speed_control(0, pitch_rate, -yaw_rate))
        {
            printf("\nGimbal Position Refreshed.\n");
		printf("Pitch = %f\n Yaw = %f\n", phi, lam);
	        printf("Pitch_rate = %d\n Yaw_rate = %d\n ------------------\n", pitch_rate, -yaw_rate);
        }
        else
            printf("\n Gimbal Position Refreshing failed\n");
//        printf("Pitch_rate = %d\n Yaw_rate = %d\n ------------------\n", pitch_rate, yaw_rate);
    }
	printf("Termina el bucle \n");    
*/
    }
    return 0;
}
