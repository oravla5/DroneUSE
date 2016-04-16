#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <iostream>
#include <pid.h>
#define C_PI (double) 3.141592653589793

using namespace DJI::onboardSDK;
float       target_pitch;
float       target_yaw;
/*
void targetPosition_callback(const geometry_msgs::PointStamped& geom_msgs)
{
    float   x       = geom_msgs.point.x;
    float   y       = geom_msgs.point.y;
    float   z       = geom_msgs.point.z;
    float   r_proy  = sqrt(x*x +  y*y);
            target_pitch  = atan2(z,r_proy)*180/C_PI;

       // printf("\n Gimbal Position Refreshing failed\n");
//    printf("Pitch = %d\n Yaw = %d\n ------------------\n", pitch, yaw);
   // printf("gimbal pitch = %f\n gimbal yaw = %f\n --------------\n", drone->gimbal.pitch, (drone->gimbal.yaw+180)%360);
}
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aprilTagFollower");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    DJIDrone* drone = new DJIDrone(nh);
    if(drone->request_sdk_permission_control())
        printf("\n Permission Control Acquired \n");
    //ros::Subscriber targetPosition = nh.subscribe("droneuse/tag_position", 10, targetPosition_callback);
    PID* pitchControl = new PID(1500.0,-1500.0,30.0,0.0,0.0);
    PID* yawControl = new PID(1000.0,-1000.0,30.0,0.0,0.0);
    int pitch_rate = 0;
    int yaw_rate = 0;
    target_pitch = 0.0;
    target_yaw   = 0.0;   
    int count = 0;
    float gimbal_pitch;
    float gimbal_yaw;
    std::cout << "\n";
    while(nh.ok())
    {   
        count++;
        ros::spinOnce();
        gimbal_pitch = drone->gimbal.pitch;
        gimbal_yaw = drone->gimbal.yaw;
        pitch_rate    = pitchControl->calculate(1.0, target_pitch, gimbal_pitch);
        yaw_rate    = yawControl->calculate(1.0, target_yaw, gimbal_yaw);
        drone->gimbal_speed_control(0, pitch_rate, yaw_rate);
        if((count%20)==0)
        {
            std::cout << "Target Pitch = " << target_pitch << "\n";
            std::cout << "Gimbal Pitch = " << gimbal_pitch << "\n";
            std::cout << "Pitch Rate = " << pitch_rate << "\n";
            std::cout << "-----------------------------------\n";
            std::cout << "Target Yaw = " << target_yaw << "\n";
            std::cout << "Gimbal Yaw = " << gimbal_yaw << "\n";
            std::cout << "Yaw Rate = " << yaw_rate << "\n";
            std::cout << "-----------------------------------\n";
        }
        rate.sleep();
    }
    //ros::spin();
    return 0;

}
