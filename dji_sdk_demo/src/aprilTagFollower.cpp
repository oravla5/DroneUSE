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
float       tPitch;
//float       yaw;

void targetPosition_callback(const geometry_msgs::PointStamped& geom_msgs)
{
    float   x       = geom_msgs.point.x;
    float   y       = geom_msgs.point.y;
    float   z       = geom_msgs.point.z;
    float   r_proy  = sqrt(x*x +  y*y);
            tPitch  = atan2(z,r_proy)*180/C_PI;

       // printf("\n Gimbal Position Refreshing failed\n");
//    printf("Pitch = %d\n Yaw = %d\n ------------------\n", pitch, yaw);
   // printf("gimbal pitch = %f\n gimbal yaw = %f\n --------------\n", drone->gimbal.pitch, (drone->gimbal.yaw+180)%360);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aprilTagFollower");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    DJIDrone* drone = new DJIDrone(nh);
    if(drone->request_sdk_permission_control())
        printf("\n Permission Control Acquired \n");
    ros::Subscriber targetPosition = nh.subscribe("droneuse/tag_position", 10, targetPosition_callback);
    PID* pitchControl = new PID(200.0,-200.0,1.0,0.0,0.0);
    tPitch = 0.0;
    int count = 0;
    float gimPitch;
    std::cout << "\n";
    while(nh.ok())
    {   
        count++;
        ros::spinOnce();
        gimPitch = drone->gimbal.pitch;
        int pitch_rate    = pitchControl->calculate(1.0, tPitch, gimPitch);
        drone->gimbal_speed_control(0, pitch_rate*10, 0);
        if((count%20)==0)
        {
            std::cout << "Target Pitch = " << tPitch << "\n";
            std::cout << "Gimbal Pitch = " << gimPitch << "\n";
            std::cout << "Pitch Rate = " << pitch_rate << "\n";
            std::cout << "-----------------------------------";
        }
        rate.sleep();
    }
    //ros::spin();
    return 0;

}
