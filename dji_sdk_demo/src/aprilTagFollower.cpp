#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <pid.h>
#define C_PI (double) 3.141592653589793

using namespace DJI::onboardSDK;
DJIDrone*   drone;
PID*        pidController;
float pitch;
float yaw;

void targetPosition_callback(const geometry_msgs::PointStamped& geom_msgs)
{
    float   x       = geom_msgs.point.x;
    float   y       = geom_msgs.point.y;
    float   z       = geom_msgs.point.z;
    float   r_proy  = sqrt(x*x +  y*y);
    pitch   = atan2(z,r_proy)*180/C_PI*10;
    yaw     = -atan2(y,x)*180/C_PI*10;
    int pitch_rate    = pidController->calculate(1.0, pitch, 0.0);
    int yaw_rate      = pidController->calculate(1.0, yaw, 0.0);
    if(drone->gimbal_speed_control(0, pitch_rate, yaw_rate))
    {
        printf("\nGimbal Position Refreshed.\n");
    }
    else
        printf("\n Gimbal Position Refreshing failed\n");
    printf("Pitch = %d\n Yaw = %d\n ------------------\n", pitch_rate, yaw_rate);


}

int main(int argc, char **argv)
{
    int direction;
    ros::init(argc, argv, "aprilTagFollower");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);
    pidController = new PID(100.0,-100.0,1.0,0,0);
    if(drone->request_sdk_permission_control())
        printf("\n Permission Control Acquired\n");
    ros::Subscriber targetPosition = nh.subscribe("droneuse/tag_position", 10, targetPosition_callback);
   
    ros::spin();
    return 0;

}
