#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#define C_PI (double) 3.141592653589793

using namespace DJI::onboardSDK;
DJIDrone* drone;

void targetPosition_callback(const geometry_msgs::PointStamped& geom_msgs)
{
    float   x       = geom_msgs.point.x;
    float   y       = geom_msgs.point.y;
    float   z       = geom_msgs.point.z;
    int     pitch   = atan2(z,x)*180/C_PI*10;
    int     yaw     = atan2(y,x)*180/C_PI*10;
    if(drone->gimbal_angle_control(0, pitch, yaw, 2, 1))
    {
        printf("Gimbal Position Refreshed.\n");
        printf("Pitch = %d\n Yaw = %d", pitch, yaw);
    }
}

int main(int argc, char **argv)
{
    int direction;
    ros::init(argc, argv, "aprilTagFollower");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);
    if(drone->request_sdk_permission_control())
        printf("\n Permission Control Acquired\n");
    ros::Subscriber targetPosition = nh.subscribe("droneuse/tag_position", 10, targetPosition_callback);
   
    ros::spin();
    return 0;

}
