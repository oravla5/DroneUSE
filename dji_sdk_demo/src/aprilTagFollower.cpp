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
#include <tf/transform_listener.h>

#define C_PI (double) 3.141592653589793

using namespace DJI::onboardSDK;
float       target_pitch;
float       target_yaw;
DJIDrone* drone;
tf::TransformListener* listener;

float getRoll(float q0, float q1, float q2, float q3)
{
  return atan2( 2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2) )*180/C_PI;
}

float getPitch(float q0, float q1, float q2, float q3)
{
  return asin( 2*(q0*q2 - q3*q1) )*180/C_PI;
}

float getYaw(float q0, float q1, float q2, float q3)
{
  return atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))*180/C_PI;
}

void targetPosition_callback(const geometry_msgs::PointStamped& geom_msgs)
{
    geometry_msgs::PointStamped geom_msgs_transformed;
    listener->transformPoint("/gimbal_NED", geom_msgs, geom_msgs_transformed);

    float   x       = geom_msgs_transformed.point.x;
    float   y       = geom_msgs_transformed.point.y;
    float   z       = geom_msgs_transformed.point.z;
    float   r_proy  = sqrt(x*x +  y*y);

    target_pitch   = atan2(-z,r_proy)*180/C_PI;
    target_yaw     = atan2(y,x)*180/C_PI;

}

/*
void compass_subscriber_callback(dji_sdk::Compass compass)
{}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aprilTagFollower");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    drone = new DJIDrone(nh);
    listener = new tf::TransformListener;
    if(drone->request_sdk_permission_control())
        printf("\n Permission Control Acquired \n");
    ros::Subscriber targetPosition = nh.subscribe("droneuse/tag_position", 10, targetPosition_callback);

    PID* pitchControl   = new PID(900.0,-900.0,40.0,0.0,0.0);
    PID* yawControl     = new PID(900.0,-900.0,40.0,0.0,0.0);
    
    int pitch_rate = 0;
    int yaw_rate = 0;

    float gimbal_pitch;
    float gimbal_yaw;

    target_yaw = 0;
    target_pitch = 0;
    
    while(nh.ok())
    {   
        //target_yaw = attitude_yaw;
        //target_pitch = attitude_pitch;

        gimbal_pitch    = drone->gimbal.pitch;
        gimbal_yaw      = drone->gimbal.yaw;

        pitch_rate      = pitchControl->calculate(1.0, target_pitch, gimbal_pitch);
        yaw_rate        = yawControl->calculate(1.0, target_yaw, gimbal_yaw);

        drone->gimbal_speed_control(0, pitch_rate, yaw_rate);

        std::cout << "Target Yaw = "    << target_yaw   << "\n";
        std::cout << "Gimbal Yaw = "    << gimbal_yaw   << "\n";
        std::cout << "Yaw Rate = " << yaw_rate << "\n";
        std::cout << "----\n";
        std::cout << "Target Pitch = " << target_pitch << "\n";
        std::cout << "Gimbal Pitch = " << gimbal_pitch << "\n";
        std::cout << "Pitch Rate = " << pitch_rate << "\n";
        std::cout << "--------------------------\n";

        //std::cout << "Quaternions: \n";
        //std::cout << "q0 = " << q0 << "\n";
        //std::cout << "q1 = " << q1 << "\n";
        //std::cout << "q2 = " << q2 << "\n";
        //std::cout << "q3 = " << q3 << "\n";
        //std::cout << "Euler Angles:" << q3 << "\n";
        //std::cout << "Drone Yaw = "   << getYaw(q0,q1,q2,q3)   << "\n";
        //std::cout << "Drone Pitch = " << getPitch(q0,q1,q2,q3) << "\n";
        //std::cout << "Roll = "  << getRoll(q0,q1,q2,q3)  << "\n";

        ros::spinOnce();
        rate.sleep();
    }
    //ros::spin();
    return 0;

}
