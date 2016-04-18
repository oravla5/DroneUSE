#include <ros/ros.h>
#include "droneuseControl.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ROS_INFO("mission_control_node");
    ros::NodeHandle nh;
    uav_controller = new droneuseControl(nh);
    ros::spin();
    return 0;
}

