#include <ros/ros.h>
#include "m100_control/gimbalController.h"
#include <iostream>
#include <cstdlib>
#include <stdio.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_controller_node");
    ros::NodeHandle nh;

    gimbalController* gimbal_controller = new gimbalController(nh);

    ros::Rate rate(20);

    while(nh.ok())
    {
        gimbal_controller->gimbal_rate_based_orientation_controller();
        rate.sleep();
    }

    delete gimbal_controller_node;
    gimbal_controller_node = NULL;

    return 0;
}
