#include <ros/ros.h>
#include "m100_control/gimbalController.h"
#include <iostream>
#include <cstdlib>
#include <stdio.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_controller_node");
    ros::NodeHandle nh;
    int control_rate = 20;
    gimbalController* gimbal_controller = new gimbalController(nh, control_rate);

    ros::Rate rate(control_rate);

    while(nh.ok())
    {
        gimbal_controller->gimbal_controller_update();
        rate.sleep();
    }

    //delete gimbal_controller_node;
    //gimbal_controller_node = NULL;

    return 0;
}
