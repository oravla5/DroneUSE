#include <ros/ros.h>
#include "m100_control/m100Controller.h"
#include <iostream>
#include <cstdlib>
#include <stdio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "m100_controller_node");
    ros::NodeHandle nh;

    int control_rate = 20;
    m100Controller* m100_controller = new m100Controller(nh, control_rate);
    ros::Rate rate(control_rate);

    while(nh.ok())
    {
        m100_controller->velocity_based_position_controller();
        rate.sleep();
    }

    //delete m100_controller_node;
    //m100_controller_node = NULL;

    return 0;
}
