#include <ros/ros.h>

#include "landingPlatformTracker.h"

using namespace ros;

int main(int argc, char **argv)
{

	init(argc, argv, "landing_platform_tracker");

	landingPlatformTracker tracker(argv[0]);
    int update_rate    = 20.0;
    double dt          = 1.0/update_rate;
    
    ros::Rate rate(update_rate);

    while(ros::ok())
    {
        tracker.landingPlatformPosUpdate(dt);
        spinOnce();
        rate.sleep();
    }
    return 0;
}
