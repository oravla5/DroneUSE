#include <ros/ros.h>

#include "landingTagMapTracker.h"

using namespace ros;

int main(int argc, char **argv)
{

	init(argc, argv, "landing_tagMap_tracker");

	landingTagMapTracker detector(argv[0],argv[1]);

	spin();
	return 0;
}

