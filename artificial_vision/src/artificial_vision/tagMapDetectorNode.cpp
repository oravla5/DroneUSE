#include <ros/ros.h>

#include "tagMapDetector.h"

using namespace ros;

int main(int argc, char **argv)
{

	init(argc, argv, "tagMap_detector");

	tagMapDetector  tagMap_detector(argv[0],argv[1]);

	spin();
	return 0;
}

