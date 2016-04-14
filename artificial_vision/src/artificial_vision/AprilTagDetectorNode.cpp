#include <ros/ros.h>

#include "AprilTagDetector.h"

using namespace ros;

int main(int argc, char **argv)
{

	init(argc, argv, "tag_detector");

	AprilTagDetector detector(argv[1], argv[2]);

	spin();
	return 0;
}
