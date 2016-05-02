#include "ros/ros.h"
#include <iostream>

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

using namespace ros;
using namespace std;

bool finish = false;

void callback(const octomap_msgs::Octomap& octomap_msg)
{
	octomap::OcTree* tree = octomap_msgs::binaryMsgToMap(octomap_msg);
	
	cout << "Ha llegado al callback" << endl;
	tree->writeBinary("/home/ubuntu/guidance_octomap.bt");
	finish = true;
	return;
}
int main(int argc, char** argv)
{
	init(argc, argv, "octo_test");

	NodeHandle nh;

	Subscriber sub = nh.subscribe("/octomap_binary", 5, callback);

	while(ros::ok())
	{
		spinOnce();
		if(finish)
			break;
	}
	return 0;
}
