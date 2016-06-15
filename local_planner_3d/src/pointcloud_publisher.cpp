#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


using namespace sensor_msgs;

bool pointcloud_received = false;
sensor_msgs::PointCloud2 cloud_in;

void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	if(pointcloud_received)
		return;

	cloud_in = *input;
	pointcloud_received = true;
	return;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "pointcloud_publisher");
	ros::NodeHandle nh;
	ros::Publisher pointcloud_pub = nh.advertise<PointCloud2>("/points", 1);
	ros::Subscriber pointcloud_sub = nh.subscribe("/octomap_point_cloud_centers",1, PointCloudCallback);
	
	ros::Duration wait_time(1/5);

	while( ros::ok() && !pointcloud_received)
	{
		ros::spinOnce();
	}
	do
	{
		cloud_in.header.stamp = ros::Time::now();	
		pointcloud_pub.publish(cloud_in);
		ros::spinOnce();
		wait_time.sleep();
	}
	while( ros::ok() );

	return 0;
}

