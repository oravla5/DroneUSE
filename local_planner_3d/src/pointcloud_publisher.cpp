#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <string>


using namespace sensor_msgs;

bool pointcloud_received = false;
sensor_msgs::PointCloud2 cloud_in;
sensor_msgs::PointCloud2 cloud_out;

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

	std::string target_frame("guidance_front");
	ros::init(argc, argv, "pointcloud_publisher");
	ros::NodeHandle nh;
	tf::TransformListener tf_listener;
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
		cloud_out.header.stamp = ros::Time::now();	
		cloud_out.header.frame_id = "guidance_front";
		cloud_in.header.frame_id = "world";
		tf_listener.waitForTransform(target_frame, cloud_in.header.frame_id, cloud_out.header.stamp, ros::Duration(0.3));
		pcl_ros::transformPointCloud(target_frame, cloud_in, cloud_out, tf_listener);
		pointcloud_pub.publish(cloud_out);
		ros::spinOnce();
		wait_time.sleep();
	}
	while( ros::ok() );

	return 0;
}

