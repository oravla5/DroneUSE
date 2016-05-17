#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

std::string stabilized_frame;

///! Aux Function: Get yaw in radians from a quaternion
double get_yaw_from_quat(geometry_msgs::Quaternion quat)
{
	double r, p, y;
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 M(q);
	M.getRPY(r, p, y);
	
	return y;
}

//! Odometry message callback. Simply remap the pose data in the odom_pose var.
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{ 
	double x = odom_msg->pose.pose.position.x;
	double y = odom_msg->pose.pose.position.y;
	double z = odom_msg->pose.pose.position.z;
	double yaw = get_yaw_from_quat(odom_msg->pose.pose.orientation);

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(x, y, z) );
	tf::Quaternion q;
	q.setRPY(0, 0, yaw);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", stabilized_frame));
}

/**************** MAIN *********************/
int main(int argc, char** argv)
{		
	ros::init(argc, argv, "base_stabilized_publisher");
	ros::NodeHandle n;

	//! Args
	// Read odometry input topic name
	char odomTopicName[200];
	if(argc < 2)
	{
	  ROS_ERROR("Base Stabilized Pub: Needs a odometry input topic name as argument");
	  exit(0);
	}
	else
	{
	  std::strcpy(odomTopicName, argv[1]);
	  ROS_INFO("Base Stabilized Pub: Odometry Topic: %s", odomTopicName);
	}
	
	if(argc < 3)
	{
	  ROS_ERROR("Base Stabilized Pub: Needs a stabilized frame name as argument");
	  exit(0);
	}
	else
	{
	  stabilized_frame = argv[2];
	  ROS_INFO("Base Stabilized Pub: Frame: %s", stabilized_frame.c_str());
	}
	
	//! Topics
	char topicPath[100];
    // Input topic: Odometry data topic subscriber
    sprintf(topicPath, "%s", odomTopicName);
	ros::Subscriber odom_sub = n.subscribe(topicPath, 1, odometryCallback);

	//! Spinning
	ros::spin();
	return 0;
};
