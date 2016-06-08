#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <vector>

typedef trajectory_msgs::MultiDOFJointTrajectory Trajectory;
typedef trajectory_msgs::MultiDOFJointTrajectoryPtr TrajectoryPtr;
typedef trajectory_msgs::MultiDOFJointTrajectoryPoint Waypoint;
typedef geometry_msgs::Transform Transform;
typedef geometry_msgs::Vector3 Vector3;
typedef geometry_msgs::Quaternion Quaternion;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "dummy_global_planner");
	ros::NodeHandle nh;
	ros::Publisher trajectory_pub = nh.advertise<Trajectory>("local_planner/input_trajectory", 1);
	
	TrajectoryPtr global_trajectory(new Trajectory);
		
	std::vector<Vector3> positions;

	Vector3 position;
	position.x = 3.0;
	position.y = 3.0;
	position.z = 2.0;
	
	positions.push_back(position);

	position.x = 3.0;
	position.y = -3.0;
	position.z = 2.0;

	positions.push_back(position);

	position.x = -3.0;
	position.y = 0.0;
	position.z = 1.0;

	positions.push_back(position);

	for(int i=0; i < 3; i++)
	{
		global_trajectory->joint_names.push_back("Waypoint");
		global_trajectory->header.stamp = ros::Time::now();
		global_trajectory->header.frame_id = "/world";


		Transform transform;
		transform.translation = positions[i];
	
		Waypoint wp;
		wp.transforms.push_back(transform);	
		
		global_trajectory->points.push_back(wp); 
	}

	ros::Time time_init =ros::Time::now();
	ros::Duration wait_time(10);

	while( (ros::Time::now() - time_init) < wait_time)
	{
		trajectory_pub.publish(global_trajectory);
		ros::spinOnce();
	}
	return 0;
}

