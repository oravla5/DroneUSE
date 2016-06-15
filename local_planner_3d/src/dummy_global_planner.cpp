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

	Quaternion rotation;
	rotation.x = 0;
	rotation.y = 0;
	rotation.z = 0;
	rotation.w = 1;	

	Vector3 position;
	position.x = 1.0;
	position.y = 0.0;
	position.z = 3.0;
	
	positions.push_back(position);

	position.x = 1.0;
	position.y = 1.0;
	position.z = 3.0;

	positions.push_back(position);

	position.x = 1.0;
	position.y = 1.0;
	position.z = 1.0;

	positions.push_back(position);

	for(int i=0; i < 3; i++)
	{
		global_trajectory->joint_names.push_back("Waypoint");
		global_trajectory->header.stamp = ros::Time::now();
		global_trajectory->header.frame_id = "/world";


		Transform transform;
		transform.translation = positions[i];
		transform.rotation = rotation;
	
		Waypoint wp;
		wp.transforms.push_back(transform);	
		wp.time_from_start = ros::Duration(10*(i+1));
		
		global_trajectory->points.push_back(wp); 
	}

	ros::Time time_init =ros::Time::now();
	ros::Duration wait_time(10);

	while( ros::ok() )
	{
		getchar();
		trajectory_pub.publish(global_trajectory);
		ros::spinOnce();
	}
	return 0;
}

