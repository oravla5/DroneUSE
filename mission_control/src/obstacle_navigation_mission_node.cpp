// LANDING NODE
#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk.h>
#include <std_msgs/UInt8.h>
#include <mission_control/droneuse_m100.h>
#include <mission_control/droneuse_gimbal.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace std;
using namespace DJI::onboardSDK;


typedef trajectory_msgs::MultiDOFJointTrajectory Trajectory;
typedef trajectory_msgs::MultiDOFJointTrajectoryPtr TrajectoryPtr;
typedef trajectory_msgs::MultiDOFJointTrajectoryPoint Waypoint;
typedef geometry_msgs::Transform Transform;
typedef geometry_msgs::Vector3 Vector3;
typedef geometry_msgs::Quaternion Quaternion;

droneuse_gimbal*    gimbal;
droneuse_m100*      m100;

ros::ServiceClient m100_attitude_control_service; 
dji_sdk::AttitudeControl m100_control_command;

const int control_rate = 20;

ros::Publisher trajectory_pub; 

Trajectory local_path;
bool trajectory_received;
void local_planning_callback(const Trajectory::ConstPtr& local_path_msg)
{
	local_path = *local_path_msg;
	trajectory_received = true;
	return;
}

void publish_global_path()
{
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
        position.z = 1.0;

        positions.push_back(position);

        position.x = 2.0;
        position.y = 0.0;
        position.z = 1.0;

        positions.push_back(position);

        position.x = 3.0;
        position.y = 0.0;
        position.z = 1.0;

        positions.push_back(position);

        position.x = 4.0;
        position.y = 0.0;
        position.z = 1.0;

        positions.push_back(position);

        global_trajectory->header.stamp = ros::Time::now();
        global_trajectory->header.frame_id = "/world";
        for(int i=0; i < 4; i++)
        {
                global_trajectory->joint_names.push_back("Waypoint");

                Transform transform;
                transform.translation = positions[i];
                transform.rotation = rotation;

                Waypoint wp;
                wp.transforms.push_back(transform);
                wp.time_from_start = ros::Duration(10*(i+1));

                global_trajectory->points.push_back(wp);
        }

                trajectory_pub.publish(global_trajectory);
	return;

}

int get_closest_waypoint()
{
	int closest_index = -1;
	double closest_distance = 0.0;
	double distance_aux = 0.0;	
	geometry_msgs::PointStamped next_wp;
	next_wp.header.frame_id = local_path.header.frame_id;
	next_wp.header.stamp = ros::Time::now();

	if(local_path.points.size() == 0)
		return closest_index;

	next_wp.point.x = local_path.points[0].transforms[0].translation.x;	
	next_wp.point.y = local_path.points[0].transforms[0].translation.y;	
	next_wp.point.z = local_path.points[0].transforms[0].translation.z;	
	closest_distance = m100->distance_to_position(next_wp);	
	closest_index = 0;

	for(int i=1; i<local_path.points.size(); i++)
	{
		next_wp.point.x = local_path.points[i].transforms[0].translation.x;	
		next_wp.point.y = local_path.points[i].transforms[0].translation.y;	
		next_wp.point.z = local_path.points[i].transforms[0].translation.z;	
		distance_aux = m100->distance_to_position(next_wp);	
		if(distance_aux <= closest_distance)
		{
			closest_distance = distance_aux;
			closest_index = i;
		}
		else
			break;
	}
	if( closest_distance <= 0.3 && (closest_index +1) < local_path.points.size() )
		closest_index += 1;

	return closest_index;
}
geometry_msgs::PointStamped get_wp(int index)
{
	geometry_msgs::PointStamped next_wp;
	next_wp.header.frame_id = local_path.header.frame_id;
	next_wp.header.stamp = ros::Time::now();
	next_wp.point.x = local_path.points[index].transforms[0].translation.x;	
	next_wp.point.y = local_path.points[index].transforms[0].translation.y;	
	next_wp.point.z = local_path.points[index].transforms[0].translation.z;	
	return next_wp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    ros::NodeHandle nh;
    ros::Rate rate(control_rate);
    gimbal  = new droneuse_gimbal(nh);
    m100    = new droneuse_m100(nh);

   ros::Subscriber local_path_sub = nh.subscribe("trajectory_tracking/input_trajectory",1,local_planning_callback);

   trajectory_pub = nh.advertise<Trajectory>("local_planner/input_trajectory", 1);

     m100_attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");

    
    // Wait for take off
	while(m100->get_flight_status() != 3)
    	{
        	rate.sleep();
		ros::spinOnce();
    	}

    while(m100->get_sdk_control() != true)
    {
        rate.sleep();
	ros::spinOnce();
    }

	//hover for 3 seconds
	cout << "Empieza hover" << endl;
	m100->hover();
       	ros::Duration(1.5).sleep();
	
	trajectory_received = false;
	while(!trajectory_received)
	{
		//Publish global path
		publish_global_path();
		cout << "WAITING FOR LOCAL PATH" << endl;
		//Get trajectory
		ros::spinOnce();
		rate.sleep();
	}

	//Go to the next waypoint
	int next_wp = -1;
	while(ros::ok())
	{
		next_wp = get_closest_waypoint();
		if(next_wp == -1)
			break;
		m100->set_target_position(get_wp(next_wp));
		rate.sleep();
		ros::spinOnce();
	}

	cout << "LANDING" << endl;
    m100->land();
    m100->disarm();

   return 0;
}    
