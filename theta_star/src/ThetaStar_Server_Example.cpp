/*
 * Copyright 2015 Ricardo Ragel de la Torre, GRVC, Univ. of Seville, Spain
 *
 * Resume: Lazy Theta Star with Optimization Server
 * 
 * Input: /theta_star/goal_position (XYZ goal to plan from the current position)
 * 
 * Output: /trajectory_tracking/trajectory_input ([XYZT] vector to the trajectory tracker)
 * 
 * NOTE: For get rviz markers debug visualization go to 'theta_star/src/ThetaStar.cpp' and un-comment the line '#define DEBUG'
 */

#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <theta_star/ThetaStar.h>
#include <visualization_msgs/MarkerArray.h>
#include <ctime>
#include <sys/timeb.h>

#include <nav_msgs/Odometry.h>


using namespace std;
using namespace PathPlanners;

// Odometry message callbacks. Simply copy position data (geometry_msgs::TransformStamped)
//~ geometry_msgs::Transform odom_pose;
//~ void odometryCallback(const geometry_msgs::TransformStamped::ConstPtr& transform_msg)
//~ {
  //~ odom_pose = transform_msg->transform;
//~ }

// Odometry message callback. Simply remap the pose data in the odom_pose var.
geometry_msgs::Transform odom_pose;
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  odom_pose.translation.x = odom_msg->pose.pose.position.x;
  odom_pose.translation.y = odom_msg->pose.pose.position.y;
  odom_pose.translation.z = odom_msg->pose.pose.position.z;
  
  odom_pose.rotation = odom_msg->pose.pose.orientation;
}

// Odometry message callback. Simply remap the octomap msg
octomap_msgs::Octomap m;
bool octomap_received =false;
void CollisionMapCallBack(const octomap_msgs::Octomap::ConstPtr& fp)
{
    m = *fp;
    octomap_received = true;
}

// Goal message callback. Simply remap the goal position
geometry_msgs::Vector3 goal;
bool goal_received = false;
void GoalPositionCallBack(const geometry_msgs::Vector3 goal_)
{
	goal = goal_;
	goal_received = true;
	ROS_INFO("Theta Star Example: New goal received!!");
}

int main(int argc, char **argv)
{
    struct timeb startT, finishT;
    float seconds, milliseconds;

    ros::init(argc, argv, "Theta_Start_Server");
    ros::NodeHandle n;

	/// Read MAV name
	char mavName[100];
	if(argc < 2)
	{
	  ROS_ERROR("Theta Star Example: Need a MAV name as argument");
	  exit(0);
	}
	else
	{
	  std::strcpy(mavName, argv[1]);
	  ROS_INFO("Theta Star Example: MAV Name: %s", mavName);
	}

	// Odometry data topic subscriber
	char topicPath[100];
	//sprintf(topicPath, "/%s/odometry_sensor1/odometry", mavName);
	sprintf(topicPath, "/odometry");
    ros::Subscriber odom_sub = n.subscribe(topicPath, 1, odometryCallback);
    ROS_INFO("Theta Star Example: odometry topic: %s", topicPath);

	// Trajectory list topic publisher to trajectory_tracker_node
	//sprintf(topicPath, "/%s/trajectory_tracking/input_trajectory", mavName);
	sprintf(topicPath, "/trajectory_tracking/input_trajectory");
	ros::Publisher trajectory_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 10);
	ROS_INFO("Theta Star Example: trajectory output topic: %s", topicPath);

	// Path solution visualization topic
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker_path", 0 );

	// Trajectory solution visualization topic
    ros::Publisher vis_pub_traj = n.advertise<visualization_msgs::Marker>( "visualization_marker_trajectory", 0 );

	// Octomap topic subscriber
	//sprintf(topicPath, "/%s/octomap_binary", mavName);
	sprintf(topicPath, "/octomap_binary");
    //ros::Subscriber sub_map = n.subscribe(topicPath, 1, CollisionMapCallBack);
    ros::Subscriber sub_map = n.subscribe(topicPath, 1, CollisionMapCallBack);
    ROS_INFO("Theta Star Example: octomap input topic: %s", topicPath);

	// Goal position topic subscriber
	//sprintf(topicPath, "/%s/goal_position", mavName);
	sprintf(topicPath, "/goal_position");
    ros::Subscriber sub_goal = n.subscribe(topicPath, 0, GoalPositionCallBack);
	ROS_INFO("Theta Star Example: trajectory input topic: %s", topicPath);

    // Wait to get the Octomap
    ros::AsyncSpinner as(0);
    as.start();
    while(ros::ok() && !octomap_received)
    {
		ROS_INFO("Waiting for octomap...");
		ros::Duration(0.1).sleep();
	}

	// Read parameters
	double ws_x_max = 0.0;
	double ws_y_max = 0.0;
	double ws_z_max = 0.0;
	double ws_x_min = 0.0;
	double ws_y_min = 0.0;
	double ws_z_min = 0.0;
	double map_resolution = 0.0;
	double map_h_real_size_inflaction = 0.0;
	double map_v_real_size_inflaction = 0.0;
	double map_h_safe_distance_inflaction = 0.0;
	double map_v_safe_distance_inflaction = 0.0;
	double initial_point_factor = 1.0;
	double z_weight_cost = 1.0;
	double traj_dxy_max = 1.0;
	double traj_dz_max = 1.0;
	double traj_vxy_m = 1.0;
	double traj_vz_m = 1.0;
	double traj_vxy_m_1 = 1.0;
	double traj_vz_m_1 = 1.0;
	double traj_est_wy = 1.0;
    ros::param::get("/theta_star/ws_x_max", ws_x_max);	
    ros::param::get("/theta_star/ws_y_max", ws_y_max);	
    ros::param::get("/theta_star/ws_z_max", ws_z_max);	
    ros::param::get("/theta_star/ws_x_min", ws_x_min);	
    ros::param::get("/theta_star/ws_y_min", ws_y_min);	
    ros::param::get("/theta_star/ws_z_min", ws_z_min);
    ros::param::get("/theta_star/map_resolution", map_resolution);	
    ros::param::get("/theta_star/map_h_real_size_inflaction", map_h_real_size_inflaction);	
    ros::param::get("/theta_star/map_v_real_size_inflaction", map_v_real_size_inflaction);	
    ros::param::get("/theta_star/map_h_safe_distance_inflaction", map_h_safe_distance_inflaction);	
    ros::param::get("/theta_star/map_v_safe_distance_inflaction", map_v_safe_distance_inflaction);	
    ros::param::get("/theta_star/initial_point_factor", initial_point_factor);
    ros::param::get("/theta_star/z_weight_cost", z_weight_cost);
    ros::param::get("/theta_star/traj_dxy_max", traj_dxy_max);
    ros::param::get("/theta_star/traj_dz_max", 	traj_dz_max);
    ros::param::get("/theta_star/traj_vxy_m", 	traj_vxy_m);
    ros::param::get("/theta_star/traj_vz_m", 	traj_vz_m);
    ros::param::get("/theta_star/traj_vxy_m_1", 	traj_vxy_m_1);
    ros::param::get("/theta_star/traj_vz_m_1", 	traj_vz_m_1);
    ros::param::get("/theta_star/traj_est_wy", 	traj_est_wy);
    
    // Debug
    printf("Theta Star Configuration:\n");
    printf("\t WorkSpace: X:[%f, %f], Y:[%f, %f], Z:[%f, %f] \n", ws_x_min, ws_x_max, ws_y_min, ws_y_max, ws_z_min , ws_z_max);
    printf("\t Map: resol.= [%f], inflac.= [%f, %f]\n", map_resolution, map_h_real_size_inflaction + map_h_safe_distance_inflaction, map_v_real_size_inflaction + map_v_safe_distance_inflaction);
    printf("\t Lazy Theta* with optim.: initial_point_factor = [%f]\n", initial_point_factor);
    printf("\t Lazy Theta* weighted.: z_w = [%f]\n", z_weight_cost);
    printf("\t Trajectory result: Pos. inc. = [%f, %f], Cruising Speed = [%f, %f] (%f, %f)\n\n", traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1);
    
    // Init Theta*
    ThetaStar theta("theta_star_example", "/world", ws_x_max, ws_y_max, ws_z_max, ws_x_min, ws_y_min, ws_z_min, map_resolution, map_h_real_size_inflaction, map_v_real_size_inflaction, map_h_safe_distance_inflaction, map_v_safe_distance_inflaction, initial_point_factor, z_weight_cost, &n);
	
	// Set timeout
	theta.setTimeOut(20);
	
	// Path solution visualization marker
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time();
    path_marker.ns = "theta_star";
    path_marker.id = 1;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.1;
    path_marker.scale.y = 1;
    path_marker.scale.z = 1;
    path_marker.color.a = 1.0;
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;
    
    // Trajectory solution visualization marker
    visualization_msgs::Marker traj_marker;
	traj_marker.points.clear();
    traj_marker.header.frame_id = "world";
    traj_marker.header.stamp = ros::Time();
    traj_marker.ns = "theta_star";
    traj_marker.id = 2;
    traj_marker.type = visualization_msgs::Marker::CUBE_LIST;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = 0.1;
    traj_marker.scale.y = 0.1;
    traj_marker.scale.z = 0.1;
    traj_marker.color.a = 1.0;
    traj_marker.color.r = 1.0;
    traj_marker.color.g = 0.0;
    traj_marker.color.b = 0.5;    

	// Build theta* discrete map with the offset from /map to /world
	double map_x_offset = 0.0;
	double map_y_offset = 0.0;
	double map_z_offset = 0.0;
    ros::param::get("/map/x_offset", map_x_offset);
    ros::param::get("/map/y_offset", map_y_offset);
    ros::param::get("/map/z_offset", map_z_offset);
	
	ftime(&startT);
		theta.updateMap(m, map_x_offset, map_y_offset, map_z_offset);
	ftime(&finishT);
	
	seconds = finishT.time - startT.time - 1;
	milliseconds = (1000 - startT.millitm) + finishT.millitm;
	cout << "Time spend in construct the occupancy map: " << (milliseconds + seconds * 1000) << " ms" << endl;
    
	// Draw theta* occupancy map
	//~ sleep(3);
		//~ theta.publishOcupatioMap();
	//~ sleep(7);

	// Start and Goal positions
	Vector3 init,fin;

	// Result Trajectory
	trajectory_msgs::MultiDOFJointTrajectoryPtr trajectory(new trajectory_msgs::MultiDOFJointTrajectory);
	trajectory->joint_names.push_back("base_link");  
	trajectory->header.stamp = ros::Time::now();
	trajectory->header.frame_id = "base_link";
	
	ROS_INFO("Waiting for new goal...");
	while(ros::ok())
	{
		// Waiting for new goal and reading odometry
		ros::spinOnce();
		
		if(goal_received)
		{
			// Get initial (= current) and goal position 
			init = odom_pose.translation;
			fin = goal;
			
			// Set initial and final position to theta* and calculate the trajectory only if these are valid positions
			if(theta.setInitialPosition(init) && theta.setFinalPosition(fin))
			{
				// Path calculation
				ROS_INFO("Path calculation...");
				ftime(&startT);
					int number_of_points = theta.calculateNewPath();
				ftime(&finishT);

				seconds = finishT.time - startT.time - 1;
				milliseconds = (1000 - startT.millitm) + finishT.millitm;
				cout << "Time spend in path calculation: " << (milliseconds + seconds * 1000) << " ms" << endl;

				// Get and draw the new path
				path_marker.points.clear();
				path_marker.header.stamp = ros::Time();
				geometry_msgs::Point pIni;
				pIni.x = init.x;
				pIni.y = init.y;
				pIni.z = init.z;
				path_marker.points.push_back(pIni);
				for(unsigned int i=0; i < theta.getCurrentPath().size();i++)
				{
					geometry_msgs::Point p;
					p.x = theta.getCurrentPath()[i].x;
					p.y = theta.getCurrentPath()[i].y;
					p.z = theta.getCurrentPath()[i].z;
					path_marker.points.push_back(p);
				}
				vis_pub.publish(path_marker);

				// Get the trajectory
				trajectory->points.clear();
				trajectory->header.stamp = ros::Time::now();
				ROS_INFO("Trajectory calculation...");
				//~ theta.getCurrentTrajectory_YawAtTime(trajectory, odom_pose, traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1, traj_est_wy);
				theta.getCurrentTrajectory_YawInAdvance(trajectory, odom_pose, traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1, traj_est_wy);
			
				// Send the trajectory
				trajectory_pub.publish(trajectory);
				
				// Parse to markers and draw the new trajectory
				traj_marker.points.clear();
				traj_marker.header.stamp = ros::Time();
				
				for(unsigned int i=0; i < trajectory->points.size();i++)
				{
					geometry_msgs::Point p;
					p.x = trajectory->points[i].transforms[0].translation.x;
					p.y = trajectory->points[i].transforms[0].translation.y;
					p.z = trajectory->points[i].transforms[0].translation.z;
					traj_marker.points.push_back(p);
				}
				vis_pub_traj.publish(traj_marker);
			
			}
			
			goal_received = false;
			ROS_INFO("Waiting for new goal...");
		}
		
	    usleep(1e5);
	}
	
    return 0;
}
