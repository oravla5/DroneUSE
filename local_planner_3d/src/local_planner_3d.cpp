/*
 * Copyright 2015 Ricardo Ragel de la Torre, GRVC, Univ. of Seville, Spain
 *
 * Resume: 	A Local Planner, implemented using the theta_star package 
 * 			(Lazy Theta Star with Optimization), to do a intermediate 
 * 			re-plannification between the global planner and trajectory
 * 			tracker.
 * 
 * Input: 	The entire trajectory (initial->target) computed by the global 
 * 			planner from /local_planner/input_trajectory topic
 * 
 * Output: 	The modified trajectory (current->target) sended throught
 * 			/trajectory_tracking/trajectory_input topic to the traject.
 * 			tracker.
 */

#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <math.h>
#include <sys/timeb.h>
#include <local_planner_3d/local_planner_3d.h>
#include <theta_star/ThetaStar.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <sensor_msgs/PointCloud2.h>

//! Un-comment to get debug info
#define PRINT_REPLANNING_TIME
#define PRINT_GLOBAL_TRAJECTORY
#define PRINT_GLOBAL_SUB_TRAJECTORY
#define PRINT_LOCAL_SUB_TRAJECTORY
#define PRINT_LOCAL_SUB_TRAJECTORY_PATCH
#define PRINT_REPLANNED_LOCAL_SUB_TRAJECTORY
#define PRINT_REPLANNED_LOCAL_SUB_TRAJECTORY_POST_PROCESSED
#define PRINT_REPLANNED_GLOBAL_SUB_TRAJECTORY

/// Changed to use rqt_reconfigure
//~ //! Un-comment to publish the theta* (local) occupation map matrix as RViz markers
#define PUBLISH_OCCUPATION_MAP
//~ 
//~ //! Un-comment to publish the theta* auxiliar (global) occupation map matrix as RViz markers
#define PUBLISH_AUX_OCCUPATION_MAP
//~ 
//~ //! Un-comment to publish the point cloud transformed to the world frame and filtered??????
//~ #define PUBLISH_POINTCLOUD_AT_WORLD

/// Input pointCloud rate. Used to ser the time of life for the occupancy matrix nodes and to the local planner while sync.
#define POINTCLOUD_RATE 5.0
#define MIN_WP_DIST 0.4
/// Local planner states for global planner feedback
//#define STARTING	euroc_motion_planning_msgs::Status::STARTING
//#define WAITING 	euroc_motion_planning_msgs::Status::WAITING
//#define RUNNING 	euroc_motion_planning_msgs::Status::RUNNING
//#define ERROR	 	euroc_motion_planning_msgs::Status::ERROR

/// Maximum replanning errors to set status as ERROR. Note: 2 seg * 5 Hz --> 10 replanning fails
#define MAX_REPLANNING_ERRORS 2*POINTCLOUD_RATE

using namespace std;
using namespace PathPlanners;
using namespace pcl_conversions;

double get_dist(const geometry_msgs::Vector3& r1, const geometry_msgs::Vector3& r2)
{
	geometry_msgs::Vector3 vector;
	vector.x = r1.x - r2.x;
	vector.y = r1.y - r2.y;
	vector.z = r1.z - r2.z;
	return (double) sqrt(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);

}
//! Odometry message callback. Simply remap the pose data in the odom_pose var.
Transform odom_pose;
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{ 
  odom_pose.translation.x = odom_msg->pose.pose.position.x;
  odom_pose.translation.y = odom_msg->pose.pose.position.y;
  odom_pose.translation.z = odom_msg->pose.pose.position.z;
  
  odom_pose.rotation = odom_msg->pose.pose.orientation;
}

//! PointCloud message callback. Get, filter and crop the input PointCloud at /sensor
PointCloud cloud_in_sensor;	// Save as PCL data (pcl::PointCloud<pcl::PointXYZ>::points[].x,y,z)
bool pointcloud_received =false;
ros::Time cloud_time_stamp;
//void PointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& input)
void PointCloudCallBack(const PointCloud::ConstPtr& input)
{
	// Save the timeStamp to get the TF transforms at the same time instant
	pcl_conversions::fromPCL(input->header.stamp, cloud_time_stamp);
	
    // - Not filter the pointcloud  (CPU LOAD: 60% running only the callback)
    // - Copy directly 			  	(CPU LOAD: 20%, Filter apply to the cloud transformed to the world)
	// Convert to PCL data type
	//pcl::fromROSMsg (*input, cloud_in_sensor);
    
	cloud_in_sensor = *input;
	// Set flag true to sincronizate with TF
    pointcloud_received = true;
}


//! Global trajectory message callback. Simply remap the received global trajectory
Trajectory global_trajectory;
bool global_traj_received = false;
void globalTrajectoryCallBack(const Trajectory::ConstPtr& global_trajectory_msg)
{
	global_trajectory = *global_trajectory_msg;
	global_traj_received = true;
	//ROS_INFO("Local Planner: New global trajectory received!!");
	cout << "Local Planner: New global trajectory received!!" << endl;
}			



/**************** MAIN *********************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Local_Planner");
    ros::NodeHandle n;

	//! Args	
	// Read odometry input topic name
    char odomTopicName[200];
    if(argc < 2)
    {
	  ROS_ERROR("Local Planner: Needs a odometry input topic name as argument");
	  exit(0);
    }
    else
    {
      std::strcpy(odomTopicName, argv[1]);
      ROS_INFO("Local Planner: Odometry Topic: %s", odomTopicName);
    }

	// Read point cloud input topic name
    char pointCloudTopicName[200];
    if(argc < 3)
    {
	  ROS_ERROR("Local Planner: Needs a point cloud input topic name as argument");
	  exit(0);
    }
    else
    {
      std::strcpy(pointCloudTopicName, argv[2]);
	  ROS_INFO("Local Planner: pointCloud Topic: %s", pointCloudTopicName);
    }
	
	//! Topics
	char topicPath[100];
    // Input topic: Odometry data topic subscriber
    sprintf(topicPath, "%s", odomTopicName);
	ros::Subscriber odom_sub = n.subscribe(topicPath, 1, odometryCallback);
	// Input topic: pointCloud topic subscriber
	//~ sprintf(topicPath, "/%s/vi_sensor/camera_depth/depth/points", mavName);
	sprintf(topicPath, "%s", pointCloudTopicName);
    ros::Subscriber pointCloud_sub = n.subscribe(topicPath, 1, PointCloudCallBack);
	// Input topic: Global trajectory topic subscriber
    sprintf(topicPath, "local_planner/input_trajectory");
    ros::Subscriber global_traj_sub = n.subscribe(topicPath, 1, globalTrajectoryCallBack);
	// Output topic: Trajectory list topic publisher to trajectory_tracker_node
    sprintf(topicPath, "trajectory_tracking/input_trajectory");
	ros::Publisher trajectory_pub = n.advertise<Trajectory>(topicPath, 1);
	// Output topic: local_planner status feedback to global planner
	// sprintf(topicPath, "local_planner/status");
	// ros::Publisher status_pub = n.advertise<euroc_motion_planning_msgs::Status>(topicPath, 10);
	// Debug output topic: Path solution visualization topic
    sprintf(topicPath, "local_planner/vis_marker_path");
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( topicPath, 1);
	// Debug output topic: Trajectory solution visualization topic
    sprintf(topicPath, "local_planner/vis_marker_trajectory");
	ros::Publisher vis_pub_traj = n.advertise<visualization_msgs::Marker>(topicPath, 1);
	// Debug output topic: Point Cloud buffered at /world
    sprintf(topicPath, "local_planner/cloud_in_world");
	ros::Publisher cloud_in_world_pub = n.advertise<PointCloud>(topicPath, 1);

	//! Feedback topic initialization
	/*ROS_INFO("Waiting for a subscriber for status topic..");
	while(ros::ok())
	{
		if(status_pub.getNumSubscribers())
			break;
		else
			ros::Duration(0.01).sleep();
	}
	euroc_motion_planning_msgs::Status status;			// Status msg
	status.header.stamp = ros::Time::now();
	status.header.frame_id = '0';
	status.state = STARTING;
	status_pub.publish(status);
	*/int replanning_errors_number = 0;			// Errors replanning in each global trajectory, if > MAX_REPLANNING_ERRORS --> Set status to ERROR
	bool local_planner_fail = false;

	//! Read parameters
	string local_planner_frame = "/ground_frame";
	string global_planner_frame = "/world";
	string visual_pointcloud_frame = "/guidance_front";
	double theta_timeout = 0.5;
	double local_ws_x_max = 0.0;
	double local_ws_y_max = 0.0;
	double local_ws_z_max = 0.0;
	double local_ws_x_min = 0.0;
	double local_ws_y_min = 0.0;
	double local_ws_z_min = 0.0;
	double global_ws_x_max = 0.0;
	double global_ws_y_max = 0.0;
	double global_ws_z_max = 0.0;
	double global_ws_x_min = 0.0;
	double global_ws_y_min = 0.0;
	double global_ws_z_min = 0.0;
	double map_resolution = 0.0;
	double map_h_real_size_inflaction = 0.0;
	double map_v_real_size_inflaction = 0.0;
	double map_h_safe_distance_inflaction = 0.0;
	double map_v_safe_distance_inflaction = 0.0;
	double initial_point_factor = 1.0;
	double z_weight_cost = 1.0;
	double lofs_margin = 0.0;
	double traj_dxy_max = 1.0;
	double traj_dz_max = 1.0;
	double traj_vxy_m = 1.0;
	double traj_vz_m = 1.0;
	double traj_vxy_m_1 = 1.0;
	double traj_vz_m_1 = 1.0;
	double traj_est_wy = 1.0;
	int traj_yaw_mode = 2;
    double cloud_points_timeout;
    ros::param::get("local_planner/local_planner_frame", local_planner_frame);
    ros::param::get("local_planner/global_planner_frame", global_planner_frame);
    ros::param::get("local_planner/visual_pointcloud_frame", visual_pointcloud_frame);
    ros::param::get("local_planner/theta_timeout", theta_timeout);
    ros::param::get("local_planner/local_ws_x_max", local_ws_x_max);
    ros::param::get("local_planner/local_ws_y_max", local_ws_y_max);
    ros::param::get("local_planner/local_ws_z_max", local_ws_z_max);
    ros::param::get("local_planner/local_ws_x_min", local_ws_x_min);
    ros::param::get("local_planner/local_ws_y_min", local_ws_y_min);
    ros::param::get("local_planner/local_ws_z_min", local_ws_z_min);
    ros::param::get("local_planner/global_ws_x_max", global_ws_x_max);
    ros::param::get("local_planner/global_ws_y_max", global_ws_y_max);
    ros::param::get("local_planner/global_ws_z_max", global_ws_z_max);
    ros::param::get("local_planner/global_ws_x_min", global_ws_x_min);
    ros::param::get("local_planner/global_ws_y_min", global_ws_y_min);
    ros::param::get("local_planner/global_ws_z_min", global_ws_z_min);
    ros::param::get("local_planner/map_resolution", 				map_resolution);
    ros::param::get("local_planner/map_h_real_size_inflaction", 		map_h_real_size_inflaction);
    ros::param::get("local_planner/map_v_real_size_inflaction", 		map_v_real_size_inflaction);
    ros::param::get("local_planner/map_h_safe_distance_inflaction", 	map_h_safe_distance_inflaction);
    ros::param::get("local_planner/map_v_safe_distance_inflaction", 	map_v_safe_distance_inflaction);
    ros::param::get("local_planner/initial_point_factor", 	initial_point_factor);
    ros::param::get("local_planner/z_weight_cost", z_weight_cost);
    ros::param::get("local_planner/line_of_sight_margin", lofs_margin);
    ros::param::get("local_planner/traj_dxy_max", 	traj_dxy_max);
    ros::param::get("local_planner/traj_dz_max", 	traj_dz_max);
    ros::param::get("local_planner/traj_vxy_m", 	traj_vxy_m);
    ros::param::get("local_planner/traj_vz_m", 	traj_vz_m);
    ros::param::get("local_planner/traj_vxy_m_1", 	traj_vxy_m_1);
    ros::param::get("local_planner/traj_vz_m_1", 	traj_vz_m_1);
    ros::param::get("local_planner/traj_est_wy", 	traj_est_wy);
    ros::param::get("local_planner/traj_yaw_mode", 	traj_yaw_mode);
    ros::param::get("local_planner/cloud_points_timeout", cloud_points_timeout);
    
    // Debug
    printf("\nLocal Planner Configuration:\n");
    printf("\t Local Planner Frame:     %s\n", local_planner_frame.c_str());
    printf("\t Global Planner Frame:    %s\n", global_planner_frame.c_str());
    printf("\t Visual PointCloud Frame: %s\n", visual_pointcloud_frame.c_str());
    printf("\t Local WorkSpace:   X:[%f, %f], Y:[%f, %f], Z:[%f, %f] \n", local_ws_x_min, local_ws_x_max, local_ws_y_min, local_ws_y_max, local_ws_z_min, local_ws_z_max);
    printf("\t Global WorkSpace:  X:[%f, %f], Y:[%f, %f], Z:[%f, %f] \n", global_ws_x_min, global_ws_x_max, global_ws_y_min, global_ws_y_max, global_ws_z_min, global_ws_z_max);
    printf("\t Cloud Points life: = [%f sec, %d cycles] \n", cloud_points_timeout, (unsigned char) (cloud_points_timeout*POINTCLOUD_RATE));
    printf("\t Map: resol.= [%f], inflac.= [%f, %f]\n", map_resolution, map_h_real_size_inflaction + map_h_safe_distance_inflaction, map_v_real_size_inflaction + map_v_safe_distance_inflaction);
    printf("\t Lazy Theta* with optim.:    initial_point_factor = [%f]\n", initial_point_factor);
    printf("\t Lazy Theta* weighted:       z_w    = [%f]\n", z_weight_cost);
    printf("\t Lazy Theta* line of sight:  margin = [%f]\n", lofs_margin);
    printf("\t Trajectory position result: Pos. inc. = [%f, %f], Cruising Speed = [%f, %f] (%f, %f)\n", traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1);
    printf("\t Trajectory yaw result:      Yaw Mode = [%d], Yaw angular velocity = [%f]\n", traj_yaw_mode, traj_est_wy);
    printf("\t Re-planning stats:   	   max accepted errors = [%f]\n\n", MAX_REPLANNING_ERRORS);
    
	//! Dynamic reconfigure parameters
	dynamic_reconfigure::Server<local_planner::localPlannerConfig> dyn_rec_server;
	dynamic_reconfigure::Server<local_planner::localPlannerConfig>::CallbackType dyn_rec_f;
	dyn_rec_f = boost::bind(&dynRecCallback, _1, _2);
	dyn_rec_server.setCallback(dyn_rec_f);
	newDynRecParam = false;
    bool publishPcloudAtWorld = false;
    bool publishOccMatrixAtWorld = false;
    bool publishOccMatrixAtLocal = false;
    
    //! Sub-trajectories
    // Flag to know if the node is executing a received global trajectory
	bool executing_global_traj = false;
    // Global Sub-trajectory (only with unexecuted waypoints)
	TrajectoryPtr global_sub_trajectory(new Trajectory);
	global_sub_trajectory->joint_names.push_back("ground_frame");  
	global_sub_trajectory->header.stamp = ros::Time::now();
	global_sub_trajectory->header.frame_id = global_planner_frame;
	// Local Sub-trajectory (= Global Sub-trajectory transformed to the sensors frame)
	TrajectoryPtr local_sub_trajectory(new Trajectory);
	local_sub_trajectory->joint_names.push_back("ground_frame");  
	local_sub_trajectory->header.stamp = ros::Time::now();
	local_sub_trajectory->header.frame_id = local_planner_frame; 
	// Local Sub-trajectory PATCH to avoid obstacles
	TrajectoryPtr local_sub_trajectory_patch(new Trajectory);
	local_sub_trajectory_patch->joint_names.push_back("ground_frame");  
	local_sub_trajectory_patch->header.stamp = ros::Time::now();
	local_sub_trajectory_patch->header.frame_id = global_planner_frame;
    // Replanned Global Sub-trajectory 
	TrajectoryPtr replanned_global_sub_trajectory(new Trajectory);
	replanned_global_sub_trajectory->joint_names.push_back("ground_frame");  
	replanned_global_sub_trajectory->header.stamp = ros::Time::now();
	replanned_global_sub_trajectory->header.frame_id = global_planner_frame;
    
    //! Init ThetaStar Planner
    ThetaStar theta("local_planner", (char *)local_planner_frame.c_str(), local_ws_x_max, local_ws_y_max, local_ws_z_max, local_ws_x_min, local_ws_y_min, local_ws_z_min, map_resolution, map_h_real_size_inflaction, map_v_real_size_inflaction, map_h_safe_distance_inflaction, map_v_safe_distance_inflaction, initial_point_factor, z_weight_cost, &n);
    theta.initAuxDiscreteMatrix(global_ws_x_max, global_ws_y_max, global_ws_z_max, global_ws_x_min, global_ws_y_min, global_ws_z_min, (unsigned char) (cloud_points_timeout*POINTCLOUD_RATE));
	theta.setTimeOut(theta_timeout);
	// Initial and final position for theta star
	Vector3 init,fin;
	// The worspace is centered in the origin of the map = UAV position, so the initial position is always zero
	init.x = 0.0;	init.y = 0.0;	init.z = 0.0;	
	if(!theta.setInitialPosition(init))
		ROS_ERROR("Local planner: Imposible set theta star initial position zero");
	
	//! Create a continuous workspace checker
	WorkSpaceChecker workspace(local_ws_x_max, local_ws_x_min, local_ws_y_max, local_ws_y_min, local_ws_z_max, local_ws_z_min);	
	
	//! TF listener and transformation
	tf::TransformListener tf_listener;
	tf::StampedTransform global2local;
	tf::StampedTransform local2global;
	tf::StampedTransform sensor2local;

	//! Point Cloud in global frame
	PointCloud cloud_in_global;
	cloud_in_global.header.frame_id = global_planner_frame;

    //! Trajectory solution visualization marker
    visualization_msgs::Marker traj_marker;
	traj_marker.points.clear();
    traj_marker.header.frame_id = "world";
    traj_marker.header.stamp = ros::Time();
    traj_marker.ns = "local_planner";
    traj_marker.id = 21;
    traj_marker.type = visualization_msgs::Marker::CUBE_LIST;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = 0.2;
    traj_marker.scale.y = 0.2;
    traj_marker.scale.z = 0.2;
    traj_marker.color.a = 0.5;
    traj_marker.color.r = 0.0;
    traj_marker.color.g = 1.0;
    traj_marker.color.b = 0.0;    
	
	//! Wait frames initialization waiting to get the lastest available transform
	// .. from /global to /local
	ROS_INFO("Local Planner: waiting for frames intialization...");
	int i = 0;
	bool frames_init = false;
	while(!frames_init)
	{
		try
		{
		  frames_init = true;
		  tf_listener.lookupTransform(local_planner_frame, global_planner_frame, ros::Time(0), global2local);
		}
		catch (tf::TransformException &ex) 
		{
		  frames_init = false;
		  i++;
		  if(i>5)
		  {
			  ROS_WARN("%s",ex.what());
			  i=0;
		  }
		  ros::Duration(0.01).sleep();
		}
	}
	// ... from /sensor to /local
	i = 0;
	frames_init = false;
	while(!frames_init)
	{
		try
		{
		  frames_init = true;
		  tf_listener.lookupTransform(local_planner_frame, visual_pointcloud_frame, ros::Time(0), sensor2local);
		}
		catch (tf::TransformException &ex) 
		{
		  frames_init = false;
		  i++;
		  if(i>5)
		  {
			  ROS_WARN("%s",ex.what());
			  i=0;
		  }
		  ros::Duration(0.01).sleep();
		}
	}
	
	//! Wait to get the first PointCloud
    ROS_INFO("Local Planner: waiting for point cloud...");
    while(ros::ok() && !pointcloud_received)
    {
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	pointcloud_received = false;
		
	//! Simply zero translation and sensor to local translation as rotation to the crop_and_voxelize function
	Transform box_pose;	
	box_pose.translation.x = 0.0; box_pose.translation.y = 0.0; box_pose.translation.z = 0.0;
	box_pose.rotation.x = sensor2local.inverse().getRotation().x();
	box_pose.rotation.y = sensor2local.inverse().getRotation().y();
	box_pose.rotation.z = sensor2local.inverse().getRotation().z();
	box_pose.rotation.w = sensor2local.inverse().getRotation().w();

	//! Global trajectory or sub-trajectory time mark
	ros::Time initial_time = ros::Time::now();
	
	ROS_INFO("Local Planner: Ready!");
	
	//! Feedback topic to WAITING state
	//status.header.stamp = ros::Time::now();
	//status.state = WAITING;
	//status_pub.publish(status);
	
	/*************************** WHILE CYCLE *************************/
	ros::Rate rate(POINTCLOUD_RATE);
	while(ros::ok())
	{	
		/************************ POINT CLOUD WAITING **********************/
		do{
			// sleep
			rate.sleep();
			
			// Reading odometry, point cloud and/or new global trajectory
			ros::spinOnce();
        }while(!pointcloud_received && ros::ok());
		
		pointcloud_received = false;
		
		/******************** GLOBAL TRAJECTORY CHECKING *******************/
		// Check if a new global trajectory has been received and execute it! (TOTAL PRIORITY)
		if(global_traj_received)
		{	
			// update flag
			global_traj_received = false;
			executing_global_traj = true;
			local_planner_fail = false;
			
			// send the new global trajectory to the trajectory tracker
			trajectory_pub.publish(global_trajectory);
			
			// init global sub-trajectory to start replanning
			*global_sub_trajectory 	= global_trajectory;
			
			// Time mark to this global trajectory
			initial_time = ros::Time::now();
			
			// Debug
			#ifdef PRINT_GLOBAL_TRAJECTORY
				printf_trajectory(global_trajectory, "Global");
			#endif
			
			//! Feedback topic to RUNNING state
			//status.header.stamp = ros::Time::now();
			//status.state = RUNNING;
			//status_pub.publish(status);
		}
		
		
		/********************* LOCAL TRAJECTORY REPLANNING ******************/
		// ... execute it locally!
		if(executing_global_traj && global_sub_trajectory->points.size() > 0)
		{
			// Debug
			#ifdef PRINT_REPLANNING_TIME
				double replanning_time = ros::Time::now().toSec();
			#endif
			
			// Get the global subtrajectory (eliminating past time waypoints)
			//if(get_dist(global_sub_trajectory->points[0].transforms[0].translation, odom_pose.translation) <= MIN_WP_DIST)
			//	global_sub_trajectory->points.erase(global_sub_trajectory->points.begin());
			bool existOldWp = true;
			while(existOldWp && global_sub_trajectory->points.size() > 0)
			{
				if(global_sub_trajectory->points.front().time_from_start.toSec() <= ros::Time::now().toSec() - initial_time.toSec())
					global_sub_trajectory->points.erase(global_sub_trajectory->points.begin());
				else
					existOldWp = false;
			}
			
			// Debug
			#ifdef PRINT_GLOBAL_SUB_TRAJECTORY
				printf_trajectory(*global_sub_trajectory, "Global Sub");
			#endif
			
			// Get the transform from /world to /local at the point cloud stamp (lastest available)
			try
			{
			  tf_listener.waitForTransform(local_planner_frame, global_planner_frame, cloud_time_stamp, ros::Duration(0.5));
			  tf_listener.lookupTransform(local_planner_frame, global_planner_frame, cloud_time_stamp, global2local);
			  //tf_listener.lookupTransform(local_planner_frame, global_planner_frame, ros::Time(0), global2local);
			}
			catch (tf::TransformException &ex) {
			  ROS_ERROR("%s",ex.what());
			  continue;
			}
			
			// .. and from /local to /world
			local2global.setData(global2local.inverse());
			
			//~ ROS_INFO("LOCAL2GLOBAL from %s to %s at dt=%f secs from the point cloud", global2local.child_frame_id_.c_str(), global2local.frame_id_.c_str(),  cloud_time_stamp.toSec() - global2local.stamp_.toSec());
			
			// Transform the global sub-trajectory to the local reference frame
			local_sub_trajectory->points.clear();
			local_sub_trajectory->points.resize(global_sub_trajectory->points.size());
			local_sub_trajectory->points[0].transforms.resize(1);
			local_sub_trajectory->points[0].velocities.resize(1);
			local_sub_trajectory->points[0].accelerations.resize(1);
			for(unsigned int i=0; i < global_sub_trajectory->points.size();i++)
			{
				local_sub_trajectory->points[i].transforms.resize(1);
				local_sub_trajectory->points[i].velocities.resize(1);
				local_sub_trajectory->points[i].accelerations.resize(1);
				local_sub_trajectory->points[i].time_from_start = global_sub_trajectory->points[i].time_from_start;
				getTransformedPoint(global_sub_trajectory->points[i].transforms[0], local_sub_trajectory->points[i].transforms[0], global2local);	// if local_frame is stabilized it is not needed to eliminate roll&pitch
			}
			
			// Debug
			#ifdef PRINT_LOCAL_SUB_TRAJECTORY
				printf_trajectory(*local_sub_trajectory, "Local Sub");
			#endif
		
			// Ony Crop Box the point cloud around the UAV
			pcl_cropBox(cloud_in_sensor, local_ws_x_max, local_ws_y_max, local_ws_z_max, local_ws_x_min, local_ws_y_min, local_ws_z_min, box_pose);
			
			// Transform (from /sensor to /global) the point cloud to the world frame
			cloud_in_global.header.stamp = cloud_in_sensor.header.stamp;
			pcl_ros::transformPointCloud(global_planner_frame, cloud_in_sensor, cloud_in_global, tf_listener);
		
			// Build thetaStar local discrete map
			// Note: It uses the updateMap(PointCloud, transform) overloaded function, so build the world auxiliar discrete matrix		
			theta.updateMap(cloud_in_global, local2global);			

			// PointCloud topics for debug
			if(publishPcloudAtWorld)
				cloud_in_world_pub.publish(cloud_in_global);

			// Publish thetaStar auxiliar occupancy map
			if(publishOccMatrixAtWorld)
				theta.publishAuxOccupationMarkersMap();

			// Publish thetaStar occupancy map
			if(publishOccMatrixAtLocal)
				theta.publishOccupationMarkersMap();
						
			// Re-planning locally
			bool existReplanning = false;
			int last_wp_inside_id = 0;
			int last_wp_id = local_sub_trajectory->points.size() - 1;
			if(local_sub_trajectory->points.size() > 0)
			{
				// Get last connected waypoint inside the local workspace
				while(workspace.isInside(local_sub_trajectory->points[last_wp_inside_id].transforms[0].translation) && local_sub_trajectory->points.size() > 1)
				{
					cout << "Esta dentro del workspace" << endl;
					last_wp_inside_id++;
					
					// If the last waypoint of the trajectory is inside stop to not try to access to a non-existing wp
					if(last_wp_inside_id > last_wp_id)
						break;
				}
				if(last_wp_inside_id>0)
					last_wp_inside_id--;
					
				ROS_INFO("Last wp inside the ws: %d/%d: [%f, %f, %f]", last_wp_inside_id+1, local_sub_trajectory->points.size(), local_sub_trajectory->points[last_wp_inside_id].transforms[0].translation.x, local_sub_trajectory->points[last_wp_inside_id].transforms[0].translation.y, local_sub_trajectory->points[last_wp_inside_id].transforms[0].translation.z);
								
				// Origin, middle and goal waypoints
				int origin = 0;
				int middle = 0;
				int target = 1;
				// Start and Goal positions
				Vector3 init,fin;
				// Flags to know if they ar inside the workspace or occupied
				bool initial_position_inside_ws, final_position_inside_ws;
				bool initial_position_occupied, final_position_occupied;
				// Flag to know if the last replanning was to the trajectory last position and it was outside the ws or occupied
				bool replanning_to_a_non_valid_traj_last_position = false;
			    bool existLocalReplanning = false;
			    
			    // Check lineofSight between all connected nodes and, if it does not exist, recalculate paths from each wp to the next waypoint with LofS, and modificating the local sub-trajectory
				while(target<=last_wp_inside_id)
				{	
					bool existLineOfSight = soft_lineOfSight_checker(theta, local_sub_trajectory->points[middle].transforms[0].translation, local_sub_trajectory->points[target].transforms[0].translation, map_resolution, lofs_margin);
					ROS_INFO("Checking lineOfSight between %d/%d and %d/%d: %d", middle+1, local_sub_trajectory->points.size(), target+1, local_sub_trajectory->points.size(), existLineOfSight);
					
					if(existLineOfSight)
						ROS_INFO("Line of Sight exists!");

					// If not exist LOF --> Exist replanning and continue checking until get a LofS correctly (stop checking if it is the last wp in the ws)
					if(!existLineOfSight)
						existLocalReplanning = true;
						
					if(!existLineOfSight && target != last_wp_inside_id)
					{
						middle++;
						target++;
					}
					else
					{
						if(existLocalReplanning)
						{
							// Save the last time at origin and target 
							ros::Duration last_origin_time = local_sub_trajectory->points[origin].time_from_start;
							ros::Duration last_target_time = local_sub_trajectory->points[target].time_from_start;

							// Compute path from origin to target (PATCH)
							init = local_sub_trajectory->points[origin].transforms[0].translation;
							fin  = local_sub_trajectory->points[target].transforms[0].translation;
							
							/// Debug
							ROS_INFO("Local Planner:  Replanning from %d/%d to %d/%d",  origin + 1, local_sub_trajectory->points.size(), target + 1, local_sub_trajectory->points.size());
														
							// Check if they are in the workspace and if they are occupied
							initial_position_inside_ws = theta.setInitialPosition(init);
							initial_position_occupied = false;
							if(initial_position_inside_ws)
							{
								initial_position_occupied = theta.isInitialPositionOccupied();
								if(initial_position_occupied)
									ROS_WARN("Local Planner: Initial position occupied");
							}
							else
								ROS_ERROR("Local Planner: Initial Position is outside the workspace");
							
							final_position_inside_ws = theta.setFinalPosition(fin);
							final_position_occupied = false;
							if(final_position_inside_ws)
							{
								final_position_occupied = theta.isFinalPositionOccupied();
								if(final_position_occupied)
									ROS_WARN("Local Planner: Final position occupied");
							}
							else
								ROS_ERROR("Local Planner: Final Position is outside the workspace");
														
							// If the initial position is outside the workspace or it is occupied --> search a valid free initial position (Typical problem: The UAV is inside the inflated obstacle) 
							bool initial_position_correct = true;
							if(!initial_position_inside_ws || initial_position_occupied)
							{
								// Check another posibilities
								if(!theta.free_initial_position_searcher_3d(2.0))
								{
									ROS_ERROR("Local Planner: Imposible to find a free initial position!!");
									initial_position_correct = false;
								}
							}
							
							// If the final position is outside the workspace or it is occupied --> search a valid free final position 
							bool final_position_correct = true;
							if(!final_position_inside_ws || final_position_occupied)
							{
								// Check another posibilities
								if(!theta.free_final_position_searcher_3d(2.0))
								{
									ROS_ERROR("Local Planner: Imposible to find a free final position!!");
									final_position_correct = false;
								}
							}						
														
							// Check if the replanning is for the trajectory last position and it is outside the workspace or occupied 
							// 	    *  if the last replanning is to traject last point  *  and  *           final position was not valid             *
							if(( target + 1 == local_sub_trajectory->points.size() )  &&   (!final_position_inside_ws || final_position_occupied) )
							{
								//~ ROS_INFO("Final trajectory position not valid: (%d, %d) (%d, %d)", target, local_sub_trajectory->points.size(), final_position_inside_ws, final_position_occupied);
								replanning_to_a_non_valid_traj_last_position = true;
							}
							
							// Calculate a path if initial and final position are correct
							int patch_num_points = 0;
							if(final_position_correct && initial_position_correct)
								patch_num_points = theta.calculateNewPath();

							//vector<Vector3> patch = theta.getCurrentPath();
							//for(int i; i<patch_num_points; i++)
								//printf("Patch [%d]:\t%f\t%f\t%f\n", i, patch[i].x, patch[i].y, patch[i].z);

							// If exist a possible path set the patch
							if(patch_num_points>0)
							{
								// Get the trayectory patch from origin to target (BOTH INCLUDED!!). The yaw will be computed as 'traj_yaw_in_advance'
								//~ printf("patch points number: %d\n", patch_num_points);
								local_sub_trajectory_patch->points.clear();
								local_sub_trajectory_patch->header.stamp = ros::Time::now();
								double last_wp_yaw;
								switch(traj_yaw_mode)
								{
									case 0:		
											theta.getCurrentTrajectory_YawCte(local_sub_trajectory_patch, local_sub_trajectory->points[origin].transforms[0], traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1);
											break;

									case 1:		
											theta.getCurrentTrajectory_YawAtTime(local_sub_trajectory_patch, local_sub_trajectory->points[origin].transforms[0], traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1, traj_est_wy);
											break;

									case 2:
											last_wp_yaw = get_yaw_from_quat(local_sub_trajectory->points[target].transforms[0].rotation);
											theta.getCurrentTrajectory_YawInAdvance_WithFinalYaw(local_sub_trajectory_patch, local_sub_trajectory->points[origin].transforms[0], traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1, traj_est_wy, last_wp_yaw);
											break;

									default:
											last_wp_yaw = get_yaw_from_quat(local_sub_trajectory->points[target].transforms[0].rotation);
											theta.getCurrentTrajectory_YawInAdvance_WithFinalYaw(local_sub_trajectory_patch, local_sub_trajectory->points[origin].transforms[0], traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1, traj_est_wy, last_wp_yaw);
								}
																
								// printf("trajectory points number: %d\n", local_sub_trajectory_patch->points.size());
								
								// Overwrite the final (target) patch position. If not, it diverges because of the planner points discretization
								// Warning: not overwrite if it was occupied or outside the workspace
								if(final_position_inside_ws && !final_position_occupied)
									local_sub_trajectory_patch->points.back().transforms[0].translation = local_sub_trajectory->points[target].transforms[0].translation;
																
								// Debug
								#ifdef PRINT_LOCAL_SUB_TRAJECTORY_PATCH
									printf_trajectory(*local_sub_trajectory_patch, "Patch for Local Sub");
								#endif
								
								//If the first patch wp (is not at zero time and) is bigger than the elapse time on the local subtrajectory at the origin of the patch, then the 'time origin' is from the previous wp.
								//else, this initial elapse time in the patch must be eliminated (for example, by subtracting it in the 'last_origin_time')
								if(origin != 0)
								{
									ros::Duration last_elapse_time = local_sub_trajectory->points[origin].time_from_start - local_sub_trajectory->points[origin-1].time_from_start;
									if(local_sub_trajectory_patch->points.front().time_from_start.toSec() >  last_elapse_time.toSec())
										last_origin_time =  local_sub_trajectory->points[origin-1].time_from_start;
									else
										last_origin_time -= local_sub_trajectory_patch->points.front().time_from_start;
									
								}
								
								// Add origin time to the patch
								for(int i=0; i<local_sub_trajectory_patch->points.size(); i++)
									local_sub_trajectory_patch->points[i].time_from_start += last_origin_time;

								// Get new target time
								ros::Duration new_target_time = local_sub_trajectory_patch->points.back().time_from_start;

								// Add patch_duration to the local trajectory
								for(int i=target; i<local_sub_trajectory->points.size(); i++)
									local_sub_trajectory->points[i].time_from_start += new_target_time - last_target_time;
								
								// Eliminate the local subtrajectory points from origin to target 
								local_sub_trajectory->points.erase(local_sub_trajectory->points.begin() + origin, local_sub_trajectory->points.begin() + target + 1);							
								
								// Insert the patch in the local subtrajectory								
								for(int i=local_sub_trajectory_patch->points.size()-1; i>=0 ; i--)
								{
									local_sub_trajectory->points.insert(local_sub_trajectory->points.begin() + origin, local_sub_trajectory_patch->points[i]);
								}
								
								// Turn on the flag to send the new trajectory 
								existReplanning = true;
							}
							else
							{
								replanning_errors_number++;
								ROS_ERROR("Local Planner: Replan from %d to %d fail, it doesn't exist a possible trajectory [Replanning failure number %d]", origin, target, replanning_errors_number);
								if(replanning_errors_number>=MAX_REPLANNING_ERRORS)
								{
									// Advertise the fail
									local_planner_fail =true;
									// Stop the trajectroy execution
									executing_global_traj = false;
								}	
							}
						}
						
						// Next step
						existLocalReplanning = false;
						origin = target;
						middle = target;
						target++;
					}
				}
				
				// Transform to global and Send the new trajectory if it has been modified 
				if(existReplanning)
				{
					
					// Debug
					#ifdef PRINT_REPLANNED_LOCAL_SUB_TRAJECTORY
						printf_trajectory(*local_sub_trajectory, "Replanned Local Sub");
					#endif
					
					// POST-PROCESSING of new local trajectory before send it (Eliminating middle visible waypoints & recalculating yaw and times)					
					switch(traj_yaw_mode)
					{
						case 0:		
								post_process_trajectory_yaw_cte (local_sub_trajectory, odom_pose, theta, traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1);
								break;

						case 1:		
								post_process_trajectory_yaw_at_time (local_sub_trajectory, odom_pose, theta, traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1, traj_est_wy);
								break;

						case 2:		
								post_process_trajectory_yaw_in_advance (local_sub_trajectory, odom_pose, theta, traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1, traj_est_wy);
								break;

						default:	
								post_process_trajectory_yaw_in_advance (local_sub_trajectory, odom_pose, theta, traj_dxy_max, traj_dz_max, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1, traj_est_wy);
					}
					

					// Debug
					#ifdef PRINT_REPLANNED_LOCAL_SUB_TRAJECTORY_POST_PROCESSED
						printf_trajectory(*local_sub_trajectory, "Post-processed Local Sub");
					#endif

					
					// Transform the local sub-trajectory to the global reference frame (WARNING: FOT THE TRAJECTORY_TRACKER FIRST WP TIME MUST BE ZERO AND ALL REST FROM THAT/ZERO)
					global_sub_trajectory->points.clear();
					global_sub_trajectory->points.resize(local_sub_trajectory->points.size());
					for(unsigned int i=0; i < local_sub_trajectory->points.size();i++)
					{
						global_sub_trajectory->points[i].transforms.resize(1);
						global_sub_trajectory->points[i].velocities.resize(1);
						global_sub_trajectory->points[i].accelerations.resize(1);
						//~ global_sub_trajectory->points[i].time_from_start = local_sub_trajectory->points[i].time_from_start - local_sub_trajectory->points[0].time_from_start;
						global_sub_trajectory->points[i].time_from_start = local_sub_trajectory->points[i].time_from_start;
						//~ getTransformedPointOnlyTurnYaw(local_sub_trajectory->points[i].transforms[0], global_sub_trajectory->points[i].transforms[0], local2global);
						getTransformedPoint(local_sub_trajectory->points[i].transforms[0], global_sub_trajectory->points[i].transforms[0], local2global);
					}
					
					// OverWrite Global Final position (if not, it will be modified bit by bit)
					// Warning: not overwrite if the replanning is to the last trajectory position and it was outside the workspace or occupied
					if(!replanning_to_a_non_valid_traj_last_position)
					{
						//~ ROS_INFO("Overwritting final trajectory position");
						global_sub_trajectory->points.back().transforms.resize(1);
						global_sub_trajectory->points.back().velocities.resize(1);
						global_sub_trajectory->points.back().accelerations.resize(1);
						global_sub_trajectory->points.back().transforms[0] = global_trajectory.points.back().transforms[0]; 
					}
					else
					{
						global_trajectory.points.back().transforms[0] = global_sub_trajectory->points.back().transforms[0];
					}
					
					// Debug
					#ifdef PRINT_REPLANNED_GLOBAL_SUB_TRAJECTORY
						printf_trajectory(*global_sub_trajectory, "Replanned Global Sub");
					#endif
					
					// Update initial_time (origin time for next "Get the global subtrajectory (eliminating past time waypoints)")
					initial_time = ros::Time::now();
								
					// Send it
					global_sub_trajectory->header.stamp = ros::Time::now();
					trajectory_pub.publish(global_sub_trajectory);
				}
				
				// DEBUG: Parse to markers and draw the local trajectory
				traj_marker.points.clear();
				traj_marker.header.stamp = ros::Time();
				for(unsigned int i=0; i < global_sub_trajectory->points.size();i++)
				{
					geometry_msgs::Point p;
					p.x = global_sub_trajectory->points[i].transforms[0].translation.x;
					p.y = global_sub_trajectory->points[i].transforms[0].translation.y;
					p.z = global_sub_trajectory->points[i].transforms[0].translation.z;
					traj_marker.points.push_back(p);
				}
				vis_pub_traj.publish(traj_marker);
			}
			
			// Clear theta* occupancy map (setting all elements to zero or not occupied)
			// Note: Its clear the local occupancy matrix used by theta* algorithm, but not the world auxiliar matrix that dynamically clear itself
			theta.clearMap();
			
			#ifdef PRINT_REPLANNING_TIME
				printf(PRINTF_GREEN "While Cycle Time: %f\n" PRINTF_REGULAR, ros::Time::now().toSec() - replanning_time);
			#endif
		}
		else
		{
			// Set global trajectory as executed = Stop trajectory replanning checking
			executing_global_traj = false;
			
			// Reset re-planning errors to zero
			replanning_errors_number = 0;
			
			if(!local_planner_fail)
			{
				//! Feedback topic to WAITING state
				//status.header.stamp = ros::Time::now();
				//status.state = WAITING;
				//status_pub.publish(status);
				printf(PRINTF_GREEN "WAITING\n" PRINTF_REGULAR);
			
			}
			else
			{
				//! Feedback topic to ERROR state
				//status.header.stamp = ros::Time::now();
				//status.state = ERROR;
				//status_pub.publish(status);
				printf(PRINTF_GREEN "ERROR\n" PRINTF_REGULAR);
			
			}
			
			// Check and Changes Dynamic Reconfigure Parameters
			if(newDynRecParam)
			{
			  newDynRecParam = false;
			  
			  publishPcloudAtWorld = dyn_rec_params.Pub_PCloud_AtWorld;
			  publishOccMatrixAtWorld = dyn_rec_params.Pub_OccupMatrix_AtWorld;
			  publishOccMatrixAtLocal = dyn_rec_params.Pub_OccupMatrix_AtLocal;
			  
			  ROS_INFO("Local Planner: dynamic reconfigured parameters update");
			}
		}
		
	}
	
    return 0;
}

