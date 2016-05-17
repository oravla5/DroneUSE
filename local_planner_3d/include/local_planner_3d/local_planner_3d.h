#include <ros/ros.h>

#include <cstdlib>
#include <string>
#include <math.h>

#include <theta_star/ThetaStar.h>

#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"

#include <stdio.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// Dynamic Reconfigure Parameters
#include <dynamic_reconfigure/server.h>
#include <local_planner_3d/localPlannerConfig.h>

#define PRINTF_REGULAR  "\x1B[0m"
#define PRINTF_RED  	"\x1B[31m"
#define PRINTF_GREEN  	"\x1B[32m"
#define PRINTF_YELLOW  	"\x1B[33m"
#define PRINTF_BLUE  	"\x1B[34m"
#define PRINTF_MAGENTA  "\x1B[35m"
#define PRINTF_CYAN  	"\x1B[36m"
#define PRINTF_WHITE	"\x1B[37m"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PathPlanners::ThetaStar 		ThetaStar;
typedef PathPlanners::DiscretePosition 	DiscretePosition;
typedef trajectory_msgs::MultiDOFJointTrajectory 	Trajectory;
typedef trajectory_msgs::MultiDOFJointTrajectoryPtr TrajectoryPtr;
typedef geometry_msgs::Transform 		Transform;
typedef geometry_msgs::Vector3 			Vector3;
typedef geometry_msgs::Quaternion 		Quaternion;



//! Dynamic Reconfigure Callbak
local_planner::localPlannerConfig dyn_rec_params;
bool newDynRecParam;
void dynRecCallback(local_planner::localPlannerConfig &config, uint32_t level) 
{
	switch(level)
	{
		case 0:	newDynRecParam = true;
				break;
		
		default:	ROS_WARN("Dynamic reconfigure call received but parameter level is unknown");
	}

	dyn_rec_params = config;
}


//! Aux Class: Check is the point is inside the workspace
class WorkSpaceChecker
{
public:
   WorkSpaceChecker(float x_max_, float x_min_, float y_max_, float y_min_, float z_max_, float z_min_)
   {
	   x_max = x_max_;
	   x_min = x_min_;
	   y_max = y_max_;
	   y_min = y_min_;
	   z_max = z_max_;
	   z_min = z_min_;
   }
   
   bool isInside(Vector3 point)
   {
	  return  	(point.x < x_max && point.x >= x_min) &&
				(point.y < y_max && point.y >= y_min) &&
				(point.z < z_max && point.z >= z_min);
   }

   bool isInside(float x, float y, float z)
   { 
	  return  	(x < x_max && x > x_min) &&
				(y < y_max && y > y_min) &&
				(z < z_max && z > z_min);
   }

   float x_max, x_min, y_max, y_min, z_max, z_min;
};

//! Aux Function: Get differential from last_yaw to next_yaw in [-PI, PI], but in absolute value --> [0, PI] 
double get_dyaw(double next_yaw, double last_yaw)
{
	double dyaw = fabs(next_yaw - last_yaw);
		if(dyaw>M_PI)
			dyaw = fabs(dyaw - 2.0 * M_PI);
	
	return dyaw;
}

///! Aux Function: Get yaw in radians from a quaternion
double get_yaw_from_quat(Quaternion quat)
{
	double r, p, y;
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 M(q);
	M.getRPY(r, p, y);
	
	return y;
}

///! Aux Function: Get roll, pitch and yaw in radians from a quaternion
void get_rpy_from_quat(Quaternion quat, double &r, double &p, double &y)
{
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 M(q);
	M.getRPY(r, p, y);
}

//! Aux Function: Get the transformation of a point (geometry_msgs::Transform) 
/// for a specified tf:Transform
void getTransformedPoint(Transform point_at_world, Transform &point_at_local, tf::StampedTransform world2local)
{
	// Transform point at world to TF transformation point->world
	tf::StampedTransform point2world;
	tf::transformMsgToTF(point_at_world, point2world);
	
	// Get the transformation point->sensor = world->sensor * point->world
	tf::transformTFToMsg(world2local*point2world, point_at_local);
}

//! Aux Function: Get the transformation of a point (geometry_msgs::Transform) 
/// for a specified tf:Transform only urning yaw (not roll&pitch)
void getTransformedPointOnlyTurnYaw(Transform point_at_world, Transform &point_at_local, tf::StampedTransform world2local)
{
	// Transform point at world to TF transformation point->world
	tf::StampedTransform point2world;
	tf::transformMsgToTF(point_at_world, point2world);
	
	// Eliminate roll and pitch from world2local transform
	double yaw = tf::getYaw(world2local.getRotation());
	world2local.setRotation(tf::createQuaternionFromYaw(yaw));

	// Get the transformation point->sensor = world->sensor * point->world
	tf::transformTFToMsg(world2local*point2world, point_at_local);
}

//! Aux Function: simply print the trajectory msg data
void printf_trajectory(Trajectory trajectory, string trajectory_name)
{	
	printf(PRINTF_RED "%s trajectory [%d]:\n", trajectory_name.c_str(), trajectory.points.size());
	
	for(unsigned int i=0; i < trajectory.points.size();i++)
	{
		double yaw = get_yaw_from_quat(trajectory.points[i].transforms[0].rotation);
		printf(PRINTF_YELLOW "\t %d: [%f, %f, %f] m\t[%f] rad\t [%f] sec\n", i, trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.y, trajectory.points[i].transforms[0].translation.z, yaw , trajectory.points[i].time_from_start.toSec());
	}

	printf(PRINTF_REGULAR);
}

//! Aux Function: Point Cloud crop box
void pcl_cropBox(PointCloud &cloud, double box_x_max, double box_y_max, double box_z_max, double box_x_min, double box_y_min, double box_z_min, Transform box_pose)
{
	// Point cloud pointer
	PointCloud::Ptr cloud_ptr (new PointCloud);
	*cloud_ptr = cloud;

	// PointCloud Crop Box
	pcl::CropBox<pcl::PointXYZ> cloud_crop_box;
	cloud_crop_box.setInputCloud(cloud_ptr);
	Eigen::Vector3f boxRotation;
	double box_roll, box_pitch, box_yaw;
	get_rpy_from_quat(box_pose.rotation, box_roll, box_pitch, box_yaw);
	boxRotation[0]=box_roll;
	boxRotation[1]=box_pitch;
	boxRotation[2]=box_yaw;
	cloud_crop_box.setRotation(boxRotation);
	Eigen::Vector3f boxTranslation;
	boxTranslation[0]=box_pose.translation.x;
	boxTranslation[1]=box_pose.translation.y;
	boxTranslation[2]=box_pose.translation.z;
	cloud_crop_box.setTranslation(boxTranslation);
	Eigen::Vector4f min_point = Eigen::Vector4f(box_x_min, box_y_min, box_z_min, 0.0);
	Eigen::Vector4f max_point = Eigen::Vector4f(box_x_max, box_y_max, box_z_max, 0.0);
	cloud_crop_box.setMin(min_point);
	cloud_crop_box.setMax(max_point);
	
	PointCloud::Ptr cloud_filtered_ptr 	(new PointCloud);
	cloud_crop_box.filter(*cloud_filtered_ptr);
	cloud = *cloud_filtered_ptr;
}

//! Aux Function: Point Cloud crop box and voxel grid
void pcl_cropBox_and_voxelGrid(PointCloud &cloud, double box_x_max, double box_y_max, double box_z_max, double box_x_min, double box_y_min, double box_z_min, Transform box_pose, double grid_size)
{
	// PointCloud Crop Box
	pcl_cropBox(cloud, box_x_max, box_y_max, box_z_max, box_x_min, box_y_min, box_z_min, box_pose);
	
	// PointCloud Voxel Grid 
	PointCloud::Ptr cloud_ptr 			(new PointCloud);
	PointCloud::Ptr cloud_filtered_ptr 	(new PointCloud);
	pcl::VoxelGrid<pcl::PointXYZ> cloud_grid;
	*cloud_ptr = cloud;
	cloud_grid.setInputCloud (cloud_ptr);
	cloud_grid.setLeafSize (grid_size, grid_size, grid_size);
	cloud_grid.filter (*cloud_filtered_ptr);
	cloud = *cloud_filtered_ptr;
}

/*************************************************** 
 * 
 * DEPRECATED: ONLY VALID FOR A 1 CELL MARGIN 
 * 
 * New function with 'margin' parameter
 * 
 * ************************************************/
//! Aux Function: Check lineOfSight between two nodes using a thetaStar 
/// algorithm. If it doesn't exist, check if it exists for proximity nodes					
//~ bool soft_lineOfSight_checker(ThetaStar &theta_, Vector3 &origin, Vector3 &goal, float step, float margin)
//~ {
	//~ // Discretize nodes
	//~ PathPlanners::ThetaStarNode n1;
	//~ n1.point.x = origin.x/step;
	//~ n1.point.y = origin.y/step;
	//~ n1.point.z = origin.z/step;
	//~ PathPlanners::ThetaStarNode n2;
	//~ n2.point.x = goal.x/step;
	//~ n2.point.y = goal.y/step;
	//~ n2.point.z = goal.z/step;
	//~ 
	//~ // Discretize margin
	//~ float margin_ = margin/step;
	//~ 
	//~ if(theta_.lineofsight(n1,n2))
	//~ {
		//~ // If there is lineOfSight direcly, return true
		//~ //printf("Exist LofS for [%f, %f, %f] --> [%f, %f, %f]\n", n1.point.x * step, n1.point.y * step, n1.point.z * step, n2.point.x * step, n2.point.y * step, n2.point.z * step);
		//~ return true;
	//~ }
	//~ else
	//~ {
		//~ // If there is not lineOfSight, check proximities
		//~ //printf("Not exist LofS for [%f, %f, %f] --> [%f, %f, %f]\n", origin.x, origin.y, origin.z, goal.x, goal.y, goal.z);
		//~ PathPlanners::ThetaStarNode n1_ = n1;
		//~ PathPlanners::ThetaStarNode n2_ = n2;
		//~ for(int j1=0; j1<=2; j1++)	// ... trying mod XY origin node if it's not enough ...
			//~ for(int i1=0; i1<=2; i1++)
				//~ for(int j2=0; j2<=2; j2++) // ... modifing only the XY goal node
					//~ for(int i2=0; i2<=2; i2++)
					//~ {				
						//~ if(i1 != 2)
							//~ n1_.point.x = n1.point.x + i1;
						//~ else
							//~ n1_.point.x = n1.point.x - 1;
						//~ if(j1 != 2)	
							//~ n1_.point.y = n1.point.y + j1;
						//~ else
							//~ n1_.point.y = n1.point.y -1;
						//~ 
						//~ 
						//~ if(i2 != 2)	
							//~ n2_.point.x = n2.point.x + i2;
						//~ else	
							//~ n2_.point.x = n2.point.x - 1;
						//~ if(j2 != 2)	
							//~ n2_.point.y = n2.point.y + j2;
						//~ else
							//~ n2_.point.y = n2.point.y - 1;
						//~ 
						//~ //printf("B) Trying [%f, %f, %f] --> [%f, %f, %f]\n", n1_.point.x * step, n1_.point.y * step, n1_.point.z * step, n2_.point.x * step, n2_.point.y * step, n2_.point.z * step);
									//~ 
						//~ if(theta_.lineofsight(n1_,n2_))
						//~ {
							//~ origin.x = n1_.point.x * step;
							//~ origin.y = n1_.point.y * step;
							//~ origin.z = n1_.point.z * step;
							//~ 
							//~ goal.x = n2_.point.x * step;
							//~ goal.y = n2_.point.y * step;
							//~ goal.z = n2_.point.z * step;
							//~ 
							//~ //printf("Changing to [%f, %f, %f] --> [%f, %f, %f]\n", origin.x, origin.y, origin.z, goal.x, goal.y, goal.z);
							//~ 
							//~ return true;
						//~ }
					//~ }
	//~ }
	//~ 
	//~ return false;
//~ }				



/**	Auxiliar function to check LofS between a specific origin node and all goal near nodes
                    7 
                    3	
    i --------> 6 2 f 1 5
                    4
                    8
**/			
bool check_LofS_mod_goal(ThetaStar &theta_, PathPlanners::ThetaStarNode &n1_, PathPlanners::ThetaStarNode &n2_, PathPlanners::ThetaStarNode &n2, int margin_)
{
	int i = 1;
	bool existLofS = false;
	
	while(!existLofS && i<=margin_)	// trying mod origin node
	{					
		if(!existLofS)
		{
			n2_.point.x = n2.point.x + i;
			n2_.point.y = n2.point.y;
			if(theta_.lineofsight(n1_,n2_))
				existLofS = true;
		}

		if(!existLofS)
		{
			n2_.point.x = n2.point.x - i;
			n2_.point.y = n2.point.y;
			if(theta_.lineofsight(n1_,n2_))
				existLofS = true;
		}

		if(!existLofS)
		{		
			n2_.point.x = n2.point.x;
			n2_.point.y = n2.point.y + i;
			if(theta_.lineofsight(n1_,n2_))
				existLofS = true;
		}
		
		if(!existLofS)
		{
			n2_.point.x = n2.point.x;
			n2_.point.y = n2.point.y - i;
			if(theta_.lineofsight(n1_,n2_))
				existLofS = true;
		}
		
		i++;
	}
	
	return existLofS;
}

//! Aux Function: Check lineOfSight between two nodes using a thetaStar 
/// algorithm. If it doesn't exist, check if it exists for proximity nodes inside the 'margin'		
bool soft_lineOfSight_checker(ThetaStar &theta_, Vector3 &origin, Vector3 &goal, float step, float margin)
{
	// Discretize nodes
	PathPlanners::ThetaStarNode n1;
	n1.point.x = origin.x/step;
	n1.point.y = origin.y/step;
	n1.point.z = origin.z/step;
	PathPlanners::ThetaStarNode n2;
	n2.point.x = goal.x/step;
	n2.point.y = goal.y/step;
	n2.point.z = goal.z/step;
	
	// Discretize margin
	float margin_ = margin/step;
	
	// Check LofS with original nodes
	if(theta_.lineofsight(n1,n2))
	{
		// If there is lineOfSight direcly, return true
		return true;
	}
	else
	{
		// If there is not lineOfSight, check near nodes
		PathPlanners::ThetaStarNode n1_ = n1;
		PathPlanners::ThetaStarNode n2_ = n2;
		
		int i = 0;
		bool existLofS = false;
		
		// For each near origin node try LofS for all near goal node
		while(!existLofS && i<=margin_)	
		{			
			n1_.point.x = n1.point.x + i;
			n1_.point.y = n1.point.y;
			if(!theta_.isOccupied(n1))
				existLofS = check_LofS_mod_goal(theta_, n1_, n2_, n2, margin_);
			
			if(i>0)
			{
				if(!existLofS)
				{
					n1_.point.x = n1.point.x - i;
					n1_.point.y = n1.point.y;
					if(!theta_.isOccupied(n1))
						existLofS = check_LofS_mod_goal(theta_, n1_, n2_, n2, margin_);
				}
				
				if(!existLofS)
				{
					n1_.point.x = n1.point.x;
					n1_.point.y = n1.point.y + i;
					if(!theta_.isOccupied(n1))
						existLofS = check_LofS_mod_goal(theta_, n1_, n2_, n2, margin_);
				}
				
				if(!existLofS)
				{
					n1_.point.x = n1.point.x;
					n1_.point.y = n1.point.y - i;
					if(!theta_.isOccupied(n1))
						existLofS = check_LofS_mod_goal(theta_, n1_, n2_, n2, margin_);
				}
			}
			
			i++;
		}
		
		// Set new visible nodes and return ok!
		if(existLofS)
		{			
			origin.x = n1_.point.x * step;
			origin.y = n1_.point.y * step;
			origin.z = n1_.point.z * step;
			
			goal.x = n2_.point.x * step;
			goal.y = n2_.point.y * step;
			goal.z = n2_.point.z * step;
						
			return true;
		}
	}
	
	return false;
}				

//! Aux Function: Check if 'v1' and 'v2' are less than 'dxy_max' 
/// horizontally and 'dz_max' vertically
bool are_close_enough(Vector3 v1, Vector3 v2, double dxy_max, double dz_max)
{
	double dx = v1.x - v2.x;
	double dy = v1.y - v2.y;
	double dz = v1.z - v2.z;
	
	if( sqrtf(dx * dx + dy * dy) <= dxy_max && fabs(dz) <= dz_max)
		return true;
	else
		return false;
}

//! Aux Function: Check if 'v1' and 'v2' are less than '2*step' 
bool are_so_close(Vector3 v1, Vector3 v2, double step)
{
	double dx = v1.x - v2.x;
	double dy = v1.y - v2.y;
	double dz = v1.z - v2.z;
	
	if( sqrtf(dx * dx + dy * dy) <= 2.0*step && fabs(dz) <= 2.0*step)
		return true;
	else
		return false;
}

//! Aux Function: Check if 'v1' and 'v2' points are closer to '3/4 dxy_max' 
/// meters horizontally.
bool are_too_close_horizontally(Vector3 v1, Vector3 v2, double dxy_max)
{
	double dx = v1.x - v2.x;
	double dy = v1.y - v2.y;
	static const double max_distance = 3.0*dxy_max/4.0;
	
	if( sqrtf(dx * dx + dy * dy) <= max_distance)
		return true;
	else
		return false;
}

//! Aux Function: return the module of [X,Y]
double get_horizontal_norm(double x, double y)
{
	return sqrtf(x * x + y * y);
}




//! Aux Function: Tajectory point to fill and send to the trajectory tracker -- Yaw always zero
void set_position_and_time(trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_point, Vector3 position, double T)
{	
  trajectory_point.transforms[0].translation.x = position.x;
  trajectory_point.transforms[0].translation.y = position.y;
  trajectory_point.transforms[0].translation.z = position.z;
  trajectory_point.transforms[0].rotation.x = 0.0;
  trajectory_point.transforms[0].rotation.y = 0.0;
  trajectory_point.transforms[0].rotation.z = 0.0;
  trajectory_point.transforms[0].rotation.w = 1.0;
  trajectory_point.velocities[0].linear.x = 0.0;
  trajectory_point.velocities[0].linear.y = 0.0;
  trajectory_point.velocities[0].linear.z = 0.0;
  trajectory_point.accelerations[0].linear.x = 0.0;
  trajectory_point.accelerations[0].linear.y = 0.0;
  trajectory_point.accelerations[0].linear.z = 0.0;
  trajectory_point.time_from_start = ros::Duration(T);
}

//! Aux Function: Trajectory post-processing. It eliminates the first 
/// waypoint nearest to the MAV and add wps along the straight line 
/// from local zero position to the first wp non-near .
/// WARNING: It does not recalculate the waypoints time
void first_wps_post_processing(TrajectoryPtr trajectory, double dxy_max, double dz_max)
{	
	//printf_trajectory(*trajectory, "Input first_wps_post_processing()");
		
	// Eliminate nearest wp to the MAV (stopping if wp number is less than 2)
	bool wp_deleted = true;
	while(wp_deleted &&  trajectory->points.size()>1)
	{
		if(get_horizontal_norm(trajectory->points[0].transforms[0].translation.x, trajectory->points[0].transforms[0].translation.y) < (dxy_max) && fabs(trajectory->points[0].transforms[0].translation.z) < dz_max)
		{
			trajectory->points.erase(trajectory->points.begin());
			wp_deleted = true;
		}
		else
			wp_deleted = false;
	}
	
	// Add neccesary middle wps from zero position to the first wp. HALF Max Distance 
	//~ dxy_max = dxy_max / 2.0;
	//~ dz_max = dz_max / 2.0;
	int traj_size = trajectory->points.size();
	if(traj_size>0)
	{
		// Zero position at local frame (current MAV position)
		Vector3 position_zero;
		position_zero.x = 0.0;
		position_zero.y = 0.0;
		position_zero.z = 0.0;
		trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
		trajectory_point.transforms.resize(1);
		trajectory_point.velocities.resize(1);
		trajectory_point.accelerations.resize(1);
		
		// loop last and next path positions
		Vector3 last_position = position_zero;
		Vector3 next_position;
		next_position.x = trajectory->points[0].transforms[0].translation.x;
		next_position.y = trajectory->points[0].transforms[0].translation.y;
		next_position.z = trajectory->points[0].transforms[0].translation.z;
		// loop middle trajectory position (needed if next position is so far)
		Vector3 middle_position = last_position;
		// Flags 
		bool pathPointGot = false;	// to know if a middle point is necessary
		bool Limited_V = false;		// to know if a the next point is limited vertically, to check correctly the horizontal limitation
		// middle waypoints number
		int middle_wp_num = 0;
		// trajectory max increments (where descomposes dxy_max)
		double DXmax, DYmax;
		
		// two path points loop: If the next waypoint is further than dz_max or dxy_max, add middle waypoints
		pathPointGot = false;		
		while(!pathPointGot)
		{			
			// At first, next point path is possible and not vertically limited
			pathPointGot = true;
			Limited_V = false;
			
			// Check Vertically Limit
			if(fabs(next_position.z - last_position.z) > (dz_max))
			{
				// Z to the max
				if(next_position.z - last_position.z > 0.0)
					middle_position.z = last_position.z + dz_max;
				else
					middle_position.z = last_position.z - dz_max;
				
				// XY proportional to the Z
				double alfa =  dz_max / fabs(next_position.z - last_position.z); // Porcentaje del recorrido que se va a realizar hasta esta Z limitada
				middle_position.x = last_position.x + alfa * (next_position.x - last_position.x);
				middle_position.y = last_position.y + alfa * (next_position.y - last_position.y);
				
				// Exist limitation, so the point is not directly reached 
				pathPointGot = false;
				Limited_V = true;
			}
			
			// Check Horizontal Limit 					
			switch(Limited_V)
			{
				case false:
							if(get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y) > (dxy_max))
							{
								// X and Y to the max (It exists a singularity at DX = 0.0, so if that directly set the known values)
								if(fabs(next_position.x - last_position.x) >= 0.001)
								{
									DXmax = dxy_max / ( sqrt(1.0 + pow(next_position.y - last_position.y, 2)/pow(next_position.x - last_position.x, 2)) );
									DYmax = fabs(next_position.y - last_position.y)/fabs(next_position.x - last_position.x) * DXmax;
								}
								else
								{
									DXmax = 0.0;
									DYmax = dxy_max;
								}
								
								if(next_position.x - last_position.x > 0.0)
									middle_position.x = last_position.x + DXmax;
								else
									middle_position.x = last_position.x - DXmax;
									
								if(next_position.y - last_position.y > 0.0)
									middle_position.y = last_position.y + DYmax;
								else	
									middle_position.y = last_position.y - DYmax;
								
								// Z proportional to the XY limited
								double beta =  dxy_max / get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y);
								middle_position.z = last_position.z + beta * (next_position.z - last_position.z);
								
								// Exist limitation, so the point is not directly reached 
								pathPointGot = false;
							}
							
							break;
											
				case true:
							if(get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y) > (dxy_max))
							{
								// X and Y to the max (It exists a singularity at DX = 0.0, so if that directly set the known values)
								if(fabs(middle_position.x - last_position.x) >= 0.001)
								{
									DXmax = dxy_max/(sqrt(1.0 + pow(middle_position.y - last_position.y, 2)/pow(middle_position.x - last_position.x, 2)));
									DYmax = fabs(middle_position.y - last_position.y)/fabs(middle_position.x - last_position.x) * DXmax;
								}
								else
								{
									DXmax = 0.0;
									DYmax = dxy_max;
								}
								
								// Z proportional to the XY limited (it need to be do previous to modificate middle_position.x and .y)
								double beta =  dxy_max / get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y);
								middle_position.z = last_position.z + beta * (middle_position.z - last_position.z);
								
								if(next_position.x - last_position.x > 0.0)
									middle_position.x = last_position.x + DXmax;
								else
									middle_position.x = last_position.x - DXmax;
									
								if(next_position.y - last_position.y > 0.0)
									middle_position.y = last_position.y + DYmax;
								else	
									middle_position.y = last_position.y - DYmax;
								
								// Exist limitation, so the point is not directly reached 
								pathPointGot = false;
							}
							
							break;
			}
			
			// Set the intermediate position if path point has not been got
			if(!pathPointGot)
			{
				// Set path position (last_position is the original last_position or the last middle_position if it exists)				
				set_position_and_time(trajectory_point, middle_position, 0.0);
				trajectory->points.insert(trajectory->points.begin() + middle_wp_num, trajectory_point);
				middle_wp_num++;
				
				//.. to the algorihtm
				last_position = middle_position;
			}
		}	
	}

	//printf_trajectory(*trajectory, "Output first_wps_post_processing()");
}



//! Aux Function: Trajectory post-processing. It eliminates the middle 
/// waypoint between three-consecutives visible nodes.
/// WARNING: It does not recalculate the waypoints time
void near_visible_wp_post_processing(TrajectoryPtr trajectory, ThetaStar &theta_, double dxy_max, double dz_max)
{
	int traj_size = trajectory->points.size();
	
	int index = 0;
	int traj_final_wp_index = traj_size - 1;
	double step = theta_.getMapResolution();
	PathPlanners::ThetaStarNode n1;				// First node
	PathPlanners::ThetaStarNode n2;				// Second node
	PathPlanners::ThetaStarNode n3;				// Thrid node
	
	// From first wp to last valid-checking wp
	while(index <= traj_final_wp_index-2)
	{
		// First node
		n1.point.x = trajectory->points[index].transforms[0].translation.x/step;
		n1.point.y = trajectory->points[index].transforms[0].translation.y/step;
		n1.point.z = trajectory->points[index].transforms[0].translation.z/step;
		
		// Second node
		n2.point.x = trajectory->points[index].transforms[0].translation.x/step;
		n2.point.y = trajectory->points[index].transforms[0].translation.y/step;
		n2.point.z = trajectory->points[index].transforms[0].translation.z/step;		
		
		// Third node
		n3.point.x = trajectory->points[index+2].transforms[0].translation.x/step;
		n3.point.y = trajectory->points[index+2].transforms[0].translation.y/step;
		n3.point.z = trajectory->points[index+2].transforms[0].translation.z/step;
		
		// First and second node are so close 
		bool areSoClose = are_so_close(trajectory->points[index].transforms[0].translation, trajectory->points[index+1].transforms[0].translation, step);
		
		// First and third node are close enough? 
		bool areCloseEnough = are_close_enough(trajectory->points[index].transforms[0].translation, trajectory->points[index+2].transforms[0].translation, dxy_max, dz_max);	
		
		/// Debugging
		//~ if(!theta_.lineofsight(n1,n3) && areCloseEnough)
		//~ {
			//~ printf("Post-processing: wp %d are close enough but not has line of sight\n", index + 1);
		//~ }
		
		if(theta_.lineofsight(n1,n3) && areCloseEnough || areSoClose)
		{
			// Elimininate middle waypoint between n1 (index) and n3 (index + 2) --> n2 (index + 1)
			trajectory->points.erase(trajectory->points.begin() + index + 1);
			//~ printf("Post-processing: eliminating near wp %d\n", index + 1);
			
			// Update sizes
			traj_size--;
			traj_final_wp_index--;
			
			// Note: No increment index
			//~ printf("Post-processing: Reducing to %d\n", traj_size);
		}
		else
		{
			index++;
		}
	}
}

//! Aux Function: Trajectory time post-processing for 'yawCte version'. 
/// No yaw changes, so it simply re-computes the time-marks 
void yaw_cte_post_processing(TrajectoryPtr trajectory, Transform init_pose, double Vxy, double Vz, double Vxy_1, double Vz_1)
{
	//~ printf_trajectory(*trajectory, "Input yaw_cte_post_processing()");

	int traj_size = trajectory->points.size();
	double total_time = 0.0;
	double dt_h, dt_v, dt;		// trajectory times

	// Set first wp
	dt_h = get_horizontal_norm(trajectory->points[0].transforms[0].translation.x - 0.0, trajectory->points[1].transforms[0].translation.y - 0.0) / Vxy;//Vxy_1;
	dt_v = fabs(trajectory->points[1].transforms[0].translation.z - 0.0) / Vz;//Vz_1;
	dt = max(dt_h, dt_v);
	total_time+=dt;
	trajectory->points[0].time_from_start = ros::Duration(total_time);

	if(traj_size>2)
	{
		// From second to pre-last waypoint
		for(int i = 1; i<traj_size-1; i++)
		{				
			// Re-compute times
			dt_h = get_horizontal_norm(trajectory->points[i].transforms[0].translation.x - trajectory->points[i-1].transforms[0].translation.x, trajectory->points[i].transforms[0].translation.y - trajectory->points[i-1].transforms[0].translation.y) / Vxy;
			dt_v = fabs(trajectory->points[i].transforms[0].translation.z - trajectory->points[i-1].transforms[0].translation.z) / Vz;
			// Set the maximum neccesary elapse time
			dt = max(dt_h, dt_v);
			total_time+=dt;
			trajectory->points[i].time_from_start = ros::Duration(total_time);
		}

		// Set last wp time using reduced mean velocity
		dt_h = get_horizontal_norm(trajectory->points[traj_size-1].transforms[0].translation.x - trajectory->points[traj_size-2].transforms[0].translation.x, trajectory->points[traj_size-1].transforms[0].translation.y - trajectory->points[traj_size-2].transforms[0].translation.y) / Vxy_1;
		dt_v = fabs(trajectory->points[traj_size-1].transforms[0].translation.z - trajectory->points[traj_size-2].transforms[0].translation.z) / Vz_1;
		dt = max(dt_h, dt_v);
		total_time+=dt;
		trajectory->points[traj_size-1].time_from_start = ros::Duration(total_time);
	}
	
	//~ printf_trajectory(*trajectory, "Output yaw_cte_post_processing()");
}

//! Aux Function: Trajectory yaw and time post-processing for 'yawAtTime version'. 
/// It modifies the trajectory to not command yaw reference changes if horizontal
/// position increment is short between 2 waypoints ('are_too_close_horizontally' 
/// function) and re-computes the time-marks 
void yaw_at_time_post_processing(TrajectoryPtr trajectory, Transform init_pose, double dxy_max, double Vxy, double Vz, double Vxy_1, double Vz_1, double Wyaw)
{
	//~ printf_trajectory(*trajectory, "Input yaw_at_time_post_processing()");

	int traj_size = trajectory->points.size();
	double yaw_at_origin = 0.0;			// Yaw at previous wp
	double yaw = 0.0;					// Reference yaw
	double dyaw = 0.0;					// Reference yaw increment
	tf::Quaternion q;
	int closes_waypoints_index = 0;		// Horizontal close enough waypoints count
	int origin_index = 0;
	int goal_index = 1;
	double total_time = 0.0;
	double dt_h, dt_v, dt_y, dt;		// trajectory times
		
	/*** FIRST WP from init pose to second wp ***/
	// Get yaw from current pose to first wp
	yaw_at_origin = 0.0; // get_yaw_from_quat(init_pose.rotation); AT LOCAL FRAME IS ZERO!!!!!!!!!!!!!!!
	yaw = atan2(trajectory->points[0].transforms[0].translation.y - 0.0, 
				trajectory->points[0].transforms[0].translation.x - 0.0);			
	// update yaw increment and calculate necessary time to get this dyaw
	dyaw =  get_dyaw(yaw, yaw_at_origin);
	dt_y = dyaw / Wyaw;
	// set yaw reference
	q.setRPY(0.0,0.0,yaw);
	trajectory->points[origin_index].transforms[0].rotation.x = q.x();
	trajectory->points[origin_index].transforms[0].rotation.y = q.y();
	trajectory->points[origin_index].transforms[0].rotation.z = q.z();
	trajectory->points[origin_index].transforms[0].rotation.w = q.w();			
	// calculate time
	dt_h = get_horizontal_norm(trajectory->points[origin_index].transforms[0].translation.x - 0.0, trajectory->points[origin_index].transforms[0].translation.y - 0.0) / Vxy;
	dt_v = fabs(trajectory->points[origin_index].transforms[0].translation.z - 0.0) / Vz;
	dt = max(max(dt_h, dt_v), dt_y);
	total_time+=dt;
	trajectory->points[origin_index].time_from_start = ros::Duration(total_time);

	/*** From second to last waypoint ***/
	yaw_at_origin = yaw;
	for(int i = 2; i<traj_size; i++)
	{
		origin_index = i-1-closes_waypoints_index; 	// initially the second wp: [2 - 1 - 0] = [1]
		goal_index = i;								// initially the third wp: 	[2]
		// If the next waypoint is close enought respect the initial, increment the count of waypoint between origin to goal
		if(i!=(traj_size-1) && are_too_close_horizontally(trajectory->points[origin_index].transforms[0].translation, trajectory->points[goal_index].transforms[0].translation, dxy_max))
		{
			// one more middle waypoint
			closes_waypoints_index++;
			//~ printf("Post-processing: not yaw ahead for %d\n", i);
		}
		else
		{
			// Yaw from i to i-1 for all waypoints from origin to target
			yaw = atan2(trajectory->points[i].transforms[0].translation.y - trajectory->points[i-1].transforms[0].translation.y, trajectory->points[i].transforms[0].translation.x - trajectory->points[i-1].transforms[0].translation.x);			
			tfScalar yaw_ = yaw;
			q.setRPY(0.0,0.0,yaw_);
			
			// update yaw increment and calculate necessary time to get this dyaw
			dyaw = fabs(yaw - yaw_at_origin);
			if(dyaw>M_PI)
				dyaw = fabs(dyaw - 2.0 * M_PI);
			dt_y = dyaw / Wyaw / (goal_index - origin_index);
			yaw_at_origin = yaw;
			
			// Set this yaw to all waypoints from next_to_origin to target and recalculate times
			for(int j = origin_index + 1 ; j<=goal_index; j++)
			{
				trajectory->points[j].transforms[0].rotation.x = q.x();
				trajectory->points[j].transforms[0].rotation.y = q.y();
				trajectory->points[j].transforms[0].rotation.z = q.z();
				trajectory->points[j].transforms[0].rotation.w = q.w();
				
				// Re-compute times (for second and last waypoints uses the reduced mean velocity)
				if(j==1 || j==traj_size-1)
				{
					dt_h = get_horizontal_norm(trajectory->points[j].transforms[0].translation.x - trajectory->points[j-1].transforms[0].translation.x, trajectory->points[j].transforms[0].translation.y - trajectory->points[j-1].transforms[0].translation.y) / Vxy_1;
					dt_v = fabs(trajectory->points[j].transforms[0].translation.z - trajectory->points[j-1].transforms[0].translation.z) / Vz_1;
				}
				else
				{
					dt_h = get_horizontal_norm(trajectory->points[j].transforms[0].translation.x - trajectory->points[j-1].transforms[0].translation.x, trajectory->points[j].transforms[0].translation.y - trajectory->points[j-1].transforms[0].translation.y) / Vxy;
					dt_v = fabs(trajectory->points[j].transforms[0].translation.z - trajectory->points[j-1].transforms[0].translation.z) / Vz;
				}
				
				// Set the maximum neccesary elapse time
				dt = max(max(dt_h, dt_v), dt_y);
				total_time+=dt;
				trajectory->points[j].time_from_start = ros::Duration(total_time);
			}
			
			// Update vars
			closes_waypoints_index = 0;
		}
	}
	
	//~ printf_trajectory(*trajectory, "Output yaw_at_time_post_processing()");
}

//! Aux Function: Trajectory yaw and time post-processing for 'yawAInAdvance version'. 
/// It modifies the trajectory to not command yaw reference changes if horizontal
/// position increment is short between 2 waypoints ('are_too_close_horizontally' 
/// function) and re-computes the time-marks 
void yaw_in_advance_post_processing(TrajectoryPtr trajectory, Transform init_pose, double dxy_max, double Vxy, double Vz, double Vxy_1, double Vz_1, double Wyaw)
{
	//printf_trajectory(*trajectory, "Input yaw_in_advance_post_processing()");
	
	int traj_size = trajectory->points.size();
	double yaw_at_origin = 0.0;			// Yaw at previous wp
	double yaw = 0.0;					// Reference yaw
	double dyaw = 0.0;					// Reference yaw increment
	tf::Quaternion q;
	int closes_waypoints_index = 0;		// Horizontal close enough waypoints count
	int origin_index = 0;
	int goal_index = 1;
	double total_time = 0.0;
	double dt_h, dt_v, dt_y, dt;		// trajectory times
		
	/*** FIRST WP from init pose to second wp ***/
	// Get yaw from current pose to first wp
	yaw_at_origin = 0.0; // get_yaw_from_quat(init_pose.rotation); AT LOCAL FRAME IS ZERO!!!!!!!!!!!!!!!

	if(traj_size>1)
	{
		yaw = atan2(trajectory->points[goal_index].transforms[0].translation.y - trajectory->points[origin_index].transforms[0].translation.y, 
					trajectory->points[goal_index].transforms[0].translation.x - trajectory->points[origin_index].transforms[0].translation.x);			
	}
	else
	{
		yaw = atan2(trajectory->points.back().transforms[0].translation.y - 0.0, 
					trajectory->points.back().transforms[0].translation.x - 0.0);			
	}
	// update yaw increment and calculate necessary time to get this dyaw
	dyaw =  get_dyaw(yaw, yaw_at_origin);
	dt_y = dyaw / Wyaw;
	// set yaw reference
	q.setRPY(0.0,0.0,yaw);
	trajectory->points[origin_index].transforms[0].rotation.x = q.x();
	trajectory->points[origin_index].transforms[0].rotation.y = q.y();
	trajectory->points[origin_index].transforms[0].rotation.z = q.z();
	trajectory->points[origin_index].transforms[0].rotation.w = q.w();			
	// calculate time
	dt_h = get_horizontal_norm(trajectory->points[origin_index].transforms[0].translation.x - 0.0, trajectory->points[origin_index].transforms[0].translation.y - 0.0) / Vxy;
	dt_v = fabs(trajectory->points[origin_index].transforms[0].translation.z - 0.0) / Vz;
	dt = max(max(dt_h, dt_v), dt_y);
	total_time+=dt;
	trajectory->points[origin_index].time_from_start = ros::Duration(total_time);

	/*** From second to last waypoint ***/
	yaw_at_origin = yaw;
	for(int i = 2; i<traj_size; i++)
	{
		origin_index = i-1-closes_waypoints_index; 	// initially the second wp: [2 - 1 - 0] = [1]
		goal_index = i;								// initially the third wp: 	[2]
		// If the next waypoint is close enought respect the initial, increment the count of waypoint between origin to goal
		if(i!=(traj_size-1) && are_too_close_horizontally(trajectory->points[origin_index].transforms[0].translation, trajectory->points[goal_index].transforms[0].translation, dxy_max))
		{
			// one more middle waypoint
			closes_waypoints_index++;
			//~ printf("Post-processing: not yaw ahead for %d\n", i);
		}
		else
		{	
			// Get yaw from origin to goal
			yaw = atan2(trajectory->points[goal_index].transforms[0].translation.y - trajectory->points[origin_index].transforms[0].translation.y, 
						trajectory->points[goal_index].transforms[0].translation.x - trajectory->points[origin_index].transforms[0].translation.x);			
			
			q.setRPY(0.0,0.0,yaw);
			
			// update yaw increment and calculate necessary time to get this dyaw
			dyaw =  get_dyaw(yaw, yaw_at_origin);
			dt_y = dyaw / Wyaw;
						
			// Set this yaw for waypoint at origin and Re-compute times taking it into account to get the elapse time from the wp[origin_index-1] to wp[origin_index]
			trajectory->points[origin_index].transforms[0].rotation.x = q.x();
			trajectory->points[origin_index].transforms[0].rotation.y = q.y();
			trajectory->points[origin_index].transforms[0].rotation.z = q.z();
			trajectory->points[origin_index].transforms[0].rotation.w = q.w();			
			dt_h = get_horizontal_norm(trajectory->points[origin_index].transforms[0].translation.x - trajectory->points[origin_index-1].transforms[0].translation.x, trajectory->points[origin_index].transforms[0].translation.y - trajectory->points[origin_index-1].transforms[0].translation.y) / Vxy;
			dt_v = fabs(trajectory->points[origin_index].transforms[0].translation.z - trajectory->points[origin_index-1].transforms[0].translation.z) / Vz;
			dt = max(max(dt_h, dt_v), dt_y);
			total_time+=dt;
			trajectory->points[origin_index].time_from_start = ros::Duration(total_time);
			
			// Set this yaw to all waypoints from next_to_origin to target and recalculate times (not taking it into account)
			// Note that if closes_waypoints_index = 0 --> original_index = i - 1, so j is from i to i-1 and it does not execute nothing 
			for(int j = origin_index + 1 ; j < goal_index; j++)
			{
				trajectory->points[j].transforms[0].rotation.x = q.x();
				trajectory->points[j].transforms[0].rotation.y = q.y();
				trajectory->points[j].transforms[0].rotation.z = q.z();
				trajectory->points[j].transforms[0].rotation.w = q.w();
				
				// Re-compute times
				dt_h = get_horizontal_norm(trajectory->points[j].transforms[0].translation.x - trajectory->points[j-1].transforms[0].translation.x, trajectory->points[j].transforms[0].translation.y - trajectory->points[j-1].transforms[0].translation.y) / Vxy;
				dt_v = fabs(trajectory->points[j].transforms[0].translation.z - trajectory->points[j-1].transforms[0].translation.z) / Vz;
				// Set the maximum neccesary elapse time
				dt = max(dt_h, dt_v);
				total_time+=dt;
				trajectory->points[j].time_from_start = ros::Duration(total_time);
			}
			
			// Update vars
			yaw_at_origin = yaw;
			closes_waypoints_index = 0;
		}
	}
		
	if(traj_size>1)
	{		
		// last waypoint is not changed, exception the time field that must be updated
		std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::iterator iter_to_last_wp = trajectory->points.end()-1;
		
		dt_h = get_horizontal_norm(iter_to_last_wp->transforms[0].translation.x - (iter_to_last_wp-1)->transforms[0].translation.x, iter_to_last_wp->transforms[0].translation.y - (iter_to_last_wp-1)->transforms[0].translation.y) / Vxy_1;
		dt_v = fabs(iter_to_last_wp->transforms[0].translation.z - (iter_to_last_wp-1)->transforms[0].translation.z) / Vz_1;
		yaw = atan2(iter_to_last_wp->transforms[0].translation.y - (iter_to_last_wp-1)->transforms[0].translation.y, 
					iter_to_last_wp->transforms[0].translation.x - (iter_to_last_wp-1)->transforms[0].translation.x);
		dyaw =  get_dyaw(yaw, yaw_at_origin);
		dt_y = dyaw / Wyaw;
		dt = max(dt_h, dt_v);
		dt = max(max(dt_h, dt_v), dt_y);
		total_time+=dt;
		trajectory->points.back().time_from_start = ros::Duration(total_time);
		//printf("Last wp calculation: \nfrom [%f, %f, %f] to [%f, %f, %f]: dt_h: %f, dt_v: %f\n", (iter_to_last_wp-1)->transforms[0].translation.x, (iter_to_last_wp-1)->transforms[0].translation.y, (iter_to_last_wp-1)->transforms[0].translation.z, (iter_to_last_wp)->transforms[0].translation.x, (iter_to_last_wp)->transforms[0].translation.y, (iter_to_last_wp)->transforms[0].translation.z, dt_h, dt_v);	
		//printf("from %f to %f: dt_y: %f\n", yaw_at_origin, yaw, dt_y);
		//printf("Final dt: %f --> %f\n", dt, total_time);
	}
	
	//printf_trajectory(*trajectory, "Output yaw_in_advance_post_processing()");
}

//! Aux Function: Complete post-proccessing = near enought wp elimination + time marks re-computation (YAW CTE VERSION)
void post_process_trajectory_yaw_cte (TrajectoryPtr trajectory, Transform init_pose, ThetaStar &theta_, double dxy_max, double dz_max, double Vxy, double Vz, double Vxy_1, double Vz_1)
{
	first_wps_post_processing(trajectory, dxy_max, dz_max);
	near_visible_wp_post_processing	(trajectory, theta_, dxy_max, dz_max);
	yaw_cte_post_processing			(trajectory, init_pose, Vxy, Vz, Vxy_1, Vz_1);
}

//! Aux Function: Complete post-proccessing = near enought wp elimination + near enought wp yaw changes at time + time marks re-computation (YAW AT TIME VERSION)
void post_process_trajectory_yaw_at_time (TrajectoryPtr trajectory, Transform init_pose, ThetaStar &theta_, double dxy_max, double dz_max, double Vxy, double Vz, double Vxy_1, double Vz_1, double Wyaw)
{
	first_wps_post_processing(trajectory, dxy_max, dz_max);
	near_visible_wp_post_processing	(trajectory, theta_, dxy_max, dz_max);
	yaw_at_time_post_processing		(trajectory, init_pose, dxy_max, Vxy, Vz, Vxy_1, Vz_1, Wyaw);
}

//! Aux Function: Complete post-proccessing = near enought wp elimination + near enought wp yaw changes in advance + time marks re-computation (YAW IN ADVANCE VERSION)
void post_process_trajectory_yaw_in_advance (TrajectoryPtr trajectory, Transform init_pose, ThetaStar &theta_, double dxy_max, double dz_max, double Vxy, double Vz, double Vxy_1, double Vz_1, double Wyaw)
{
	first_wps_post_processing(trajectory, dxy_max, dz_max);
	near_visible_wp_post_processing	(trajectory, theta_, dxy_max, dz_max);
	yaw_in_advance_post_processing	(trajectory, init_pose, dxy_max, Vxy, Vz, Vxy_1, Vz_1, Wyaw);
}
