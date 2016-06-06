/*
 * Copyright 2015 Ricardo Ragel de la Torre, GRVC, Univ. of Seville, Spain
 *
 * Resume: Lazy Theta Star with Optimization Class definitions
 * 
 */

#include <theta_star/ThetaStar.h>
#include <tf/transform_datatypes.h>

// Uncomment to get the explored and non-LineOfSight visual markers
#define DEBUG 

#define SIGHT_AHEAD_MIN_HORIZ_MOVEMENT 0.20	//[m] minimun horizontal position increment to set a sight ahead yaw

namespace PathPlanners {


// Print step by step the nodes explored
#define STEP_BY_STEP false


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

//! Aux Function: Tajectory point to fill and send to the trajectory tracker sight ahead -- Yaw is calculated in such a way that cameras always see ahead
void set_position_yaw_and_time(trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_point, Vector3 position, double _yaw, double T)
{
  tfScalar roll = 0.0;
  tfScalar pitch = 0.0;
  tfScalar yaw = _yaw;
  tf::Quaternion q;
  q.setRPY(roll,pitch,yaw);
  //~ printf("Commanded Yaw: %f --> Quaternion: [%f, %f, %f, %f]\n", yaw, q.x(), q.y(), q.z(), q.w());
  
  trajectory_point.transforms[0].translation.x = position.x;
  trajectory_point.transforms[0].translation.y = position.y;
  trajectory_point.transforms[0].translation.z = position.z;
  trajectory_point.transforms[0].rotation.x = q.x();
  trajectory_point.transforms[0].rotation.y = q.y();
  trajectory_point.transforms[0].rotation.z = q.z();
  trajectory_point.transforms[0].rotation.w = q.w();
  trajectory_point.velocities[0].linear.x = 0.0;
  trajectory_point.velocities[0].linear.y = 0.0;
  trajectory_point.velocities[0].linear.z = 0.0;
  trajectory_point.accelerations[0].linear.x = 0.0;
  trajectory_point.accelerations[0].linear.y = 0.0;
  trajectory_point.accelerations[0].linear.z = 0.0;
  trajectory_point.time_from_start = ros::Duration(T);
}

//! Aux Function: return the horizonatl module of [X,Y]
double get_horizontal_norm(double x, double y)
{
	return sqrtf(x * x + y * y);
}

//! Aux Function: return the horizontal distance from Point1 to Point2
double get_horizontal_distance(trajectory_msgs::MultiDOFJointTrajectoryPoint P1, trajectory_msgs::MultiDOFJointTrajectoryPoint P2)
{
	return get_horizontal_norm(P1.transforms[0].translation.x - P2.transforms[0].translation.x, P1.transforms[0].translation.y - P2.transforms[0].translation.y);
}

//! Aux Function: Get yaw in radians from a quaternion
float get_yaw_from_quat(geometry_msgs::Quaternion quat)
{
	double r, p, y;
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 M(q);
	M.getRPY(r, p, y);
	
	return y;
}

//! Aux Function: Get differential from last_yaw to next_yaw in [-PI, PI], but in absolute value --> [0, PI] 
double get_dyaw(double next_yaw, double last_yaw)
{
	double dyaw = fabs(next_yaw - last_yaw);
		if(dyaw>M_PI)
			dyaw = fabs(dyaw - 2.0 * M_PI);
	
	return dyaw;
}

//! Aux Function: simply print the trajectory msg data
void printf_trajectory(trajectory_msgs::MultiDOFJointTrajectory trajectory, string trajectory_name)
{	
	printf(PRINTF_YELLOW "%s trajectory [%d]:\n", trajectory_name.c_str(), trajectory.points.size());
	
	for(unsigned int i=0; i < trajectory.points.size();i++)
	{
		double yaw = get_yaw_from_quat(trajectory.points[i].transforms[0].rotation);
		printf(PRINTF_BLUE "\t %d: [%f, %f, %f] m\t[%f] rad\t [%f] sec\n", i, trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.y, trajectory.points[i].transforms[0].translation.z, yaw , trajectory.points[i].time_from_start.toSec());
	}

	printf(PRINTF_REGULAR);
}

//! Aux Function: Get the transformation of a point (geometry_msgs::Transform) 
/// for a specified tf:Transform
void getTransformedPoint(Transform point_atWorld, tf::StampedTransform world_to_sensor, Transform &point_atSensor)
{
	// Transform point at world to TF transformation point->world
	tf::StampedTransform point_to_world;
	tf::transformMsgToTF(point_atWorld, point_to_world);
	// Get the transformation point->sensor = world->sensor * point->world
	tf::transformTFToMsg(world_to_sensor*point_to_world, point_atSensor);
}

//! Aux Function: Get a global point in the local frame using a transformation
inline void global_to_local(float &x_l, float &y_l, float &z_l, float &x_g, float &y_g, float &z_g, tf::Transform &transf_tf)
{
	// point at world
	tf::Transform global(tf::Quaternion(0, 0, 0, 0), tf::Vector3(x_g, y_g, z_g));
	
	// point at local
	// W->L = G->L * W ->G
	tf::Transform local =  transf_tf.inverse() * global;
	x_l = local.getOrigin().x();
	y_l = local.getOrigin().y();
	z_l = local.getOrigin().z();
}

//! Aux Function: Get a global point in the local frame using a translation and pre-computes cos(yaw) and sin(yaw). Continuous Version
inline void global_to_local(float &x_l, float &y_l, float &z_l, float &x_g, float &y_g, float &z_g, tf::Vector3 &translation, float &cos_yaw, float &sin_yaw)
{
	x_l =   (x_g - translation.x()) * cos_yaw + (y_g - translation.y()) * sin_yaw;
	y_l = - (x_g - translation.x()) * sin_yaw + (y_g - translation.y()) * cos_yaw;
	z_l = 	 z_g - translation.z();
}



/*********************************************************************************
 * 
 * ThetaStar Class Definitions
 * 
 * *******************************************************************************/

// Constructor: creates the discrete node matrix (occupancy) from the bounding box sizes and resolution arguments
ThetaStar::ThetaStar(char* plannerName, char* planner_frame_id, float x_max, float y_max, float z_max, float x_min, float y_min, float z_min, float step_, float h_real_inflation_, float v_real_inflation_, float h_safe_inflation_, float v_safe_inflation_, float initial_point_factor_, float z_weight_cost_, ros::NodeHandle *n):
    n_debug(n),
    disc_initial(NULL),
    disc_final(NULL),
    max_point_life(0)
{
	timeout = 100; // by default 100 seconds (infinite)
	
    /* MODIFICADO PARA PODER USAR UN WORKSPACE ASIMETRICO DESDE EL ORIGEN (en vez de un solo x_limit --> poder usar un x_max, x_min) */
    ws_x_max = ((x_max/step_) + 1);
    ws_y_max = ((y_max/step_) + 1);
    ws_z_max = ((z_max/step_) + 1);
    ws_x_min = ((x_min/step_) - 1);
    ws_y_min = ((y_min/step_) - 1);
    ws_z_min = ((z_min/step_) - 1);
    step = step_;
    step_inv = 1.0/step_;
	h_real_inflation = h_real_inflation_;
	h_safe_inflation = h_safe_inflation_;
	v_real_inflation = v_real_inflation_;
	v_safe_inflation = v_safe_inflation_;
	h_inflation = h_real_inflation_ + h_safe_inflation_;
	v_inflation = v_real_inflation_ + v_safe_inflation_;
	h_inflation_ = (int) (h_inflation/step_);
	v_inflation_ = (int) (v_inflation/step_);
	initial_point_factor = initial_point_factor_;
	z_weight_cost = z_weight_cost_;
	
	// Matrix axis length and size
	/// The discrete matrix size will be bigger than the workspace (+ inflation_map_node_) for it won't be neccesary a "inside" checking during the inflaction
	ws_x_max_inflate = (ws_x_max + 2.0 * h_inflation_);
    ws_y_max_inflate = (ws_y_max + 2.0 * h_inflation_);
    ws_z_max_inflate = (ws_z_max + 2.0 * v_inflation_);
    ws_x_min_inflate = (ws_x_min - 2.0 * h_inflation_);
    ws_y_min_inflate = (ws_y_min - 2.0 * h_inflation_);
    ws_z_min_inflate = (ws_z_min - 2.0 * v_inflation_);
	matrix_size = (abs(ws_x_max_inflate) - ws_x_min_inflate + 1)*(abs(ws_y_max_inflate) - ws_y_min_inflate + 1)*(abs(ws_z_max_inflate) - ws_z_min_inflate + 1);
	Lx = ws_x_max_inflate - ws_x_min_inflate + 1;
    Ly = ws_y_max_inflate - ws_y_min_inflate + 1;
    Lz = ws_z_max_inflate - ws_z_min_inflate + 1;
    Lx_inv = 1.0/Lx;
    Ly_inv = 1.0/Ly;
    Lz_inv = 1.0/Lz;

    cerr << "Matrix Size: " << matrix_size << " nodes." << endl;
    cerr << "It needs : " << (matrix_size * sizeof(ThetaStarNode))/(1024*1024) << " MB" << endl;

    discrete_world.resize(matrix_size);

    cerr << "ThetaStar: Memory has been initialized."<< endl;

	// Visualitazion Markers
	char topicPath[100];
	sprintf(topicPath, "%s/vis_marker_explored", plannerName);
    marker_pub_ = n_debug->advertise<visualization_msgs::Marker>( topicPath, 1 );
    sprintf(topicPath, "%s/vis_marker_ocuppancy", plannerName);
    occupancy_marker_pub_ = n_debug->advertise<visualization_msgs::Marker>( topicPath, 1 );
    sprintf(topicPath, "%s/vis_marker_ocuppancy_aux", plannerName);
    aux_occupancy_marker_pub_ = n_debug->advertise<visualization_msgs::Marker>( topicPath, 1 );
    sprintf(topicPath, "%s/vis_marker_no_lineOfSight", plannerName);
	no_los_marker_pub_ = n_debug->advertise<visualization_msgs::Marker>( topicPath, 1 );

    marker.header.frame_id = planner_frame_id; //"world";
    marker.header.stamp = ros::Time();
    marker.ns = "debug";
    marker.id = 66;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0*step;
    marker.scale.y = 1.0*step;
    marker.scale.z = 1.0*step;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    
    occupancy_marker.header.frame_id = planner_frame_id; // "world";
    occupancy_marker.header.stamp = ros::Time();
    occupancy_marker.ns = "debug";
    occupancy_marker.id = 66;
    occupancy_marker.type = visualization_msgs::Marker::CUBE_LIST;
    occupancy_marker.action = visualization_msgs::Marker::ADD;
    occupancy_marker.pose.orientation.w = 1.0;
    occupancy_marker.scale.x = 1.0*step;
    occupancy_marker.scale.y = 1.0*step;
    occupancy_marker.scale.z = 1.0*step;
    occupancy_marker.color.a = 1.0;
    occupancy_marker.color.r = 0.5;
    occupancy_marker.color.g = 0.0;
    occupancy_marker.color.b = 0.5;

    aux_occupancy_marker.header.frame_id = "world";
    aux_occupancy_marker.header.stamp = ros::Time();
    aux_occupancy_marker.ns = "debug";
    aux_occupancy_marker.id = 66;
    aux_occupancy_marker.type = visualization_msgs::Marker::CUBE_LIST;
    aux_occupancy_marker.action = visualization_msgs::Marker::ADD;
    aux_occupancy_marker.pose.orientation.w = 1.0;
    aux_occupancy_marker.scale.x = 1.0*step;
    aux_occupancy_marker.scale.y = 1.0*step;
    aux_occupancy_marker.scale.z = 1.0*step;
    aux_occupancy_marker.color.a = 1.0;
    aux_occupancy_marker.color.r = 0.5;
    aux_occupancy_marker.color.g = 0.0;
    aux_occupancy_marker.color.b = 0.5;
    
    marker_no_los.header.frame_id = planner_frame_id; //"world";
    marker_no_los.header.stamp = ros::Time();
    marker_no_los.ns = "debug";
    marker_no_los.id = 66;
    marker_no_los.type = visualization_msgs::Marker::CUBE_LIST;
    marker_no_los.action = visualization_msgs::Marker::ADD;
    marker_no_los.pose.orientation.w = 1.0;
    marker_no_los.scale.x = 1.0*step;
    marker_no_los.scale.y = 1.0*step;
    marker_no_los.scale.z = 1.0*step;
    marker_no_los.color.a = 1.0;
    marker_no_los.color.r = 1.0;
    marker_no_los.color.g = 1.0;
    marker_no_los.color.b = 0.0;
}

ThetaStar::~ThetaStar() {
}

// Create the auxiliar discrete world matrix: 
// arg: global workspace (same that used for the global_planner it's ok)
// arg: Points life (cycles that a point is kept in the auxiliar world discrete matrix from the last time it was seen) 
void ThetaStar::initAuxDiscreteMatrix(float x_max, float y_max, float z_max, float x_min, float y_min, float z_min, unsigned char max_point_life_)
{
    // Auxiliar matrix initialization    
    ws_x_max_aux = ((x_max/step) + 1);
    ws_y_max_aux = ((y_max/step) + 1);
    ws_z_max_aux = ((z_max/step) + 1);
    ws_x_min_aux = ((x_min/step) - 1);
    ws_y_min_aux = ((y_min/step) - 1);
    ws_z_min_aux = ((z_min/step) - 1);
    ws_x_max_aux_inflate = (ws_x_max_aux + 2.0 * h_inflation_);
    ws_y_max_aux_inflate = (ws_y_max_aux + 2.0 * h_inflation_);
    ws_z_max_aux_inflate = (ws_z_max_aux + 2.0 * v_inflation_);
    ws_x_min_aux_inflate = (ws_x_min_aux - 2.0 * h_inflation_);
    ws_y_min_aux_inflate = (ws_y_min_aux - 2.0 * h_inflation_);
    ws_z_min_aux_inflate = (ws_z_min_aux - 2.0 * v_inflation_);
	aux_matrix_size = (abs(ws_x_max_aux_inflate) - ws_x_min_aux_inflate + 1)*(abs(ws_y_max_aux_inflate) - ws_y_min_aux_inflate + 1)*(abs(ws_z_max_aux_inflate) - ws_z_min_aux_inflate + 1);
	Lx_aux = ws_x_max_aux_inflate - ws_x_min_aux_inflate + 1;
    Ly_aux = ws_y_max_aux_inflate - ws_y_min_aux_inflate + 1;
    Lz_aux = ws_z_max_aux_inflate - ws_z_min_aux_inflate + 1;
    Lx_inv_aux = 1.0/Lx_aux;
    Ly_inv_aux = 1.0/Ly_aux;
    Lz_inv_aux = 1.0/Lz_aux;
    
    cerr << "Auxiliar Matrix Size: " << aux_matrix_size << " nodes." << endl;
    cerr << "It needs : " << (aux_matrix_size * sizeof(ThetaStarNode))/(1024*1024) << " MB" << endl;
    aux_discrete_world.resize(aux_matrix_size);
    cerr << "Auxiliar matrix memory has been initialized."<< endl;
    
    // Cloud points time-out
    max_point_life = max_point_life_;
}

// Update the discrete node matrix (occupancy) from octomap_server with the XYZ offset relative to the required reference frame
void ThetaStar::updateMap(octomap_msgs::Octomap message, float x_offset, float y_offset, float z_offset)
{
    // READ occupation data from the octomap_server
    m = octomap_msgs::binaryMsgToMap(message);

    /*
     * Update discrete world with the read octomap data
     */

	// Discrete matrix index
	// unsigned int world_index_;
	
	// Continuous position in world frame
	// float x_w, y_w, z_w;
	//and discretized
	// int x_,y_,z_; 

	// Occupied and not occupied points, only for debug
    u_int64_t occupied_leafs=0, free_leafs=0;

	// Read from first leaf of the tree to the last and set for its xyz (prev discretized) if it is occupied into the discretes node matrix
    if(m->begin_leafs()!=NULL)
    {
        for(octomap::OcTree::leaf_iterator it = m->begin_leafs(), end=m->end_leafs();   it!= end;    ++it)
        {
            if(m->isNodeOccupied(*it))
            {
				// Get occupied cells in /map and transform to /world
				float x_w = it.getX() + x_offset;
				float y_w = it.getY() + y_offset;
				float z_w = it.getZ() + z_offset;

				// Exact discretization
                int x_ = (int)(x_w*step_inv);
                int y_ = (int)(y_w*step_inv);
                int z_ = (int)(z_w*step_inv);

				// Set as occupied in the discrete matrix
                if(isInside(x_, y_, z_))
                {
                    unsigned int world_index_ = getWorldIndex(x_, y_, z_);
                    discrete_world[world_index_].notOccupied = false;
                    //~ discrete_world[world_index_].time = 0;
					
					// Inflates nodes -- Empty Cube
					if(h_inflation_ >= step || v_inflation_ >= step)
					{
						fastInflateNodeAsSolidCube(x_, y_, z_);
					}
                }
                
				#ifdef DEBUG				
                occupied_leafs++;//debug
				#endif          
            }
            else
            {
				#ifdef DEBUG				
				free_leafs++;
				#endif				
			}
        }
    }
    
	#ifdef DEBUG    
		std::cout << "Occupied cells: " << occupied_leafs  << " NO occupied cells: " << free_leafs << std::endl;
	#endif
}

// Update the discrete node matrix (occupancy) from a point cloud 
void ThetaStar::updateMap(PointCloud cloud)
{	
    /*
     * Update discrete world with the Point Cloud = ocuppieds cells
     */
	// unsigned int world_index_;	// discrete matrix index for occupied cells
	// float x_w, y_w, z_w;			// continuous occupied points
	// int x_,y_,z_; 				// discretized
	
	for(int it = 0; it<cloud.points.size(); it++)
	{
			// Get occupied points
			const pcl::PointXYZ &p = cloud.points[it];
			float x_w = p.x;
			float y_w = p.y;
			float z_w = p.z;

			// Exact discretization
			int x_ = (int)(x_w*step_inv);
			int y_ = (int)(y_w*step_inv);
			int z_ = (int)(z_w*step_inv);

			if(isInside(x_, y_, z_))
			{
				unsigned int world_index_ = getWorldIndex(x_, y_, z_);
				discrete_world[world_index_].notOccupied = false;
				//~ discrete_world[world_index_].time = 0;
				
				// Inflates nodes -- Empty Cube
				if(h_inflation_ >= step || v_inflation_ >= step)
				{
					fastInflateNodeAsSolidCube(x_, y_, z_);
				}
			}
	}
}

// Update the discrete node matrix (occupancy) from a point cloud that is not in the theta_star reference system (for example PC at global and theta_star at local)
// and we need to buffer this point cloud during a time (Time_of_life)
void ThetaStar::updateMap(PointCloud cloud, tf::Transform &transf)
{	
    /*
     * Update auxiliar discrete world matrix with the Point Cloud = ocuppieds cells + Time_of_life
     */

	// float xw, yw, zw;	// continuous point at world frame
    // int xw_,yw_,zw_;	// discrete point at world frame
    // float xl, yl, zl;	// continuous point at local frame
    // int xl_, yl_, zl_;	// discrete point at local frame
    // unsigned int world_index_; // discrete matrix index
    		
	for(int it = 0; it<cloud.points.size(); it++)
	{
		// Get occupied points
		const pcl::PointXYZ &p = cloud.points[it];
		float xw = p.x;
		float yw = p.y;
		float zw = p.z;

		// Exact discretization
		int xw_ = (int)(xw * step_inv);
		int yw_ = (int)(yw * step_inv);
		int zw_ = (int)(zw * step_inv);
					
		if(isInsideAux(xw_, yw_, zw_))
		{
			unsigned int world_index_ = getAuxWorldIndex(xw_, yw_, zw_);
			
			if(aux_discrete_world[world_index_].lastTimeSeen != 0)
			{
				aux_discrete_world[world_index_].lastTimeSeen = 0;
				// Inflates nodes -- Empty Cube
				if(h_inflation_ >= step || v_inflation_ >= step)
				{
					fastInflateAuxNodeAsSolidCube(xw_, yw_, zw_);
				}
			}
		}
	}
	
    /*
     * Update discrete world matrix with the auxiliar discrete matrix and the transformation received as argument
     */

	// Pre-calculate yaw, cos(yaw) and sin(yaw)
	geometry_msgs::Quaternion quat;
	tf::quaternionTFToMsg(transf.getRotation(), quat);
	float yaw = tf::getYaw(quat);
	float cos_yaw = cos(yaw);
	float sin_yaw = sin(yaw);
 	
	// loop through the entire auxiliar discrete matrix  
    for(int zw_=ws_z_min_aux; zw_<=ws_z_max_aux; zw_++)
		for(int yw_=ws_y_min_aux; yw_<=ws_y_max_aux; yw_++)
			for(int xw_=ws_x_min_aux; xw_<=ws_x_max_aux; xw_++)
			{
				// get auxiliar discrete matrix index for this position
				unsigned int world_index_ = getAuxWorldIndex(xw_, yw_, zw_);
				
				// if it's a occupied cell
				if(aux_discrete_world[world_index_].lastTimeSeen<=max_point_life)
				{
					// decrement time_of_life of occupied cell
					aux_discrete_world[world_index_].lastTimeSeen++;
					
					// convert point to continous
					float xw = xw_ * step;
					float yw = yw_ * step;
					float zw = zw_ * step;
					
					// transform point to local
					float xl, yl, zl;
					global_to_local(xl, yl, zl, xw, yw, zw, transf.getOrigin(), cos_yaw, sin_yaw);		// only using the yaw
					//~ global_to_local(xl, yl, zl, xw, yw, zw, transf);								// using the complete transform
					
					// convert local point to discrete
					int xl_ = (int)(xl*step_inv);
					int yl_ = (int)(yl*step_inv);
					int zl_ = (int)(zl*step_inv);
					
					// update local matrix if isInside local discrete_world, getting the world index and setting the point as occupied
					if(isInside(xl_, yl_, zl_))
					{
						world_index_ = getWorldIndex(xl_, yl_, zl_);
						discrete_world[world_index_].notOccupied = false;
						
						// Fill holes (transformation maybe creates unitary holes along Z axis between occupied nodes)
						fillAroundHoles(xl_, yl_, zl_);
						
					}
				}
			}
}

// Clear the map
void ThetaStar::clearMap()
{
	for(int i = 0; i< matrix_size; i++)
	{
		//~ if(!discrete_world[i].notOccupied)
		//~ {
			//~ discrete_world[i].notOccupied = true;
		//~ }

		// more efficient set all to true (1 operation for all nodes) than check and set (1 operation for all nodes + another for some of them)
		discrete_world[i].notOccupied = true;
	}
}

// Inflates nodes -- Solid Cube
void ThetaStar::inflateNodeAsSolidCube(int &x_, int &y_, int &z_)
{
	// discrete matrix index
	// unsigned int world_index_;
	// Inflation limits
	int x_inflated_max = (x_ + h_inflation_) + 1;
	int x_inflated_min = (x_ - h_inflation_) - 1;
	int y_inflated_max = (y_ + h_inflation_) + 1;
	int y_inflated_min = (y_ - h_inflation_) - 1;
	int z_inflated_max = (z_ + v_inflation_) + 1;
	int z_inflated_min = (z_ - v_inflation_) - 1;

	/// Due to the discrete matrix size increment, the inside checking is not neccesary
	for(int i = x_inflated_min; i <= x_inflated_max; i++)
		for(int j = y_inflated_min; j <= y_inflated_max; j++)
			for(int k = z_inflated_min; k <= z_inflated_max; k++)
			{
				unsigned int world_index_ = getWorldIndex(i, j, k);
				discrete_world[world_index_].notOccupied = false;
				//~ discrete_world[world_index_].time = 0;
			}
}

// Inflates nodes -- Solid Cube -- Fast version
void ThetaStar::fastInflateNodeAsSolidCube(int &x_, int &y_, int &z_)
{
	// Inflated limit nodes
	int x_inflated_max = (x_ + h_inflation_) + 1;
	int x_inflated_min = (x_ - h_inflation_) - 1;
	int y_inflated_max = (y_ + h_inflation_) + 1;
	int y_inflated_min = (y_ - h_inflation_) - 1;
	int z_inflated_max = (z_ + v_inflation_) + 1;
	int z_inflated_min = (z_ - v_inflation_) - 1;

	// discrete matrix index for the x axis inflated lower limit node 
	//~ unsigned int world_index_x_inflated_min;
	
	// loop 'x axis' by 'x axis' for all discrete occupancy matrix cube around the node that must be inflated
	for(int j = y_inflated_min; j <= y_inflated_max; j++)
		for(int k = z_inflated_min; k <= z_inflated_max; k++)
		{
			unsigned int world_index_x_inflated_min = getWorldIndex(x_inflated_min, j, k);
			memset(&discrete_world[world_index_x_inflated_min], 0, (x_inflated_max - x_inflated_min)*sizeof(ThetaStartNodeLink)); 
		}
}

// Fill Occupancy Matrix holes by the XY unitary inflation
void ThetaStar::fillAroundHoles(int &x_, int &y_, int &z_)
{
	// Inflated limit nodes
	int x_inflated_max = x_ + 1;
	int x_inflated_min = x_ - 1;
	int y_inflated_max = y_ + 1;
	int y_inflated_min = y_ - 1;

	// discrete matrix index for the x axis inflated lower limit node 
	//~ unsigned int world_index_x_inflated_min;
	
	// loop 'x axis' by 'x axis' for all discrete occupancy matrix cube around the node that must be inflated
	for(int j = y_inflated_min; j <= y_inflated_max; j++)
	{
		unsigned int world_index_x_inflated_min = getWorldIndex(x_inflated_min, j, z_);
		memset(&discrete_world[world_index_x_inflated_min], 0, (x_inflated_max - x_inflated_min)*sizeof(ThetaStartNodeLink)); 
	}
}

// Inflates nodes -- Solid Cube -- Fast version -- Auxiliar Occupancy Matrix
void ThetaStar::fastInflateAuxNodeAsSolidCube(int &x_, int &y_, int &z_)
{
	// Inflated limit nodes
	int x_inflated_max = (x_ + h_inflation_) + 1;
	int x_inflated_min = (x_ - h_inflation_) - 1;
	int y_inflated_max = (y_ + h_inflation_) + 1;
	int y_inflated_min = (y_ - h_inflation_) - 1;
	int z_inflated_max = (z_ + v_inflation_) + 1;
	int z_inflated_min = (z_ - v_inflation_) - 1;

	// discrete matrix index for the x axis inflated lower limit node 
	//~ unsigned int world_index_x_inflated_min;
	
	// loop 'x axis' by 'x axis' for all discrete occupancy matrix cube around the node that must be inflated
	for(int j = y_inflated_min; j <= y_inflated_max; j++)
		for(int k = z_inflated_min; k <= z_inflated_max; k++)
		{
			unsigned int world_index_x_inflated_min = getAuxWorldIndex(x_inflated_min, j, k);
			memset(&aux_discrete_world[world_index_x_inflated_min], 0, (x_inflated_max - x_inflated_min)*sizeof(ThetaStartNodeLink)); 
		}
}

// Puplish to RViz the occupancy map matrix as markers
void ThetaStar::publishOccupationMarkersMap()
{
	occupancy_marker.points.clear();
	for(int i=ws_x_min_inflate;i<=ws_x_max_inflate; i++)
		for(int j=ws_y_min_inflate;j<= ws_y_max_inflate; j++)
			for(int k=ws_z_min_inflate ;k<= ws_z_max_inflate; k++)
			{
				unsigned int matrixIndex = getWorldIndex(i,j,k);
				
				if(!discrete_world[matrixIndex].notOccupied)
				{
					geometry_msgs::Point point;
					point.x = i*step;
					point.y = j*step;
					point.z = k*step;
					occupancy_marker.points.push_back(point);
				}
			}
				
    occupancy_marker_pub_.publish( occupancy_marker );
}

// Puplish to RViz the AUXILIAR occupancy map matrix as markers
void ThetaStar::publishAuxOccupationMarkersMap()
{
	aux_occupancy_marker.points.clear();
	for(int i=ws_x_min_aux;i<=ws_x_max_aux; i++)
		for(int j=ws_y_min_aux;j<= ws_y_max_aux; j++)
			for(int k=ws_z_min_aux ;k<= ws_z_max_aux; k++)
			{
				unsigned int matrixIndex = getAuxWorldIndex(i,j,k);

				if(aux_discrete_world[matrixIndex].lastTimeSeen<=max_point_life)
				{
					geometry_msgs::Point point;
					point.x = i*step;
					point.y = j*step;
					point.z = k*step;
					aux_occupancy_marker.points.push_back(point);
				}
			}
				
    aux_occupancy_marker_pub_.publish( aux_occupancy_marker );
}

void ThetaStar::publishRvizPoint(ThetaStarNode &s, bool publish)
{
    geometry_msgs::Point point;
    point.x = s.point.x * step;
    point.y = s.point.y * step;
    point.z = s.point.z * step;

    marker.points.push_back(point);

    if(STEP_BY_STEP)
			publish = true;
    
    if(publish)
    {
        marker.header.stamp = ros::Time();
        marker.header.seq++;
        marker_pub_.publish( marker );
    }
        
    if(STEP_BY_STEP)
			usleep(1e4);
}

octomap::OcTree *ThetaStar::getMap()
{
    return m;
}

bool ThetaStar::setDiscreteInitialPosition(DiscretePosition p_)
{
    if(isInside(p_.x,p_.y,p_.z))
    {
        ThetaStartNodeLink *initialNodeInWorld = &discrete_world[getWorldIndex(
                    p_.x,
                    p_.y,
                    p_.z)];

        if(initialNodeInWorld->node==NULL)
        {
            initialNodeInWorld->node = new ThetaStarNode();
            initialNodeInWorld->node->point.x = p_.x;
            initialNodeInWorld->node->point.x = p_.y;
            initialNodeInWorld->node->point.x = p_.z;

            initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
        }
        disc_initial = initialNodeInWorld->node;

        initial_position.x = p_.x * step;
        initial_position.y = p_.y * step;
        initial_position.z = p_.z * step;
        disc_initial->point = p_;
        disc_initial->parentNode = disc_initial;

        return true;
    }
    else
    {
        //~ std::cerr << "ThetaStar: Initial point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
        disc_initial = NULL;
        return false;
    }
}

bool ThetaStar::setInitialPosition(Vector3 p)
{
    DiscretePosition p_ = discretizePosition(p);
		
	if(isInside(p_.x,p_.y,p_.z))
    {
        ThetaStartNodeLink *initialNodeInWorld = &discrete_world[getWorldIndex(
                    p_.x,
                    p_.y,
                    p_.z)];

        if(initialNodeInWorld->node==NULL)
        {
            initialNodeInWorld->node = new ThetaStarNode();
            initialNodeInWorld->node->point.x = p_.x;
            initialNodeInWorld->node->point.x = p_.y;
            initialNodeInWorld->node->point.x = p_.z;

            initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
        }
        disc_initial = initialNodeInWorld->node;

        initial_position = p;
        disc_initial->point = p_;
        disc_initial->parentNode = disc_initial;

        return true;
    }
    else
    {
        //ROS_ERROR("ThetaStar: Initial point [%f, %f, %f] not valid.", p.x, p.y, p.z);
        disc_initial = NULL;
        return false;
    }
}

Vector3 ThetaStar::getInitialPosition()
{
    return initial_position;
}


bool ThetaStar::setDiscreteFinalPosition(DiscretePosition p_)
{
    if(isInside(p_.x,p_.y,p_.z))
    {
        ThetaStartNodeLink *finalNodeInWorld = &discrete_world[getWorldIndex(
                    p_.x,
                    p_.y,
                    p_.z)];

        if(finalNodeInWorld->node==NULL)
        {
            finalNodeInWorld->node = new ThetaStarNode();
            finalNodeInWorld->node->point.x = p_.x;
            finalNodeInWorld->node->point.x = p_.y;
            finalNodeInWorld->node->point.x = p_.z;

            finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
        }
        disc_final = finalNodeInWorld->node;

        final_position.x = p_.x * step;
        final_position.y = p_.y * step;
        final_position.z = p_.z * step;
        disc_final->point = p_;

        return true;
    }
    else
    {
        //~ std::cerr << "ThetaStar: Final point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
        disc_final = NULL;
        return false;
    }
}

bool ThetaStar::setFinalPosition(Vector3 p)
{
    DiscretePosition p_ = discretizePosition(p);
    
    if(isInside(p_.x,p_.y,p_.z))
    {
        ThetaStartNodeLink *finalNodeInWorld = &discrete_world[getWorldIndex(
                    p_.x,
                    p_.y,
                    p_.z)];

        if(finalNodeInWorld->node==NULL)
        {
            finalNodeInWorld->node = new ThetaStarNode();
            finalNodeInWorld->node->point.x = p_.x;
            finalNodeInWorld->node->point.y = p_.y;
            finalNodeInWorld->node->point.z = p_.z;

            finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
        }
        disc_final = finalNodeInWorld->node;

        final_position = p;
        disc_final->point = p_;

        return true;
    }
    else
    {

        //~ std::cerr << "ThetaStar: Final point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
        disc_final = NULL;
        return false;
    }
}

Vector3 ThetaStar::getFinalPosition()
{
    return final_position;
}

void ThetaStar::setTimeOut(int sec)
{
    timeout = sec;
}

int  ThetaStar::getTimeOut()
{
    return timeout;
}

int  ThetaStar::calculateNewPath(void)
{
	//~ printf("Calculating...\n");

    if(disc_initial==NULL || disc_final==NULL)
    {
        std::cerr << "ThetaStar: Cannot calculate path. Initial or Final point not valid." << std::endl;
        return 0;
    }


    marker.points.clear();
    geometry_msgs::Point p;
    p.x = disc_initial->point.x * step;
    p.y = disc_initial->point.y * step;
    p.z = disc_initial->point.z * step;
    marker.points.push_back(p);

    //Initialize data structure. --> clear lists
    ThetaStarNode *erase_node;
    while(!open.empty())
    {
        erase_node = *open.begin();
        open.erase(open.begin());
        if(erase_node!=NULL)
        {
            erase_node->nodeInWorld->isInCandidateList = false;
            erase_node->nodeInWorld->isInOpenList = false;
        }
    }

    while(!candidates.empty())
    {
        erase_node = *candidates.begin();
        candidates.erase(candidates.begin());
        if(erase_node!=NULL)
        {
            erase_node->nodeInWorld->isInCandidateList = false;
            erase_node->nodeInWorld->isInOpenList = false;
        }
    }

    open.clear();
    candidates.clear();



    disc_initial->distanceFromInitialPoint = 0;
    disc_initial->lineDistanceToFinalPoint = weightedDistanceToGoal(*disc_initial);
    disc_initial->totalDistance = disc_initial->lineDistanceToFinalPoint + 0;
    disc_initial->parentNode = disc_initial;

    open.insert(disc_initial);
    disc_initial->nodeInWorld->isInOpenList = true;


    //Inicialize loop
    ThetaStarNode *min_distance = disc_initial; // s : current node
    bool noSolution = false;
    long iter= 0;

    if(isOccupied(*disc_initial) || isOccupied(*disc_final))
    {
        noSolution = true;
        std::cerr << "ThetaStar: Initial or Final point not free." << std::endl;
    }

    ros::Time last_time_ = ros::Time::now();
    while(!noSolution && (*min_distance)!=(*disc_final))
    {
        iter++;
        if(iter%100==0)
        {
            if((ros::Time::now() - last_time_).toSec() > timeout)
            {
                noSolution= true;
                std::cerr << "Theta Star: Timeout. Iteractions:" << iter << std::endl;
            }
        }

        //If there are more nodes...
        if(!open.empty())
        {
            //Look for minimun distance node
            min_distance = *open.begin();

#ifdef DEBUG
            publishRvizPoint(*min_distance,false);
            //usleep(20000);
#endif

            open.erase(open.begin());
            min_distance->nodeInWorld->isInOpenList = false;

            //Insert it in candidate list
            candidates.insert(min_distance);
            min_distance->nodeInWorld->isInCandidateList = true;

            //Look for Neighbors with line of sight
            set<ThetaStarNode*,NodePointerComparator> neighbors;
            getNeighbors(*min_distance,neighbors);

            //Check if exist line of sight.
            SetVertex(*min_distance,neighbors);

            set<ThetaStarNode*,NodePointerComparator>::iterator it_;
            it_ = neighbors.begin();
            ThetaStarNode *new_node;
            while(it_!= neighbors.end())
            {
                new_node = *it_;
                if(!new_node->nodeInWorld->isInCandidateList)
                {
                    if(!new_node->nodeInWorld->isInOpenList)
                    {
                        new_node->totalDistance =  std::numeric_limits<float>::max();
                        new_node->lineDistanceToFinalPoint = weightedDistanceToGoal(*new_node);
                        new_node->parentNode = min_distance;
                    }
                    UpdateVertex(*min_distance,*new_node);
                }
                it_++;
            }
        }
        else//If there are not nodes, do not exist solution
        {
            noSolution = true;

        }

    }

#ifdef DEBUG
    publishRvizPoint(*min_distance,true);
#endif
    //Path finished, get final path
    last_path.clear();
    ThetaStarNode *path_point;
    Vector3 point;

    path_point = min_distance;
    if(noSolution)
    {
        std::cerr << "Imposible to calculate a solution" << std::endl;
    }

    while(!noSolution && path_point!=disc_initial)
    {
        point.x = path_point->point.x * step;
        point.y = path_point->point.y * step;
        point.z = path_point->point.z * step;

        last_path.insert(last_path.begin(),point);

        path_point = path_point->parentNode;
        if(!path_point  || (path_point == path_point->parentNode && path_point!=disc_initial))
        {
            last_path.clear();
            break;
        }
    }
    return last_path.size();
}

vector<Vector3> ThetaStar::getCurrentPath()
{
    return last_path;
}


// Get trajectory from the thetaStar current path solution. First Implementation: Yaw constant along the entire trajectory
bool ThetaStar::getCurrentTrajectory_YawCte(trajectory_msgs::MultiDOFJointTrajectoryPtr &trajectory, geometry_msgs::Transform init_pose, float DPmax_h, float DPmax_v, float Vm_h, float Vm_v, float Vm_h_1, float Vm_v_1)
{
	// number of PATH points without the initial point
	int n_path = last_path.size();
	
	// index to trace the path
	int i = 0;
	
	// Time respect first path/trajectory position (0.0)
	double total_time = 0.0;
	
	// Trajectory point to fill the complete trajectory msg
	trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
	trajectory_point.transforms.resize(1);
    trajectory_point.velocities.resize(1);
    trajectory_point.accelerations.resize(1);
	
	// loop last and next path positions
	Vector3 last_position = initial_position;
	Vector3 next_position;
	
	// loop middle trajectory position (needed if next position is so far)
	Vector3 middle_position;
	
	// Flags 
	bool isFirst = true;		// to set a different velocity from initial position to the first position
	bool pathPointGot = false;	// to know if a middle point is necessary
	bool Limited_V = false;		// to know if a the next point is limited vertically, to check correctly the horizontal limitation

	// trajectory max increments (where descomposes DPmax_h)
	double DXmax, DYmax;

	// trajectory times
	double dt_h, dt_v, dt;
	
	// get the current yaw (odom) 
	double yaw_odom = get_yaw_from_quat(init_pose.rotation);
	
	/**---------- FIRST GET THE [X,Y,Z,T] TRAJECTORY, WIHTOUT YAW CHANGES ---------**/
	// entire path loop
	while(i<n_path)
	{
		// get next path position
		next_position = last_path[i];
			//~ printf("From: [%f, %f, %f] ---> to: [%f, %f, %f]\n", last_position.x, last_position.y, last_position.z, next_position.x, next_position.y, next_position.z);
		
		// trajectory middle waypoints
		middle_position = last_position;

		// two path points loop		
		pathPointGot = false;		
		while(!pathPointGot)
		{			
			// At first, next point path is possible and not vertically limited
			pathPointGot = true;
			Limited_V = false;
			
			// Check Vertically Limit
			if(fabs(next_position.z - last_position.z) > (DPmax_v + TOLERANCE))
			{
				// Z to the max
				if(next_position.z - last_position.z > 0.0)
					middle_position.z = last_position.z + DPmax_v;
				else
					middle_position.z = last_position.z - DPmax_v;
				
				// XY proportional to the Z
				double alfa =  DPmax_v / fabs(next_position.z - last_position.z); // Porcentaje del recorrido que se va a realizar hasta esta Z limitada
				middle_position.x = last_position.x + alfa * (next_position.x - last_position.x);
				middle_position.y = last_position.y + alfa * (next_position.y - last_position.y);
				
				// Exist limitation, so the point is not directly reached 
				pathPointGot = false;
				Limited_V = true;
			}
			
			// Check Horizontal Limit 
				/* WARNING: it is necessary to distinguish between if Z has been limited or not: 
				 *	- If not: The current X Y target are the next position in the path --> the next_position 
				 *	- If yes: The current X Y target are the proportional to the Z limited --> the current middle_position */					
			switch(Limited_V)
			{
				case false:
							if(get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y) > (DPmax_h + TOLERANCE))
							{
								// X and Y to the max (It exists a singularity at DX = 0.0, so if that directly set the known values)
								if(fabs(next_position.x - last_position.x) >= 0.001)
								{
									DXmax = DPmax_h / ( sqrt(1.0 + pow(next_position.y - last_position.y, 2)/pow(next_position.x - last_position.x, 2)) );
									DYmax = fabs(next_position.y - last_position.y)/fabs(next_position.x - last_position.x) * DXmax;
								}
								else
								{
									DXmax = 0.0;
									DYmax = DPmax_h;
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
								double beta =  DPmax_h / get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y);
								middle_position.z = last_position.z + beta * (next_position.z - last_position.z);
								
								// Exist limitation, so the point is not directly reached 
								pathPointGot = false;
							}
							
							break;
											
				case true:
							if(get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y) > (DPmax_h + TOLERANCE))
							{
								// X and Y to the max (It exists a singularity at DX = 0.0, so if that directly set the known values)
								if(fabs(middle_position.x - last_position.x) >= 0.001)
								{
									DXmax = DPmax_h/(sqrt(1.0 + pow(middle_position.y - last_position.y, 2)/pow(middle_position.x - last_position.x, 2)));
									DYmax = fabs(middle_position.y - last_position.y)/fabs(middle_position.x - last_position.x) * DXmax;
								}
								else
								{
									DXmax = 0.0;
									DYmax = DPmax_h;
								}
								
								// Z proportional to the XY limited (it need to be do previous to modificate middle_position.x and .y)
								double beta =  DPmax_h / get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y);
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
				//.. to the trajectory				
				// calculate neccesary time to get the horizontal and vertical position
				if(isFirst)
				{
					dt_h = get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y) / Vm_h_1;
					dt_v = fabs(middle_position.z - last_position.z) / Vm_v_1;
					isFirst = false;
				}
				else
				{
					dt_h = get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y) / Vm_h;
					dt_v = fabs(middle_position.z - last_position.z) / Vm_v;
				}
				
				// Set the maximum neccesary elapse time
				dt = max(dt_h, dt_v);
				total_time+=dt;
				//~ printf("Times: DT_H: %f, DT_V: %f, DT_Y: %f, DT: %f\n", dt_h, dt_v, dt_y, dt);
					
				// Set path position (last_position is the original last_position or the last middle_position if it exists)				
				set_position_yaw_and_time(trajectory_point, middle_position, yaw_odom, total_time);
				trajectory->points.push_back(trajectory_point);
					//~ printf("\tMiddle waypoint: [%f, %f, %f] at %f sec\n", middle_position.x, middle_position.y, middle_position.z, total_time);
				
				//.. to the algorihtm
				last_position = middle_position;
			}
			
		}

		// Set path position, always as Vm_1 (last_position is the original last_position or the last middle_position if it exists)
		dt_h = get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y) / Vm_h_1;
		dt_v = fabs(next_position.z - last_position.z) / Vm_v_1;

		// Set the maximum neccesary elapse time
		dt = max(dt_h, dt_v);	
		total_time+=dt;
		
		set_position_yaw_and_time(trajectory_point, next_position, yaw_odom, total_time);
		trajectory->points.push_back(trajectory_point);

		// next path position
		isFirst = true;
		last_position = next_position;
		i++;
	}
	
	
	//print actual trajectory vector state
	//~ printf_trajectory(*trajectory, "Trajectory_state_1");

	return true;
}


// Get trajectory from the thetaStar current path solution. Second Implementation: Set yaw ahead from Wp[k] to Wp[k+1] as reference from Wp[k] to Wp[k+1]
bool ThetaStar::getCurrentTrajectory_YawAtTime(trajectory_msgs::MultiDOFJointTrajectoryPtr &trajectory, geometry_msgs::Transform init_pose, float DPmax_h, float DPmax_v, float Vm_h, float Vm_v, float Vm_h_1, float Vm_v_1, float Wy_est)
{
	// number of PATH points without the initial point
	int n_path = last_path.size();
	
	// index to trace the path
	int i = 0;
	
	// Time respect first path/trajectory position (0.0)
	double total_time = 0.0;
	
	// Trajectory point to fill the complete trajectory msg
	trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
	trajectory_point.transforms.resize(1);
    trajectory_point.velocities.resize(1);
    trajectory_point.accelerations.resize(1);
	
	// loop last and next path positions
	Vector3 last_position = initial_position;
	Vector3 next_position;
	
	// loop middle trajectory position (needed if next position is so far)
	Vector3 middle_position;
	
	// Flags 
	bool isFirst = true;		// to set a different velocity from initial position to the first position
	bool pathPointGot = false;	// to know if a middle point is necessary
	bool Limited_V = false;		// to know if a the next point is limited vertically, to check correctly the horizontal limitation

	// trajectory max increments (where descomposes DPmax_h)
	double DXmax, DYmax;

	// trajectory times
	double dt_h, dt_v, dt;
	
	// get the current yaw (odom) 
	double yaw_odom = get_yaw_from_quat(init_pose.rotation);
	
	/**---------- FIRST GET THE [X,Y,Z,T] TRAJECTORY, WIHTOUT YAW CHANGES ---------**/
	// entire path loop
	while(i<n_path)
	{
		// get next path position
		next_position = last_path[i];
			//~ printf("From: [%f, %f, %f] ---> to: [%f, %f, %f]\n", last_position.x, last_position.y, last_position.z, next_position.x, next_position.y, next_position.z);
		
		// trajectory middle waypoints
		middle_position = last_position;

		// two path points loop		
		pathPointGot = false;		
		while(!pathPointGot)
		{			
			// At first, next point path is possible and not vertically limited
			pathPointGot = true;
			Limited_V = false;
			
			// Check Vertically Limit
			if(fabs(next_position.z - last_position.z) > (DPmax_v + TOLERANCE))
			{
				// Z to the max
				if(next_position.z - last_position.z > 0.0)
					middle_position.z = last_position.z + DPmax_v;
				else
					middle_position.z = last_position.z - DPmax_v;
				
				// XY proportional to the Z
				double alfa =  DPmax_v / fabs(next_position.z - last_position.z); // Porcentaje del recorrido que se va a realizar hasta esta Z limitada
				middle_position.x = last_position.x + alfa * (next_position.x - last_position.x);
				middle_position.y = last_position.y + alfa * (next_position.y - last_position.y);
				
				// Exist limitation, so the point is not directly reached 
				pathPointGot = false;
				Limited_V = true;
			}
			
			// Check Horizontal Limit 
				/* WARNING: it is necessary to distinguish between if Z has been limited or not: 
				 *	- If not: The current X Y target are the next position in the path --> the next_position 
				 *	- If yes: The current X Y target are the proportional to the Z limited --> the current middle_position */					
			switch(Limited_V)
			{
				case false:
							if(get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y) > (DPmax_h + TOLERANCE))
							{
								// X and Y to the max (It exists a singularity at DX = 0.0, so if that directly set the known values)
								if(fabs(next_position.x - last_position.x) >= 0.001)
								{
									DXmax = DPmax_h / ( sqrt(1.0 + pow(next_position.y - last_position.y, 2)/pow(next_position.x - last_position.x, 2)) );
									DYmax = fabs(next_position.y - last_position.y)/fabs(next_position.x - last_position.x) * DXmax;
								}
								else
								{
									DXmax = 0.0;
									DYmax = DPmax_h;
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
								double beta =  DPmax_h / get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y);
								middle_position.z = last_position.z + beta * (next_position.z - last_position.z);
								
								// Exist limitation, so the point is not directly reached 
								pathPointGot = false;
							}
							
							break;
											
				case true:
							if(get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y) > (DPmax_h + TOLERANCE))
							{
								// X and Y to the max (It exists a singularity at DX = 0.0, so if that directly set the known values)
								if(fabs(middle_position.x - last_position.x) >= 0.001)
								{
									DXmax = DPmax_h/(sqrt(1.0 + pow(middle_position.y - last_position.y, 2)/pow(middle_position.x - last_position.x, 2)));
									DYmax = fabs(middle_position.y - last_position.y)/fabs(middle_position.x - last_position.x) * DXmax;
								}
								else
								{
									DXmax = 0.0;
									DYmax = DPmax_h;
								}
								
								// Z proportional to the XY limited (it need to be do previous to modificate middle_position.x and .y)
								double beta =  DPmax_h / get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y);
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
				//.. to the trajectory				
				// calculate neccesary time to get the horizontal and vertical position
				if(isFirst)
				{
					dt_h = get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y) / Vm_h_1;
					dt_v = fabs(middle_position.z - last_position.z) / Vm_v_1;
					isFirst = false;
				}
				else
				{
					dt_h = get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y) / Vm_h;
					dt_v = fabs(middle_position.z - last_position.z) / Vm_v;
				}
				
				// Set the maximum neccesary elapse time
				dt = max(dt_h, dt_v);
				total_time+=dt;
				//~ printf("Times: DT_H: %f, DT_V: %f, DT_Y: %f, DT: %f\n", dt_h, dt_v, dt_y, dt);
					
				// Set path position (last_position is the original last_position or the last middle_position if it exists)				
				set_position_yaw_and_time(trajectory_point, middle_position, yaw_odom, total_time);
				trajectory->points.push_back(trajectory_point);
					//~ printf("\tMiddle waypoint: [%f, %f, %f] at %f sec\n", middle_position.x, middle_position.y, middle_position.z, total_time);
				
				//.. to the algorihtm
				last_position = middle_position;
			}
			
		}

		// Set path position, always as Vm_1 (last_position is the original last_position or the last middle_position if it exists)
		dt_h = get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y) / Vm_h_1;
		dt_v = fabs(next_position.z - last_position.z) / Vm_v_1;

		// Set the maximum neccesary elapse time
		dt = max(dt_h, dt_v);	
		total_time+=dt;
		
		set_position_yaw_and_time(trajectory_point, next_position, yaw_odom, total_time);
		trajectory->points.push_back(trajectory_point);

		// next path position
		isFirst = true;
		last_position = next_position;
		i++;
	}
	
	
	//print actual trajectory vector state
	//~ printf_trajectory(*trajectory, "Trajectory_state_1");
	
	/**---------- SECOND GET THE YAWS AHEAD REFERENCES FOR THE PREVIOUS COMPUTED TRAJECTORY ---------**/
	// first yaw from initial pose (odom)
	double yaw = yaw_odom;
	double dyaw = 0.0;
	double dt_y = 0.0;
	double last_yaw = yaw;
	tf::Quaternion q; 		// yaw as quaternion to the msg	
	double time_inc = 0.0;  // time increment if yaw reference changes
	for(int k=0; k < trajectory->points.size(); k++)
	{
		// get yaw ahead for the next segment, yaw increment and neccesary time for this turn
		if(k==0)
		{
			yaw = atan2(trajectory->points[k].transforms[0].translation.y - initial_position.y, trajectory->points[k].transforms[0].translation.x - initial_position.x);
		}
		else
		{
			yaw = atan2(trajectory->points[k].transforms[0].translation.y - trajectory->points[k-1].transforms[0].translation.y, trajectory->points[k].transforms[0].translation.x - trajectory->points[k-1].transforms[0].translation.x);
		}
		dyaw = get_dyaw(yaw, last_yaw);
		dt_y = dyaw / Wy_est;
		last_yaw = yaw;
		// set as quaternion
		q.setRPY(0.0,0.0,yaw);
		trajectory->points[k].transforms[0].rotation.x = q.x();
		trajectory->points[k].transforms[0].rotation.y = q.y();
		trajectory->points[k].transforms[0].rotation.z = q.z();
		trajectory->points[k].transforms[0].rotation.w = q.w();
		// calculate time increment
		if(k==0)
			time_inc = dt_y - trajectory->points[k].time_from_start.toSec();
		else
			time_inc = dt_y - ( trajectory->points[k].time_from_start.toSec() - trajectory->points[k-1].time_from_start.toSec() );
		
		// check if neccesary yaw turn time is bigger than for the position increment
		if(time_inc > 0.0)
		{
			//~ ROS_INFO("[%d] Exist time increment: %f",  k, time_inc);
			// update time for this waypoint and all next waypoints
			for(int j=k;j<trajectory->points.size();j++)
			{
				trajectory->points[j].time_from_start += ros::Duration(time_inc);
			}
			//~ printf_trajectory(*trajectory, "Trajectory_state_middle");
		}
	}
	
	//print actual trajectory vector state
	//~ printf_trajectory(*trajectory, "Trajectory_state_2");

	return true;
}

// Get trajectory from the thetaStar current path solution. Third Implementation: Set yaw ahead from Wp[k] to Wp[k+1] as reference from Wp[k-1] to Wp[k]
bool ThetaStar::getCurrentTrajectory_YawInAdvance(trajectory_msgs::MultiDOFJointTrajectoryPtr &trajectory, geometry_msgs::Transform init_pose, float DPmax_h, float DPmax_v, float Vm_h, float Vm_v, float Vm_h_1, float Vm_v_1, float Wy_est)
{
	// number of PATH points without the initial point
	int n_path = last_path.size();
		
	// index to trace the path
	int i = 0;
	
	// Time respect first path/trajectory position (0.0)
	double total_time = 0.0;
	
	// Trajectory point to fill the complete trajectory msg
	trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
	trajectory_point.transforms.resize(1);
    trajectory_point.velocities.resize(1);
    trajectory_point.accelerations.resize(1);
	
	// loop last and next path positions
	Vector3 last_position = initial_position;
	Vector3 next_position;
	
	// loop middle trajectory position (needed if next position is so far)
	Vector3 middle_position;
	
	// Flags 
	bool isFirst = true;		// to set a different velocity from initial position to the first position
	bool pathPointGot = false;	// to know if a middle point is necessary
	bool Limited_V = false;		// to know if a the next point is limited vertically, to check correctly the horizontal limitation

	// trajectory max increments (where descomposes DPmax_h)
	double DXmax, DYmax;

	// trajectory times
	double dt_h, dt_v, dt;
	
	/**---------- FIRST TRAJECTORY WP = INITIAL POSITION + YAW AHEAD TO THE FIRST PATH WP WITH ENOUGH HORIZONTAL MOVEMENT---------**/
	// first get the current yaw (odom) and declare vars.
	double yaw_odom = get_yaw_from_quat(init_pose.rotation);
	double yaw = yaw_odom;
	double dyaw = 0.0;
	double dt_y = 0.0;
	double last_yaw = yaw;
	total_time = dt_y;
	
	// Set the first wp as the initial position and the yaw = dir(odom,wp1) IF the horizontal movement is enough. 
	// If not try with next wp. If not let yaw reference as yaw_odom and not se this first wp 
	while(get_horizontal_norm(last_path[i].x - initial_position.x, last_path[i].y - initial_position.y) < SIGHT_AHEAD_MIN_HORIZ_MOVEMENT)
	{
		i++;
		
		if(i>n_path)
		{	
			i=-1;
			break;
		}
	}
	if(i>=0)
	{
		yaw = atan2(last_path[i].y - initial_position.y, last_path[i].x - initial_position.x);
		dyaw = get_dyaw(yaw, yaw_odom);
		dt_y = dyaw / Wy_est;
		last_yaw = yaw;
		total_time = dt_y;
		set_position_yaw_and_time(trajectory_point, initial_position, yaw, total_time);
		trajectory->points.push_back(trajectory_point);
	}
	
	
	/**---------- SECOND GET THE [X,Y,Z,T] TRAJECTORY, WIHTOUT YAW CHANGES ---------**/
	// entire path loop
	i=0;
	while(i<n_path)
	{
		// get next path position
		next_position = last_path[i];
			//~ printf("From: [%f, %f, %f] ---> to: [%f, %f, %f]\n", last_position.x, last_position.y, last_position.z, next_position.x, next_position.y, next_position.z);
		
		// trajectory middle waypoints
		middle_position = last_position;

		// two path points loop		
		pathPointGot = false;		
		while(!pathPointGot)
		{			
			// At first, next point path is possible and not vertically limited
			pathPointGot = true;
			Limited_V = false;
			
			// Check Vertically Limit
			if(fabs(next_position.z - last_position.z) > (DPmax_v + TOLERANCE))
			{
				// Z to the max
				if(next_position.z - last_position.z > 0.0)
					middle_position.z = last_position.z + DPmax_v;
				else
					middle_position.z = last_position.z - DPmax_v;
				
				// XY proportional to the Z
				double alfa =  DPmax_v / fabs(next_position.z - last_position.z); // Porcentaje del recorrido que se va a realizar hasta esta Z limitada
				middle_position.x = last_position.x + alfa * (next_position.x - last_position.x);
				middle_position.y = last_position.y + alfa * (next_position.y - last_position.y);
				
				// Exist limitation, so the point is not directly reached 
				pathPointGot = false;
				Limited_V = true;
			}
			
			// Check Horizontal Limit 
				/* WARNING: it is necessary to distinguish between if Z has been limited or not: 
				 *	- If not: The current X Y target are the next position in the path --> the next_position 
				 *	- If yes: The current X Y target are the proportional to the Z limited --> the current middle_position */					
			switch(Limited_V)
			{
				case false:
							if(get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y) > (DPmax_h + TOLERANCE))
							{
								// X and Y to the max (It exists a singularity at DX = 0.0, so if that directly set the known values)
								if(fabs(next_position.x - last_position.x) >= 0.001)
								{
									DXmax = DPmax_h / ( sqrt(1.0 + pow(next_position.y - last_position.y, 2)/pow(next_position.x - last_position.x, 2)) );
									DYmax = fabs(next_position.y - last_position.y)/fabs(next_position.x - last_position.x) * DXmax;
								}
								else
								{
									DXmax = 0.0;
									DYmax = DPmax_h;
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
								double beta =  DPmax_h / get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y);
								middle_position.z = last_position.z + beta * (next_position.z - last_position.z);
								
								// Exist limitation, so the point is not directly reached 
								pathPointGot = false;
							}
							
							break;
											
				case true:
							if(get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y) > (DPmax_h + TOLERANCE))
							{
								// X and Y to the max (It exists a singularity at DX = 0.0, so if that directly set the known values)
								if(fabs(middle_position.x - last_position.x) >= 0.001)
								{
									DXmax = DPmax_h/(sqrt(1.0 + pow(middle_position.y - last_position.y, 2)/pow(middle_position.x - last_position.x, 2)));
									DYmax = fabs(middle_position.y - last_position.y)/fabs(middle_position.x - last_position.x) * DXmax;
								}
								else
								{
									DXmax = 0.0;
									DYmax = DPmax_h;
								}
								
								// Z proportional to the XY limited (it need to be do previous to modificate middle_position.x and .y)
								double beta =  DPmax_h / get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y);
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
				//.. to the trajectory				
				// calculate neccesary time to get the horizontal and vertical position
				if(isFirst)
				{
					dt_h = get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y) / Vm_h_1;
					dt_v = fabs(middle_position.z - last_position.z) / Vm_v_1;
					isFirst = false;
				}
				else
				{
					dt_h = get_horizontal_norm(middle_position.x - last_position.x, middle_position.y - last_position.y) / Vm_h;
					dt_v = fabs(middle_position.z - last_position.z) / Vm_v;
				}
				
				// Set the maximum neccesary elapse time
				dt = max(dt_h, dt_v);
				total_time+=dt;
				//~ printf("Times: DT_H: %f, DT_V: %f, DT_Y: %f, DT: %f\n", dt_h, dt_v, dt_y, dt);
					
				// Set path position (last_position is the original last_position or the last middle_position if it exists)				
				set_position_yaw_and_time(trajectory_point, middle_position, yaw, total_time);
				trajectory->points.push_back(trajectory_point);
					//~ printf("\tMiddle waypoint: [%f, %f, %f] at %f sec\n", middle_position.x, middle_position.y, middle_position.z, total_time);
				
				//.. to the algorihtm
				last_position = middle_position;
			}
			
		}

		// Set path position, always as Vm_1 (last_position is the original last_position or the last middle_position if it exists)
		dt_h = get_horizontal_norm(next_position.x - last_position.x, next_position.y - last_position.y) / Vm_h_1;
		dt_v = fabs(next_position.z - last_position.z) / Vm_v_1;

		// Set the maximum neccesary elapse time
		dt = max(dt_h, dt_v);	
		total_time+=dt;
		
		set_position_yaw_and_time(trajectory_point, next_position, yaw, total_time);
		trajectory->points.push_back(trajectory_point);

		// next path position
		isFirst = true;
		last_position = next_position;
		i++;
	}
	
	
	//print actual trajectory vector state
	//~ printf_trajectory(*trajectory, "Trajectory_state_1");
	
	/**---------- THIRD GET THE YAWS AHEAD REFERENCES FOR THE PREVIOUS COMPUTED TRAJECTORY ---------**/
	tf::Quaternion q; 		// yaw as quaternion to the msg
	double time_inc = 0.0;  // time increment if yaw reference changes
	for(int k=1; k < trajectory->points.size()-1; k++)
	{
		// Search the next minimum horizontal movement to set as a yaw reference
		i = 0; // index to search the next bigger enough segment to compute the yaw reference (eliminating yaw references for small horizontal increments)
		while(get_horizontal_distance(trajectory->points[k+1+i], trajectory->points[k+i]) < SIGHT_AHEAD_MIN_HORIZ_MOVEMENT)
		{
			i++;
			
			if(k+i+1>=trajectory->points.size())
			{
				// if all segments are shorter than SIGHT_AHEAD_MIN_HORIZ_MOVEMENT, doesn't change the yaw reference because it would be a only-vertical movement
				i = -1; 
				break;
			}
		}
		
		// get yaw ahead for the next segment, yaw increment and neccesary time for this turn
		if(i>=0)
			yaw = atan2(trajectory->points[k+1+i].transforms[0].translation.y - trajectory->points[k+i].transforms[0].translation.y, trajectory->points[k+1+i].transforms[0].translation.x - trajectory->points[k+i].transforms[0].translation.x);
		
		dyaw = get_dyaw(yaw, last_yaw);
		dt_y = dyaw / Wy_est;
		last_yaw = yaw;
		// set as quaternion
		q.setRPY(0.0,0.0,yaw);
		trajectory->points[k].transforms[0].rotation.x = q.x();
		trajectory->points[k].transforms[0].rotation.y = q.y();
		trajectory->points[k].transforms[0].rotation.z = q.z();
		trajectory->points[k].transforms[0].rotation.w = q.w();
		// calculate time increment
		time_inc = dt_y - ( trajectory->points[k].time_from_start.toSec() - trajectory->points[k-1].time_from_start.toSec() );
		// check if neccesary yaw turn time is bigger than for the position increment
		if(time_inc > 0.0)
		{
			//~ ROS_INFO("[%d] Exist time increment: %f",  k, time_inc);
			// update time for this waypoint and all next waypoints
			for(int j=k;j<trajectory->points.size();j++)
			{
				trajectory->points[j].time_from_start += ros::Duration(time_inc);
			}
			//~ printf_trajectory(*trajectory, "Trajectory_state_middle");
		}
	}
	// last waypoint yaw reference (same that second last, so yaw does not change, so time does not change)
	q.setRPY(0.0,0.0,yaw); // Note: If it's only one wp the 'for' statement does not execute nothing, so its neccesary this line
	trajectory->points.back().transforms[0].rotation.x = q.x();
	trajectory->points.back().transforms[0].rotation.y = q.y();
	trajectory->points.back().transforms[0].rotation.z = q.z();
	trajectory->points.back().transforms[0].rotation.w = q.w();
	
	//print actual trajectory vector state
	//~ printf_trajectory(*trajectory, "Trajectory_state_2");

	return true;
}

// Same as 'getCurrentTrajectory_YawInAdvance()' but setting a pre-computed yaw reference to the last waypoint
bool ThetaStar::getCurrentTrajectory_YawInAdvance_WithFinalYaw(trajectory_msgs::MultiDOFJointTrajectoryPtr &trajectory, geometry_msgs::Transform init_pose, float DPmax_h, float DPmax_v, float Vm_h, float Vm_v, float Vm_h_1, float Vm_v_1, float Wy_est, double final_yaw_ref)
{
	// Get the current trajectory with yaw in advance
	//~ printf("getCurrentTrajectory_YawInAdvance_WithFinalYaw()...\n");
	getCurrentTrajectory_YawInAdvance(trajectory, init_pose, DPmax_h, DPmax_v, Vm_h, Vm_v, Vm_h_1, Vm_v_1, Wy_est);
	//~ printf("getCurrentTrajectory_YawInAdvance_WithFinalYaw()... size: %d\n", trajectory->points.size());
	
	// Modify the yaw reference and the time if it's neccesary
	double last_wp_yaw = get_yaw_from_quat(trajectory->points.back().transforms[0].rotation);
	double dyaw = fabs(final_yaw_ref - last_wp_yaw);
	if(dyaw>0.0)
	{
		// Set new yaw
		tf::Quaternion q;
		q.setRPY(0.0,0.0,final_yaw_ref);
		trajectory->points.back().transforms[0].rotation.x = q.x();
		trajectory->points.back().transforms[0].rotation.y = q.y();
		trajectory->points.back().transforms[0].rotation.z = q.z();
		trajectory->points.back().transforms[0].rotation.w = q.w();
		
		// Check if its neccesary more time (only if trajectory is longer than only one wp)
		int traj_size = trajectory->points.size();
		if(traj_size > 1)
		{
			double pre_last_yaw  = get_yaw_from_quat(trajectory->points.at(traj_size - 2).transforms[0].rotation);
			double pre_last_time = trajectory->points.at(traj_size - 2).time_from_start.toSec();
			dyaw = get_dyaw(final_yaw_ref, pre_last_yaw);
			double current_dt = trajectory->points.back().time_from_start.toSec() - pre_last_time;
			double new_dt = dyaw/Wy_est;
			if(new_dt > current_dt)
			{
				// Set new time
				trajectory->points.back().time_from_start = ros::Duration(new_dt + pre_last_time);
			} 
		}
	}
}

bool ThetaStar::lineofsight(ThetaStarNode &p1, ThetaStarNode &p2)
{
    //min distance for no collision, in discrete space
    float min_distance = 5.0 * step; // 1.9; //sqrt(8);
    int extra_cells = 0; // min(2.0f,min_distance);

    //distance doble triangle.
    int x0 = max(min(p1.point.x,p2.point.x) - extra_cells,ws_x_min);
    int x1 = min(max(p1.point.x,p2.point.x) + extra_cells,ws_x_max);
    int y0 = max(min(p1.point.y,p2.point.y) - extra_cells,ws_y_min);
    int y1 = min(max(p1.point.y,p2.point.y) + extra_cells,ws_y_max);
    int z0 = max(min(p1.point.z,p2.point.z) - extra_cells,ws_z_min);
    int z1 = min(max(p1.point.z,p2.point.z) + extra_cells,ws_z_max);


    if(isOccupied(p1) || isOccupied(p2))
        return false;


    float base = distanceBetween2nodes(p1,p2);
    for(int x= x0; x <= x1; x++)
        for(int y= y0; y <= y1; y++)
            for(int z = z0; z <= z1; z++)
            {
                //If the point is occupied, we have to calculate distance to the line.
                if(!discrete_world[getWorldIndex(x,y,z)].notOccupied)
                {
                    //If base is zero and node is occcupied, directly does not exist line of sight
                    if(base==0)
                    {
#ifdef DEBUG
						geometry_msgs::Point p;
						p.x = p2.point.x * step;
						p.y = p2.point.y * step;
						p.z = p2.point.z * step;
                        marker_no_los.header.stamp = ros::Time();
						marker_no_los.header.seq++;
                        marker_no_los.points.push_back(p);
                        no_los_marker_pub_.publish( marker_no_los );
#endif               
                        
                        return false;
                    }

                    //cout << "Cell is occupied" << endl;
                    float a = sqrt(pow(x-p1.point.x,2)+ pow(p1.point.y-y,2) + pow(p1.point.z-z,2));
                    float c = sqrt(pow(p2.point.x-x,2)+ pow(p2.point.y-y,2) + pow(p2.point.z-z,2));
                    float l = (pow(base,2) + pow(a,2) - pow(c,2))/(2*base);
                    float distance = sqrt(abs(pow(a,2) - pow(l,2)));

                    if(distance <= min_distance)
                    {
#ifdef DEBUG                        
                        geometry_msgs::Point p;
						p.x = p2.point.x * step;
						p.y = p2.point.y * step;
						p.z = p2.point.z * step;
                        marker_no_los.header.stamp = ros::Time();
						marker_no_los.header.seq++;
                        marker_no_los.points.push_back(p);
                        no_los_marker_pub_.publish( marker_no_los );
#endif                                       
                        return false;
                    }
                }
            }

    return true;
}

void ThetaStar::getNeighbors(ThetaStarNode &node, set<ThetaStarNode*,NodePointerComparator> &neighbors)
{
    neighbors.clear();
    
    DiscretePosition node_temp;
    
    for(int i=-1;i<2;i++)
    {
        for(int j=-1;j<2;j++)
        {
            for(int k=-1;k<2;k++)
            {
                /*
                 *Ignore ourselves
                */
                if(i!=0 || j!=0 || k!=0)
                {
					node_temp.x = node.point.x + i;
					node_temp.y = node.point.y + j;
					node_temp.z = node.point.z + k;
                    
                    if(isInside(node_temp.x, node_temp.y, node_temp.z))
                    {
                        int nodeInWorld = getWorldIndex(node_temp.x, node_temp.y, node_temp.z);
                        
                        ThetaStartNodeLink *new_neighbor = &discrete_world[nodeInWorld];

                        if(new_neighbor->node==NULL)
                        {
                            new_neighbor->node = new ThetaStarNode();
                            new_neighbor->node->point.x = node_temp.x;
                            new_neighbor->node->point.y = node_temp.y;
                            new_neighbor->node->point.z = node_temp.z;
                            new_neighbor->node->nodeInWorld = new_neighbor;
                            new_neighbor->node->parentNode = &node;
                        }

                        if(new_neighbor->isInCandidateList || lineofsight(node,*new_neighbor->node))
                        {
                            neighbors.insert (new_neighbor->node);
                        }
                    }
                    else
                    {
                        // delete new_neighbor;
                    }
                }
            }
        }
    }
}

float ThetaStar::distanceToGoal(ThetaStarNode node)
{
    return sqrt(pow(disc_final->point.x-node.point.x,2) +
                pow(disc_final->point.y-node.point.y,2) +
                pow(disc_final->point.z-node.point.z,2));
}

float ThetaStar::weightedDistanceToGoal(ThetaStarNode node)
{
    return sqrt(				pow(disc_final->point.x-node.point.x,2) +
								pow(disc_final->point.y-node.point.y,2) +
                z_weight_cost * pow(disc_final->point.z-node.point.z,2));
}

float ThetaStar::distanceBetween2nodes(ThetaStarNode &n1,ThetaStarNode &n2)
{
    return sqrt(pow(n1.point.x-n2.point.x,2) +
                pow(n1.point.y-n2.point.y,2) +
                pow(n1.point.z-n2.point.z,2));
}

float ThetaStar::weightedDistanceBetween2nodes(ThetaStarNode &n1,ThetaStarNode &n2)
{
    return sqrt(				pow(n1.point.x-n2.point.x,2) +
								pow(n1.point.y-n2.point.y,2) +
                z_weight_cost * pow(n1.point.z-n2.point.z,2));
}

float ThetaStar::distanceFromInitialPoint(ThetaStarNode node, ThetaStarNode parent)
{
    float res;
    if(isOccupied(node))
        res =  std::numeric_limits<float>::max();
    else
        if(parent.distanceFromInitialPoint==std::numeric_limits<float>::max())
            res = parent.distanceFromInitialPoint;
        else
        {
            res = parent.distanceFromInitialPoint + initial_point_factor * (sqrt(	pow(node.point.x-parent.point.x,2) +
																					pow(node.point.y-parent.point.y,2) +
																					pow(node.point.z-parent.point.z,2)));
        }

    return res;
}

float ThetaStar::weightedDistanceFromInitialPoint(ThetaStarNode node, ThetaStarNode parent)
{
    float res;
    if(isOccupied(node))
        res =  std::numeric_limits<float>::max();
    else
        if(parent.distanceFromInitialPoint==std::numeric_limits<float>::max())
            res = parent.distanceFromInitialPoint;
        else
        {
            res = parent.distanceFromInitialPoint + initial_point_factor * (sqrt(				pow(node.point.x-parent.point.x,2) +
																								pow(node.point.y-parent.point.y,2) +
																				z_weight_cost *	pow(node.point.z-parent.point.z,2)));
        }

    return res;
}

DiscretePosition ThetaStar::discretizePosition(Vector3 p)
{
    DiscretePosition res;

    res.x = p.x*step_inv;
    res.y = p.y*step_inv;
    res.z = p.z*step_inv;

    return res;
}

bool ThetaStar::isOccupied(ThetaStarNode n)
{
    return !discrete_world[getWorldIndex(n.point.x,n.point.y,n.point.z)].notOccupied;
}


bool ThetaStar::isInitialPositionOccupied()
{
    if(isOccupied(*disc_initial))
    	return true;
	else
		return false;
}

bool ThetaStar::isFinalPositionOccupied()
{
    if(isOccupied(*disc_final))
    	return true;
	else
		return false;
}

float ThetaStar::getMapResolution()
{
	return step;
}

inline bool ThetaStar::isInside(ThetaStarNode n)
{
    return  isInside(n.point.x,n.point.y,n.point.z);
}

inline bool ThetaStar::isInside(int &x, int &y, int &z)
{
    return  (x <= ws_x_max && x >= ws_x_min) &&
            (y <= ws_y_max && y >= ws_y_min) &&
            (z <= ws_z_max && z >= ws_z_min);
}

inline bool ThetaStar::isInsideAux(int &x, int &y, int &z)
{
    return  (x <= ws_x_max_aux && x >= ws_x_min_aux) &&
            (y <= ws_y_max_aux && y >= ws_y_min_aux) &&
            (z <= ws_z_max_aux && z >= ws_z_min_aux);
}

void ThetaStar::ComputeCost(ThetaStarNode &s, ThetaStarNode &s2)
{
    double distanceParent2 = weightedDistanceBetween2nodes((*s.parentNode),s2);
    if(s.parentNode->distanceFromInitialPoint + distanceParent2 + s2.lineDistanceToFinalPoint < s2.totalDistance)
    {
        s2.parentNode = s.parentNode;
        s2.distanceFromInitialPoint = weightedDistanceFromInitialPoint(s2,*s2.parentNode);
        s2.totalDistance = s2.distanceFromInitialPoint + s2.lineDistanceToFinalPoint;
    }
}

void ThetaStar::UpdateVertex(ThetaStarNode &s, ThetaStarNode &s2)
{
    float g_old = s2.totalDistance;

    ComputeCost(s,s2);
    if(s2.totalDistance < g_old)
    {
        if(s2.nodeInWorld->isInOpenList)
        {
            open.erase(&s2);
        }
        open.insert(&s2);
    }

}

void ThetaStar::SetVertex(ThetaStarNode &s,set<ThetaStarNode*,NodePointerComparator> &neighbors)
{
    if(!lineofsight(*s.parentNode,s))
    {
        float g_value = std::numeric_limits<float>::max();
        ThetaStarNode *parentCandidate = NULL;
        set<ThetaStarNode*,NodePointerComparator>::iterator it_;
        it_ = neighbors.begin();
        ThetaStarNode *new_node;
        bool candidatesNotEmpty = false;
        bool hasLessNumber = false;

        while(it_!= neighbors.end())
        {
            new_node = *it_;
            if(new_node->nodeInWorld->isInCandidateList)
            {
                candidatesNotEmpty = true;
                float g_new = new_node->distanceFromInitialPoint + weightedDistanceBetween2nodes(*new_node,s);
                if(g_new < g_value)
                {
                    hasLessNumber = true;
                    g_value = g_new;
                    parentCandidate = new_node;
                }
            }
            it_++;
        }

#ifdef DEBUG
        if(parentCandidate==NULL)
        {
            std::cerr << "Parent not found" << std::endl;
            if(candidatesNotEmpty)
            {
                std::cerr << "There are one candidate" << std::endl;
            }

            if(hasLessNumber)
            {
                std::cerr << "Is less than..." << std::endl;
            }
        }
#endif
        if(parentCandidate!=NULL)
        {
            s.parentNode = parentCandidate;
        }
        s.distanceFromInitialPoint = g_value;
        s.totalDistance = g_value + s.lineDistanceToFinalPoint;
    }
}

double ThetaStar::g(ThetaStarNode &s)
{
    if(s.parentNode==NULL)
    {
        std::cerr << "Error: Parent is null" << std::endl;
        return std::numeric_limits<double>::max();
    }
    else
    {
        float distanceFromInitial_ = weightedDistanceFromInitialPoint(s,*s.parentNode);
        float distanceToGoal_ = weightedDistanceToGoal(s);
        return distanceFromInitial_ + distanceToGoal_;
    }
}

// Aux Function: Set and check if a position is a valid initial position. 
// Return false is this position is outside the workspace or is occupied 
inline bool ThetaStar::is_a_valid_discrete_initial_position(DiscretePosition p)
{
	if(setDiscreteInitialPosition(p))
	{
		if(!isInitialPositionOccupied())	
		{
			ROS_INFO("ThetaStar: Initial discrete position [%d, %d, %d] set correctly", p.x,p.y,p.z);
			return true;
		}
	}
	else	
		ROS_WARN("ThetaStar: Initial position outside the workspace attempt!!");
	
	return false;
}
// Aux Function: Set and check if a position is a valid final position. 
// Return false is this position is outside the workspace or is occupied 
inline bool ThetaStar::is_a_valid_discrete_final_position(DiscretePosition p)
{
	if(setDiscreteFinalPosition(p))
	{
		if(!isFinalPositionOccupied())	
		{
			ROS_INFO("ThetaStar: Final discrete position [%d, %d, %d] set correctly", p.x,p.y,p.z);
			return true;
		}
	}
	else	
		ROS_WARN("ThetaStar: Final position outside the workspace attempt!!");
	
	return false;
}

// Aux Function: Set and search a valid initial position in a horizontal ring 
// centered in '(xs,ys,zs)' and radius 'd'
inline bool ThetaStar::search_initial_position_in_xy_ring(int xs, int ys, int zs, int d)
{
	// 2 sides of the rectangular ring ..
	for (int i = 0; i < d + 1; i++)
	{
		DiscretePosition p1;
		p1.x = xs - d + i;
		p1.y = ys - i;
		p1.z = zs;
		
		if(is_a_valid_discrete_initial_position(p1))
			return true;
		
		if(d != 0)	// if level is zero, is the same that previous point check
		{
			p1.x = xs + d - i;
			p1.y = ys + i;

			if(is_a_valid_discrete_initial_position(p1))
				return true;
		}
	}
	
	// .. and the other ones
	for (int i = 1; i < d; i++)
	{
		DiscretePosition p2;
		p2.x = xs - i;
		p2.y = ys + d - i;
		p2.z = zs;

		if(is_a_valid_discrete_initial_position(p2))
			return true;

		p2.x = xs + d - i;
		p2.y = ys - i;
		
		if(is_a_valid_discrete_initial_position(p2))
			return true;
	}
	
	return false;
}

// Aux Function: Set and search a valid final position in a horizontal ring 
// centered in '(xs,ys,zs)' and radius 'd'
inline bool ThetaStar::search_final_position_in_xy_ring(int xs, int ys, int zs, int d)
{
	// 2 sides of the rectangular ring ..
	for (int i = 0; i < d + 1; i++)
	{
		DiscretePosition p1;
		p1.x = xs - d + i;
		p1.y = ys - i;
		p1.z = zs;
		
		if(is_a_valid_discrete_final_position(p1))
			return true;
		
		if(d != 0)	// if level is zero, is the same that previous point check
		{
			p1.x = xs + d - i;
			p1.y = ys + i;

			if(is_a_valid_discrete_final_position(p1))
				return true;
		}
	}
	
	// .. and the other ones
	for (int i = 1; i < d; i++)
	{
		DiscretePosition p2;
		p2.x = xs - i;
		p2.y = ys + d - i;
		p2.z = zs;

		if(is_a_valid_discrete_final_position(p2))
			return true;

		p2.x = xs + d - i;
		p2.y = ys - i;
		
		if(is_a_valid_discrete_final_position(p2))
			return true;
	}
	
	return false;
}

// Aux Function: Check if exist a free position around the 'init' position 
// (around X, around Y and Z up) using the 'theta_' occupancy matrix. 
bool ThetaStar::free_initial_position_searcher_3d(float maxDistance)
{
	DiscretePosition init_ = discretizePosition(initial_position);; 			// Start coordinates
	
	int maxDistance_ = maxDistance / step;	 		// celdas mas lejanas a 1 metro
	
	// Check init point, first if is inside the workspace and second if is occupied	
	if(is_a_valid_discrete_initial_position(init_))
		return true;
		
	// Check from z=0 to d from the near xy ring to the far xy ring
	for (int d = 1; d<=maxDistance_; d++)
	{
		for(int z=init_.z; z<=init_.z+d; z++)
		{
			if(search_initial_position_in_xy_ring(init_.x, init_.y, z, d-(z-init_.z)))
				return true;
		}
	}
	
	return false;
}


// Check if exist a free position around the 'final' position 
// (around X, around Y and Z up) using the 'theta_' occupancy matrix.
bool ThetaStar::free_final_position_searcher_3d(float maxDistance)
{
	DiscretePosition final_ = discretizePosition(final_position);; 			// Start coordinates
	
	int maxDistance_ = maxDistance / step;	 					// celdas mas lejanas a compromabar a maxDistance meters
	
	// Check final point, first if is inside the workspace and second if is occupied	
	if(is_a_valid_discrete_final_position(final_))
		return true;
		
	// Check from z=0 to d from the near xy ring to the far xy ring
	for (int d = 1; d<=maxDistance_; d++)
	{
		for(int z=final_.z; z<=final_.z+d; z++)
		{
			if(search_final_position_in_xy_ring(final_.x, final_.y, z, d-(z-final_.z)))
				return true;
		}
	}
	
	return false;
}


// Check if exist a free position around the 'final' position 
// (around X, around Y) using the occupancy matrix.
bool ThetaStar::free_final_position_searcher_2d(float maxDistance)
{
	DiscretePosition final_ = discretizePosition(final_position);; 			// Start coordinates
	
	int maxDistance_ = maxDistance / step;	 								// celdas mas lejanas a compromabar a maxDistance meters
	
	// Check final point, first if is inside the workspace and second if is occupied	
	if(is_a_valid_discrete_final_position(final_))
		return true;
		
	// Check from z=0 from the near xy ring to the far xy ring
	for (int d = 1; d<=maxDistance_; d++)
	{
		if(search_final_position_in_xy_ring(final_.x, final_.y, final_.z, d))
			return true;
	}
	
	return false;
}

}
