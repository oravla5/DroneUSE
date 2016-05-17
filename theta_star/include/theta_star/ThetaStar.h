/*
 * Copyright 2015 Ricardo Ragel de la Torre, GRVC, Univ. of Seville, Spain
 *
 * Resume: Lazy Theta Star with Optimization Class declarations
 * 
 */

#ifndef THETASTAR_H_
#define THETASTAR_H_

#include <vector>
#include <set>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

#include <octomap_msgs/Octomap.h>//Octomap Binary
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#define TOLERANCE 0.2 	// tolerance in meters to path-->trajectory

#define PRINTF_REGULAR  "\x1B[0m"
#define PRINTF_RED  	"\x1B[31m"
#define PRINTF_GREEN  	"\x1B[32m"
#define PRINTF_YELLOW  	"\x1B[33m"
#define PRINTF_BLUE  	"\x1B[34m"
#define PRINTF_MAGENTA  "\x1B[35m"
#define PRINTF_CYAN  	"\x1B[36m"
#define PRINTF_WHITE	"\x1B[37m"


//using namespace Eigen;
//using namespace arm_navigation_msgs;
using namespace geometry_msgs;
using namespace std;

namespace PathPlanners {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class DiscretePosition
{
public:
   int x,y,z;
};

// Nodos (Ver mas tarde)
class ThetaStarNode;

// Union entre nodos (Contiene el nodo al que une(que a su vez este contiene a su parent) y si esta "ocupado" y en que listas esta)
class ThetaStartNodeLink
{
public:
   ThetaStartNodeLink():
      node(NULL),
      isInOpenList(false),
      isInCandidateList(false),
      notOccupied(true),			//! Modified from occupied(false). This change (10/03/16) is to use the fast inflation version, that using memset() set "all" memory to zero
      lastTimeSeen(100)
   {

   }

   ThetaStarNode *node;
   bool isInOpenList;
   bool isInCandidateList;
   bool notOccupied;
   unsigned char lastTimeSeen; ///< To Calculate trajectory occupation
};

// Nodos (Contiene la posicion discreta dentro de la matriz de ocupacion, su parent y el NodeLink a este ultimo)
class ThetaStarNode{
public:
   ThetaStarNode():
      parentNode(NULL),
      nodeInWorld(NULL),
      lineDistanceToFinalPoint( std::numeric_limits<float>::max()),
      distanceFromInitialPoint( std::numeric_limits<float>::max()),
      totalDistance(std::numeric_limits<float>::max())
   {

   }

   DiscretePosition    point;
   ThetaStarNode      *parentNode;
   ThetaStartNodeLink *nodeInWorld;
   float lineDistanceToFinalPoint;
   float distanceFromInitialPoint;
   float totalDistance;

   /*!
    * \brief operator <
    * \param lhs
    * \param rhs
    * \return
    */
   friend bool operator !=(const ThetaStarNode& lhs, const ThetaStarNode& rhs){
      return lhs.point.x!=rhs.point.x ||
            lhs.point.y!=rhs.point.y ||
            lhs.point.z!=rhs.point.z;
   }
};


struct NodePointerComparator
{
   bool operator()(const ThetaStarNode* const& lhs__, const ThetaStarNode* const& rhs__) const{
      const ThetaStarNode *lhs = lhs__;
      const ThetaStarNode *rhs = rhs__;
      float res = 0;

      res = lhs->totalDistance - rhs->totalDistance;
      if(res==0)
      {
         res = lhs->point.x - rhs->point.x;
      }
      if(res==0)
      {
         res = lhs->point.y - rhs->point.y;
      }
      if(res==0)
      {
         res = lhs->point.z - rhs->point.z;
      }

      if(res==0)
      {
         res = lhs->parentNode->point.x - rhs->parentNode->point.x;
      }
      if(res==0)
      {
         res = lhs->parentNode->point.y - rhs->parentNode->point.y;
      }
      if(res==0)
      {
         res = lhs->parentNode->point.z - rhs->parentNode->point.z;
      }

      return res < 0;
   }
};


class ThetaStar {
public:
   /**
      Default constructor
      * 
      * @param simetric workspace 
      * @param occupancy matrix resolution
      * @param occupancy matrix nodes inflation
      * @param h(s) weight -- Lazy Theta* with Optimization
      * @param NodeHandle 
      * 
   */
   ThetaStar(char* plannerName, char* planner_frame_id, float x_max, float y_max, float z_max, float x_min, float y_min, float z_min, float step, float h_real_inflation, float v_real_inflation, float h_safe_inflation, float v_safe_inflation, float initial_point_factor_, float z_weight_cost_, ros::NodeHandle *n);

   /**
      Default destructor
   */
   virtual ~ThetaStar();

   /**
      Init the auxiliar matrix if you will to use updateMap using that
   */
   void initAuxDiscreteMatrix(float x_max, float y_max, float z_max, float x_min, float y_min, float z_min, unsigned char max_point_life);


   /**
      Override actual collision map.

      @param new collision map.
      @param xyz offsets from /map to /world or the required reference frame 
   */
   void updateMap(octomap_msgs::Octomap message, float x_offset, float y_offset, float z_offset);


   //! OVERLOAD: PointCloud instead of octomap
   
   void updateMap(PointCloud cloud);


   /// OVERLOAD: Update the discrete node matrix (occupancy) from a point cloud that is not in the theta_star reference system (for example PC at global and theta_star at local)
   /// and we need to buffer this point cloud same_time
   void updateMap(PointCloud cloud, tf::Transform &transf);

   //! CLEAR occupancy discrete matrix
   void clearMap();

   /**
      Inflate occupied cells filling all cells around
   */
   inline void inflateNodeAsSolidCube(int &x_, int &y_, int &z_);


   /**
      Inflate occupied cells filling only external cells around
   */
   inline void inflateNodeAsEmptyCube(int &x_, int &y_, int &z_);

   /**
      Inflate occupied cells filling all cells around. This is a fast version using memset() to set to zero all discrete occupation matrix that have to be inflated
      
      discrete_world is a 'ThetaStartNodeLink' vector whose all members must be zero initially. Respect old version we change ThetaStartNodeLink::occupied to ThetaStartNodeLink::notOccupied,
      then notOccupied is set to zero at inflation, so "is occupied", and all members set to 0 => false, that is the initial correct value.
   */
   inline void fastInflateNodeAsSolidCube	(int &x_, int &y_, int &z_);
   inline void fastInflateAuxNodeAsSolidCube(int &x_, int &y_, int &z_);	// Auxiliar Occupancy Matrix version

	/**
	 * Fill Occupancy Matrix holes by the XY unitary inflation
	 **/
	void fillAroundHoles(int &x_, int &y_, int &z_);

   /**
      Returns current collision map.

      @return current collision map.
   */
   octomap::OcTree * getMap();

   /**
      Set initial position of the path that we want to calculate.

      @param 3D position data
   */
   bool    setDiscreteInitialPosition(DiscretePosition p_);
   bool    setInitialPosition(Vector3 p);

   /**
      Returns current initial position.
   */
   Vector3 getInitialPosition();

   /**
         Set final position of the path that we want to calculate.

         @param 3D position data
      */
   bool setDiscreteFinalPosition(DiscretePosition p_);
   bool setFinalPosition(Vector3 p);

   /**
         Returns current initial position.
   */
   Vector3 getFinalPosition();

   /**
            Set timeout

            @param Natural number with time in seconds
   */
   void setTimeOut(int sec);

   /**
        Returns current timeout in seconds.
   */
   int  getTimeOut();

   /**
    Publish via topic the discrete map constructed
   */
   void publishOccupationMarkersMap();
   void publishAuxOccupationMarkersMap();
   
   /**
        Try to calculate a new path in the wanted timeout.
		
        @return number of points in the path.
   */
   int  calculateNewPath(void);

   /**
    Get the current Path as vector [Xi Yi Zi]   
   */
   vector<Vector3> getCurrentPath();
   
   /**
    Get the current Trajectory as trajectory_msgs::MultiDOFJointTrajectory [Xi, Yi, Zi, Yawi, ti]. 
    Get trajectory from the thetaStar current path solution. First Implementation: Yaw constant along the entire trajectory
   */
   bool getCurrentTrajectory_YawCte(trajectory_msgs::MultiDOFJointTrajectoryPtr &trajectory, geometry_msgs::Transform intial_pose, float DPmax_h, float DPmax_v, float Vm_h, float Vm_v, float Vm_h_1, float Vm_v_1);
   
   /**
    Get the current Trajectory as trajectory_msgs::MultiDOFJointTrajectory [Xi, Yi, Zi, Yawi, ti]. 
    Get trajectory from the thetaStar current path solution. Second Implementation: Yaw ahead from Wp[k] to Wp[k+1] as reference from Wp[k] to Wp[k+1]
   */
   bool getCurrentTrajectory_YawAtTime(trajectory_msgs::MultiDOFJointTrajectoryPtr &trajectory, geometry_msgs::Transform intial_pose, float DPmax_h, float DPmax_v, float Vm_h, float Vm_v, float Vm_h_1, float Vm_v_1, float Wy_est);

   /**
    Get the current Trajectory as trajectory_msgs::MultiDOFJointTrajectory [Xi, Yi, Zi, Yawi, ti]. 
    Get trajectory from the thetaStar current path solution. Third Implementation: Yaw ahead from Wp[k] to Wp[k+1] as reference from Wp[k-1] to Wp[k]
   */
   bool getCurrentTrajectory_YawInAdvance(trajectory_msgs::MultiDOFJointTrajectoryPtr &trajectory, geometry_msgs::Transform intial_pose, float DPmax_h, float DPmax_v, float Vm_h, float Vm_v, float Vm_h_1, float Vm_v_1, float Wy_est);

   /**
    Same as 'getCurrentTrajectory_YawInAdvance()' but setting a pre-computed yaw reference for the last waypoint
   */
   bool getCurrentTrajectory_YawInAdvance_WithFinalYaw(trajectory_msgs::MultiDOFJointTrajectoryPtr &trajectory, geometry_msgs::Transform intial_pose, float DPmax_h, float DPmax_v, float Vm_h, float Vm_v, float Vm_h_1, float Vm_v_1, float Wy_est, double final_yaw_ref);
      
   
   /**	CONVERTED TO PUBLIC
           Returns if exists line of sight between two points.

           @param Position of first one point
           @param Position of second one point
           @return true if exists line of sight, false in the other case.
   */
   virtual bool lineofsight(ThetaStarNode &p1, ThetaStarNode &p2);
   
   
   /** 
		Public function to know if initial position is occupied previously to calculate a new path
   */
   virtual bool isInitialPositionOccupied();
   virtual bool isFinalPositionOccupied();
   
   float getMapResolution();
   
   DiscretePosition discretizePosition(Vector3 p);

	/** 
		Check if exist a free position around the 'init' position 
		(around X, around Y and Z up) using the 'theta_' occupancy matrix. 
	**/
	bool free_initial_position_searcher_3d(float maxDistance);

	/**
		Check if exist a free position around the 'final' position 
		(around X, around Y and Z up) using the 'theta_' occupancy matrix.
	**/
	bool free_final_position_searcher_3d(float maxDistance);
	bool free_final_position_searcher_2d(float maxDistance);


   virtual bool isOccupied(ThetaStarNode n);


private:

   /**
   */
   void getNeighbors(ThetaStarNode &node,  set<ThetaStarNode*,NodePointerComparator> &neighbors);

   /**
                 Returns distance to goal.
                 @param node to calculate distance from.
                 @return distance to goal.
   */

   void publishRvizPoint(ThetaStarNode &s, bool publish);

   float distanceToGoal(ThetaStarNode node);
   float weightedDistanceToGoal(ThetaStarNode node);
   
   float distanceBetween2nodes(ThetaStarNode &n1,ThetaStarNode &n2);
   float weightedDistanceBetween2nodes(ThetaStarNode &n1,ThetaStarNode &n2);

   float distanceFromInitialPoint(ThetaStarNode node, ThetaStarNode parent);
   float weightedDistanceFromInitialPoint(ThetaStarNode node, ThetaStarNode parent);

	// Get discrete matrix index for this discrete (x,y,z) position
    inline unsigned int getWorldIndex(int &x, int &y, int &z)
	{
		return (unsigned int)((x - ws_x_min_inflate) + (Lx)*((y - ws_y_min_inflate) + (Ly)*(z - ws_z_min_inflate)));
	}

    inline unsigned int getAuxWorldIndex(int &x,int &y, int &z)
	{
		return (unsigned int)((x - ws_x_min_aux_inflate) + (Lx_aux)*((y - ws_y_min_aux_inflate) + (Ly_aux)*(z - ws_z_min_aux_inflate)));
	}

	// Get discrete (x,y,z) position from the discrete matrix index
	inline void getDiscreteWorldPositionFromIndex(int &x, int &y, int &z, int index)
	{
		// Discrete matrix = [(ALL_X_SEGMENT)(ALL_X_SEGMENT)...(ALL_X_SEGMENT) | (ALL_X_SEGMENT)(ALL_X_SEGMENT)...(ALL_X_SEGMENT)| ............. |(ALL_X_SEGMENT)(ALL_X_SEGMENT)...(ALL_X_SEGMENT)]
		//		for			   Y=WS_Y_MIN	  Y=WS_Y_MIN+1  ... Y=WS_Y_MAX     |  Y=WS_Y_MIN	  Y=WS_Y_MIN+1  ... Y=WS_Y_MAX   | ............. | Y=WS_Y_MIN	  Y=WS_Y_MIN+1  ... Y=WS_Y_MAX    
		//		for			   				  Z = WS_Z_MIN 					   |                  Z = WS_Z_MIN+1				 | ............. |            Z = WS_Z_MAX = WS_Z_MIN+Lz      
		
		int z_n_segmts = floor( index * Lx_inv * Ly_inv );					// Index z segment (0 to Lz)
		int y_n_segmts = floor( (index - z_n_segmts * Lx * Ly) * Ly_inv );	// Index y segment (0 to Ly)
		
		x = ws_x_min_inflate + (index - z_n_segmts*Lx*Ly - y_n_segmts*Lx);
		y = ws_y_min_inflate + y_n_segmts;
		z = ws_z_min_inflate + z_n_segmts;
	}

	inline void getDiscreteAuxWorldPositionFromIndex(int &x, int &y, int &z, int index)
	{
		int z_n_segmts = (int)( index * Lx_inv_aux * Ly_inv_aux );							// Index z segment (0 to Lz)
		int y_n_segmts = (int)( (index- z_n_segmts * Lx_aux * Ly_aux) * Ly_inv_aux );		// Index y segment (0 to Ly)
		
		x = ws_x_min_aux + (index - z_n_segmts*Lx_aux*Ly_aux - y_n_segmts*Lx_aux);
		y = ws_y_min_aux + y_n_segmts;
		z = ws_z_min_aux + z_n_segmts;
	}
	

   inline bool isInside(ThetaStarNode n);
   inline bool isInside(int &x, int &y, int &z);
   inline bool isInsideAux(int &x, int &y, int &z);


   //Reimplementacion
   void ComputeCost(ThetaStarNode &s, ThetaStarNode &s2);
   void UpdateVertex(ThetaStarNode &s, ThetaStarNode &s2);
   void SetVertex(ThetaStarNode &s,set<ThetaStarNode*,NodePointerComparator> &neighbors);
   double g(ThetaStarNode &s);


	// Aux Functions to search intial and final free positions
		// Set and check if a position is a valid initial position. 
		// Return false is this position is outside the workspace or is occupied 
	inline bool is_a_valid_discrete_initial_position(DiscretePosition p);

		// Set and check if a position is a valid final position. 
		// Return false is this position is outside the workspace or is occupied 
	inline bool is_a_valid_discrete_final_position(DiscretePosition p);

		// Set and search a valid initial position in a horizontal ring 
		// centered in '(xs,ys,zs)' and radius 'd'
	inline bool search_initial_position_in_xy_ring(int xs, int ys, int zs, int d);

		// Set and search a valid final position in a horizontal ring 
		// centered in '(xs,ys,zs)' and radius 'd'
	inline bool search_final_position_in_xy_ring(int xs, int ys, int zs, int d);

   // discrete_world: Matriz discreta de nodos. El tamaño se define al llamar al contructor segun las XYZ max y el step (resolucion) que le pasen
   std::vector<ThetaStartNodeLink> discrete_world;
   int matrix_size;
   int matrix_half_size;
   int ws_x_max, ws_y_max, ws_z_max;
   int ws_x_min, ws_y_min, ws_z_min;
   int ws_x_max_inflate, ws_y_max_inflate, ws_z_max_inflate;
   int ws_x_min_inflate, ws_y_min_inflate, ws_z_min_inflate;   
   int Lx, Ly, Lz;
   float Lx_inv, Ly_inv, Lz_inv;
   float step;
   float step_inv;

   // discrete_world: Matriz discreta de nodos. El tamaño se define al llamar al contructor segun las XYZ max y el step (resolucion) que le pasen
   std::vector<ThetaStartNodeLink> aux_discrete_world;
   int aux_matrix_size;
   unsigned char max_point_life;
   int ws_x_max_aux, ws_y_max_aux, ws_z_max_aux;
   int ws_x_min_aux, ws_y_min_aux, ws_z_min_aux;
   int ws_x_max_aux_inflate, ws_y_max_aux_inflate, ws_z_max_aux_inflate;
   int ws_x_min_aux_inflate, ws_y_min_aux_inflate, ws_z_min_aux_inflate; 
   int Lx_aux, Ly_aux, Lz_aux;
   float Lx_inv_aux, Ly_inv_aux, Lz_inv_aux;
   
   // open and candidates: Listas ordenadas ("std::set") de menor a mayor distancia total ("ThetaStarNode::totalDistance")
   set<ThetaStarNode*, NodePointerComparator> open,candidates;
   
   // Objeto donde se guarda toda la informacion del OCTOMAP que debe usar
   octomap::OcTree *m;
   
   Vector3 initial_position, final_position;
   ThetaStarNode *disc_initial, *disc_final;
   int timeout;
   vector<Vector3> last_path;

   //Debug
   visualization_msgs::Marker marker;
   ros::Publisher marker_pub_;
   visualization_msgs::Marker occupancy_marker;
   ros::Publisher occupancy_marker_pub_;
   visualization_msgs::Marker aux_occupancy_marker;
   ros::Publisher aux_occupancy_marker_pub_;
   ros::NodeHandle *n_debug;
   
   visualization_msgs::Marker marker_no_los;
   ros::Publisher no_los_marker_pub_;
   
   // INFLATION NODE
   float h_real_inflation;  // for UAV wingspan[m] -- horizontal and vertical separated
   float v_real_inflation;
   float h_safe_inflation;  // for safe distance[m] -- horizontal and vertical separated
   float v_safe_inflation;
   float h_inflation;  // TOTAL [m]
   float v_inflation;
   int h_inflation_; // TOTAL [discrete]
   int v_inflation_;
   
   // Reduce weight of 'distance from initial position cost': C(s) = initial_point_factor * g(s) + h(s) 
   // (inverse of the h(s) weight: C(s) = g(s) + w*h(s) -- Lazy Theta* with Optimization)
   float initial_point_factor;
   
   // Weight for height changes
   float z_weight_cost; 
};

} /* namespace PathPlanners */
#endif /* THETASTAR_H_ */
