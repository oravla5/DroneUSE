#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/LaserScan.h> //obstacle distance
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <math.h>

#define BASELINE 0.15 //meters
#define MAX_DISTANCE 1.0 //meters
#define MAP_RESOLUTION 0.10 //meters 
#define C_PI (double) 3.141592653589793
#define WIDTH_MAX_ANGLE (30*C_PI)/180  
#define HEIGHT_MAX_ANGLE (28*C_PI)/180  

using namespace message_filters;
using namespace ros;
using namespace sensor_msgs;
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::PointCloud2> MySincPolicy;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


ros::Publisher obstacle_pointcloud_pub;

void callback(const sensor_msgs::LaserScanConstPtr& obstacles_distance, const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{ 
	pcl::PointXYZ point;
	PointCloud cloud_wall;
	PointCloud cloud_obstacles;

	double wall_width, wall_height; //Half width and height wall distance in meters
	double obst_dist = obstacles_distance->ranges[1];

	pcl_conversions::toPCL(point_cloud->header, cloud_wall.header);
	pcl_conversions::toPCL(point_cloud->header, cloud_obstacles.header);

	if(obst_dist < MAX_DISTANCE)
	{
		wall_width = BASELINE/2 + obst_dist*tan(WIDTH_MAX_ANGLE); 
		wall_height =  obst_dist*tan(HEIGHT_MAX_ANGLE); 
		
		for(int i = -wall_width/MAP_RESOLUTION; i <= wall_width/MAP_RESOLUTION; i++)	
		{
			for(int j = -wall_height/MAP_RESOLUTION; j <= wall_height/MAP_RESOLUTION; j++)	
			{
				point.x = i*MAP_RESOLUTION;
				point.y = j*MAP_RESOLUTION;
				point.z = obst_dist;
				cloud_wall.push_back(point);
			}
		}
	}
	pcl::fromROSMsg(*point_cloud, cloud_obstacles);
	//cloud_obstacles += cloud_wall;

	obstacle_pointcloud_pub.publish(cloud_obstacles);
	return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObstacleNode");
    ros::NodeHandle my_node;
    message_filters::Subscriber<sensor_msgs::LaserScan> obstacle_distance_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
    message_filters::Synchronizer<MySincPolicy>* sync_;

    obstacle_distance_sub.subscribe(my_node,"/guidance/obstacle_distance", 10);
    pointcloud_sub.subscribe(my_node,"/points", 10);

    sync_ = new Synchronizer<MySincPolicy>(MySincPolicy(10), obstacle_distance_sub, pointcloud_sub);
    sync_->registerCallback(boost::bind(&callback, _1, _2));

    obstacle_pointcloud_pub = my_node.advertise<PointCloud2>("/droneuse/obstacles", 1);

    while (ros::ok())
        ros::spinOnce();

 //   delete sync_;

    return 0;
}
