#ifndef TAGMAP_DETECTOR_H
#define TAGMAP_DETECTOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>

#include "TagDetector.hpp"

#include <opencv2/opencv.hpp>
#include <cameraparameters.h>
#include <markermap.h>

#include <kalman_filter.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
/*
struct trackedTag_{
	AprilTags::TagDetection *tag;
	int frames_num;
};
*/

class tagMapDetector
{
	ros::NodeHandle					nh_;

	image_transport::ImageTransport			it_;
	image_transport::CameraSubscriber		imgSub_;

	ros::Publisher					tagMapPub_;
	ros::Publisher					orig_tagMapPub_;
	image_transport::Publisher                      imgPub_;

	cv::Ptr<april::tag::TagDetector>		tag_detector_;
	std::vector< cv::Ptr<april::tag::TagFamily> > 	gTagFamilies_;
    
    // MarkerMap object stores the Tag Map configuration: Tag position, id and family name. 
	aruco::MarkerMap                                markerMapCfg;

    // The file name where the Tag Map configuration is stored
	std::string                                     markerMapCfg_file; 
	aruco::CameraParameters                         cam_parameters;
	tf::TransformListener* 		tf_listener;
	tf::TransformBroadcaster 	br;
	tf::Quaternion			q;
	tf::Transform 			tf_camera_apriltagMap;
    
    // Class Parameters

	public:
			tagMapDetector(char *imageTopic, string marker_map_cfg_file);
			~tagMapDetector();
	void    imgSub_callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);	

private:
	float				SCALE_FACTOR;
	float 				SCALE_min;
	float 				SCALE_max;
	float 				DIST_min;
	float 				DIST_max;

	geometry_msgs::PointStamped	last_tagMap;
	ros::Time			last_det_time;
	float 				LPF_beta_x;
	float 				LPF_beta_y;
	float 				LPF_beta_z;
	bool 				first_detection = true;

};

#endif
