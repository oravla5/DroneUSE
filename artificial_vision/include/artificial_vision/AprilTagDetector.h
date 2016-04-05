#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <ros/ros.h>
#include "TagDetector.h"

class AprilTagDetector
{
	ros::NodeHandle				nh_;
	
	image_geometry::PinholeCameraModel	cam_model_;
	image_transport::ImageTransport		it_;
	image_transport::Subscriber		sub_;

	tf::TransformListener			*tf_listener_;

	ros::Publisher				aprilTagPug_;
	
	public:
			AprilTagDetector(char *imageTopic, char* infoTopic, char *aprilTag);
		void	callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);	

}

#endif
