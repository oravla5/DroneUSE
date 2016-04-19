#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>

#include <ros/ros.h>
#include "TagDetector.h"

struct trackedTag_{
	AprilTags::TagDetection *tag;
	int consecutive_detections;
};

class AprilTagDetector
{
	ros::NodeHandle				nh_;

	image_transport::ImageTransport		it_;
	image_transport::CameraSubscriber	imgSub_;

	ros::Publisher				tagPub_;

	AprilTags::TagDetector			*tag_detector_;
	std::vector<trackedTag_>		tracked_tags_;	

	const int				MIN_FRAME_NUM_;
	const int				MIN_CONSECUTIVE_DETECTIONS_;
	const float				MIN_TAG_DIST_;
	const float				TAG_SIZE_;
	const float				SCALE_FACTOR_;

	void	updateTrackedTags(std::vector<AprilTags::TagDetection> &tags_detected);
	float	getTagDistance(const AprilTags::TagDetection&, const AprilTags::TagDetection&);

	public:
			AprilTagDetector(char *imageTopic);
			~AprilTagDetector();
		void	callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);	
};

#endif
