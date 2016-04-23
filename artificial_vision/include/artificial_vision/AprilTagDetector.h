#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>

#include <ros/ros.h>
#include "TagDetector.hpp"

#include <opencv2/opencv.hpp>

/*
struct trackedTag_{
	AprilTags::TagDetection *tag;
	int frames_num;
};
*/

class AprilTagDetector
{
	ros::NodeHandle				nh_;

	image_transport::ImageTransport		it_;
	image_transport::CameraSubscriber	imgSub_;

	ros::Publisher				tagPub_;

	cv::Ptr<april::tag::TagDetector>	tag_detector_;
	std::vector<cv::Ptr<april::tag::TagFamily> > gTagFamilies_;

	const int				MIN_FRAME_NUM_;
	const float				MIN_TAG_DIST_;
	const float				TAG_SIZE_;
	const float				SCALE_FACTOR_;

	public:
			AprilTagDetector(char *imageTopic);
			~AprilTagDetector();
		void	callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);	
};

#endif
