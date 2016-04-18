#include "AprilTagDetector.h"
#include "Tag16h5.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <geometry_msgs/PointStamped.h>

using namespace ros;
using namespace std;
using namespace sensor_msgs;
using namespace AprilTags;
using namespace geometry_msgs;

AprilTagDetector::AprilTagDetector(char *imageTopic, char *infoTopic) : it_(nh_)
{
	//Subscription
	imgSub_ = it_.subscribeCamera(imageTopic, 1, &AprilTagDetector::callback, this);

	//AprilTag position publication
	tagPub_ = nh_.advertise<PointStamped>("droneuse/tag_position", 1);

	//AprilTag detector initialization
	tag_detector_ = new TagDetector(tagCodes16h5);

	return;
}

AprilTagDetector::~AprilTagDetector()
{
	delete tag_detector_;
	return;
}

void AprilTagDetector::callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& info_msg)
{
	//Ros image message to cv format
	cv::Mat frame_original = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
    	cv::Mat frame;

    	cv::resize(frame_original, frame, cv::Size(), 0.5, 0.5);

	if(frame.empty())
		return;

	cam_model_.fromCameraInfo(info_msg);

	vector<TagDetection> tags_detected;
    	geometry_msgs::PointStamped tags_position;

	tags_detected = tag_detector_->extractTags(frame);

	Eigen::Matrix4d transform;

	for(int i=0; i<tags_detected.size(); i++)
	{
		if(tags_detected[i].good)
		{
			transform = tags_detected[i].getRelativeTransform( 0.155, info_msg->K[0], info_msg->K[4], info_msg->K[2], info_msg->K[5]);

            tags_detected[i].draw(frame);
			PointStamped tag_pos;
			tag_pos.header.stamp = image_msg->header.stamp;
			tag_pos.header.frame_id = "/camera";

			//Camera system tag coordinates to 3D world coordinates
			tag_pos.point.x = transform(2,3);
			tag_pos.point.y = -1 * transform(0,3);
			tag_pos.point.z = -1 * transform(1,3);
			
			tagPub_.publish(tag_pos);
		}
	}
    	cv::imshow("frame", frame);
	cv::waitKey(1);
	return; 
}
