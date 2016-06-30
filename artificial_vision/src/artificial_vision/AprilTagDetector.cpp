#include "AprilTagDetector.h"

#include "TagDetector.hpp"
#include "TagFamilyFactory.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PointStamped.h>

#include <math.h>
#include <boost/timer.hpp>

using namespace ros;
using namespace std;
using namespace sensor_msgs;
using namespace april::tag;
using namespace geometry_msgs;
using namespace boost;

AprilTagDetector::AprilTagDetector(char *imageTopic) : it_(nh_), MIN_FRAME_NUM_(3), MIN_TAG_DIST_(1000), TAG_SIZE_(0.06), SCALE_FACTOR_(1.0)
{
	//Subscription
	imgSub_ = it_.subscribeCamera("/dji_sdk/image_raw", 1, &AprilTagDetector::callback, this);

	//AprilTag position publication
	//tagPub_ = nh_.advertise<PointStamped>("droneuse/tag_position", 1);
	tagPub_ = nh_.advertise<PointStamped>("droneuse/tag_position", 1);

	//AprilTag detector initialization
	TagFamilyFactory::create("0", gTagFamilies_);	
	tag_detector_ = new TagDetector(gTagFamilies_);

	return;
}

AprilTagDetector::~AprilTagDetector()
{
	return;
}

void AprilTagDetector::callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& info_msg)
{

	//Ros image message to cv format
	cv::Mat frame_ori = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
	cv::Mat frame_cropped;
	cv::Mat frame_scaled;
	cv::Mat frame;

	if(frame_ori.empty())
		return;

	int crop_width = frame_ori.size().width/1;
	int crop_height  = frame_ori.size().height/1;
	int crop_x = (frame_ori.size().width   - crop_width)/2; 
	int crop_y = (frame_ori.size().height  - crop_height)/2; 

	cv::Rect roiRect = cv::Rect(crop_x, crop_y, crop_width, crop_height); // ROI in source image

	frame_cropped = frame_ori(roiRect);
	cv::resize(frame_cropped, frame_scaled, cv::Size(), SCALE_FACTOR_, SCALE_FACTOR_); 
	frame = frame_scaled;

	vector<TagDetection> tags_detected;
	tag_detector_->process(frame, tags_detected);

	//Publish tags
	for(int i=0; i<tags_detected.size(); i++)
	{
		if(tags_detected[i].good && tags_detected[i].id == 2)
		{
			cv::Matx13d tvec = tags_detected[i].getPosition( TAG_SIZE_, 2.0*SCALE_FACTOR_*info_msg->K[0], 1.5*SCALE_FACTOR_*info_msg->K[4], 2.0*SCALE_FACTOR_*(info_msg->K[2]-(float)crop_x/2.0), 1.5*SCALE_FACTOR_*(info_msg->K[5]-(float)crop_y/1.5));

			PointStamped tag_pos;
			tag_pos.header.stamp = image_msg->header.stamp;
			tag_pos.header.frame_id = "/camera";

			//Camera system tag coordinates to 3D world coordinates
			tag_pos.point.x = tvec(0);
			tag_pos.point.y = tvec(1);
			tag_pos.point.z = tvec(2);
			
			tagPub_.publish(tag_pos);
		}
	}
	return;
}

//void AprilTagDetector::callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& info_msg)
//{
//
//	//Ros image message to cv format
//	cv::Mat frame_ori = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
//	cv::Mat frame_cropped;
//	cv::Mat frame_scaled;
//	cv::Mat frame;
//
//	if(frame_ori.empty())
//		return;
//
//	int crop_width = frame_ori.size().width/1;
//	int crop_height  = frame_ori.size().height/1;
//	int crop_x = (frame_ori.size().width   - crop_width)/2; 
//	int crop_y = (frame_ori.size().height  - crop_height)/2; 
//
//	cv::Rect roiRect = cv::Rect(crop_x, crop_y, crop_width, crop_height); // ROI in source image
//
//	frame_cropped = frame_ori(roiRect);
//	cv::resize(frame_cropped, frame_scaled, cv::Size(), SCALE_FACTOR_, SCALE_FACTOR_); 
//	frame = frame_scaled;
//
//	vector<TagDetection> tags_detected;
//	tag_detector_->process(frame, tags_detected);
//
//	//Publish tags
//	for(int i=0; i<tags_detected.size(); i++)
//	{
//		if(tags_detected[i].good && tags_detected[i].id == 2)
//		{
//            cv::Mat rvec;
//            cv::Mat tvec;
//
//			tags_detected[i].getPose( TAG_SIZE_, 2.0*SCALE_FACTOR_*info_msg->K[0], 1.5*SCALE_FACTOR_*info_msg->K[4], 2.0*SCALE_FACTOR_*(info_msg->K[2]-(float)crop_x/2.0), 1.5*SCALE_FACTOR_*(info_msg->K[5]-(float)crop_y/1.5), tvec, rvec);
//
//            PoseStamped tag_pose;
//			
//			tag_pose.header.stamp = image_msg->header.stamp;
//			tag_pose.header.frame_id = "/camera";
//
//			//Camera system tag coordinates to 3D world coordinates
//			tag_pose.pose.position.x = tvec.at<double>(0);
//			tag_pose.pose.position.y = tvec.at<double>(1);
//			tag_pose.pose.position.z = tvec.at<double>(2);
//
//            tf::Vector3 tag_position(tag_pose.pose.position.x,tag_pose.pose.position.y,tag_pose.pose.position.z);
//			
//            cv::Matx33d rot_mat;
//            cv::Rodrigues(rvec, rot_mat);
//
//            tf::Matrix3x3 rot_mat_tf(   rot_mat(0,0),rot_mat(0,1),rot_mat(0,2),
//                                        rot_mat(1,0),rot_mat(1,1),rot_mat(1,2),
//                                        rot_mat(2,0),rot_mat(2,1),rot_mat(2,2));
//            tf::Quaternion q_rot;
//            rot_mat_tf.getRotation(q_rot);
//            tag_pose.pose.orientation.x = q_rot.x();
//            tag_pose.pose.orientation.y = q_rot.y();
//            tag_pose.pose.orientation.z = q_rot.z();
//            tag_pose.pose.orientation.w = q_rot.w();
//			tagPub_.publish(tag_pose);
//		}
//	}
//	return;
//}
