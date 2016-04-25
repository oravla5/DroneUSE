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

AprilTagDetector::AprilTagDetector(char *imageTopic) : it_(nh_), MIN_FRAME_NUM_(3), MIN_TAG_DIST_(1000), TAG_SIZE_(0.155), SCALE_FACTOR_(0.5)
{
	//Subscription
	imgSub_ = it_.subscribeCamera("/dji_sdk/image_raw", 1, &AprilTagDetector::callback, this);

	//AprilTag position publication
	tagPub_ = nh_.advertise<PointStamped>("droneuse/tag_position", 10);

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

	cout << "ENTRA EN EL CALLBACK" << endl;
	//Ros image message to cv format
	cv::Mat frame_ori = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
	cv::Mat frame;

	cv::resize(frame_ori, frame, cv::Size(), SCALE_FACTOR_, SCALE_FACTOR_); 

	//cout << "FRAME ORI SIZE = " << frame_ori.size().height << " x " << frame_ori.size().width << endl;
	//cout << "FRAME  SIZE = " << frame.size().height << " x " << frame.size().width << endl;
	
	if(frame.empty())
		return;

	vector<TagDetection> tags_detected;

	timer t;
	tag_detector_->process(frame, tags_detected);
/*

	for(int i=0; i<tags_detected.size(); i++)
		if(tags_detected[i].good && tags_detected[i].id == 2)
			tags_detected[i].draw(frame);
*/
//	cv::imshow("Frame", frame);
//	cv::waitKey(1);

	//Publish tags

	for(int i=0; i<tags_detected.size(); i++)
	{
		if(tags_detected[i].good && tags_detected[i].id == 2)
		{
			cv::Matx13d tvec = tags_detected[i].getPosition( TAG_SIZE_, SCALE_FACTOR_*info_msg->K[0], SCALE_FACTOR_*info_msg->K[4], SCALE_FACTOR_*info_msg->K[2], SCALE_FACTOR_*info_msg->K[5]);

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

	//cout << "TIEMPO: " << t.elapsed() << endl;
	return;
}
