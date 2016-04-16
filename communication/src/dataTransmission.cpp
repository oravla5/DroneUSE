#include "DataTransmission.h"

#include <ros/ros.h>

using namespace ros;
using namespace std;


DataTransmission::DataTransmission()
{
	//Subscription
	imgSub_.subscribe(nh_, imageTopic, 20);
	cam_infoSub_.subscribe(nh_infoTopic, 20);

	//AprilTag position publication
	tagPub_ = nh_.advertise<tipo>("tag_position", 1);

	//AprilTag detector initialization
	tag_detector = new TagDetector(tagCodes16h5);

	return;
}

AprilTagDetector::~AprilTagDetector()
{
	delete tag_detector;
	return;
}

void AprilTagDetector::callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& info_msg)
{
	//Ros image message to cv format
	cv::Mat image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

	cam_model_.fromCameraInfo(info_msg);

	return 0;
}
