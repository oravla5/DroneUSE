#include "AprilTagDetector.h"
#include "Tag16h5.h"

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
using namespace AprilTags;
using namespace geometry_msgs;
using namespace boost;

AprilTagDetector::AprilTagDetector(char *imageTopic) : it_(nh_), MIN_FRAME_NUM_(3), MIN_TAG_DIST_(1000), TAG_SIZE_(0.155), SCALE_FACTOR_(0.5)
{
	//Subscription
	imgSub_ = it_.subscribeCamera("/dji_sdk/image_raw", 20, &AprilTagDetector::callback, this);

	//AprilTag position publication
	tagPub_ = nh_.advertise<PointStamped>("droneuse/tag_position", 60);

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
	cv::Mat frame_ori = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
	cv::Mat frame;

	cv::resize(frame_ori, frame, cv::Size(), SCALE_FACTOR_, SCALE_FACTOR_); 

	cout << "FRAME ORI SIZE = " << frame_ori.size().height << " x " << frame_ori.size().width << endl;
	cout << "FRAME  SIZE = " << frame.size().height << " x " << frame.size().width << endl;
	
	if(frame.empty())
		return;

	vector<TagDetection> tags_detected;

	timer t;
	tags_detected = tag_detector_->extractTags(frame);

	cout << "TIEMPO: " << t.elapsed() << endl;
	//associateTags(tags_detected);

	//Publish tags
	Eigen::Matrix4d transform;

	for(int i=0; i<tags_detected.size(); i++)
	{
		if(tags_detected[i].good)
		{
			transform = tags_detected[i].getRelativeTransform( TAG_SIZE_, SCALE_FACTOR_*info_msg->K[0], SCALE_FACTOR_*info_msg->K[4], SCALE_FACTOR_*info_msg->K[2], SCALE_FACTOR_*info_msg->K[5]);

			PointStamped tag_pos;
			tag_pos.header.stamp = image_msg->header.stamp;
			tag_pos.header.frame_id = "/camera";

			tag_pos.point.x = transform(0,3);
			tag_pos.point.y = transform(1,3);
			tag_pos.point.z = transform(2,3);
			
			tags_detected[i].draw(frame);
			tagPub_.publish(tag_pos);
		}
	}

	cv::imshow("Frame", frame);
	cv::waitKey(1);
	return;
}

void AprilTagDetector::associateTags(vector<TagDetection>& tags_detected)
{
	vector<trackedTag_> old_tags = tracked_tags_;
	tracked_tags_.clear();

	for(int i=0; i< tags_detected.size() && tags_detected[i].good; i++)
	{
		for(int j=0; j< tracked_tags_.size(); j++)
			if(tags_detected[i].id == tracked_tags_[j].tag->id)
			{
				if( getTagDistance(tags_detected[i],*(tracked_tags_[j].tag)) < MIN_TAG_DIST_)
				{
					tracked_tags_[j].tag = &tags_detected[i];
					tracked_tags_[j].frames_num++;
					break;
				}
			}	
	}
	return;
}

float AprilTagDetector::getTagDistance(const TagDetection& tag1,const TagDetection& tag2)
{
	float r1 = tag2.cxy.first - tag1.cxy.first;
	float r2 = tag2.cxy.second - tag2.cxy.second; 

	return sqrt(r1*r1 + r2*r2); 
}
