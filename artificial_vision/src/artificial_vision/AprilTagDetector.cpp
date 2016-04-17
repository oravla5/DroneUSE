#include "AprilTagDetector.h"
#include "Tag16h5.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <geometry_msgs/PointStamped.h>

#include <math.h>

using namespace ros;
using namespace std;
using namespace sensor_msgs;
using namespace AprilTags;
using namespace geometry_msgs;

AprilTagDetector::AprilTagDetector() : it_(nh_), MIN_FRAME_NUM_(3), MIN_TAG_DIST_(1000)
{
	//Subscription
	imgSub_ = it_.subscribeCamera("/dji_sdk/image_raw", 1, &AprilTagDetector::callback, this);

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
	cv::Mat frame = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

	if(frame.empty())
		return;

	vector<TagDetection> tags_detected;
	tags_detected = tag_detector_->extractTags(frame);

	associateTags(tags_detected);

	//Publish tags
	Eigen::Matrix4d transform;

	for(int i=0; i<tracked_tags_.size(); i++)
	{
		if(tracked_tags_[i].frames_num >= MIN_FRAME_NUM_)
		{
			transform = tracked_tags_[i].getRelativeTransform( 0.155, info_msg->K[0], info_msg->K[4], info_msg->K[2], info_msg->K[5]);

			PointStamped tag_pos;
			tag_pos.header.stamp = ros::Time::now();

			//Camera system tag coordinates to 3D world coordinates
			tag_pos.point.x = transform(2,3);
			tag_pos.point.y = -1 * transform(0,3);
			tag_pos.point.z = -1 * transform(1,3);
			
			tagPub_.publish(tag_pos);
		}
	}
	return;
}

void AprilTagDetector::associateTags(const vector<TagDetection>& tags_detected)
{
	vector<trackedTag> old_tags = tracked_tags_;
	tracked_tags_.clear();

	for(int i=0; i< tags_detected.size() && tags_detected[i].good; i++)
	{
		for(int j=0; j< tracked_tags_.size(); j++)
			if(tags_detected[i].id == tracked_tags[j].tag->id)
			{
				if( getTagDistance(tags_detected[i],*(possible_tags[j].tag)) < MIN_TAG_DIST_)
				{
					tracked_tags[j].tag = &tags_detected[i];
					tracked_tags[j].frames_num++;
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
