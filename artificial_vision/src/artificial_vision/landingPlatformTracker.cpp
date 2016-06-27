#include "landingPlatformTracker.h"

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
using namespace geometry_msgs;
using namespace boost;

landingPlatformTracker::landingPlatformTracker(char *imageTopic) : it_(nh_), newMeasure(false), tracking_on(false), MAX_ELAPSED_TIME_(2)
{
    imgSub_ = it_.subscribeCamera("/dji_sdk/image_raw", 1, &landingPlatformTracker::imgSub_callback, this);
    tagMapPosSub_ = nh_.subscribe("/droneuse/tagMap_position", 1, &landingPlatformTracker::tagMapPosSub_callback, this);

    positionPub_    = nh_.advertise<PointStamped>("droneuse/landing_platform_position", 1);
    imgPub_         = it_.advertise("droneuse/landingPlatform_videoFootage", 1);

    position_kf     = KalmanFilter();
    last_detection_time = ros::Time::now() - MAX_ELAPSED_TIME_;
    

    return;
}

landingPlatformTracker::~landingPlatformTracker()
{
	return;
}


void landingPlatformTracker::imgSub_callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    if(cam_parameters.CameraMatrix.empty() || cam_parameters.Distorsion.empty())
    {
        float cameraMatrix_components[9] = {float(info_msg->K[0]), float(info_msg->K[1]), float(info_msg->K[2]),
                                            float(info_msg->K[3]), float(info_msg->K[4]), float(info_msg->K[5]), 
                                            float(info_msg->K[6]), float(info_msg->K[7]), float(info_msg->K[8])}; 

        float cameraDistor_components[9] = {float(info_msg->D[0]),float(info_msg->D[1]),float(info_msg->D[2]),float(info_msg->D[3])};

        cv::Mat cameraMatrix(3,3,CV_32F, cameraMatrix_components);
        cv::Mat distorsion(1,4,CV_32F, cameraDistor_components);
        cv::Size imgSize(info_msg->height, info_msg->width);
        cam_parameters.setParams(cameraMatrix, distorsion, imgSize);
    }

	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
	cv::Mat frame = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
    if(tracking_on)
        frame = draw_tagMap(frame);

    cv_bridge::CvImage send (cv_ptr->header, cv_ptr->encoding, frame);
    imgPub_.publish(send.toImageMsg());
}


void landingPlatformTracker::tagMapPosSub_callback(const geometry_msgs::PointStamped& tagMap_pos)
{
    detected_position.x  = tagMap_pos.point.x;    
    detected_position.y  = tagMap_pos.point.y;    
    detected_position.z  = tagMap_pos.point.z;    

    last_detection_time = tagMap_pos.header.stamp;
    newMeasure          = true;
}


void landingPlatformTracker::landingPlatformPosUpdate(double dt)
{
    if(newMeasure)
    {
        if(!tracking_on)
        {
            // Kalman Filter Initialisation
            position_kf.init(detected_position.x, detected_position.y, detected_position.z, 0.0, 0.0, 0.0, last_detection_time);
            estimated_position.x = position_kf.x(0,0);
            estimated_position.y = position_kf.x(1,0);
            estimated_position.z = position_kf.x(2,0);
            tracking_on = true;
        }
        else
        {
            // Kalman Filter Updating
            position_kf.update(detected_position.x, detected_position.y, detected_position.z, last_detection_time);
            estimated_position.x = position_kf.x(0,0);
            estimated_position.y = position_kf.x(1,0);
            estimated_position.z = position_kf.x(2,0);
        }

        newMeasure = false;
        // Tag Map Position Publishing
        geometry_msgs::PointStamped tagMap_pos;
        tagMap_pos.header.stamp = ros::Time::now();
        tagMap_pos.header.frame_id = "/camera";
        tagMap_pos.point.x = estimated_position.x;
        tagMap_pos.point.y = estimated_position.y;
        tagMap_pos.point.z = estimated_position.z;
        positionPub_.publish(tagMap_pos);
        
    }
    else if( ros::Duration( ros::Time::now() - last_detection_time ) < MAX_ELAPSED_TIME_ )
    {
        // Kalman Filter Update
        position_kf.predict(dt);
        estimated_position.x = position_kf.x(0,0);
        estimated_position.y = position_kf.x(1,0);
        estimated_position.z = position_kf.x(2,0);
        
        // Tag Map Position Publishing
        geometry_msgs::PointStamped tagMap_pos;
        tagMap_pos.header.stamp = ros::Time::now();
        tagMap_pos.header.frame_id = "/camera";
        tagMap_pos.point.x = estimated_position.x;
        tagMap_pos.point.y = estimated_position.y;
        tagMap_pos.point.z = estimated_position.z;
        positionPub_.publish(tagMap_pos);
    }
    else
        tracking_on = false;

}

cv::Mat landingPlatformTracker::draw_tagMap(cv::Mat frame)
{
    // Board Center Display
    std::vector<cv::Point3f> world_points;
    world_points.push_back(cv::Point3f(0.0, 0.0, 0.0));
    cv::Mat tvec(3,1, CV_32FC1);
    tvec.at<float>(0,0) = estimated_position.x;
    tvec.at<float>(1,0) = estimated_position.y;
    tvec.at<float>(2,0) = estimated_position.z;
    cv::Mat rvec(3,1, CV_32FC1);
    rvec.at<float>(0,0) = 0.0;
    rvec.at<float>(1,0) = 0.0;
    rvec.at<float>(2,0) = 0.0;
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(world_points, rvec, tvec, cam_parameters.CameraMatrix, cam_parameters.Distorsion, image_points);
    //line(frame, image_points[0], image_points[1], cv::Scalar( 0, 0, 255 ), 3);
    //line(frame, image_points[1], image_points[2], cv::Scalar( 0, 0, 255 ), 3);
    //line(frame, image_points[2], image_points[3], cv::Scalar( 0, 0, 250 ), 3);
    //line(frame, image_points[3], image_points[0], cv::Scalar( 0, 0, 250 ), 3);
    circle(frame, image_points[0], 20, cv::Scalar( 0, 0, 255 ), 4);
    return frame;


}
