#include "landingPlatformTracker.h"

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <math.h>
#include <boost/timer.hpp>

using namespace ros;
using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace boost;

landingPlatformTracker::landingPlatformTracker(char *imageTopic) : newMeasure(false), tracking_on(false), MAX_ELAPSED_TIME_(3.0)
{
    tagMapPosSub_ = nh_.subscribe("/droneuse/tagMap_position", 1, &landingPlatformTracker::tagMapPosSub_callback, this);
    positionPub_    = nh_.advertise<PointStamped>("droneuse/landing_platform_position", 1);

    float imPosx_cov, imPosy_cov, imPosz_cov, predVelx_cov, predVely_cov, predVelz_cov;
    ros::param::get("artificial_vision/img_pos_cov_x", imPosx_cov);
    ros::param::get("artificial_vision/img_pos_cov_y", imPosy_cov);
    ros::param::get("artificial_vision/img_pos_cov_z", imPosz_cov);

    ros::param::get("artificial_vision/pred_velx_cov", predVelx_cov);
    ros::param::get("artificial_vision/pred_vely_cov", predVely_cov);
    ros::param::get("artificial_vision/pred_velz_cov", predVelz_cov);

    //position_kf     = KalmanFilter();
    position_kf     = KalmanFilter(imPosx_cov, imPosy_cov, imPosz_cov, predVelx_cov, predVely_cov, predVelz_cov);
    last_detection_time = ros::Time::now() - MAX_ELAPSED_TIME_;

    return;
}

landingPlatformTracker::~landingPlatformTracker()
{
	return;
}


void landingPlatformTracker::tagMapPosSub_callback(const geometry_msgs::PointStamped& tagMap_pos)
{
    last_detection_time = tagMap_pos.header.stamp;
	detected_position.x = tagMap_pos.point.x;
	detected_position.y = tagMap_pos.point.y;
	detected_position.z = tagMap_pos.point.z;
    ros::Time pred_time = ros::Time::now();

        if(!tracking_on)
        {
            // Kalman Filter Initialisation
            position_kf.init(detected_position.x, detected_position.y, detected_position.z, 0.0, 0.0, 0.0, last_detection_time);
//	    position_kf.predict(pred_time);
            tracking_on = true;
        }
        else
        {
            // Kalman Filter Updating
            position_kf.update(detected_position.x, detected_position.y, detected_position.z, last_detection_time);
//	    position_kf.predict(pred_time);
        }
/*
        geometry_msgs::PointStamped landing_platform_pos;
        landing_platform_pos.header.stamp = pred_time;
        landing_platform_pos.header.frame_id = "/body_frame";
        landing_platform_pos.point.x = position_kf.x(0,0);
        landing_platform_pos.point.y = position_kf.x(1,0);
        landing_platform_pos.point.z = position_kf.x(2,0);
        positionPub_.publish(tagMap_pos);
*/
}


void landingPlatformTracker::landingPlatformPosUpdate(double dt)
{
/*
    if(newMeasure)
    {
        if(!tracking_on)
        {
            // Kalman Filter Initialisation
            position_kf.init(detected_position.x, detected_position.y, detected_position.z, 0.0, 0.0, 0.0, last_detection_time);
	    position_kf.predict(ros::Time::now());
            estimated_position.x = position_kf.x(0,0);
            estimated_position.y = position_kf.x(1,0);
            estimated_position.z = position_kf.x(2,0);
            tracking_on = true;
        }
        else
        {
            // Kalman Filter Updating
            position_kf.update(detected_position.x, detected_position.y, detected_position.z, last_detection_time);
	    position_kf.predict(ros::Time::now());
            estimated_position.x = position_kf.x(0,0);
            estimated_position.y = position_kf.x(1,0);
            estimated_position.z = position_kf.x(2,0);
        }

        newMeasure = false;
        // Tag Map Position Publishing
        geometry_msgs::PointStamped tagMap_pos;
        tagMap_pos.header.stamp = position_kf.tStamp;
        tagMap_pos.header.frame_id = "/body_frame";
        tagMap_pos.point.x = estimated_position.x;
        tagMap_pos.point.y = estimated_position.y;
        tagMap_pos.point.z = estimated_position.z;
        positionPub_.publish(tagMap_pos);
//	std::cout << "tagMap_position_kf x " << tagMap_pos.point.x << std::endl;
//	std::cout << "tagMap_position_kf y " << tagMap_pos.point.y << std::endl;
//	std::cout << "tagMap_position_kf z " << tagMap_pos.point.z << std::endl;
        
    }
    else if( ros::Duration( ros::Time::now() - last_detection_time ) < MAX_ELAPSED_TIME_ )
*/
	ros::Time time_now = ros::Time::now();
    if( ros::Duration( time_now - last_detection_time ) < MAX_ELAPSED_TIME_ )
    {
        // Kalman Filter Update
        position_kf.predict(dt);
        
        // Tag Map Position Publishing
	estimated_position.x = 0.5*(position_kf.x(0,0) + estimated_position.x);
	estimated_position.y = 0.5*(position_kf.x(1,0) + estimated_position.y);
	estimated_position.z = 0.3*position_kf.x(2,0) + 0.7*estimated_position.z;
        geometry_msgs::PointStamped tagMap_pos;
        tagMap_pos.header.stamp = position_kf.tStamp;
        tagMap_pos.header.frame_id = "/body_frame";
        tagMap_pos.point.x = estimated_position.x;
        tagMap_pos.point.y = estimated_position.y;
        tagMap_pos.point.z = estimated_position.z;
        positionPub_.publish(tagMap_pos);
    }
    else
    {
        //std::cout << "Tracking Deactivated!" << std::endl;
        tracking_on = false;
    }
}
