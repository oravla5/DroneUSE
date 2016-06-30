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

landingPlatformTracker::landingPlatformTracker(char *imageTopic) : newMeasure(false), tracking_on(false), MAX_ELAPSED_TIME_(2.5)
{
    tagMapPosSub_ = nh_.subscribe("/droneuse/tagMap_position", 1, &landingPlatformTracker::tagMapPosSub_callback, this);
    positionPub_    = nh_.advertise<PointStamped>("droneuse/landing_platform_position", 1);

    position_kf     = KalmanFilter();
    last_detection_time = ros::Time::now() - MAX_ELAPSED_TIME_;

    return;
}

landingPlatformTracker::~landingPlatformTracker()
{
	return;
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
    {
        // Kalman Filter Update
        position_kf.predict(dt);
        estimated_position.x = position_kf.x(0,0);
        estimated_position.y = position_kf.x(1,0);
        estimated_position.z = position_kf.x(2,0);
        
        // Tag Map Position Publishing
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
        std::cout << "Tracking Deactivated!" << std::endl;
        tracking_on = false;
    }
}
