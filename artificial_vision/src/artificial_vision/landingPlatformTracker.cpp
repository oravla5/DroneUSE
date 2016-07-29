#include "landingPlatformTracker.h"

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <boost/timer.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace ros;
using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace boost;

landingPlatformTracker::landingPlatformTracker(char *imageTopic) : newMeasure(false), tracking_on(false), MAX_ELAPSED_TIME_(3.0)
{
    tagMapPosSub_ = nh_.subscribe("/droneuse/tagMap_position", 1, &landingPlatformTracker::tagMapPosSub_callback, this);
    positionPub_    = nh_.advertise<Odometry>("droneuse/landing_platform_position", 1);

    float imPosx_cov, imPosy_cov, imPosz_cov, predVelx_cov, predVely_cov, predVelz_cov;
    ros::param::get("artificial_vision/img_pos_cov_x", imPosx_cov);
    ros::param::get("artificial_vision/img_pos_cov_y", imPosy_cov);
    ros::param::get("artificial_vision/img_pos_cov_z", imPosz_cov);

    ros::param::get("artificial_vision/pred_velx_cov", predVelx_cov);
    ros::param::get("artificial_vision/pred_vely_cov", predVely_cov);
    ros::param::get("artificial_vision/pred_velz_cov", predVelz_cov);

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
            tracking_on = true;
        }
        else
        {
            // Kalman Filter Updating
            position_kf.update(detected_position.x, detected_position.y, detected_position.z, last_detection_time);
        }
}


void landingPlatformTracker::landingPlatformPosUpdate(double dt)
{
	ros::Time time_now = ros::Time::now() + ros::Duration(0.1);
    if( ros::Duration( time_now - last_detection_time ) < MAX_ELAPSED_TIME_ )
    {
        // Kalman Filter Update
        Eigen::Matrix<double,6,1> x_state = position_kf.predict(time_now);
        
        // Tag Map Position Publishing

	estimated_state.header.stamp = position_kf.tStamp;
	estimated_state.header.frame_id = "/body_frame";

	estimated_state.pose.pose.position.x = 0.5*(x_state(0,0) + estimated_state.pose.pose.position.x);
	estimated_state.pose.pose.position.y = 0.5*(x_state(1,0) + estimated_state.pose.pose.position.y);
	estimated_state.pose.pose.position.z = 0.3*x_state(2,0) + 0.7*estimated_state.pose.pose.position.z;

	estimated_state.twist.twist.linear.x = 0.3*x_state(3,0) + 0.7*estimated_state.twist.twist.linear.x;
	estimated_state.twist.twist.linear.y = 0.3*x_state(4,0) + 0.7*estimated_state.twist.twist.linear.y;
	estimated_state.twist.twist.linear.z = 0.3*x_state(5,0) + 0.7*estimated_state.twist.twist.linear.z;

        positionPub_.publish(estimated_state);
    }
    else
    {
        //std::cout << "Tracking Deactivated!" << std::endl;
	estimated_state.pose.pose.position.x = 0.0;
	estimated_state.pose.pose.position.y = 0.0;
	estimated_state.pose.pose.position.z = 0.0;

	estimated_state.twist.twist.linear.x = 0.0;
	estimated_state.twist.twist.linear.y = 0.0;
	estimated_state.twist.twist.linear.z = 0.0;
        tracking_on = false;
    }
}
