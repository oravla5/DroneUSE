#ifndef LANDING_PLATFORM_TRACKER_H
#define LANDING_PLATFORM_TRACKER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>

#include <kalman_filter.h>

class landingPlatformTracker
{
    ros::NodeHandle                                 nh_;

    ros::Subscriber                                 tagMapPosSub_;
    ros::Publisher					                positionPub_;

    // TagMap position and Kalman Filter
    KalmanFilter                                    position_kf;
    bool                                            tracking_on;
    bool                                            newMeasure;
    cv::Point3f                                     detected_position;
    cv::Point3f                                     estimated_position;
    ros::Time                                       last_detection_time;    // Time elapsed since the last detection

    const ros::Duration                             MAX_ELAPSED_TIME_;

    public:
            landingPlatformTracker(char *imageTopic);
            ~landingPlatformTracker();
    void    tagMapPosSub_callback(const geometry_msgs::PointStamped &tagMap_pos);

    void    landingPlatformPosUpdate(double dt);
};

#endif
