#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

#include <cameraparameters.h>
#include <image_transport/image_transport.h>

using namespace ros;
using namespace sensor_msgs;
using namespace message_filters;

aruco::CameraParameters                         cam_parameters;

image_transport::Publisher                      imgPub_;

cv::Mat draw_tagMap(cv::Mat frame, geometry_msgs::PointStamped point);

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, geometry_msgs::PointStamped> MySincPolicy;

void callback(const ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, const geometry_msgs::PointStampedConstPtr& tagMap_pos)
{
    if(cam_parameters.CameraMatrix.empty() || cam_parameters.Distorsion.empty())
    {
        float cameraMatrix_components[9] = {float(2.0*info_msg->K[0]), 		float(info_msg->K[1]), 		float(2.0*info_msg->K[2]),
                                            float(info_msg->K[3]), 		float(1.5*info_msg->K[4]), 	float(1.5*info_msg->K[5]), 
                                            float(info_msg->K[6]), 		float(info_msg->K[7]), 		float(info_msg->K[8])}; 

        float cameraDistor_components[4] = {float(info_msg->D[0]),float(info_msg->D[1]),float(info_msg->D[2]),float(info_msg->D[3])};

        cv::Mat cameraMatrix(3,3,CV_32F, cameraMatrix_components);
        cv::Mat distorsion(1,4,CV_32F, cameraDistor_components);
        cv::Size imgSize(info_msg->width, info_msg->height);
        cam_parameters.setParams(cameraMatrix, distorsion, imgSize);
    }

    cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
	cv::Mat frame = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

    frame = draw_tagMap(frame, *tagMap_pos);
    cv_bridge::CvImage send (cv_ptr->header, cv_ptr->encoding, frame);
    imgPub_.publish(send.toImageMsg());


}

int main(int argc, char **argv)
{

	init(argc, argv, "landing_platform_display");

    ros::NodeHandle nh;
    
    image_transport::ImageTransport			        it_(nh);
    imgPub_ = it_.advertise("droneuse/landingPlatform_videoFootage", 1);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/dji_sdk/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camInfo_sub(nh, "/dji_sdk/camera_info",1);
    message_filters::Subscriber<geometry_msgs::PointStamped> platformPos_sub(nh, "/droneuse/landing_platform_position", 1);
    message_filters::Synchronizer<MySincPolicy>* sync_;

    sync_ = new Synchronizer<MySincPolicy>(MySincPolicy(10), image_sub, camInfo_sub, platformPos_sub);
    sync_->registerCallback(boost::bind(&callback, _1, _2, _3));

    spin();
    return 0;
}


cv::Mat draw_tagMap(cv::Mat frame, geometry_msgs::PointStamped point2draw)
{
    // Board Center Display
    std::vector<cv::Point3f> world_points;
    world_points.push_back(cv::Point3f(0.0, 0.0, 0.0));
    cv::Mat tvec(3,1, CV_32FC1);
    tvec.at<float>(0,0) = point2draw.point.x;
    tvec.at<float>(1,0) = point2draw.point.y;
    tvec.at<float>(2,0) = point2draw.point.z;
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

