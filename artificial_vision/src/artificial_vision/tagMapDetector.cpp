#include "tagMapDetector.h"

#include "TagDetector.hpp"
#include "TagFamilyFactory.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PointStamped.h>

#include <math.h>
#include <boost/timer.hpp>

#include <markermap.h>

using namespace ros;
using namespace std;
using namespace sensor_msgs;
using namespace april::tag;
using namespace geometry_msgs;
using namespace boost;

tagMapDetector::tagMapDetector(char *imageTopic, string marker_map_cfg_file) : it_(nh_), markerMapCfg_file(marker_map_cfg_file), MIN_FRAME_NUM_(3), MIN_TAG_DIST_(1000), TAG_SIZE_(0.06), SCALE_FACTOR_(0.7)
{
	//Subscription
	imgSub_ = it_.subscribeCamera("/dji_sdk/image_raw", 1, &tagMapDetector::imgSub_callback, this);

	//Landing AprilTag position publisher
	tagMapPub_ = nh_.advertise<PointStamped>("droneuse/tagMap_position", 1);
	imgPub_ = it_.advertise("droneuse/image2", 1);

	//AprilTag detector initialization
		// 0 : Tag type tag16h5
		// 4 : Tag type tag36h11
	TagFamilyFactory::create("04", gTagFamilies_);	
	tag_detector_ = new TagDetector(gTagFamilies_);

	markerMapCfg.readFromFile(markerMapCfg_file);

	// Conversion to meters is required
	// Tag16h5 size in pixel = 213
	// Tag16h5 size in meters = 0.06
	markerMapCfg = markerMapCfg.convertToMeters_pixelSize(float(0.06/213.0));


	return;
}

tagMapDetector::~tagMapDetector()
{
	return;
}

void tagMapDetector::imgSub_callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& info_msg)
{
    if(cam_parameters.CameraMatrix.empty() || cam_parameters.Distorsion.empty())
    {
        float cameraMatrix_components[9] = {float(info_msg->K[0])*SCALE_FACTOR_, float(info_msg->K[1])*SCALE_FACTOR_, float(info_msg->K[2])*SCALE_FACTOR_,
                                            float(info_msg->K[3])*SCALE_FACTOR_, float(info_msg->K[4])*SCALE_FACTOR_, float(info_msg->K[5])*SCALE_FACTOR_, 
                                            float(info_msg->K[6])*SCALE_FACTOR_, float(info_msg->K[7])*SCALE_FACTOR_, float(info_msg->K[8])*SCALE_FACTOR_}; 

        float cameraDistor_components[9] = {float(info_msg->D[0]),float(info_msg->D[1]),float(info_msg->D[2]),float(info_msg->D[3])};

        cv::Mat cameraMatrix(3,3,CV_32F, cameraMatrix_components);
        cv::Mat distorsion(1,4,CV_32F, cameraDistor_components);
        cv::Size imgSize(info_msg->height, info_msg->width);
        cam_parameters.setParams(cameraMatrix, distorsion, imgSize);
    }
	//Ros image message to cv format
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
	cv::Mat frame_ori = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
	cv::Mat frame_scaled;
	cv::Mat frame;


	cv::resize(frame_ori, frame_scaled, cv::Size(), SCALE_FACTOR_, SCALE_FACTOR_); 
	frame = frame_scaled;

	vector<TagDetection> tags_detected;
	tag_detector_->process(frame, tags_detected);

    if(tags_detected.size()>0)
    {
        vector<aruco::Marker> detected_aruco_markers;

        //Publish tags
        std::cout << "Number of tags detected = " << tags_detected.size() << std::endl; 
        for(size_t i=0; i<tags_detected.size(); i++)
        {
            if(tags_detected[i].good)
                {
                    std::vector< cv::Point2f > corners;
                    tags_detected[i].draw(frame);
                    for(int k=0; k<4; k++)
                        corners.push_back(cv::Point2f(tags_detected[i].p[k][0], tags_detected[i].p[k][1]));

                    aruco::Marker aruco_marker(corners, tags_detected[i].id, tags_detected[i].familyName);
                    detected_aruco_markers.push_back(aruco_marker);

                }
        }
        // Calculate Tag Map Position with PnP
        std::pair <cv::Mat, cv::Mat> result; 
        //cv::Mat rvec_pnp(3, 1, CV_64FC1); 
        //cv::Mat tvec_pnp(3, 1, CV_64FC1);
        cv::Mat rvec_pnp; 
        cv::Mat tvec_pnp;
        result = markerMapCfg.calculateExtrinsics(detected_aruco_markers, 0.0, cam_parameters.CameraMatrix, cam_parameters.Distorsion);
        //markerMapCfg.calculateExtrinsics(detected_aruco_markers, 0.0, cam_parameters.CameraMatrix, cam_parameters.Distorsion, tvec_pnp, rvec_pnp);

        std::cout << "tvec_pnp type " << tvec_pnp.type() << std::endl;
        rvec_pnp = result.first;
        tvec_pnp = result.second;

        if(!(tvec_pnp.empty() || rvec_pnp.empty()))     // Has any marker of the map been detected?
        { 
            geometry_msgs::PointStamped tagMap_pos;
            tagMap_pos.header.stamp = ros::Time::now();
            tagMap_pos.header.frame_id = "/camera";
            tagMap_pos.point.x = tvec_pnp.at<double>(0,0);
            tagMap_pos.point.y = tvec_pnp.at<double>(0,1);
            tagMap_pos.point.z = tvec_pnp.at<double>(0,2);
            tagMapPub_.publish(tagMap_pos);

        }
            // Get Tag Map Rotation quaternion            
/*
            cv::Mat rot_mat; 
            cv::Rodrigues(rvec_pnp, rot_mat);

            tf::Matrix3x3 rot_mat_tf(   rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
                                        rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
                                        rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2));
            tf::Quaternion q_rot;
            rot_mat_tf.getRotation(q_rot);
            geometry_msgs::PoseStamped tag_pose;
            tag_pose.header.stamp = image_msg->header.stamp;
            tag_pose.header.frame_id = "/camera";

            //Tag Map Reference System pose
            tag_pose.pose.position.x = tvec_pnp.at<double>(0,0);
            tag_pose.pose.position.y = tvec_pnp.at<double>(0,1);
            tag_pose.pose.position.z = tvec_pnp.at<double>(0,2);
            tag_pose.pose.orientation.x = 0;//q_rot.x();
            tag_pose.pose.orientation.y = 0;//q_rot.y();
            tag_pose.pose.orientation.z = 0;//q_rot.z();
            tag_pose.pose.orientation.w = 0;// q_rot.w();
            tag_pose.pose.orientation.x = q_rot.x();
            tag_pose.pose.orientation.y = q_rot.y();
            tag_pose.pose.orientation.z = q_rot.z();
            tag_pose.pose.orientation.w = q_rot.w();

            tagMapPub_.publish(tag_pose);

            // Board Center Display
            std::vector<cv::Point3f> world_points;
            //objectPoints.push_back(cv::Point3f(float(markerMapCfg.mapSize_meters.x/2.0), float(markerMapCfg.mapSize_meters.y/2.0), 0.0));
            world_points.push_back(cv::Point3f(0.0, 0.0, 0.0));
            world_points.push_back(cv::Point3f(float(markerMapCfg.mapSize_meters.x), 0.0, 0.0));
            world_points.push_back(cv::Point3f(float(markerMapCfg.mapSize_meters.x), float(markerMapCfg.mapSize_meters.y), 0.0));
            world_points.push_back(cv::Point3f(0.0, float(markerMapCfg.mapSize_meters.y), 0.0));
            std::vector<cv::Point2f> image_points;
            cv::projectPoints(world_points, rvec_pnp, tvec_pnp, cam_parameters.CameraMatrix, cam_parameters.Distorsion, image_points);
            //line(frame, image_points[0], image_points[1], cv::Scalar( 0, 0, 255 ), 3);
            //line(frame, image_points[1], image_points[2], cv::Scalar( 0, 0, 255 ), 3);
            //line(frame, image_points[2], image_points[3], cv::Scalar( 0, 0, 250 ), 3);
            //line(frame, image_points[3], image_points[0], cv::Scalar( 0, 0, 250 ), 3);
            circle(frame, image_points[0], 20, cv::Scalar( 0, 0, 255 ), 4);

            cv_bridge::CvImage send (cv_ptr->header, cv_ptr->encoding, frame);
            imgPub_.publish(send.toImageMsg());


           */
    }
	return;
}

