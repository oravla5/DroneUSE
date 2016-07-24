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

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
using namespace ros;
using namespace std;
using namespace sensor_msgs;
using namespace april::tag;
using namespace geometry_msgs;
using namespace boost;

#define C_PI (double) 3.141592653589793

tagMapDetector::tagMapDetector(char *imageTopic, string marker_map_cfg_file) : it_(nh_), markerMapCfg_file(marker_map_cfg_file) 
{
	//Subscription
	imgSub_ = it_.subscribeCamera("/dji_sdk/image_raw", 1, &tagMapDetector::imgSub_callback, this);

	//Landing AprilTag position publisher
	tagMapPub_ = nh_.advertise<PointStamped>("droneuse/tagMap_position", 1);
	orig_tagMapPub_ = nh_.advertise<PointStamped>("droneuse/orig_tagMap_position", 1);
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
	SCALE_min = 0.2;
	SCALE_max = 0.8;
	ros::param::get("artificial_vision/scale_min", SCALE_min);
	ros::param::get("artificial_vision/scale_max", SCALE_max);
	SCALE_FACTOR = (SCALE_min + SCALE_max)/2;
	DIST_min = 0.1;
	DIST_max = 10.0;
	ros::param::get("artificial_vision/dist_max", DIST_max);
	ros::param::get("artificial_vision/dist_min", DIST_min);
	

	tf_listener = new tf::TransformListener;
	LPF_beta_x = 0.125;
	LPF_beta_y = 0.125;
	LPF_beta_z = 0.125;
	ros::param::get("artificial_vision/LPF_beta_x", LPF_beta_x);	
	ros::param::get("artificial_vision/LPF_beta_y", LPF_beta_y);	
	ros::param::get("artificial_vision/LPF_beta_z", LPF_beta_z);	
	return;
}

tagMapDetector::~tagMapDetector()
{
	return;
}

void tagMapDetector::imgSub_callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& info_msg)
{
//   if(cam_parameters.CameraMatrix.empty() || cam_parameters.Distorsion.empty())
//    {
        float cameraMatrix_components[9] = {2.0*float(info_msg->K[0]*SCALE_FACTOR), 	2.0*float(info_msg->K[1]*SCALE_FACTOR), 	 2.0*float(info_msg->K[2]*SCALE_FACTOR),
                                            1.5*float(info_msg->K[3]*SCALE_FACTOR), 	1.5*float(info_msg->K[4]*SCALE_FACTOR), 1.5*float(info_msg->K[5]*SCALE_FACTOR), 
                                            float(info_msg->K[6]*SCALE_FACTOR), 	float(info_msg->K[7]*SCALE_FACTOR),	 float(info_msg->K[8]*SCALE_FACTOR)}; 

        float cameraDistor_components[4] = {float(info_msg->D[0]),float(info_msg->D[1]),float(info_msg->D[2]),float(info_msg->D[3])};
//        float cameraDistor_components[4] = {0.0,0.0,0.0,0.0};

        cv::Mat cameraMatrix(3,3,CV_32F, cameraMatrix_components);
        cv::Mat distorsion(1,4,CV_32F, cameraDistor_components);
        cv::Size imgSize( info_msg->width, info_msg->height);
        cam_parameters.setParams(cameraMatrix, distorsion, imgSize);
//    }
	//Ros image message to cv format
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
	cv::Mat frame_ori = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
	cv::Mat frame_scaled;
	cv::Mat frame;


	cv::resize(frame_ori, frame_scaled, cv::Size(), SCALE_FACTOR, SCALE_FACTOR); 
	frame = frame_scaled;

	vector<TagDetection> tags_detected;
	ros::Time image_processing = ros::Time::now();
	tag_detector_->process(frame, tags_detected);
	std::cout << "it takes " << ros::Duration(ros::Time::now() - image_processing) << " seconds to process image" << std::endl;

    if(tags_detected.size()>0)
    {
        vector<aruco::Marker> detected_aruco_markers;

        //Publish tags
        //std::cout << "Number of tags detected = " << tags_detected.size() << std::endl; 
        for(size_t i=0; i<tags_detected.size(); i++)
        {
            if(tags_detected[i].good && (tags_detected[i].id == 0 || tags_detected[i].id == 1 || tags_detected[i].id == 2 || tags_detected[i].id == 3 ||
					tags_detected[i].id == 4 || tags_detected[i].id == 5 || tags_detected[i].id == 5 || tags_detected[i].id == 6))
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
	ros::Time pnp_time = ros::Time::now();
        result = markerMapCfg.calculateExtrinsics(detected_aruco_markers, 0.0, cam_parameters.CameraMatrix, cam_parameters.Distorsion);
	std::cout << "it takes " << ros::Duration(ros::Time::now() - pnp_time) << " seconds to perform pnp" << std::endl;
        //markerMapCfg.calculateExtrinsics(detected_aruco_markers, 0.0, cam_parameters.CameraMatrix, cam_parameters.Distorsion, tvec_pnp, rvec_pnp);

        rvec_pnp = result.first;
        tvec_pnp = result.second;

	// Important to access tvec_pnp data with <float>
	float pos_x = tvec_pnp.at<float>(0,0);
	float pos_y = tvec_pnp.at<float>(0,1);
	float pos_z = tvec_pnp.at<float>(0,2);
	
	float tagMap_dist = sqrt(pos_x*pos_x + pos_y*pos_y + pos_z*pos_z);

	// Filtered vs non-filtered
	geometry_msgs::PointStamped orig_tagMap_pos;
	orig_tagMap_pos.header.stamp = image_msg->header.stamp;
	orig_tagMap_pos.header.frame_id = "/camera";
	orig_tagMap_pos.point.x = pos_x;
	orig_tagMap_pos.point.y = pos_y;
	orig_tagMap_pos.point.z = pos_z;
	try
	{
		tf_listener->waitForTransform("/body_frame", "/camera",image_msg->header.stamp, ros::Duration(3.0/20));
		tf_listener->transformPoint("/body_frame", orig_tagMap_pos, orig_tagMap_pos);
		orig_tagMapPub_.publish(orig_tagMap_pos);
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}

	// Check tagMap detection and its distance to the camera 
        if(!(tvec_pnp.empty() || rvec_pnp.empty()) && (tagMap_dist < 20.0) && (tagMap_dist > 0.1) && (pos_z > 0.1))     // Has any marker of the map been detected?
        { 
		geometry_msgs::PointStamped tagMap_pos;
		tagMap_pos.header.stamp = image_msg->header.stamp;
		tagMap_pos.header.frame_id = "/camera";
		tagMap_pos.point.x = pos_x;
		tagMap_pos.point.y = pos_y;
		tagMap_pos.point.z = pos_z;

		//tf_camera_apriltagMap.setOrigin( tf::Vector3(tvec_pnp.at<float>(0,0), tvec_pnp.at<float>(0,1), tvec_pnp.at<float>(0,2)) );
		//q.setRPY(rvec_pnp.at<float>(0,0), rvec_pnp.at<float>(0,2), rvec_pnp.at<float>(0,1));
		//tf_camera_apriltagMap.setRotation(q);
		//br.sendTransform(tf::StampedTransform(tf_camera_apriltagMap, image_msg->header.stamp, "camera", "apriltag_map"));


		try
		{
			tf_listener->waitForTransform("/body_frame", "/camera",image_msg->header.stamp, ros::Duration(3.0/20));
			tf_listener->transformPoint("/body_frame", tagMap_pos, tagMap_pos);
			float gimbal_req_yaw = atan2(tagMap_pos.point.y, tagMap_pos.point.x)*180/C_PI;
			float gimbal_req_roll = atan2(tagMap_pos.point.z, tagMap_pos.point.x)*180/C_PI;
			// Check if the point is on  a logic position
			//if( (fabs(gimbal_req_yaw) < 90) && ( gimbal_req_roll > -20 ) && ( gimbal_req_roll < 120 ) )
			if( ( gimbal_req_roll > -20 ) && ( gimbal_req_roll < 120 ) )
			{
				
				if(first_detection)		// Check if the Low Pass Filter is initialized
				{	
					last_tagMap = tagMap_pos;
					first_detection = false;
				}
				else if((tagMap_pos.header.stamp - last_tagMap.header.stamp) > ros::Duration(2.0)) 	// Check if the last detection is too old
				{
					last_tagMap = tagMap_pos;
				}

				tagMap_pos.point.x = LPF_beta_x*tagMap_pos.point.x + (1 - LPF_beta_x)*last_tagMap.point.x;
				tagMap_pos.point.y = LPF_beta_y*tagMap_pos.point.y + (1 - LPF_beta_y)*last_tagMap.point.y;
				tagMap_pos.point.z = LPF_beta_z*tagMap_pos.point.z + (1 - LPF_beta_z)*last_tagMap.point.z;
				
				last_tagMap = tagMap_pos;
				tagMapPub_.publish(tagMap_pos);
				float dist = sqrt(tagMap_pos.point.x*tagMap_pos.point.x + tagMap_pos.point.y*tagMap_pos.point.y + tagMap_pos.point.z*tagMap_pos.point.z);

				SCALE_FACTOR = SCALE_min + (SCALE_max - SCALE_min)/(DIST_max - DIST_min)*(dist - DIST_min);
				if(SCALE_FACTOR > SCALE_max)
					SCALE_FACTOR = SCALE_max;
				else if(SCALE_FACTOR < SCALE_min)
					SCALE_FACTOR = SCALE_min;
				std::cout << "scale factor = " << SCALE_FACTOR << " dist = " << dist << std::endl;
	//				std::cout << "tagMap_pos_detector x " << tagMap_pos.point.x << std::endl;
	//				std::cout << "tagMap_pos_detector y " << tagMap_pos.point.y << std::endl;
	//				std::cout << "tagMap_pos_detector z " << tagMap_pos.point.z << std::endl;
			}
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			return;
		}
        }
    }
	return;
}

