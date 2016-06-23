#include "landingTagMapTracker.h"

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

landingTagMapTracker::landingTagMapTracker(char *imageTopic, string marker_map_cfg_file) : it_(nh_), markerMapCfg_file(marker_map_cfg_file), MIN_FRAME_NUM_(3), MIN_TAG_DIST_(1000), TAG_SIZE_(0.06), SCALE_FACTOR_(0.7)
{
	//Subscription
	imgSub_ = it_.subscribeCamera("/dji_sdk/image_raw", 1, &landingTagMapTracker::imgSub_callback, this);

	//Landing AprilTag position publication
	landingTagPub_ = nh_.advertise<PoseStamped>("droneuse/landing_tag_position", 1);
    imgPub_ = it_.advertise("droneuse/image", 1);

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
    std::cout << "Tag Id: " << markerMapCfg[0].dict << std::endl;

	return;
}

landingTagMapTracker::~landingTagMapTracker()
{
	return;
}

void landingTagMapTracker::imgSub_callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& info_msg)
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
	//cv::Mat frame_cropped;
	cv::Mat frame_scaled;
	cv::Mat frame;

	//if(frame_ori.empty())
	//	return;

	//int crop_width = frame_ori.size().width/1;
	//int crop_height  = frame_ori.size().height/1;
	//int crop_x = (frame_ori.size().width   - crop_width)/2; 
	//int crop_y = (frame_ori.size().height  - crop_height)/2; 

	//cv::Rect roiRect = cv::Rect(crop_x, crop_y, crop_width, crop_height); // ROI in source image

	//frame_cropped = frame_ori(roiRect);
	//cv::resize(frame_cropped, frame_scaled, cv::Size(), SCALE_FACTOR_, SCALE_FACTOR_); 
	cv::resize(frame_ori, frame_scaled, cv::Size(), SCALE_FACTOR_, SCALE_FACTOR_); 
	frame = frame_scaled;
	//frame = frame_ori;

	vector<TagDetection> tags_detected;
	tag_detector_->process(frame, tags_detected);

    if(tags_detected.size()>0)
    {
        vector<aruco::Marker> detected_aruco_markers;
        //std::cout << "Detected Tag Id: " << tags_detected[1].familyName << std::endl;
	//Publish tags
        std::cout << "Number of tags detected = " << tags_detected.size() << std::endl; 
	    for(size_t i=0; i<tags_detected.size(); i++)
	    {
            //std::cout << "Tag index = " << i << std::endl;
	    	if(tags_detected[i].good)
	    	{
                std::vector< cv::Point2f > corners;
                for(int k=0; k<4; k++)
                    corners.push_back(cv::Point2f(tags_detected[i].p[k][0], tags_detected[i].p[k][1]));
                aruco::Marker aruco_marker(corners, tags_detected[i].id, tags_detected[i].familyName);
                // Marker Identification
                //aruco_marker.id = tags_detected[i].id;
                //aruco_marker.family = tags_detected[i].familyName;  // TAG16h5, TAG36h11
                //// Marker corners
                ////std::cout << "hi there!" << std::endl << "size aruco_marker= " << aruco_marker[] << std::endl; //<< "size tags_detected = " << tags_detected[i].p[0][0] << std::end;
                //std::cout << "hi there!" << std::endl << "tags_detected coordinates: = " << tags_detected[i].p[0][0] << ", " << tags_detected[i].p[0][1] << std::endl;
                //aruco_marker.at(0).x = tags_detected[i].p[0][0];
                //aruco_marker.at(0).y = tags_detected[i].p[0][1];
                //std::cout << "hi there!" << std::endl << "tags_detected coordinates: = " << tags_detected[i].p[1][0] << ", " << tags_detected[i].p[1][1] << std::endl;
                //aruco_marker[1].x = tags_detected[i].p[1][0];
                //aruco_marker[1].y = tags_detected[i].p[1][1];
                //std::cout << "hi there!" << std::endl << "tags_detected coordinates: = " << tags_detected[i].p[2][0] << ", " << tags_detected[i].p[2][1] << std::endl;
                //aruco_marker[2].x = tags_detected[i].p[2][0];
                //aruco_marker[2].y = tags_detected[i].p[2][1];
                //std::cout << "hi there!" << std::endl << "tags_detected coordinates: = " << tags_detected[i].p[3][0] << ", " << tags_detected[i].p[3][1] << std::endl;
                //aruco_marker[3].x = tags_detected[i].p[3][0];
                //aruco_marker[3].y = tags_detected[i].p[3][1];
                //std::cout << "tag family " << aruco_marker.family << std::endl;
                //std::cout << "tag id " << aruco_marker.id << std::endl;
                //std::cout  << "tags_detected coordinates 0: = " << aruco_marker[0].x << ", " << aruco_marker[0].y << std::endl;
                //std::cout  << "tags_detected coordinates 1: = " << aruco_marker[1].x << ", " << aruco_marker[1].y << std::endl;
                //std::cout  << "tags_detected coordinates 2: = " << aruco_marker[2].x << ", " << aruco_marker[2].y << std::endl;
                //std::cout  << "tags_detected coordinates 3: = " << aruco_marker[3].x << ", " << aruco_marker[3].y << std::endl;
                detected_aruco_markers.push_back(aruco_marker);


            }

        }
        //for(size_t kk=0;kk<detected_aruco_markers.size(); kk++)
        //    std::cout << "Marker " << kk << ": Family " << detected_aruco_markers[kk].family << " Id " << detected_aruco_markers[kk].id << std::endl;

        std::pair <cv::Mat, cv::Mat> result; 
	result = markerMapCfg.calculateExtrinsics(detected_aruco_markers, 0.0, cam_parameters.CameraMatrix, cam_parameters.Distorsion);
std::cout << "El tamano de rvec es: " << result.first.size() << std::endl;
cv::Mat rvec_pnp, tvec_pnp;
result.first.copyTo(rvec_pnp);
result.second.copyTo(tvec_pnp);
std::cout << "rvec esta vacio?: " << result.second.empty() << std::endl;
        if(!(tvec_pnp.empty() || rvec_pnp.empty()))     // Has any marker of the map been detected?
        {
            geometry_msgs::PoseStamped tag_pose;
            
            tag_pose.header.stamp = image_msg->header.stamp;
            tag_pose.header.frame_id = "/camera";

            //Camera system tag coordinates to 3D world coordinates
            tag_pose.pose.position.x = tvec_pnp.at<double>(0,0);
            tag_pose.pose.position.y = tvec_pnp.at<double>(0,1);
            tag_pose.pose.position.z = tvec_pnp.at<double>(0,2);

            tf::Vector3 tag_position(tag_pose.pose.position.x,tag_pose.pose.position.y,tag_pose.pose.position.z);
            
            cv::Matx33d rot_mat;
std::cout << "Antes de rodrigues"; 
            cv::Rodrigues(rvec_pnp, rot_mat);
std::cout << "Despues de rodrigues"; 
            tf::Matrix3x3 rot_mat_tf(   rot_mat(0,0),rot_mat(0,1),rot_mat(0,2),
                                        rot_mat(1,0),rot_mat(1,1),rot_mat(1,2),
                                        rot_mat(2,0),rot_mat(2,1),rot_mat(2,2));
            tf::Quaternion q_rot;
            rot_mat_tf.getRotation(q_rot);
            tag_pose.pose.orientation.x = q_rot.x();
            tag_pose.pose.orientation.y = q_rot.y();
            tag_pose.pose.orientation.z = q_rot.z();
            tag_pose.pose.orientation.w = q_rot.w();
            landingTagPub_.publish(tag_pose);

            // Point Display
            
            cv::Matx31d map_pos(double(markerMapCfg.mapSize_meters.x/2),double(markerMapCfg.mapSize_meters.y/2),0.0);
            cv::Matx31d trans_vec(tvec_pnp.at<double>(0,0), tvec_pnp.at<double>(0,1),tvec_pnp.at<double>(0,2));
            cv::Matx31d map_pos_wrt_cam = rot_mat*map_pos+trans_vec;
            cv::Mat tvec_camera(3,1,CV_32F,{0,0,0});
            //cv::Mat tvec_camera(3,1,CV_32F,{tvec_pnp.at<double>(0,0), tvec_pnp.at<double>(0,1),tvec_pnp.at<double>(0,2)});
            //cv::Mat rvec_camera(3,1,CV_32F,{rvec_pnp.at<double>(0,0), rvec_pnp.at<double>(0,1),rvec_pnp.at<double>(0,2)});
            cv::Mat rvec_camera(3,1,CV_32F,{0,0,0});
            std::vector<cv::Point3f> objectPoints;
            //objectPoints.push_back(cv::Point3f(tvec_pnp.at<double>(0,0), tvec_pnp.at<double>(0,1),tvec_pnp.at<double>(0,2)));
            objectPoints.push_back(cv::Point3f(map_pos_wrt_cam(0), map_pos_wrt_cam(1),map_pos_wrt_cam(2)));
            std::vector<cv::Point2f> image_point;
            cv::projectPoints(objectPoints, rvec_camera, tvec_camera, cam_parameters.CameraMatrix, cam_parameters.Distorsion, image_point);
            circle(frame, image_point[0], 20, cv::Scalar( 0, 0, 255 ), 4);
            cv_bridge::CvImage send (cv_ptr->header, cv_ptr->encoding, frame);
            imgPub_.publish(send.toImageMsg());
            //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
            //cv::imshow( "Display window", frame ); 
            //cv::waitKey(0.01);
/*
            */


        }
    }
	return;
}
	    		//tags_detected[i].getPose( TAG_SIZE_, 2.0*SCALE_FACTOR_*info_msg->K[0], 1.5*SCALE_FACTOR_*info_msg->K[4], 2.0*SCALE_FACTOR_*(info_msg->K[2]-(float)crop_x/2.0), 1.5*SCALE_FACTOR_*(info_msg->K[5]-(float)crop_y/1.5), tvec, rvec);

                //PoseStamped tag_pose;
	    		//
	    		//tag_pose.header.stamp = image_msg->header.stamp;
	    		//tag_pose.header.frame_id = "/camera";

	    		////Camera system tag coordinates to 3D world coordinates
	    		//tag_pose.pose.position.x = tvec.at<double>(0);
	    		//tag_pose.pose.position.y = tvec.at<double>(1);
	    		//tag_pose.pose.position.z = tvec.at<double>(2);

                //tf::Vector3 tag_position(tag_pose.pose.position.x,tag_pose.pose.position.y,tag_pose.pose.position.z);
	    		//
                //cv::Matx33d rot_mat;
                //cv::Rodrigues(rvec, rot_mat);

                //tf::Matrix3x3 rot_mat_tf(   rot_mat(0,0),rot_mat(0,1),rot_mat(0,2),
                //                            rot_mat(1,0),rot_mat(1,1),rot_mat(1,2),
                //                            rot_mat(2,0),rot_mat(2,1),rot_mat(2,2));
                //tf::Quaternion q_rot;
                //rot_mat_tf.getRotation(q_rot);
                //tag_pose.pose.orientation.x = q_rot.x();
                //tag_pose.pose.orientation.y = q_rot.y();
                //tag_pose.pose.orientation.z = q_rot.z();
                //tag_pose.pose.orientation.w = q_rot.w();
	    		//landingTagPub_.publish(tag_pose);
	    	//}

//void landingTagMapTracker::imgSub_callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& info_msg)
//{
//
//	//Ros image message to cv format
//	cv::Mat frame_ori = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
//	cv::Mat frame_cropped;
//	cv::Mat frame_scaled;
//	cv::Mat frame;
//
//	if(frame_ori.empty())
//		return;
//
//	int crop_width = frame_ori.size().width/1;
//	int crop_height  = frame_ori.size().height/1;
//	int crop_x = (frame_ori.size().width   - crop_width)/2; 
//	int crop_y = (frame_ori.size().height  - crop_height)/2; 
//
//	cv::Rect roiRect = cv::Rect(crop_x, crop_y, crop_width, crop_height); // ROI in source image
//
//	frame_cropped = frame_ori(roiRect);
//	cv::resize(frame_cropped, frame_scaled, cv::Size(), SCALE_FACTOR_, SCALE_FACTOR_); 
//	frame = frame_scaled;
//
//	vector<TagDetection> tags_detected;
//	tag_detector_->process(frame, tags_detected);
//
//	//Publish tags
//	for(int i=0; i<tags_detected.size(); i++)
//	{
//		if(tags_detected[i].good)
//		{
//            cv::Mat rvec;
//            cv::Mat tvec;
//
//			tags_detected[i].getPose( TAG_SIZE_, 2.0*SCALE_FACTOR_*info_msg->K[0], 1.5*SCALE_FACTOR_*info_msg->K[4], 2.0*SCALE_FACTOR_*(info_msg->K[2]-(float)crop_x/2.0), 1.5*SCALE_FACTOR_*(info_msg->K[5]-(float)crop_y/1.5), tvec, rvec);
//
//            PoseStamped tag_pose;
//			
//			tag_pose.header.stamp = image_msg->header.stamp;
//			tag_pose.header.frame_id = "/camera";
//
//			//Camera system tag coordinates to 3D world coordinates
//			tag_pose.pose.position.x = tvec.at<double>(0);
//			tag_pose.pose.position.y = tvec.at<double>(1);
//			tag_pose.pose.position.z = tvec.at<double>(2);
//
//            tf::Vector3 tag_position(tag_pose.pose.position.x,tag_pose.pose.position.y,tag_pose.pose.position.z);
//			
//            cv::Matx33d rot_mat;
//            cv::Rodrigues(rvec, rot_mat);
//
//            tf::Matrix3x3 rot_mat_tf(   rot_mat(0,0),rot_mat(0,1),rot_mat(0,2),
//                                        rot_mat(1,0),rot_mat(1,1),rot_mat(1,2),
//                                        rot_mat(2,0),rot_mat(2,1),rot_mat(2,2));
//            tf::Quaternion q_rot;
//            rot_mat_tf.getRotation(q_rot);
//            tag_pose.pose.orientation.x = q_rot.x();
//            tag_pose.pose.orientation.y = q_rot.y();
//            tag_pose.pose.orientation.z = q_rot.z();
//            tag_pose.pose.orientation.w = q_rot.w();
//			landingTagPub_.publish(tag_pose);
//		}
//	}
//	return;
//}
