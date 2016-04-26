#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <dji_sdk/dji_drone.h>
#define C_PI (double) 3.141592653589793

// Measures from body_frame
#define CAMERA_X (float) 0.085
#define CAMERA_Y (float) 0
#define CAMERA_Z (float) 0.085

#define GUIDANCE_FRONT_X (float) 0.1
#define GUIDANCE_FRONT_Y (float) 0
#define GUIDANCE_FRONT_Z (float) -0.07

#define GUIDANCE_RIGHT_X (float) 0
#define GUIDANCE_RIGHT_Y (float) 0.09
#define GUIDANCE_RIGHT_Z (float) 0.09

#define GUIDANCE_BACK_X (float) -0.13
#define GUIDANCE_BACK_Y (float) 0
#define GUIDANCE_BACK_Z (float) 0.09

#define GUIDANCE_LEFT_X (float) 0
#define GUIDANCE_LEFT_Y (float) -0.09
#define GUIDANCE_LEFT_Z (float) 0.09

#define GUIDANCE_DOWN_X (float) 0
#define GUIDANCE_DOWN_Y (float) 0
#define GUIDANCE_DOWN_Z (float) 0.12

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nh;
  ros::Rate rate(60); // 60Hz
  DJIDrone* drone = new DJIDrone(nh);

  tf::TransformBroadcaster br;
  tf::Quaternion q;

  // Dinamic frames
  tf::Transform tf_world_groundFrame;
  tf::Transform tf_groundFrame_bodyFrame;
  tf::Transform tf_bodyFrame_gimbalNED;
  tf::Transform tf_gimbalNED_gimbal;

  // Static Frames
  tf::Transform tf_gimbal_camera;
  tf::Transform tf_body_guidance_front;
  tf::Transform tf_body_guidance_right;
  tf::Transform tf_body_guidance_back;
  tf::Transform tf_body_guidance_left;
  tf::Transform tf_body_guidance_down;

  // gimbal - camera
  q.setRPY(C_PI/2, 0, C_PI/2);
  tf_gimbal_camera.setRotation(q);  
  tf_gimbal_camera.setOrigin(tf::Vector3(0, 0, 0));

  // body_frame - guidance front
  q.setRPY(C_PI/2, 0, C_PI/2);
  tf_body_guidance_front.setRotation(q);  
  tf_body_guidance_front.setOrigin(tf::Vector3(GUIDANCE_FRONT_X, GUIDANCE_FRONT_Y, GUIDANCE_FRONT_Z));
  // body_frame - guidance right
  q.setRPY(C_PI/2, 0, C_PI);
  tf_body_guidance_right.setRotation(q);  
  tf_body_guidance_right.setOrigin(tf::Vector3(GUIDANCE_RIGHT_X, GUIDANCE_RIGHT_Y, GUIDANCE_RIGHT_Z));
  // body_frame - guidance back
  q.setRPY(C_PI/2, 0, 3*C_PI/2);
  tf_body_guidance_back.setRotation(q);  
  tf_body_guidance_back.setOrigin(tf::Vector3(GUIDANCE_BACK_X, GUIDANCE_BACK_Y, GUIDANCE_BACK_Z));
  // body_frame - guidance left
  q.setRPY(C_PI/2, 0, 0);
  tf_body_guidance_left.setRotation(q);  
  tf_body_guidance_left.setOrigin(tf::Vector3(GUIDANCE_LEFT_X, GUIDANCE_LEFT_Y, GUIDANCE_LEFT_Z));
  // body_frame - guidance down
  q.setRPY(0, 0, C_PI/2);
  tf_body_guidance_down.setRotation(q);  
  tf_body_guidance_down.setOrigin(tf::Vector3(GUIDANCE_DOWN_X, GUIDANCE_DOWN_Y, GUIDANCE_DOWN_Z));

  while(nh.ok())
  {
      // ---------------------------------------------------------
      // world - ground_frame
      // world: North-West-Up
      // ground_frame: Nort-Est-Down, origin at m100's local_position
      // ---------------------------------------------------------

      tf_world_groundFrame.setOrigin(tf::Vector3(drone->local_position.x, drone->local_position.y, drone->local_position.z));
      q.setRPY(C_PI, 0, 0);
      tf_world_groundFrame.setRotation(q);
      br.sendTransform(tf::StampedTransform(tf_world_groundFrame, ros::Time::now(), "world", "ground_frame"));

      // ---------------------------------------------------------
      // ground_frame - body_frame
      // ground_frame: North-Est-Down, origin at m100's local_position
      // body_frame: front, right, down
      // ---------------------------------------------------------

      tf_groundFrame_bodyFrame.setOrigin( tf::Vector3(0, 0, 0) );
      q = tf::Quaternion(drone->attitude_quaternion.q1, drone->attitude_quaternion.q2, drone->attitude_quaternion.q3, drone->attitude_quaternion.q0);
      tf_groundFrame_bodyFrame.setRotation(q);
      br.sendTransform(tf::StampedTransform(tf_groundFrame_bodyFrame, ros::Time::now(), "ground_frame", "body_frame"));

      // ---------------------------------------------------------
      // body_frame - camera
      // body_frame: front m100, right m100, down m100
      // camera: front camera, left camera, up camera 
      // ---------------------------------------------------------

      tf_bodyFrame_gimbalNED = tf_groundFrame_bodyFrame.inverse();
      tf_bodyFrame_gimbalNED.setOrigin(tf::Vector3(CAMERA_X, CAMERA_Y, CAMERA_Z));
      br.sendTransform(tf::StampedTransform(tf_bodyFrame_gimbalNED, ros::Time::now(), "body_frame", "gimbal_NED"));

      tf::Transform tf_yaw;
      q.setRPY(0, 0, drone->gimbal.yaw/180*C_PI);
      tf_yaw.setRotation(q);
      tf_yaw.setOrigin(tf::Vector3(0, 0, 0));
      tf::Transform tf_pitch;
      q.setRPY(0, drone->gimbal.pitch/180*C_PI, 0);
      tf_pitch.setRotation(q);
      tf_pitch.setOrigin(tf::Vector3(0, 0, 0));
      tf::Transform tf_roll;
      q.setRPY(drone->gimbal.roll/180*C_PI, 0, 0);
      tf_roll.setRotation(q);
      tf_roll.setOrigin(tf::Vector3(0, 0, 0));

      tf_gimbalNED_gimbal = tf_yaw*tf_pitch*tf_roll;
      br.sendTransform(tf::StampedTransform(tf_gimbalNED_gimbal, ros::Time::now(), "gimbal_NED", "gimbal"));

      br.sendTransform(tf::StampedTransform(tf_gimbal_camera, ros::Time::now(), "gimbal", "camera"));

      // ---------------------------------------------------------
      // body_frame - guidance front, right, back, left, down
      // body_frame: front m100, right m100, down m100
      // guidance: front guidance, left guidance, up guidance 
      // ---------------------------------------------------------

      br.sendTransform(tf::StampedTransform(tf_body_guidance_front, ros::Time::now(), "body_frame", "guidance_front"));
      br.sendTransform(tf::StampedTransform(tf_body_guidance_right, ros::Time::now(), "body_frame", "guidance_right"));
      br.sendTransform(tf::StampedTransform(tf_body_guidance_back,  ros::Time::now(), "body_frame", "guidance_back"));
      br.sendTransform(tf::StampedTransform(tf_body_guidance_left,  ros::Time::now(), "body_frame", "guidance_left"));
      br.sendTransform(tf::StampedTransform(tf_body_guidance_down,  ros::Time::now(), "body_frame", "guidance_down"));

      ros::spinOnce();
      rate.sleep();
  }
  return 0;
};
