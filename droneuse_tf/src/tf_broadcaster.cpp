#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <dji_sdk/dji_drone.h>
#define C_PI (double) 3.141592653589793

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nh;
  ros::Rate rate(10); // 10Hz
  DJIDrone* drone = new DJIDrone(nh);

  tf::TransformBroadcaster br;
  tf::Quaternion q;

  // Dinamic frames
  tf::Transform tf_world_groundFrame;
  tf::Transform tf_groundFrame_bodyFrame;
  tf::Transform tf_bodyFrame_gimbalNED;
  tf::Transform tf_gimbalNED_gimbalHorizon;
  tf::Transform tf_gimbalHorizon_gimbal;
  tf::Transform tf_gimbal_camera;

  // Static Frames
  tf::Transform tf_body_guidance_front;
  tf::Transform tf_body_guidance_right;
  tf::Transform tf_body_guidance_back;
  tf::Transform tf_body_guidance_left;
  tf::Transform tf_body_guidance_down;

  // gimbal - camera
  q.setRPY(C_PI, 0, 0);
  tf_gimbal_camera.setRotation(q);  
  tf_gimbal_camera.setOrigin(tf::Vector3(0, 0, 0));

  // body_frame - guidance front
  q.setRPY(C_PI, 0, 0);
  tf_body_guidance_front.setRotation(q);  
  tf_body_guidance_front.setOrigin(tf::Vector3(1, 0, -1));
  // body_frame - guidance right
  q.setRPY(C_PI, 0, C_PI/2);
  tf_body_guidance_right.setRotation(q);  
  tf_body_guidance_right.setOrigin(tf::Vector3(0, 1, 1));
  // body_frame - guidance back
  q.setRPY(C_PI, 0, C_PI);
  tf_body_guidance_back.setRotation(q);  
  tf_body_guidance_back.setOrigin(tf::Vector3(-1, 0, 1));
  // body_frame - guidance left
  q.setRPY(C_PI, 0, 3*C_PI/2);
  tf_body_guidance_left.setRotation(q);  
  tf_body_guidance_left.setOrigin(tf::Vector3(0, -1, 1));
  // body_frame - guidance down
  q.setRPY(C_PI, -C_PI/2, 0);
  tf_body_guidance_down.setRotation(q);  
  tf_body_guidance_down.setOrigin(tf::Vector3(0, 0, 1));

  while(nh.ok())
  {
      // ---------------------------------------------------------
      // world - ground_frame
      // world: North-West-Up
      // ground_frame: Nort-Est-Down, origin at m100's local_position
      // ---------------------------------------------------------

      //transform.setOrigin(tf::Vector3(drone->local_position.x, drone->local_position.y, drone->local_position.z));
      tf_world_groundFrame.setOrigin(tf::Vector3(3, 3, 3));
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
      // body_frame - camer
      // body_frame: front m100, right m100, down m100
      // camera: front camera, left camera, up camera 
      // ---------------------------------------------------------

      tf_bodyFrame_gimbalNED = tf_groundFrame_bodyFrame.inverse();
      tf_bodyFrame_gimbalNED.setOrigin(tf::Vector3(0.2, 0, 0.2));
      br.sendTransform(tf::StampedTransform(tf_bodyFrame_gimbalNED, ros::Time::now(), "body_frame", "gimbal_NED"));

      q.setRPY(0, 0, drone->gimbal.yaw/180*C_PI);
      tf_gimbalNED_gimbalHorizon.setRotation(q);
      tf_gimbalNED_gimbalHorizon.setOrigin(tf::Vector3(0, 0, 0));
      br.sendTransform(tf::StampedTransform(tf_gimbalNED_gimbalHorizon, ros::Time::now(), "gimbal_NED", "gimbal_horizon"));

      q.setRPY(drone->gimbal.roll/180*C_PI, drone->gimbal.pitch/180*C_PI, 0);
      tf_gimbalHorizon_gimbal.setRotation(q);  
      tf_gimbalHorizon_gimbal.setOrigin(tf::Vector3(0, 0, 0));
      br.sendTransform(tf::StampedTransform(tf_gimbalHorizon_gimbal, ros::Time::now(), "gimbal_horizon", "gimbal"));

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
