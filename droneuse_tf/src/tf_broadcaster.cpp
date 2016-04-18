#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <dji_sdk/dji_drone.h>
#define C_PI (double) 3.141592653589793

/*
void localPosition_callback(const dji_sdk::LocalPosition& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  //transform.setOrigin( tf::Vector3(msg.x, msg.y, msg.z) );
  transform.setOrigin( tf::Vector3(2, 2, 2) );
  tf::Quaternion q;
  q.setRPY(C_PI, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ground_frame"));
}

void attitudeQuaternion_callback(const dji_sdk::AttitudeQuaternion& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  tf::Quaternion q(msg.q1, msg.q2, msg.q3, msg.q0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ground_frame", "body_frame"));
}

void attitudeGimbal_callback(const dji_sdk::Gimbal& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  tf::Quaternion q;
  q.setRPY(msg.roll/180*C_PI, msg.pitch/180*C_PI, msg.yaw/180*C_PI);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body_frame", "gimbal"));
}
*/

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nh;
  ros::Rate rate(10); // 10Hz
  DJIDrone* drone = new DJIDrone(nh);

  //ros::Subscriber local_position = nh.subscribe("/dji_sdk/local_position", 10, &localPosition_callback);
  //ros::Subscriber attitude_quaternion = nh.subscribe("/dji_sdk/attitude_quaternion", 10, &attitudeQuaternion_callback);
  //ros::Subscriber gimbal  = nh.subscribe("/dji_sdk/gimbal", 10, &attitudeGimbal_callback);

  tf::TransformBroadcaster br;

  tf::Transform tf_world_groundFrame;
  tf::Transform tf_groundFrame_bodyFrame;
  tf::Transform tf_bodyFrame_gimbalHorizon;
  tf::Transform tf_gimbalNED_gimbalHorizon;
  tf::Transform tf_gimbalHorizon_gimbal;
  tf::Transform tf_groundFrame_gimbalNED;

  tf::Quaternion q;
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
      // body_frame - gimbal
      // body_frame: front, right, down
      // gimbal: North-East-Down, origin at gimbal
      // ---------------------------------------------------------

      //q.setRPY(drone->gimbal.roll/180*C_PI, drone->gimbal.pitch/180*C_PI, drone->gimbal.yaw/180*C_PI);
      q.setRPY(0, 0, 0);
      tf_groundFrame_gimbalNED.setRotation(q);
      tf_bodyFrame_gimbalHorizon.setOrigin(tf::Vector3(0.2, 0, 0.2));
      br.sendTransform(tf::StampedTransform(tf_bodyFrame_gimbalHorizon, ros::Time::now(), "body_frame", "gimbal_NED"));

      q.setRPY(0, 0, drone->gimbal.yaw/180*C_PI);
      tf_groundFrame_gimbalNED.setRotation(q);
      tf_bodyFrame_gimbalHorizon = tf_groundFrame_gimbalHorizon*tf_groundFrame_bodyFrame.inverse();
      tf_bodyFrame_gimbalHorizon.setOrigin(tf::Vector3(0, 0, 0));
      br.sendTransform(tf::StampedTransform(tf_bodyFrame_gimbalHorizon, ros::Time::now(), "gimbal_NED", "gimbal_horizon"));

      q.setRPY(drone->gimbal.roll/180*C_PI, drone->gimbal.pitch/180*C_PI, 0);
      tf_gimbalHorizon_gimbal.setRotation(q);  
      tf_gimbalHorizon_gimbal.setOrigin(tf::Vector3(0, 0, 0));
      br.sendTransform(tf::StampedTransform(tf_gimbalHorizon_gimbal, ros::Time::now(), "gimbal_horizon", "gimbal"));

      ros::spinOnce();
      rate.sleep();
  }
  return 0;
};
