#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <dji_sdk/dji_drone.h>
#define C_PI (double) 3.141592653589793

void localPosition_callback(const dji_sdk::LocalPosition& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  //transform.setOrigin( tf::Vector3(msg.x, msg.y, -msg.z) );
  transform.setOrigin( tf::Vector3(2, 2, -2) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "local_position"));
}

void attitudeQuaternion_callback(const dji_sdk::AttitudeQuaternion& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  tf::Quaternion q(msg.q1, msg.q2, msg.q3, msg.q0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_position", "m100_attitude"));
}

void attitudeGimbal_callback(const dji_sdk::Gimbal& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.2, 0, 0.2) );
  tf::Quaternion q;
  q.setRPY(msg.roll/180*C_PI, msg.pitch/180*C_PI, msg.yaw/180*C_PI);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_position", "gimbal"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nh;
  ros::Subscriber local_position = nh.subscribe("/dji_sdk/local_position", 10, &localPosition_callback);
  ros::Subscriber attitude_quaternion = nh.subscribe("/dji_sdk/attitude_quaternion", 10, &attitudeQuaternion_callback);
  ros::Subscriber gimbal  = nh.subscribe("/dji_sdk/gimbal", 10, &attitudeGimbal_callback);
  ros::spin();
  return 0;
};
