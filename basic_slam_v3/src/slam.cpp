#include "basic_slam_v3/slam.h"


BasicSlam::BasicSlam(){
  initialization();
}

BasicSlam::~BasicSlam(){
  std::cout << "Destroyer" << std::endl;
}

void BasicSlam::initialization(){
  ROS_INFO("Initialization of the class");
  
  odom_sub_ = nh_.subscribe("odom",1,&BasicSlam::odom_cb,this);
  scan_sub_ = nh_.subscribe("scan",1,&BasicSlam::scan_cb,this);
  meta_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

  pose_th_ = 0.0;
}

void BasicSlam::odom_cb(const nav_msgs::Odometry& odom_msg){
	pose_x_ = odom_msg.pose.pose.position.x;
	pose_y_ = odom_msg.pose.pose.position.y;

  double pitch, roll;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, pose_th_);
}

void BasicSlam::scan_cb(const sensor_msgs::LaserScan& scan_msg){
	float var_scan = scan_msg.angle_increment;
  ROS_INFO("th position: %f", pose_th_);
  //ROS_INFO("time_increment: %f",var_scan);
}