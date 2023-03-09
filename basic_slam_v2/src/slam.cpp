#include "basic_slam_v2/slam.h"


BasicSlam::BasicSlam(){
  initialization();
  std::cout << "Init the publisher and subscribers" << std::endl;
  odom_sub_ = nh_.subscribe("odom",1,&BasicSlam::odom_cb,this);
}

BasicSlam::~BasicSlam(){
  std::cout << "Destroyer" << std::endl;
}

void BasicSlam::initialization(){
  ROS_INFO("Initialization of the class");
}

void BasicSlam::odom_cb(const nav_msgs::Odometry& odom_msg){
	double var = odom_msg.pose.pose.position.x;
  ROS_INFO("x position: %f",var);
}