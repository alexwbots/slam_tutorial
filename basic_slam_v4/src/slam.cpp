#include "basic_slam_v4/slam.h"


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
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
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

  // Map meta data information
  mapmetadata_.resolution = 0.5;
  mapmetadata_.width = 3;
  mapmetadata_.height = 3;
  mapmetadata_.origin.position.x = -0.75;
  mapmetadata_.origin.position.y = -0.75;
  mapmetadata_.origin.orientation.w = 1.0;

  meta_pub_.publish(mapmetadata_);

  // Building the map
  map_.header.stamp = ros::Time::now();
  map_.header.frame_id = "map";
  map_.info = mapmetadata_;

  int map[9] = {0,0,0,0,0,0,0,0,0};

  map_.data = {0,0,0,0,0,0,0,0,0};
  map_pub_.publish(map_);
}