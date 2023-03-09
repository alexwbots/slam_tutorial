#include "basic_slam_v6/slam.h"

#define PI 3.1416

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
  initialize_ = true;
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

  map_width_ = 7;
  map_height_ = 7;
  map_resolution_ = 0.05;
  bot_origin_x_ = round(map_width_/2);
  bot_origin_y_ = round(map_height_/2);

  // Map meta data information
  mapmetadata_.resolution = 0.05;                                       // [m/cell]
  mapmetadata_.width = map_width_;                                      // [cells]
  mapmetadata_.height = map_height_;                                    // [cells]
  mapmetadata_.origin.position.x = map_width_*(-0.5)*map_resolution_;   // The origin of the map 
  mapmetadata_.origin.position.y = map_height_*(-0.5)*map_resolution_;  // [m, m, rad]
  mapmetadata_.origin.orientation.w = 1.0;

  meta_pub_.publish(mapmetadata_);

  // Building the map
  map_.header.stamp = ros::Time::now();
  map_.header.frame_id = "map";
  map_.info = mapmetadata_;
  map_.data.resize(map_width_*map_height_);

  for(int j = 0; j < map_width_; j++){
    for(int i = 0; i < map_height_; i++){

      double relative_pos_x_ = double(i - bot_origin_x_);
      double relative_pos_y_ = double(j - bot_origin_y_);
      double angle_target_ = atan2(relative_pos_y_,relative_pos_x_);
      int angle_idx_ = round(angle_correction(angle_target_)/scan_msg.angle_increment);
      double range_ = scan_msg.ranges[angle_idx_];
      std::cout << "[" << angle_idx_ << "]: " << range_ << "\t";
      
      double relative_distance_ = sqrt(pow(relative_pos_y_,2) + pow(relative_pos_x_,2));
      if (range_ > relative_distance_){
        map_.data[i + map_width_*j] = 0;
      } else {
        map_.data[i + map_width_*j] = 100;
      }
    }
    std::cout << std::endl;
  }

  map_pub_.publish(map_);
  initialize_ = false;
  std::cout << "END" << std::endl;
}

double BasicSlam::angle_correction(double angle){

  if(angle >= 0 and angle <= 2*PI) return (2*PI - angle);
  else return (-angle);
}