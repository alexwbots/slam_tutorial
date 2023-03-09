#include "basic_slam_v5/slam.h"


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

  // Map meta data information
  mapmetadata_.resolution = 0.05;           // [m/cell]
  mapmetadata_.width = map_width_;          // [cells]
  mapmetadata_.height = map_height_;        // [cells]
  mapmetadata_.origin.position.x = map_width_*(-0.5)*map_resolution_;    // The origin of the map 
  mapmetadata_.origin.position.y = map_height_*(-0.5)*map_resolution_;    // [m, m, rad]
  mapmetadata_.origin.orientation.w = 1.0;

  meta_pub_.publish(mapmetadata_);

  // Building the map
  map_.header.stamp = ros::Time::now();
  map_.header.frame_id = "map";
  map_.info = mapmetadata_;

  int map_arr_[map_width_][map_height_] = {0};

  float range_max_ = scan_msg.range_max;
  int len_range_ = round(scan_msg.angle_max/scan_msg.angle_increment);
  for(int i = 0; i < len_range_; i++){
    if(scan_msg.intensities[i] == 0.0){
      if(scan_msg.ranges[i] >= scan_msg.range_min && scan_msg.ranges[i] <= scan_msg.range_max){
        std::cout << "Ray[" << i << "]: " << scan_msg.ranges[i] << std::endl;

      }
    }
  }

  map_.data.resize(map_width_*map_height_);
  map_.data[0] = -1;
  map_.data[1] = -1;
  map_.data[2] = -1;
  map_.data[3] = -1;
  
  if(initialize_){
    for(int j = 0; j < map_width_; j++){
      for(int i = 0; i < map_height_; i++){
        std::cout << map_.data[i + map_width_*j] << " ";
      }
      std::cout << std::endl;
    }
  }

  map_pub_.publish(map_);
  initialize_ = false;
}