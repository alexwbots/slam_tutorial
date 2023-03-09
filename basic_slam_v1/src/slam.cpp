#include "basic_slam_v1/slam.h"


BasicSlam::BasicSlam(){
  initialization();
  std::cout << "Hola" << std::endl;
}

BasicSlam::~BasicSlam(){
  std::cout << "Destroyer" << std::endl;
}

void BasicSlam::initialization(){
  ROS_INFO("Initialization of the class");
}