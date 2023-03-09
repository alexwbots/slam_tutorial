#include <ros/ros.h>
#include "basic_slam_v1/slam.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "basic_slam_v1");

  BasicSlam bs;
  
  ros::spin();
  return 0;
}