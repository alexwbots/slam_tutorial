#include <ros/ros.h>
#include "basic_slam_v3/slam.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "basic_slam");

  BasicSlam bs;
  
  ros::spin();
  return 0;
}