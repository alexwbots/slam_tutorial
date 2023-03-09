#include <vector>
#include <iostream>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_datatypes.h>

class BasicSlam {
  
  public:
  	BasicSlam();
  	~BasicSlam();

    void initialization();

  private:

    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Publisher meta_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;

    void odom_cb(const nav_msgs::Odometry& odom_msg);
    void scan_cb(const sensor_msgs::LaserScan& scan_msg);

    double pose_x_;
    double pose_y_;
    double pose_th_;

    int map_width_;
    int map_height_;
    float map_resolution_;

    bool initialize_;

    nav_msgs::MapMetaData mapmetadata_;
    nav_msgs::OccupancyGrid map_;

};