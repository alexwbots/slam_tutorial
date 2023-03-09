#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/MapMetaData.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

class BasicSlam {
  
  public:
  	BasicSlam();
  	~BasicSlam();

    void initialization();

  private:

    ros::NodeHandle nh_;
    ros::Publisher meta_pub_;
    ros::Subscriber odom_sub_;
    void odom_cb(const nav_msgs::Odometry& odom_msg);
    ros::Subscriber scan_sub_;
    void scan_cb(const sensor_msgs::LaserScan& scan_msg);

    double pose_x_;
    double pose_y_;
    double pose_th_;
};