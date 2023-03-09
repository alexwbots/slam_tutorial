#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class BasicSlam {
  
  public:
  	BasicSlam();
  	~BasicSlam();

    void initialization();

  private:
    ros::NodeHandle nh_;
    //ros::Publisher pub_;
    ros::Subscriber odom_sub_;
    void odom_cb(const nav_msgs::Odometry& odom_msg);
};