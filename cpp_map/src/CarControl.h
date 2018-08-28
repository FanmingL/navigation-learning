//
// Created by erdou on 18-8-28.
//

#ifndef CPP_MAP_CARCONTROL_H
#define CPP_MAP_CARCONTROL_H

#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <string>
#include "geometry_msgs/Twist.h"
#include "PID.h"
#include "tf/transform_broadcaster.h"
#include <cmath>

class CarControl {
public:
    CarControl();
    void run();
    void map_cb(const nav_msgs::OccupancyGridConstPtr &msg);
    void robot_pos_cb(const geometry_msgs::PoseStampedConstPtr &msg);
private:
    ros::NodeHandle nh_;
    ros::Publisher control_pub;
    ros::Subscriber map_sub, robot_pos_sub;
    nav_msgs::OccupancyGrid map;
    geometry_msgs::Twist robot_control;
    geometry_msgs::PoseStamped robot_pose;
    PID yaw_pid, dis_pid;
    bool map_got;

};


#endif //CPP_MAP_CARCONTROL_H
