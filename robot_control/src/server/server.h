//
// Created by erdou on 18-8-28.
//

#ifndef SERVER_H
#define SERVER_H

#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <string>
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include <cmath>
#include <fcntl.h>
#include "pid.pb.h"
#include "PID.h"

#include <google/protobuf/message.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

class CarControl {
public:
    CarControl();
    void run();
    void map_cb(const nav_msgs::OccupancyGridConstPtr &msg);
    void robot_pos_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    bool read_from_file(const char *path);
    void target_pos_cb(const geometry_msgs::TwistConstPtr &msg);
private:
    ros::NodeHandle nh_;
    ros::Publisher control_pub;
    ros::Subscriber map_sub, robot_pos_sub, target_pos_sub;
    nav_msgs::OccupancyGrid map;
    geometry_msgs::Twist robot_control;
    geometry_msgs::PoseStamped robot_pose;
    ros_pid::Pid pid_config;
    PID* yaw_pid, *dis_pid;
    const char *yaw_config_path, *pos_config_path;
    /* /home/erdou/workspace/src/cpp_map/config/yaw_pid.config */
    bool map_got;

};


#endif //SERVER_H
