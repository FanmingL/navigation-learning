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
#include "tf/transform_listener.h"
#include <cmath>
#include "pid.pb.h"
#include "PID.h"
#include "common_io.h"
#include "actionlib/server/simple_action_server.h"
#include "robot_control/SetTargetAction.h"


class robot_server {
public:
    robot_server();
    void run();
    void robot_pos_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    bool read_from_file(const char *path);
    void ActionCB(const robot_control::SetTargetGoalConstPtr &msg);
    void send_br(const geometry_msgs::PoseStamped &msg, const char *name, const char *child_name);
private:
    ros::NodeHandle nh_;
    ros::Publisher control_pub;
    ros::Subscriber map_sub, robot_pos_sub, target_pos_sub;
    geometry_msgs::Twist robot_control_msg;
    geometry_msgs::PoseStamped robot_pose, robot_target;
    tf::StampedTransform target_to_robot;
    bool target_set;
    ros_pid::Pid pid_config;
    tf::Transform trans;
    tf::Quaternion q;
    PID* yaw_pid, *dis_pid;
    const char *yaw_config_path, *pos_config_path;
    actionlib::SimpleActionServer<robot_control::SetTargetAction> ac;
    tf::TransformBroadcaster br;
    tf::TransformListener tr_ls;
    /* /home/erdou/workspace/src/cpp_map/config/yaw_pid.config */

};


#endif //SERVER_H
