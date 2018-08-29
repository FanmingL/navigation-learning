/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : client.h
*   Author      : FanmingL
*   Created date: 2018-08-29 11:17:18
*   Description : 
*
*===============================================================*/


#ifndef _CLIENT_H
#define _CLIENT_H
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "robot_control/SetTargetAction.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "pos.pb.h"
#include "common_io.h"

class robot_client{
public:
    robot_client();
    void run();
    void ac_active();
    void ac_done(const actionlib::SimpleClientGoalState &msg, const robot_control::SetTargetResultConstPtr &res);
    void ac_fb(const robot_control::SetTargetFeedbackConstPtr &msg);
    bool get_from_text(const char *file_path);

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<robot_control::SetTargetAction> ac;
    robot_control::SetTargetGoal goal;
    std::vector<std::vector<float> > path;
    ros_pos::path config_proto;
    const char *file_path;
    enum {
        STATE_IDLE = 0,
        STATE_RUNNING
    };
    int state;
};

#endif //CLIENT_H
