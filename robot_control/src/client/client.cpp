/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : client.cpp
*   Author      : FanmingL
*   Created date: 2018-08-29 11:16:53
*   Description : 
*
*===============================================================*/


#include "client.h"

robot_client::robot_client() : ac("robot_action", true) , state(STATE_IDLE),
file_path("/home/erdou/workspace/src/robot_control/config/path.config")
{
    if (get_from_text(file_path))ROS_INFO("path data loaded!");
    for (int i = 0; i < config_proto.point_size(); i++)
    {
        std::vector<float> _tmp = {config_proto.point(i).x(), config_proto.point(i).y()};
        path.push_back(_tmp);
    }
    //ROS_INFO("point num is %d", config_proto.point_size())
    ROS_INFO("point num is %d", (int)path.size());
    ROS_INFO("Wait For Server");
    ac.waitForServer();
    ROS_INFO("Connect Successfully!");
}

void robot_client::run() {
    for (int i = 0; i < path.size() && ros::ok(); i++){
        ROS_INFO("The %dth point, (%.2f, %.2f)", i+1, path[i][0], path[i][1]);
        goal.goal.header.stamp = ros::Time::now();
        goal.goal.header.frame_id = "map";
        goal.goal.header.seq ++;
        goal.command = 0;
        goal.goal.pose.position.x = path[i][0];
        goal.goal.pose.position.y = path[i][1];
        goal.goal.pose.orientation.z = 1;
        ac.sendGoal(goal,
                boost::bind(&robot_client::ac_done, this, _1, _2),
                boost::bind(&robot_client::ac_active, this),
                boost::bind(&robot_client::ac_fb, this, _1));
        ac.waitForResult();
        if (i == path.size()-1)
            i = -1;
    }
}

void robot_client::ac_active() {
    state = STATE_RUNNING;
    //ROS_INFO("start action");
}

void robot_client::ac_fb(const robot_control::SetTargetFeedbackConstPtr &msg) {

}

void robot_client::ac_done(const actionlib::SimpleClientGoalState &msg,
                           const robot_control::SetTargetResultConstPtr &res) {
    state = STATE_IDLE;
    //ROS_INFO("finished task");
}

bool robot_client::get_from_text(const char *file_path) {
    return robot_io::read_proto_from_text(file_path, &config_proto);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_client");
    robot_client rc;
    //ros::Rate(1).sleep();
    rc.run();
    return 0;
}

