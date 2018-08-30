/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : global_planner.cpp
*   Author      : FanmingL
*   Created date: 2018-08-29 12:19:27
*   Description : 
*
*===============================================================*/


#include "global_planner.h"

global_planner::global_planner() : map_got(false) {
    map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&global_planner::mapCB, this, _1));
    path_pub = nh_.advertise<nav_msgs::Path>("robot_path", 1);
}

void global_planner::run() {
    ros::spin();
}

void global_planner::mapCB(const nav_msgs::OccupancyGridConstPtr &msg) {
    map_got = true;
    map = *msg;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");
    global_planner pg;
    pg.run();
    return 0;
}
