/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : global_planner.h
*   Author      : FanmingL
*   Created date: 2018-08-29 12:19:46
*   Description : 
*
*===============================================================*/


#ifndef _GLOBAL_PLANNER_H
#define _GLOBAL_PLANNER_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

class global_planner{
public:
    global_planner();
    void run();
    void mapCB(const nav_msgs::OccupancyGridConstPtr &msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub;
    nav_msgs::OccupancyGrid map;
    nav_msgs::Path path;
    ros::Publisher path_pub;

    bool map_got;
};
#endif //GLOBAL_PLANNER_H
