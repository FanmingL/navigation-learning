//
// Created by erdou on 18-8-28.
//
#include "main.h"

void cb(const nav_msgs::OccupancyGridConstPtr &msg)
{
    int width = msg->info.width, height = msg->info.height;
    float resolution = msg->info.resolution;
    auto p = msg->data.data();
    for (int i = 0; i < height; i++)
    {
        for (int ii = 0; ii < width; ii++)
        {
            std::cout<<(int)*(p + i * height + ii)<<"\t";
        }
        std::cout<<std::endl;
    }
    std::cout<<(int)*msg->data.data()<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_navigation");
    CarControl c;
    c.run();
    //ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, cb);
    //ros::spin();
    return 0;
}
