//
// Created by erdou on 18-8-28.
//

#include "CarControl.h"

CarControl::CarControl() : map_got(false),
    yaw_pid(70, 1.2, 8, 20, 100, 20),
    dis_pid(30, 0, 8, 10, 60, 50)
{
    control_pub = nh_.advertise<geometry_msgs::Twist>("robot_control", 1);
    map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>
            ("/map", 1, boost::bind(&CarControl::map_cb, this, _1));
    robot_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>
            ("robot_pos", 1, boost::bind(&CarControl::robot_pos_cb, this, _1));
    while (ros::ok())
    {
        ros::spinOnce();
        if (map_got)
            break;
    }
}

void CarControl::run() {
    /*ros::Rate rate(10);
    while (ros::ok())
    {
        robot_control.linear.x = 10.0f;
        robot_control.angular.z = 2.0f;
        pub.publish(robot_control);
        rate.sleep();
    }*/
    ros::spin();
}

void CarControl::map_cb(const nav_msgs::OccupancyGridConstPtr &msg) {
    map = *msg;
    map_got = true;
    ROS_INFO("get map!");
}

void CarControl::robot_pos_cb(const geometry_msgs::PoseStampedConstPtr &msg) {
    robot_pose = *msg;
    float x_t = -1.5f, y_t = -1.5f;
    static auto t0 = msg->header.stamp;
    auto period = (float)(msg->header.stamp.toSec() - t0.toSec());
    auto yaw_feedback = (float)tf::getYaw(msg->pose.orientation);
    t0 = msg->header.stamp;
    auto yaw = (float)atan2(msg->pose.position.y - y_t, msg->pose.position.x - x_t);
    robot_control.angular.z = yaw_pid.step(yaw_feedback, yaw, period);
    //ROS_INFO("%f, %f", yaw_feedback, yaw);
    auto dist = (float)sqrt((pow(msg->pose.position.x-x_t, 2)));
    robot_control.linear.x = dis_pid.step(0, dist, period);
    if (dist < 0.4)
    {
        robot_control.angular.z = 0.0f;
    }
    //robot_control.linear.x = 0;
    control_pub.publish(robot_control);
}