//
// Created by erdou on 18-8-28.
//

#include "server.h"

CarControl::CarControl() : map_got(false),
    yaw_config_path("/home/erdou/workspace/src/cpp_map/config/yaw_pid.config"),
    pos_config_path("/home/erdou/workspace/src/cpp_map/config/pos_pid.config")
{
    control_pub = nh_.advertise<geometry_msgs::Twist>("robot_control", 1);
    if (read_from_file(yaw_config_path))ROS_INFO("yaw config ok");
    yaw_pid = new PID(pid_config.kp(), pid_config.ki(), pid_config.kd(),
            pid_config.inte_lim(), pid_config.out_lim(), pid_config.lp_hz());
    if (read_from_file(pos_config_path))ROS_INFO("position config ok");
    dis_pid = new PID(pid_config.kp(), pid_config.ki(), pid_config.kd(),
                  pid_config.inte_lim(), pid_config.out_lim(), pid_config.lp_hz());

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
    robot_control.angular.z = yaw_pid->step(yaw_feedback, yaw, period);
    auto dist = (float)sqrt((pow(msg->pose.position.x-x_t, 2)));
    robot_control.linear.x = dis_pid->step(0, dist, period);
    if (dist < 0.1)
    {
        robot_control.angular.z = 0.0f;
    }
    control_pub.publish(robot_control);
}


bool CarControl::read_from_file(const char *path) {
    using google::protobuf::io::FileInputStream;
    using google::protobuf::io::FileOutputStream;
    using google::protobuf::io::ZeroCopyInputStream;
    using google::protobuf::io::CodedInputStream;
    using google::protobuf::io::ZeroCopyOutputStream;
    using google::protobuf::io::CodedOutputStream;
    using google::protobuf::Message;

    int fd = open(path, O_RDONLY);
    if (fd == -1){
        ROS_WARN("No Such File");
        return false;
    }
    FileInputStream *input = new FileInputStream(fd);
    bool success = google::protobuf::TextFormat::Parse(input, &pid_config);
    delete input;
    close(fd);
    return success;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_server");
    CarControl client;
    client.run();
    return 0;
}
