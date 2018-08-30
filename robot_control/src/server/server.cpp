//
// Created by erdou on 18-8-28.
//

#include "server.h"

robot_server::robot_server() :
    yaw_config_path("/home/erdou/workspace/src/robot_control/config/yaw_pid.config"),
    pos_config_path("/home/erdou/workspace/src/robot_control/config/pos_pid.config"),
    ac(nh_, "robot_action", boost::bind(&robot_server::ActionCB, this, _1), false),
    target_set(false)/*,
    tr_ls(ros::Duration(50))*/
{
    control_pub = nh_.advertise<geometry_msgs::Twist>("robot_control", 1);
    if (read_from_file(yaw_config_path))ROS_INFO("yaw config ok");
    yaw_pid = new PID(pid_config.kp(), pid_config.ki(), pid_config.kd(),
            pid_config.inte_lim(), pid_config.out_lim(), pid_config.lp_hz());
    if (read_from_file(pos_config_path))ROS_INFO("position config ok");
    dis_pid = new PID(pid_config.kp(), pid_config.ki(), pid_config.kd(),
                  pid_config.inte_lim(), pid_config.out_lim(), pid_config.lp_hz());
    robot_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>
            ("robot_pos", 1, boost::bind(&robot_server::robot_pos_cb, this, _1));
    goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    map_pose.header.stamp = ros::Time::now();
    map_pose.header.frame_id = "map";
    map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
}

void robot_server::run() {
    ac.start();
    ros::spin();
}

void robot_server::send_br(const geometry_msgs::PoseStamped &msg, const char *name, const char * child_name) {
    trans.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
    q.setValue(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    trans.setRotation(q);
    br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), name, child_name));
}

void robot_server::robot_pos_cb(const geometry_msgs::PoseStampedConstPtr &msg) {
    robot_pose = *msg;
    send_br(robot_pose, "world", "base_link");
    send_br(map_pose, "world", "map");
    if (!target_set)
    {
        robot_control_msg.angular.z = 0.0;
        robot_control_msg.linear.x = 0.0;
        control_pub.publish(robot_control_msg);
        return;
    }
    robot_target.header.stamp = msg->header.stamp;
    //robot_target.header.frame_id = "world";
    send_br(robot_target, "world", "target");

    try {
        tr_ls.lookupTransform("base_link", "target", ros::Time(0), target_to_robot);
    }
    catch(tf::TransformException &ex){
        ROS_WARN(" exception received :%s ", ex.what());
        robot_control_msg.angular.z = 0.0;
        robot_control_msg.linear.x = 0.0;
        control_pub.publish(robot_control_msg);
        return;
    }
    float x_t = (float)robot_target.pose.position.x, y_t = (float)robot_target.pose.position.y;
    static auto t0 = msg->header.stamp;
    auto period = (float)(msg->header.stamp.toSec() - t0.toSec());
    t0 = msg->header.stamp;
    auto yaw = (float)atan2(target_to_robot.getOrigin().y(), target_to_robot.getOrigin().x());
    robot_control_msg.angular.z = yaw_pid->step(0, yaw, period);
    auto dist = -(float)target_to_robot.getOrigin().x();
    robot_control_msg.linear.x = dis_pid->step(0, dist, period);
    if (fabsf(dist) < 0.03 && (float)sqrt(pow(msg->pose.position.x - x_t, 2) + pow(msg->pose.position.y - y_t, 2)) < 0.3)
    {
        target_set = false;
        yaw_pid->reset();
        dis_pid->reset();
    }
    if ((float)sqrt(pow(target_to_robot.getOrigin().x(), 2) + pow(target_to_robot.getOrigin().y(), 2)) < 0.3)
    {
        robot_control_msg.angular.z = 0.0f;
    }
    control_pub.publish(robot_control_msg);
}


bool robot_server::read_from_file(const char *path) {
    return robot_io::read_proto_from_text(path, &pid_config);
}

void robot_server::ActionCB(const robot_control::SetTargetGoalConstPtr &msg) {
    robot_target = msg->goal;
    //robot_target.header.frame_id = "map";
    target_set = true;
    robot_control::SetTargetFeedback fb;
    auto rate = ros::Rate(3);
    while (target_set && ros::ok())
    {
        fb.error_code = 0;
        fb.position = robot_pose;
        ac.publishFeedback(fb);
        goal_pub.publish(robot_target);
        rate.sleep();
    }
    ac.setSucceeded();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_server");
    robot_server client;
    client.run();
    return 0;
}
