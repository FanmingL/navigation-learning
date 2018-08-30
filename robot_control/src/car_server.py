#! /usr/bin/python
import pybullet as p
import pybullet_data
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np
import time


class CarServer:

    def __init__(self, mode=p.VELOCITY_CONTROL):
        rospy.init_node('car_server')
        self.start_position = [0.87, 1.15, 1]
        self.start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        p.connect(p.GUI)
        p.setRealTimeSimulation(1)
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.setTimeStep(1./480.)
        self.plane_id = p.loadURDF('plane100.urdf')
        self.robot_id = p.loadURDF('husky/husky.urdf', self.start_position, self.start_orientation)
        p.addUserDebugText('Car Position', [0, 0, 0.5], textColorRGB=[1, 0, 0],
                           parentObjectUniqueId=self.robot_id, parentLinkIndex=0)
        """
        0 LF     1 RF
            \     /
             \   /
               o
             /   \ 
            /     \ 
        3 LB     2 RB
        """
        self.RFW = 3
        self.RBW = 5
        self.LFW = 2
        self.LBW = 4
        self.INDEX_LF = 0
        self.INDEX_RF = 1
        self.INDEX_RB = 2
        self.INDEX_LB = 3
        self.motor_dictionary = [self.LFW, self.RFW, self.RBW, self.LBW]
        self.SPACE = 32l
        self.motor_target_velocity = [0, 0, 0, 0]       # rad per second
        self.motor_target_force = [0, 0, 0, 0]          # Newton
        self.motor_target_position = [0, 0, 0, 0]
        const_force = 40
        self.max_force = [const_force, const_force, const_force, const_force]           # Newton
        self.pub_pos = rospy.Publisher('robot_pos', PoseStamped, queue_size=1)
        self.pos_msg = PoseStamped()
        self.mode = mode
        self.velocity_x = 0
        self.velocity_rotate = 0
        self.motor_const_velocity = 30
        # self.text_handler = p.addUserDebugText(' ', [0, 0, 0], textColorRGB=[1, 0, 1])
        # self.sub = rospy.Subscriber('robot_control', Twist, self.cb,  queue_size=1)
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cb,  queue_size=1)

    def cb(self, msg):
        self.velocity_rotate = -msg.angular.z * 8
        self.velocity_x = msg.linear.x * 20
        self.set_motor()

    def pub_msg(self):
        robot_state = p.getBasePositionAndOrientation(self.robot_id)
        self.pos_msg.pose.orientation.x = robot_state[1][0]
        self.pos_msg.pose.orientation.y = robot_state[1][1]
        self.pos_msg.pose.orientation.z = robot_state[1][2]
        self.pos_msg.pose.orientation.w = robot_state[1][3]
        self.pos_msg.pose.position.x = robot_state[0][0]
        self.pos_msg.pose.position.y = robot_state[0][1]
        self.pos_msg.pose.position.z = robot_state[0][2]
        self.pos_msg.header.seq = self.pos_msg.header.seq + 1
        self.pos_msg.header.stamp = rospy.Time.now()
        self.pos_msg.header.frame_id = 'world'
        self.pub_pos.publish(self.pos_msg)

    def get_joint(self):
        for i in range(self.joint_num):
            print p.getJointInfo(self.robot_id, i)

    def set_motor(self):
        self.motor_target_velocity[self.INDEX_LF] = self.velocity_x + self.velocity_rotate
        self.motor_target_velocity[self.INDEX_RF] = self.velocity_x - self.velocity_rotate
        self.motor_target_velocity[self.INDEX_LB] = self.velocity_x + self.velocity_rotate
        self.motor_target_velocity[self.INDEX_RB] = self.velocity_x - self.velocity_rotate
        p.setJointMotorControlArray(self.robot_id, self.motor_dictionary,
                                    controlMode=self.mode, targetVelocities=self.motor_target_velocity,
                                    forces=self.max_force)

    def set_speed_from_keyboard(self):
        keys = p.getKeyboardEvents()
        self.velocity_x = 0
        self.velocity_rotate = 0
        if p.B3G_UP_ARROW in keys:
            self.velocity_x = self.velocity_x + 1
        if p.B3G_LEFT_ARROW in keys:
            self.velocity_rotate = self.velocity_rotate - 1
        if p.B3G_DOWN_ARROW in keys:
            self.velocity_x = self.velocity_x - 1
        if p.B3G_RIGHT_ARROW in keys:
            self.velocity_rotate = self.velocity_rotate + 1
        self.velocity_x = self.velocity_x * self.motor_const_velocity
        self.velocity_rotate = self.velocity_rotate * self.motor_const_velocity
        self.set_motor()

    def dog(self):
        keys = p.getKeyboardEvents()
        if self.SPACE in keys:
            p.resetBasePositionAndOrientation(self.robot_id, self.start_position, self.start_orientation)
            return True
        return False


if __name__ == '__main__':
    t = CarServer()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        t.dog()
        t.pub_msg()
        rate.sleep()
