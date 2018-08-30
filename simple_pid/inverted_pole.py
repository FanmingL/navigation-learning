import pybullet as p
import pybullet_data
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np
import tf
import time
from my_pid import MyPid


class InvertedPendulum:

    def __init__(self, mode=p.VELOCITY_CONTROL):
        rospy.init_node('inverted_pendulum')
        self.start_position = [0., 0., 1]
        self.start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        p.connect(p.GUI)
        p.setRealTimeSimulation(1)
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.setTimeStep(1./480.)
        self.plane_id = p.loadURDF('plane.urdf')
        self.robot_id = p.loadURDF('husky/husky.urdf', self.start_position, self.start_orientation)
        p.addUserDebugText('Car Position', [0, 0, 0.5], textColorRGB=[1, 0, 0],
                           parentObjectUniqueId=self.robot_id, parentLinkIndex=0)
        self.joint_num = p.getNumJoints(self.robot_id)
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
        self.pub_control = rospy.Publisher('robot_control', Twist, queue_size=1)
        self.control_msg = Twist()
        self.mode = mode
        self.velocity_x = 0
        self.velocity_rotate = 0
        self.motor_const_velocity = 30
        self.br = tf.TransformBroadcaster()
        self.ls = tf.TransformListener()
        self.yaw_pid = MyPid(120, 0.6, 15, 10, 100, 30)
        self.pos_pid = MyPid(20, 0.5, 10, 10, 40, 50)
        self.text_handler = p.addUserDebugText(' ', [0, 0, 0], textColorRGB=[1, 0, 1])

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
        self.pos_msg.header.frame_id = 'robot position'
        self.control_msg.linear.x = self.velocity_x
        self.control_msg.angular.z = self.velocity_rotate
        self.br.sendTransform((robot_state[0][0], robot_state[0][1], robot_state[0][2]),
                              (robot_state[1][0], robot_state[1][1], robot_state[1][2], robot_state[1][3]),
                              rospy.Time.now(),
                              'robot',
                              'world')
        self.pub_control.publish(self.control_msg)
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

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.set_speed_from_keyboard()
            self.velocity_rotate = 7
            self.velocity_x = 15
            self.set_motor()
            self.pub_msg()
            rate.sleep()

    def get_euler(self):
        _tmp = self.pos_msg.pose.orientation
        quaternion = [_tmp.x, _tmp.y, _tmp.z, _tmp.w]
        euler = p.getEulerFromQuaternion(quaternion)
        return euler

    # arctan(l/r)
    def go_to_target(self, end_position, end_orientation, precision=0.01):
        rate = rospy.Rate(100)
        p.removeUserDebugItem(self.text_handler)
        self.text_handler = p.addUserDebugText('Target', end_position, textColorRGB=[1, 0, 1])
        while not rospy.is_shutdown():
            self.pub_msg()
            self.br.sendTransform(end_position, end_orientation,
                                  rospy.Time.now(),
                                  'target',
                                  'world')
            try:
                (trans, rot) = self.ls.lookupTransform('robot', 'target', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            yaw_feedback = np.arctan2(trans[1], trans[0])
            yaw_out = self.yaw_pid.step(feedback=yaw_feedback, expect=0)
            x_out = self.pos_pid.step(feedback=trans[0], expect=0)
            if ((end_position[0] - self.pos_msg.pose.position.x) ** 2 +
               (end_position[1] - self.pos_msg.pose.position.y) ** 2) < precision:
                break
            elif np.abs(x_out) < 5:
                self.velocity_rotate = 0
            else:
                self.velocity_rotate = yaw_out
            self.velocity_x = -x_out

            self.set_motor()
            rate.sleep()


if __name__ == '__main__':
    t = InvertedPendulum()
    while 1:
        keys = p.getKeyboardEvents()
        if t.SPACE in keys:
            break

    endPosArray = [[1.5, 1.5, 0], [-1.5, -1.5, 0], [2, -1, 0], [-1, 2, 0]]

    endOri = p.getQuaternionFromEuler([0, 0, 0])
    for i in range(len(endPosArray)):
        t.go_to_target(endPosArray[i], endOri)
