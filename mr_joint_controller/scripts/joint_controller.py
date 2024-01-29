#!/usr/bin/env python3

# Description:
# This script is used to control the joints of the 3DoF robot arm.
# It subscribes to the /joint_target topic to get the current L1, L2 and L3 joint angles.
# and publishes to the /joint_states topic to set the target joint angles.
# It only have the position control for now.


import sys
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState
from mr_joint_controller.msg import JointTarget

DEG_TO_RAD = math.pi/180

class JointController:

    def __init__(self):
        rospy.init_node('joint_controller')
        self.rate = rospy.Rate(10)
        self.joint_target_sub = rospy.Subscriber('/joint_target', JointTarget, self.joint_target_callback)
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.joint_states = JointState()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['base', 'L1', 'L2', 'L3_joint', 'auxiliar_2_joint', 'auxiliar_21_joint', 'auxiliar_joint', 'auxiliar_elbow', 'elbow_auxiliar_L4_joint']
        self.joint1_target = 0
        self.joint2_target = 0
        self.joint3_target = 0

        self.base_joint = 0
        self.L1_joint = 0
        self.L2_joint = 0
        self.L3_joint = 0
        self.auxiliar_2_joint = 0
        self.auxiliar_21_joint = 0
        self.auxiliar_joint = 0
        self.auxiliar_elbow = 0
        self.elbow_auxiliar_L4_joint = 0
    

    def joint_target_callback(self, data):
        self.joint1_target = data.joint1
        self.joint2_target = data.joint2
        self.joint3_target = data.joint3

    def joint_states_publish(self):
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.position = [self.base_joint, self.L1_joint, self.L2_joint, self.L3_joint, self.auxiliar_2_joint, self.auxiliar_21_joint, self.auxiliar_joint, self.auxiliar_elbow, self.elbow_auxiliar_L4_joint]
        # self.joint_states.velocity = [self.joint1_vel, self.joint2_vel, self.joint3_vel]
        # self.joint_states.effort = [self.joint1_effort, self.joint2_effort, self.joint3_effort]
        self.joint_states_pub.publish(self.joint_states)

    def base_control(self):
        self.base_joint = -self.joint1_target
        self.joint_states_publish()

    def L1_control(self):
        self.L1_joint = self.joint2_target 
        self.joint_states_publish()

    def L2_control(self):
        self.L2_joint = -self.joint3_target + self.joint2_target
        self.auxiliar_joint = -self.joint2_target 
        self.auxiliar_elbow = self.joint2_target
        self.joint_states_publish()

    def L3_control(self):
        self.L3_joint = -self.joint3_target# - self.joint2_target
        self.auxiliar_2_joint = self.joint3_target # + self.joint2_target
        self.auxiliar_21_joint = self.joint3_target - self.joint2_target
        self.elbow_auxiliar_L4_joint = -self.joint3_target# - self.joint2_target
        self.joint_states_publish()


    def run(self):
        while not rospy.is_shutdown():
            self.base_control()
            self.L1_control()
            self.L2_control()
            self.L3_control()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        joint_controller = JointController()
        joint_controller.run()
    except rospy.ROSInterruptException:
        pass


