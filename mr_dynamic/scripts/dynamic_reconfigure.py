#!/usr/bin/env python3

# Description:
# This script is used to define the dynamic reconfigure server for the 3DoF robot arm.
# It creates the following parameters:
# vx, vy, vz: linear velocity in x, y and z axis
# x, y, z: position in x, y and z axis
# stop: stop the robot arm
# ...


import rospy
import dynamic_reconfigure
from mr_dynamic.cfg import RobotArmConfig
from geometry_msgs.msg import PointStamped, Vector3
from sensor_msgs.msg import JointState



class Config:


    def __init__(self):
        pass

    #Function to call when a parameter is changed
    def callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {x}, {y}, {z}, {vx}, {vy}, {vz}, {stop}""".format(**config))
        self.x = config.x
        self.y = config.y
        self.z = config.z
        self.vx = config.vx
        self.vy = config.vy
        self.vz = config.vz
        self.stop = config.stop
        return config




if __name__ == '__main__':
    config = Config()
    rospy.init_node('dynamic_reconfigure')

    #Create the dynamic reconfigure server
    srv = dynamic_reconfigure.server.Server(RobotArmConfig, config.callback)

    #Create the publishers
    waypoint_pub = rospy.Publisher('/waypoint_clicked', PointStamped, queue_size=10)
    velocity_pub = rospy.Publisher('/velocity', PointStamped, queue_size=10)
    set_velocity_pub = rospy.Publisher('/set_velocity', PointStamped, queue_size=10)

    #Create the subscribers
    joint_states_sub = rospy.Subscriber('/joint_states', JointState, config.callback)

    rospy.spin()

