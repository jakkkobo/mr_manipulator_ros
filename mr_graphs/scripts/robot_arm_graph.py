#!/usr/bin/env python3

# Description:
# This script subscribes to the robot's joints angles and convert to x,y,z coordinates
# also calculates the robot's velocity
# and plots the robot's trajectory in a graph

import rospy
import numpy as np
import math, time

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import tf
from mr_joint_controller.msg import JointTarget

import matplotlib.pyplot  as plt
from matplotlib.animation import FuncAnimation



class graphs:

    respawn = True
    update = True

    def __init__(self):

        self.first_timestamp = None
        self.nr_of_poses = 0
        self.seconds = 0
        self.joints = []
        # self.joints_time = []
        self.pose = []
        self.sent_pose = []
        self.robot_velocities = []
        self.range = 0.01
        self.counter = 0
        self.flag_received = 0

        self.L1 = 0.129
        self.L2 = 0.135
        self.L3 = 0.146
        self.L4 = 0.029
        self.L5 = 0.030


        self.joints_sub = rospy.Subscriber("/joint_target", JointTarget, self.joints_callback)
        self.real_pose_sub = rospy.Subscriber("/real_pose", PointStamped, self.pose_callback)
        
        fig = plt.figure(figsize=(15, 10))

        self.ax = fig.add_subplot(331)
        self.ax2 = fig.add_subplot(334)
        self.ax3 = fig.add_subplot(337)
        self.ax4 = fig.add_subplot(332)
        self.ax5 = fig.add_subplot(335)
        self.ax6 = fig.add_subplot(338)
        self.ax7 = fig.add_subplot(333)
        self.ax8 = fig.add_subplot(336)
        self.ax9 = fig.add_subplot(339)
        fig.subplots_adjust(hspace=0.5)
        # self.ax7 = fig.add_subplot(427)
        # self.ax8 = fig.add_subplot(428)

        animation = FuncAnimation(fig, self.update_graph, frames = None, interval=10)

        plt.show()

    
    def compute_poses(self, theta_1, theta_2, theta_3):

        theta_offset = 15*np.pi/180
        theta_2 = theta_2 - 90*np.pi/180 + theta_offset
        theta_2 = -theta_2 + 360*np.pi/180
        # theta_3 = -theta_3
        # theta_2 = -theta_2 - theta_offset
        # theta_1 = -theta_1

        #direct kinematics
        Pd = self.L2*np.cos(theta_2) + self.L3*np.cos(theta_3) + self.L4

        #compute the x,y,z coordinates
        x = Pd*np.cos(theta_1)
        y = Pd*np.sin(theta_1)
        z = self.L1 + self.L2*np.sin(theta_2) - self.L3*np.sin(theta_3) - self.L5

        # print("x: ", x)
        # print("y: ", y)
        # print("z: ", z)
        # print("\n")
        #append the coordinates to a list
        self.pose.append([x, y, z, self.seconds])

        if len(self.pose) > 1:
            delta_t = self.seconds - self.prev_seconds
            delta_x = x - self.prev_pose_x
            delta_y = y - self.prev_pose_y
            delta_z = z - self.prev_pose_z


            # print("delta_t: ", delta_t)
            # print("delta_x: ", delta_x)
            # print("delta_y: ", delta_y)
            # print("delta_z: ", delta_z)
            # print("\n")

            v_x = delta_x/delta_t
            v_y = delta_y/delta_t
            v_z = delta_z/delta_t


            print("v_x: ", v_x)
            print("v_y: ", v_y)
            print("v_z: ", v_z)
            print("\n")

            self.prev_pose_x = x
            self.prev_pose_y = y
            self.prev_pose_z = z
            self.prev_seconds = self.seconds

            self.robot_velocities.append([v_x, v_y, v_z, self.seconds])
        else:
            self.prev_pose_x = x
            self.prev_pose_y = y
            self.prev_pose_z = z
            self.prev_seconds = self.seconds
            self.robot_velocities.append([0, 0, 0, self.seconds])


    def joints_callback(self, msg):

        if self.first_timestamp is None:
            self.first_timestamp = msg.header.stamp
            self.seconds = 0
            # self.joints_time.append(self.seconds)
        else:
            self.seconds = (msg.header.stamp - self.first_timestamp).to_sec()
            # self.joints_time.append(self.seconds - self.first_timestamp)

        #get joint angles from -pi to pi
        joint_1 = msg.joint1
        joint_2 = msg.joint2
        joint_3 = msg.joint3

        #append the joint angles to a list
        self.joints.append([joint_1, joint_2, joint_3, self.seconds])

        self.compute_poses(joint_1, joint_2, joint_3)

        self.flag_received = 1

        
    def pose_callback(self, msg):

        if self.first_timestamp is None:
            self.first_timestamp = msg.header.stamp
            seconds = 0
        else:        
            seconds = (msg.header.stamp - self.first_timestamp).to_sec()

        # save the sent pose to a list
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        self.sent_pose.append([x, y, z, seconds])
            
        

        
    # def end_callback(self, msg):
            
    #         if msg.data == True:

    #             while (self.respawn == False):
    #                 print("Waiting for respawn...")
    #                 self.update = False

    #             x_coords, y_coords = zip(*self.robot_poses)
    #             #save the graph coordinates in a file
    #             with open('robot_graph_results.txt', 'w') as f:
    #                 for x, y in zip(x_coords, y_coords):
    #                     f.write("%s %s\n" % (x, y))
        

    #             #reset the graph
    #             self.robot_poses = []
    #             self.robot_theta = []
    #             self.robot_velocities = []
    #             self.robot_accelerations = []
    #             self.range = 0.01
  
    #             #clear the graph
    #             self.ax.cla()
    #             self.ax2.cla()
    #             self.ax3.cla()
    #             self.ax4.cla()
    #             self.ax5.cla()
    #             self.ax6.cla()
            
    
    #             # print("Saving the graph...")
    
    #             # plt.savefig('robot_graph_results' + str(self.counter) + '.png', dpi=1200, bbox_inches='tight')
    #             # self.counter += 1
    #             # print("Graph saved!")

    #             self.update = True
            
    

        

    def update_graph(self, frame):

        # if the robot_poses as more than 1 pose
        if self.joints != [] and self.flag_received == 1:
            
            self.ax.cla()
            #graph 1
            #set x limit from 0 to last joint_time position in the list
            self.ax.set_xlim(0, self.seconds) 
            #set y limit from -pi to pi
            self.ax.set_ylim(-math.pi, math.pi)
            y_coords_1, y_coords_2, y_coords_3, x_coords = zip(*self.joints)
            # print(y_coords_1)
            # print(y_coords_2)
            # print(y_coords_3)
            # print(x_coords)
            self.ax.plot(x_coords, y_coords_1, linestyle='-', label='theta_1', color='blue')

            self.ax.set_xlabel('time (sec)')
            self.ax.set_ylabel('angular position (rad)')

            self.ax.set_title('L1 joint')
            self.ax.grid(True)
            self.ax.legend()

            self.ax2.cla()
            #graph 2
            #set x limit from 0 to last joint_time position in the list
            self.ax2.set_xlim(0, self.seconds)  
            #set y limit from -pi to pi
            self.ax2.set_ylim(-math.pi, math.pi)
            self.ax2.plot(x_coords, y_coords_2, linestyle='-', label='theta_2', color='blue')

            self.ax2.set_xlabel('time (sec)')
            self.ax2.set_ylabel('angular position (rad)')

            self.ax2.set_title('L2 Joint')
            self.ax2.grid(True)
            self.ax2.legend()

            
            #graph 3
            self.ax3.cla()
            #set x limit from 0 to last joint_time position in the list
            self.ax3.set_xlim(0, self.seconds)   
            #set y limit from -pi to pi
            self.ax3.set_ylim(-math.pi, math.pi)
            self.ax3.plot(x_coords, y_coords_3, linestyle='-', label='theta_3', color='blue')

            self.ax3.set_xlabel('time (sec)')
            self.ax3.set_ylabel('angular position (rad)')

            self.ax3.set_title('L3 joint')
            self.ax3.grid(True)
            self.ax3.legend()

            #graph 4
            self.ax4.cla()
            #set x limit from 0 to last joint_time position in the list
            self.ax4.set_xlim(0, self.seconds)
            set_y_1, set_y_2, set_y_3, set_x = zip(*self.sent_pose)
            y_coords_1, y_coords_2, y_coords_3, x_coords = zip(*self.pose)
            #set y limit from 0 to 0.4
            self.ax4.set_ylim(0, 0.4)
            self.ax4.plot(x_coords, y_coords_1, linestyle='-', label='x_position', color='blue')
            self.ax4.plot(set_x, set_y_1, linestyle='-', label='x_set_position', color='red')

            self.ax4.set_xlabel('time (sec)')
            self.ax4.set_ylabel('position (m)')

            self.ax4.set_title('End Effector X position')
            self.ax4.grid(True)
            self.ax4.legend()

            
            #graph 5
            self.ax5.cla()
            #set x limit from 0 to last joint_time position in the list
            self.ax5.set_xlim(0, self.seconds)
            #set y limit from 0 to 0.4
            self.ax5.set_ylim(-0.4, 0.4)
            self.ax5.plot(x_coords, y_coords_2, linestyle='-', label='y_position', color='blue')
            self.ax5.plot(set_x, set_y_2, linestyle='-', label='y_set_position', color='red')

            self.ax5.set_xlabel('time (sec)')
            self.ax5.set_ylabel('position (m)')

            self.ax5.set_title('End Effector Y position')
            self.ax5.grid(True)
            self.ax5.legend()

            #graph 6
            self.ax6.cla()
            #set x limit from 0 to last joint_time position in the list
            self.ax6.set_xlim(0, self.seconds)
           #set y limit from 0 to 0.4
            self.ax6.set_ylim(0, 0.4)
            self.ax6.plot(x_coords, y_coords_3, linestyle='-', label='z_position', color='blue')
            self.ax6.plot(set_x, set_y_3, linestyle='-', label='z_position', color='red')

            self.ax6.set_xlabel('time (sec)')
            self.ax6.set_ylabel('position (m)')

            self.ax6.set_title('End Effector Z position')
            self.ax6.grid(True)
            self.ax6.legend()

            #graph 7
            self.ax7.cla()
            #set x limit from 0 to last joint_time position in the list
            self.ax7.set_xlim(0, self.seconds)
            #set y limit from 0 to 0.99
            self.ax7.set_ylim(0, 0.99)
            y_coords_1, y_coords_2, y_coords_3, x_coords = zip(*self.robot_velocities)
            self.ax7.plot(x_coords, y_coords_1, linestyle='-', label='x_velocity', color='green')
            
            self.ax7.set_xlabel('time (sec)')
            self.ax7.set_ylabel('velocity (m/s)')

            self.ax7.set_title('End Effector X velocity')
            self.ax7.grid(True)
            self.ax7.legend()

            #graph 8
            self.ax8.cla()
            #set x limit from 0 to last joint_time position in the list
            self.ax8.set_xlim(0, self.seconds)
            #set y limit from 0 to 0.99
            self.ax8.set_ylim(0, 0.99)
            self.ax8.plot(x_coords, y_coords_2, linestyle='-', label='y_velocity', color='green')

            self.ax8.set_xlabel('time (sec)')
            self.ax8.set_ylabel('velocity (m/s)')

            self.ax8.set_title('End Effector Y velocity')
            self.ax8.grid(True)
            self.ax8.legend()

            #graph 9
            self.ax9.cla()
            #set x limit from 0 to last joint_time position in the list
            self.ax9.set_xlim(0, self.seconds)
            #set y limit from 0 to 0.99
            self.ax9.set_ylim(0, 0.99)
            self.ax9.plot(x_coords, y_coords_3, linestyle='-', label='z_velocity', color='green')
            
            self.ax9.set_xlabel('time (sec)')
            self.ax9.set_ylabel('velocity (m/s)')

            self.ax9.set_title('End Effector Z velocity')
            self.ax9.grid(True)
            self.ax9.legend()

            # #graph 2
            # self.ax2.cla()

            # theta, r = zip(*self.robot_theta)

            # self.ax2.set_theta_direction(1)
            # self.ax2.set_theta_offset(np.pi/2.0)
            # self.ax2.plot(theta, r, linestyle='-', label='robot_heading', color='red')

            # self.ax2.set_title('Orientation')
            # self.ax.grid(True)
            # self.ax2.set_rlabel_position(self.ax2.get_rmax()+0.5)
            # angle = np.deg2rad(315)
            # self.ax2.legend(loc="lower left", bbox_to_anchor=(.5 + np.cos(angle)/2, .5 + np.sin(angle)/2))

            # #graph 3,4
            # self.ax3.cla()
            # self.ax4.cla()

            # self.ax3.set_xlim(0, self.seconds)
            # self.ax3.set_ylim(-1, 1)
            # self.ax4.set_xlim(0, self.seconds)
            # self.ax4.set_ylim(-1, 1)

            # v_x, v_theta, time = zip(*self.robot_velocities)
            # self.ax3.plot(time, v_x, linestyle='-', label='linear_velocity', color='orange')
            # self.ax4.plot(time, v_theta, linestyle='-', label='angular_velocity', color='green')

            # self.ax3.set_xlabel('Time (s)')
            # self.ax3.set_ylabel('Velocity (m/s)')
            # self.ax4.set_xlabel('Time (s)')
            # self.ax4.set_ylabel('Velocity (rad/s)')
            # self.ax3.set_title('Linear Velocity')
            # self.ax4.set_title('Angular Velocity')
            # self.ax3.grid(True)
            # self.ax4.grid(True)
            # self.ax3.legend()
            # self.ax4.legend()

            # #graph 5,6
            # self.ax5.cla()
            # self.ax6.cla()

            # self.ax5.set_xlim(0, self.seconds)
            # self.ax5.set_ylim(-1, 1)
            # self.ax6.set_xlim(0, self.seconds)
            # self.ax6.set_ylim(-5, 5)

            # a_x, a_theta, time = zip(*self.robot_accelerations)
            # self.ax5.plot(time, a_x, linestyle='-', label='linear_acceleration', color='orange')
            # self.ax6.plot(time, a_theta, linestyle='-', label='angular_acceleration', color='green')

            # self.ax5.set_xlabel('Time (s)')
            # self.ax5.set_ylabel('Acceleration (m/s^2)')
            # self.ax6.set_xlabel('Time (s)')
            # self.ax6.set_ylabel('Acceleration (rad/s^2)')
            # self.ax5.set_title('Linear Acceleration')
            # self.ax6.set_title('Angular Acceleration')
            # self.ax5.grid(True)
            # self.ax6.grid(True)
            # self.ax5.legend()
            # self.ax6.legend()

            self.flag_received = 0


def main():

    rospy.init_node('graphs', anonymous=True)

    graphs()

    #try to catch the Ctrl+C
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
        plt.close('all')
        print('Done.')

if __name__ == '__main__':
    main()