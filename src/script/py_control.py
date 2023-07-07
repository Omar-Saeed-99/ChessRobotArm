#! /usr/bin/env python3
from threading import *
import time

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import String , Bool 
import serial

import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


p.connect(p.DIRECT)


def node():
    control = controller()
    control.run()


class controller():
    def __init__(self) -> None:
        self.done = False
        self.alpha_list = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        self.traj = []
        self.new_traj = False
        self.traj_old = []
        self.serial_init_()
        self.done = Bool()

        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Define the end effector link and target position

        self.robot = p.loadURDF("/home/omar/mambaforge/envs/ros_env/gp_ws/src/chess_rob/urdf/chess_rob_ik.urdf")
        self.ik_solver = p.createConstraint(
        self.robot, 3, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0]
        )
        target_ori = p.getQuaternionFromEuler([0, 0, 0])
        # Set up the inverse kinematics solver
        num_joints = p.getNumJoints(self.robot)
        joint_indices = range(num_joints)
        self.lower_limits, self.upper_limits = zip(*[p.getJointInfo(self.robot, i)[8:10] for i in joint_indices])
        # joint_ranges = [upper-lower for lower, upper in zip(self.lower_limits, self.upper_limits)]
        # rest_positions = [(lower+upper)/2 for lower, upper in zip(self.lower_limits, self.upper_limits)]
        # joint_damping = [0.1]*num_joints


    def run(self):
        
        rospy.init_node('py_control') 

        self.control_publisher = rospy.Publisher('/chess_rob/arm_controller/command', JointTrajectory, queue_size=10)
        self.done_publisher = rospy.Publisher("/chess_rob/python_api/done", Bool, queue_size=10)

        while not rospy.is_shutdown():

            if self.new_traj and self.traj_old != self.traj :
                self.done.data = False
                self.str_of_points = ""
                self.thread()

                for ik in self.traj:
                    time.sleep(1)
                    j = p.calculateInverseKinematics(self.robot, 3, ik, lowerLimits=self.lower_limits, upperLimits=self.upper_limits,
                                                        maxNumIterations=1000, residualThreshold=1e-5)
                    if self.traj.index(ik) == 0:
                        self.str_of_points = str(-29)+","+str(45)+","+str(10)+",0"
                    elif self.traj.index(ik) == 1:
                        self.str_of_points = str(-29)+","+str(105)+","+str(18)+",1"
                    if self.traj.index(ik) == 2:
                        self.str_of_points = str(-29)+","+str(45)+","+str(10)+",1"
                    elif self.traj.index(ik) == 3:
                        self.str_of_points = str(-19)+","+str(50)+","+str(30)+",1"
                    elif self.traj.index(ik) == 5:
                        self.str_of_points = str(-19)+","+str(105)+","+str(35)+",1"  
                    elif self.traj.index(ik) == 6:
                        self.str_of_points = str(-19)+","+str(105)+","+str(35)+",0"                       
                    elif self.traj.index(ik) == 7:
                        self.str_of_points = str(0)+","+str(0)+","+str(0)+",0"   
                    elif self.traj.index(ik) == 8:
                        self.str_of_points = str(0)+","+str(0)+","+str(0)+",0"   
                    self.send(self.str_of_points)
                    while (True):

                        self.feedback = self.receive()
                        if self.feedback != None:
                            if (self.feedback[0:2] == "ok"):
                                break
                        

                    self.traj_old = self.traj
                

                self.new_traj = False

            
            self.done.data = True
            self.done_publisher.publish(self.done)


            rospy.Subscriber("/chess_rob/python_api/move", String, self.order_callback)
            

    def get_point_between(self, p1, p2):
        # Initialize list to store the intermediate points
        points = []
        # Calculate the x and y distances between the two points
        p1 = (p1[0], p1[1],0.0)
        p2 = (p2[0], p2[1],0.0)

        x3 = (p2[0] + p1[0])  / 2
        y3 = (p2[1] + p1[1]) / 2

        p3 = [x3,y3,0.10]

        points = [p1,  p3,  p2 , p1,  p3,  p2 ,p1 ,p2,p3]
        # Return the list of intermediate points
        # print(points[-1])
        self.traj = points


    def thread(self):
        self.t1=Thread(target=self.gazebo_thread)
        self.t1.start()

    def gazebo_thread(self):
        
        for i in range(3):
            print(i)

            msg = JointTrajectory()

            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = ''
            msg.joint_names = ['joint_1', 'joint_2', 'joint_3' ]

            point = JointTrajectoryPoint()

            # print(ik)
            try:
                j = p.calculateInverseKinematics(self.robot, 3, self.traj[i], lowerLimits=self.lower_limits, upperLimits=self.upper_limits,
                                                maxNumIterations=3000, residualThreshold=1e-5)
                point.positions = [j[0]-1.57, j[1], j[2]]
                print(j)
                # point.positions = [-1.57, 0, 0]

            except:
                point.positions = [0, 0 , 0]
            
            point.velocities = []
            point.accelerations = []
            point.effort = []
            point.time_from_start = rospy.Duration(1)
            msg.points.append(point)
            self.control_publisher.publish( msg )

            time.sleep(1.5)

        self.t1._delete()


    def order_callback(self,msg):
        point1 = [round((64.61+((8-int(msg.data[1]))*17.5))/1000,3),round((123-(self.alpha_list.index(msg.data[0])*17.5))/1000,3),0.20]
        point2 = [round((64.61+((8-int(msg.data[3]))*17.5))/1000,3),round((123-(self.alpha_list.index(msg.data[2])*17.5))/1000,3),0.20]
        
        self.get_point_between(point1,point2)
        self.new_traj = True


    def plot_points(self,points):
        # Initialize a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Extract the x, y, and z coordinates of the points
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        
        # Plot the points
        ax.scatter(xs, ys, zs)

        # Set the plot limits and labels
        ax.set_xlim3d(min(xs), max(xs))
        ax.set_ylim3d(min(ys), max(ys))
        ax.set_zlim3d(min(zs), max(zs))
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

    def serial_init_(self):
        try:
            self.__srl = serial.Serial('/dev/rfcomm0', 57600, timeout=0.01)
            print("connected to bluetooth")
        except:
            print("arduino bluetooth connection error !")

        try:
            self.__srl = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.01)
            print("connected to USB")

        except:
            print("arduino usb connection error !")


    def send(self,data:str):
        try:
            self.data = "<444," + data + ">"
            self.__srl.write(self.data.encode())
            print("Sending: ",self.data)
            time.sleep(0.05)
        # MyPort.close()
        # MyPort.open()
        except:
            print("sending error !")
            self.serial_init_()

    def receive(self):
        try:
            data = self.__srl.readall().decode()
            time.sleep(0.1)
            if len(data)>1:
                print("recived: ", data)
                return(data)
            else:
                return None
        except:
            print("No recived data")
            self.serial_init_()



        # Show the plot
        plt.show()
if __name__ == '__main__':
    node()
