#!/usr/bin/env python2

'''
Record points of the wheel from

@author lukashuber
@date 2019-03-01
'''

import rospy
import rospkg

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped, Twist, Pose, PoseStamped, WrenchStamped
from std_msgs.msg import Bool

import numpy as np
import numpy.linalg as LA
import warnings

from os.path import join, isfile
from os import makedirs
# import matplotlib.pyplot as plt

# import json

import sys
import signal

import datetime

# from robot_calbration.src.robot_calbration.recorder import Recorder
# src.robot_calbration.recorder import Recorder``
# from recorder import Recorder

# import os
# os.chdir("/home/lukas/catkin_ws/src/wheel_polishing/scripts/")

# from testLines import calculate_cuver

from multiprocessing import Lock


import sys
sys.path.insert(0, "/home/lukas/catkin_ws/src/wheel_polishing/scripts/")
# from class_quaternion import QuatClass

# MODES_PROGRAM = {"c":0,
                # "passive_ds":1,
                # "gravity_comp":2}

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# DT = 1./25 # fixed DT
DT_INPUT = 0.04 # make sure this is bigger than self.dt_pub


class RecordJointsStates():
    def __init__(self, topicName="ur5/joint_states", topicType=JointState, n_loops=0):
        self.mutex = Lock()

        self.start_time = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        rospy.init_node("listener_positionMeasure", anonymous=True)

        self.rospack = rospkg.RosPack()

        self.count_joSta = 0 
        self.count_eeVel = 0
        self.count_eePose = 0
        self.recieved_forceTorque = False
        
        rospy.Subscriber("/lwr/joint_states", JointState, self.callback_jointState)
        rospy.Subscriber("/lwr/ee_pose", Pose, self.callback_eePose)
        rospy.Subscriber("/lwr/ee_vel", Twist, self.callback_eeVel)
        rospy.Subscriber("/ft_sensor/netft_data", WrenchStamped, self.callback_forceTorque)

        self.pub_gravComp = rospy.Publisher("/lwr/joint_controllers/command_grav", Bool, queue_size=5)
        self.n_joints = 7
        self.dim = 3

        self.motion_states_list = {"home":0,"recording":1,"transition":2}
        self.motion_state =  self.motion_states_list["home"]
        self.last_force_recieved=0

        # Initialize topics
        self.recorded_data = []

        self.freq = 50 # Warning
        self.rate = rospy.Rate(self.freq)    # Frequency 10Hz

        # Recording KUKA 
        self.data_description = []
        self.data_description.append("time")
        # 1
        # 1-21 (if n_joints=7)
        for topic in ["position", "velocity", "effort"]:
            for nn in range(self.n_joints):
                self.data_description.append(topic + str(nn))

        # 22-24
        for dd in range(self.dim):
            self.data_description.append("ee_pos"+str(dd))

        # 25-28
        for dd in range(self.dim+1):
            self.data_description.append("ee_quat"+str(dd))

        # 29-34
        for topic in ["ee_velLin", "ee_velAng"]:
            for dd in range(self.dim):
                self.data_description.append(topic+str(dd))

        # if record_forceTorque:
        # 35-41
        for topic in ["ft_force", "ft_torque"]:
            for dd in range(self.dim):
                self.data_description.append(topic+str(dd))
        
        self.data_description.append("trajectory_label") 
        self.data_description.append("loop_label")
        self.data_description.append("active_point")
        # import pdb; pdb.set_trace();

        self.shutdown_executed = False

        print('Node initialized')
        
        while (not rospy.is_shutdown() 
               and
               (not self.count_joSta 
               or not self.count_eeVel 
               or not self.count_eePose 
               or not self.recieved_forceTorque) ):

            print("waiting for callback messages..")
            rospy.sleep(0.25)

    def run(self):              # 
        print("Hello Master. We are ready to record positiosns.")
        print("Please enter a filename without specification: ")
        print("default name is <<recording[DATE-TIME].csv>>)")
        self.file_name = raw_input("")

        if self.file_name.find(".") == -1: # Default file name
            self.file_name = self.file_name + ".csv"

        if not len(self.file_name):
            self.file_name = "recoring"+self.start_time+".csv"
        
        self.data_points = np.zeros((0, len(self.data_description)))

        pos_old = [self.eePose_msg.position.x, self.eePose_msg.position.y, self.eePose_msg.position.z]

        self.it_loop = 0
        self.trajectory_counter = 0 # 
        self.movement_loop_counter = 0

        while not rospy.is_shutdown():
            self.update_state()
            self.pub_gravComp.publish(Bool(True))

            with self.mutex:
                self.it_loop += 1

                # ee Vel
                # dx = pos_old - np.array([self.eePose_msg.position.x, self.eePose_msg.position.y, self.eePose_msg.position.z])
                # dVel = LA.norm(dx)*self.freq
                # if dVel > 0.01:

                # if self.motion_state==self.motion_states_list["recording"]:
                if not self.motion_state==self.motion_states_list["home"]:
                    # Save new point
                    new_dataPoint = []
                    new_dataPoint.append(rospy.get_time())
                # Joint
                    new_dataPoint += (self.joSta_msg.position[:self.n_joints])
                    new_dataPoint += (self.joSta_msg.velocity[:self.n_joints])
                    new_dataPoint += (self.joSta_msg.effort[:self.n_joints])

                    # ee Pose
                    new_dataPoint += [self.eePose_msg.position.x, self.eePose_msg.position.y, self.eePose_msg.position.z]

                    # print('pos', self.eePose_msg.position)
                    # print('pos', new_dataPoint[22:25])

                    new_dataPoint += [self.eePose_msg.orientation.x, self.eePose_msg.orientation.y, self.eePose_msg.orientation.z, self.eePose_msg.orientation.w]
                    pos_old = np.array([self.eePose_msg.position.x, self.eePose_msg.position.y, self.eePose_msg.position.z])
                    
                    new_dataPoint += [self.eeVel_msg.linear.x, self.eeVel_msg.linear.y, self.eeVel_msg.linear.z]
                    new_dataPoint += [self.eeVel_msg.angular.x, self.eeVel_msg.angular.y, self.eeVel_msg.angular.z]
                    
                    new_dataPoint += [self.forceTorque_msg.wrench.force.x, 
                                      self.forceTorque_msg.wrench.force.y, 
                                      self.forceTorque_msg.wrench.force.z]
                    new_dataPoint += [self.forceTorque_msg.wrench.torque.x,
                                      self.forceTorque_msg.wrench.torque.y,
                                      self.forceTorque_msg.wrench.torque.z]

                    new_dataPoint += [self.trajectory_counter, self.movement_loop_counter]
                    
                    # Recording state
                    new_dataPoint.append(self.motion_state==self.motion_states_list["recording"])  

                    self.data_points = np.vstack((self.data_points, np.tile(new_dataPoint, (1,1))))
                    # print("Recorded another point.")

                # else:
                    # print("Resting position with vel={}. Not recording loop {}".format(dVel, self.it_loop))                
                    # print(np.array([self.eePose_msg.position.x, self.eePose_msg.position.y, self.eePose_msg.position.z]))
                    # print("non
                    # print("not recording")
                self.rate.sleep()

                
                # print('shape 1', self.data_points.shape)
                # print('shape 2', np.array(new_dataPoint).shape)
                # import pdb; pdb.set_trace()
            
            self.rate.sleep()


    def shutdown_command(self, sig, frame):
        rospy.signal_shutdown('Shutdown initiated....')
        print('Shutdown initiated....')

        if self.shutdown_executed:
            return 

        print("Joint states recieved #{}".format(self.count_joSta))
        print("EE velocity recieved #{}".format(self.count_eeVel))
        print("EE Pose recieved #{}".format(self.count_eePose))
        
        self.save_to_file()

        self.shutdown_executed = True
        print('... finished shutdown.')


    def update_state(self, home_dist_z=0.4, force_z_margin=4, maximum_force_waiting_steps=20):
        force_z = self.forceTorque_msg.wrench.force.z   
 
        if self.eePose_msg.position.z>home_dist_z:
            if not self.motion_state==self.motion_states_list["home"]:
                self.motion_state = self.motion_states_list["home"]
                self.trajectory_counter = -1 # Start at zero, as += of state entering will make it 1
                self.movement_loop_counter += 1
                print("Resting state. Sampling #{}".format(self.movement_loop_counter))

        else:
            if force_z<force_z_margin:
                if not self.motion_state==self.motion_states_list["recording"]:
                    self.trajectory_counter += 1
                    self.last_force_recieved=0
                    self.motion_state = self.motion_states_list["recording"]
                    print("Start recording...")
            else: # force_z<=force_z_margin
                if self.motion_state==self.motion_states_list["recording"]:
                    self.last_force_recieved += 1
                    print("No force recieved {}/{}. Stopping recording soon.".format(self.last_force_recieved, maximum_force_waiting_steps))
                    if self.last_force_recieved>=maximum_force_waiting_steps:
                        self.motion_state = self.motion_states_list["transition"]
                        print("Transitioning..")

        
    def save_to_file(self, package_name="learn_motion", ):
        print("Saving #{} points to file".format(self.data_points.shape[0]) )

        rospack = rospkg.RosPack()
        data_dir = join(rospack.get_path(package_name), "data")
        if not exists(data_dir):
            makedirs(data_dir)

        header_string = "".join([self.data_description[ii]+", " for ii in range(len(self.data_description))])
        np.savetxt(join(data_dir, self.file_name), self.data_points, delimiter=",", header=header_string)
        print("saving completed.")
        
    def callback_jointState(self, msg):
        with self.mutex:
            self.count_joSta += 1
            self.joSta_msg = msg

    def callback_eeVel(self, msg):
        with self.mutex:
            self.count_eeVel += 1
            self.eeVel_msg = msg

    def callback_eePose(self, msg):
        with self.mutex:
            self.count_eePose += 1
            self.eePose_msg = msg

    def callback_forceTorque(self, msg):
        with self.mutex:
            # self.count_eePose += 1
            self.forceTorque_msg = msg

        self.recieved_forceTorque = True


if __name__ == '__main__':
    try:
        print('Input argmunt', sys.argv)
        
        RecordJointsStates_instance = RecordJointsStates('conveyerBelt_basket')
        signal.signal(signal.SIGINT, RecordJointsStates_instance.shutdown_command)
        if not rospy.is_shutdown():
            RecordJointsStates_instance.run()
        
    except rospy.ROSInterruptException:
        pass
