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

from multiprocessing import Lock

import sys
sys.path.insert(0, "/home/lukas/catkin_ws/src/wheel_polishing/scripts/")



class PlaySequenceDS():
    def __init__(self, ds_names, ds_paths):
        self.mutex = Lock()

        self.start_time = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        rospy.init_node("listener_positionMeasure", anonymous=True)

        self.count_eePose = 0
        self.recieved_forceTorque = False
        
        rospy.Subscriber("/lwr/ee_pose", Pose, self.callback_eePose)
        #rospy.Subscriber("/lwr/ee_vel", Twist, self.callback_eeVel)
        rospy.Subscriber("/ft_sensor/netft_data", WrenchStamped, self.callback_forceTorque)

        self.pub_gravComp = rospy.Publisher("/lwr/joint_controllers/command_grav", Bool, queue_size=5)
        
        self.pub_gravComp = rospy.Publisher("/lwr/joint_controllers/ee_vel", Bool, queue_size=5)
        
        # self.n_joints = 7
        self.dimension = 3

        self.motion_states_list = {"home":0,"write":1,"reset":2}
        self.motion_state =  self.motion_states_list["home"]
        self.last_force_recieved=0

        # Initialize topics
        self.recorded_data = []

        self.freq = 50 # Warning
        self.rate = rospy.Rate(self.freq)    # Frequency 10Hz

        self.shutdown_executed = False

        print('Node initialized')
        
        while (not rospy.is_shutdown() 
               and
               (not self.count_eePose 
               or not self.recieved_forceTorque) ):

            print("waiting for callback messages..")
            rospy.sleep(0.25)

    def run(self):              # 
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

                new_DS = Twist()

                self.estimate_desired_moition_state()
                
                # if self.motion_state==self.motion_states_list["recording"]:
                if self.motion_state==self.motion_states_list["home"]:
                    self.get_home_ds()

                if self.motion_state==self.motion_states_list["write"]:
                    # check for unusual force
                    if :
                        
                    
                    # Evaluate new velocity
                    if :
                        self.
                        
                if self.motion_state==self.motion_states_list["reset"]:
                    # go back to initial position
                    self.get_reset_ds()
                    


                    # new_dataPoint += [self.eePose_msg.position.x, self.eePose_msg.position.y, self.eePose_msg.position.z]

                    # new_dataPoint += [self.eePose_msg.orientation.x, self.eePose_msg.orientation.y, self.eePose_msg.orientation.z, self.eePose_msg.orientation.w]
                    # pos_old = np.array([self.eePose_msg.position.x, self.eePose_msg.position.y, self.eePose_msg.position.z])
                    
                    # new_dataPoint += [self.eeVel_msg.linear.x, self.eeVel_msg.linear.y, self.eeVel_msg.linear.z]
                    # new_dataPoint += [self.eeVel_msg.angular.x, self.eeVel_msg.angular.y, self.eeVel_msg.angular.z]
                    
                    # new_dataPoint += [self.forceTorque_msg.wrench.force.x, 
                                      # self.forceTorque_msg.wrench.force.y, 
                                      # self.forceTorque_msg.wrench.force.z]
                    # new_dataPoint += [self.forceTorque_msg.wrench.torque.x,
                                      # self.forceTorque_msg.wrench.torque.y,
                                      # self.forceTorque_msg.wrench.torque.z]
                    self.get_ds()
            self.rate.sleep()


    def shutdown_command(self, sig, frame):
        rospy.signal_shutdown('Shutdown initiated....')
        print('Shutdown initiated....')

        if self.shutdown_executed:
            return 

        self.shutdown_executed = True
        print('... finished shutdown.')

    
    def estimate_desired_moition_state(self):
        pass

    
    def get_home_ds(self, pos, home_position=[-0.6, 0, 0.6], max_velocity=0.3):
        desired_velocity = np.array(home_position) - pos
        vel_norm = LA.norm(desired_velocity)
        if vel_norm>max_velocity:
            desired_velocity *= max_velocity/vel_norm
            
        return desired_velocity

    
    def get_reset_ds(self, pos, orientation=None, max_velocity=0.3):
        # DS to reset between attractors and to converge to initial
        
        desired_velocity = (self.reset_attractor-pos)
        if self.dimension:
            lift_velocity = self.lift_velocity_linear(LA.norm(self.reset_attractor-pos),vel_repulsion_max=0.3)
            desired_velocity[2] += lift_velocity

        if not orientation_matrix==None:
            orientation_matrix = orientation
            desired_velocity = orientation_matrix.dot(desired_velocity)

        vel_norm = LA.norm(desired_velocity)
        if vel_norm>max_velocity:
            desired_velocity *= max_velocity/vel_norm
            
        return desired_velocity


    def lift_velocity_linear(self, dist_attr, dist_convegence=0.2, vel_repulsion_max=0.3):
        # This function pushes 
        if dist_attr > dist_convegence:
            return vel_repulsion_max
        else:
            return dist_attr/dist_convegence*vel_repulsion_max
    

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

        
    def callback_eePose(self, msg):
        with self.mutex:
            self.count_eePose += 1
            self.eePose_msg = msg

    def callback_forceTorque(self, msg):
        with self.mutex:
            self.forceTorque_msg = msg

        self.recieved_forceTorque = True


if __name__ == '__main__':
    try:
        print('Input argmunt', sys.argv)

        ds_names = [""]
        ds_paths = [""]
        write_letters = PlaySequenceDS(ds_names, ds_paths)
        # Capture CRTL+C to for extra command
        signal.signal(signal.SIGINT, PlaySequenceDS.shutdown_command)
        if not rospy.is_shutdown():
            PlaySequenceDS.run()
        
    except rospy.ROSInterruptException:
        pass
