#!/usr/bin/env python2

'''
Movement of EE of the KUKA to learn

@author lukashuber
@date 2019-04-01
'''

import rospy
import rospkg
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped, Twist, Pose, PoseStamped, Quaternion, Vector3
from std_msgs.msg import Float64MultiArray, Bool

import numpy as np
import numpy.linalg as LA
import warnings

from os.path import join

import json

import warnings
import sys
import signal

from multiprocessing import Lock

DT_INPUT = 0.04 # make sure this is bigger than self.dt_pub

# TODO -- regression algorithm

class ReplayCallibration_KUKA():
    def __init__(self, setUp="", n_loops=0):
        self.n_joints = 7 # == DIM

        # n_loops: negative number account result in infinite loops
        self.mutex = Lock()
        rospack = rospkg.RosPack()
        rospy.init_node("talker_playBack", anonymous=True)
        
        self.pub_jointPos = rospy.Publisher("/lwr/joint_controllers/command_joint_pos", Float64MultiArray, queue_size=5)
        self.pub_gravComp = rospy.Publisher("/lwr/joint_controllers/command_grav", Bool, queue_size=5)

        self.pub_eeVel = rospy.Publisher("/lwr/joint_controllers/passive_ds_command_vel", Twist, queue_size=5)
        self.pub_eeOrient = rospy.Publisher("/lwr/joint_controllers/passive_ds_command_orient", Quaternion, queue_size=5)

        self.recieved_jointState_msg = False
        self.recieved_eeVel_msg = False
        self.recieved_eePose_msg = False

        # Initialize topics
        self.sub_joint_pos = rospy.Subscriber("/lwr/joint_states", JointState, self.callback_jointState)
        self.sub_eeVel = rospy.Subscriber("/lwr/ee_vel", Twist, self.callback_eeVel)
        self.sub_eeVel = rospy.Subscriber("/lwr/ee_pose", Pose, self.callback_eePose)

        self.freq = 50 # Warning - too high frequence leads to jerky behavior
        self.dt_pub = 1./self.freq
        self.rate = rospy.Rate(self.freq)    # Frequency 10Hz

        self.n_loops = n_loops # number times the points are looped (-1==inf)

        self.data_points = []
        self.pos_next = []
        # with open(join(rospack.get_path("robot_calibration"), "data", "lwr_calibration.json")) as json_data:
            # self.data_points p= json.load(json_data)

        self.n_points=len(self.data_points) # maximum number of points

        self.dt_loopPrint = 50

        while not (self.recieved_jointState_msg) and not rospy.is_shutdown():
            rospy.sleep(0.5)
            print("Waiting for first callbacks...")

        while not (self.recieved_eeVel_msg) and not rospy.is_shutdown():
            rospy.sleep(0.5)
            print("Waiting for first callbacks...")

        while not (self.recieved_eePose_msg) and not rospy.is_shutdown():
            rospy.sleep(0.5)
            print("Waiting for first callbacks...")

        self.it_loop = 0
        self.it_attr = 0

        start_margin = 0.1

        # Import Dynamic sytem parameters
        center=[-0.5, 0.15, 0.37]
        # self.attractors = np.array([[0,0,0],
                                    # [0,0,0],
                                    # [0,0,0])
        self.attractors = np.tile(center, (3,1))
        self.attractors[0,:1] += 0.2
        self.attractors[1,0] += 0.2
        self.attractors[2,1] += 0.2

        self.ds_scaling = np.array([1,1,1])

        self.weight_ds = 1
        self.it_ds = 0
        self.it_loop=0

        self.record_points = True


    def run(self):
        # TODO more general, to include more points in spline
        print('Starting loop')
        while not rospy.is_shutdown():
            
            self.pub_gravComp.publish(Bool(True))

            # TODO update spline each loop to have flexible DS
            self.it_loop += 1
            
            # if not(self.it_loop%self.dt_loopPrint):
                # print('Loop #', self.it_loop)
            if not(self.it_loop%self.dt_loopPrint):
                print('Loop #{}'.format(self.it_loop) )

                print("The goal is at:", self.attractors[self.it_ds,:])

            self.rate.sleep()


    def attractor_is_reached(self, margin_attractor=0.1):
        if margin_attractor > LA.norm(np.array(self.ee_pos)-np.array(self.attractors[self.it_ds, :])):
            self.weight_ds = 0
            # return True
        # else:
            # return False

    def transition_ds(self, weight_step=0.05):
        self.ds_linear()
        ds_1 = self.ds_final[:]

        it2_ds = np.mod(self.it_ds+1, self.ds_scaling.shape[0])
        self.ds_final = self.ds_final[:]*self.weight_ds + ds_1*(1-self.weight_ds)

        self.weight_ds += weight_step

        if self.weight_ds>=1:
            self.it_ds += 1
            if self.it_ds>= self.ds_scaling.shape[0]:
                self.it_ds = 0
        

    def get_desired_orientation(self):
        self.des_quat = np.array([0.83, 0.53, -0.06, -0.15])


    def shutdown_command(self, signal, frame):
        # TODO activate after CTRL-c
        print('Shuting down....')

        msg_jointVel = JointTrajectory()
        msg_jointVel.header.stamp = rospy.Time.now()

        msg_jointVel.joint_names = JOINT_NAMES
        # msg_jointVel.points.velocities = des_joint_vel.tolist()

        newPoint = JointTrajectoryPoint()
        newPoint.newPoint.velocities = np.zeros(self.joint_pos).tolist()
        newPoint.newPoint.velocities = np.zeros(self.n_joints).tolist()
        for ii in range(3): # repeat several times
            msg_jointVel.points.append(newPoint)

        self.pub_jointVel.publish(msg_jointVel)

        self.pub_gravComp.publish(Bool(true))
        
        rospy.signal_shutdown('User shutdown.')
        print('See you next time')


    def callback_jointState(self, msg):
        with self.mutex:
            # TODO apply transform to np.array only when used
            self.joint_pos = np.array(msg.position[:self.n_joints])
            self.joint_vel = np.array(msg.velocity[:self.n_joints])
            self.joint_torque = msg.effort[:self.n_joints]

            if not self.recieved_jointState_msg:
                self.recieved_jointState_msg = True
                print("Recieved first joint state msg.")


    def callback_eePose(self, msg):
        with self.mutex:
            # TODO apply transform to np.array only when used
            self.ee_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
            self.ee_quat = msg.orientation

            if not self.recieved_eePose_msg:
                self.recieved_eePose_msg = True
                print("Recieved first ee-Pose msg.")


    def callback_eeVel(self, msg):
        with self.mutex:
            # TODO apply transform to np.array only when used
            self.ee_velLin = msg.linear
            self.ee_velAng = msg.angular

            if not self.recieved_eeVel_msg:
                self.recieved_eeVel_msg = True
                print("Recieved first ee-Velocity msg.")


    def callback_eeAcc(self, msg):
        with self.mutex:
            # TODO apply transform to np.array only when used
            self.ee_accLin = msg.linear
            self.ee_accAng = msg.angular

            if not self.recieved_eeAcc_msg:
                self.recieved_eeAcc_msg = True
                print("Recieved first ee-Acceleration msg.")



if __name__ == '__main__':
    try:
        print('Input argmunt', sys.argv)
        
        ReplayCallibration_instance = ReplayCallibration_KUKA()
        signal.signal(signal.SIGINT, ReplayCallibration_instance.shutdown_command)

        if not rospy.is_shutdown():
            ReplayCallibration_instance.run()
        
    except rospy.ROSInterruptException:
        pass

