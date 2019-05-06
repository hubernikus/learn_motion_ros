#!/usr/bin/env python2
'''
Movement of EE of the KUKA to move to motion.
n
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

import sys
sys.path.insert(0, "/home/lukas/catkin_ws/src/wheel_polishing/scripts/") # Quaternion class
from class_quaternion import QuatClass

# TODO -- regression algorithm

class ReplayCallibration_KUKA():
    def __init__(self, setUp="", n_loops=0, input_file="recordW01.csv"):
    #def __init__(self, setUp="", n_loops=0, input_file=[]):
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

        self.n_joints = 7 # == DIM

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
        if len(input_file):
            if input_file.find(".") == -1: # Default file name
                input_file = input_file + ".csv"
            data_csv = np.genfromtxt(input_file, delimiter=",")
            
            # import pdb; pdb.set_trace()
            # position 
            self.attractors = data_csv[:, 22:25]

            #  quaternions
            self.quaternions = data_csv[:, 25:29]

            self.ds_scaling = np.ones(self.attractors.shape[0])*3
            
            print('shape', data_csv.shape)

        else:
            print("Move default movement")
            center=[-0.5, 0.15, 0.37]
            # self.attractors = np.array([[0,0,0],
            # [0,0,0],
            # [0,0,0])
            self.attractors = np.tile(center, (3,1))
            self.attractors[0,:1] += 0.2
            self.attractors[1,0] += 0.2
            self.attractors[2,1] += 0.2

            self.ds_scaling = np.array([1,1,1])

            quat_const = [0.83, 0.53, -0.06, -0.15]
            self.quaternions = np.tile(quat_const, (self.attractors.shape[0], 1))

        self.weight_ds = 1
        self.it_ds = 0
        self.it_loop=0


    def run(self):
        # TODO more general, to include more points in spline
        print('Starting loop')
        while not rospy.is_shutdown():
            
            if self.weight_ds >= 1:
                self.attractor_is_reached()

            #print(self.weight_ds)
            if self.weight_ds >= 1:
                self.ds_linear()
            else:
                self.transition_ds() 

            # Publish messages
            msg_eeVel = Twist()
            msg_eeVel.linear = Vector3(self.ds_final[0], self.ds_final[1], self.ds_final[2])
            msg_eeVel.angular = Vector3(0,0,0)
            self.pub_eeVel.publish(msg_eeVel)

            self.get_desired_orientation()
            msg_eeOrientation = Quaternion(self.des_quat[0],self.des_quat[1],self.des_quat[2],self.des_quat[3])
            self.pub_eeOrient.publish(msg_eeOrientation)

            # TODO update spline each loop to have flexible DS
            self.it_loop += 1
            
            # if not(self.it_loop%self.dt_loopPrint):
                # print('Loop #', self.it_loop)
            if not(self.it_loop%self.dt_loopPrint):
                print('Loop #{}'.format(self.it_loop) )

                print("The goal is at:", self.attractors[self.it_ds,:])

            self.rate.sleep()


    def attractor_is_reached(self, margin_attractor=0.05): 
        if margin_attractor > LA.norm(np.array(self.ee_pos)-np.array(self.attractors[self.it_ds, :])):
            self.weight_ds = 0
            print('Starting transition period')
            # return True
        # else:
            # return False


    def transition_ds(self, t_transition=0.25):
        self.ds_linear()
        self.ds_angular()
        ds_1 = self.ds_final[:]
        dsAng_1 = self.dsAng_final[:]

        self.ds_linear()
        self.ds_angular()

        it2_ds = np.mod(self.it_ds+1, self.ds_scaling.shape[0])
        self.ds_final = self.ds_final[:]*self.weight_ds + ds_1*(1-self.weight_ds)
        self.dsAng_final = self.dsAng_final[:]*self.weight_ds + dsAng_1*(1-self.weight_ds)
        
        self.weight_ds += 1/t_transition*self.dt_pub

        if self.weight_ds>=1:
            self.it_ds += 1
            print('Moving to next attractor')
            if self.it_ds>= self.ds_scaling.shape[0]:
                self.it_ds = 0


    def ds_linear(self, vel_max=0.1):
        self.ds_final = self.ds_scaling[self.it_ds]*(np.array(self.attractors[self.it_ds, :])-np.array(self.ee_pos))

        if LA.norm(self.ds_final) > vel_max:
            self.ds_final = self.ds_final/LA.norm(self.ds_final)*vel_max

        
    def ds_limit_cycle(self, center=[-5, 0.15, 0.37], rad_init=0.2, pow_dist=3, vel_max=0.1):

        dist_rel = LA.norm(pos_rel)
        ds_rad = -pos_rel*(dist_rel**(pow_dist-1))

        self.ds_final = ds_ang+ds_rad
        
        if LA.norm(self.ds_final) > vel_max:
            self.ds_final = self.ds_final/LA.norm(self.ds_final)*vel_max


    def ds_angular(self, angVel_max=0.1):
        # print('shape ang', self.quaternions[self.it_ds,:].squeeze().shape)
        # print('qut' , self.quaternions[self.it_ds,:].squeeze())
        self.dsAng_final = QuatClass(self.ee_quat).get_angularVel(self.quaternions[self.it_ds,:].squeeze(), dt=self.dt_pub)

        if LA.norm(self.dsAng_final) > angVel_max:
            self.dsAng_final = self.dsAng_final/LA.norm(self.dsAng_final)*angVel_max


    def get_desired_orientation(self):
        # Desired quaternion [x,y,z, w]
        #self.des_quat = np.array([0, 0, 0, 1])
        #self.des_quat = np.array([0, 0, 1, 0])
        # self.des_quat = np.array([0.67, 0.74, -0.01, 0.00])
        # self.des_quat = np.array([0.83, 0.53, -0.06, -0.15])
        self.des_quat = self.quaternions[self.it_ds, :]


    def shutdown_command(self, signal, frame):
        # TODO activate after CTRL-c
        print('\n\n\nShutting down....')

        # msg_jointVel = JointTrajectory()
        # msg_jointVel.header.stamp = rospy.Time.now()

        # msg_jointVel.joint_names = 
        # msg_jointVel.points.velocities = des_joint_vel.tolist()

        # newPoint = JointTrajectoryPoint()
        # newPoint.newPoint.velocities = np.zeros(self.joint_pos).tolist()
        # newPoint.newPoint.velocities = np.zeros(self.n_joints).tolist()
        # for ii in range(3): # repeat several times
            # msg_jointVel.points.append(newPoint)

        # self.pub_jointVel.publish(msg_jointVel)
        self.pub_gravComp.publish(Bool(True))
        
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
            self.ee_quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

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

                    
        if len(sys.argv)==1:
            ReplayCallibration_instance = ReplayCallibration_KUKA()
        else:
            ReplayCallibration_instance = ReplayCallibration_KUKA(input_file=sys.argv[1])

        signal.signal(signal.SIGINT, ReplayCallibration_instance.shutdown_command)

        if not rospy.is_shutdown():
            ReplayCallibration_instance.run()
        
    except rospy.ROSInterruptException:
        pass

