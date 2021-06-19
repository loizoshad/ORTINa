#!/usr/bin/env python

import rospy
import numpy as np  
import math 

from scipy.optimize import fsolve
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mav_msgs.msg import Actuators
from ORTILo.msg import ControlVector
from ORTILo.msg import StateVector
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class KalmanFilter:
    def __init__(self):
        self.count = 0
        self.yolo_flag_x = 0
        self.yolo_flag_y = 0
        self.yolo_flag_z = 0
        self.uwb_flag = 0

        self.kalmanEstimatedState = StateVector()
        self.imuData = Imu()
        self.uwbPose = PoseStamped()
        self.uwbPose.pose.position.x = -10
        self.uwbPose.pose.position.y = -10
        self.uwbPose.pose.position.z = -10
        self.cameraPose = PoseStamped()
        self.cameraPose.pose.position.x = -10
        self.cameraPose.pose.position.y = -10
        self.cameraPose.pose.position.z = -10       
        self.actualPose = PoseStamped()
        self.odometryData = Odometry()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        #self.mass = 2.7
        #self.armLength = 0.17

        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 0.0

        # X = A*x + B*u + W
        # State Vector
        self.X = np.array([
            [0.0],    # x 
            [0.0],    # y
            [0.08],     # z
            [0.0],   # phi (roll)
            [0.0],   # theta (pitch)
            [0.0],   # psi (yaw)
            [0.0],   # x_dot
            [0.0],   # y_dot
            [0.0],   # z_dot
            [0.0],   # phi_dot
            [0.0],   # theta_dot
            [0.0]    # psi_dot
        ])
        

        # State Transition Matrix
        self.A = np.array([
            [1.0000,         0,         0,         0,    0.0054,         0,    0.0333,         0,         0,         0,    0.0001,         0],
            [     0,    1.0000,         0,   -0.0054,         0,         0,         0,    0.0333,         0,   -0.0001,         0,         0],
            [     0,         0,    1.0000,         0,         0,         0,         0,         0,    0.0333,         0,         0,         0],
            [     0,         0,         0,    1.0000,         0,         0,         0,         0,         0,    0.0333,         0,         0],
            [     0,         0,         0,         0,    1.0000,         0,         0,         0,         0,         0,    0.0333,         0],
            [     0,         0,         0,         0,         0,    1.0000,         0,         0,         0,         0,         0,    0.0333],
            [     0,         0,         0,         0,    0.3269,         0,    1.0000,         0,         0,         0,    0.0054,         0],
            [     0,         0,         0,   -0.3269,         0,         0,         0,    1.0000,         0,   -0.0054,         0,         0],
            [     0,         0,         0,         0,         0,         0,         0,         0,    1.0000,         0,         0,         0],
            [     0,         0,         0,         0,         0,         0,         0,         0,         0,    1.0000,         0,         0],
            [     0,         0,         0,         0,         0,         0,         0,         0,         0,         0,    1.0000,         0],
            [     0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,    1.0000]
        ])

        self.B = np.array([
            [     0,         0, 3.0409e-6,         0],
            [     0, -3.259e-6,         0,         0],
            [0.0002,         0,         0,         0],
            [     0,    0.0036,         0,         0],
            [     0,         0,    0.0033,         0],
            [     0,         0,         0,    0.0027],
            [     0,         0,    0.0004,         0],
            [     0,   -0.0004,         0,         0],
            [0.0123,         0,         0,         0], 
            [     0,    0.2154,         0,         0],
            [     0,         0,    0.2009,         0],
            [     0,         0,         0,    0.1605]
        ])


        self.W = np.zeros((12, 1))
        self.W[8] = -0.3298883333 # discrete-time

        self.U = np.zeros((4, 1))

        self.C = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],         
        ])  

        # Used for the sensor fusion: C_aug = E*C
        self.E = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ])

        self.C_aug = np.zeros((9, 9))

        '''
        P: Initial process covariance matrix. It models the errors in the calculation of the state variables
        This matrix is filled based on the initialization error. That is, how much our initial belief about 
        the state deviates from the actual state. The higher the confidence we have in our initial belief,
        the smaller should the the covariances and variances in 'P' be.
        '''    
        self.P = np.array([
            [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        ]) 
  
        self.K = np.zeros((12, 9))

        # Process Covariance Matrix 
        self.Q = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ])
        
        self.R = np.array([
            [0.01, 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0  , 0.0  , 0.0  ], # uwb x
            [0.0 , 0.01, 0.0 , 0.0 , 0.0 , 0.0 , 0.0  , 0.0  , 0.0  ], # uwb y
            [0.0 , 0.0 , 0.01, 0.0 , 0.0 , 0.0 , 0.0  , 0.0  , 0.0  ], # uwb z
            [0.0 , 0.0 , 0.0 , 0.05, 0.0 , 0.0 , 0.0  , 0.0  , 0.0  ], # YOLO x
            [0.0 , 0.0 , 0.0 , 0.0 , 0.05, 0.0 , 0.0  , 0.0  , 0.0  ], # YOLO y
            [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.1, 0.0  , 0.0  , 0.0  ], # YOLO z
            [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.01 , 0.0  , 0.0  ], # IMU roll
            [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0  , 0.01 , 0.0  ], # IMU pitch
            [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0  , 0.0  , 0.01 ]  # IMU yaw
        ])

        # Measurement Vector (Update it with the data received from the sensors)
        self.Z = np.array([
            [0.0],   # x (UWB)
            [0.0],   # y (UWB)
            [0.08],  # z (UWB)
            [0.0],   # x (yolo)
            [0.0],   # y (yolo)
            [0.0],   # z (yolo)
            [0.0],   # roll
            [0.0],   # pitch
            [0.0]    # yaw
        ])

    # --------------------------------------------------------------------------- KALMAN FILTER FUNCTIONS ---------------------------------------------------------------------------  
    def kalmanFilter(self):
        # Prediction Step
        self.X = np.dot(self.A, self.X) + np.dot(self.B, self.U) + self.W # X[k+1] = A*X[k] + B*U[k] + W
        self.P = np.dot(np.dot(self.A, self.P), self.A.transpose()) + self.Q
             
        if(self.uwb_flag == 0 and self.count >= 500):
            self.E[0][0] = 0.0
            self.E[1][1] = 0.0
            self.E[2][2] = 0.0
        else:
            self.E[0][0] = 1.0
            self.E[1][1] = 1.0
            self.E[2][2] = 1.0            
            self.uwb_flag = 0

        if(self.yolo_flag_x == 0):
            self.E[3][3] = 0.0
        else:
            self.E[3][3] = 1.0            
            self.yolo_flag_x = 0

        if(self.yolo_flag_y == 0):
            self.E[4][4] = 0.0
        else:
            self.E[4][4] = 1.0
            self.yolo_flag_y = 0

        if(self.yolo_flag_z == 0):
            self.E[5][5] = 0.0
        else:
            self.E[5][5] = 1.0
            self.yolo_flag_z = 0

        # Update Step
        self.C_aug = np.dot(self.E, self.C) # C_aug = E*C
        self.S = np.dot(np.dot(self.C_aug, self.P), self.C_aug.transpose()) + self.R # S = C_aug*P*C_aug' + R
        self.innov = np.linalg.pinv(self.S) # innov = (C_aug*P*C_aug' + R)^+
        self.V = self.Z - np.dot(self.C_aug, self.X) # v = y[k] - C_aug*X[k], where y[k]: measurements from sensors

        self.K = np.dot(np.dot(self.P, self.C_aug.transpose()), self.innov) # K = P*C_aug'*(C_aug*P*C_aug' + R)^-1
        self.X = self.X + np.dot(self.K, self.V) # X[k+1] = A*X[k] + B*U[k] + K*(y[k] - C_aug*X[k])
        self.P = self.P - np.dot(np.dot(self.K, self.C_aug), self.P) # P = P - K*C_aug*P

        self.hold_x = self.X[0]
        self.hold_y = self.X[1]
        self.hold_z = self.X[2]   

        self.count = self.count + 1        

        self.kalmanEstimatedState.x = self.X[0]
        self.kalmanEstimatedState.y = self.X[1]
        self.kalmanEstimatedState.z = self.X[2]  
        self.kalmanEstimatedState.phi = self.X[3]
        self.kalmanEstimatedState.theta = self.X[4]
        self.kalmanEstimatedState.psi = self.X[5]
        self.kalmanEstimatedState.u_x = self.X[6]
        self.kalmanEstimatedState.u_y = self.X[7]
        self.kalmanEstimatedState.u_z = self.X[8]
        self.kalmanEstimatedState.w_phi = self.X[9]
        self.kalmanEstimatedState.w_theta = self.X[10]
        self.kalmanEstimatedState.w_psi = self.X[11]                    
    # --------------------------------------------------------------------------- CALLBACK FUNCTION ---------------------------------------------------------------------------
    def controlInputCb(self, msg):
        self.U[0] = msg.f_t
        self.U[1] = msg.tau_x
        self.U[2] = msg.tau_y
        self.U[3] = msg.tau_z

    def uwbPoseCb(self, msg):
        self.uwbPose = msg
        self.Z[0] = self.uwbPose.pose.position.x # Position in X axis
        self.Z[1] = self.uwbPose.pose.position.y # Position in Y axis
        self.Z[2] = self.uwbPose.pose.position.z # Position in Z axis    
        self.uwb_flag = 1

    def cameraPoseCb(self, msg):
        self.cameraPose = msg
        self.Z[3] = self.cameraPose.pose.position.x # Position in X axis
        self.Z[4] = self.cameraPose.pose.position.y # Position in Y axis
        self.Z[5] = self.cameraPose.pose.position.y # Position in Z axis
        
    
        if(abs(self.Z[3] - self.hold_x) <= 0.05):
            self.yolo_flag_x = 1
        if(abs(self.Z[4] - self.hold_y) <= 0.07):
            self.yolo_flag_y = 1
        if(abs(self.Z[5] - self.hold_z) <= 0.05):
            self.yolo_flag_z = 1      
        
    def imuCb(self, msg):
        self.imuData = msg
        # quaternions to euler angles
        quat = msg.orientation
        orientationList = [quat.x, quat.y, quat.z, quat.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientationList)        
        self.Z[6] = self.roll
        self.Z[7] = self.pitch
        self.Z[8] = self.yaw


    def actualPoseCb(self, msg): # Debugging
        self.actualPose = msg     
    def odometryCb(self, msg): # Debugging
        self.odometryData = msg
    # --------------------------------------------------------------------------- MAIN FUNCTION ---------------------------------------------------------------------------
    def mainLoop(self):
        self.kalmanFilter()
        self.pub.publish(self.kalmanEstimatedState)

    def main(self):
        rospy.init_node('kalman_filter', anonymous=True)
        rate = rospy.Rate(30)
        print("kalman_filter node has been successfully initialized")
    
        rospy.Subscriber('/uav1/imu', Imu, self.imuCb)
        rospy.Subscriber('/control_input', ControlVector, self.controlInputCb)
        rospy.Subscriber('/uwb_estimated_position', PoseStamped, self.uwbPoseCb)
        rospy.Subscriber('/camera_estimated_position', PoseStamped, self.cameraPoseCb)
        rospy.Subscriber('/uav1/pose', PoseStamped, self.actualPoseCb)
        rospy.Subscriber('/uav1/odometry', Odometry, self.odometryCb)

        self.pub=rospy.Publisher('/kalman_filter_estimated_state', StateVector, queue_size=1)

        while not rospy.is_shutdown():
            kf.mainLoop()
            self.kalmanEstimatedState.header.stamp = rospy.Time.now()
            rate.sleep()

if __name__ == '__main__':
    kf = KalmanFilter()
    kf.main()

