#!/usr/bin/env python

import rospy
import math
import numpy as np
from scipy.optimize import fsolve

from geometry_msgs.msg import PoseStamped
from pozyx_simulation.msg import uwb_data
from ORTILo.msg import StateVector

class UwbPoseEstimator:
    def __init__(self):

        # self.anchorPositionMatrix[anchor_id][axis]
        self.anchorPositionMatrix = np.array([
            [-0.345296, 0.022235,  2.139182],  # 0
            [0.210963,  2.940959,  3.00000000], # 1
            [4.031500,  -0.070944, 2.675591], # 2
            [9.995051, 2.770610,  2.422272],   # 3
            [8.551760,  -0.693485, 2.308465], # 4
            [11.188900, 1.057870,  2.031038], # 5
            [14.338000, 1.112870,  2.101508], # 6
            [17.053692, -0.475284, 2.614461], # 7
            [16.645300, 4.044180,  1.130429], # 8
            [7.9943230, -4.205842, 2.987858], # 9
            [16.944441, -1.759508, 2.69665045], # 10
            [16.0     , 0.45     , 0.35    ] # 11
        ])

        self.numOfAnchors = 12
        self.distances = [0.0] * self.numOfAnchors
        self.id = [0] * self.numOfAnchors
        self.estimatedPosition = PoseStamped()
        self.prevPoseStamped = StateVector()
        self.actualPoseStamped = PoseStamped()
        # Initialize it to use it as the initial guess for the fsolve() function
        self.prevPoseStamped.x = 0.1
        self.prevPoseStamped.y = 0.1
        self.prevPoseStamped.z = 0.1
        
    # --------------------------------------------------- POSE ESTIMATOR FUNCTIONS ---------------------------------------------------
    def posSolver(self, xyz):
        self.x = xyz[0]
        self.y = xyz[1]
        self.z = xyz[2]

        self.f = (self.x - self.px[0])**2 + (self.y - self.py[0])**2 + (self.z - self.pz[0])**2 - self.r[0]**2
        self.g = (self.x - self.px[1])**2 + (self.y - self.py[1])**2 + (self.z - self.pz[1])**2 - self.r[1]**2
        self.h = (self.x - self.px[2])**2 + (self.y - self.py[2])**2 + (self.z - self.pz[2])**2 - self.r[2]**2

        return np.array([self.f, self.g, self.h])


    def estimatePosition(self):        

        self.px = [self.anchorPositionMatrix[self.id[0]][0], self.anchorPositionMatrix[self.id[1]][0], self.anchorPositionMatrix[self.id[2]][0]]
        self.py = [self.anchorPositionMatrix[self.id[0]][1], self.anchorPositionMatrix[self.id[1]][1], self.anchorPositionMatrix[self.id[2]][1]]
        self.pz = [self.anchorPositionMatrix[self.id[0]][2], self.anchorPositionMatrix[self.id[1]][2], self.anchorPositionMatrix[self.id[2]][2]]

        self.r = [self.distances[0], self.distances[1], self.distances[2]]

        self.xyz0 = np.array([self.prevPoseStamped.x, self.prevPoseStamped.y, self.prevPoseStamped.z])
        self.xyz = fsolve(self.posSolver, self.xyz0)

        self.estimatedPosition.pose.position.x = self.xyz[0]
        self.estimatedPosition.pose.position.y = self.xyz[1]
        self.estimatedPosition.pose.position.z = self.xyz[2]   
    
    def insertionSort(self):

        # We ascendingly sort the lists that contain the distance from each UWB anchor to the quadrotor. At the same time the respective ID's from the self.uwbAnchors.destination_id
        # list are swapped accordinly, so as to follow their assigned distance.

        for i in range(0, len(self.distances) - 1):
            self.key = self.distances[i]
            self.key2 = self.id[i]
            j = i-1
            while j >= 0 and self.key < self.distances[j]:
                self.distances[j+1] = self.distances[j]
                self.id[j+1] = self.id[j]
                j -= 1
            self.distances[j+1] = self.key
            self.id[j+1] = self.key2


    # --------------------------------------------------- CALLBACK FUNCTIONS ---------------------------------------------------
    
    def prevPositionCallback(self, msg):
        self.prevPoseStamped = msg

    def actualPoseCallback(self, msg):
        self.actualPoseStamped = msg

    def uwbCallback(self, msg):

        if (len(msg.destination_id) >= 3):
            self.id = list(msg.destination_id)
            self.distances = np.array(list(msg.distance))
            self.distances = self.distances/1000.0
            self.insertionSort()

            flag = 0           

            # Over here we can use the actual position of the quadrotor as it is given by the simulator.
            # This is the case because this is only used to ignore the measurements of the UWB during specific periods of the flight
            # We use this so we can test the performance of the estimator (sensor fusion) when there are no available measurements from the UWB system
            if(self.actualPoseStamped.pose.position.x >= 1.5 and self.actualPoseStamped.pose.position.x < 5.5):
                flag = 1
            if(self.actualPoseStamped.pose.position.x >= 12.9 and self.actualPoseStamped.pose.position.x < 14.0):
                flag = 1                
            if(self.actualPoseStamped.pose.position.x >= 16.9 and self.actualPoseStamped.pose.position.x < 18.8): 
                flag = 1                           

            if(flag == 0):
                self.estimatePosition()
                self.pub.publish(self.estimatedPosition)

            self.rate.sleep()
        else:
            print("Not enough UWB anchors nearby: Only {} anchors were detected" .format(len(msg.destination_id)))    

    def main(self):
        rospy.init_node('uwb_pose_estimator', anonymous=True)
        self.rate = rospy.Rate(200)
        print("uwb_pose_estimator node has been successfully initialized")
    
        rospy.Subscriber('/uwb_data_topic', uwb_data, self.uwbCallback)
        rospy.Subscriber('/kalman_filter_estimated_state', StateVector, self.prevPositionCallback)
        rospy.Subscriber('/uav1/pose', PoseStamped, self.actualPoseCallback)
        self.pub=rospy.Publisher('/uwb_estimated_position', PoseStamped, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    uwbPoseEstimator = UwbPoseEstimator()    
    uwbPoseEstimator.main()    

