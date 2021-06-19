#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ORTILo.msg import StateVector
import csv

class DataPlotter:
    def __init__(self):
        self.flag0 = 0
        self.flag1 = 0
        self.flag2 = 0
        self.pathFlag = 0
        self.initTime = 0
        self.actualPose = PoseStamped()
        self.desiredPath = Path()
        self.desiredPathX = []
        self.desiredPathY = []
        self.kalmanEstimatedX = 0.0
        self.kalmanEstimatedY = 0.0
        self.kalmanPoseList = {}
        self.kalmanPoseList["x"] = []
        self.kalmanPoseList["y"] = []
        self.kalmanPoseList["z"] = []
        self.mse = 0.0
        self.mseKalman_X = 0.0
        self.mseKalman_Y = 0.0
        self.mseKalman_Z = 0.0
        self.mseCount = 0.0
        self.mseKalmanCount = 0.0

        self.actualPoseList = {}
    
        self.actualPoseList["x"] = []
        self.actualPoseList["y"] = []
        self.actualPoseList["z"] = []
        self.actualPoseList["time"] = []
        self.actualPoseList["r"] = []

        # Open csv file
        self.f = open('ate', 'w')
        self.writer = csv.writer(self.f)
        row0 = ['Actual X' , 'Desired X', 'Actual Y', 'Desired Y']
        self.writer.writerow(row0)

    def actualPoseCb(self, msg):
        if(self.flag0 == 0):
            self. flag0 = 1
            self.initTime = int(str(msg.header.stamp))
        self.actualPose = msg
        self.actualPoseList["x"].append(msg.pose.position.x)
        self.actualPoseList["y"].append(msg.pose.position.y)
        self.actualPoseList["z"].append(msg.pose.position.z)
        self.actualPoseList["time"].append((float(str(msg.header.stamp)) - self.initTime)/1000000000.0)


        if(msg.pose.position.x >=21.0):
            #print('DONE!!!!')
            self.flag1 = 1

    def desiredPathCb(self, msg):
        if (self.pathFlag == 0):
            self.desiredPath = msg
            self.pathFlag = 1
    
    def kalmanEstimatedPoseCb(self, msg):
        self.kalmanEstimatedX = msg.x
        self.kalmanEstimatedY = msg.y
        self.kalmanEstimatedZ = msg.z
        self.kalmanPoseList["x"].append(msg.x)
        self.kalmanPoseList["y"].append(msg.y)
        self.kalmanPoseList["z"].append(msg.z)
        self.mseKalman_X = self.mseKalman_X + ( msg.x - self.actualPose.pose.position.x ) ** 2
        self.mseKalman_Y = self.mseKalman_Y + ( msg.y - self.actualPose.pose.position.y ) ** 2
        self.mseKalman_Z = self.mseKalman_Z + ( msg.z - self.actualPose.pose.position.z ) ** 2
        self.mseKalmanCount = self.mseKalmanCount + 1
   
    def main(self):
        rospy.init_node('coord_plotter', anonymous=True)
        rate = rospy.Rate(50)
        print('coord_plotter node has been successfully initialized')

        rospy.Subscriber('/uav1/pose', PoseStamped, self.actualPoseCb)

        rospy.Subscriber('/smoothedPath', Path, self.desiredPathCb)
        rospy.Subscriber('/kalman_filter_estimated_state', StateVector, self.kalmanEstimatedPoseCb)

        major_ticks_x = np.arange(0, 20, 5)
        minor_ticks_x = np.arange(0, 20, 1)        
        major_ticks_y = np.arange(-3, 2, 1)
        minor_ticks_y = np.arange(-3, 2, 0.25)                        

        while not rospy.is_shutdown():
            if(self.flag1 == 1 and self.flag2 == 0):

                for i in range(len(self.desiredPath.poses)):
                    self.desiredPathX.append(self.desiredPath.poses[i].pose.position.x)
                    self.desiredPathY.append(self.desiredPath.poses[i].pose.position.y)

                    for j in range(len(self.actualPoseList['x'])):
                        if( abs( self.desiredPath.poses[i].pose.position.x - self.actualPoseList['x'][j] ) <= 0.01 ):
                            # Mean Squared Error
                            self.mse = self.mse + ( self.desiredPath.poses[i].pose.position.y - self.actualPoseList['y'][j] ) ** 2
                            self.mseCount = self.mseCount + 1.0
                            # print('mse = {}' .format(self.mse))
                            
                            # Write data in csv file
                            row = [self.actualPoseList['x'][j], self.desiredPath.poses[i].pose.position.x, self.actualPoseList['y'][j], self.desiredPath.poses[i].pose.position.y]
                            self.writer.writerow(row)
                            break
                self.f.close()

                #self.mse = self.mse/len(self.desiredPath.poses)
                self.mse = self.mse/self.mseCount
                self.mseKalman_X = self.mseKalman_X/self.mseKalmanCount
                self.mseKalman_Y = self.mseKalman_Y/self.mseKalmanCount
                self.mseKalman_Z = self.mseKalman_Z/self.mseKalmanCount
                print('number of points for Path = {}' .format(len(self.desiredPath.poses)))
                print('Path mse = {}' .format(self.mse))

                print('mseKalmanX = {}' .format(self.mseKalman_X))
                print('mseKalmanY = {}' .format(self.mseKalman_Y))
                print('mseKalmanZ = {}' .format(self.mseKalman_Z))
                
                plt.xlim (0, 20)
                plt.ylim (-3, 2)
                fig1, ax = plt.subplots()

                ax.set_xticks(major_ticks_x)
                ax.set_xticks(minor_ticks_x, minor=True)
                ax.set_yticks(major_ticks_y)
                ax.set_yticks(minor_ticks_y, minor=True)

                ax.plot(self.actualPoseList["x"], self.actualPoseList["y"], label='Actual Path')
                
                plt.xlim (0, 20)
                plt.ylim (-3, 2)
                ax.legend() 
                ax.set_xlabel('X Coordinate (m)')
                ax.set_ylabel('Y Coordinate (m)')
                ax.grid(which='both')
                ax.grid(which='minor', alpha=0.3)
                ax.grid(which='major', alpha=0.7)

                ax.plot(self.desiredPathX, self.desiredPathY, label='Goal Path')
                ax.legend()

                # Kalman Filter
                fig2, ax2 = plt.subplots()

                ax2.set_xticks(major_ticks_x)
                ax2.set_xticks(minor_ticks_x, minor=True)
                ax2.set_yticks(major_ticks_y)
                ax2.set_yticks(minor_ticks_y, minor=True)

                ax2.plot(self.actualPoseList["x"], self.actualPoseList["y"], label='Actual Position')

                plt.xlim (0, 20)
                plt.ylim (-3, 2)
                ax2.legend()
                ax2.set_xlabel('X Coordinate (m)')
                ax2.set_ylabel('Y Coordinate (m)')
                ax2.grid(which='both')
                ax2.grid(which='minor', alpha=0.3)
                ax2.grid(which='major', alpha=0.7)
                                
                ax2.plot(self.kalmanPoseList["x"], self.kalmanPoseList["y"], label='Estimated Position')
                ax2.legend()

                plt.show()
                fig1.show()
                fig2.show()

            rate.sleep()                
            #rospy.spin()

if __name__ == '__main__':
    dataPlotterObj = DataPlotter()
    dataPlotterObj.main()

