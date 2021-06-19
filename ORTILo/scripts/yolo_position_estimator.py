#!/usr/bin/env python

import rospy
import math
import numpy as np
from scipy.optimize import fsolve

from geometry_msgs.msg import PoseStamped
from ORTILo.msg import yolov3_bounding_box_vector
from ORTILo.msg import yolov3_bounding_box
from ORTILo.msg import StateVector

class CameraPoseEstimator:
	def __init__(self):
		self.averageX = 0.0
		self.averageY = 0.0
		self.averageZ = 0.0
		self.counter = 0.0

		self.actualPose = PoseStamped()
		self.prevPoseStamped = StateVector()
		self.validBoundingBoxes = yolov3_bounding_box_vector()

		self.prevPoseStamped.x = 1.6
		self.prevPoseStamped.y = 1.6
		self.prevPoseStamped.z = 1.6
	
		# final_7.world
		self.landmarkPositionMatrix = np.array([
									[6.6556810, -0.075008, 0.99665045], # person_1
									[10.396109, 2.4353880, 0.99665045], # person_2
									[4.5316200, 0.4173470, 0.16946200], # fire hydrant_1
									[5.4564900, 3.7004500, 0.16946200], # fire hydrant_2
									[5.5370100, 0.9533430, 0.32379000], # dog_1
									[8.5368900, 3.5412600, 0.32379000], # dog_2
									[3.3911900, -0.859048, 1.12384500], # potted plant_1
									[6.8321900, 3.7254300, 0.26257600], # potted plant_2
									[12.133500, -0.211472, 1.14009900], # potted plant_3
									[9.8989100, 3.1001200, 0.34448600], # chair_1
									[12.104700, 1.3027000, 0.54542000], # bicycle_1
									[3.7176100, -0.775260, 1.06947000], # bird_1
									[4.4414400, 3.8475200, 0.29079000], # cat_1
									[12.337500, 1.2592300, 2.20603000], # clock_1
									[13.361613, -1.141143, 0.32379000], # dog_3
									[17.106400, -3.566880, 0.32379000], # dog_4
									[13.904320, -3.785573, 0.99665045], # person_3
									[15.370720, -0.707531, 0.99665045], # person_4
									[18.352700, -3.157630, 0.46946200], # fire hydrant_3
									[19.297400, -3.365240, 1.14009900], # potted plant_4
									[19.586200, -2.292580, 0.29079000], # cat_2
									[20.426400, -0.926361, 0.54542000], # bicycle_2
									[20.708800, -1.048830, 2.20603000], # clock_2
									[21.258900, 3.9011100, 0.34448600], # chair_2
									[22.729200, 0.3598270, 0.32379000], # dog_5
									[25.081200, 3.7670400, 1.14009900], # potted plant_5
									[25.024500, 0.4175530, 0.26257600], # potted plant_6
									[26.845389, 0.8212110, 0.99665045], # person_5
									[27.289400, 1.8243900, 0.46946200]  # fire hydrant_4
									])	

		# The three following sets of data can be obtained by the camera_info topic
		# Check this link for more info http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html

		# Dimensions of the image in pixels:
		self.imageHeight = 1200
		self.imageWidth = 1600		

		# UAV-Camera offset
		self.x_offset = 0.049798
		self.y_offset = -0.001493
		self.z_offset = 0.0429629

		# Principal point of the image in pixels
		self.cx = 800.5
		self.cy = 600.5

		# Focal Length
		self.fx = 746.632634717237
		self.fy = 746.632634717237

		self.estimatedPosition = PoseStamped()

		self.objID = []

	# --------------------------------------------------------------------------- n OBJECTS, n >= 3 ---------------------------------------------------------------------------
	def posSolver(self, xyz):
		positionFunctions = []
		
		for i in range (len(self.objID)):

			positionFunctions.append( (xyz[0] - self.px[i])**2 + (xyz[1] - self.py[i])**2 + (xyz[2] - self.pz[i])**2 - self.calculatedLandmarkDistances[i]**2 )
			i = i + 1


		positionFunctions = np.array(positionFunctions)
		return positionFunctions

	def estimatePosition(self):	
		self.xyz_init = [1.6] * int(len(self.objID))
		self.xyz_init[0] = self.prevPoseStamped.x; self.xyz_init[1] = self.prevPoseStamped.y; self.xyz_init[2] = self.prevPoseStamped.z

		self.xyz = fsolve(camPoseEstim.posSolver, self.xyz_init)
		print("\nQuadrotor Estimated Position x = {}, y = {}, z = {}" .format(self.xyz[0], self.xyz[1], self.xyz[2]))
		print("\nActual Position:             x = {}, y = {}, z = {}" .format(self.actualPose.pose.position.x, self.actualPose.pose.position.y, self.actualPose.pose.position.z) )
	
		# Take into consideration the distance between the camera and the centre of the UAV
		self.estimatedPosition.pose.position.x = self.xyz[0]
		self.estimatedPosition.pose.position.y = self.xyz[1]
		self.estimatedPosition.pose.position.z = self.xyz[2] + 0.163		
		self.estimatedPosition.header.stamp = rospy.Time.now()

		self.averageX = self.averageX + self.estimatedPosition.pose.position.x
		self.averageY = self.averageY + self.estimatedPosition.pose.position.y
		self.averageZ = self.averageZ + self.estimatedPosition.pose.position.z
		self.counter = self.counter + 1

		for i in range (len(self.objID)):
			self.validBoundingBoxes.bb_data.pop()

		del self.objID[0:len(self.objID)]
	

	# This function is used to solve the non-linear system of equations in order to calculate the distances between the camera and the landmarks
	def distSolver(self, init_dist):
		distFunctions = []
		i = 0
		for j in range (len(self.objID) - 1):
			for k in range (j+1, len(self.objID)):
				distFunctions.append(init_dist[j]**2 + init_dist[k]**2 - 2*init_dist[j]*init_dist[k]*self.calculatedCosines[i] - self.r[i]**2)
				i = i + 1

		distFunctions = np.array(distFunctions)
		return distFunctions

	def calcLandmarkDistances(self):
		# Calculate the distance between the camera and the landmarks
		self.px = []; self.py = []; self.pz = []
		self.r = [0.0] * int( math.factorial( int( len(self.objID) ) ) / ( 2 * math.factorial( int ( int(len(self.objID)) - 2) ) ) )

		for i in range (len(self.objID)):
			self.px.append( self.landmarkPositionMatrix[self.objID[i] - 1][0] )
			self.py.append( self.landmarkPositionMatrix[self.objID[i] - 1][1] )
			self.pz.append( self.landmarkPositionMatrix[self.objID[i] - 1][2] )

		i = 0
		for j in range (len(self.objID) - 1):
			for k in range (j+1, len(self.objID)):
				self.r[i] = ( math.sqrt((self.px[j] - self.px[k])**2 + (self.py[j] - self.py[k])**2 + (self.pz[j] - self.pz[k])**2) )

				i = i + 1

		self.init_dist = [4.0] * int( math.factorial( int( len(self.objID) ) ) / ( 2 * math.factorial( int ( int(len(self.objID)) - 2) ) ) )
		self.landmarkDist = fsolve(camPoseEstim.distSolver, self.init_dist)

		return self.landmarkDist

	def calcCosines(self):
		#Coordinates in pixels
		x_coord = [0.0] * len(self.objID); y_coord = [0.0]*len(self.objID)
		a_comp = [0.0] * int( math.factorial( int( len(self.objID) ) ) / ( 2 * math.factorial( int ( int(len(self.objID)) - 2) ) ) )
		b_comp = [0.0] * int( math.factorial( int( len(self.objID) ) ) / ( 2 * math.factorial( int ( int(len(self.objID)) - 2) ) ) )
		cosine = [0.0] * int( math.factorial( int( len(self.objID) ) ) / ( 2 * math.factorial( int ( int(len(self.objID)) - 2) ) ) )
		for i in range (len(self.objID)):
			x_coord[i] = float((float(self.validBoundingBoxes.bb_data[i].x_left) + float(self.validBoundingBoxes.bb_data[i].y_top))/2)
			y_coord[i] = float((float(self.validBoundingBoxes.bb_data[i].x_right) + float(self.validBoundingBoxes.bb_data[i].y_bottom))/2)
			
		i = 0
		for j in range (len(self.objID) - 1):
			for k in range (j+1, len(self.objID)):
				a_comp[i] = ((((x_coord[j] - self.cx)/self.fx)**2) + (((y_coord[j] - self.cy)/self.fy)**2) + 1) + ((((x_coord[k] - self.cx)/self.fx)**2) + (((y_coord[k] - self.cy)/self.fy)**2) + 1) - ((((x_coord[j] - x_coord[k])/self.fx)**2) + (((y_coord[j] - y_coord[k])/self.fy)**2))
				b_comp[i] = 2 * math.sqrt((((x_coord[j] - self.cx)/self.fx)**2) + (((y_coord[j] - self.cy)/self.fy)**2) + 1) * math.sqrt((((x_coord[k] - self.cx)/self.fx)**2) + (((y_coord[k] - self.cy)/self.fy)**2) + 1)
				cosine[i] = a_comp[i]/b_comp[i]
				i = i + 1
		return cosine


	def calcCameraPosition(self):
		self.calculatedCosines = camPoseEstim.calcCosines()
		self.calculatedLandmarkDistances = camPoseEstim.calcLandmarkDistances()
		camPoseEstim.estimatePosition()
		print('')

	# --------------------------------------------------------------------------- DISTINGUISH OBJECTS ---------------------------------------------------------------------------
	def distinguishObjects(self):
		for objCount in range (len(self.boundingBoxes.bb_data)):
			if (self.boundingBoxes.bb_data[objCount].object_label == 'person' and self.prevPoseStamped.x < 6.0 and self.boundingBoxes.bb_data[objCount].x_left >= 700.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(1)
				print('person_1')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'person' and self.prevPoseStamped.x < 8.0 and self.boundingBoxes.bb_data[objCount].x_left < 700.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(2)
				print('person_2')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'fire hydrant' and self.prevPoseStamped.x < 6.0 and self.boundingBoxes.bb_data[objCount].x_left >= 550.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(3)
				print('fire_hydrant_1')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'fire hydrant' and self.prevPoseStamped.x < 6.0 and self.boundingBoxes.bb_data[objCount].x_left < 550.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(4)
				print('fire_hydrant_2')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'dog' and self.prevPoseStamped.x < 8.0 and self.boundingBoxes.bb_data[objCount].x_left >= 550.0  ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(5)
				print('dog_1')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'dog' and self.prevPoseStamped.x < 8.0 and self.boundingBoxes.bb_data[objCount].x_left < 550.0  ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(6)
				print('dog_2')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'pottedplant' and self.prevPoseStamped.x < 1.5 and self.boundingBoxes.bb_data[objCount].x_left >= 700.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(7)
				print('pottedplant_1')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'pottedplant' and self.prevPoseStamped.x < 7.0 and self.boundingBoxes.bb_data[objCount].x_left < 700.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(8)
				print('pottedplant_2')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'pottedplant' and self.prevPoseStamped.x >= 10.0 and self.prevPoseStamped.x < 18.0 and self.boundingBoxes.bb_data[objCount].x_left >= 700.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(9)
				print('pottedplant_3')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'chair' and self.prevPoseStamped.x < 10.0):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(10)
				print('chair_1')	
			if (self.boundingBoxes.bb_data[objCount].object_label == 'bicycle' and self.prevPoseStamped.x < 12.0):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(11)
				print('bicycle_1')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'bird' and self.prevPoseStamped.x < 4.0):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(12)
				print('bird_1')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'cat' and self.prevPoseStamped.x < 4.0):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(13)
				print('cat_1')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'clock' and self.prevPoseStamped.x < 12.0):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(14)
				print('clock_1')


			if (self.boundingBoxes.bb_data[objCount].object_label == 'dog' and self.prevPoseStamped.x >= 10.0 and self.prevPoseStamped.x < 17.0 and self.boundingBoxes.bb_data[objCount].x_left < 750.0  ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(15)
				print('dog_3')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'dog' and self.prevPoseStamped.x >= 10.0 and self.prevPoseStamped.x < 17.0 and self.boundingBoxes.bb_data[objCount].x_left >= 750.0  ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(16)
				print('dog_4')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'person' and self.prevPoseStamped.x >= 6.0 and self.prevPoseStamped.x < 17.0 and self.boundingBoxes.bb_data[objCount].x_left >= 700.0  ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(17)
				print('person_3')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'person' and self.prevPoseStamped.x >= 10.0 and self.prevPoseStamped.x < 17.0 and self.boundingBoxes.bb_data[objCount].x_left < 700.0  ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(18)
				print('person_4')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'fire hydrant' and self.prevPoseStamped.x >= 5.0 and self.prevPoseStamped.x < 18.0 and self.boundingBoxes.bb_data[objCount].x_left >= 750.0  ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(19)
				print('fire_hydrant_3')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'pottedplant' and self.prevPoseStamped.x >= 12.0 and self.prevPoseStamped.x < 18.5 and self.boundingBoxes.bb_data[objCount].x_left >= 700.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(20)
				print('pottedplant_4')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'cat' and self.prevPoseStamped.x >= 4.0):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(21)
				print('cat_2')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'bicycle' and self.prevPoseStamped.x >= 12.0):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(22)
				print('bicycle_2')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'clock' and self.prevPoseStamped.x >= 12.0):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(23)
				print('clock_2')

			if (self.boundingBoxes.bb_data[objCount].object_label == 'chair' and self.prevPoseStamped.x >= 10.0):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(24)
				print('chair_2')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'dog' and self.prevPoseStamped.x >= 17.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(25)
				print('dog_5')				
			if (self.boundingBoxes.bb_data[objCount].object_label == 'pottedplant' and self.prevPoseStamped.x >= 12.5 and self.boundingBoxes.bb_data[objCount].x_left < 700.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(26)
				print('pottedplant_5')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'pottedplant' and self.prevPoseStamped.x >= 19.0 and self.boundingBoxes.bb_data[objCount].x_left >= 700.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(27)
				print('pottedplant_6')
			if (self.boundingBoxes.bb_data[objCount].object_label == 'person' and self.prevPoseStamped.x >= 18.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(28)
				print('person_5')				
			if (self.boundingBoxes.bb_data[objCount].object_label == 'fire hydrant' and self.prevPoseStamped.x >= 18.0 ):
				self.validBoundingBoxes.bb_data.append(self.boundingBoxes.bb_data[objCount])
				self.objID.append(29)
				print('fire_hydrant_4')

		print("objID = {}" .format(self.objID))
	# --------------------------------------------------------------------------- CALLBACK FUNCTION ---------------------------------------------------------------------------
	def prevPositionCallback(self, msg):
		self.prevPoseStamped = msg
	
	def boundingBoxesCb(self, msg):
		self.boundingBoxes = msg
		print("----------------------------------------Estimating Position-----------------------------------------\n")
		camPoseEstim.distinguishObjects()
		
		if (len(self.objID) >= 3):
			print("Detected {} objects" .format(len(self.objID)))
			camPoseEstim.calcCameraPosition()
			self.pub.publish(self.estimatedPosition)
			self.rate.sleep()   	
		else:
			print("Not enough objects detected: Only {} objects were detected" .format(len(self.objID)))  

		for i in range (len(self.objID)):
			self.validBoundingBoxes.bb_data.pop()

			del self.objID[0:len(self.objID)]  
	
	def actualPoseCb(self, msg):
		# You can use this for debugging purposes
		self.actualPose = msg

	# --------------------------------------------------------------------------- MAIN FUNCTION ---------------------------------------------------------------------------
	
	def main(self):
		rospy.init_node('camera_pose_estimator', anonymous=True)
		self.rate = rospy.Rate(50)		
		print("camera_pose_estimator node has been successfully initialized")

		rospy.Subscriber('yoloV3_detected_objects', yolov3_bounding_box_vector, self.boundingBoxesCb)
		rospy.Subscriber('/kalman_filter_estimated_state', StateVector, self.prevPositionCallback)
		rospy.Subscriber('/uav1/pose', PoseStamped, self.actualPoseCb)
		self.pub=rospy.Publisher('/camera_estimated_position', PoseStamped, queue_size=1)

		rospy.spin()

		self.averageX = self.averageX/self.counter
		self.averageY = self.averageY/self.counter
		self.averageZ = self.averageZ/self.counter
		
		print('Average X = {}' .format(self.averageX) )
		print('Average Y = {}' .format(self.averageY) )
		print('Average Z = {}' .format(self.averageZ) )		

if __name__ == '__main__':
	camPoseEstim = CameraPoseEstimator()
	camPoseEstim.main()
