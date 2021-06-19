#!/usr/bin/env python

import rospy
import std_msgs.msg
import math
import os.path
import time
import re
import numpy as np
import copy
from collections import namedtuple

from ORTILo.msg import yolov3_bounding_box_vector
from ORTILo.msg import yolov3_bounding_box


def talker():

    pub = rospy.Publisher('yoloV3_detected_objects', yolov3_bounding_box_vector, queue_size=1)
    rospy.init_node('yolo_objects_parser', anonymous=True)
    rate = rospy.Rate(100)
    
    h = std_msgs.msg.Header()

    file_counter_0 = 0
    file_counter_1 = 0
    file_counter_2 = 0

    while not rospy.is_shutdown():
        file_counter_string_0 = str(file_counter_0).zfill(6)
        file_counter_string_1 = str(file_counter_1).zfill(3)
        file_counter_string_2 = str(file_counter_2).zfill(2)
        file_path = "../DS_YoloV3_Detected_Objects/"+file_counter_string_2+"_"+file_counter_string_1+"_"+file_counter_string_0+".txt"
        
        while not os.path.exists(file_path):
            time.sleep(1/100)

        if os.path.isfile(file_path):
            # Open file and read lines (one line for every detected object)
            f = open(file_path, "r")
            lines = f.readlines()
            
            bounding_boxes = yolov3_bounding_box_vector()
            temp = yolov3_bounding_box_vector()
            
            print("%d object(s) were detected" %len(lines))

            # Split each line in order to be able to use the metadata provided by YOLO
            if(len(lines) > 0 ):
                i = 0
                j = 0
                for line in range(len(lines)):
                    temp.bb_data.append(copy.copy(line_parser(lines[line])))
                    if(not (temp.bb_data[line].x_left == 0.0 and temp.bb_data[line].x_right == 0)):
                        i += 1
                        bounding_boxes.bb_data.append(copy.copy(temp.bb_data[line]))
                    else:
                        j += 1
                
                for line in range(len(lines)):
                    if(len(lines) - j == 0):
                        break

            print(file_counter_string_2+"_"+file_counter_string_1+"_"+file_counter_string_0)

            if file_counter_0 == 999999 and file_counter_1 == 999:
                file_counter_0 = 0
                file_counter_1 += 1
            elif file_counter_0 == 999999 and file_counter_1 == 999:
                file_counter_0 = 0
                file_counter_1 = 0
                file_counter_2 += 1
            else:
                file_counter_0 += 1 
        else:
            raise ValueError("%s isn't a file!" % file_path)

        h.stamp = rospy.Time.now()
        bounding_boxes.header = h
        pub.publish(bounding_boxes)
        rate.sleep()


def line_parser(line):
    bounding_boxes_temp = yolov3_bounding_box()

    components = line.split(' 0.0', 10)
            
    # Object label
    bounding_boxes_temp.object_label = components[0]

    # Pixel coordinates
    pixel_coordinates_components = components[2].split(' ', 5)

    if(len(pixel_coordinates_components) < 5):
        print('discarded!')
    else:
        bounding_boxes_temp.x_left = (float(pixel_coordinates_components[1]))
        bounding_boxes_temp.x_right = (float(pixel_coordinates_components[2]))
        bounding_boxes_temp.y_top = (float(pixel_coordinates_components[3]))
        bounding_boxes_temp.y_bottom = (float(pixel_coordinates_components[4]))

        # Confidence
        bounding_boxes_temp.confidence = float(re.sub("[^\d\.]", "",components[9]))

    return bounding_boxes_temp


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass