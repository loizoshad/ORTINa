# ORTILo: Onboard Real-Time Multi-Sensor Pose Estimation for Indoor Quadrotor Navigation with Intermittent Communication

This repository contains the source-code developed and used to generate all the results in the research paper ORTILo

If you use our code or data in your work, please cite our research paper:

    @inproceedings{
        hadjiloizou2022,
        title = {Onboard Real-Time Multi-Sensor Pose Estimation for Indoor Quadrotor Navigation with Intermittent Communication},
        author = {Loizos Hadjiloizou and Kyriakos Deliparaschos and Evagoras Makridis and Themistoklis Charalambous},
        booktitle = {IEEE Globecom 2022},
        year = {2022}
    }

> **Abstract**: We propose a multisensor fusion framework for onboard real-time navigation of a quadrotor in an indoor environment, by integrating sensor readings from an Inertial Measurement Unit (IMU), a camera-based object detection algorithm, and an Ultra-WideBand (UWB) localization system. The sensor readings from the camera-based object detection algorithm and the UWB localization system arrive intermittently, since the measurements are not readily available. We design a Kalman filter that manages intermittent observations in order to handle and fuse the readings and estimate the pose of the quadrotor for tracking a predefined trajectory. The system is implemented via a Hardware-in-the-loop (HIL) simulation  technique, in which the dynamic model of the quadrotor is simulated in an open-source 3D robotics simulator tool, and the whole navigation system is implemented on Artificial Intelligence (AI) enabled edge GPU. The simulation results show that our proposed framework offers low positioning and trajectory errors, while handling intermittent sensor measurements.


## Setup

### Simulation Environment

* Add to $GAZEBO_MODEL_PATH the path to the extra models: "ORTILo/gazebo_extra_models"

### Dependencies


* Ubuntu 18.04 LTE
* ROS Melodic
* Deepstream 5.0 SDK (Any implementatioon of the YOLOv3 algorithm can be used, as long as it complies with the message format used in this work.)
* UWB package. (The pozyx simulation package https://github.com/bekirbostanci/pozyx_simulation.git was used in this work. Any UWB package can be used, as long as it complies with the message format used in this work.)


## Demo

* roslaunch simulation.launch
* roslaunch uwb_anchors_setup.launch 
* rosrun ORTILo uwb_position_estimator.py
* Launch the YOLO algorithm
* rosrun ORTILo yolo_objects_parser.py
* rosrun ORTILo yolo_position_estimator.py
* Use the "/lqi/command/single_pose" and "move_base_simple/goal" topics to command the quadrotor.

A demonstration of one full test is provided here

https://user-images.githubusercontent.com/92587570/199697729-e40b847e-3784-44d3-af1a-7c63216fc134.mp4

#

This project is released under a GPLv3 license
