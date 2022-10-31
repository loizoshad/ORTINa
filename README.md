# ORTILo: Onboard Real-Time Multi-Sensor Pose Estimation for Indoor Quadrotor Navigation with Intermittent Communication

This repository contains the source-code developed and used to generate all the results in our research work ORTILo

## Description

We propose a multisensor fusion framework for onboard real-time navigation of a quadrotor in an indoor environment, by integrating sensor readings from an Inertial Measurement Unit (IMU), a camera-based object detection algorithm, and an Ultra-WideBand (UWB) localization system. The sensor readings from the camera-based object detection algorithm and the UWB localization system arrive intermittently, since the measurements are not readily available. We design a Kalman filter that manages intermittent observations in order to handle and fuse the readings and estimate the pose of the quadrotor for tracking a predefined trajectory. The system is implemented via a Hardware-in-the-loop (HIL) simulation technique, in which the dynamic model of the quadrotor is simulated in an open-source 3D robotics simulator tool, and the whole navigation system is implemented on Artificial Intelligence (AI) enabled edge GPU. The simulation results show that our proposed framework offers low positioning and trajectory errors, while handling intermittent sensor measurements.

## Getting Started

### Dependencies

* Ubuntu 18.04 LTE
* ROS Melodic
* Deepstream 5.0 SDK (Any implementatioon of the YOLOv3 algorithm can be used, as long as it complies with the message format used in this work.)
* UWB package. (The pozyx simulation package https://github.com/bekirbostanci/pozyx_simulation.git was used in this work. Any UWB package can be used, as long as it complies with the message format used in this work.)

### Setup Simulation Environment

* Add to $GAZEBO_MODEL_PATH the path to the extra models: "ORTILo/gazebo_extra_models"

### Demo

* roslaunch simulation.launch
* roslaunch uwb_anchors_setup.launch 
* rosrun ORTILo uwb_position_estimator.py
* Launch the YOLO algorithm
* rosrun ORTILo yolo_objects_parser.py
* rosrun ORTILo yolo_position_estimator.py
* Use the "/lqi/command/single_pose" and "move_base_simple/goal" topics to command the quadrotor.

## Authors

* Loizos Hadjiloizou
* Kyriakos M. Deliparaschos
* Evagoras Makridis
* Themistoklis Charalambous

## License

This project is licensed under the GPLv3 License - see the LICENSE.md file for details
