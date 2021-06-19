# README #

This README would normally document whatever steps are necessary to get your application up and running.

### Simulation Environment ###

* Add to $GAZEBO_MODEL_PATH the path to the extra models: "ORTILo/gazebo_extra_models"

### Demo ###

* roslaunch simulation.launch
* roslaunch uwb_anchors_setup.launch (This step is optional. If it is used, the pozyx_simulation is required https://github.com/bekirbostanci/pozyx_simulation.git, otherwise any UWB package can be used, as long as it complies with the message format used in this work.
* rosrun ORTILo uwb_position_estimator.py
* Launch YOLO (In this work the Deepstream 5.0 SDK was used)
* rosrun ORTILo yolo_objects_parser.py
* rosrun ORTILo yolo_position_estimator.py
* Use the "/lqi/command/single_pose" and "move_base_simple/goal" topics to command the quadrotor.