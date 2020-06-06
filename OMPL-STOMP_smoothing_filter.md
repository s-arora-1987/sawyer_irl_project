## This readme details the steps to add STOMP as a smoothing filter over OMPL.
### Initial test show planning slows down with this - More testing is underway.
#### This was installed and tested with Ubuntu 18.04, ROS Melodic, and Sawyer robot.

- For some reason catkin_make doesn't work with stomp-ros package, while catkin build does (to be reverified). So first delete the build and devel folders if you have them within catkin_ws
- Clone stomp-ros into catkin_ws/src:
  `git clone https://github.com/ros-industrial/stomp_ros.git`
- Move to catkin_ws and build:
  `cd ~/catkin_ws;`
  `catkin build;`
  `catkin run_tests;`
  Refer this [link](https://github.com/ros-industrial/stomp_ros#stomp_ros) if you have questions.
- If everything succeeds, you can start adding stomp to the path planning config:
  `roscd sawyer_moveit_config;`
  `cd config;`
  `touch stomp_planning.yaml;`
  `gedit stomp_planning.yaml;`
  Now copy everything from this [file](https://github.com/ros-planning/panda_moveit_config/blob/kinetic-devel/config/stomp_planning.yaml) and paste it into stomp_planning.yaml

- If you are using Sawyer robot, change the first two lines from:
  `stomp/panda_arm:
    group_name: panda_arm`  to
  `stomp/right_arm:
    group_name: right_arm`
    
 - From here, follow these [instructions](https://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/planning_adapters/planning_adapters_tutorial.html#running-ompl-as-a-pre-processor-for-stomp) with the following change:
    `default_planner_request_adapters/STOMPSmoothingAdapter` in step 1 needs to be `stomp_moveit/StompSmoothingAdapter` instead.
