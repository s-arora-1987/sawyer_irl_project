# sawyer_irl_project
Author: Prasanth Sengadu Suresh.

Owned By: THINC Lab, Department of Computer Science,
          University of Georgia.

This package uses Inverse Reinforcement Learning -- Apprenticeship Learning/Learning from Demonstration to teach Sawyer Robot to perform vegetable sorting on a conveyor line alongside a Human Expert.

This package is built upon the Sawyer/Intera ROS packages and uses Robotiq 2F-85 Gripper as the End Effector.

The following are the steps to be followed to get this package working:

  Assuming you have a working version of Ubuntu (This package has been built and tested on Ubuntu 16.04)
  
  1.) Install ROS (This package was built on ROS Kinectic, for other versions you might need to make appropriate changes)
  
      https://wiki.ros.org/kinetic/Installation/Ubuntu   
      
      https://wiki.ros.org/catkin/Tutorials/create_a_workspace
      
      https://moveit.ros.org/install/
      
  2.) Now that you have a catkin workspace setup, in you src folder, git clone the following packages:
  
      https://github.com/thinclab/sawyer_moveit/tree/release-5.2.0
      
      https://github.com/thinclab/sawyer_robot/tree/release-5.2.0
      
      https://github.com/thinclab/sawyer_simulator/tree/release-5.2.0
      
      https://github.com/thinclab/robotiq
      
      https://github.com/RethinkRobotics/intera_sdk/tree/release-5.2.0
      
      https://github.com/RethinkRobotics/intera_common/tree/release-5.2.0
      
      https://github.com/prasuchit/roboticsgroup_gazebo_plugins-1
      
      https://github.com/pal-robotics/gazebo_ros_link_attacher
      
      *ADD THE KINECT PACKAGE HERE*
      
      Install all dependencies for these packages: (For kinetic: rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y)
      Now use catkin_make to compile
      
  3.) You are almost ready to run the simulation. Double check if you have installed all the required plugins for moveit (esp moveit controllers)
  
      You have to modify one path that is hardcoded for my local directory to yours (PS:I'm working on fixing this!)
      
      In  sawyer_irl_project/worlds/sawyer_lab.world, check under model name="sawyer_lab", modify the mesh location to your local filesystem.
      
  4.) Run roslaunch sawyer_irl_project robot_gazebo.launch
  
      Run roslaunch kinectv2 kinect_gazebo.launch
      
      Run rosrun robotiq simple_pnp_gazebo.py    (Make sure all files in this folder are set to executable in file properties)
      
  5.) Now, if you want to run the same on the real Sawyer Robot,
  
        http://sdk.rethinkrobotics.com/intera/Robot_Setup
        
        http://sdk.rethinkrobotics.com/intera/Workstation_Setup
        
        Make sure you are able to connect to the gripper using the following steps: (DO ALL THESE STEPS WITHIN THE ./intera.sh ENV IN ALL TERMINALS)
        
          sudo chmod 777 /dev/ttyUSB0
          
          roslaunch robotiq_2f_gripper_action_server robotiq_2f_gripper_action_server_rtu.launch    (DRIVER FILE: Keep this running until the gripper is being used)
          
          rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py     (In a new tab)
          
        For Sawyer, do the following steps:
        
          roslaunch sawyer_irl_project robot.launch
          
          roslaunch sawyer_irl_project upload_gripper.launch      (In a new tab)
          
          roslaunch moveit_configs sawyer_moveit.launch
          
        For pick and place:
        
            rosrun robotiq simple_pnp.py    (Make sure all files in this folder are set to executable in file properties)
            
          
          
      
      
