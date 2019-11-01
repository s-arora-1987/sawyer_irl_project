# sawyer_irl_project
Author: Prasanth Sengadu Suresh.

Owned By: THINC Lab, Department of Computer Science,
          University of Georgia.

This package uses Inverse Reinforcement Learning -- Apprenticeship Learning/Learning from Demonstration to teach Sawyer Robot to perform vegetable sorting on a conveyor line alongside a Human Expert.

This package is built upon the Sawyer/Intera ROS packages and uses Robotiq 2F-85 Gripper as the End Effector.

The following are the steps to be followed to get this package working:

  Assuming you have a working version of Ubuntu (This package has been built and tested on Ubuntu 16.04)
  
  1.) Install ROS (This package was built on ROS Kinectic, for other versions you might need to make appropriate changes)
  
   [ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu)
      
   [Catkin Workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)
      
   [Moveit Install](https://moveit.ros.org/install/)
   
   [Moveit Workspace Setup](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
      
  2.) Now that you have a catkin workspace setup, in you src folder, git clone the following packages:
  
   - These packages have changes that are not a part of their default branches. Make sure you clone them from the links below.
          
          git clone --branch release-5.2.0 https://github.com/RethinkRobotics/intera_sdk.git
      
          git clone --branch release-5.2.0 https://github.com/RethinkRobotics/intera_common.git
          
   - cd into catkin_ws and do a catkin_make at this point. This will generate the intera custom messages that the following packages use.
  
          git clone --branch release-5.2.0 https://github.com/thinclab/sawyer_moveit.git
      
          git clone --branch release-5.2.0 https://github.com/thinclab/sawyer_robot.git
      
          git clone --branch release-5.2.0 https://github.com/thinclab/sawyer_simulator.git
      
          git clone --branch kinetic-devel https://github.com/thinclab/robotiq.git
      
          git clone https://github.com/prasuchit/roboticsgroup_gazebo_plugins-1
      
          git clone https://github.com/prasuchit/gazebo_ros_link_attacher
      
          git clone https://github.com/prasuchit/kinect_v2_udrf
          
          git clone https://github.com/prasuchit/sawyer_irl_project.git
          
          git clone https://github.com/prasuchit/velocity_plugin.git
   
   - Use the following command to update all your packages and drivers:
   
          sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
      
   - cd into catkin_ws and install all dependencies for these packages: 
   
          (For kinetic: rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y -i --verbose)
          
     - If you see any uninstalled dependencies, you might have to manually install them using apt-get install or pip install.
     - If you still have errors, use 
     
           rospack depends [name of package]
           
     - This should give you the dependencies you are missing for any given package.
     - Do a catkin_make to compile. If you face a soem error, this is a useful link to resolve some related errors: [Soem error](https://github.com/tork-a/minas/issues/64)
     
  3.) You are almost ready to run the simulation. Double check if you have installed all the required plugins for moveit (esp moveit controllers)
  
   - **You have to modify one path that is hardcoded for my local directory to yours (PS: I'm working on fixing this!)**
      
     - In  sawyer_irl_project/worlds/sawyer_lab.world, check under model name="sawyer_lab", modify the mesh location to your local filesystem.
   
  4.) Run the following commands in seperate terminals:
  
      roslaunch sawyer_irl_project robot_gazebo.launch
  
      roslaunch kinect_v2 kinectv2_gazebo.launch
      
      rosrun robotiq_2f_gripper_control simple_pnp_gazebo.py    (Make sure all files in this folder are set to executable in file properties)
      
  5.) Now, if you want to run the same on the real Sawyer Robot,
  
   [Robot Setup](http://sdk.rethinkrobotics.com/intera/Robot_Setup)
        
   [Workstation Setup](http://sdk.rethinkrobotics.com/intera/Workstation_Setup)
        
   - Make sure you are able to connect to the gripper using the following steps: (DO ALL THESE STEPS WITHIN THE ./intera.sh ENV, ALL IN NEW TERMINALS)
        
         sudo chmod 777 /dev/ttyUSB0
          
         roslaunch robotiq_2f_gripper_action_server robotiq_2f_gripper_action_server_rtu.launch  (DRIVER FILE: Keep this running until the gripper is being used)
          
         rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py     
          
   - For Sawyer, do the following steps (In new tabs):
        
         roslaunch sawyer_irl_project robot.launch
          
         roslaunch sawyer_irl_project upload_gripper.launch      
          
         roslaunch moveit_configs sawyer_moveit.launch
          
   - For pick and place:
        
         rosrun robotiq simple_pnp.py    (Make sure all files in this folder are set to executable in file properties)

