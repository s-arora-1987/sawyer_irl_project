## This file talks about the additional changes to be made that aren't mentioned in the Kinetic version readme

Assuming you have ROS Melodic properly installed and setup fully. The following are the additional steps you need to do in Melodic:
  - You will also need this package:
  
    `git clone https://github.com/RethinkRobotics-opensource/sns_ik.git`
    
  - If you've already cloned the Robotiq and gazebo-ros-link-attacher packages from the other readme link, remove them.
  - Use the following links to clone them into your src:
    ```
    git clone --branch melodic-devel https://github.com/thinclab/robotiq.git
    
    git clone --branch melodic-devel https://github.com/prasuchit/gazebo_ros_link_attacher
    ```
  - Some DAE files might not show up on Gazebo 9+ that comes with melodic (although they show up on rviz). I found a crude fix that works, the following files weren't showing up on gazebo:
      - head.DAE, l5.DAE, robotiq_arg2f_85_base_link.dae
      - I opened the DAE files with a text editor, replaced all occurences of: `<transparent opaque="A_ONE">` with `<transparent opaque="RGB-Zero">`
      - The concept is this tag value should be to RGB-Zero for gazebo 9 to be able to display the visual model.
  - Next, you need to install:
  
    `sudo apt install ros-melodic-joint-state-publisher-gui`
    
    
 ### Will keep adding to this as and when I find any discrepancies
