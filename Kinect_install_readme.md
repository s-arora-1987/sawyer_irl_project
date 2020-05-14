# This file provides the instructions to install libfreenect2 and iai-kinect2 in order to use Kinect v2 with Ubuntu and ROS.

## libfreenect2 Install

1. Follow the instructions provided at [libfreenect2](https://github.com/OpenKinect/libfreenect2#linux) to install it on Ubuntu/linux OSes.
2. The additional steps to follow while following those instructions are:
    - In the cmake step, while building the libfreenect2 package, use this command instead: **`cmake .. -DENABLE_CXX11=ON -DCUDA_PROPAGATE_HOST_FLAGS=off -DCMAKE_INSTALL_PREFIX=$HOME/freenect2`**
    - Also install this: `sudo apt-get install libturbojpeg0-dev`
    - If your GPU is Nvidia, don't install any optional dependancies mentioned there.
    - Instead, make sure to **open Software and Updates-> Additional Drivers and switch to the most recent version of Nvidia driver available and restart.**
    
 ## iai-kinect2 Install
 
 1. Follow the instructions provided at [iai-kinect2](https://github.com/code-iai/iai_kinect2) to get it working with ROS.
 2. Except, in the catkin_make step, use this command instead: **`catkin_make -DCMAKE_BUILD_TYPE="Release" -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2`**
 3. Everything should work as intended after compilation succeeds.
 
 # More information will be added here if and when additional problems are encountered. 
