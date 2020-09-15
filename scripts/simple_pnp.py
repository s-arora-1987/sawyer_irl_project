#!/usr/bin/env python

from PickandPlace import PickAndPlace

import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Int8MultiArray
from operator import mod
import random
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import sys
import rospy
from sawyer_irl_project.msg import onions_blocks_poses
from gripper_to_position import reset_gripper, activate_gripper, gripper_to_pos
import numpy as np

pnp = PickAndPlace()

def main():
  try:

    # pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
  
    reset_gripper()

    activate_gripper()

  #   # 255 = closed, 0 = open
    gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER

    rospy.sleep(1.0)
    gripper_to_pos(255, 60, 200, False)    # GRIPPER TO POSITION 50

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
	try:
  		main()
	except rospy.ROSInterruptException:
	    pass
