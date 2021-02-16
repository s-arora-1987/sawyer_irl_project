#!/usr/bin/env python 
import rospy 
import sys 

sys.path.append('/home/psuresh/catkin_ws/src/sorting_patrol_MDP_irl')
from sortingMDP.model import sortingModelbyPSuresh4multipleInit_onlyPIP, \
    sortingState, ClaimNewOnion

if __name__ == '__main__':
    rospy.init_node('test_node')
    s_mdp = sortingModelbyPSuresh4multipleInit_onlyPIP()
    s = sortingState(2,2,2,2)
    a = ClaimNewOnion()
    ns = s_mdp.T(s,a).keys()[0]
    print(ns)
    