#!/usr/bin/env python

from PickandPlace import PickAndPlace

import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int8MultiArray
from operator import mod
import random
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import sys
from os import system
import rospy
from time import sleep
from sawyer_irl_project.msg import OBlobs
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import numpy as np
import _thread
# sys.path.append('/home/psuresh/catkin_ws/src/sanet_onionsorting/scripts')
# from rgbd_imgpoint_to_tf import Camera
# Global initializations
pnp = PickAndPlace()
done_onions = []
flag = False
nOnionLoc = 4
nEEFLoc = 4
nPredict = 3
nlistIDStatus = 3
nS = nOnionLoc*nEEFLoc*nPredict*nlistIDStatus
nA = 7
policy = np.genfromtxt('/home/psuresh/catkin_ws/src/sawyer_irl_project/scripts/expert_policy_pick.csv', delimiter=' ')

def sid2vals(s, nOnionLoc=4, nEEFLoc=4, nPredict=3, nlistIDStatus=3):
    sid = s
    onionloc = int(mod(sid, nOnionLoc))
    sid = (sid - onionloc)/nOnionLoc
    eefloc = int(mod(sid, nEEFLoc))
    sid = (sid - eefloc)/nEEFLoc
    predic = int(mod(sid, nPredict))
    sid = (sid - predic)/nPredict
    listidstatus = int(mod(sid, nlistIDStatus))
    return [onionloc, eefloc, predic, listidstatus]


def vals2sid(ol, eefl, pred, listst, nOnionLoc=4, nEEFLoc=4, nPredict=3, nlistIDStatus=3):
    return(ol + nOnionLoc * (eefl + nEEFLoc * (pred + nPredict * listst)))


def getState(onionName, predic):
    global pnp
    print("Getting state")
    current_pose = pnp.group.get_current_pose().pose
    if current_pose.position.x > 0.5 and current_pose.position.x < 0.9 and current_pose.position.y > -0.5 and current_pose.position.y < 0.5 and current_pose.position.z > 0 and current_pose.position.z < 0.2:
        pnp.eefLoc = 0  # On Conveyor
    elif current_pose.position.x > 0.4 and current_pose.position.x < 0.5 and current_pose.position.y > -0.1 and current_pose.position.y < 0.15 and current_pose.position.z > 0.44 and current_pose.position.z < 0.6:
        pnp.eefLoc = 1  # In Front
    elif current_pose.position.x > 0 and current_pose.position.x < 0.15 and current_pose.position.y > 0.5 and current_pose.position.y < 0.8:
        pnp.eefLoc = 2  # In Bin
    elif current_pose.position.x > 0.45 and current_pose.position.x < 0.9 and current_pose.position.y > -0.5 and current_pose.position.y < 0.5 and current_pose.position.z > 0.15 and current_pose.position.z < 0.5:
        pnp.eefLoc = 3  # In the Hover plane
    else:
        print("Couldn't find valid eef state!")
        print("EEF coordinates: ", current_pose)
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        model_coordinates = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        print("Onion name is: ", onionName)
        onion_coordinates = model_coordinates(onionName, "").pose
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    if onion_coordinates.position.x > 0.5 and onion_coordinates.position.x < 0.9 and onion_coordinates.position.y > -0.6 and onion_coordinates.position.y < 0.3 and onion_coordinates.position.z > 0.8 and onion_coordinates.position.z < 0.9:
        pnp.onionLoc =  0    # On Conveyor
        print("OnionLoc is: On conveyor {0}".format(pnp.onionLoc))
    elif onion_coordinates.position.x > 0.15 and onion_coordinates.position.x < 0.3 and onion_coordinates.position.z > 1 and onion_coordinates.position.z < 2:
        pnp.onionLoc =  1    # In Front
        print("OnionLoc is: In Front {0}".format(pnp.onionLoc))
    elif onion_coordinates.position.x > 0 and onion_coordinates.position.x < 0.15 and onion_coordinates.position.y > 0.5 and onion_coordinates.position.y < 0.8:
        pnp.onionLoc =  2    # In Bin
        print("OnionLoc is: In Bin {0}".format(pnp.onionLoc))
    elif onion_coordinates.position.x > 0.35 and onion_coordinates.position.x < 0.9 and onion_coordinates.position.z > 0.9 and onion_coordinates.position.z < 1.5:
        pnp.onionLoc =  3    # In Hover Plane
        print("OnionLoc is: In Hover plane {0}".format(pnp.onionLoc))
    elif onion_coordinates.position.x > 0.5 and onion_coordinates.position.x < 0.9 and onion_coordinates.position.y > 0.3 and onion_coordinates.position.y < 0.6 and onion_coordinates.position.z > 0.8 and onion_coordinates.position.z < 0.9:
        pnp.onionLoc =  0    # Placed on Conveyor   # we removed the placed on conv location
        print("OnionLoc is: Placed on conveyor {0}".format(pnp.onionLoc))
    else:
        print("Couldn't find valid onion state!")
        print("Onion coordinates: ", onion_coordinates)

    pnp.prediction = predic
    print("EEfloc is: ", pnp.eefLoc)
    print("prediction is: ", predic)
    if len(pnp.bad_onions) > 0:
        pnp.listIDstatus = 1
    elif len(pnp.bad_onions) == 0:
        pnp.listIDstatus = 0
    else:
        pnp.listIDstatus = 2
    print("List status is: ", pnp.listIDstatus)

    return vals2sid(ol=pnp.onionLoc, eefl=pnp.eefLoc, pred=pnp.prediction, listst=pnp.listIDstatus)



def callback_vision_poses(msg):
    global pnp, policy
    if len(msg.color) == 0:
        print("No onions found by vision!")
        return
    total_onions = 4
    max_index = len(msg.color)
    # attach and detach service
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()
    pnp.req.model_name_1 = None
    pnp.req.link_name_1 = "base_link"
    pnp.req.model_name_2 = "sawyer"
    pnp.req.link_name_2 = "right_l6"
    pnp.target_location_x = msg.x[pnp.onion_index]
    pnp.target_location_y = msg.y[pnp.onion_index]
    pnp.target_location_z = msg.z[pnp.onion_index]
    color = int(msg.color[pnp.onion_index])
    # pnp.bad_onions = [i for i in range(pnp.onion_index, max_index) if (msg.color[i] == 0)]
    # print("Bad onion indices are: ", pnp.bad_onions)
    
    for i in range(total_onions):
        if len(done_onions) == len(msg.color):
            return
        if i in done_onions:
            pass
        else:
            onion_guess = "onion_" + str(i)

            rospy.wait_for_service('/gazebo/get_model_state')
            try:
                model_coordinates = rospy.ServiceProxy(
                    '/gazebo/get_model_state', GetModelState)
                print 'Onion name guessed: ', onion_guess
                onion_coordinates = model_coordinates(onion_guess, "").pose
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

            if pnp.target_location_y - 0.05 <= onion_coordinates.position.y <= pnp.target_location_y + 0.05:
                if pnp.target_location_x - 0.05 <= onion_coordinates.position.x <= pnp.target_location_x + 0.05:
                    pnp.req.model_name_1 = onion_guess
                    done_onions.append(i)
                    break

    if pnp.req.model_name_1 == None:
        return
    else:
        print "Onion name set as: ", pnp.req.model_name_1
    pnp.goto_home(tolerance=0.1, goal_tol=0.1, orientation_tol=0.1)
    # s = getState(pnp.req.model_name_1, 2)   # Sending unknown prediction until viewed
    # a = 3   # Pick
    # policy[s] = a
    status = pnp.goAndPick()
    rospy.sleep(0.01)
    pnp.scene.remove_world_object(onion_guess)
    pnp.staticDip()
    # if(status):
    attach_srv.call(pnp.req)
    rospy.sleep(0.1)
    pnp.liftgripper()
    # s = getState(pnp.req.model_name_1, 2)   # Sending unknown prediction until viewed
    # a = 0   # Inspect after picking
    # policy[s] = a
    rospy.sleep(0.01)
    pnp.view(0.3)
    rospy.sleep(0.01)
    pnp.rotategripper(0.3)
    rospy.sleep(0.01)
    if color:
        # s = getState(pnp.req.model_name_1, color)
        # a = 1   # Place on conveyor
        # policy[s] = a
        pnp.placeOnConveyor()
    else:
        # s = getState(pnp.req.model_name_1, color)
        # a = 2   # Place in bin
        # policy[s] = a
        pnp.goto_bin()
    rospy.sleep(0.01)
    detach_srv.call(pnp.req)
    # s = getState(pnp.req.model_name_1, 2)   # Sending unknown prediction until viewed again
    # a = 4   # Claim new onion
    # policy[s] = a
    if pnp.onion_index < len(msg.x) - 1:
        pnp.onion_index = pnp.onion_index + 1
    else:
        print("No more onions to sort!!")
        # rospy.signal_shutdown("Shutting down node, work is done")
        pass
    # np.savetxt('modified_policy'+str(pnp.onion_index)+'.csv',policy, delimiter = ',')
    return

##################################### Now to Main ##################################################

def main():
    print("I'm in main!")
    try:
        if len(sys.argv) < 2:
            sortmethod = "pick"   # Default sort method
        else:
            sortmethod = sys.argv[1]

        # if(sortmethod == "pick"):
        #     print("Pick method selected")
        #     rospy.Subscriber("current_onions_blocks",
        #                      Int8MultiArray, callback_onion_pick)
        # elif(sortmethod == "roll"):
        #     print("Roll method selected")
        #     rospy.Subscriber("current_onions_blocks",
        #                      Int8MultiArray, callback_onion_roll)
        while not rospy.is_shutdown():
            # rospy.Subscriber("onions_blocks_poses", onions_blocks_poses, callback_poses)
            # _thread.start_new_thread( system, ('rosrun sanet_onionsorting yolo_service.py gazebo',))
            # _thread.start_new_thread( system, ('rosrun sanet_onionsorting rgbd_imgpoint_to_tf.py',))
            rospy.Subscriber("/object_location", OBlobs, callback_vision_poses)
        #########################################################################################
        # callback_onion_pick(rospy.wait_for_message("current_onions_blocks", Int8MultiArray))  #
        # If you're using this method, it will only listen once until it hears something,      #
        # this may cause trouble later, watch out!                                            #
        #######################################################################################

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    rospy.spin()


if __name__ == '__main__':
    print("Calling Main!")
    try:
        main()
    except rospy.ROSInterruptException:
        print "Main function not found! WTH dude!"