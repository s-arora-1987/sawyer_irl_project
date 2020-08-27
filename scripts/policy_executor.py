from simple_pnp_gazebo import PickAndPlace
import intera_interface
from robotiq_2f_gripper_control.srv import move_robot, move_robotResponse
import numpy as np
from sawyer_irl_project.msg import onions_blocks_poses
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from operator import mod
import random
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import csv

''' Picked/AtHome - means Sawyer is in hover plane at home position
    Placed - means placed back on conveyor after inspecting and finding status as good
    onionLoc = {0: 'OnConveyor', 1: 'InFront',
        2: 'InBin', 3: 'Picked/AtHome', 4: 'Placed'}
    eefLoc = {0: 'OnConveyor', 1: 'InFront', 2: 'InBin', 3: 'Picked/AtHome'}
    predictions = {0: 'Bad', 1: 'Good', 2: 'Unknown'}
    listIDstatus = {0: 'Empty', 1: 'Not Empty', 2: 'Unavailable'}
    actList = {0: 'InspectAfterPicking', 1: 'PlaceOnConveyor', 2: 'PlaceInBin', 3: 'Pick',
        4: 'ClaimNewOnion', 5: 'InspectWithoutPicking', 6: 'ClaimNextInList'} '''


# Global initializations
pnp = PickAndPlace(limb, tip_name)

req = AttachRequest()
initialized = False
num_onions = 0
flag = False
joint_state_topic = ['joint_states:=/robot/joint_states']
limb = 'right'
tip_name = "right_gripper_tip"
target_location_x = -100
target_location_y = -100
onion_index = 0
bad_onion_index = 0
# at HOME position, orientation of gripper frame w.r.t world x=0.7, y=0.7, z=0.0, w=0.0 or [ rollx: -3.1415927, pitchy: 0, yawz: -1.5707963 ]
# (roll about an X-axis w.r.t home) / (subsequent pitch about the Y-axis) / (subsequent yaw about the Z-axis)
rollx = 3.30
pitchy = 0.0
yawz = -1.57
# use q with moveit because giving quaternion.x doesn't work.
q = quaternion_from_euler(rollx, pitchy, yawz)
overhead_orientation_moveit = Quaternion(
    x=q[0],
    y=q[1],
    z=q[2],
    w=q[3])
bad_onions = []


def sid2vals(s, nOnionLoc=5, nEEFLoc=4, nPredict=3, nlistIDStatus=3):
    sid = s
    onionloc = int(mod(sid, nOnionLoc))
    sid = (sid - onionloc)/nOnionLoc
    eefloc = int(mod(sid, nEEFLoc))
    sid = (sid - eefloc)/nEEFLoc
    predic = int(mod(sid, nPredict))
    sid = (sid - predic)/nPredict
    listidstatus = int(mod(sid, nlistIDStatus))
    return [onionloc, eefloc, predic, listidstatus]


def vals2sid(ol, eefl, pred, listst, nOnionLoc=5, nEEFLoc=4, nPredict=3, nlistIDStatus=3):
    return(ol + nOnionLoc * (eefl + nEEFLoc * (pred + nPredict * listst)))


def getState(onionName, predic):
    current_pose = pnp.group.get_current_pose().pose
    if current_pose.position.x > 0.4 and current_pose.position.z > 0.15:
        eefLoc = 3  # In the Hover plane
    elif current_pose.position.x > 0 and current_pose.position.x < 0.15/
            current_pose.position.y > 0.5 and current_pose.position.y < 0.8:
        eefLoc = 2  # In Bin
    elif current_pose.position.x > 0.4 and current_pose.position.x < 0.6/
            current_pose.position.z > 0.4 and current_pose.position.z < 0.6:
        eefLoc = 1  # In Front
    elif current_pose.position.x > 0.5 and current_pose.position.x < 0.9/
            current_pose.position.z > 0 and current_pose.position.z < 0.2:
        eefLoc = 0  # On Conveyor

    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        model_coordinates = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        onion_coordinates = model_coordinates(onionName, "").pose
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    if onion_coordinates.position.x > 0.5 and current_pose.position.x < 0.9/
            current_pose.position.z > 0 and current_pose.position.z < 0.2:
        onionLoc = 0    # On Conveyor
    elif onion_coordinates.position.x > 0.15 and current_pose.position.x < 0.3/
            current_pose.position.z > 0.8 and current_pose.position.z < 2:
        onionLoc = 1    # In Front
    elif onion_coordinates.position.x > 0 and current_pose.position.x < 0.15/
            current_pose.position.y > 0.5 and current_pose.position.y < 0.8:
        onionLoc = 2    # In Bin
    elif onion_coordinates.position.x > 0.3 and current_pose.position.x < 0.5/
            current_pose.position.z > 0.1 and current_pose.position.z < 0.5:
        onionLoc = 3    # In Hover Plane
    prediction = predic
    listIDstatus = 2

    return vals2sid(onionLoc, eefLoc, prediction, listIDstatus)

def executePolicyAct(action):
    if action == 0:
        pass
    return

    
def callback_poses(onions_poses_msg):
    global req, target_location_x, target_location_y, target_location_z, onion_index
    if "good" in req.model_name_1:
        # print("I'm waiting for a bad onion!")
        return
    if(onion_index == -1):
        print("No more onions to sort!")
        return
    else:
        if(onion_index == len(onions_poses_msg.x)):
            return
        else:
            current_onions_x = onions_poses_msg.x
            current_onions_y = onions_poses_msg.y
            current_onions_z = onions_poses_msg.z
            target_location_x = current_onions_x[onion_index]
            target_location_y = current_onions_y[onion_index]
            target_location_z = current_onions_z[onion_index]
    # print "target_location_x,target_location_y"+str((target_location_x,target_location_y))
    return


def callback_exec_policy(color_indices_msg):
    global req, onion_index, bad_onion_index, num_onions, flag, bad_onions
    max_index = len(color_indices_msg.data)
    if (color_indices_msg.data[onion_index] == 0):
        req.model_name_1 = "good_onion_" + str(onion_index)
        print "Onion name set in IF as: ", req.model_name_1
        if(onion_index is not max_index - 1):
            onion_index = onion_index + 1
        else:
            onion_index = -1
        return
    else:
        req.model_name_1 = "bad_onion_" + str(onion_index)
        print "Onion name set in ELSE as: ", req.model_name_1
        bad_onions.append(onion_index)

    # attach and detach service
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()
    req.link_name_1 = "base_link"
    req.model_name_2 = "sawyer"
    req.link_name_2 = "right_l6"
    if not flag:
        num_onions = len(color_indices_msg.data)
        flag = True

    if(num_onions > 0):

        print "(model_1,link_1,model_2,link_2)", req.model_name_1, req.link_name_1, req.model_name_2, req.link_name_2
        ##############################################
        print "goto_home()"
        pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
        s = getState(req.model_name_1, color_indices_msg.data[onion_index])
        with open ('policy.csv', newline='') as csvfile:
            policy = csv.reader(csvfile, delimiter=' ', quotechar='|')
            policy = np.array(policy)
            a = policy[s]
        executePolicyAct(a)

def main():

    try:
        rospy.Subscriber("current_onions_blocks",
                         Int8MultiArray, callback_exec_policy)
        rospy.Subscriber("onions_blocks_poses",
                         onions_blocks_poses, callback_poses)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "Main function not found! WTH dude!"
