#!/usr/bin/env python
# coding: utf8

from PickandPlace import PickAndPlace
import sys
import rospy

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
import threading
import message_filters
from smach import *
from smach_ros import *
from smach_msgs.msg import *
import copy
import time

sys.path.append('/home/psuresh/catkin_ws/src/sorting_patrol_MDP_irl')
from sortingMDP.model import sortingModelbyPSuresh4multipleInit_onlyPIP, \
    sortingState, ClaimNewOnion, PlaceOnConveyor, PlaceInBin, \
    Pick, InspectAfterPicking
from sorting_patrol_MDP_irl.srv import requestPolicy


# Global initializations
pnp = PickAndPlace()
done_onions = []
flag = False
total_onions = 4
attach_srv = None
detach_srv = None

def vals2sid(ol, eefl, pred, listst, nOnionLoc=5, nEEFLoc=4, nPredict=3, nlistIDStatus=3):
    return(ol + nOnionLoc * (eefl + nEEFLoc * (pred + nPredict * listst)))

class Get_info(State):
    def __init__(self):
        State.__init__(self, outcomes=['updated', 'not_updated', 'timed_out','completed'],
                    input_keys = ['x','y','z','color','counter'],
                    output_keys = ['x','y','z','color','counter'])
        self.x = []
        self.y = []
        self.z = []
        self.color = []
        # rospy.Subscriber("/object_location", OBlobs, self.callback_vision)
        self.is_updated = False
        self.callback_vision(rospy.wait_for_message("/object_location", OBlobs))

    def callback_vision(self, msg):
        # print '\nCallback vision\n'
        while None in [msg.x,msg.y,msg.z,msg.color]:
            rospy.sleep(0.05)
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.color = msg.color
        self.is_updated = True
    
    def execute(self, userdata):
        rospy.loginfo('Executing state: Get_info')
        if self.is_updated == False:
            rospy.loginfo('Get_info execute: Calling vision to update userdata')
            self.callback_vision(rospy.wait_for_message("/object_location", OBlobs))
            rospy.sleep(0.5)
            for i in range(len(self.x)):
                rospy.loginfo(str((self.x[i],self.y[i])))
        
        if len(self.x)==0:
            # onions dropped out accidentally
            return 'completed'
        
        if userdata.counter >= 500:
            userdata.counter = 0
            return 'timed_out'

        if self.is_updated == True:
            userdata.x = self.x
            userdata.y = self.y
            userdata.z = self.z
            userdata.color = self.color
            userdata.counter = 0
            rospy.sleep(0.05)
            # print '\n after updated \n'
            # for next run
            self.is_updated = False
            self.x, self.y, self.z, self.color = None, None, None, None
            return 'updated'
        else:
            userdata.counter += 1
            # print '\n after not updated \n'
            return 'not_updated'

class Claim(State):
    def __init__(self):
        State.__init__(self, outcomes=['updated', 'not_updated', 'timed_out','not_found', 'completed'],
                input_keys = ['x','y','z','color','counter'],
                output_keys = ['x','y','z','color','counter'])
        self.is_updated = False

    def execute(self, userdata):        
        global pnp, total_onions, attach_srv, detach_srv, done_onions 
        while len(userdata.x) == 0: 
            rospy.sleep(0.05)
        # rospy.loginfo('Executing state: Claim')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'
        if len(userdata.color) == 0:
            return 'not_found'
        max_index = len(userdata.color)

        # attach and detach service
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        attach_srv.wait_for_service()
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        detach_srv.wait_for_service()
        pnp.req.model_name_1 = None
        pnp.req.link_name_1 = "base_link"
        pnp.req.model_name_2 = "sawyer"
        pnp.req.link_name_2 = "right_l6"
        pnp.target_location_x = userdata.x[pnp.onion_index]
        pnp.target_location_y = userdata.y[pnp.onion_index]
        pnp.target_location_z = userdata.z[pnp.onion_index]
        # pnp.bad_onions = [i for i in range(pnp.onion_index, max_index) if (oi.color[i] == 0)]
        # print("Bad onion indices are: ", pnp.bad_onions)
        
        for i in range(total_onions):
            if len(done_onions) == total_onions:
                return 'completed'
            
            if i in done_onions:
                pass
            else:
                onion_guess = "onion_" + str(i)
                rospy.wait_for_service('/gazebo/get_model_state')

                try:
                    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 
                    # print 'Onion name guessed: ', onion_guess 
                    onion_coordinates = model_coordinates(onion_guess, "").pose 
                    print "real world locations for all onions "+str(onion_coordinates) 

                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e

                if pnp.target_location_y - 0.05 <= onion_coordinates.position.y <= pnp.target_location_y + 0.05:
                    if pnp.target_location_x - 0.05 <= onion_coordinates.position.x <= pnp.target_location_x + 0.05:
                        pnp.req.model_name_1 = onion_guess
                        done_onions.append(i)
                        pnp.scene.remove_world_object(pnp.req.model_name_1)
                        self.is_updated = True
                        break
                    else: print("x doesn't match with ",onion_guess)
                else: print("y doesnt match with ",onion_guess)

        if self.is_updated == False:
            userdata.counter += 1
            return 'not_updated'
        else:
            print "Onion name set as: ", pnp.req.model_name_1
            userdata.counter = 0
            rospy.sleep(0.05)
            # for next run 
            self.is_updated = False
            return 'updated'

class Approach(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out','not_found'],
                    input_keys = ['x','y','z','color','counter'],
                    output_keys = ['x','y','z','color','counter'])

    def execute(self, userdata): 
        global pnp, total_onions
        # rospy.loginfo('Executing state: Approach')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        goal_tolerance = 0.1

        if userdata.x[pnp.onion_index] != pnp.target_location_x or \
            userdata.y[pnp.onion_index] != pnp.target_location_y or \
            userdata.z[pnp.onion_index] != pnp.target_location_z:
            print("userdata.x[pnp.onion_index] "+str(userdata.x[pnp.onion_index])+" pnp.target_location_x "+str(pnp.target_location_x)+\
                "userdata.y[pnp.onion_index] "+str(userdata.y[pnp.onion_index])+" pnp.target_location_y "+str(pnp.target_location_y)+\
                "userdata.z[pnp.onion_index] "+str(userdata.z[pnp.onion_index])+" pnp.target_location_z "+str(pnp.target_location_z))
            home = pnp.goto_home(tolerance=0.1, goal_tol=goal_tolerance, orientation_tol=0.1) 
            rospy.sleep(0.5) 
            print("Approach: can't find location for index ",pnp.onion_index) 

            return 'not_found'

        else:
            home = pnp.goto_home(tolerance=0.1, goal_tol=goal_tolerance, orientation_tol=0.1)
            rospy.sleep(0.5)
            if home:
                status = pnp.goAndPick()
                rospy.sleep(2.0)
                if status:
                    userdata.counter = 0
                    return 'success'
                else:
                    userdata.counter += 1
                    return 'failed'
            else:
                userdata.counter += 1
                return 'failed'

class PickSM(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out','not_found'],
                    input_keys = ['x','y','z','color','counter'],
                    output_keys = ['x','y','z','color','counter'])

    def execute(self, userdata): 
        global pnp, total_onions, attach_srv, detach_srv
        # rospy.loginfo('Executing state: Pick')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        # print '\nPICK State: Userdata.x\n', userdata.x 
        # rospy.sleep(50)

        if userdata.x[pnp.onion_index] != pnp.target_location_x or \
            userdata.y[pnp.onion_index] != pnp.target_location_y or \
                userdata.z[pnp.onion_index] != pnp.target_location_z:
            return 'not_found'
        else:
            dip = pnp.dip_incrementally(z_pose = 0.08)
            rospy.sleep(2.0)
            if dip:
                userdata.counter = 0
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'

class Grasp_object(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out','not_found'],
                    input_keys = ['x','y','z','color','counter'],
                    output_keys = ['x','y','z','color','counter'])

    def execute(self, userdata): 
        global pnp, total_onions, attach_srv, detach_srv
        # rospy.loginfo('Executing state: Pick')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        # print userdata.x
        # rospy.sleep(50)

        if userdata.x[pnp.onion_index] != pnp.target_location_x or \
            userdata.y[pnp.onion_index] != pnp.target_location_y or \
                userdata.z[pnp.onion_index] != pnp.target_location_z:
            return 'not_found'
        else:           
            at = attach_srv.call(pnp.req)
            rospy.sleep(0.5)
            if at:
                userdata.counter = 0
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'

class Liftup(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                        input_keys = ['counter'],
                        output_keys = ['counter'])

    def execute(self, userdata): 
        global pnp, total_onions, attach_srv, detach_srv
        # rospy.loginfo('Executing state: Pick')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'
        else:
            lift = pnp.liftgripper()
            rospy.sleep(2.0)
            if lift:
                userdata.counter = 0
                return 'success'
            else:
                detach_srv.call(pnp.req)
                userdata.counter += 1
                return 'failed'

class View(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                        input_keys = ['counter'],
                        output_keys = ['counter'])

    def execute(self, userdata): 
        global pnp, total_onions, attach_srv, detach_srv
        # rospy.loginfo('Executing state: View')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        view = pnp.view()
        rospy.sleep(1.0)
        print "\n\n View(State) pnp.view() "+str(view)+"\n"
        if view:
            rotate = pnp.rotategripper(0.3)
            rospy.sleep(2.0)
            if rotate:
                userdata.counter = 0
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'
        else:
            userdata.counter += 1
            return 'failed'

class PlaceInBinSM(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                        input_keys = ['color', 'counter'],
                        output_keys = ['color', 'counter'])

    def execute(self, userdata): 
        global pnp
        # rospy.loginfo('Executing state: Place')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        place = pnp.goto_bin()
        rospy.sleep(2.0)
        if place:
            userdata.counter = 0
            rospy.loginfo("Place SM successful: userdata.counter = 0")
            return 'success' 
        else:
            userdata.counter += 1
            return 'failed' 

class Detach_object_wo_ClaimNew(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                        input_keys = ['counter'],
                        output_keys = ['counter'])

    def execute(self, userdata): 
        global pnp, detach_srv
        # rospy.loginfo('Executing state: Place')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'
        else:
            detach = detach_srv.call(pnp.req)
            rospy.sleep(0.5)

            if detach:
                userdata.counter = 0
                return 'success'
            else:
                userdata.counter += 1
                return 'failed'

class PlaceOnConveyorSM(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out'],
                        input_keys = ['color', 'counter'],
                        output_keys = ['color', 'counter'])

    def execute(self, userdata): 
        global pnp
        # rospy.loginfo('Executing state: Place')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'

        # problem: object detached before reaching conveyor bcz placeonconveyor finished before reaching conveyor. 
        # place = pnp.placeOnConveyor()
        # rospy.sleep(0.01)

        goal_tolerance = 0.1 
        home = pnp.goto_home(tolerance=0.1, goal_tol=goal_tolerance, orientation_tol=0.1)
        rospy.sleep(0.5)
        approach_status = False
        dip_status = False
        if home:
            time_limit = 120
            start_time = time.time() 
            while (not approach_status) and (time.time()-start_time)<time_limit:
                approach_status = pnp.goAndPick()
                rospy.sleep(1.0)
            
            if approach_status:
                dip_status = pnp.dip_incrementally(z_pose = 0.08)
                rospy.sleep(1.0)

        place = approach_status and dip_status
        if place:
            userdata.counter = 0
            return 'success'
        else:
            userdata.counter += 1
            return 'failed'

class Detach_object(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed', 'timed_out','completed'],
                        input_keys = ['x','y','z','color','counter'],
                        output_keys = ['x','y','z','color','counter'])

    def execute(self, userdata): 
        global pnp, detach_srv
        # rospy.loginfo('Executing state: Place')
        if userdata.counter >= 50:
            userdata.counter = 0
            return 'timed_out'
        else:
            detach = detach_srv.call(pnp.req)
            rospy.sleep(0.5)

            if detach:
                userdata.counter = 0
                
                # if pnp.onion_index < total_onions - 1:
                #     pnp.onion_index = pnp.onion_index + 1

                # accomodating onion accidentally getting out of field of view: find next onion in field of view 
                msg = rospy.wait_for_message("/object_location", OBlobs) 
                userdata.x = msg.x 
                userdata.y = msg.y 
                userdata.z = msg.z 
                userdata.color = msg.color 
                userdata.counter = 0 

                while None in [msg.x,msg.y,msg.z,msg.color]:
                    rospy.sleep(0.05)

                if len(msg.y)==0:
                    return 'completed'

                for gazebo_i in range(total_onions):
                    if gazebo_i in done_onions:
                        pass
                    else:
                        onion_guess = "onion_" + str(gazebo_i)
                        rospy.wait_for_service('/gazebo/get_model_state')

                        try:
                            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 
                            # print 'Onion name guessed: ', onion_guess 
                            onion_coordinates = model_coordinates(onion_guess, "").pose 
                            print "real world locations for all onions "+str(onion_coordinates) 

                        except rospy.ServiceException, e:
                            print "Service call failed: %s" % e

                        for vision_i in range(len(userdata.y)):
                            vision_on_y = userdata.y[vision_i]
                            if vision_on_y - 0.05 <= onion_coordinates.position.y <= vision_on_y + 0.05:
                                vision_on_x = userdata.x[vision_i]
                                if vision_on_x - 0.05 <= onion_coordinates.position.x <= vision_on_x + 0.05:
                                    pnp.onion_index = vision_i
                                    pnp.target_location_x = userdata.x[pnp.onion_index]
                                    pnp.target_location_y = userdata.y[pnp.onion_index]
                                    pnp.target_location_z = userdata.z[pnp.onion_index]

                                    print("next onion claimed")
                                    pnp.req.model_name_1 = onion_guess
                                    done_onions.append(gazebo_i) 
                                    pnp.scene.remove_world_object(pnp.req.model_name_1)
                                    print("done onions updated and collision object removed")

                                    return 'success'                
            else:
                userdata.counter += 1
                return 'failed'

def main():
    
    # rospy.init_node('execute_I2RL_policy',anonymous=True, disable_signals=True)
    global pnp
    rate = rospy.Rate(10) 

    s_mdp = sortingModelbyPSuresh4multipleInit_onlyPIP()
    print("rospy.init_node('execute_I2RL_policy'), s_mdp ")

    call_service = False
    if call_service:
        nOnionLoc=5
        nEEFLoc=4
        nPredict=3
        nlistIDStatus=3
        prev_array_pol=[4]*nOnionLoc*nEEFLoc*nPredict*nlistIDStatus

        rospy.wait_for_service('/runRobustIrlGetPolicy')
        try:
            irl_service = rospy.ServiceProxy("/runRobustIrlGetPolicy", requestPolicy)
            session_index = 0
            for i in range(5):
                response = irl_service()

                if len(response.policy) > 0:
                    if response.policy != prev_array_pol:
                        print("I2RL session number ",session_index)
                        # print ('\n policy learned by IRL \n',response.policy)
                        policy_pip = response.policy
                        prev_array_pol = response.policy
                        session_index += 1
                    else:
                        print("current session did not change policy: either solver broke half-way (a non-reproducible problem) or learning can't be improved further ") 
                else:
                    print ('\nNo policy response from irl service') 
            
        except rospy.ServiceException as e: 
            print("Service call failed: %s"%e) 

        # exit(0)
    else:
        policy_pip = np.array(
            [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 
            4, 4, 4, 4, 4, 4, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0, 4, 3, 4, 4, 4, 
            4, 3, 4, 4, 4, 4, 3, 4, 4, 4, 4, 3, 4, 4, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 
            4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 4, 
            4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 
            3, 4, 4, 4, 4, 3, 4, 4, 4, 4, 3, 4, 4, 4, 4, 4, 4, 4, 0, 4])

    ''' 
    Solution:
    
    Use fixed initial state instead of usin getState,  
    use action 
    pick the deterministic transition model from sortingMDP to identify next state, 
    pick next action 
    PlaceInBin action takes to [2, 2, 2, 2] from where new onion can be claimed 
    PlaceOnConveyor action takes to [ 0, 2, 0, 2 ] where new onion is already claimed 
    So state machine for placeonconveyor do two things: 
    place object on conveyor and move to next index of bounding box. 

    '''

    # state machines for actions claimnewonion, pick, inspectafterpicking, placeinbin, placeonconveyor 
    sm_claimnewonion = StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED','SORTCOMPLETE']) 
    sm_claimnewonion.userdata.sm_x = [] 
    sm_claimnewonion.userdata.sm_y = [] 
    sm_claimnewonion.userdata.sm_z = [] 
    sm_claimnewonion.userdata.sm_color = [] 
    sm_claimnewonion.userdata.sm_counter = 0 
    with sm_claimnewonion: 
        StateMachine.add('GETINFO', Get_info(), 
                        transitions={'updated':'CLAIM', 
                                    'not_updated':'GETINFO',
                                    'timed_out': 'TIMED_OUT',
                                    'completed': 'SORTCOMPLETE'},
                        remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                            'color':'sm_color','counter':'sm_counter'})
        StateMachine.add('CLAIM', Claim(), 
                        transitions={'updated':'SUCCEEDED', 
                                    'not_updated':'CLAIM',
                                    'timed_out': 'TIMED_OUT',
                                    'not_found': 'GETINFO',
                                    'completed': 'SORTCOMPLETE'},
                        remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                            'color':'sm_color','counter':'sm_counter'})

    print("Hey I got here!")

    sm_pick = StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED'])
    sm_pick.userdata.sm_x = []
    sm_pick.userdata.sm_y = []
    sm_pick.userdata.sm_z = []
    sm_pick.userdata.sm_color = []
    sm_pick.userdata.sm_counter = 0
    with sm_pick:
        StateMachine.add('APPROACH', Approach(), 
                        transitions={'success':'PICK', 
                                    'failed':'APPROACH',
                                    'timed_out': 'TIMED_OUT',
                                    'not_found': 'TIMED_OUT'},
                        remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                            'color':'sm_color','counter':'sm_counter'})
        StateMachine.add('PICK', PickSM(), 
                        transitions={'success':'GRASP', 
                                    'failed':'PICK',
                                    'timed_out': 'TIMED_OUT',
                                    'not_found': 'TIMED_OUT'},
                        remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                            'color':'sm_color','counter':'sm_counter'})
        
        StateMachine.add('GRASP', Grasp_object(), transitions={'success':'LIFTUP', 
                                'failed':'GRASP',
                                'timed_out': 'TIMED_OUT',
                                'not_found': 'TIMED_OUT'},
                        remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z',
                        'color':'sm_color','counter':'sm_counter'})
        StateMachine.add('LIFTUP', Liftup(),
                        transitions={'success':'SUCCEEDED', 
                                    'failed':'LIFTUP',
                                    'timed_out': 'TIMED_OUT'},
                        remapping={'counter':'sm_counter'})

    sm_inspectafterpicking = StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED'])
    sm_inspectafterpicking.userdata.sm_x = []
    sm_inspectafterpicking.userdata.sm_y = []
    sm_inspectafterpicking.userdata.sm_z = []
    sm_inspectafterpicking.userdata.sm_color = []
    sm_inspectafterpicking.userdata.sm_counter = 0
    with sm_inspectafterpicking:
        StateMachine.add('VIEW', View(), 
                        transitions={'success':'SUCCEEDED', 
                                    'failed':'VIEW',
                                    'timed_out': 'TIMED_OUT'},
                        remapping={'counter':'sm_counter'})


    sm_placeinbin = StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED'])
    sm_placeinbin.userdata.sm_x = []
    sm_placeinbin.userdata.sm_y = []
    sm_placeinbin.userdata.sm_z = []
    sm_placeinbin.userdata.sm_color = []
    sm_placeinbin.userdata.sm_counter = 0
    with sm_placeinbin:
        StateMachine.add('PLACEINBIN', PlaceInBinSM(), 
                        transitions={'success':'DETACH', 
                                    'failed':'PLACEINBIN',
                                    'timed_out': 'TIMED_OUT'},
                        remapping={'color': 'sm_color','counter':'sm_counter'})
        StateMachine.add('DETACH', Detach_object_wo_ClaimNew(),
                        transitions={'success':'SUCCEEDED', 
                                'failed':'DETACH',
                                'timed_out': 'TIMED_OUT'},
                        remapping={'counter':'sm_counter'})


    sm_placeonconveyor = StateMachine(outcomes=['TIMED_OUT', 'SUCCEEDED', 'SORTCOMPLETE'])
    sm_placeonconveyor.userdata.sm_x = []
    sm_placeonconveyor.userdata.sm_y = []
    sm_placeonconveyor.userdata.sm_z = []
    sm_placeonconveyor.userdata.sm_color = []
    sm_placeonconveyor.userdata.sm_counter = 0
    with sm_placeonconveyor:
        StateMachine.add('PLACEONCONVEYOR', PlaceOnConveyorSM(), 
                        transitions={'success':'DETACH', 
                                    'failed':'PLACEONCONVEYOR',
                                    'timed_out': 'TIMED_OUT'},
                        remapping={'color': 'sm_color','counter':'sm_counter'})
        StateMachine.add('DETACH', Detach_object(),
                        transitions={'success':'LIFTUP', 
                                'failed':'DETACH',
                                'timed_out': 'TIMED_OUT',
                                'completed':'SORTCOMPLETE'},
                        remapping={'x':'sm_x', 'y': 'sm_y', 'z': 'sm_z','color': 'sm_color','counter':'sm_counter'}) 
        StateMachine.add('LIFTUP', Liftup(),
                        transitions={'success':'SUCCEEDED', 
                                    'failed':'LIFTUP',
                                    'timed_out': 'TIMED_OUT'},
                        remapping={'counter':'sm_counter'})

    aid2act = {0:'InspectAfterPicking',
            1:'PlaceOnConveyor',
            2:'PlaceInBin',
            3:'Pick',
            4:'ClaimNewOnion',
            5:'InspectWithoutPicking',
            6:'ClaimNextInList' 
        } 

    act2sm = {'InspectAfterPicking':sm_inspectafterpicking,
            'PlaceOnConveyor':sm_placeonconveyor,
            'PlaceInBin':sm_placeinbin,
            'Pick':sm_pick,
            'ClaimNewOnion':sm_claimnewonion
            }

    prev_sm = sm_claimnewonion
    s = sortingState(2,2,2,2)

    while not rospy.is_shutdown(): 
        print("current state ",s)

        # use policy to pick current action and SM 
        sid = vals2sid(s._onion_location, s._EE_location, s._prediction, s._listIDs_status) 
        aid = policy_pip[sid]
        current_action = aid2act[aid]
        print("current action ",current_action)

        # current state machine based on action, and pass userdata to next sm
        current_sm = act2sm[current_action]
        current_sm.userdata = prev_sm.userdata
        current_sm.userdata.sm_counter = 0

        # execute action 
        outcome = current_sm.execute() 
        print("outcome ",outcome)
        prev_sm = current_sm
        
        # exit(0)
        if outcome=='SORTCOMPLETE':
            exit(0)
        elif outcome == 'SUCCEEDED':
            # next state
            if current_action=='ClaimNewOnion': 
                a = ClaimNewOnion()         
            elif current_action=='PlaceOnConveyor': 
                a = PlaceOnConveyor() 
            elif current_action=='PlaceInBin': 
                a = PlaceInBin() 
            elif current_action=='Pick': 
                a = Pick() 
            elif current_action=='InspectAfterPicking': 
                a = InspectAfterPicking() 
            else: 
                print ("Unepxected action chosen action.") 

            for ns in s_mdp.T(s,a).keys():
                if ns!=s:
                    s = ns
                    print("ns!=s next state is ",s)
                    break
            
            if current_action=='InspectAfterPicking':
                print("prediction ",int(prev_sm.userdata.sm_color[pnp.onion_index]))
                s._prediction = int(prev_sm.userdata.sm_color[pnp.onion_index])
        
        rate.sleep()

def check_transformations():
    msg = rospy.wait_for_message("/object_location", OBlobs)
    print("onion corrdinates from vision")
    for i in range(len(msg.x)):
        print(msg.x[i],msg.y[i])

    print("coordinates in gazebo")
    for i in range(total_onions):
        onion_guess = "onion_" + str(i)
        rospy.wait_for_service('/gazebo/get_model_state')

        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 
            onion_coordinates = model_coordinates(onion_guess, "").pose.position
            print("\n"+str(onion_coordinates))
    
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

if __name__ == '__main__': 

    check_transformations()
    rospy.signal_shutdown("Manual shutdown ")

    try:
        main()
    except rospy.ROSInterruptException:
        print ("exeute_policy_statemachine_gazebo: Main function not found ") 
    except KeyboardInterrupt:
        rospy.signal_shutdown("Manual shutdown ")