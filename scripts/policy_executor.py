from simple_pnp_gazebo import PickAndPlace
import intera_interface
from robotiq_2f_gripper_control.srv import move_robot, move_robotResponse
import numpy
from sawyer_irl_project.msg import onions_blocks_poses
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

''' Picked/AtHome - means Sawyer is in hover plane at home position
    Placed - means placed back on conveyor after inspecting and finding status as good
    onionLoc = {0: 'OnConveyor', 1: 'InFront', 2: 'InBin', 3: 'Picked/AtHome', 4: 'Placed'}
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

def callback_exec_policy(color_indices_msg):

    current_pose = pnp.group.get_current_pose().pose

def main():
    
    try:
        rospy.Subscriber("current_onions_blocks", Int8MultiArray, callback_exec_policy)
        rospy.Subscriber("onions_blocks_poses", onions_blocks_poses, callback_poses)
        
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