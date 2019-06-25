#!/usr/bin/env python

import rospy
from math import pow, atan2, sqrt
from tf.transformations import *

import smach
import smach_ros
from smach_ros import SimpleActionState
from smach_ros import ServiceState



import threading
from robotics_main.srv import *
# Navigation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Manipulator 
from geometry_msgs.msg import Pose
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import SetKinematicsPose
# parameter
namespace = 'om_with_tb3'  # '/om_with_tb3'
planning_group = 'arm'  # '/arm'

# joint parameter
cup_state = [0.0, 0.7, -0.23, 0.9]
lego_state = [0.0, 0.7, -0.23, 0.9]
place_state = [0.0, 0.4, -0.4, 0.0]
init_state = [0.0, 0.5, -0.1, 0.34]
holding_state = [0.0, -1.5, 1.0, 0.6]

# gripper parameter
open_state = [0.0]
close_state = [0.015]


class getCloserToBox(smach.State): # go straight for some time duration

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.namespace = 'om_with_tb3' # /om_with_tb3
        self.cmd_vel_pub = rospy.Publisher(self.namespace + '/cmd_vel', Twist, queue_size=10)

        self.cmd_vel = Twist()
    
    def execute(self, userdata):
        self.cmd_vel.linear.x = 0.3
        rospy.sleep(3.)
        return 'succeeded'
def get_state(tr_type):
    if tr_type == "cup":
        return cup_state
    elif tr_type =="lego":
        return lego_state


def execute(req):
    tr_type = req.type
    pick_center = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    # place the trash to trashBin
    if tr_type == "place":
        
        # place trash
        with pick_center:
            pick_center.userdata.planning_group = planning_group

            def joint_position_request_cb(userdata, request):
                joint = JointPosition()
                joint.position = userdata.input_position
                joint.max_velocity_scaling_factor = 1.0
                joint.max_accelerations_scaling_factor = 1.0

                request.planning_group = userdata.input_planning_group
                request.joint_position = joint
                return request

            def joint_position_response_cb(userdata, response):
                if response.is_planned == False:
                    return 'aborted'
                else:
                    rospy.sleep(3.5)
                    return 'succeeded'

            def gripper_request_cb(userdata, request):
                joint = JointPosition()
                joint.position = userdata.input_gripper
                joint.max_velocity_scaling_factor = 1.0
                joint.max_accelerations_scaling_factor = 1.0

                request.planning_group = userdata.input_planning_group
                request.joint_position = joint
                return request

            def gripper_response_cb(userdata, response):
                rospy.sleep(1.5)
                return 'succeeded'
            # open gripper
            pick_center.userdata.open_gripper = close_state
            smach.StateMachine.add('OPEN_GRIPPER',
                                    ServiceState(namespace + '/gripper',
                                                    SetJointPosition,
                                                    request_cb=gripper_request_cb,
                                                    response_cb=gripper_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_gripper']),
                                    transitions={'succeeded':'SET_INIT_POSITION'},
                                    remapping={'input_planning_group':'planning_group',
                                            'input_gripper':'open_gripper'})
            # before pick up state
            pick_center.userdata.init_position = place_state  # adjust using teleop_test_node
            smach.StateMachine.add('SET_INIT_POSITION',
                                    ServiceState(planning_group + '/moveit/set_joint_position',
                                                SetJointPosition,
                                                    request_cb=joint_position_request_cb,
                                                    response_cb=joint_position_response_cb,
                                                    input_keys=['input_planning_group',
                                                                    'input_position']),
                                    transitions={'succeeded':'CLOSE_TO_OBJECT'},  # change to ALIGN_WITH_TRASH
                                    remapping={'input_planning_group':'planning_group',
                                            'input_position':'init_position'})

          # pick up trash
            pick_center.userdata.object_pose = place_state
            smach.StateMachine.add('CLOSE_TO_OBJECT',
                                    ServiceState(planning_group + '/moveit/set_joint_position',
                                                    SetJointPosition,
                                                    request_cb=joint_position_request_cb,
                                                    response_cb=joint_position_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_position']),                                                    
                                    transitions={'succeeded':'CLOSE_GRIPPER',
                                                'aborted':'CLOSE_TO_OBJECT'},
                                    remapping={'input_planning_group':'planning_group',
                                            'input_position':'object_pose'})
            # close gripper
            pick_center.userdata.close_gripper = open_state
            smach.StateMachine.add('CLOSE_GRIPPER',
                                    ServiceState(namespace + '/gripper',
                                                    SetJointPosition,
                                                    request_cb=gripper_request_cb,
                                                    response_cb=gripper_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_gripper']),
                                    transitions={'succeeded':'SET_HOLDING_POSITION'},
                                    remapping={'input_planning_group':'planning_group',
                                            'input_gripper':'close_gripper'})
            # holding
            pick_center.userdata.holding_position = holding_state  # adjust using teleop_test_node
            smach.StateMachine.add('SET_HOLDING_POSITION',
                                    ServiceState(planning_group + '/moveit/set_joint_position',
                                                    SetJointPosition,
                                                    request_cb=joint_position_request_cb,
                                                    response_cb=joint_position_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_position']),
                                    transitions={'succeeded':'succeeded'},
                                    remapping={'input_planning_group':'planning_group',
                                            'input_position':'holding_position'})
    # pick up the trash by trash type
    else :    
        trash_state = get_state(tr_type)
        # pick up trash
        with pick_center:
            pick_center.userdata.planning_group = planning_group

            def joint_position_request_cb(userdata, request):
                joint = JointPosition()
                joint.position = userdata.input_position
                joint.max_velocity_scaling_factor = 1.0
                joint.max_accelerations_scaling_factor = 1.0

                request.planning_group = userdata.input_planning_group
                request.joint_position = joint
                return request

            def joint_position_response_cb(userdata, response):
                if response.is_planned == False:
                    return 'aborted'
                else:
                    rospy.sleep(3.5)
                    return 'succeeded'

            def gripper_request_cb(userdata, request):
                joint = JointPosition()
                joint.position = userdata.input_gripper
                joint.max_velocity_scaling_factor = 1.0
                joint.max_accelerations_scaling_factor = 1.0

                request.planning_group = userdata.input_planning_group
                request.joint_position = joint
                return request

            def gripper_response_cb(userdata, response):
                rospy.sleep(1.5)
                return 'succeeded'
            # open gripper
            pick_center.userdata.open_gripper = open_state
            smach.StateMachine.add('OPEN_GRIPPER',
                                    ServiceState(namespace + '/gripper',
                                                    SetJointPosition,
                                                    request_cb=gripper_request_cb,
                                                    response_cb=gripper_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_gripper']),
                                    transitions={'succeeded':'SET_INIT_POSITION'},
                                    remapping={'input_planning_group':'planning_group',
                                            'input_gripper':'open_gripper'})
            # before pick up state
            pick_center.userdata.init_position = init_state  # adjust using teleop_test_node
            smach.StateMachine.add('SET_INIT_POSITION',
                                    ServiceState(planning_group + '/moveit/set_joint_position',
                                                SetJointPosition,
                                                    request_cb=joint_position_request_cb,
                                                    response_cb=joint_position_response_cb,
                                                    input_keys=['input_planning_group',
                                                                    'input_position']),
                                    transitions={'succeeded':'CLOSE_TO_OBJECT'},  # change to ALIGN_WITH_TRASH
                                    remapping={'input_planning_group':'planning_group',
                                            'input_position':'init_position'})

            # pick up trash
            pick_center.userdata.object_pose = trash_state
            smach.StateMachine.add('CLOSE_TO_OBJECT',
                                    ServiceState(planning_group + '/moveit/set_joint_position',
                                                    SetJointPosition,
                                                    request_cb=joint_position_request_cb,
                                                    response_cb=joint_position_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_position']),                                                    
                                    transitions={'succeeded':'CLOSE_GRIPPER',
                                                'aborted':'CLOSE_TO_OBJECT'},
                                    remapping={'input_planning_group':'planning_group',
                                            'input_position':'object_pose'})
            # close gripper
            pick_center.userdata.close_gripper = close_state
            smach.StateMachine.add('CLOSE_GRIPPER',
                                    ServiceState(namespace + '/gripper',
                                                    SetJointPosition,
                                                    request_cb=gripper_request_cb,
                                                    response_cb=gripper_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_gripper']),
                                    transitions={'succeeded':'SET_HOLDING_POSITION'},
                                    remapping={'input_planning_group':'planning_group',
                                            'input_gripper':'close_gripper'})
            # holding
            pick_center.userdata.holding_position = holding_state  # adjust using teleop_test_node
            smach.StateMachine.add('SET_HOLDING_POSITION',
                                    ServiceState(planning_group + '/moveit/set_joint_position',
                                                    SetJointPosition,
                                                    request_cb=joint_position_request_cb,
                                                    response_cb=joint_position_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_position']),
                                    transitions={'succeeded':'succeeded'},
                                    remapping={'input_planning_group':'planning_group',
                                            'input_position':'holding_position'})
    sis = smach_ros.IntrospectionServer('server_name', pick_center, '/PICK_CENTER')
    sis.start()
    sis.stop()
    # Execute SMACH plan
    outcome = pick_center.execute()
    return ObjectTypeResponse(True)
     
def nothing(req):
    print("exeute type is {}".format(req.type))
    return ObjectTypeResponse(True)
def main():
    print("planning")
    rospy.init_node('kong_test_node')
    # service 
    s = rospy.Service("pickUp_trash",ObjectType,execute)
    rospy.spin()

if __name__ == '__main__':
    main()                                                                  