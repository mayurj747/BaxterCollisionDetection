#!/usr/bin/python

import numpy as np
import rospy
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse
from moveit_commander import RobotCommander
from moveit_msgs.msg import RobotState, DisplayRobotState, ContactInformation
import baxter_interface
from set_environment import main as set_environment
from baxter_tests.msg import ContactInformationArray

DEFAULT_SV_SERVICE = "/check_state_validity"

class StateValidity():
    def __init__(self):
        rospy.loginfo("Initializing stateValidity class")
        self.sv_srv = rospy.ServiceProxy(DEFAULT_SV_SERVICE, GetStateValidity)
        rospy.loginfo("Connecting to State Validity service")
        try:
            self.sv_srv.wait_for_service()
            rospy.wait_for_service("check_state_validity")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))

        if rospy.has_param('/play_motion/approach_planner/planning_groups'):
            list_planning_groups = rospy.get_param('/play_motion/approach_planner/planning_groups')
        else:
            rospy.logwarn("Param '/play_motion/approach_planner/planning_groups' not set. We can't guess controllers")
        rospy.loginfo("Ready for making Validity calls")


    def close_SV(self):
        self.sv_srv.close()


    def getStateValidity(self, robot_state, group_name='both_arms_torso', constraints=None):
        """Given a RobotState and a group name and an optional Constraints
        return the validity of the State"""
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = robot_state
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)
        return result # valid is boolean, return result for full contact info


def sample_loc(name):
    joint = {}
    # ref hardware specs for baxter joint limits
    joint[name+'_s0'] = float(np.random.uniform(-1.7016, 1.7016)) # s0
    joint[name+'_s1'] = float(np.random.uniform(-2.147, 1.047))  # s1
    joint[name+'_e0'] = float(np.random.uniform(-3.0541, 3.0541)) # e0
    joint[name+'_e1'] = float(np.random.uniform(-0.05, 2.618)) # e1
    joint[name+'_w0'] = float(np.random.uniform(-3.059, 3.059)) # w0
    joint[name+'_w1'] = float(np.random.uniform(-1.5707, 2.094)) # w1
    joint[name+'_w2'] = float(np.random.uniform(-3.059, 3.059)) # w2
    return joint


if __name__ == '__main__':

    set_environment() # initializes rospy node, creates a test environment with collision objects

    """ Preliminaries, initialize limb, set up publisher for collision checking """
    limb_name = 'right'
    limb = baxter_interface.Limb(limb_name)
    # robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState, queue_size=0)
    robot_contact_pub = rospy.Publisher('/robot_collision_contacts', ContactInformationArray, queue_size=0) # publish collision contact info
    rospy.sleep(1)

    sv = StateValidity()

    # Sample a waypoint
    waypoint = sample_loc(name=limb_name)

    """
    CONSTRUCT A ROBOTSTATE MSG FROM SAMPLED WAYPOINT FOR COLLISION CHECKING
    """
    # ---- Create filler RobotState msg from RobotCommander ----------------
    rs = RobotState()
    robot = RobotCommander()
    robot_state = robot.get_current_state()
    rs.joint_state.name = robot_state.joint_state.name
    rs.joint_state.position = list(robot_state.joint_state.position) # filler for rest of the joint angles not found in waypoinr
    # ----------------------------------------------------------------------

    joint_name_indices = [rs.joint_state.name.index(n) for n in waypoint.keys()]
    for i, idx in enumerate(joint_name_indices):
        rs.joint_state.position[idx] = waypoint.values()[i]

    collision = sv.getStateValidity(rs, group_name=limb_name+'_arm')
    print(collision.valid) # Boolean

    # Publish collision information

    # drs = DisplayRobotState()
    # drs.state = rs
    # rospy.sleep(1)
    # robot_state_collision_pub.publish(drs)

    contacts_msg = ContactInformationArray()
    contacts_msg.contacts = collision.contacts

    if not collision.valid: robot_contact_pub.publish(contacts_msg)

    limb.move_to_joint_positions(waypoint) # moves to waypoint for visual confirmation
