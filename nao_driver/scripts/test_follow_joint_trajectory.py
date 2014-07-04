#!/usr/bin/env python

import roslib
#from rospy.exceptions import ROSException
roslib.load_manifest('nao_driver')
import rospy
from rospy import Duration


import actionlib
from actionlib_msgs.msg import GoalStatus
import nao_msgs.msg
import control_msgs.msg
from control_msgs.msg import JointTolerance
import trajectory_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
import std_srvs.srv

def joint_angle_client():

    client = actionlib.SimpleActionClient("follow_joint_trajectory",
                                                      control_msgs.msg.FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for follow_joint_trajectory server...")
    client.wait_for_server()
    rospy.loginfo("Done.")

    try:
        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = ["HeadYaw", "HeadPitch"]
        goal.trajectory.points = []
        goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.0),
                                                           positions = [1.0, 1.0]))
        goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.0),
                                                           positions = [1.0, 0.0]))
        goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.5),
                                                           positions = [0.0, 0.0]))

        rospy.loginfo("Sending goal...")
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo("Getting results...")
        result = client.get_result()


    finally:
        pass #uninhibitWalkSrv()

def joint_angle_client2():

    client = actionlib.SimpleActionClient("ra_right/follow_joint_trajectory",
                                                      control_msgs.msg.FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for follow_joint_trajectory server...")
    client.wait_for_server()
    rospy.loginfo("Done.")

    try:
        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        goal.trajectory.points = []
        #goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.5),
        #                                                   positions = [0.82043, -1.14472, -0.320568, -1.16398, 0.457667]))
        goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.0),
                                                           positions = [0.983887, 0.730487, 0.731723, -1.32891, -0.426781]))

        rospy.loginfo("Sending goal...")
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo("Getting results...")
        result = client.get_result()
        print result


    finally:
        pass #uninhibitWalkSrv()


def refine_joint_target_client():
    client = actionlib.SimpleActionClient("ra/refine_joint_target",
                                                      nao_msgs.msg.ReachJointValuesPreciseAction)

    rospy.loginfo("Waiting for follow_joint_trajectory server...")
    client.wait_for_server()
    rospy.loginfo("Done.")

    times = [[0.333897915, 0.477343645, 0.593739838, 0.695000216, 0.788512476, 0.893884431, 1.017441335, 1.175894663, 1.503395099], [0.333897915, 0.477343645, 0.593739838, 0.695000216, 0.788512476, 0.893884431, 1.017441335, 1.175894663, 1.503395099], [0.333897915, 0.477343645, 0.593739838, 0.695000216, 0.788512476, 0.893884431, 1.017441335, 1.175894663, 1.503395099], [0.333897915, 0.477343645, 0.593739838, 0.695000216, 0.788512476, 0.893884431, 1.017441335, 1.175894663, 1.503395099], [0.333897915, 0.477343645, 0.593739838, 0.695000216, 0.788512476, 0.893884431, 1.017441335, 1.175894663, 1.503395099]]
    angles = [[1.021336238804823, 0.9979766234226346, 0.9746170080404459, 0.9512573926582574, 0.9278977772760688, 0.9045381618938803, 0.8811785465116917, 0.857818931129503, 0.8344593157473145], [-0.3545378146523323, -0.37922376353623194, -0.40390971242013163, -0.42859566130403126, -0.453281610187931, -0.47796755907183064, -0.5026535079557303, -0.5273394568396299, -0.5520254057235296], [-0.28902225538070453, -0.29583034602752234, -0.30263843667434015, -0.30944652732115796, -0.31625461796797577, -0.3230627086147936, -0.3298707992616114, -0.3366788899084292, -0.343486980555247], [-0.9964530779689642, -0.952812257164735, -0.909171436360506, -0.8655306155562769, -0.8218897947520477, -0.7782489739478187, -0.7346081531435895, -0.6909673323393604, -0.6473265115351312], [-0.31876162496812277, -0.2877293389560209, -0.25669705294391904, -0.22566476693181714, -0.19463248091971527, -0.16360019490761343, -0.13256790889551157, -0.1015356228834097, -0.07050333687130783]]
    positions = [0.983887, 0.730487, 0.731723, -1.32891, -0.426781]
    pos = [0.920934, -0.808424, -0.439794, -0.963082, 0.097087]
    try:
        goal = nao_msgs.msg.ReachJointValuesPreciseGoal()

        goal.trajectory.joint_names = ['RElbowRoll', 'RElbowYaw', 'RShoulderPitch', 'RShoulderRoll', 'RWristYaw']
        goal.trajectory.points = []
        #pos = []
        #for a in angles:
            #    pos.append(a[-1])
        goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(times[0][-1]),
                                                           positions = pos))
        for i in range(len( goal.trajectory.joint_names )):
            jt = JointTolerance()
            jt.position = 0.001
            goal.goal_tolerance.append ( jt )

        rospy.loginfo("Sending goal...")
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo("Getting results...")
        result = client.get_result()
        print "result is" , result
    finally:
        pass #uninhibitWalkSrv()






if __name__ == '__main__':
    try:
        rospy.init_node('joint_angle_client')
        joint_angle_client()
        #refine_joint_target_client()
        rospy.sleep(2.0)

    except rospy.ROSInterruptException:
        print "program interrupted before completion"

