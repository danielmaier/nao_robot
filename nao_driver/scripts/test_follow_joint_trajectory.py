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
import trajectory_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
import std_srvs.srv

# go to crouching position
def joint_angle_client():
    #inhibitWalkSrv = rospy.ServiceProxy("inhibit_walk", std_srvs.srv.Empty)
    #uninhibitWalkSrv = rospy.ServiceProxy("uninhibit_walk", std_srvs.srv.Empty)

    client = actionlib.SimpleActionClient("follow_joint_trajectory",
                                                      control_msgs.msg.FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for follow_joint_trajectory server...")
    client.wait_for_server()
    #stiffness_client.wait_for_server()
    #angle_client.wait_for_server()
    rospy.loginfo("Done.")

    #inhibitWalkSrv()
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
        #print "Result:", ', '.join([str(n) for n in result.goal_position.position])


    finally:
        pass #uninhibitWalkSrv()



if __name__ == '__main__':
    try:
        rospy.init_node('joint_angle_client')
        joint_angle_client()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"

