#!/usr/bin/env python

#
# ROS node to provide joint angle control to Nao by wrapping NaoQI
# This code is currently compatible to NaoQI version 1.6 or newer (latest
# tested: 1.12)
#
# Copyright 2011 Armin Hornung, Daniel Maier, University of Freiburg
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


import copy
import random
import rospy
import actionlib
import nao_msgs.msg
from nao_msgs.msg import(
    JointTrajectoryGoal,
    JointTrajectoryResult,
    JointTrajectoryAction,
    JointAnglesWithSpeed,
    JointAnglesWithSpeedGoal,
    JointAnglesWithSpeedResult,
    JointAnglesWithSpeedAction,
    ReachJointValuesPreciseGoal,
    ReachJointValuesPreciseResult,
    ReachJointValuesPreciseAction
)
from control_msgs.msg import(
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryAction
)

#from trajectory_msgs import JointTrajectoryPoint
from nao_driver import NaoNode

import math
from math import fabs
import time

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState

class NaoController(NaoNode):
    def __init__(self):
        NaoNode.__init__(self)

        # ROS initialization:
        rospy.init_node('nao_controller')

        self.connectNaoQi()

        # store the number of joints in each motion chain and collection, used for sanity checks
        self.collectionSize = {}
        for collectionName in ['Head', 'LArm', 'LLeg', 'RLeg', 'RArm', 'Body', 'BodyJoints', 'BodyActuators']:
            self.collectionSize[collectionName] = len(self.motionProxy.getJointNames(collectionName));


        # Get poll rate for actionlib (ie. how often to check whether currently running task has been preempted)
        # Defaults to 200ms
        self.poll_rate = int(rospy.get_param("~poll_rate", 0.2)*1000)

        # initial stiffness (defaults to 0 so it doesn't strain the robot when no teleoperation is running)
        # set to 1.0 if you want to control the robot immediately
        initStiffness = rospy.get_param('~init_stiffness', 0.0)

        # TODO: parameterize
        if initStiffness > 0.0 and initStiffness <= 1.0:
            self.motionProxy.stiffnessInterpolation('Body', initStiffness, 0.5)


        # start services / actions:
        self.enableStiffnessSrv = rospy.Service("body_stiffness/enable", Empty, self.handleStiffnessSrv)
        self.disableStiffnessSrv = rospy.Service("body_stiffness/disable", Empty, self.handleStiffnessOffSrv)


        #Start simple action servers
        self.jointTrajectoryServer = actionlib.SimpleActionServer("joint_trajectory", JointTrajectoryAction,
                                                                  execute_cb=self.executeJointTrajectoryAction,
                                                                  auto_start=False)

        self.jointStiffnessServer = actionlib.SimpleActionServer("joint_stiffness_trajectory", JointTrajectoryAction,
                                                                  execute_cb=self.executeJointStiffnessAction,
                                                                  auto_start=False)

        self.jointAnglesServer = actionlib.SimpleActionServer("joint_angles_action", JointAnglesWithSpeedAction,
                                                                  execute_cb=self.executeJointAnglesWithSpeedAction,
                                                                  auto_start=False)

        self.followJointTrajectoryServer = actionlib.SimpleActionServer("follow_joint_trajectory", FollowJointTrajectoryAction,
                                                                  execute_cb=self.executeFollowJointTrajectoryAction,
                                                                  auto_start=False)

        self.refineJointTargetServer = actionlib.SimpleActionServer("refine_joint_target", ReachJointValuesPreciseAction,
                                                                  execute_cb=self.refineJointTarget,
                                                                  auto_start=False)

        # cach goal
        self.active_goal_ = None # will be a FollowJointTrajectoryGoal
        self.current_traj_ = None # trajectory_msgs/JointTrajectory
        self.last_trajectory_state_ = None # FollowJointTrajectoryFeedback

        #Start action servers
        self.jointTrajectoryServer.start()
        self.jointStiffnessServer.start()
        self.jointAnglesServer.start()
        self.followJointTrajectoryServer.start()
        self.refineJointTargetServer.start()

        # subsribers last:
        rospy.Subscriber("joint_angles", JointAnglesWithSpeed, self.handleJointAngles, queue_size=10)
        rospy.Subscriber("joint_stiffness", JointState, self.handleJointStiffness, queue_size=10)

        rospy.loginfo("nao_controller initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.getProxy("ALMotion")
        if self.motionProxy is None:
            exit(1)

    # bool (FollowJointTrajectoryFeedback msg, JointTrajectory traj)
    def withinGoalConstraints( self, msg, traj):
        return True
    # void (FollowJointTrajectoryFeedback msg)
    def controllerStateCB( self, msg):
        None

    # FollowJointTrajectory callbacks
    def cancelFollowJointTrajCB(self, gh):
        None
    def goalFollowJointTrajCB(self, gh):
        None

    def unused(self):
        while False and not target_reached and not preempt and not rospy.is_shutdown():
            #If we haven't started the task already...
            if task_id is None:
                print "posting new angle target"
                # ...Start it in another thread (thanks to motionProxy.post)
                print "setting angles:"
                print angles
                task_id = self.motionProxy.post.angleInterpolation(names, angles, times, True) # True = absolute

            preempt = self.followJointTrajectoryServer.is_preempt_requested()
            #Wait for a bit to complete, otherwise check we can keep running
            task_running = self.motionProxy.wait(task_id, self.poll_rate)
            if task_running:
                continue
            cur_angles = self.motionProxy.getAngles(names, True)
            diff = [target_angles[i][0] - cur_angles[i] for i in range(n)]
            print names
            print "target:"
            print target_angles
            print "current:"
            print cur_angles
            print "delta:"
            print diff
            check = [abs(a) < 1e-3 for a in diff]
            if all(check):
                target_reached = True
                if not target_reached:
                    times = [[0.4]] * n
                task_id = None # delete task_id (not running)
                for i in range(len(check)):
                    if not check[i]:
                        angles[i][0] = target_angles[i][0] + random.gauss(0, 0.01)
                print "sampling new angles:"
                print angles
# [[0.8344593157473145], [-0.5520254057235296], [-0.343486980555247], [-0.6473265115351312], [-0.07050333687130783]]


    def setAngles(self, jn, target_angle, thresh, t, inc, lim, max_dur=15.0 ):
        new_target = target_angle
        diff = 1000.0
        crucial_error = False
        start_time = time.time()
        while not crucial_error :
            if ( time.time() - start_time) > max_dur:
                rospy.logwarn("Time limit %f for joint %s exceeded"%(max_dur, jn))
                crucial_error = True
                break
            if self.refineJointTargetServer.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo("Preempt requested")
                break
            self.motionProxy.angleInterpolation(jn, new_target, t, True)
            cur = self.motionProxy.getAngles(jn, True)
            diff = cur[0] - target_angle
            if fabs(diff) < thresh:
                print "%s: difference is %lf < %lf with target=%lf, cur=%lf. quitting succesfully"%(jn,fabs(diff),thresh, target_angle,cur[0])
                break
            if fabs(diff) > 0.45:
                rospy.logerr("%s angle difference is %lf. Joint is probably overheated"%(jn, diff))
                crucial_error = True
                break
            if diff < 0:
                new_target = new_target + inc
            elif diff > 0:
                new_target = new_target - inc
            print "%s target: %lf, cur: %lf, setting: %lf, min: %lf, max: %lf"%(jn, target_angle, cur[0], new_target,lim[0],lim[1])
            if new_target > lim[1] or new_target < lim[0]:
                crucial_error = True
                rospy.logerr("Reached limit for %s. target is %lf and limits are [%lf, %lf]"%(jn, new_target, lim[0], lim[1]))
        return (not crucial_error, new_target)

    def refineJointTarget(self, goal):
        dur = 0.25
        max_dur = 15.0
        increment = 0.01
        rospy.loginfo("Refine called")
        if len(goal.trajectory.points) > 1:
            rospy.logerr("only supporting single goal right now")

        res = ReachJointValuesPreciseResult()
        res.error_code = -6;

        names, target_angles, times = self.jointTrajectoryGoalMsgToAL(goal)
        old_stiff = self.motionProxy.getStiffnesses(names)
        #angles = copy.deepcopy(target_angles) # make a copy
        n = len(target_angles)
        print "initial angles: \n", names
        print target_angles
        #print "goal tolerance"
        #print goal.goal_tolerance
        pos_tolerance = [a.position for a in goal.goal_tolerance]

        class Pair:
            def __init__(self, a, b):
                self.reading = a
                self.command = b

        final_command = JointState()
        final_reading = JointState()
        final_reading.name = copy.deepcopy(names)
        final_command.name = copy.deepcopy(names)
        final_reading.position = []
        final_command.position = []
        finals = dict()
        for name in names:
            finals[ name ] = Pair(0.0, 0.0)

        error = False
        preempt = False
        self.motionProxy.setSmartStiffnessEnabled(False)
        for jn, ta, tol in zip(names, target_angles, pos_tolerance):
            if self.refineJointTargetServer.is_preempt_requested() or rospy.is_shutdown():
                preempt = True
                break
            new_target = ta[0]
            #TODO: Reenable
            # self.motionProxy.stiffnessInterpolation( jn, 1.0, dur)
            lim = self.motionProxy.getLimits(jn)[0]
            ok, target = self.setAngles(jn, new_target, tol, dur, increment, lim, max_dur)
            finals[jn].command = target
            #finals[jn].reading = target
            #final_command.name.append( jn )
            #final_command.position.append( target )
            #final_reading.name.append( jn )
            #final_reading.position.append( target ) # will be overwritten later
            if not ok:
                error = True
                break


        cur_angles = self.motionProxy.getAngles(names, True)
        diff = [target_angles[i][0] - cur_angles[i] for i in range(n)]
        check = [abs(a) < t for a,t in zip(diff, pos_tolerance) ]
        # print "reached: ", names
        # print cur_angles
        # print "diff is: "
        # print diff
        # print "success?"
        # print check
        if not error and not preempt:
            for i in range(n):
                if self.refineJointTargetServer.is_preempt_requested() or rospy.is_shutdown():
                    preempt = True
                    rospy.loginfo("Preempt requested")
                    break
                if not check[i]:
                    print "Trying to fix %s once more"%(names[i])
                    lim = self.motionProxy.getLimits(names[i])[0]
                    ok, target = self.setAngles(names[i], target_angles[i][0], pos_tolerance[i], 2*dur, increment/2.0, lim, max_dur)
                    finals[ names[i] ].command = target
                    #idx = final_command.name.index(n)
                    #final_command.position[idx] = target
                    if not ok:
                        error = True
                        break

        if error:
            rospy.logerr('Error occured')

        #restore previous stiffness
        self.motionProxy.stiffnessInterpolation( names, old_stiff, [[dur]]*n)

        cur_angles = self.motionProxy.getAngles(names, True)
        diff = [target_angles[i][0] - cur_angles[i] for i in range(n)]
        check = [abs(a) < t for a,t in zip(diff, pos_tolerance) ]
        print "reached: ", names
        print cur_angles
        print "diff is: "
        print diff
        print "success?"
        print check
        for (name, c) in zip(names, cur_angles):
            finals[name].reading = c

        #for n, c in zip(names, cur_angles):
        #    idx = final_reading.name.index(n)
        #    final_reading.position[idx]=c

        #for jn, val in zip(final_reading.name, final_reading.position):
        #    print "final reading ", jn, val
        for (key, val) in finals.iteritems():
            print key, ":", val.command, ",", val.reading
            res.final_command.name.append(key)
            res.final_command.position.append(val.command)
            res.final_reading.name.append(key)
            res.final_reading.position.append(val.reading)

        res.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
        if all(check):
            print 'succ'
            res.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self.refineJointTargetServer.set_succeeded(res)
            rospy.loginfo("refineJointTargetServer action done")
        elif error:
            print 'error'
            self.refineJointTargetServer.set_preempted(res)
        elif preempt:
            print 'preempt'
            self.refineJointTargetServer.set_preempted(res)
            rospy.logdebug("refineJointTargetServer preempted")
        else: # neither success nor error nor preempt-->violation of tolerance
            print 'nothing'
            rospy.loginfo("refineJointTargetServer action done")


    def executeFollowJointTrajectoryAction(self, goal):
        rospy.loginfo("FollowJointTrajectory action executing");

        if len(goal.trajectory.points) > 1:
            goal.trajectory.points = goal.trajectory.points[1:]
        # convert to naoqi names, angles, times
        names, angles, times = self.jointTrajectoryGoalMsgToAL(goal)

        # Hack needed because moveit sends position[0].time_from_start=0


        rospy.loginfo("Received trajectory for joints: %s times: %s", str(names), str(times))
        rospy.loginfo("Trajectory angles: %s", str(angles))
        #rospy.loginfo("goal_tolerance: ", str([jt for jt in goal.goal_tolerance]))
        print "goal_tolerance"
        print goal.goal_tolerance

        task_id = None
        running = True
        preempt = False
        #Wait for task to complete
        # TODO 1: get current position and check if position[0]==current position+path_tolerance (mix with TODO 3)
        # TODO 2: add timout using goal.goal_time_tolerance
        # TODO 3: add JointTolerance check using goal.path_tolerance
        # TODO 4: add feedback

        while running and not preempt and not rospy.is_shutdown():
            #If we haven't started the task already...
            if task_id is None:
                # ...Start it in another thread (thanks to motionProxy.post)
                task_id = self.motionProxy.post.angleInterpolation(names, angles, times, True) # True = absolute

            preempt = self.followJointTrajectoryServer.is_preempt_requested()
            #Wait for a bit to complete, otherwise check we can keep running
            running = self.motionProxy.wait(task_id, self.poll_rate)

        # If still running at this point, stop the task
        if running and task_id:
            self.motionProxy.stop( task_id )

        followJointTrajectoryResult = FollowJointTrajectoryResult()
        # followJointTrajectoryResult.goal_position.header.stamp = rospy.Time.now()
        # followJointTrajectoryResult.goal_position.position = self.motionProxy.getAngles(names, True)
        # followJointTrajectoryResult.goal_position.name = names

#         if not self.checkJointsLen(followJointTrajectoryResult.goal_position):
#             followJointTrajectoryResult.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
#             self.followJointTrajectoryServer.set_aborted(followJointTrajectoryResult)
#             rospy.logerr("FollowJointTrajectory action error in result: sizes mismatch")

        cur_angles = self.motionProxy.getAngles(names, True)
        diff = [angles[i][-1] - cur_angles[i] for i in range(len(angles))]
        #check = [ fabs(diff[i] -
        print "reached: ", cur_angles
        # TODO: call refine if tolerance is violated (BUG: tolerance not specified by MoveIt!)
        if running and preempt:
            self.followJointTrajectoryServer.set_preempted(followJointTrajectoryResult)
            rospy.logdebug("FollowJointTrajectory preempted")

        else:
            self.followJointTrajectoryServer.set_succeeded(followJointTrajectoryResult)
            rospy.loginfo("FollowJointTrajectory action done")


    def handleJointAngles(self, msg):
        rospy.logdebug("Received a joint angle target")
        try:
            # Note: changeAngles() and setAngles() are non-blocking functions.
            if (msg.relative==0):
                self.motionProxy.setAngles(list(msg.joint_names), list(msg.joint_angles), msg.speed)
            else:
                self.motionProxy.changeAngles(list(msg.joint_names), list(msg.joint_angles), msg.speed)
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)

    def handleJointStiffness(self, msg):
        rospy.logdebug("Received a joint angle stiffness")
        try:
            self.motionProxy.setStiffnesses(list(msg.name), list(msg.effort))
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)

    def handleStiffnessSrv(self, req):
        try:
            self.motionProxy.stiffnessInterpolation("Body", 1.0, 0.5)
            rospy.loginfo("Body stiffness enabled")
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleStiffnessOffSrv(self, req):
        try:
            self.motionProxy.stiffnessInterpolation("Body", 0.0, 0.5)
            rospy.loginfo("Body stiffness removed")
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None


    def jointTrajectoryGoalMsgToAL(self, goal):
        """Helper, convert action goal msg to Aldebraran-style arrays for NaoQI"""
        names = list(goal.trajectory.joint_names)
#        if goal.trajectory.joint_names == ["Body"]:
#            names = self.motionProxy.getJointNames('Body')

        if len(goal.trajectory.points) == 1 and len(goal.trajectory.points[0].positions) == 1:
            angles = goal.trajectory.points[0].positions[0]
        else:
            # convert angles to naoqi style: outer array contains the joints, inner array the angle values
            angles = list(list(p.positions[i] for p in goal.trajectory.points) for i in range(0,len(goal.trajectory.points[0].positions)))

        #strip 6,7 and last 2 from angles if the pose was for H25 but we're running an H21
        if not isinstance(angles, float) and len(angles) > self.collectionSize["Body"]:
            #TODO: This is dangerous!
            rospy.loginfo("Stripping angles from %d down to %d", len(angles), self.collectionSize["Body"])
            angles.pop(6)
            angles.pop(7)
            angles.pop()
            angles.pop()

        if len(names) > self.collectionSize["Body"]:
            #TODO: This is dangerous!
            rospy.loginfo("Stripping names from %d down to %d", len(names), self.collectionSize["Body"])
            names.pop(6)
            names.pop(7)
            names.pop()
            names.pop()

        times = list(p.time_from_start.to_sec() for p in goal.trajectory.points)
        if len(times) == 1 and len(names) == 1:
            times = times[0]
        if (len(names) > 1):
            times = [times]*len(names)

        return (names, angles, times)


    def executeJointTrajectoryAction(self, goal):
        rospy.loginfo("JointTrajectory action executing");

        names, angles, times = self.jointTrajectoryGoalMsgToAL(goal)

        rospy.logdebug("Received trajectory for joints: %s times: %s", str(names), str(times))
        rospy.logdebug("Trajectory angles: %s", str(angles))

        task_id = None
        running = True
        #Wait for task to complete
        while running and not self.jointTrajectoryServer.is_preempt_requested() and not rospy.is_shutdown():
            #If we haven't started the task already...
            if task_id is None:
                # ...Start it in another thread (thanks to motionProxy.post)
                task_id = self.motionProxy.post.angleInterpolation(names, angles, times, (goal.relative==0))

            #Wait for a bit to complete, otherwise check we can keep running
            running = self.motionProxy.wait(task_id, self.poll_rate)

        # If still running at this point, stop the task
        if running and task_id:
            self.motionProxy.stop( task_id )

        jointTrajectoryResult = JointTrajectoryResult()
        jointTrajectoryResult.goal_position.header.stamp = rospy.Time.now()
        jointTrajectoryResult.goal_position.position = self.motionProxy.getAngles(names, True)
        jointTrajectoryResult.goal_position.name = names

        if not self.checkJointsLen(jointTrajectoryResult.goal_position):
            self.jointTrajectoryServer.set_aborted(jointTrajectoryResult)
            rospy.logerr("JointTrajectory action error in result: sizes mismatch")

        elif running:
            self.jointTrajectoryServer.set_preempted(jointTrajectoryResult)
            rospy.logdebug("JointTrajectory preempted")

        else:
            self.jointTrajectoryServer.set_succeeded(jointTrajectoryResult)
            rospy.loginfo("JointTrajectory action done")


    def executeJointStiffnessAction(self, goal):
        rospy.loginfo("JointStiffness action executing");

        names, angles, times = self.jointTrajectoryGoalMsgToAL(goal)

        rospy.logdebug("Received stiffness trajectory for joints: %s times: %s", str(names), str(times))
        rospy.logdebug("stiffness values: %s", str(angles))

        task_id = None
        running = True
        #Wait for task to complete
        while running and not self.jointStiffnessServer.is_preempt_requested() and not rospy.is_shutdown():
            #If we haven't started the task already...
            if task_id is None:
                # ...Start it in another thread (thanks to motionProxy.post)
                task_id = self.motionProxy.post.stiffnessInterpolation(names, angles, times)

            #Wait for a bit to complete, otherwise check we can keep running
            running = self.motionProxy.wait(task_id, self.poll_rate)

        # If still running at this point, stop the task
        if running and task_id:
            self.motionProxy.stop( task_id )

        jointStiffnessResult = JointTrajectoryResult()
        jointStiffnessResult.goal_position.header.stamp = rospy.Time.now()
        jointStiffnessResult.goal_position.position = self.motionProxy.getStiffnesses(names)
        jointStiffnessResult.goal_position.name = names

        if not self.checkJointsLen(jointStiffnessResult.goal_position):
            self.jointStiffnessServer.set_aborted(jointStiffnessResult)
            rospy.logerr("JointStiffness action error in result: sizes mismatch")

        elif running:
            self.jointStiffnessServer.set_preempted(jointStiffnessResult)
            rospy.logdebug("JointStiffness preempted")

        else:
            self.jointStiffnessServer.set_succeeded(jointStiffnessResult)
            rospy.loginfo("JointStiffness action done")


    def executeJointAnglesWithSpeedAction(self, goal):

        names = list(goal.joint_angles.joint_names)
        angles = list(goal.joint_angles.joint_angles)
        rospy.logdebug("Received JointAnglesWithSpeed for joints: %s angles: %s", str(names), str(angles))

        if goal.joint_angles.relative == 1:
            # TODO: this uses the current angles instead of the angles at the given timestamp
            currentAngles = self.motionProxy.getAngles(names, True)
            angles = list(map(lambda x,y: x+y, angles, currentAngles))

        task_id = None
        running = True
        #Wait for task to complete
        while running and not self.jointAnglesServer.is_preempt_requested() and not rospy.is_shutdown():
            #If we haven't started the task already...
            if task_id is None:
                # ...Start it in another thread (thanks to motionProxy.post)
                task_id = self.motionProxy.post.angleInterpolationWithSpeed(names, angles, goal.joint_angles.speed)

            #Wait for a bit to complete, otherwise check we can keep running
            running = self.motionProxy.wait(task_id, self.poll_rate)

        # If still running at this point, stop the task
        if running and task_id:
            self.motionProxy.stop( task_id )

        jointAnglesWithSpeedResult = JointAnglesWithSpeedResult()
        jointAnglesWithSpeedResult.goal_position.header.stamp = rospy.Time.now()
        jointAnglesWithSpeedResult.goal_position.position = self.motionProxy.getAngles(names, True)
        jointAnglesWithSpeedResult.goal_position.name = names

        if not self.checkJointsLen(jointAnglesWithSpeedResult.goal_position):
            self.jointAnglesServer.set_aborted(jointAnglesWithSpeedResult)
            rospy.logerr("JointAnglesWithSpeed action error in result: sizes mismatch")

        elif running:
            self.jointAnglesServer.set_preempted(jointAnglesWithSpeedResult)
            rospy.logdebug("JointAnglesWithSpeed preempted")

        else:
            self.jointAnglesServer.set_succeeded(jointAnglesWithSpeedResult)
            rospy.loginfo("JointAnglesWithSpeed action done")

    def checkJointsLen(self, goal_position):
        if len(goal_position.name) == 1 and self.collectionSize.has_key(goal_position.name[0]):
            return len(goal_position.position) == self.collectionSize[goal_position.name[0]]
        else:
            return len(goal_position.position) ==  len(goal_position.name)

if __name__ == '__main__':

    controller = NaoController()
    rospy.loginfo("nao_controller running...")
    rospy.spin()

    rospy.loginfo("nao_controller stopped.")
    exit(0)
