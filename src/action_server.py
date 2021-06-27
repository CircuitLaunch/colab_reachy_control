#!/usr/bin/env python3

import rospy
import actionlib
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from colab_reachy_control.srv import *
from threading import Lock

jointIdDict = {
    'r_shoulder_pitch': 10,
    'r_shoulder_roll': 11,
    'r_arm_yaw': 12,
    'r_elbow_pitch': 13,
    'r_forearm_yaw': 14,
    'r_wrist_pitch': 15,
    'r_wrist_roll': 16,
    'r_gripper': 17,
    'l_shoulder_pitch': 20,
    'l_shoulder_roll': 21,
    'l_arm_yaw': 22,
    'l_elbow_pitch': 23,
    'l_forearm_yaw': 24,
    'l_wrist_pitch': 25,
    'l_wrist_roll': 26,
    'l_gripper': 27
}

rightArmActionServer = None
leftArmActionServer = None

#rPub = rospy.Publisher('action_server/right_arm_trajectory', Trajectory, queue_size = 5)
#lPub = rospy.Publisher('action_server/left_arm_trajectory', Trajectory, queue_size = 5)

def rightArmActionServerCallback(goal: FollowJointTrajectoryGoal):
    actionServerCallback('right', rightArmActionServer, goal)

def leftArmActionServerCallback(goal: FollowJointTrajectoryGoal):
    actionServerCallback('left', leftArmActionServer, goal)

currentPosLock = Lock()
currentPos = {}

def rightJointStateCallback(state: JointState):
    with currentPosLock:
        for i in range(0, len(state.name)):
            currentPos[state.name[i]] = state.position[i]

def leftJointStateCallback(state: JointState):
    with currentPosLock:
        for i in range(0, len(state.name)):
            currentPos[state.name[i]] = state.position[i]

rightJointStateSubscriber = rospy.Subscriber("right_arm_controller/joint_states", JointState, rightJointStateCallback)
leftJointStateSubscriber = rospy.Subscriber("left_arm_controller/joint_states", JointState, leftJointStateCallback)

rightArmActionServer = actionlib.SimpleActionServer("right_arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb = rightArmActionServerCallback, auto_start = False)
leftArmActionServer = actionlib.SimpleActionServer("left_arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb = leftArmActionServerCallback, auto_start = False)

def actionServerCallback(side, actionServer, goal: FollowJointTrajectoryGoal):
    jointNames = goal.trajectory.joint_names

    if len(list(set(jointNames) - set(jointIdDict.keys()))) > 0:
        rospy.logerr(f'{side}_arm action_server received invalid joint names')
        actionServer.set_aborted()
        return

    jointIds = [jointIdDict[name] for name in jointNames]
    waypoints = [point.positions for point in goal.trajectory.points]
    goalTimeTol = goal.goal_time_tolerance if goal.goal_time_tolerance else rospy.Duration(rospy.get_param('~goal_time_tolerance'))

    if len(waypoints) == 0:
        rospy.logerr(f'{side}_arm action_server received empty trajectory')
        actionServer.set_aborted()
        return

    if len(waypoints) < 3:
        if len(waypoints) == 1:
            with currentPosLock:
                currentPoints = [currentPos[name] for name in jointNames]
            waypoints.insert(0, currentPoints)
        if len(waypoints) == 2:
            intermed = [(waypoints[0][i] + waypoints[1][i]) * 0.5 for i in range(0, len(waypoints[0]))]
            waypoints.insert(1, intermed)

    flattenedWaypoints = [position for waypoint in waypoints for position in waypoint]

    trajectoryRequest = TrajectoryRequest()
    trajectoryRequest.dxl_ids = jointIds
    trajectoryRequest.time_from_start = goal.trajectory.points[-1].time_from_start
    trajectoryRequest.goal_time_tolerance = goalTimeTol
    trajectoryRequest.control_points = flattenedWaypoints

    trajectoryService = rospy.ServiceProxy(f'action_server/{side}_arm_trajectory', Trajectory)
    trajectoryService(trajectoryRequest);

    actionServer.set_succeeded()

def main():
    rospy.init_node("reachy_action_server")
    spinRate = rospy.Rate(30)

    rightArmActionServer.start()
    leftArmActionServer.start()

    while not rospy.is_shutdown():
        spinRate.sleep();

if __name__ == "__main__":
    main()
