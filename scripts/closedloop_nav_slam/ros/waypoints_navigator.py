#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file nav_slam_test.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-07-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""


"""
Functions
1. Send waypoints.
2. Check failures, bumping or wheel drop.
3. Reset gazebo model, turtlebot pose.
4. Reset odometry.
5. Reset costmap.
"""

import argparse
import os
import sys
from pathlib import Path

import actionlib
import move_base_msgs.msg as move_base_msgs
import numpy as np
import rospkg
import rospy
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from kobuki_msgs.msg import BumperEvent, ButtonEvent, WheelDropEvent
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Header


class NavSlamTest:
    def __init__(self, args):
        # Init ROS.
        rospy.init_node("waypoints_navigator", anonymous=True, disable_signals=True)

        # Init ros pack.
        self._rospack = rospkg.RosPack()
        self._pkg_dir = Path(self._rospack.get_path("closedloop_nav_slam"))
        self._wpts_prefix = self._pkg_dir / "configs/path" / args.env

        # Extract args values.
        self._robot_init_pose = args.robot_init_pose
        self._loops = args.loops
        self._reset = args.reset
        self._idle_time = args.idle_time
        self._output_dir = args.output_dir
        self._test_type = args.test_type

        # Other parameters.
        self._wait_for_resetting = True
        self._stop = False
        self._task_failed = False
        self._goal_generator = None
        self._button_pressed = False
        self._goals = []
        self._planned_wpts = []
        self._actual_path = []
        self._gt_odom = None
        self._gt_odoms = []
        self._et_odoms = []
        self._gt_poses = []
        self._et_poses = []
        self._robot_odom = None
        self._robot_odoms = []

        # ROS subscribers.
        rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.__buttonEventCallback)
        rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, self.__wheelDropEventCallback)
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.__bumperEventCallback)
        rospy.Subscriber("/gt_odom", Odometry, self.__groundTruthOdometryCallback)
        rospy.Subscriber("/et_odom", Odometry, self.__estimatedOdometryCallback)
        rospy.Subscriber("/robot_odom", Odometry, self.__robotOdometryCallback)
        rospy.Subscriber("/gt_pose", PoseWithCovarianceStamped, self.__groundTruthPoseCallback)
        rospy.Subscriber("/et_pose", PoseWithCovarianceStamped, self.__estimatedPoseCallback)

        # ROS publisher.
        self._odom_reset_pub = rospy.Publisher(
            "/mobile_base/commands/reset_odometry", std_msgs.Empty, queue_size=1, latch=True
        )
        self._init_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self._nav_path_pub = rospy.Publisher("/visited_waypoints", PathMsg, queue_size=1)
        self._pose_array_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=1)

        # ROS service.
        self._clear_costmap_srv = None
        self._reset_robot_model_state_srv = None
        self._reset_slam_toolbox_srv = None

        # Setup goals.
        self.__setupGoals(args.path_file)

        # Reset a few.
        if "gazebo" == self._test_type:
            self.__resetRobotModelState()
        self.__resetOdom()
        self._wait_for_resetting = False

        # Hook-up with move_base
        self._client = actionlib.SimpleActionClient("move_base", move_base_msgs.MoveBaseAction)
        rospy.loginfo("Waiting for server ...")
        self._client.wait_for_server()
        rospy.loginfo("Done (action server)!")

        # Start navigation.
        self.__navigate()

        rospy.spin()

        self.saveToFile()

    def __buttonEventCallback(self, msg):
        if msg.state == ButtonEvent.PRESSED and msg.button == ButtonEvent.Button1:
            rospy.loginfo("Reset request received.")
            self._stop = True
            self.__reset()
        elif msg.state == ButtonEvent.PRESSED and msg.button == ButtonEvent.Button0:
            rospy.loginfo("Trigger the robot to move.")
            self._stop = False
            self._button_pressed = True

    def __wheelDropEventCallback(self, msg):
        if msg.state == WheelDropEvent.DROPPED:
            rospy.loginfo("Stop request received.")
            self._stop = True
            self._client.cancel_all_goals()

    def __bumperEventCallback(self, msg):
        rospy.loginfo("Bumper triggered.")
        self._stop = True
        self._client.cancel_all_goals()
        self._task_failed = True

    def __robotOdometryCallback(self, msg):
        if self._wait_for_resetting:
            self._robot_odoms.clear()
            return
        self._robot_odom = msg
        self._robot_odoms.append(self.__convertNavOdomMsgToArray(msg))

    def __groundTruthOdometryCallback(self, msg):
        if self._wait_for_resetting:
            self._gt_odoms.clear()
            return
        self._gt_odom = msg
        self._gt_odoms.append(self.__convertNavOdomMsgToArray(msg))

    def __estimatedOdometryCallback(self, msg):
        if self._wait_for_resetting:
            self._et_odoms.clear()
            return
        self._et_odoms.append(self.__convertNavOdomMsgToArray(msg))

    def __groundTruthPoseCallback(self, msg):
        if self._wait_for_resetting:
            self._gt_poses.clear()
            return
        self._gt_poses.append(self.__convertStampedPoseMsgToArray(msg))

    def __estimatedPoseCallback(self, msg):
        if self._wait_for_resetting:
            self._et_poses.clear()
            return
        self._et_poses.append(self.__convertStampedPoseMsgToArray(msg))

    def __setupGoals(self, path_file):
        self._planned_wpts = self.__readGoals(path_file)
        for p in self._planned_wpts:
            goal = Pose()
            goal.position.x = p[0]
            goal.position.y = p[1]
            goal.orientation.w = np.cos(p[2] / 2.0)
            goal.orientation.z = np.sin(p[2] / 2.0)
            self._goals.append(goal)

    def __readGoals(self, path_file):
        waypoints = np.loadtxt(self._wpts_prefix / "waypoints.txt")
        path = np.loadtxt(self._wpts_prefix / path_file, dtype=int)
        xys = waypoints[path, 1:]
        goals = []  # [[xys[0, 0] - offset[0], xys[0, 1] - offset[1], 0.0]]
        for i in range(1, len(xys)):  # skip start pos
            p0 = xys[i - 1, :]
            p1 = xys[i, :]
            if i + 1 < len(xys):
                p2 = xys[i + 1, :]
            # @TODO(yanwei) Either to compute a new theta or use the default one.
            # theta = np.arctan2(p2[1] - p0[1], p2[0] - p0[0])
            theta = xys[i, 2]
            goals.append([xys[i, 0], xys[i, 1], theta])
        return goals

    def __publish_waypoints(self, goals):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()
        for goal in goals:
            pose_array.poses.append(goal)
        self._pose_array_pub.publish(pose_array)

    def __navigate(self):
        while not rospy.is_shutdown():
            if self._stop or not self._button_pressed:
                rospy.loginfo_throttle(60, "WptNavigator: waiting for CMD to start the robot.")
                rospy.sleep(0.2)
                continue
            try:
                self._actual_path = PathMsg(header=Header(frame_id="/actual"))
                success = True
                for loop_count in range(self._loops):
                    rospy.loginfo(f"----- Loop: {loop_count} -----")
                    self.__publish_waypoints(self._goals)
                    for goal in self._goals:
                        rospy.loginfo(f"goal: \n {goal}")
                        success = self.__navigateToGoal(goal_pose=goal)
                        if not success:
                            rospy.loginfo(f"Failed to reach goal: {goal}\n Mission Failed.")
                            break
                        rospy.sleep(self._idle_time)
                        self._actual_path.poses.append(PoseStamped(Header(stamp=self._robot_odom.header.stamp), goal))
                    if not success:
                        break
                    rospy.loginfo(f"Sequencing finished: {loop_count}.")
                rospy.loginfo("Publish planned path with timestamp.")
                self._nav_path_pub.publish(self._actual_path)
                rospy.loginfo("Done.")
                self._stop = True
                self._button_pressed = False
                self._client.cancel_all_goals()
            except Exception as e:
                rospy.loginfo(e)
            break

    def __navigateToGoal(self, goal_pose):
        # Create the goal point
        goal = move_base_msgs.MoveBaseGoal()
        goal.target_pose.pose = goal_pose
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"

        # Send the goal!
        rospy.loginfo("Sending goal.")
        self._client.send_goal(goal)
        rospy.loginfo("Waiting for result ...")

        r = rospy.Rate(5)

        # start_time = rospy.Time.now()

        keep_waiting = True
        while keep_waiting and not rospy.is_shutdown():
            state = self._client.get_state()
            # print "State: " + str(state)
            if state is not GoalStatus.ACTIVE and state is not GoalStatus.PENDING:
                keep_waiting = False
            else:
                r.sleep()

        state = self._client.get_state()
        return state == GoalStatus.SUCCEEDED

    def __getNextGoal(self):
        for goal in self._goals:
            rospy.loginfo(goal)
            yield goal

    def __reset(self):
        self._wait_for_resetting = True
        self.__resetGoals()
        rospy.sleep(0.2)
        if "gazebo" == self._test_type:
            self.__resetRobotModelState()
            rospy.sleep(0.2)
        self.__resetCostmaps()
        rospy.sleep(0.2)
        self.__resetOdom()
        rospy.sleep(0.2)
        self._wait_for_resetting = False

    def __resetCostmaps(self):
        rospy.loginfo("Reset costmaps ...")
        if self._clear_costmap_srv is None:
            rospy.wait_for_service("/move_base/clear_costmaps")
            self._clear_costmap_srv = rospy.ServiceProxy("/move_base/clear_costmaps", std_srvs.Empty)
        self._clear_costmap_srv()
        rospy.loginfo("Done (reset costmaps).")

    def __resetOdom(self):
        rospy.loginfo("Reset odom ...")
        self._odom_reset_pub.publish(std_msgs.Empty())
        rospy.loginfo("Done (reset odom).")

    def __resetGoals(self):
        rospy.loginfo("Reset goals ... ")
        self._client.cancel_all_goals()
        self._goal_generator = self.__getNextGoal()
        rospy.loginfo("Done (reset goals).")

    def __resetRobotModelState(self):
        rospy.loginfo("Reset robot model state in gazebo ... ")
        if self._reset_robot_model_state_srv is None:
            rospy.wait_for_service("/gazebo/set_model_state")
            self._reset_robot_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        pose = Pose()
        pose.position.x = self._robot_init_pose[0]
        pose.position.y = self._robot_init_pose[1]
        pose.position.z = self._robot_init_pose[2]
        pose.orientation.w = np.cos(self._robot_init_pose[3] / 2.0)
        pose.orientation.z = np.sin(self._robot_init_pose[3] / 2.0)
        state = ModelState(model_name="mobile_base", pose=pose)
        response = self._reset_robot_model_state_srv(state)
        assert response
        rospy.loginfo("Done (reset gazebo state).")

    def __resetSlamToolbox(self):
        rospy.loginfo("Reset slam ... ")
        if self._reset_slam_toolbox_srv is None:
            rospy.wait_for_service("/slam_toolbox/clear_localization_buffer")
            self._reset_slam_toolbox_srv = rospy.ServiceProxy("/slam_toolbox/clear_localization_buffer", std_srvs.Empty)
        self._reset_slam_toolbox_srv()

        init_pose = PoseWithCovarianceStamped()
        self._init_pose_pub.publish(init_pose)
        rospy.loginfo("Done (reset slam).")

    def __convertNavOdomMsgToArray(self, msg) -> list:
        return [
            msg.header.stamp.to_sec(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]

    def __convertStampedPoseMsgToArray(self, msg) -> list:
        return [
            msg.header.stamp.to_sec(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]

    def __convertPathMsgToArray(self, msg) -> list:
        stamped_wpts = []
        for p in msg.poses:
            stamped_wpts.append(
                [
                    p.header.stamp.to_sec(),
                    p.pose.position.x,
                    p.pose.position.y,
                    2.0 * np.arctan2(p.pose.orientation.z, p.pose.orientation.w),
                ]
            )
        return stamped_wpts

    def saveToFile(self):
        if "" == self._output_dir:
            rospy.logwarn("No directory is specified, save nothing!")
            return
        odom_file_header = "timestamp tx ty tz qx qy qz qw vx vy vz wx wy wz"
        pose_file_header = "timestamp tx ty tz qx qy qz qw"

        # Start saving.
        prefix_path = Path(self._output_dir)
        if len(self._gt_poses) > 0:
            np.savetxt(prefix_path / "gt_slam_poses.txt", self._gt_poses, fmt="%.6f", header=pose_file_header)
            rospy.loginfo("Saved gt slam poses.")
        if len(self._et_poses) > 0:
            np.savetxt(prefix_path / "est_slam_poses.txt", self._et_poses, fmt="%.6f", header=pose_file_header)
            rospy.loginfo("Saved est slam poses.")
        if len(self._gt_odoms) > 0:
            np.savetxt(prefix_path / "act_odoms.txt", self._gt_odoms, fmt="%.6f", header=odom_file_header)
            np.savetxt(
                prefix_path / "act_poses.txt", np.array(self._gt_odoms)[:, :8], fmt="%.6f", header=pose_file_header
            )
            rospy.loginfo("Saved act(gt) odoms.")
        if len(self._et_odoms) > 0:
            np.savetxt(prefix_path / "est_odoms.txt", self._et_odoms, fmt="%.6f", header=odom_file_header)
            np.savetxt(
                prefix_path / "est_poses.txt", np.array(self._et_odoms)[:, :8], fmt="%.6f", header=pose_file_header
            )
            rospy.loginfo("Saved est odoms.")
        if len(self._robot_odoms) > 0:
            np.savetxt(prefix_path / "robot_odoms.txt", self._robot_odoms, fmt="%.6f", header=odom_file_header)
            rospy.loginfo("Saved robot odoms.")
        stamped_wpts = self.__convertPathMsgToArray(self._actual_path)
        if len(stamped_wpts) > 0:
            np.savetxt(prefix_path / "visited_waypoints.txt", stamped_wpts, fmt="%.6f", header="timestamp x y theta")
            rospy.loginfo("Saved visited waypoints.")
        if len(self._planned_wpts) > 0:
            np.savetxt(prefix_path / "planned_waypoints.txt", self._planned_wpts, fmt="%.6f", header="x y theta")
            rospy.loginfo("Saved planned waypoints.")
        rospy.loginfo("Saving Done!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # parser.add_argument("--mode", dest="mode", default="localization", help="mode: localization|mapping")
    parser.add_argument("--env", dest="env", default="tsrb", help="environment (tsrb | classroom)")
    parser.add_argument("--test_type", dest="test_type", default="gazebo", help="test_type (gazebo | realworld)")
    parser.add_argument("--path_file", dest="path_file", default="path0.txt", help="path")
    parser.add_argument("--loops", default="1", type=int, help="number of loops (repeatability)")
    parser.add_argument("--reset", default=False, action="store_true")
    parser.add_argument(
        "--robot_init_pose",
        nargs=4,
        default=[34.0, -6.0, 0.0, 0.0],
        help="robot init pose: [x, y, z, theta]",
        type=float,
    )
    parser.add_argument("--idle_time", default="1.0", type=float, help="idle time in seconds at each waypoint")
    parser.add_argument("--output_dir", nargs="?", default="", type=str, help="directory to save stats data")

    args, unknown = parser.parse_known_args()

    print(args)

    try:
        nst = NavSlamTest(args)
    except rospy.ROSInterruptException:
        rospy.loginfo("exception caught !!!")
