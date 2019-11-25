#!/usr/bin/env python
# coding: utf8

import sys
import rospy
import roslib
import math
import tf
import numpy as np
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

def rot_per_joint(plan, degrees=False):
        np_traj = np.array([p.positions for p in plan.joint_trajectory.points])
        if len(np_traj) == 0:
            raise ValueError
        np_traj_max_per_joint = np_traj.max(axis=0)
        np_traj_min_per_joint = np_traj.min(axis=0)
        ret = abs(np_traj_max_per_joint - np_traj_min_per_joint)
        if degrees:
            ret = [math.degrees(j) for j in ret]
        return ret

def is_crazy_plan(plan, max_rotation_per_joint):
	abs_rot_per_joint = rot_per_joint(plan)
        if (abs_rot_per_joint > max_rotation_per_joint).any():
            return True
        else:
            return False

def movel(pose_xyz,fallback_joint_limit):
        #waypoints = []
	#waypoints.append(group.get_current_pose().pose)
	pose_target = geometry_msgs.msg.Pose()	
	pose_target.position.x = pose_xyz[0]
	pose_target.position.y = pose_xyz[1]
	pose_target.position.z = pose_xyz[2]
	pose_target.orientation.x = -0.5  # waypoints[0].orientation.x 0.5
	pose_target.orientation.y =  0.5  # waypoints[0].orientation.y
	pose_target.orientation.z =  0.5  # waypoints[0].orientation.z -0.5
	pose_target.orientation.w =  0.5  # waypoints[0].orientation.w
        # print pose_target
	group.set_pose_target(pose_target)
	plan1 = group.plan()
	if not is_crazy_plan(plan1, fallback_joint_limit):
		group.execute(plan1, wait=True)
		group.set_start_state_to_current_state()
		rospy.loginfo("arm moved")
	else: print 'sth wrong'


if __name__ == "__main__":
	rospy.init_node('moveit_grasp_app', anonymous=True)
	rospy.loginfo("Starting grasp app")
        listener = tf.TransformListener()
        rate = rospy.Rate(2)
	moveit_commander.roscpp_initialize(sys.argv)
	display_trajectory_publisher = rospy.Publisher(
		                      '/move_group/display_planned_path',
		                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)
	group = moveit_commander.MoveGroupCommander("manipulator")
	rospy.sleep(2)
	group.set_max_acceleration_scaling_factor(0.1)
	group.set_max_velocity_scaling_factor(0.1)
	group.set_planer_id = "RRTkConfigDefault"
	group.set_planning_time(50)
	end_effector_link = group.get_end_effector_link()
	fallback_joint_limits = [math.radians(90)] * 3 + [math.radians(90)] + [math.radians(180)] + [math.radians(350)]
	movel([-0.153563474854, 0.255494091763, 0.287675973599],fallback_joint_limits)
        while not rospy.is_shutdown():
            try:
		current_pose = []
		current_pose.append(group.get_current_pose().pose)
		print(current_pose)
                movel([0, 0.255494091763, 0.287675973599],fallback_joint_limits)
		movel([-0.153563474854, 0.255494091763, 0.287675973599],fallback_joint_limits)	

		rate.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        group.stop()
	moveit_commander.roscpp_shutdown() 
	rospy.loginfo("Stopping grasp app")


