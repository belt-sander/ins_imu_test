#!/usr/bin/env python3

"""
Brief:	Linear motion plan designed to excite one axis at time for calibration of INS / IMU
Author:	Sander
"""

import rospy
import copy
import moveit_commander
import argparse

from geometry_msgs.msg import Pose

class INSTest:
	def __init__(self):
		rospy.init_node("obsidian_ins_test_node")
		self.args = self.parse_args()

		self.robot = moveit_commander.RobotCommander()
		group_name = "xarm6"
		self.group = moveit_commander.MoveGroupCommander(group_name)

		# We can get the name of the reference frame for this robot:
		planning_frame = self.group.get_planning_frame()
		print("============ Reference frame: %s" % planning_frame)

		# We can also print the name of the end-effector link for this group:
		eef_link = self.group.get_end_effector_link()
		print("============ End effector: %s" % eef_link)

		# We can get a list of all the groups in the robot:
		group_names = self.robot.get_group_names()
		print("============ Robot Groups:", self.robot.get_group_names())
		print("\n")

		self.go_home()
		self.init_pose = self.group.get_current_pose().pose

	def parse_args(self):
		arg_parse = argparse.ArgumentParser(description="Args for robot movement plans")
		arg_parse.add_argument("-n", "--number_of_iters", type=int, required=True, help="Number of plan iterations")
		arg_parse.add_argument("-s", "--sleep", type=float,  default=1.0)
		arg_parse.add_argument("-m", "--mode", default="x", help="Translation mode. <x> or <y>")
		arg_parse.add_argument("-r", "--rotation_test", default=False, type=bool)
		return arg_parse.parse_args()

	def generate_plan(self):
		waypoints = []
		scale = 0.35  # Meters
		wpose = self.init_pose

		if self.args.mode == "x":
			wpose.position.x += scale
			waypoints.append(copy.deepcopy(wpose))
			wpose.position.x -= scale
			waypoints.append(copy.deepcopy(wpose))

		if self.args.mode == "y":
			wpose.position.x += scale
			waypoints.append(copy.deepcopy(wpose))
			wpose.position.y -= scale
			waypoints.append(copy.deepcopy(wpose))
			wpose.position.y += scale
			waypoints.append(copy.deepcopy(wpose))
			wpose.position.x -= scale
			waypoints.append(copy.deepcopy(wpose))

#		IGNORE Z TRANSLATION FOR NOW
#		wpose.position.z += scale
#		waypoints.append(copy.deepcopy(wpose))
#		wpose.position.z -= scale
#		waypoints.append(copy.deepcopy(wpose))

		(plan, fraction) = self.group.compute_cartesian_path(
										   waypoints,   # waypoints to follow
										   0.01,        # eef_step
										   0.0)         # jump_threshold
		return plan

	def run_plan(self):
		p = self.generate_plan()
		self.group.execute(p, wait=True)

	def go_home(self):
		joint_goal = self.group.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = 0
		joint_goal[2] = 0
		joint_goal[3] = 0
		joint_goal[4] = 0
		joint_goal[5] = 0
		self.group.go(joint_goal, wait=True)
		self.group.stop()

	def go_to_start_pose(self):
		p = Pose()
		p.orientation.x = 0.8660254
		p.orientation.y = 0.5 		
		p.orientation.z = 0.0
		p.orientation.w = 0.0
		p.position.x = self.init_pose.position.x
		p.position.y = self.init_pose.position.y
		p.position.z = self.init_pose.position.z

		self.group.set_pose_target(p)
		self.group.go(wait=True)
		self.init_pose = self.group.get_current_pose().pose

def main():
	ins = INSTest()

	if ins.args.rotation_test:
		ins.go_to_start_pose()

	for i in range(ins.args.number_of_iters):
		ins.run_plan()
		print("INS / IMU plan", i, "complete")
		rospy.sleep(ins.args.sleep)

	ins.go_home()
	print("\nDone!\n")

if __name__ == "__main__":
	main()
