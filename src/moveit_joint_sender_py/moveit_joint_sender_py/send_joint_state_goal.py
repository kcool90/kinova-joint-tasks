#!/usr/bin/env python3

import os
import time
import yaml
import argparse
import re

def extract_index(filename):
    match = re.search(r'position(\d+)', filename)
    return int(match.group(1)) if match else -1

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand


ARM_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"


class JointTaskExecutor(Node):
    def __init__(self, task_path):
        super().__init__("joint_task_executor")

        self.task_path = task_path

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/robotiq_gripper_controller/gripper_cmd",
        )

        self.get_logger().info("Waiting for arm controller...")
        self.arm_client.wait_for_server()

        self.get_logger().info("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()

        self.get_logger().info("Controllers ready.")

    def load_yaml(self, filepath):
        with open(filepath, "r") as f:
            return yaml.safe_load(f)

    def send_arm_goal(self, joint_positions, duration=2.0):
        traj = JointTrajectory()
        traj.joint_names = list(joint_positions.keys())

        point = JointTrajectoryPoint()
        point.positions = list(joint_positions.values())
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)

        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def send_gripper_goal(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 50.0

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def execute_task(self):
        files = sorted(
            (f for f in os.listdir(self.task_path) if f.endswith(".yaml")),
            key=extract_index,
        )
        self.get_logger().info(f"Executing task with {len(files)} waypoints")

        for filename in files:
            filepath = os.path.join(self.task_path, filename)
            self.get_logger().info(f"→ {filename}")

            data = self.load_yaml(filepath)

            arm_positions = {}
            gripper_position = None

            for name, pos in zip(data["joint_names"], data["positions"]):
                if name in ARM_JOINTS:
                    arm_positions[name] = pos
                elif name == GRIPPER_JOINT:
                    gripper_position = pos

            if arm_positions:
                if not self.send_arm_goal(arm_positions, duration=6.0):
                    self.get_logger().error("Stopping task due to arm failure")
                    return

            if gripper_position is not None:
                self.send_gripper_goal(gripper_position)

            time.sleep(0.3)

        self.get_logger().info("Task complete.")


def main():
    parser = argparse.ArgumentParser(description="Execute a joint-space task")
    parser.add_argument("task_name", help="Task folder name (e.g. task1)")
    parser.add_argument(
        "--base_path",
        default="/home/pascal/kinova_joint_data",
        help="Base directory containing task folders",
    )
    args = parser.parse_args()

    task_path = os.path.join(args.base_path, args.task_name)
    if not os.path.isdir(task_path):
        raise RuntimeError(f"Task path does not exist: {task_path}")

    rclpy.init()
    node = JointTaskExecutor(task_path)
    node.execute_task()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
