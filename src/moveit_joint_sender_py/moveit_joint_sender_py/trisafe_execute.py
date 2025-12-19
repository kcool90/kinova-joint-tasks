#!/usr/bin/env python3
"""
TriSaFe-Trans -> ROS2 Kinova task executor

IMPORTANT:
- Use --infer_python to run inference with a python that has torch installed (e.g., your venv python).
"""

from __future__ import annotations

import os
import re
import sys
import time
import yaml
import argparse
import subprocess
from pathlib import Path
from collections import deque, Counter
from typing import Dict, Optional, Tuple, List

import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand


ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"

def run_send_joint_state_goal(task_folder: str, pkg: str, exe: str, debug: bool = False) -> None:
    """
    Runs:
      ros2 run <pkg> <exe> taskN

    task_folder can be "task1" (preferred) or "1" (will be converted to "task1").
    """
    t = (task_folder or "").strip()
    if not t:
        raise RuntimeError("Empty task_folder passed to run_send_joint_state_goal()")

    # If you ever pass "1" instead of "task1", convert it
    if t.isdigit():
        t = f"task{t}"

    cmd = ["ros2", "run", pkg, exe, t]
    if debug:
        print("[robot] CMD =", " ".join(cmd), flush=True)

    subprocess.run(cmd, check=True)



def extract_index(filename: str) -> int:
    m = re.search(r"position(\d+)", filename)
    return int(m.group(1)) if m else -1


def parse_map(map_str: str) -> Dict[int, str]:
    """
    "1:task1,2:task2,3:task4,4:task6,5:task8" -> {1:"task1",...}
    """
    m: Dict[int, str] = {}
    s = (map_str or "").strip()
    if not s:
        return {1: "task1", 2: "task2", 3: "task3", 4: "task4", 5: "task5"}
    for part in s.split(","):
        part = part.strip()
        if not part or ":" not in part:
            continue
        k, v = part.split(":", 1)
        k = k.strip()
        v = v.strip()
        if not k.isdigit():
            continue
        m[int(k)] = v
    return m


def final_task_from_pred_csv(pred_csv: Path) -> Tuple[int, float, Dict[int, int]]:
    """
    Returns: (task_code, action_rate, counts)
      task_code: majority pred_task_code among windows with pred_action==1 and pred_task_code!=0
    """
    df = pd.read_csv(pred_csv)
    if len(df) == 0:
        return 0, 0.0, {}

    if "pred_action" not in df.columns or "pred_task_code" not in df.columns:
        return 0, 0.0, {}

    action_rate = float((df["pred_action"] == 1).mean())

    act = df[(df["pred_action"] == 1) & (df["pred_task_code"] != 0)]
    if len(act) == 0:
        return 0, action_rate, {}

    vc = act["pred_task_code"].value_counts()
    task_code = int(vc.index[0])
    counts = {int(k): int(v) for k, v in vc.to_dict().items()}
    return task_code, action_rate, counts


class JointTaskExecutor(Node):
    def __init__(self, base_path: str):
        super().__init__("trisafe_execute")
        self.base_path = base_path

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

    @staticmethod
    def load_yaml(filepath: str):
        with open(filepath, "r") as f:
            return yaml.safe_load(f)

    def send_arm_goal(self, joint_positions: Dict[str, float], duration: float = 6.0) -> bool:
        # deterministic joint order
        joint_names = [j for j in ARM_JOINTS if j in joint_positions]
        if not joint_names:
            self.get_logger().error("No arm joints found in goal.")
            return False

        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(joint_positions[j]) for j in joint_names]
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

    def send_gripper_goal(self, position: float) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
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

    def execute_task_folder(self, task_folder: str) -> None:
        task_path = os.path.join(self.base_path, task_folder)
        if not os.path.isdir(task_path):
            self.get_logger().error(f"Task path does not exist: {task_path}")
            return

        files = sorted(
            (f for f in os.listdir(task_path) if f.endswith(".yaml")),
            key=extract_index,
        )

        self.get_logger().info(f"Executing {task_folder} with {len(files)} waypoints")

        for filename in files:
            filepath = os.path.join(task_path, filename)
            self.get_logger().info(f"→ {filename}")
            data = self.load_yaml(filepath)

            if not data or "joint_names" not in data or "positions" not in data:
                self.get_logger().error(f"Bad waypoint yaml: {filepath}")
                return

            arm_positions: Dict[str, float] = {}
            gripper_position = None

            for name, pos in zip(data["joint_names"], data["positions"]):
                if name in ARM_JOINTS:
                    arm_positions[name] = float(pos)
                elif name == GRIPPER_JOINT:
                    gripper_position = float(pos)

            if arm_positions:
                if not self.send_arm_goal(arm_positions, duration=6.0):
                    self.get_logger().error("Stopping task due to arm failure")
                    return

            if gripper_position is not None:
                self.send_gripper_goal(gripper_position)

            time.sleep(0.3)

        self.get_logger().info("Task complete.")


def run_inference(
    csv_path: Path,
    ckpt: Path,
    stats: Path,
    out_pred: Path,
    tau: float,
    ts_stats: Optional[Path],
    infer_script: Optional[Path],
    infer_module: Optional[str],
    infer_python: Optional[Path] = None,
    extra_args: Optional[List[str]] = None,
    debug: bool = False,
) -> None:
    """
    Runs inference using infer_python if provided (venv python with torch).
    """
    if extra_args is None:
        extra_args = []

    if infer_script is None and not infer_module:
        raise RuntimeError("Provide either --infer_script or --infer_module.")

    py = str(infer_python) if infer_python is not None else sys.executable

    cmd: List[str] = [py]
    if infer_script is not None:
        cmd += [str(infer_script)]
    else:
        cmd += ["-m", infer_module]

    cmd += [
        "--csv", str(csv_path),
        "--ckpt", str(ckpt),
        "--stats", str(stats),
        "--out", str(out_pred),
        "--tau", str(tau),
        "--pad_short",
    ]

    if ts_stats is not None:
        cmd += ["--ts_stats", str(ts_stats)]

    cmd += list(extra_args)

    if debug:
        print("[infer] PY =", py, flush=True)
        print("[infer] CMD =", " ".join(cmd), flush=True)

    subprocess.run(cmd, check=True)


def safe_shutdown(node: Optional[Node] = None) -> None:
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        # Avoid "rcl_shutdown already called" crash on Ctrl-C
        if node is not None:
            node.get_logger().warn(f"Shutdown warning (safe to ignore): {e}")


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("--csv", required=True)
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--stats", required=True)
    ap.add_argument("--ts_stats", default=None)
    ap.add_argument("--tau", type=float, default=0.60)

    ap.add_argument("--infer_script", default=None)
    ap.add_argument("--infer_module", default=None)

    ap.add_argument("--infer_python", default=None,
                    help="Python interpreter for inference (venv python with torch).")

    ap.add_argument("--base_path", default="/home/pascal/kinova_joint_data")
    ap.add_argument("--map", default="1:task1,2:task2,3:task3,4:task4,5:task5")
    ap.add_argument("--debug", action="store_true")

    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--stable_k", type=int, default=3)
    ap.add_argument("--cooldown", type=float, default=3.0)

    ap.add_argument("--robot_pkg", default="moveit_joint_sender_py",
                help="ROS2 package that contains send_joint_state_goal")
    ap.add_argument("--robot_exec", default="send_joint_state_goal",
                help="Executable name (default: send_joint_state_goal)")


    args, unknown = ap.parse_known_args()

    csv_path = Path(args.csv).expanduser()
    ckpt = Path(args.ckpt).expanduser()
    stats = Path(args.stats).expanduser()
    ts_stats = Path(args.ts_stats).expanduser() if args.ts_stats else None

    infer_script = Path(args.infer_script).expanduser() if args.infer_script else None
    infer_module = args.infer_module

    infer_python = Path(args.infer_python).expanduser() if args.infer_python else None

    if not csv_path.exists():
        raise RuntimeError(f"CSV not found: {csv_path}")
    if not ckpt.exists():
        raise RuntimeError(f"CKPT not found: {ckpt}")
    if not stats.exists():
        raise RuntimeError(f"STATS not found: {stats}")
    if ts_stats is not None and not ts_stats.exists():
        raise RuntimeError(f"TS_STATS not found: {ts_stats}")
    if infer_python is not None and not infer_python.exists():
        raise RuntimeError(f"INFER_PYTHON not found: {infer_python}")

    code_to_task = parse_map(args.map)

    out_dir = csv_path.parent / "_inference_out"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_pred = out_dir / f"{csv_path.stem}_pred.csv"

    rclpy.init()
    node = JointTaskExecutor(args.base_path)

    if args.debug:
        node.get_logger().info(f"[cfg] node sys.executable = {sys.executable}")
        node.get_logger().info(f"[cfg] args.infer_python  = {args.infer_python!r}")
        node.get_logger().info(f"[cfg] resolved infer_py  = {str(infer_python) if infer_python else '(None -> sys.executable)'}")

    hist = deque(maxlen=max(1, int(args.stable_k)))
    last_executed = None
    last_exec_time = 0.0

    def one_cycle():
        nonlocal last_executed, last_exec_time

        try:
            run_inference(
                csv_path=csv_path,
                ckpt=ckpt,
                stats=stats,
                out_pred=out_pred,
                tau=float(args.tau),
                ts_stats=ts_stats,
                infer_script=infer_script,
                infer_module=infer_module,
                infer_python=infer_python,
                extra_args=unknown,
                debug=bool(args.debug),
            )
        except subprocess.CalledProcessError as e:
            node.get_logger().error(f"[infer] failed: {e}")
            hist.clear()
            return

        task_code, action_rate, counts = final_task_from_pred_csv(out_pred)

        if args.debug:
            node.get_logger().info(f"[model] task_code={task_code} action_rate={action_rate:.3f} counts={counts}")

        if task_code == 0:
            hist.clear()
            return

        hist.append(task_code)
        if len(hist) < hist.maxlen:
            return

        c = Counter(hist)
        winner, wcount = c.most_common(1)[0]
        if wcount < hist.maxlen:
            return

        now = time.time()
        if (now - last_exec_time) < float(args.cooldown):
            return

        if last_executed == winner and (now - last_exec_time) < float(args.cooldown) * 2.0:
            return

        task_folder = code_to_task.get(int(winner), None)
        if not task_folder:
            node.get_logger().error(f"No mapping for task_code={winner}. Use --map.")
            return

        node.get_logger().info(f"[EXEC] winner={winner} -> {task_folder} (calling send_joint_state_goal)")
        try:
            run_send_joint_state_goal(
                task_folder=task_folder,
                pkg=args.robot_pkg,
                exe=args.robot_exec,
                debug=bool(args.debug),
            )
        except subprocess.CalledProcessError as e:
            node.get_logger().error(f"[robot] failed: {e}")
            return


        last_executed = winner
        last_exec_time = time.time()
        hist.clear()

    try:
        if not args.loop:
            one_cycle()
        else:
            node.get_logger().info("Loop mode ON (inference -> stable -> execute)")
            while rclpy.ok():
                try:
                    one_cycle()
                    time.sleep(float(args.interval))
                except KeyboardInterrupt:
                    node.get_logger().info("Ctrl-C received, exiting loop.")
                    break
    finally:
        safe_shutdown(node)


if __name__ == "__main__":
    main()
