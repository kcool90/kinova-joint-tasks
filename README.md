# kinova-joint-tasks
This repo runs **pre-recorded joint waypoint tasks** on a Kinova Gen3 (6DOF) + Robotiq 2F-85 by sending:
- `FollowJointTrajectory` to `/joint_trajectory_controller/follow_joint_trajectory`
- `GripperCommand` to `/robotiq_gripper_controller/gripper_cmd`
  
-----------------------------------------------

  ## Prerequisites
- ROS 2 installed (same distro you built Kortex for) (ours uses Jazzy)
- Kinova Kortex ROS2 packages installed and working
- Robot reachable on the network (example: `robot_ip:=192.168.1.10`)

-----------------------------------------------

Step 1: **Download "kinova_joint_data" folder from main branch:**

Place the `kinova_joint_data/` folder somewhere. Default expected location is:

- `~/kinova_joint_data/task1`
- `~/kinova_joint_data/task2`
- ...

If you want it elsewhere, set:

```bash
export KINOVA_JOINT_DATA=/path/to/kinova_joint_data
```

Step 2: **Build the workspace:**
```bash
cd workspace/ros2_kortex_ws/
colcon build --symlink-install
```

Step 3: **Open two terminals and source the workspace:** 
```bash
cd workspace/ros2_kortex_ws/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Step 4: **Bring up the robot:**
```bash
ros2 launch kortex_bringup gen3.launch.py   dof:=6   gripper:=robotiq_2f_85   use_fake_hardware:=false   fake_sensor_commands:=false   robot_ip:=192.168.1.10   launch_rviz:=true
```
Change use_fake_hardware and fake_senser_commands to true if using simulation. 


Step 5(a): **Run a task manually to test that it is working:**
```bash
ros2 run moveit_joint_sender_py send_joint_state_goal task1
```
Replace "task1" with any available task folder from kinova_joint_data, e.g., "task2", "task4", "task6", or "task8".

You should notice this command running a task on the robot arm. 

Step 5(b): **Run a task from human sensor data:**
```bash
ros2 run moveit_joint_sender_py trisafe_execute   --infer_python "/home/pascal/workspace/ros2_kortex_ws/.venv/bin/python"   --csv "/home/pascal/Downloads/Tipu1/Sub-16/066_T116_synchronized_corrected_icml_consensus_labels.csv"   --ckpt "/home/pascal/Downloads/Tipu1/best_model_allsubjects_fold1.pt"   --stats "/home/pascal/Downloads/Tipu1/stats_allsubjects_fold1.json"   --tau 0.60   --infer_script "/home/pascal/Downloads/Tipu1/infer_single_trisafe_phase4match.py"   --map "1:task1,2:task2,3:task4,4:task6,5:task8"   --robot_pkg moveit_joint_sender_py   --robot_exec send_joint_state_goal   --loop --interval 1.0 --stable_k 3 --cooldown 3.0 --debug
```
Change the path to your path.
