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

Step 1: Download "kinova_joint_data" folder from main branch.
Place the `kinova_joint_data/` folder somewhere. Default expected location is:

- `~/kinova_joint_data/task1`
- `~/kinova_joint_data/task2`
- ...

If you want it elsewhere, set:

```bash
export KINOVA_JOINT_DATA=/path/to/kinova_joint_data


Step 2: Build the workspace:

cd workspace/ros2_kortex_ws/

colcon build --symlink-install


Step 3: Open two terminals with the workspace root and source them both: 

cd workspace/ros2_kortex_ws/

source install/setup.bash


Step 4: Run this command in terminal 1:

ros2 launch kortex_bringup gen3.launch.py   dof:=6   gripper:=robotiq_2f_85   use_fake_hardware:=false   fake_sensor_commands:=false   robot_ip:=192.168.1.10   launch_rviz:=true

Change use_fake_hardware and fake_senser_commands to true if using simulation. 


Step 5: Run this command in terminal 2:

ros2 run moveit_joint_sender_py send_joint_state_goal task1

Replace "task1" with any available task folder, e.g., "task2", "task4", "task6", or "task8".

You should notice this command running a task on the robot arm. 
