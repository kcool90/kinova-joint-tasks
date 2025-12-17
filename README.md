# kinova-joint-tasks

Step 1: Run this command in 3 different terminals:
cd workspace/ros2_kortex_ws/

Step 2: Run this command in those 3 terminals: 
source install/setup.bash

Step 3: Run this command in terminal 1:
ros2 launch kortex_bringup gen3.launch.py   dof:=6   gripper:=robotiq_2f_85   use_fake_hardware:=false   fake_sensor_commands:=false   robot_ip:=192.168.1.10   launch_rviz:=true

Step 4: Run this command in terminal 2:
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config move_group.launch.py   robot_ip:=192.168.1.10   use_fake_hardware:=false   use_sim_time:=false

Step 5: Run this command in terminal 3:
ros2 run moveit_joint_sender_py send_joint_state_goal task1
where "task1" can be replaced by "task2", "task4", "task6", or "task8".

