# ur10_ros1_ros2
instruction for running ur10 robot


## ROS1

roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=123.124.125.xx

rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller


## ROS2

ros2 launch ur_bringup ur_control.launch.py ur_type:=ur10 robot_ip:=123.124.125.12 use_fake_hardware:=false launch_rviz:=false

os2 launch ur_bringup ur_moveit.launch.py ur_type:=ur10 robot_ip:=123.124.125.12 use_fake_hardware:=false launch_rviz:=true
