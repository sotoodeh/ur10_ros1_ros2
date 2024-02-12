# ur10_ros1_ros2
instruction for running ur10 robot


## ROS1

mkdir -p catkin_ws/src && cd catkin_ws

git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

rosdep install --from-paths src --ignore-src -y

catkin_make

source devel/setup.bash

roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=123.124.125.xx

rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller


## ROS2

mkdir -p ur_ws/src && cd ur_ws

git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver

rosdep install --ignore-src --from-paths src -y -r

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

ros2 launch ur_bringup ur_control.launch.py ur_type:=ur10 robot_ip:=123.124.125.xx use_fake_hardware:=false launch_rviz:=false

ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur10 robot_ip:=123.124.125.xx use_fake_hardware:=false launch_rviz:=true





## stereo network

to copy from remote PC into the current directory:

- scp -r remote_pc@xx.xx.xx.xx:"/path/to/directory" .

to merg the rosbags to gether:

- ros2 bag convert -i rosbag2_2024_02_08-16_13_36 -i rosbag2_2024_02_08-16_14_01 -i rosbag2_2024_02_08-16_15_16 -o output_bags.yaml

- ros2 launch zed_wrapper zed2.launch.py camera_name:=zedB



