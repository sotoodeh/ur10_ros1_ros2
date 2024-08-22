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

to launch the cameras

- ros2 launch zed_wrapper zed2.launch.py camera_name:=zedB

to record bags run the bash file

to copy from remote PC into the current directory:

- scp -r remote_pc@xx.xx.xx.xx:"/path/to/directory" .

to merg the rosbags to gether:

- ros2 bag convert -i rosbag2_2024_02_08-16_13_36 -i rosbag2_2024_02_08-16_14_01 -i rosbag2_2024_02_08-16_15_16 -o output_bags.yaml

- ros2 topic echo /zed2/zed_node2/body_trk/skeletons | grep sec

to export the images into a directory:

- https://github.com/MapIV/ros2_bag_to_image
- ros2 launch ros2_bag_to_image bag_to_image.xml input/path:=/PATH_TO/input_bag/ input/topics:="['camera1/image_rawcompressed', 'camera2/image_raw']" output/path:=/images
- ros2 launch ros2_bag_to_image bag_to_image.xml input/path:=merged_bag3 input/topics:="['/cam2/zed_node/stereo_raw/image_raw_color','/cam3/zed_node/stereo_raw/image_raw_color', '/cam8/zed_node/stereo_raw/image_raw_color']" output/path:=images


## Universal Robots simulator for e-Series

- newgrp docker
- docker pull universalrobots/ursim_e-series
- docker run --rm -it universalrobots/ursim_e-series

## Usage with official UR simulator (UR driver: https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver)

- ros2 run ur_client_library start_ursim.sh -m ur10e
- ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.56.101 launch_rviz:=true
- You can view the polyscope GUI by opening http://192.168.56.101:6080/vnc.html 
- run URCap on the URSim
- ros2 launch ur_bringup ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.56.101 launch_rviz:=false
- ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e robot_ip:=192.168.56.101 launch_rviz:=true

## ROS2 controllers

- https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers 
