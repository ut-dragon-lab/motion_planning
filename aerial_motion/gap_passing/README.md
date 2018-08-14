# Compile
1. Moveit:

``` sudo apt-get install ros-{$ROS_DISTRO}-moveit ```

# Usage
## 1. Hydrus (2D transformation): TODO

## 2. Dragon (3D transformation)

### old version 
```
aerial_robot: https://github.com/tongtybj/aerial_robot/tree/dragon_20180628
gap_passing: https://github.com/tongtybj/motion_planning/tree/dragon_20180628
onboard pc: intel euclid with host wifi.
mocap: remote launch in local pc.
```

  **online planning and check the planned path in rviz**

```
$ roslaunch gap_passing dragon_passing_planning.launch play_path_flag:=true load_path_flag:=false replay_flag:=true
```
  or

```
$ roslaunch gap_passing dragon_passing_planning.launch play_path_flag:=true load_path_flag:=true replay_flag:=false
```
  after the solution is found
```
$ rostopic pub -1 /move_start std_msgs/Empty "{}"
```

  2D:
```
roslaunch gap_passing dragon_passing_planning.launch play_path_flag:=true load_path_flag:=false se2:=true horizontal_gap:=true
```

  **visualize the planned path in rviz**
```
$ roslaunch gap_passing dragon_passing_planning.launch play_path_flag:=true load_path_flag:=true replay_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
```
  or

```
$ roslaunch gap_passing dragon_passing_planning.launch play_path_flag:=true load_path_flag:=true replay_flag:=false path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
$ rostopic pub -1 /move_start std_msgs/Empty "{}"
```

  **experiment in real_machine:**
```
$ roslaunch gap_passing dragon_passing_planning.launch play_path_flag:=false load_path_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
```

  **check in simulation (gazebo):**
```
$ roslaunch dragon bringup.launch real_machine:=false simulation:=true headless:=true
$ roslaunch gap_passing dragon_passing_planning.launch play_path_flag:=true load_path_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
```

  **replay rosbag in rviz:**
```
$ roslaunch gap_passing dragon_passing_planning.launch play_path_flag:=true load_path_flag:=true replay_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
```

  **Note**: if you want visualization in the simulation (gazebo) system, please do ```headless:=true``` in the dragon bringup launch, since the gazebo gui and gap_passing motion conflict with each other. In other words, only watch in rviz mode.


