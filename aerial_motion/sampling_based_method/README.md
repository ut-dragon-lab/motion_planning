# Compile
1. Moveit:

``` sudo apt-get install ros-{$ROS_DISTRO}-moveit ```

# Usage

## Only Aerial Transformation: TODO

## Gap Passing

### 1. Hydrus: TODO

### 2. Dragon

### requirements: 
```
onboard pc: intel euclid with host wifi.
mocap: remote launch in local pc.
```

  **online planning and check the planned path in rviz**

```
$ roslaunch sampling_based_method dragon_passing_planning.launch play_path_flag:=true load_path_flag:=false replay_flag:=true
```

  2D:
```
roslaunch sampling_based_method dragon_passing_planning.launch play_path_flag:=true load_path_flag:=false replay_flag:=true se2:=true horizontal_gap:=true
```

  **visualize the planned path in rviz**
```
$ roslaunch sampling_based_method dragon_passing_planning.launch play_path_flag:=true load_path_flag:=true replay_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
```
  or check with true time stamp

```
$ roslaunch sampling_based_method dragon_passing_planning.launch play_path_flag:=true load_path_flag:=true replay_flag:=false path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
$ rostopic pub -1 /move_start std_msgs/Empty "{}"
```

  **experiment in real_machine:**
```
$ roslaunch sampling_based_method dragon_passing_planning.launch play_path_flag:=false load_path_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
```

  **check in simulation (gazebo):**
```
$ roslaunch dragon bringup.launch real_machine:=false simulation:=true
$ roslaunch sampling_based_method dragon_passing_planning.launch play_path_flag:=true load_path_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
```

  **replay rosbag in rviz:**
```
$ roslaunch sampling_based_method dragon_passing_planning.launch play_path_flag:=true load_path_flag:=true replay_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
$ rosbag play xxxx.bag
```

