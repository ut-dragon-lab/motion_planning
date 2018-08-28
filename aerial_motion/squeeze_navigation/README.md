# Usage for Gap Passing

### 1. Hydrus: TODO

### 2. Dragon


#### online planning and check the planned path in rviz

** sampling_base_methods** :
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=false  discrete_path_debug_flag:=true
```

** differential kinematics **:
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=false  discrete_path_debug_flag:=true discrete_path_search_method_type:=1
```

<span style="font-size: 150%> **Note**: after this, all commands are explained for the case of **sampling_base_methods**,  plase add option `discrete_path_search_method_type:=1` if using **differential kinematics** </span>


#### visualize the planned path in rviz

** discrete path **:
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt  discrete_path_debug_flag:=true
$ rostopic pub -1 /plan_start std_msgs/Empty "{}"
```

** continous path **:
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt  discrete_path_debug_flag:=true
$ rostopic pub -1 /plan_start std_msgs/Empty "{}"
$ rostopic pub -1 /move_start std_msgs/Empty "{}"
```

#### experiment in real_machine:

**requirements**: 
-onboard pc: intel euclid with host wifi.
-mocap: remote launch in local pc.

```
$ roslaunch dragon bringup.launch
$ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt  headless:=true
```

** for sampling_base methods: **
1. execte following command (path loading from file) before takeoff:
```
$ rostopic pub -1 /plan_start std_msgs/Empty "{}"
```
2. execte following commands after hovering:
```
$ rostopic pub -1 /adjust_robot_initial_state std_msgs/Empty "{}"
$ rostopic pub -1 /move_start std_msgs/Empty "{}"
```


#### check in simulation (gazebo):
```
$ roslaunch dragon bringup.launch real_machine:=false simulation:=true
$ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
```

#### replay rosbag in rviz:
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=true path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
$ rosbag play xxxx.bag
```

