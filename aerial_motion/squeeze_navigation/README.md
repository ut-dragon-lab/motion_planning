# Small opening squeezing by transformation

## 1. Hydrus: TODO

## 2. Dragon

### sampling based methods (e.g., RRT*): TODO

#### - do online planning and check the planning discrete path
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=false discrete_path_debug_flag:=true  path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
$ rostopic pub -1 /dragon/plan_start std_msgs/Empty "{}"
```

#### - load the planned path from file and check the discrete path
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=true discrete_path_debug_flag:=true  path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
$ rostopic pub -1 /dragon/plan_start std_msgs/Empty "{}"
```

#### - load the planned path from file and check the continuous path 
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=true discrete_path_debug_flag:=true  path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
$ rostopic pub -1 /dragon/plan_start std_msgs/Empty "{}"
$ rostopic pub -1 /dragon/move_start std_msgs/Empty "{}"
```

#### - do path tracking from the loaded path from

  **for simualtion**:
  ```
  $ roslaunch dragon bringup.launch real_machine:=false simulation:=true headless:=false
  $ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=true discrete_path_debug_flag:=true   path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt
  ```

  **for real machine**:
  ```
  $ roslaunch dragon bringup.launch
  $ roslaunch squeeze_navigation dragon_passing_planning.launch load_path_flag:=true discrete_path_debug_flag:=true  path_file_name:=dragon_planning_log_new_vertial_gap_0p5_ceil_1p4_with_side_wall3_best.txt headless:=true
  ```

  **common commands**:
  ```
  $ rostopic pub -1 /dragon/plan_start std_msgs/Empty "{}"
  $ rostopic pub -1 /dragon/adjust_robot_initial_state std_msgs/Empty "{}"
  $ rostopic pub -1 /dragon/move_start std_msgs/Empty "{}"
  ```
  **return motion**: `$ rostopic pub -1 /return std_msgs/Empty "{}"`

### differential kinematics

#### - do online planning and check the planning discrete path
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch  discrete_path_debug_flag:=true
$ rostopic pub -1 /dragon/plan_start std_msgs/Empty "{}"
$ rostopic pub -1 /dragon/move_start std_msgs/Empty "{}"
```

#### - do online planning and check the planning continuous path
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch start_squeeze_path_from_real_state:=false
$ rostopic pub -1 /dragon/plan_start std_msgs/Empty "{}"
$ rostopic pub -1 /dragon/move_start std_msgs/Empty "{}"
```

#### - online planing and do path tracking

  **1. for simualtion**:
  ```
  $ roslaunch squeeze_navigation dragon_bringup.launch headless:=false simulation:=true real_machine:=false
  $ roslaunch squeeze_navigation dragon_passing_planning.launch start_squeeze_path_from_real_state:=false
  ```

  **2. for real machine**:
  ```
  $ roslaunch squeeze_navigation dragon_bringup.launch
  $ roslaunch squeeze_navigation dragon_passing_planning.launch start_squeeze_path_from_real_state:=false headless:=true
  ```

  **common commands**:
  ```
  $ rostopic pub -1 /dragon/plan_start std_msgs/Empty "{}"
  $ rostopic pub -1 /dragon/adjust_robot_initial_state std_msgs/Empty "{}"
  $ rostopic pub -1 /dragon/move_start std_msgs/Empty "{}"
  ```

### important parameters:

#### post-process for discrete path
- [**low pass filter**](https://github.com/tongtybj/motion_planning/blob/master/aerial_motion/squeeze_navigation/src/squeeze_navigation.cpp#L166-L230): do the low pass filtering after the discrete path planning, which is necessary for [differential kinemaitcs](https://github.com/tongtybj/motion_planning/blob/master/aerial_motion/squeeze_navigation/config/differential_kinematics/dragon_quad.yaml#L8-L10).
- [**resampling**](https://github.com/tongtybj/motion_planning/blob/master/aerial_motion/squeeze_navigation/src/squeeze_navigation.cpp#L228-L323): **under development**, current implementation is based on the average translational motion, current is no use for both [methods](https://github.com/tongtybj/motion_planning/blob/master/aerial_motion/squeeze_navigation/config/differential_kinematics/dragon_quad.yaml#L12-L13).