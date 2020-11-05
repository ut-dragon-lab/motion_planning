# Opening squeezing by transformation (+ Flap manipulation by end-effector)

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

  **for simualtion**:
  ```
  $ roslaunch squeeze_navigation dragon_bringup.launch headless:=false simulation:=true real_machine:=false
  $ roslaunch squeeze_navigation dragon_passing_planning.launch start_squeeze_path_from_real_state:=false
  ```

  **for real machine**:
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

### other important parameters:
- [**low pass filter**](https://github.com/tongtybj/motion_planning/blob/master/aerial_motion/squeeze_navigation/src/squeeze_navigation.cpp#L166-L230): do the low pass filtering after the discrete path planning, which is necessary for [differential kinemaitcs](https://github.com/tongtybj/motion_planning/blob/master/aerial_motion/squeeze_navigation/config/differential_kinematics/dragon_quad.yaml#L8-L10).
- [**resampling**](https://github.com/tongtybj/motion_planning/blob/master/aerial_motion/squeeze_navigation/src/squeeze_navigation.cpp#L228-L323): **under development**, current implementation is based on the average translational motion, current is no use for both [methods](https://github.com/tongtybj/motion_planning/blob/master/aerial_motion/squeeze_navigation/config/differential_kinematics/dragon_quad.yaml#L12-L13).

### replay rosbag in rviz:
```
$ roslaunch squeeze_navigation dragon_passing_planning.launch replay:=true
$ rosbag play xxxx.bag /dragon/plan_start:=/dragon/plan_start_temp
```
**note**: the ros topic `=/dragon/plan_start_temp` should be renamed to avoid the replaning

### plot in rqt_plot from rosbag

**note**: use the modifed rqt_plot from [here](https://github.com/tongtybj/rqt_plot/tree/correct_timestamp) (branch: correct_timestamp)

1. path position error
  ```
  $ rqt_plot /dragon/debug/pose/pid/x/err_p /dragon/debug/pose/pid/y/err_p /dragon/debug/pose/pid/z/err_p
  ```

2. path attitude error
  ```
  $ rqt_plot /dragon/uav/full_state/states[6]/state[0]/x /dragon/uav/full_state/states[7]/state[0]/x /dragon/debug/pose/pid/yaw/err_p
  ```

3. position desired/current trajectory plot
   ```
    $ rqt_plot /dragon/uav/nav/target_pos_x /dragon/uav/nav/target_pos_y /dragon/uav/nav/target_pos_z /dragon/uav/cog/odom/pose/pose/position/x /dragon/uav/cog/odom/pose/pose/position/y /dragon/uav/cog/odom/pose/pose/position/z
   ```

4. attitude desired/current trajectory plot
   ```
   $ rqt_plot /dragon/imu/angles[0] /dragon/imu/angles[1] /dragon/uav/full_statates[8]/state[2]/x /dragon/desire_coordinate/roll /dragon/desire_coordinate/pitch /dragon/uav/nav/target_psi
   ```

5. joint state current trajectory plot
   ```
   $ rqt_plot /dragon/joint_states/position[0] /dragon/joint_stateposition[1] /dragon/joint_states/position[2] /dragon/joint_states/position[3] /dragon/joint_states/position[4] /dragon/joint_states/position[5]
   ```

### squeezing motion with flap moving
### basic command
```
$ roslaunch squeeze_navigation dragon_bringup.launch
$ roslaunch squeeze_navigation dragon_passing_planning.launch headless:=true start_squeeze_path_from_real_state:=true teleop_flag:=true
```

### use following command to proceed the phase 
```
$ rostopic pub -1 /dragon/phase_proceed std_msgs/Empty "{}"
```
#### Phase:
- phase0: hovering
- phase1: approach to the contact point on the flap
- phase2: contact with the flap
- phase3: move the flap
- pahse4: reset to the form before squeezing
- phase5: plan and pass through the openning
