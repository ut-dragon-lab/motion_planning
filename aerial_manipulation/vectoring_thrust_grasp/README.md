## Use vectoring force to grasp

### offline for find the miminum joint torque by QP

```
$ roslaunch vectoring_thrust_grasp test.launch verbose:=true
```


### offline for find the maximum grasping force by nlopt
```
$ roslaunch vectoring_thrust_grasp test.launch verbose:=true search_maximum:=true
```

### Grasping motion in gazebo without object (only test with arbitray pose )
```
$ roslaunch vectoring_thrust_grasp dragon_bringup.launch simulation:=true real_machine:=false headless:=false

$ roslaunch vectoring_thrust_grasp test.launch from_real_joint_angle:=true
```

- change joint: `/dragon/joints_ctrl`

- change psoe: `/dragon/final_target_baselink_rpy` with roll and pitch -0.5 or roll or pitch -0.78

- clear vectoring forces: `/dragon/extra_vectoring_force` with empty forces


### Grasping motion in gazebo with object
```
$ roslaunch vectoring_thrust_grasp dragon_bringup.launch simulation:=true real_machine:=false headless:=false

$ roslaunch vectoring_thrust_grasp grasping_motion.launch simulation:=true measure_object_mass:=true
```

- start: `$ rostopic pub -1 /motion_start std_msgs/Empty "{}"`
- release: ` rostopic pub -1 /release_object std_msgs/Empty "{}"`


### Grasping motion of real machine with object

```
$ roslaunch vectoring_thrust_grasp dragon_bringup.launch 

$ roslaunch vectoring_thrust_grasp grasping_motion.launch measure_object_mass:=true
```
