Try following command:
```
$ roslaunch vectoring_thrust_grasp dragon_bringup.launch simulation:=false real_machine:=false headless:=false
```

### offline for find the maximum grasping force by nlopt
```
$ roslaunch vectoring_thrust_grasp test.launch verbose:=true
```

### from real joint angle
```
$ roslaunch vectoring_thrust_grasp test.launch from_real_joint_angle:=true
```
