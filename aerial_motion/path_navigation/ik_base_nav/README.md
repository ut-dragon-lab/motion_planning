# Navigate DRAGON based on IK (differential kinematics) planner 

## Usage

### 1. for simulation:

#### Bring-up
```
  $ roslaunch dragon bringup.launch headless:=false simulation:=true real_machine:=false
  $ roslaunch ik_base_nav dragon_navigation.launch 
  ```

  Use keyboard or jotstick to make robot takeoff

#### End-effecot waypoint
```
$
rostopic pub -1 /dragon/end_effector_ik differential_kinematics/TargetPose "target_pos: {x: -0.6, y: 0.6, z: 1.0}
target_rot: {x: 0.0, y: 1.0, z: 3.14}
orientation: true
full_body: true 
collision_avoidance: false 
tran_free_axis: ''
rot_free_axis: ''
```

#### Options for end-effector waypoint:
```
tran_free_axis: 'x', 'y', 'z'
tran_free_plane: 'xy', 'yz', 'xz'
rot_free_axis: 'x', 'y', 'z'
```



