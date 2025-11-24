## Samples for end-effecotr IK

### dependency
- aeiral_robot:
   - url: https://github.com/tongtybj/aerial_robot
   - version: [1.2.0](https://github.com/tongtybj/aerial_robot/tree/1.2.0)


### Hydrus (SE2 model)
1. start ``` $ roscore```
2.  create the environment obstacle. following command will create a simple cylinder in the environment.
```$ rosrun differential_kinematics simple_collision.py _position_x:=0.3 _position_y:=0.3 _scale_x:=0.1 _scale_y:=0.1 _scale_z:=0.3```

3. launch the IK solver
``` $ roslaunch  differential_kinematics  hydrus_end_effector_ik.launch ```

4. send the IK target point to this solver via rosservice.
```
$
rostopic pub -1 /hydrus/end_effector_ik differential_kinematics/TargetPose "target_pos: {x: -0.6, y: 0.6, z: 0.0}
target_rot: {x: 0.0, y: 0.0, z: 3.14}
orientation: true
full_body: true 
collision_avoidance: true 
tran_free_axis: ''
rot_free_axis: ''
debug: false"
```

*Note*: You should confirm the animation in rviz, which the end-effector (attached at the 4th link) moves to the target point [-0.6, 0.6] and yaw: -3.14 gradually without colliding into the red cylinder

### Dragon (SE3 model)
1. launch the IK solver
``` $ roslaunch  differential_kinematics  dragon_end_effector_ik.launch ```

4. send the IK target point to this solver via rosservice.
```
$
rostopic pub -1 /dragon/end_effector_ik differential_kinematics/TargetPose "target_pos: {x: -0.6, y: 0.6, z: 0.4}
target_rot: {x: 0.0, y: 1.0, z: 3.14}
orientation: true
full_body: true 
collision_avoidance: false 
tran_free_axis: ''
rot_free_axis: 'x'
debug: false"
```
*Note*: the above service call means the rotation around x axis in th target frame is free, so you can see in the final result, the y&z axes of end effector do not match the target frame's y&z axes.

#### For other options, you can set:

```
tran_free_axis: 'x', 'y', 'z'
tran_free_plane: 'xy', 'yz', 'xz'
rot_free_axis: 'x', 'y', 'z'
```

